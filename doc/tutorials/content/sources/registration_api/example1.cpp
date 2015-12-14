#include <fstream>
#include <pcl/common/angles.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/io/pcd_io.h>
#include <pcl/console/time.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_estimation_normal_shooting.h>
#include <pcl/registration/correspondence_estimation_backprojection.h>
#include <pcl/registration/correspondence_rejection_median_distance.h>
#include <pcl/registration/correspondence_rejection_surface_normal.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>
#include <pcl/registration/default_convergence_criteria.h>

#include <pcl/visualization/pcl_visualizer.h>

using namespace std;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;
using namespace pcl::registration;
using namespace pcl::visualization;

typedef PointNormal PointT;
typedef PointCloud<PointT> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef Cloud::ConstPtr CloudConstPtr;

CloudPtr src, tgt;

bool rejection = true;
bool visualize = false;

boost::shared_ptr<PCLVisualizer> vis;

////////////////////////////////////////////////////////////////////////////////
void
findCorrespondences (const CloudPtr &src,
                     const CloudPtr &tgt,
                     Correspondences &all_correspondences)
{
  //CorrespondenceEstimationNormalShooting<PointT, PointT, PointT> est;
  //CorrespondenceEstimation<PointT, PointT> est;
  CorrespondenceEstimationBackProjection<PointT, PointT, PointT> est;
  est.setInputSource (src);
  est.setInputTarget (tgt);
  
  est.setSourceNormals (src);
  est.setTargetNormals (tgt);
  est.setKSearch (10);
  est.determineCorrespondences (all_correspondences);
  //est.determineReciprocalCorrespondences (all_correspondences);
}

////////////////////////////////////////////////////////////////////////////////
void
rejectBadCorrespondences (const CorrespondencesPtr &all_correspondences,
                          const CloudPtr &src,
                          const CloudPtr &tgt,
                          Correspondences &remaining_correspondences)
{
  CorrespondenceRejectorMedianDistance rej;
  rej.setMedianFactor (8.79241104);
  rej.setInputCorrespondences (all_correspondences);

  rej.getCorrespondences (remaining_correspondences);
  return;
  
  CorrespondencesPtr remaining_correspondences_temp (new Correspondences);
  rej.getCorrespondences (*remaining_correspondences_temp);
  PCL_DEBUG ("[rejectBadCorrespondences] Number of correspondences remaining after rejection: %d\n", remaining_correspondences_temp->size ());

  // Reject if the angle between the normals is really off
  CorrespondenceRejectorSurfaceNormal rej_normals;
  rej_normals.setThreshold (acos (deg2rad (45.0)));
  rej_normals.initializeDataContainer<PointT, PointT> ();
  rej_normals.setInputCloud<PointT> (src);
  rej_normals.setInputNormals<PointT, PointT> (src);
  rej_normals.setInputTarget<PointT> (tgt);
  rej_normals.setTargetNormals<PointT, PointT> (tgt);
  rej_normals.setInputCorrespondences (remaining_correspondences_temp);
  rej_normals.getCorrespondences (remaining_correspondences);
}

////////////////////////////////////////////////////////////////////////////////
void
findTransformation (const CloudPtr &src,
                    const CloudPtr &tgt,
                    const CorrespondencesPtr &correspondences,
                    Eigen::Matrix4d &transform)
{
  TransformationEstimationPointToPlaneLLS<PointT, PointT, double> trans_est;
  trans_est.estimateRigidTransformation (*src, *tgt, *correspondences, transform);
}

////////////////////////////////////////////////////////////////////////////////
void
view (const CloudConstPtr &src, const CloudConstPtr &tgt, const CorrespondencesPtr &correspondences)
{
  if (!visualize || !vis) return;
  PointCloudColorHandlerCustom<PointT> green (tgt, 0, 255, 0);
  if (!vis->updatePointCloud<PointT> (src, "source"))
  {
    vis->addPointCloud<PointT> (src, "source");
    vis->resetCameraViewpoint ("source");
  }
  if (!vis->updatePointCloud<PointT> (tgt, green, "target")) vis->addPointCloud<PointT> (tgt, green, "target");
  vis->setPointCloudRenderingProperties (PCL_VISUALIZER_OPACITY, 0.5, "source");
  vis->setPointCloudRenderingProperties (PCL_VISUALIZER_OPACITY, 0.7, "target");
  vis->setPointCloudRenderingProperties (PCL_VISUALIZER_POINT_SIZE, 6, "source");
  pcl::console::TicToc tt;
  tt.tic ();
  if (!vis->updateCorrespondences<PointT> (src, tgt, *correspondences, 1)) 
    vis->addCorrespondences<PointT> (src, tgt, *correspondences, 1, "correspondences");
  tt.toc_print ();
  vis->setShapeRenderingProperties (PCL_VISUALIZER_LINE_WIDTH, 5, "correspondences");
  //vis->setShapeRenderingProperties (PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "correspondences");
  vis->spin ();
}

////////////////////////////////////////////////////////////////////////////////
void
icp (const PointCloud<PointT>::Ptr &src, 
     const PointCloud<PointT>::Ptr &tgt,
     Eigen::Matrix4d &transform)
{
  CorrespondencesPtr all_correspondences (new Correspondences), 
                     good_correspondences (new Correspondences);

  PointCloud<PointT>::Ptr output (new PointCloud<PointT>);
  *output = *src;

  Eigen::Matrix4d final_transform (Eigen::Matrix4d::Identity ());

  int iterations = 0;
  DefaultConvergenceCriteria<double> converged (iterations, transform, *good_correspondences);

  // ICP loop
  do
  {
    // Find correspondences
    findCorrespondences (output, tgt, *all_correspondences);
    PCL_DEBUG ("Number of correspondences found: %d\n", all_correspondences->size ());

    if (rejection)
    {
      // Reject correspondences
      rejectBadCorrespondences (all_correspondences, output, tgt, *good_correspondences);
      PCL_DEBUG ("Number of correspondences remaining after rejection: %d\n", good_correspondences->size ());
    }
    else
      *good_correspondences = *all_correspondences;

    // Find transformation
    findTransformation (output, tgt, good_correspondences, transform);
 
    // Obtain the final transformation    
    final_transform = transform * final_transform;

    // Transform the data
    transformPointCloudWithNormals (*src, *output, final_transform.cast<float> ());

    // Check if convergence has been reached
    ++iterations;
  
    // Visualize the results
    view (output, tgt, good_correspondences);
  }
  while (!converged);
  transform = final_transform;
}

////////////////////////////////////////////////////////////////////////////////
void
saveTransform (const std::string &file, const Eigen::Matrix4d &transform)
{
  ofstream ofs;
  ofs.open (file.c_str (), ios::trunc | ios::binary);
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j)
      ofs.write (reinterpret_cast<const char*>(&transform (i, j)), sizeof (double));  
  ofs.close ();
}

/* ---[ */
int
main (int argc, char** argv)
{
  // Check whether we want to enable debug mode
  bool debug = false;
  parse_argument (argc, argv, "-debug", debug);
  if (debug)
    setVerbosityLevel (L_DEBUG);

  parse_argument (argc, argv, "-rejection", rejection);
  parse_argument (argc, argv, "-visualization", visualize);
  if (visualize)
    vis.reset (new PCLVisualizer ("Registration example"));

  // Parse the command line arguments for .pcd and .transform files
  std::vector<int> p_pcd_file_indices, p_tr_file_indices;
  p_pcd_file_indices = parse_file_extension_argument (argc, argv, ".pcd");
  if (p_pcd_file_indices.size () != 2)
  {
    print_error ("Need one input source PCD file and one input target PCD file to continue.\n");
    print_error ("Example: %s source.pcd target.pcd output.transform\n", argv[0]);
    return (-1);
  }
  p_tr_file_indices = parse_file_extension_argument (argc, argv, ".transform");
  if (p_tr_file_indices.size () != 1)
  {
    print_error ("Need one output transform file to continue.\n");
    print_error ("Example: %s source.pcd target.pcd output.transform\n", argv[0]);
    return (-1);
  }
  
  // Load the files
  print_info ("Loading %s as source and %s as target...\n", argv[p_pcd_file_indices[0]], argv[p_pcd_file_indices[1]]);
  src.reset (new PointCloud<PointT>);
  tgt.reset (new PointCloud<PointT>);
  if (loadPCDFile (argv[p_pcd_file_indices[0]], *src) == -1 || loadPCDFile (argv[p_pcd_file_indices[1]], *tgt) == -1)
  {
    print_error ("Error reading the input files!\n");
    return (-1);
  }

  // Compute the best transformtion
  Eigen::Matrix4d transform;
  icp (src, tgt, transform);

  saveTransform (argv[p_tr_file_indices[0]], transform);

  cerr.precision (15);
  std::cerr << transform << std::endl;
}
/* ]--- */
