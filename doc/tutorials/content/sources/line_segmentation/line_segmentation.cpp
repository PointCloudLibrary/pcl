#include <pcl/common/distances.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>

void
compute (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input, 
         pcl::PointCloud<pcl::PointXYZRGB>::Ptr &output,
         pcl::ModelCoefficients &coefficients,
         pcl::PointIndices &inliers)
{
  // Filter
  pcl::VoxelGrid<pcl::PointXYZRGB> pass;
  pass.setInputCloud (input);
  pass.setLeafSize (0.01, 0.01, 0.01);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 2.0);
  pass.filter (*output);

  // Segment
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  seg.setOptimizeCoefficients (false);
  seg.setProbability (0.99);
  seg.setMaxIterations (100000);
  seg.setModelType (pcl::SACMODEL_STICK);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.02);
  seg.setRadiusLimits (0.02, 0.03);
  seg.setInputCloud (output);
  seg.segment (inliers, coefficients);
}

int
main (int argc, char** argv)
{
  srand (time (0));

  pcl::visualization::PCLVisualizer p (argc, argv, "Line segmentation");

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::ModelCoefficients coefficients;
  pcl::PointIndices inliers;

  std::vector<int> p_file_indices = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");

  for (size_t i = 0; i < p_file_indices.size (); ++i)
  {
    pcl::io::loadPCDFile (argv[p_file_indices[i]], *cloud);
    
    // Compute
    pcl::console::TicToc tt;
    tt.tic ();
    compute (cloud, cloud_f, coefficients, inliers);
    tt.toc_print ();


    // Display
    PCL_INFO ("Found %zu inliers.\n", inliers.indices.size ());

    pcl::PointCloud<pcl::PointXYZ>::Ptr line (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud (*cloud_f, inliers, *line);

    if (!p.updatePointCloud (cloud_f, "all"))
    {
      p.addPointCloud (cloud_f, "all");
      p.resetCameraViewpoint ("all");
    }

    if (!p.updatePointCloud (line, "line inliers")) 
      p.addPointCloud (line, "line inliers");

    pcl::PointXYZRGB pmin, pmax;
    if (pcl::getMaxSegment (*cloud_f, inliers.indices, pmin, pmax) != std::numeric_limits<double>::min ())
      p.addLine<pcl::PointXYZRGB> (pmin, pmax);
    else
    {
      PCL_ERROR ("Couldn't compute the maximum segment!\n");
      p.addLine (coefficients);
    }
    p.setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 50.0, "line");
    p.setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 0.0, "line");

    p.spinOnce ();
    p.removeShape ("line");
  }

  p.spin ();
  return (0);
}
