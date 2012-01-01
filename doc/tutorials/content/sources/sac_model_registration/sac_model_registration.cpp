#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_registration.h>

using namespace pcl;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
compute (const PointCloud<PointXYZ>::Ptr &input, 
         const PointCloud<PointXYZ>::Ptr &target,
         Eigen::Matrix4f &transformation,
         const double thresh)
{
  SampleConsensusModelRegistration<PointXYZ>::Ptr model (new SampleConsensusModelRegistration<PointXYZ> (input));
  model->setInputTarget (target);

  RandomSampleConsensus<PointXYZ> sac (model, thresh);
  sac.setMaxIterations (100000);

  if (!sac.computeModel (2))
  {
    PCL_ERROR ("Could not compute a valid transformation!\n");
    return;
  }
  Eigen::VectorXf coeff;
  sac.getModelCoefficients (coeff);
  transformation.row (0) = coeff.segment<4>(0);
  transformation.row (1) = coeff.segment<4>(4);
  transformation.row (2) = coeff.segment<4>(8);
  transformation.row (3) = coeff.segment<4>(12);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int
main (int argc, char** argv)
{
  PointCloud<PointXYZ>::Ptr source (new PointCloud<PointXYZ>);
  PointCloud<PointXYZ>::Ptr target (new PointCloud<PointXYZ>);

  std::vector<int> p_file_indices = console::parse_file_extension_argument (argc, argv, ".pcd");
  if (p_file_indices.size () < 2)
  {
    PCL_ERROR ("Needs a source.PCD, and an output source_transformed.PCD file! For example: %s bun0.pcd bun0-tr.pcd\n", argv[0]);
    PCL_INFO ("Additionally, please specify a threshold (0.002 used by default) via: -thresh X\n");
    return (-1);
  }

  double thresh = 0.002;
  console::parse_argument (argc, argv, "-thresh", thresh);

  io::loadPCDFile (argv[p_file_indices[0]], *source);

  // Transform the dataset by translations on x (+1), y (+2), and z (+3) and add noise
  *target = *source;
  for (size_t i = 0; i < source->points.size (); ++i)
  {
    target->points[i].x += ((double)rand () / RAND_MAX) * 0.01 + 1.0;
    target->points[i].y += ((double)rand () / RAND_MAX) * 0.01 + 2.0;
    target->points[i].z += ((double)rand () / RAND_MAX) * 0.01 + 3.0;
  }
  io::savePCDFileBinary ("target.pcd", *target);

  // Compute
  Eigen::Matrix4f transform;
  console::TicToc tt;
  tt.tic ();
  compute (source, target, transform, thresh);
  tt.toc_print ();
  std::cerr << transform << std::endl;

  PointCloud<PointXYZ> output;
  transformPointCloud (*source, output, transform);
  io::savePCDFileBinary (argv[p_file_indices[1]], output);

  return (0);
}
