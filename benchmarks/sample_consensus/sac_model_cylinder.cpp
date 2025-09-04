#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>

#include <benchmark/benchmark.h>

static void
BM_SacModelCylinder(benchmark::State& state, const std::string& file)
{
  // All the objects needed
  pcl::PCDReader reader;
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;

  // Datasets
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
  pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);

  // Read in the cloud data
  reader.read(file, *cloud);

  // Estimate point normals
  ne.setInputCloud(cloud);
  ne.setKSearch(50);
  ne.compute(*cloud_normals);

  // Create the segmentation object for cylinder segmentation and set all the parameters
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_CYLINDER);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setNormalDistanceWeight(0.1);
  seg.setMaxIterations(10000);
  seg.setDistanceThreshold(0.05);
  seg.setRadiusLimits(0, 0.1);
  seg.setInputCloud(cloud);
  seg.setInputNormals(cloud_normals);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxelized(
      new pcl::PointCloud<pcl::PointXYZ>);
  for (auto _ : state) {
    // This code gets timed
    seg.segment(*inliers_cylinder, *coefficients_cylinder);
  }
}

int
main(int argc, char** argv)
{
  if (argc < 2) {
    std::cerr << "No test files given. Please download "
                 "`table_scene_mug_stereo_textured_cylinder.pcd` "
                 "and pass its path to the test."
              << std::endl;
    return (-1);
  }

  benchmark::RegisterBenchmark("BM_SacModelCylinder_mug", &BM_SacModelCylinder, argv[1])
      ->Unit(benchmark::kMillisecond);

  benchmark::Initialize(&argc, argv);
  benchmark::RunSpecifiedBenchmarks();
}
