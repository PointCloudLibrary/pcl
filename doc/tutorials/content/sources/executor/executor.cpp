#include <pcl/common/generate.h>
#include <pcl/filters/functor_filter.h>
#include <pcl/visualization/cloud_viewer.h>

int
main()
{
  // Fill the cloud with randomly generated points
  auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::common::CloudGenerator<pcl::PointXYZ, pcl::common::UniformGenerator<float>>
      generator{{-20., 20., 128}};
  generator.fill(200, 200, *cloud);

  // Create executors
  const auto simple_exec = executor::default_inline_executor();
  const auto parallel_exec = executor::default_omp_executor(4); // Set max threads to 4

  // Create a functor filter that filters point outside a fixed radius
  constexpr auto filter_radius = 15.0;
  const auto radius_cond = [filter_radius](const pcl::PointCloud<pcl::PointXYZ>& cloud,
                                           pcl::index_t idx) {
    return cloud[idx].getVector3fMap().norm() < filter_radius;
  };

  auto radius_filter =
      pcl::experimental::FunctorFilter<pcl::PointXYZ, decltype(radius_cond)>(
          radius_cond);
  radius_filter.setInputCloud(cloud);

  // Create a functor filter which removes points with negative y co-ordinates
  const auto positive_y_cond =
      [filter_radius](const pcl::PointCloud<pcl::PointXYZ>& cloud, pcl::index_t idx) {
        return cloud[idx].y > 0;
      };

  auto positive_y_filter =
      pcl::experimental::FunctorFilter<pcl::PointXYZ, decltype(positive_y_cond)>(
          positive_y_cond);
  positive_y_filter.setInputCloud(cloud);

  // Apply the filters with the respective executors
  radius_filter.filter(parallel_exec, *cloud);
  positive_y_filter.filter(simple_exec, *cloud);

  // Visualize the filtered cloud
  pcl::visualization::PCLVisualizer::Ptr viewer(
      new pcl::visualization::PCLVisualizer("3D Viewer"));
  viewer->setBackgroundColor(0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ>(cloud, "Filtered Cloud");
  viewer->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Filtered Cloud");
  viewer->addCoordinateSystem(1.0);
  viewer->initCameraParameters();

  while (!viewer->wasStopped()) {
    viewer->spinOnce(100);
  }

  return 0;
}
