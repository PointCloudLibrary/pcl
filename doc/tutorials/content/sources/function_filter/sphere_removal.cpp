#include <pcl/common/generate.h>
#include <pcl/filters/experimental/functor_filter.h>
#include <pcl/point_types.h>

#include <iostream>

int
main(int argc, char** argv)
{
  using XYZCloud = pcl::PointCloud<pcl::PointXYZ>;
  const auto cloud = pcl::make_shared<XYZCloud>();
  const auto filtered_cloud = pcl::make_shared<XYZCloud>();

  // Create a random generator to fill in the cloud
  pcl::common::CloudGenerator<pcl::PointXYZ, pcl::common::UniformGenerator<float>>
      generator{{-2.0, 2, 1234}};
  generator.fill(10, 1, *cloud);

  std::cerr << "Cloud before filtering: " << std::endl;
  for (const auto& pt : *cloud)
    std::cerr << "    " << pt.x << " " << pt.y << " " << pt.z << std::endl;

  // Setup a condition to reject points inside a filter
  Eigen::Vector3f center{0, 0, 2};
  float radius = 2;

  pcl::experimental::SimpleFilterFunction<pcl::PointXYZ> filter;
  filter = [=](const XYZCloud& cloud, pcl::index_t idx) {
    if ((cloud[idx].getVector3fMap() - center).norm() < radius) {
      // point is inside the sphere -> let's remove it
      return false;
    }
    else {
      return true;
    }
  };

  // build the filter
  pcl::experimental::FunctionFilter<pcl::PointXYZ> func_filter (filter);
  func_filter.setInputCloud(cloud);

  // apply filter
  func_filter.filter(*filtered_cloud);

  // display pointcloud after filtering
  std::cerr << "Cloud after filtering: " << std::endl;
  for (const auto& pt : *filtered_cloud)
    std::cerr << "    " << pt.x << " " << pt.y << " " << pt.z << std::endl;

  return (0);
}
