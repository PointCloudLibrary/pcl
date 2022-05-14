#include <pcl/common/generate.h>
#include <pcl/filters/experimental/functor_filter.h>
#include <pcl/point_types.h>

#include <iostream>

int
main()
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
  const Eigen::Vector3f center{0, 0, 2};
  const float radius = 2;

  pcl::experimental::FilterFunction<pcl::PointXYZ> filter;
  filter = [=](const XYZCloud& cloud, pcl::index_t idx) {
    return ((cloud[idx].getVector3fMap() - center).norm() >= radius);
  };

  // build the filter
  pcl::experimental::FunctionFilter<pcl::PointXYZ> func_filter(filter);
  func_filter.setInputCloud(cloud);

  // apply filter
  func_filter.filter(*filtered_cloud);

  // display pointcloud after filtering
  std::cerr << "Cloud after filtering: " << std::endl;
  for (const auto& pt : *filtered_cloud)
    std::cerr << "    " << pt.x << " " << pt.y << " " << pt.z << std::endl;

  return (0);
}
