#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>

int
 main ()
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

  // Fill in the cloud data
  cloud->width  = 5;
  cloud->height = 1;
  cloud->points.resize (cloud->width * cloud->height);

  for (auto& point: *cloud)
  {
    point.x = 1024 * rand () / (RAND_MAX + 1.0f);
    point.y = 1024 * rand () / (RAND_MAX + 1.0f);
    point.z = 1024 * rand () / (RAND_MAX + 1.0f);
  }

  std::cerr << "Cloud before filtering: " << std::endl;
  for (const auto& point: *cloud)
    std::cerr << "    " << point.x << " "
                        << point.y << " "
                        << point.z << std::endl;
  // build the filter
  pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
  outrem.setInputCloud(cloud);
  outrem.setRadiusSearch(0.8);
  outrem.setMinNeighborsInRadius (2);

  // apply filter
  outrem.filter (*cloud_filtered);

  // display pointcloud after filtering
  std::cerr << "Cloud after filtering: " << std::endl;
  for (const auto& point: *cloud_filtered)
    std::cerr << "    " << point.x << " "
                        << point.y << " "
                        << point.z << std::endl;
  return (0);
}
