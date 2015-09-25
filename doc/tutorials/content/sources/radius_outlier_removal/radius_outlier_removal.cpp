#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>

int
 main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> (5));
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

  // Fill in the cloud data

  for (size_t i = 0; i < cloud->size (); ++i)
  {
    (*cloud)[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    (*cloud)[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    (*cloud)[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
  }

  std::cerr << "Cloud before filtering: " << std::endl;
  for (size_t i = 0; i < cloud->size (); ++i)
    std::cerr << "    " << (*cloud)[i].x << " "
                        << (*cloud)[i].y << " "
                        << (*cloud)[i].z << std::endl;
  // build the filter
  pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
  outrem.setInputCloud(cloud);
  outrem.setRadiusSearch(0.8);
  outrem.setMinNeighborsInRadius (2);

  // apply filter
  outrem.filter (*cloud_filtered);

  // display pointcloud after filtering
  std::cerr << "Cloud after filtering: " << std::endl;
  for (size_t i = 0; i < cloud_filtered->size (); ++i)
    std::cerr << "    " << (*cloud_filtered)[i].x << " "
                        << (*cloud_filtered)[i].y << " "
                        << (*cloud_filtered)[i].z << std::endl;
  return (0);
}
