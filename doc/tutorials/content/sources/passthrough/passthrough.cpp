#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

int
 main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

  // Fill in the cloud data
  cloud->width  = 5;
  cloud->height = 1;
  cloud->points.resize (cloud->width * cloud->height);

  for (std::size_t i = 0; i < cloud->points.size (); ++i)
  {
    (*cloud)[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    (*cloud)[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    (*cloud)[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
  }

  std::cerr << "Cloud before filtering: " << std::endl;
  for (std::size_t i = 0; i < cloud->points.size (); ++i)
    std::cerr << "    " << (*cloud)[i].x << " " 
                        << (*cloud)[i].y << " " 
                        << (*cloud)[i].z << std::endl;

  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 1.0);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_filtered);

  std::cerr << "Cloud after filtering: " << std::endl;
  for (std::size_t i = 0; i < cloud_filtered->points.size (); ++i)
    std::cerr << "    " << (*cloud_filtered)[i].x << " " 
                        << (*cloud_filtered)[i].y << " " 
                        << (*cloud_filtered)[i].z << std::endl;

  return (0);
}
