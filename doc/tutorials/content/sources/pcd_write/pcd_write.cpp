#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int
  main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;

  // Fill in the cloud data
  cloud.width    = 5;
  cloud.height   = 1;
  cloud.is_dense = false;
  cloud.points.resize (cloud.width * cloud.height);

  for (std::size_t i = 0; i < cloud.points.size (); ++i)
  {
    cloud[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
  }

  pcl::io::savePCDFileASCII ("test_pcd.pcd", cloud);
  std::cerr << "Saved " << cloud.points.size () << " data points to test_pcd.pcd." << std::endl;

  for (std::size_t i = 0; i < cloud.points.size (); ++i)
    std::cerr << "    " << cloud[i].x << " " << cloud[i].y << " " << cloud[i].z << std::endl;

  return (0);
}