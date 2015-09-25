#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int
  main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ> cloud (5);

  // Fill in the cloud data
  cloud.is_dense = false;

  for (size_t i = 0; i < cloud.size (); ++i)
  {
    cloud[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
  }

  pcl::io::savePCDFileASCII ("test_pcd.pcd", cloud);
  std::cerr << "Saved " << cloud.size () << " data points to test_pcd.pcd." << std::endl;

  for (size_t i = 0; i < cloud.size (); ++i)
    std::cerr << "    " << cloud[i].x << " " << cloud[i].y << " " << cloud[i].z << std::endl;

  return (0);
}