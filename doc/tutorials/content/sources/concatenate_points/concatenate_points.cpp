#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int
  main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ> cloud_a (5), cloud_b (3), cloud_c;

  // Fill in the cloud data

  for (size_t i = 0; i < cloud_a.size (); ++i)
  {
    cloud_a[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud_a[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud_a[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
  }

  for (size_t i = 0; i < cloud_b.size (); ++i)
  {
    cloud_b[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud_b[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud_b[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
  }

  std::cerr << "Cloud A: " << std::endl;
  for (size_t i = 0; i < cloud_a.size (); ++i)
    std::cerr << "    " << cloud_a[i].x << " " << cloud_a[i].y << " " << cloud_a[i].z << std::endl;

  std::cerr << "Cloud B: " << std::endl;
  for (size_t i = 0; i < cloud_b.size (); ++i)
    std::cerr << "    " << cloud_b[i].x << " " << cloud_b[i].y << " " << cloud_b[i].z << std::endl;

  // Copy the point cloud data
  cloud_c  = cloud_a;
  cloud_c += cloud_b;

  std::cerr << "Cloud C: " << std::endl;
  for (size_t i = 0; i < cloud_c.size (); ++i)
    std::cerr << "    " << cloud_c[i].x << " " << cloud_c[i].y << " " << cloud_c[i].z << " " << std::endl;

  return (0);
}