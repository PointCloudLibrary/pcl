#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int
  main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ> cloud_a (5), cloud_b (3), cloud_c;

  // Fill in the cloud data

  for (size_t i = 0; i < cloud_a.points.size (); ++i)
  {
    cloud_a.points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud_a.points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud_a.points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
  }

  for (size_t i = 0; i < cloud_b.points.size (); ++i)
  {
    cloud_b.points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud_b.points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud_b.points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
  }

  std::cerr << "Cloud A: " << std::endl;
  for (size_t i = 0; i < cloud_a.points.size (); ++i)
    std::cerr << "    " << cloud_a.points[i].x << " " << cloud_a.points[i].y << " " << cloud_a.points[i].z << std::endl;

  std::cerr << "Cloud B: " << std::endl;
  for (size_t i = 0; i < cloud_b.points.size (); ++i)
    std::cerr << "    " << cloud_b.points[i].x << " " << cloud_b.points[i].y << " " << cloud_b.points[i].z << std::endl;

  // Copy the point cloud data
  cloud_c  = cloud_a;
  cloud_c += cloud_b;

  std::cerr << "Cloud C: " << std::endl;
  for (size_t i = 0; i < cloud_c.points.size (); ++i)
    std::cerr << "    " << cloud_c.points[i].x << " " << cloud_c.points[i].y << " " << cloud_c.points[i].z << " " << std::endl;

  return (0);
}