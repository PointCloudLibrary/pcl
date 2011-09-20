#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int
  main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ> cloud_a;
  pcl::PointCloud<pcl::Normal> cloud_b;
  pcl::PointCloud<pcl::PointNormal> cloud_c;

  // Fill in the cloud data
  cloud_a.width  = cloud_b.width  = 5;
  cloud_a.height = cloud_b.height = 1;
  cloud_a.points.resize (cloud_a.width * cloud_a.height);
  cloud_b.points.resize (cloud_b.width * cloud_b.height);

  for (size_t i = 0; i < cloud_a.points.size (); ++i)
  {
    cloud_a.points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud_a.points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud_a.points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
  }

  for (size_t i = 0; i < cloud_b.points.size (); ++i)
  {
    cloud_b.points[i].normal[0] = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud_b.points[i].normal[1] = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud_b.points[i].normal[2] = 1024 * rand () / (RAND_MAX + 1.0f);
  }

  std::cerr << "Cloud A: " << std::endl;
  for (size_t i = 0; i < cloud_a.points.size (); ++i)
    std::cerr << "    " << cloud_a.points[i].x << " " << cloud_a.points[i].y << " " << cloud_a.points[i].z << std::endl;

  std::cerr << "Cloud B: " << std::endl;
  for (size_t i = 0; i < cloud_b.points.size (); ++i)
    std::cerr << "    " << cloud_b.points[i].normal[0] << " " << cloud_b.points[i].normal[1] << " " << cloud_b.points[i].normal[2] << std::endl;

  pcl::concatenateFields (cloud_a, cloud_b, cloud_c);
  std::cerr << "Cloud C: " << std::endl;
  for (size_t i = 0; i < cloud_c.points.size (); ++i)
    std::cerr << "    " <<
      cloud_c.points[i].x << " " << cloud_c.points[i].y << " " << cloud_c.points[i].z << " " <<
      cloud_c.points[i].normal[0] << " " << cloud_c.points[i].normal[1] << " " << cloud_c.points[i].normal[2] << std::endl;

  return (0);
}