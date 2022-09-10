#include <iostream>
#include <pcl/common/io.h> // for concatenateFields
#include <pcl/point_types.h>

int
  main ()
{
  pcl::PointCloud<pcl::PointXYZ> cloud_a;
  pcl::PointCloud<pcl::Normal> cloud_b;
  pcl::PointCloud<pcl::PointNormal> cloud_c;

  // Fill in the cloud data
  cloud_a.width  = cloud_b.width  = 5;
  cloud_a.height = cloud_b.height = 1;
  cloud_a.resize (cloud_a.width * cloud_a.height);
  cloud_b.resize (cloud_b.width * cloud_b.height);

  for (std::size_t i = 0; i < cloud_a.size (); ++i)
  {
    cloud_a[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud_a[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud_a[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
  }

  for (std::size_t i = 0; i < cloud_b.size (); ++i)
  {
    cloud_b[i].normal[0] = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud_b[i].normal[1] = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud_b[i].normal[2] = 1024 * rand () / (RAND_MAX + 1.0f);
  }

  std::cerr << "Cloud A: " << std::endl;
  for (std::size_t i = 0; i < cloud_a.size (); ++i)
    std::cerr << "    " << cloud_a[i].x << " " << cloud_a[i].y << " " << cloud_a[i].z << std::endl;

  std::cerr << "Cloud B: " << std::endl;
  for (std::size_t i = 0; i < cloud_b.size (); ++i)
    std::cerr << "    " << cloud_b[i].normal[0] << " " << cloud_b[i].normal[1] << " " << cloud_b[i].normal[2] << std::endl;

  pcl::concatenateFields (cloud_a, cloud_b, cloud_c);
  std::cerr << "Cloud C: " << std::endl;
  for (std::size_t i = 0; i < cloud_c.size (); ++i)
    std::cerr << "    " <<
      cloud_c[i].x << " " << cloud_c[i].y << " " << cloud_c[i].z << " " <<
      cloud_c[i].normal[0] << " " << cloud_c[i].normal[1] << " " << cloud_c[i].normal[2] << std::endl;

  return (0);
}
