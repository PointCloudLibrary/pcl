#include <iostream>
#include <pcl/point_types.h>
#include <pcl/common/io.h> // for concatenateFields

int
  main (int argc, char** argv)
{
  if (argc != 2)
  {
    std::cerr << "please specify command line arg '-f' or '-p'" << std::endl;
    exit(0);
  }
  pcl::PointCloud<pcl::PointXYZ> cloud_a, cloud_b, cloud_c;
  pcl::PointCloud<pcl::Normal> n_cloud_b;
  pcl::PointCloud<pcl::PointNormal> p_n_cloud_c;

  // Fill in the cloud data
  cloud_a.width  = 5;
  cloud_a.height = cloud_b.height = n_cloud_b.height = 1;
    cloud_a.resize (cloud_a.width * cloud_a.height);
  if (strcmp(argv[1], "-p") == 0)
  {
    cloud_b.width  = 3;
    cloud_b.resize (cloud_b.width * cloud_b.height);
  }
  else{
    n_cloud_b.width = 5;
    n_cloud_b.resize (n_cloud_b.width * n_cloud_b.height);
  }

  for (std::size_t i = 0; i < cloud_a.size (); ++i)
  {
    cloud_a[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud_a[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud_a[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
  }
  if (strcmp(argv[1], "-p") == 0)
    for (std::size_t i = 0; i < cloud_b.size (); ++i)
    {
      cloud_b[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
      cloud_b[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
      cloud_b[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
    }
  else
    for (std::size_t i = 0; i < n_cloud_b.size (); ++i)
    {
      n_cloud_b[i].normal[0] = 1024 * rand () / (RAND_MAX + 1.0f);
      n_cloud_b[i].normal[1] = 1024 * rand () / (RAND_MAX + 1.0f);
      n_cloud_b[i].normal[2] = 1024 * rand () / (RAND_MAX + 1.0f);
    }
  std::cerr << "Cloud A: " << std::endl;
  for (std::size_t i = 0; i < cloud_a.size (); ++i)
    std::cerr << "    " << cloud_a[i].x << " " << cloud_a[i].y << " " << cloud_a[i].z << std::endl;

  std::cerr << "Cloud B: " << std::endl;
  if (strcmp(argv[1], "-p") == 0)
    for (std::size_t i = 0; i < cloud_b.size (); ++i)
      std::cerr << "    " << cloud_b[i].x << " " << cloud_b[i].y << " " << cloud_b[i].z << std::endl;
  else
    for (std::size_t i = 0; i < n_cloud_b.size (); ++i)
      std::cerr << "    " << n_cloud_b[i].normal[0] << " " << n_cloud_b[i].normal[1] << " " << n_cloud_b[i].normal[2] << std::endl;

  // Copy the point cloud data
  if (strcmp(argv[1], "-p") == 0)
  {
    cloud_c  = cloud_a;
    cloud_c += cloud_b;
    std::cerr << "Cloud C: " << std::endl;
    for (std::size_t i = 0; i < cloud_c.size (); ++i)
      std::cerr << "    " << cloud_c[i].x << " " << cloud_c[i].y << " " << cloud_c[i].z << " " << std::endl;
  }
  else
  {
    pcl::concatenateFields (cloud_a, n_cloud_b, p_n_cloud_c);
    std::cerr << "Cloud C: " << std::endl;
    for (std::size_t i = 0; i < p_n_cloud_c.size (); ++i)
      std::cerr << "    " <<
        p_n_cloud_c[i].x << " " << p_n_cloud_c[i].y << " " << p_n_cloud_c[i].z << " " <<
        p_n_cloud_c[i].normal[0] << " " << p_n_cloud_c[i].normal[1] << " " << p_n_cloud_c[i].normal[2] << std::endl;
  }
  return (0);
}
