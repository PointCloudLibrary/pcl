#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

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
    cloud_a.points.resize (cloud_a.width * cloud_a.height);
  if (strcmp(argv[1], "-p") == 0)
  {
    cloud_b.width  = 3;
    cloud_b.points.resize (cloud_b.width * cloud_b.height);
  }
  else{
    n_cloud_b.width = 5;
    n_cloud_b.points.resize (n_cloud_b.width * n_cloud_b.height);
  }

  for (size_t i = 0; i < cloud_a.points.size (); ++i)
  {
    cloud_a.points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud_a.points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud_a.points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
  }
  if (strcmp(argv[1], "-p") == 0)
    for (size_t i = 0; i < cloud_b.points.size (); ++i)
    {
      cloud_b.points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
      cloud_b.points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
      cloud_b.points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
    }
  else
    for (size_t i = 0; i < n_cloud_b.points.size (); ++i)
    {
      n_cloud_b.points[i].normal[0] = 1024 * rand () / (RAND_MAX + 1.0f);
      n_cloud_b.points[i].normal[1] = 1024 * rand () / (RAND_MAX + 1.0f);
      n_cloud_b.points[i].normal[2] = 1024 * rand () / (RAND_MAX + 1.0f);
    }
  std::cerr << "Cloud A: " << std::endl;
  for (size_t i = 0; i < cloud_a.points.size (); ++i)
    std::cerr << "    " << cloud_a.points[i].x << " " << cloud_a.points[i].y << " " << cloud_a.points[i].z << std::endl;

  std::cerr << "Cloud B: " << std::endl;
  if (strcmp(argv[1], "-p") == 0)
    for (size_t i = 0; i < cloud_b.points.size (); ++i)
      std::cerr << "    " << cloud_b.points[i].x << " " << cloud_b.points[i].y << " " << cloud_b.points[i].z << std::endl;
  else
    for (size_t i = 0; i < n_cloud_b.points.size (); ++i)
      std::cerr << "    " << n_cloud_b.points[i].normal[0] << " " << n_cloud_b.points[i].normal[1] << " " << n_cloud_b.points[i].normal[2] << std::endl;

  // Copy the point cloud data
  if (strcmp(argv[1], "-p") == 0)
  {
    cloud_c  = cloud_a;
    cloud_c += cloud_b;
    std::cerr << "Cloud C: " << std::endl;
    for (size_t i = 0; i < cloud_c.points.size (); ++i)
      std::cerr << "    " << cloud_c.points[i].x << " " << cloud_c.points[i].y << " " << cloud_c.points[i].z << " " << std::endl;
  }
  else
  {
    pcl::concatenateFields (cloud_a, n_cloud_b, p_n_cloud_c);
    std::cerr << "Cloud C: " << std::endl;
    for (size_t i = 0; i < p_n_cloud_c.points.size (); ++i)
      std::cerr << "    " <<
        p_n_cloud_c.points[i].x << " " << p_n_cloud_c.points[i].y << " " << p_n_cloud_c.points[i].z << " " <<
        p_n_cloud_c.points[i].normal[0] << " " << p_n_cloud_c.points[i].normal[1] << " " << p_n_cloud_c.points[i].normal[2] << std::endl;
  }
  return (0);
}
