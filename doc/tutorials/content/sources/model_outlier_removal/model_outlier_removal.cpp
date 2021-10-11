#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/model_outlier_removal.h>

int
main ()
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sphere_filtered (new pcl::PointCloud<pcl::PointXYZ>);

  // 1. Generate cloud data
  std::size_t noise_size = 5;
  std::size_t sphere_data_size = 10;
  cloud->width = noise_size + sphere_data_size;
  cloud->height = 1;
  cloud->points.resize (cloud->width * cloud->height);
  // 1.1 Add noise
  for (std::size_t i = 0; i < noise_size; ++i)
  {
    (*cloud)[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    (*cloud)[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    (*cloud)[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
  }
  // 1.2 Add sphere:
  double rand_x1 = 1;
  double rand_x2 = 1;
  for (std::size_t i = noise_size; i < (noise_size + sphere_data_size); ++i)
  {
    // See: http://mathworld.wolfram.com/SpherePointPicking.html
    while (pow (rand_x1, 2) + pow (rand_x2, 2) >= 1)
    {
      rand_x1 = (rand () % 100) / (50.0f) - 1;
      rand_x2 = (rand () % 100) / (50.0f) - 1;
    }
    double pre_calc = sqrt (1 - pow (rand_x1, 2) - pow (rand_x2, 2));
    (*cloud)[i].x = 2 * rand_x1 * pre_calc;
    (*cloud)[i].y = 2 * rand_x2 * pre_calc;
    (*cloud)[i].z = 1 - 2 * (pow (rand_x1, 2) + pow (rand_x2, 2));
    rand_x1 = 1;
    rand_x2 = 1;
  }

  std::cerr << "Cloud before filtering: " << std::endl;
  for (const auto& point: *cloud)
    std::cout << "    " << point.x << " " << point.y << " " << point.z << std::endl;

  // 2. filter sphere:
  // 2.1 generate model:
  // modelparameter for this sphere:
  // position.x: 0, position.y: 0, position.z:0, radius: 1
  pcl::ModelCoefficients sphere_coeff;
  sphere_coeff.values.resize (4);
  sphere_coeff.values[0] = 0;
  sphere_coeff.values[1] = 0;
  sphere_coeff.values[2] = 0;
  sphere_coeff.values[3] = 1;

  pcl::ModelOutlierRemoval<pcl::PointXYZ> sphere_filter;
  sphere_filter.setModelCoefficients (sphere_coeff);
  sphere_filter.setThreshold (0.05);
  sphere_filter.setModelType (pcl::SACMODEL_SPHERE);
  sphere_filter.setInputCloud (cloud);
  sphere_filter.filter (*cloud_sphere_filtered);

  std::cerr << "Sphere after filtering: " << std::endl;
  for (const auto& point: *cloud_sphere_filtered)
    std::cout << "    " << point.x << " " << point.y << " " << point.z << std::endl;

  return (0);
}
