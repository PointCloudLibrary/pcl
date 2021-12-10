#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/conditional_removal.h>

int
 main ()
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

  // Fill in the cloud data
  cloud->width  = 5;
  cloud->height = 1;
  cloud->points.resize (cloud->width * cloud->height);

  for (std::size_t i = 0; i < cloud->size (); ++i)
  {
    (*cloud)[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    (*cloud)[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    (*cloud)[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
  }

  std::cerr << "Cloud before filtering: " << std::endl;
  for (std::size_t i = 0; i < cloud->size (); ++i)
    std::cerr << "    " << (*cloud)[i].x << " "
                        << (*cloud)[i].y << " "
                        << (*cloud)[i].z << std::endl;
  // build the condition
  pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond (new
                  pcl::ConditionAnd<pcl::PointXYZ> ());
  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
      pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, 0.0)));
  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
      pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, 0.8)));

  // build the filter
  pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
  condrem.setCondition (range_cond);
  condrem.setInputCloud (cloud);
  condrem.setKeepOrganized (true);

  // apply filter
  condrem.filter (*cloud_filtered);

  // display pointcloud after filtering
  std::cerr << "Cloud after filtering: " << std::endl;
  for (std::size_t i = 0; i < cloud_filtered->size (); ++i)
    std::cerr << "    " << (*cloud_filtered)[i].x << " "
                        << (*cloud_filtered)[i].y << " "
                        << (*cloud_filtered)[i].z << std::endl;
  return (0);
}
