#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>

int
 main (int argc, char** argv)
{
  if (argc != 2)
  {
    std::cerr << "please specify command line arg '-r' or '-c'" << std::endl;
    exit(0);
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> (5));
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

  // Fill in the cloud data

  for (size_t i = 0; i < cloud->size (); ++i)
  {
    (*cloud)[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    (*cloud)[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    (*cloud)[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
  }

  if (strcmp(argv[1], "-r") == 0){
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    // build the filter
    outrem.setInputCloud(cloud);
    outrem.setRadiusSearch(0.8);
    outrem.setMinNeighborsInRadius (2);
    // apply filter
    outrem.filter (*cloud_filtered);
  }
  else if (strcmp(argv[1], "-c") == 0){
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
    condrem.setKeepOrganized(true);
    // apply filter
    condrem.filter (*cloud_filtered);
  }
  else{
    std::cerr << "please specify command line arg '-r' or '-c'" << std::endl;
    exit(0);
  }
  std::cerr << "Cloud before filtering: " << std::endl;
  for (size_t i = 0; i < cloud->size (); ++i)
    std::cerr << "    " << (*cloud)[i].x << " "
                        << (*cloud)[i].y << " "
                        << (*cloud)[i].z << std::endl;
  // display pointcloud after filtering
  std::cerr << "Cloud after filtering: " << std::endl;
  for (size_t i = 0; i < cloud_filtered->size (); ++i)
    std::cerr << "    " << (*cloud_filtered)[i].x << " "
                        << (*cloud_filtered)[i].y << " "
                        << (*cloud_filtered)[i].z << std::endl;
  return (0);
}
