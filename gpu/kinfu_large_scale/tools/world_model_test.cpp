#include <pcl/point_cloud.h>
#include <pcl/gpu/kinfu_large_scale/world_model.h>
#include <pcl/gpu/kinfu_large_scale/impl/world_model.hpp>
#include <pcl/filters/conditional_removal.h>


void testConditionalRemoval(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_to_filter, double limitX, double limitY, double limitZ)
{
 
  PCL_INFO ("Cloud to filter has %d points\n",  cloud_to_filter->size ());
  
  pcl::ConditionOr<pcl::PointXYZI>::Ptr range_condOR (new pcl::ConditionOr<pcl::PointXYZI> ());
  
  range_condOR->addComparison (pcl::FieldComparison<pcl::PointXYZI>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZI> ("x", pcl::ComparisonOps::GE,  limitX)));

  range_condOR->addComparison (pcl::FieldComparison<pcl::PointXYZI>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZI> ("y", pcl::ComparisonOps::GE,  limitY )));

  range_condOR->addComparison (pcl::FieldComparison<pcl::PointXYZI>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZI> ("z", pcl::ComparisonOps::GE,  limitZ)));

 
  // build the filter
  pcl::ConditionalRemoval<pcl::PointXYZI> condrem (range_condOR, true);
  condrem.setInputCloud (cloud_to_filter);
  condrem.setKeepOrganized (false);
  
  // apply filter
  pcl::PointCloud<pcl::PointXYZI>::Ptr output (new pcl::PointCloud<pcl::PointXYZI>);
  condrem.filter (*output);  
  
    pcl::io::savePCDFile<pcl::PointXYZI> ("output_or.pcd", *output, false);
  
  pcl::IndicesConstPtr indices = condrem.getRemovedIndices ();
  
  PCL_INFO ("Indices vector has %d points\n",  indices->size ());
  PCL_INFO ("Ouput has %d points\n", output->points.size ());


  // Negative filter
  pcl::ConditionAnd<pcl::PointXYZI>::Ptr range_condAND (new pcl::ConditionAnd<pcl::PointXYZI> ());
  
  range_condAND->addComparison (pcl::FieldComparison<pcl::PointXYZI>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZI> ("x", pcl::ComparisonOps::LT,  limitX)));

  range_condAND->addComparison (pcl::FieldComparison<pcl::PointXYZI>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZI> ("y", pcl::ComparisonOps::LT,  limitY )));

  range_condAND->addComparison (pcl::FieldComparison<pcl::PointXYZI>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZI> ("z", pcl::ComparisonOps::LT,  limitZ)));

  pcl::ConditionalRemoval<pcl::PointXYZI> condremNegative (range_condAND, true);
  condremNegative.setInputCloud (cloud_to_filter);
  condremNegative.setKeepOrganized (false);
  
  // apply filter
  pcl::PointCloud<pcl::PointXYZI>::Ptr output_neg (new pcl::PointCloud<pcl::PointXYZI>);
  condremNegative.filter (*output_neg);  
  
  pcl::IndicesConstPtr indices_neg = condremNegative.getRemovedIndices ();
  
  PCL_INFO ("Indices vector has %d points\n",  indices_neg->size ());
  PCL_INFO ("Ouput has %d points\n", output_neg->points.size ());
  
  pcl::io::savePCDFile<pcl::PointXYZI> ("output_and.pcd", *output_neg, false);
}


int
main (int argc, char* argv[])
{  
  
  srand ( time(NULL) );
  
  //create a cloud
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr world_cloud (new pcl::PointCloud<pcl::PointXYZI>);

  // Fill in the cloud data
  cloud->width  = 500;
  cloud->height = 1;
  cloud->points.resize (cloud->width * cloud->height);

  for (size_t i = 0; i < cloud->points.size (); ++i)
  {
    cloud->points[i].x = 512 * (rand () / (RAND_MAX + 1.0f));
    cloud->points[i].y = 512 * (rand () / (RAND_MAX + 1.0f));
    cloud->points[i].z = 512 * (rand () / (RAND_MAX + 1.0f));
    cloud->points[i].intensity = 512 * (rand () / (RAND_MAX + 1.0f));
  }
  
  
  //testConditionalRemoval(cloud, 512, 512, 512);
  
  std::cout << "a" << std::endl;
  pcl::io::savePCDFile<pcl::PointXYZI> ("original_cloud.pcd", *cloud, false);
  
  //cretae world model
  pcl::WorldModel<pcl::PointXYZI> world;

  //feed in a cloud
  world.addSlice(cloud);
    
  world_cloud = world.getWorld();
  std::cout << "b" << std::endl;
  pcl::io::savePCDFile<pcl::PointXYZI> ("original_world.pcd", *world_cloud, false);
  
  
  //set half of it to nans
  world.setSliceAsNans (0, 0, 0,
                        -10, -30, -10,
                        512, 512, 512);
  
  //remove nans
   world_cloud = world.getWorld();
   std::cout << "c" << std::endl;
  pcl::io::savePCDFile<pcl::PointXYZI> ("world_with_nans.pcd", *world_cloud, false);
  
  PCL_INFO ("world contains %d points after update\n", world.getWorldSize ());
  world.cleanWorldFromNans ();                               
  PCL_INFO ("world contains %d points after cleaning\n", world.getWorldSize ());
  
  pcl::PointCloud<pcl::PointXYZI>::Ptr last (new pcl::PointCloud<pcl::PointXYZI>);
  last = world.getWorld();
  pcl::io::savePCDFile<pcl::PointXYZI> ("world_without_nans.pcd", *last, false);
  
  //say how many points are left
  
  
}
