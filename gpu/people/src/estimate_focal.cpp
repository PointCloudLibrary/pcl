#include <pcl/gpu/people/people_detector.h>
#include <pcl/point_cloud.h>
#include <pcl/search/pcl_search.h>
#include  <Eigen/Core>

namespace
{
  struct SearchD : public pcl::search::OrganizedNeighbor<pcl::PointXYZRGB>
  {  
    using pcl::search::OrganizedNeighbor<pcl::PointXYZRGB>::projection_matrix_;
  };
}

float pcl::gpu::people::estimateFocalLength(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
{
  SearchD search;
  search.setInputCloud(cloud);  
  Eigen::Matrix3f KR = search.projection_matrix_.topLeftCorner <3, 3> ();    
  return (KR(0,0) + KR(1,1))/KR(2,2)/2;
}