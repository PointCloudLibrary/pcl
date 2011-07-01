#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <iostream>
#include <vector>
#include <ctime>

using namespace pcl;

int
main (int argc, char** argv)
{

  srand (time (NULL));

  PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);

  // Generate pointcloud data
  cloud->width = 1000;
  cloud->height = 1;
  cloud->points.resize (cloud->width * cloud->height);

  for (size_t i = 0; i < cloud->points.size (); ++i)
  {
    cloud->points[i].x = 1024.0f * rand () / (RAND_MAX + 1.0);
    cloud->points[i].y = 1024.0f * rand () / (RAND_MAX + 1.0);
    cloud->points[i].z = 1024.0f * rand () / (RAND_MAX + 1.0);
  }

  KdTreeFLANN<PointXYZ> kdtree;

  kdtree.setInputCloud (cloud);

  PointXYZ searchPoint;

  searchPoint.x = 1024.0f * rand () / (RAND_MAX + 1.0);
  searchPoint.y = 1024.0f * rand () / (RAND_MAX + 1.0);
  searchPoint.z = 1024.0f * rand () / (RAND_MAX + 1.0);

  // K nearest neighbor search

  int K = 10;

  std::vector<int> pointIdxNKNSearch(K);
  std::vector<float> pointNKNSquaredDistance(K);

  std::cerr << "K nearest neighbor search at (" << searchPoint.x 
      << " " << searchPoint.y 
      << " " << searchPoint.z
      << ") with K=" << K << std::endl;

  if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
  {
    for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
   std::cerr << "    "  <<   cloud->points[ pointIdxNKNSearch[i] ].x 
       << " " << cloud->points[ pointIdxNKNSearch[i] ].y 
       << " " << cloud->points[ pointIdxNKNSearch[i] ].z 
       << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
  }

  // Neighbors within radius search

  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;

  float radius = 256.0f * rand () / (RAND_MAX + 1.0);

  std::cerr << "Neighbors within radius search at (" << searchPoint.x 
      << " " << searchPoint.y 
      << " " << searchPoint.z
      << ") with radius=" << radius << std::endl;


  if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
  {
    for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
   std::cerr << "    "  <<   cloud->points[ pointIdxRadiusSearch[i] ].x 
       << " " << cloud->points[ pointIdxRadiusSearch[i] ].y 
       << " " << cloud->points[ pointIdxRadiusSearch[i] ].z 
       << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
  }


  return 0;
}
