#include <iostream>
#include <gtest/gtest.h>


using namespace std;

#include <pcl/common/time.h>
//#include <pcl/kdtree/kdtree_ann.h>
#include <pcl/search/generic_search.h>
#include <pcl/search/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


using namespace pcl;




PointCloud<PointXYZ> cloud, cloud_big;
void
  init ()
{
  float resolution = 0.1;
  for (float z = -0.5f; z <= 0.5f; z += resolution)
    for (float y = -0.5f; y <= 0.5f; y += resolution)
      for (float x = -0.5f; x <= 0.5f; x += resolution)
        cloud.points.push_back (PointXYZ (x, y, z));
  cloud.width  = cloud.points.size ();
  cloud.height = 1;

  cloud_big.width  = 640;
  cloud_big.height = 480;
  srand (time (NULL));
  // Randomly create a new point cloud
  for (size_t i = 0; i < cloud_big.width * cloud_big.height; ++i)
    cloud_big.points.push_back (PointXYZ (1024 * rand () / (RAND_MAX + 1.0),
                                         1024 * rand () / (RAND_MAX + 1.0),
                                         1024 * rand () / (RAND_MAX + 1.0)));
}



TEST (PCL, KdTreeFLANN_nearestKSearch)
{

  Search<PointXYZ> kdtree(KDTREE_FLANN);
//  kdtree.initSearchDS();
  kdtree.setInputCloud (cloud.makeShared ());
  PointXYZ test_point (0.01f, 0.01f, 0.01f);
  unsigned int no_of_neighbors = 20;
  multimap<float, int> sorted_brute_force_result;
  for (size_t i = 0; i < cloud.points.size (); ++i)
  {
    float distance = euclideanDistance (cloud.points[i], test_point);
    sorted_brute_force_result.insert (make_pair (distance, i));
  }
  float max_dist = 0.0f;
  unsigned int counter = 0;
  for (multimap<float, int>::iterator it = sorted_brute_force_result.begin (); it != sorted_brute_force_result.end () && counter < no_of_neighbors; ++it)
  {
    max_dist = max (max_dist, it->first);
    ++counter;
  }

  vector<int> k_indices;
  k_indices.resize (no_of_neighbors);
  vector<float> k_distances;
  k_distances.resize (no_of_neighbors);
  kdtree.nearestKSearch (test_point, no_of_neighbors, k_indices, k_distances);
  //if (k_indices.size() != no_of_neighbors)  cerr << "Found "<<k_indices.size()<<" instead of "<<no_of_neighbors<<" neighbors.\n";
  EXPECT_EQ (k_indices.size (), no_of_neighbors);

#if 1
  // Check if all found neighbors have distance smaller than max_dist
  for (size_t i = 0; i < k_indices.size (); ++i)
  {
    const PointXYZ& point = cloud.points[k_indices[i]];
    bool ok = euclideanDistance (test_point, point) <= max_dist;
    if (!ok)
      ok = (fabs (euclideanDistance (test_point, point)) - max_dist) <= 1e-6;
    //if (!ok)  cerr << k_indices[i] << " is not correct...\n";
    //else      cerr << k_indices[i] << " is correct...\n";
    EXPECT_EQ (ok, true);
  }

  ScopeTime scopeTime ("FLANN nearestKSearch");
  {
    Search<PointXYZ> kdtree(KDTREE_FLANN);
//    kdtree.initSearchDS();
    kdtree.setInputCloud (cloud_big.makeShared ());
    for (size_t i = 0; i < cloud_big.points.size (); ++i)
      kdtree.nearestKSearch (cloud_big.points[i], no_of_neighbors, k_indices, k_distances);
  }
#endif

}	



int main(int argc, char** argv)
{


  testing::InitGoogleTest (&argc, argv);
  init ();

/* Testing using explicit instantiation of inherited class */
  Search<PointXYZ>* kdtree = new KdTreeFLANN<PointXYZ>();
  kdtree->setInputCloud (cloud.makeShared ());


  return (RUN_ALL_TESTS ());


};
