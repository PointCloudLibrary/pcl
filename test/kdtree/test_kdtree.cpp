/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009-2011, Willow Garage, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *
 */

#include <gtest/gtest.h>
#include <iostream>  // For debug
#include <map>
#include <pcl/common/time.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/distances.h>

using namespace std;
using namespace pcl;

struct MyPoint : public PointXYZ 
{
  MyPoint (float x, float y, float z) {this->x=x; this->y=y; this->z=z;}
};

PointCloud<MyPoint> cloud, cloud_big;
PointCloud<Eigen::MatrixXf> cloud_eigen;

// Includ the implementation so that KdTree<MyPoint> works
#include <pcl/kdtree/impl/kdtree_flann.hpp>

void 
init ()
{
  float resolution = 0.1f;
  for (float z = -0.5f; z <= 0.5f; z += resolution)
    for (float y = -0.5f; y <= 0.5f; y += resolution)
      for (float x = -0.5f; x <= 0.5f; x += resolution)
        cloud.points.push_back (MyPoint (x, y, z));
  cloud.width  = static_cast<uint32_t> (cloud.points.size ());
  cloud.height = 1;

  cloud_big.width  = 640;
  cloud_big.height = 480;
  srand (static_cast<unsigned int> (time (NULL)));
  // Randomly create a new point cloud
  for (size_t i = 0; i < cloud_big.width * cloud_big.height; ++i)
    cloud_big.points.push_back (MyPoint (static_cast<float> (1024 * rand () / (RAND_MAX + 1.0)),
                                         static_cast<float> (1024 * rand () / (RAND_MAX + 1.0)),
                                         static_cast<float> (1024 * rand () / (RAND_MAX + 1.0))));
}

void 
initEigen ()
{
  cloud_eigen.width  = 640;
  cloud_eigen.height = 480;
  cloud_eigen.points.resize (cloud_eigen.width * cloud_eigen.height, 3);
  srand (static_cast<unsigned int> (time (NULL)));
  // Randomly create a new point cloud
  for (int i = 0; i < cloud_eigen.points.rows (); ++i)
  {
    cloud_eigen.points (i, 0) = static_cast<float> (1024 * rand () / (RAND_MAX + 1.0));
    cloud_eigen.points (i, 1) = static_cast<float> (1024 * rand () / (RAND_MAX + 1.0));
    cloud_eigen.points (i, 2) = static_cast<float> (1024 * rand () / (RAND_MAX + 1.0));
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, KdTreeFLANN_radiusSearch)
{
  KdTreeFLANN<MyPoint> kdtree;
  kdtree.setInputCloud (cloud.makeShared ());
  MyPoint test_point(0.0f, 0.0f, 0.0f);
  double max_dist = 0.15;
  set<int> brute_force_result;
  for (unsigned int i=0; i<cloud.points.size(); ++i)
    if (euclideanDistance(cloud.points[i], test_point) < max_dist)
      brute_force_result.insert(i);
  vector<int> k_indices;
  vector<float> k_distances;
  kdtree.radiusSearch (test_point, max_dist, k_indices, k_distances, 100);
  
  //cout << k_indices.size()<<"=="<<brute_force_result.size()<<"?\n";
  
  for (size_t i = 0; i < k_indices.size (); ++i)
  {
    set<int>::iterator brute_force_result_it = brute_force_result.find (k_indices[i]);
    bool ok = brute_force_result_it != brute_force_result.end ();
    //if (!ok)  cerr << k_indices[i] << " is not correct...\n";
    //else      cerr << k_indices[i] << " is correct...\n";
    EXPECT_EQ (ok, true);
    if (ok)
      brute_force_result.erase (brute_force_result_it);
  }
  //for (set<int>::const_iterator it=brute_force_result.begin(); it!=brute_force_result.end(); ++it)
    //cerr << "FLANN missed "<<*it<<"\n";
  
  bool error = brute_force_result.size () > 0;
  //if (error)  cerr << "Missed too many neighbors!\n";
  EXPECT_EQ (error, false);

  {
    KdTreeFLANN<MyPoint> kdtree;
    kdtree.setInputCloud (cloud_big.makeShared ());

    ScopeTime scopeTime ("FLANN radiusSearch");
    {
      for (size_t i = 0; i < cloud_big.points.size (); ++i)
        kdtree.radiusSearch (cloud_big.points[i], 0.1, k_indices, k_distances);
    }
  }
  
  {
    KdTreeFLANN<MyPoint> kdtree;
    kdtree.setInputCloud (cloud_big.makeShared ());

    ScopeTime scopeTime ("FLANN radiusSearch (max neighbors in radius)");
    {
      for (size_t i = 0; i < cloud_big.points.size (); ++i)
        kdtree.radiusSearch (cloud_big.points[i], 0.1, k_indices, k_distances, 10);
    }
  }
  
  
  {
    KdTreeFLANN<MyPoint> kdtree (false);
    kdtree.setInputCloud (cloud_big.makeShared ());
	
    ScopeTime scopeTime ("FLANN radiusSearch (unsorted results)");
    {
      for (size_t i = 0; i < cloud_big.points.size (); ++i)
        kdtree.radiusSearch (cloud_big.points[i], 0.1, k_indices, k_distances);
    }
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, KdTreeFLANN_radiusSearchEigen)
{
  KdTreeFLANN<Eigen::MatrixXf> kdtree;
  kdtree.setInputCloud (cloud_eigen.makeShared ());

  kdtree.setInputCloud (cloud_eigen.makeShared ());
  Eigen::VectorXf test_point = Eigen::Vector3f (0.0f, 0.0f, 0.0f);
  double max_dist = 0.15;
  set<int> brute_force_result;
  for (int i = 0; i < cloud_eigen.points.rows (); ++i)
    if ((cloud_eigen.points.row (i) - test_point.transpose ()).norm () < max_dist)
      brute_force_result.insert (i);
  vector<int> k_indices;
  vector<float> k_distances;
  kdtree.radiusSearch (test_point, max_dist, k_indices, k_distances);
  
  for (size_t i = 0; i < k_indices.size (); ++i)
  {
    set<int>::iterator brute_force_result_it = brute_force_result.find (k_indices[i]);
    bool ok = brute_force_result_it != brute_force_result.end ();
    EXPECT_EQ (ok, true);
    if (ok)
      brute_force_result.erase (brute_force_result_it);
  }
  
  bool error = brute_force_result.size () > 0;
  EXPECT_EQ (error, false);

  {
    KdTreeFLANN<Eigen::MatrixXf> kdtree;
    kdtree.setInputCloud (cloud_eigen.makeShared ());

    ScopeTime scopeTime ("FLANN radiusSearch");
    {
      for (int i = 0; i < cloud_eigen.points.rows (); ++i)
        kdtree.radiusSearch (cloud_eigen.points.row (i), 0.1, k_indices, k_distances);
    }
  }
  
  {
    KdTreeFLANN<Eigen::MatrixXf> kdtree (false);
    kdtree.setInputCloud (cloud_eigen.makeShared ());

    ScopeTime scopeTime ("FLANN radiusSearch (unsorted results)");
    {
      for (int i = 0; i < cloud_eigen.points.rows (); ++i)
        kdtree.radiusSearch (cloud_eigen.points.row (i), 0.1, k_indices, k_distances);
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, KdTreeFLANN_nearestKSearch)
{
  KdTreeFLANN<MyPoint> kdtree;
  kdtree.setInputCloud (cloud.makeShared ());
  MyPoint test_point (0.01f, 0.01f, 0.01f);
  unsigned int no_of_neighbors = 20;
  multimap<float, int> sorted_brute_force_result;
  for (size_t i = 0; i < cloud.points.size (); ++i)
  {
    float distance = euclideanDistance (cloud.points[i], test_point);
    sorted_brute_force_result.insert (make_pair (distance, static_cast<int> (i)));
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

  // Check if all found neighbors have distance smaller than max_dist
  for (size_t i = 0; i < k_indices.size (); ++i)
  {
    const MyPoint& point = cloud.points[k_indices[i]];
    bool ok = euclideanDistance (test_point, point) <= max_dist;
    if (!ok)
      ok = (fabs (euclideanDistance (test_point, point)) - max_dist) <= 1e-6;
    //if (!ok)  cerr << k_indices[i] << " is not correct...\n";
    //else      cerr << k_indices[i] << " is correct...\n";
    EXPECT_EQ (ok, true);
  }

  ScopeTime scopeTime ("FLANN nearestKSearch");
  {
    KdTreeFLANN<MyPoint> kdtree;
    kdtree.setInputCloud (cloud_big.makeShared ());
    for (size_t i = 0; i < cloud_big.points.size (); ++i)
      kdtree.nearestKSearch (cloud_big.points[i], no_of_neighbors, k_indices, k_distances);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, KdTreeFLANN_nearestKSearchEigen)
{
  KdTreeFLANN<Eigen::MatrixXf> kdtree;
  kdtree.setInputCloud (cloud_eigen.makeShared ());
  Eigen::VectorXf test_point = Eigen::Vector3f (0.01f, 0.01f, 0.01f);
  unsigned int no_of_neighbors = 20;
  multimap<float, int> sorted_brute_force_result;
  for (int i = 0; i < cloud_eigen.points.rows (); ++i)
  {
    float distance = (cloud_eigen.points.row (i) - test_point.transpose ()).norm ();
    sorted_brute_force_result.insert (make_pair (distance, static_cast<int> (i)));
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

  // Check if all found neighbors have distance smaller than max_dist
  for (size_t i = 0; i < k_indices.size (); ++i)
  {
    const Eigen::VectorXf& point = cloud_eigen.points.row (k_indices[i]);
    bool ok = (test_point - point).norm () <= max_dist;
    if (!ok)
      ok = (fabs ((test_point - point).norm () - max_dist)) <= 1e-6;
    //if (!ok)  cerr << k_indices[i] << " is not correct...\n";
    //else      cerr << k_indices[i] << " is correct...\n";
    EXPECT_EQ (ok, true);
  }

  ScopeTime scopeTime ("FLANN nearestKSearch");
  {
    KdTreeFLANN<Eigen::MatrixXf> kdtree;
    kdtree.setInputCloud (cloud_eigen.makeShared ());
    for (int i = 0; i < cloud_eigen.points.rows (); ++i)
      kdtree.nearestKSearch (cloud_eigen.points.row (i), no_of_neighbors, k_indices, k_distances);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class MyPointRepresentationXY : public PointRepresentation<MyPoint>
{
public:
  MyPointRepresentationXY () 
  { 
    this->nr_dimensions_ = 2; 
  }
  
  void copyToFloatArray (const MyPoint &p, float *out) const
  {
    out[0] = p.x;
    out[1] = p.y;
  }
};

TEST (PCL, KdTreeFLANN_setPointRepresentation)
{
  PointCloud<MyPoint>::Ptr random_cloud (new PointCloud<MyPoint> ());
  random_cloud->points.push_back (MyPoint (86.6f, 42.1f, 92.4f));
  random_cloud->points.push_back (MyPoint (63.1f, 18.4f, 22.3f));
  random_cloud->points.push_back (MyPoint (35.5f, 72.5f, 37.3f));
  random_cloud->points.push_back (MyPoint (99.7f, 37.0f,  8.7f));
  random_cloud->points.push_back (MyPoint (22.4f, 84.1f, 64.0f));
  random_cloud->points.push_back (MyPoint (65.2f, 73.4f, 18.0f));
  random_cloud->points.push_back (MyPoint (60.4f, 57.1f,  4.5f));
  random_cloud->points.push_back (MyPoint (38.7f, 17.6f, 72.3f));
  random_cloud->points.push_back (MyPoint (14.2f, 95.7f, 34.7f));
  random_cloud->points.push_back (MyPoint ( 2.5f, 26.5f, 66.0f));

  KdTreeFLANN<MyPoint> kdtree;
  kdtree.setInputCloud (random_cloud);
  MyPoint p (50.0f, 50.0f, 50.0f);
  
  // Find k nearest neighbors
  const int k = 10;
  vector<int> k_indices (k);
  vector<float> k_distances (k);
  kdtree.nearestKSearch (p, k, k_indices, k_distances);
  for (int i = 0; i < k; ++i) 
  {
    // Compare to ground truth values, computed independently
    static const int gt_indices[10] = {2, 7, 5, 1, 4, 6, 9, 0, 8, 3};
    static const float gt_distances[10] = 
      {877.8f, 1674.7f, 1802.6f, 1937.5f, 2120.6f, 2228.8f, 3064.5f, 3199.7f, 3604.2f, 4344.8f};
    EXPECT_EQ (k_indices[i], gt_indices[i]);
    EXPECT_NEAR (k_distances[i], gt_distances[i], 0.1);
  }
  
  // Find k nearest neighbors with a different point representation
  boost::shared_ptr<MyPointRepresentationXY> ptrep (new MyPointRepresentationXY);
  kdtree.setPointRepresentation (ptrep);
  kdtree.nearestKSearch (p, k, k_indices, k_distances);
  for (int i = 0; i < k; ++i) 
  {
    // Compare to ground truth values, computed independently
    static const int gt_indices[10] = {6, 2, 5, 1, 7, 0, 4, 3, 9, 8};
    static const float gt_distances[10] = 
      {158.6f, 716.5f, 778.6f, 1170.2f, 1177.5f, 1402.0f, 1924.6f, 2639.1f, 2808.5f, 3370.1f};
    EXPECT_EQ (k_indices[i], gt_indices[i]);
    EXPECT_NEAR (k_distances[i], gt_distances[i], 0.1);
  }

  // Go back to the default, this time with the values rescaled
  DefaultPointRepresentation<MyPoint> point_rep;
  float alpha[3] = {1.0f, 2.0f, 3.0f};
  point_rep.setRescaleValues(alpha);
  kdtree.setPointRepresentation (point_rep.makeShared ());
  kdtree.nearestKSearch (p, k, k_indices, k_distances);
  for (int i = 0; i < k; ++i) 
  {
    // Compare to ground truth values, computed independently
    static const int gt_indices[10] =  {2, 9, 4, 7, 1, 5, 8, 0, 3, 6};
    static const float gt_distances[10] = 
      {3686.9f, 6769.2f, 7177.0f, 8802.3f, 11071.5f, 11637.3f, 11742.4f, 17769.0f, 18497.3f, 18942.0f};
    EXPECT_EQ (k_indices[i], gt_indices[i]);
    EXPECT_NEAR (k_distances[i], gt_distances[i], 0.1);
  }
}

/* ---[ */
int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  init ();
  initEigen ();
  return (RUN_ALL_TESTS ());
}
/* ]--- */
