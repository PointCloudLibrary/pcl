/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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

#include <iostream>
#include <pcl/test/gtest.h>
#include <pcl/common/io.h> // for copyPointCloud
#include <pcl/common/distances.h>
#include <pcl/common/time.h>
#include <pcl/search/kdtree.h> // for pcl::search::KdTree
#include <pcl/search/flann_search.h> // for pcl::search::FlannSearch
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace pcl;

PointCloud<PointXYZ> cloud, cloud_big;

void
init ()
{
  float resolution = 0.1f;
  for (float z = -0.5f; z <= 0.5f; z += resolution)
    for (float y = -0.5f; y <= 0.5f; y += resolution)
      for (float x = -0.5f; x <= 0.5f; x += resolution)
        cloud.emplace_back(x, y, z);
  cloud.width = cloud.size ();
  cloud.height = 1;

  srand (int (time (nullptr)));
  // Randomly create a new point cloud, use points.emplace_back
  cloud_big.width = 640;
  cloud_big.height = 480;
  for (std::size_t i = 0; i < cloud_big.width * cloud_big.height; ++i)
    cloud_big.points.emplace_back(static_cast<float>(1024 * rand() / (RAND_MAX + 1.0)),
                                  static_cast<float>(1024 * rand() / (RAND_MAX + 1.0)),
                                  static_cast<float>(1024 * rand() / (RAND_MAX + 1.0)));
}


/* Test for FlannSearch nearestKSearch */
TEST (PCL, FlannSearch_nearestKSearch)
{
  pcl::search::FlannSearch<PointXYZ> FlannSearch (new search::FlannSearch<PointXYZ>::KdTreeIndexCreator);
  FlannSearch.setInputCloud (cloud.makeShared ());
  PointXYZ test_point (0.01f, 0.01f, 0.01f);
  unsigned int no_of_neighbors = 20;
  std::multimap<float, int> sorted_brute_force_result;
  for (std::size_t i = 0; i < cloud.size (); ++i)
  {
    float distance = euclideanDistance (cloud[i], test_point);
    sorted_brute_force_result.insert (std::make_pair (distance, int (i)));
  }
  float max_dist = 0.0f;
  unsigned int counter = 0;
  for (auto it = sorted_brute_force_result.begin (); it != sorted_brute_force_result.end ()
      && counter < no_of_neighbors; ++it)
  {
    max_dist = std::max (max_dist, it->first);
    ++counter;
  }

  pcl::Indices k_indices;
  k_indices.resize (no_of_neighbors);
  std::vector<float> k_distances;
  k_distances.resize (no_of_neighbors);

  FlannSearch.nearestKSearch (test_point, no_of_neighbors, k_indices, k_distances);

  //if (k_indices.size () != no_of_neighbors)  std::cerr << "Found "<<k_indices.size ()<<" instead of "<<no_of_neighbors<<" neighbors.\n";
  EXPECT_EQ (k_indices.size (), no_of_neighbors);

  // Check if all found neighbors have distance smaller than max_dist
  for (const auto &k_index : k_indices)
  {
    const PointXYZ& point = cloud[k_index];
    bool ok = euclideanDistance (test_point, point) <= max_dist;
    if (!ok)
    ok = (std::abs (euclideanDistance (test_point, point)) - max_dist) <= 1e-6;
    //if (!ok)  std::cerr << k_indices[i] << " is not correct...\n";
    //else      std::cerr << k_indices[i] << " is correct...\n";
    EXPECT_TRUE (ok);
  }

  ScopeTime scopeTime ("FLANN nearestKSearch");
  {
    pcl::search::FlannSearch<PointXYZ> FlannSearch( new search::FlannSearch<PointXYZ>::KdTreeIndexCreator);
    //FlannSearch.initSearchDS ();
    FlannSearch.setInputCloud (cloud_big.makeShared ());
    for (const auto &point : cloud_big.points)
      FlannSearch.nearestKSearch (point, no_of_neighbors, k_indices, k_distances);
  }
}

/* Test the templated NN search (for different query point types) */
TEST (PCL, FlannSearch_differentPointT)
{

  unsigned int no_of_neighbors = 20;

  pcl::search::FlannSearch<PointXYZ> FlannSearch (new search::FlannSearch<PointXYZ>::KdTreeIndexCreator);
  //FlannSearch.initSearchDS ();
  FlannSearch.setInputCloud (cloud_big.makeShared ());

  PointCloud<PointXYZRGB> cloud_rgb;

  copyPointCloud (cloud_big, cloud_rgb);



  std::vector< std::vector< float > > dists;
  std::vector< pcl::Indices > indices;
  FlannSearch.nearestKSearchT (cloud_rgb, pcl::Indices (),no_of_neighbors,indices,dists);

  pcl::Indices k_indices;
  k_indices.resize (no_of_neighbors);
  std::vector<float> k_distances;
  k_distances.resize (no_of_neighbors);

  //pcl::Indices k_indices_t;
  //k_indices_t.resize (no_of_neighbors);
  //vector<float> k_distances_t;
  //k_distances_t.resize (no_of_neighbors);

  for (std::size_t i = 0; i < cloud_rgb.size (); ++i)
  {
    //FlannSearch.nearestKSearchT (cloud_rgb[i], no_of_neighbors, k_indices_t, k_distances_t);
    FlannSearch.nearestKSearch (cloud_big[i], no_of_neighbors, k_indices, k_distances);
    EXPECT_EQ (k_indices.size (), indices[i].size ());
    EXPECT_EQ (k_distances.size (), dists[i].size ());
    for (std::size_t j = 0; j< no_of_neighbors; j++)
    {
      EXPECT_TRUE (k_indices[j] == indices[i][j] || k_distances[j] == dists[i][j]);
      //EXPECT_EQ (k_indices[j], k_indices_t[j]);
      //EXPECT_EQ (k_distances[j], k_distances_t[j]);
    }

  }
}

/* Test for FlannSearch nearestKSearch with multiple query points */
TEST (PCL, FlannSearch_multipointKnnSearch)
{

  unsigned int no_of_neighbors = 20;


  pcl::search::FlannSearch<PointXYZ> FlannSearch (new search::FlannSearch<PointXYZ>::KdTreeIndexCreator);
  //FlannSearch.initSearchDS ();
  FlannSearch.setInputCloud (cloud_big.makeShared ());

  std::vector< std::vector< float > > dists;
  std::vector< pcl::Indices > indices;
  FlannSearch.nearestKSearch (cloud_big, pcl::Indices(),no_of_neighbors,indices,dists);

  pcl::Indices k_indices;
  k_indices.resize (no_of_neighbors);
  std::vector<float> k_distances;
  k_distances.resize (no_of_neighbors);

  for (std::size_t i = 0; i < cloud_big.size (); ++i)
  {
    FlannSearch.nearestKSearch (cloud_big[i], no_of_neighbors, k_indices, k_distances);
    EXPECT_EQ (k_indices.size (), indices[i].size ());
    EXPECT_EQ (k_distances.size (), dists[i].size ());
    for (std::size_t j = 0; j< no_of_neighbors; j++ )
    {
      EXPECT_TRUE (k_indices[j] == indices[i][j] || k_distances[j] == dists[i][j]);
    }

  }
}

/* Test for FlannSearch nearestKSearch with multiple query points */
TEST (PCL, FlannSearch_knnByIndex)
{

  unsigned int no_of_neighbors = 3;


  pcl::search::FlannSearch<PointXYZ> flann_search (new search::FlannSearch<PointXYZ>::KdTreeIndexCreator);
  //FlannSearch->initSearchDS ();
  flann_search.setInputCloud (cloud_big.makeShared ());

  std::vector< std::vector< float > > dists;
  std::vector< pcl::Indices > indices;
  pcl::Indices query_indices;
  for (std::size_t i = 0; i<cloud_big.size (); i+=2)
  {
    query_indices.push_back (int (i));
  }
  flann_search.nearestKSearch (cloud_big, query_indices,no_of_neighbors,indices,dists);

  pcl::Indices k_indices;
  k_indices.resize (no_of_neighbors);
  std::vector<float> k_distances;
  k_distances.resize (no_of_neighbors);

  for (std::size_t i = 0; i < query_indices.size (); ++i)
  {
    flann_search.nearestKSearch (cloud_big[2*i], no_of_neighbors, k_indices, k_distances);
    EXPECT_EQ (k_indices.size (), indices[i].size ());
    EXPECT_EQ (k_distances.size (), dists[i].size ());
    for (std::size_t j = 0; j< no_of_neighbors; j++)
    {
      EXPECT_TRUE (k_indices[j] == indices[i][j] || k_distances[j] == dists[i][j]);
    }
    flann_search.nearestKSearch (cloud_big,query_indices[i], no_of_neighbors, k_indices, k_distances);
    EXPECT_EQ (k_indices.size (), indices[i].size ());
    EXPECT_EQ (k_distances.size (), dists[i].size ());
    for (std::size_t j = 0; j< no_of_neighbors; j++)
    {
      EXPECT_TRUE (k_indices[j] == indices[i][j] || k_distances[j] == dists[i][j]);
    }

  }
}


/* Test for FlannSearch nearestKSearch */
TEST (PCL, FlannSearch_compareToKdTreeFlann)
{

  int no_of_neighbors=3;
  pcl::Indices k_indices;
  k_indices.resize (no_of_neighbors);
  std::vector<float> k_distances;
  k_distances.resize (no_of_neighbors);

  pcl::search::Search<PointXYZ> *flann_search, *kdtree_search;

  PointCloud<PointXYZ>::Ptr pc = cloud_big.makeShared();
  {
    ScopeTime scopeTime ("FLANN build");
    flann_search = new pcl::search::FlannSearch<PointXYZ> (new search::FlannSearch<PointXYZ>::KdTreeIndexCreator);
    flann_search->setInputCloud (pc);
  }

  {
    ScopeTime scopeTime ("kdtree build");
    kdtree_search = new pcl::search::KdTree<PointXYZ> ();
    kdtree_search->setInputCloud (pc);
  }



  {
    ScopeTime scopeTime ("FLANN nearestKSearch");
    for (const auto &point : cloud_big.points)
      flann_search->nearestKSearch (point, no_of_neighbors, k_indices, k_distances);
  }
  {
    ScopeTime scopeTime ("kd tree  nearestKSearch");
    for (const auto &point : cloud_big.points)
      kdtree_search->nearestKSearch (point, no_of_neighbors, k_indices, k_distances);
  }

  std::vector<pcl::Indices> indices_flann;
  std::vector<std::vector<float> > dists_flann;
  std::vector<pcl::Indices> indices_tree;
  std::vector<std::vector<float> > dists_tree;
  indices_flann.resize (cloud_big.size ());
  dists_flann.resize (cloud_big.size ());
  indices_tree.resize (cloud_big.size ());
  dists_tree.resize (cloud_big.size ());
  for (std::size_t i = 0; i<cloud_big.size (); ++i)
  {
    indices_flann[i].resize (no_of_neighbors);
    dists_flann[i].resize (no_of_neighbors);
    indices_tree[i].resize (no_of_neighbors);
    dists_tree[i].resize (no_of_neighbors);
  }

  {
    ScopeTime scopeTime ("FLANN multi nearestKSearch");
    flann_search->nearestKSearch (cloud_big, pcl::Indices (), no_of_neighbors, indices_flann,dists_flann);
  }
  {
    ScopeTime scopeTime ("kd tree multi nearestKSearch");
    kdtree_search->nearestKSearch (cloud_big, pcl::Indices (), no_of_neighbors, indices_tree,dists_tree);
  }

  ASSERT_EQ (indices_flann.size (), dists_flann.size ());
  ASSERT_EQ (indices_flann.size (), indices_tree.size ());
  ASSERT_EQ (indices_flann.size (), dists_tree.size ());

  for (std::size_t i = 0; i<indices_flann.size ();i++)
  {
    ASSERT_EQ (indices_flann[i].size (), no_of_neighbors);
    ASSERT_EQ (indices_tree[i].size (), no_of_neighbors);
    ASSERT_EQ (dists_flann[i].size (), no_of_neighbors);
    ASSERT_EQ (dists_tree[i].size (), no_of_neighbors);
    for (int j = 0; j<no_of_neighbors; j++)
    {

      ASSERT_TRUE( indices_flann[i][j] == indices_tree[i][j] || dists_flann[i][j]==dists_tree[i][j]);
    }
  }


  pcl::Indices query_indices;
  for (std::size_t i = 0; i<cloud_big.size (); i+=2)
    query_indices.push_back (int (i));

  {
    ScopeTime scopeTime ("FLANN multi nearestKSearch with indices");
    flann_search->nearestKSearch (cloud_big, query_indices, no_of_neighbors, indices_flann,dists_flann);
  }
  {
    ScopeTime scopeTime ("kd tree multi nearestKSearch with indices");
    kdtree_search->nearestKSearch (cloud_big, query_indices, no_of_neighbors, indices_tree,dists_tree);
  }
  ASSERT_EQ (indices_flann.size (), dists_flann.size ());
  ASSERT_EQ (indices_flann.size (), indices_tree.size ());
  ASSERT_EQ (indices_flann.size (), dists_tree.size ());

  for (std::size_t i = 0; i<indices_flann.size ();i++)
  {
    ASSERT_EQ (indices_flann[i].size (), no_of_neighbors);
    ASSERT_EQ (indices_tree[i].size (), no_of_neighbors);
    ASSERT_EQ (dists_flann[i].size (), no_of_neighbors);
    ASSERT_EQ (dists_tree[i].size (), no_of_neighbors);
    for (int j = 0; j < no_of_neighbors; j++ )
    {
      ASSERT_TRUE( indices_flann[i][j] == indices_tree[i][j] || dists_flann[i][j]==dists_tree[i][j]);
    }
  }

  delete flann_search;
  delete kdtree_search;
}

int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  init ();

  // Testing using explicit instantiation of inherited class
  pcl::search::FlannSearch<PointXYZ> FlannSearch ( new search::FlannSearch<PointXYZ>::KdTreeIndexCreator);
  FlannSearch.setInputCloud (cloud.makeShared ());

  return (RUN_ALL_TESTS ());
}
