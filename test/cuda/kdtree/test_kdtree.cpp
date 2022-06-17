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
 * test_kdtree.h
 * Adapted from: test/kdtree/test_kdtree.cpp
 * Created on: Jun 01, 2022
 * Author: Ramzi Sabra
 */

#include <pcl/cuda/kdtree/impl/kdtree_flann.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/distances.h>
#include <pcl/common/point_tests.h> // for pcl::isFinite
#include <pcl/common/time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/test/gtest.h>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

#include <algorithm>
#include <iostream>  // For debug
#include <map>
#include <unordered_set>

using namespace pcl;

boost::property_tree::ptree xml_property_tree;


PointCloud<PointXYZ>::Ptr cloud_in (new PointCloud<PointXYZ> ());

struct MyPoint : public PointXYZ 
{
    MyPoint (float x, float y, float z) {this->x=x; this->y=y; this->z=z;}
};

using PointVector = std::vector<MyPoint, Eigen::aligned_allocator<MyPoint>>;

using IndexMatrix = Eigen::Matrix<index_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
using DistanceMatrix = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

template <typename TupleType>
class PCLCUDAKdTreeTestFixture : public ::testing::Test
{
  public:
    using TreeMyPoint = std::tuple_element_t<0, TupleType>;
    using TreePointXY = std::tuple_element_t<1, TupleType>;
  
    PointCloud<MyPoint> cloud_, cloud_big_;

    PCLCUDAKdTreeTestFixture()
    {
      float resolution = 0.1f;
      for (float z = -0.5f; z <= 0.5f; z += resolution)
        for (float y = -0.5f; y <= 0.5f; y += resolution)
          for (float x = -0.5f; x <= 0.5f; x += resolution)
            cloud_.emplace_back(x, y, z);
      cloud_.width  = cloud_.size ();
      cloud_.height = 1;

      cloud_big_.width  = 640;
      cloud_big_.height = 480;
      srand (static_cast<unsigned int> (time (nullptr)));
      // Randomly create a new point cloud
      for (std::size_t i = 0; i < cloud_big_.width * cloud_big_.height; ++i)
        cloud_big_.emplace_back(static_cast<float> (1024 * rand () / (RAND_MAX + 1.0)),
                                            static_cast<float> (1024 * rand () / (RAND_MAX + 1.0)),
                                            static_cast<float> (1024 * rand () / (RAND_MAX + 1.0)));
      }
};

using KdTreeTestTypes = ::testing::Types<
  std::tuple<cuda::KdTreeFLANN<MyPoint>, cuda::KdTreeFLANN<PointXY>>
>;
TYPED_TEST_SUITE(PCLCUDAKdTreeTestFixture, KdTreeTestTypes);

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TYPED_TEST (PCLCUDAKdTreeTestFixture, KdTree_radiusSearch)
{
  using Tree = typename TestFixture::TreeMyPoint;
  
  auto& cloud = this->cloud_;
  auto& cloud_big = this->cloud_big_;
  
  Tree kdtree;
  kdtree.setInputCloud (cloud.makeShared ());
  MyPoint test_point(0.0f, 0.0f, 0.0f);
  unsigned int num_points = 4;
  double max_dist = 0.15;
  int max_nn = 100;
  std::set<int> brute_force_result;
  for (std::size_t i=0; i < cloud.size(); ++i)
    if (euclideanDistance(cloud[i], test_point) < max_dist)
      brute_force_result.insert(i);

  PointVector test_points;
  test_points.reserve(num_points);
  for (int i = 0; i < num_points; ++i)
    test_points.push_back(test_point);
  
  IndexMatrix indices_mat;
  DistanceMatrix distances_mat;
  kdtree.radiusSearch (test_points, max_dist, indices_mat, distances_mat, max_nn);
  
  std::vector<pcl::Indices> indices_vec;
  std::vector<std::vector<float>> distances_vec;
  kdtree.radiusSearch (test_points, max_dist, indices_vec, distances_vec, max_nn);
  
  //std::cout << k_indices.size()<<"=="<<brute_force_result.size()<<"?\n";

  EXPECT_EQ(indices_mat.rows(), num_points);
  EXPECT_EQ(indices_mat.cols(), max_nn);
  EXPECT_EQ(distances_mat.rows(), num_points);
  EXPECT_EQ(distances_mat.cols(), max_nn);

  EXPECT_EQ(indices_vec.size(), num_points);
  EXPECT_EQ(distances_vec.size(), num_points);
  
  // check if indices are equal across matrix and vector representations
  auto indices_row_itr = indices_mat.rowwise().cbegin();
  auto indices_itr = indices_vec.cbegin();
  
  for (; indices_itr != indices_vec.cend(); ++indices_row_itr, ++indices_itr)
  {
    auto indices_row = *indices_row_itr;
    const auto& indices = *indices_itr;

    EXPECT_LE(indices.size(), max_nn);

    std::vector<index_t> indices_row_vec;
    indices_row_vec.reserve(max_nn);
    
    // remove -1 for cases where (#NN for a point) < k (due to fixed number of columns = k in matrix)
    std::copy_if(
      indices_row.cbegin(),
      indices_row.cend(),
      std::back_inserter(indices_row_vec),
      [](const index_t& index)
      { return index != -1; }
    );

    EXPECT_EQ(indices_row_vec, indices);
  }

  // check if distances are equal across matrix and vector representations
  auto distances_row_itr = distances_mat.rowwise().cbegin();
  auto distances_itr = distances_vec.cbegin();

  for (; distances_itr != distances_vec.cend(); ++distances_row_itr, ++distances_itr)
  {
    auto distances_row = *distances_row_itr;
    const auto& distances = *distances_itr;

    EXPECT_LE(distances.size(), max_nn);

    std::vector<float> distances_row_vec;
    distances_row_vec.reserve(max_nn);
    
    // remove infinite distances for cases where (#NN for a point) < k (due to fixed number of columns = k in matrix)
    std::copy_if(
      distances_row.cbegin(),
      distances_row.cend(),
      std::back_inserter(distances_row_vec),
      [](const float& distance)
      { return std::isfinite(distance); }
    );

    EXPECT_EQ(distances_row_vec, distances);
  }
  
  for (const auto& indices : indices_vec)
  {
    auto brute_force_result_copy = brute_force_result;
    for (const auto &index : indices)
    {
      std::set<int>::iterator brute_force_result_it = brute_force_result_copy.find (index);
      bool ok = brute_force_result_it != brute_force_result_copy.end ();
      //if (!ok)  std::cerr << index << " is not correct...\n";
      //else      std::cerr << index << " is correct...\n";
      EXPECT_TRUE (ok);
      if (ok)
        brute_force_result_copy.erase (brute_force_result_it);
    }
    for (std::set<int>::const_iterator it=brute_force_result_copy.cbegin(); it!=brute_force_result_copy.cend(); ++it)
      std::cerr << "FLANN missed "<<*it<<"\n";
    
    bool error = !brute_force_result_copy.empty ();
    if (error)  std::cerr << "Missed too many neighbors!\n";
    EXPECT_FALSE (error);
  }

  {
    Tree kdtree;
    kdtree.setInputCloud (cloud_big.makeShared ());

    ScopeTime scopeTime ("FLANN CUDA radiusSearch");
    {
      kdtree.radiusSearch (cloud_big.points, 0.1, indices_vec, distances_vec);
    }
  }
  
  {
    Tree kdtree;
    kdtree.setInputCloud (cloud_big.makeShared ());

    ScopeTime scopeTime ("FLANN CUDA radiusSearch (max neighbors in radius)");
    {
      kdtree.radiusSearch (cloud_big.points, 0.1, indices_vec, distances_vec, 10);
    }
  }
  
  
  {
    Tree kdtree (false);
    kdtree.setInputCloud (cloud_big.makeShared ());

    ScopeTime scopeTime ("FLANN CUDA radiusSearch (unsorted results)");
    {
      kdtree.radiusSearch (cloud_big.points, 0.1, indices_vec, distances_vec);
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TYPED_TEST (PCLCUDAKdTreeTestFixture, KdTree_nearestKSearch)
{
  using Tree = typename TestFixture::TreeMyPoint;
  
  auto& cloud = this->cloud_;
  auto& cloud_big = this->cloud_big_;
  
  Tree kdtree;
  kdtree.setInputCloud (cloud.makeShared ());
  MyPoint test_point (0.01f, 0.01f, 0.01f);
  unsigned int num_points = 4;
  unsigned int no_of_neighbors = 20;
  std::multimap<float, int> sorted_brute_force_result;
  for (std::size_t i = 0; i < cloud.size (); ++i)
  {
    float distance = euclideanDistance (cloud[i], test_point);
    sorted_brute_force_result.insert (std::make_pair (distance, static_cast<int> (i)));
  }
  float max_dist = 0.0f;
  unsigned int counter = 0;
  for (std::multimap<float, int>::iterator it = sorted_brute_force_result.begin (); it != sorted_brute_force_result.end () && counter < no_of_neighbors; ++it)
  {
    max_dist = std::max (max_dist, it->first);
    ++counter;
  }
  
  PointVector test_points;
  test_points.reserve(num_points);
  for (int i = 0; i < num_points; ++i)
    test_points.push_back(test_point);
  
  IndexMatrix k_indices_mat;
  DistanceMatrix k_distances_mat;
  kdtree.nearestKSearch (test_points, no_of_neighbors, k_indices_mat, k_distances_mat);
  
  std::vector<pcl::Indices> k_indices_vec;
  std::vector<std::vector<float>> k_distances_vec;
  kdtree.nearestKSearch (test_points, no_of_neighbors, k_indices_vec, k_distances_vec);
  
  EXPECT_EQ(k_indices_mat.rows(), num_points);
  EXPECT_EQ(k_indices_mat.cols(), no_of_neighbors);
  EXPECT_EQ(k_distances_mat.rows(), num_points);
  EXPECT_EQ(k_distances_mat.cols(), no_of_neighbors);

  EXPECT_EQ(k_indices_vec.size(), num_points);
  EXPECT_EQ(k_distances_vec.size(), num_points);
  
  // check if indices are equal across matrix and vector representations
  auto k_indices_row_itr = k_indices_mat.rowwise().cbegin();
  auto k_indices_itr = k_indices_vec.cbegin();
  
  for (; k_indices_itr != k_indices_vec.cend(); ++k_indices_row_itr, ++k_indices_itr)
  {
    auto k_indices_row = *k_indices_row_itr;
    const auto& k_indices = *k_indices_itr;

    EXPECT_EQ(k_indices.size(), no_of_neighbors);

    std::vector<index_t> k_indices_row_vec (no_of_neighbors);
    
    std::copy(
      k_indices_row.cbegin(),
      k_indices_row.cend(),
      k_indices_row_vec.begin()
    );

    EXPECT_EQ(k_indices_row_vec, k_indices);
  }

  // check if distances are equal across matrix and vector representations
  auto k_distances_row_itr = k_distances_mat.rowwise().cbegin();
  auto k_distances_itr = k_distances_vec.cbegin();

  for (; k_distances_itr != k_distances_vec.cend(); ++k_distances_row_itr, ++k_distances_itr)
  {
    auto k_distances_row = *k_distances_row_itr;
    const auto& k_distances = *k_distances_itr;

    EXPECT_EQ(k_distances.size(), no_of_neighbors);

    std::vector<float> k_distances_row_vec (no_of_neighbors);
    
    std::copy(
      k_distances_row.cbegin(),
      k_distances_row.cend(),
      k_distances_row_vec.begin()
    );

    EXPECT_EQ(k_distances_row_vec, k_distances);
  }

  for (const auto& k_indices : k_indices_vec)
  {
    EXPECT_EQ (k_indices.size (), no_of_neighbors);

    // Check if all found neighbors have distance smaller than max_dist
    for (const auto &k_index : k_indices)
    {
      const MyPoint& point = cloud[k_index];
      bool ok = euclideanDistance (test_point, point) <= max_dist;
      if (!ok)
        ok = (std::abs (euclideanDistance (test_point, point)) - max_dist) <= 1e-6;
      //if (!ok)  std::cerr << k_index << " is not correct...\n";
      //else      std::cerr << k_index << " is correct...\n";
      EXPECT_TRUE (ok);
    }
  }

  ScopeTime scopeTime ("FLANN CUDA nearestKSearch");
  {
    Tree kdtree;
    kdtree.setInputCloud (cloud_big.makeShared ());
    kdtree.nearestKSearch (cloud_big.points, no_of_neighbors, k_indices_vec, k_distances_vec);
  }
}

class MyPointRepresentationXYZ : public PointRepresentation<MyPoint>
{
  public:
    MyPointRepresentationXYZ ()
    {
      this->nr_dimensions_ = 3;
    }

    void copyToFloatArray (const MyPoint &p, float *out) const override
    {
      out[0] = p.x;
      out[1] = p.y;
      out[2] = p.z;
    }
};

class MyPointRepresentationXY : public PointRepresentation<MyPoint>
{
  public:
    MyPointRepresentationXY ()
    {
      this->nr_dimensions_ = 2;
    }

    void copyToFloatArray (const MyPoint &p, float *out) const override
    {
      out[0] = p.x;
      out[1] = p.y;
    }
};

TYPED_TEST (PCLCUDAKdTreeTestFixture, KdTree_setPointRepresentation)
{
  using Tree = typename TestFixture::TreeMyPoint;
  using TreePointXY = typename TestFixture::TreePointXY;
  
  PointCloud<MyPoint>::Ptr random_cloud (new PointCloud<MyPoint> ());
  random_cloud->points.emplace_back(86.6f, 42.1f, 92.4f);
  random_cloud->points.emplace_back(63.1f, 18.4f, 22.3f);
  random_cloud->points.emplace_back(35.5f, 72.5f, 37.3f);
  random_cloud->points.emplace_back(99.7f, 37.0f,  8.7f);
  random_cloud->points.emplace_back(22.4f, 84.1f, 64.0f);
  random_cloud->points.emplace_back(65.2f, 73.4f, 18.0f);
  random_cloud->points.emplace_back(60.4f, 57.1f,  4.5f);
  random_cloud->points.emplace_back(38.7f, 17.6f, 72.3f);
  random_cloud->points.emplace_back(14.2f, 95.7f, 34.7f);
  random_cloud->points.emplace_back( 2.5f, 26.5f, 66.0f);

  Tree kdtree;
  kdtree.setInputCloud (random_cloud);
  MyPoint test_point (50.0f, 50.0f, 50.0f);
  unsigned int num_points = 4;

  PointVector test_points;
  test_points.reserve(num_points);
  for (int i = 0; i < num_points; ++i)
    test_points.push_back(test_point);
  
  const int k = 10;
  std::vector<pcl::Indices> k_indices_vec(num_points);
  std::vector<std::vector<float>> k_distances_vec(num_points);
  
  // Find k nearest neighbors
  {
    kdtree.nearestKSearch (test_points, k, k_indices_vec, k_distances_vec);

    auto k_indices_itr = k_indices_vec.cbegin();
    auto k_distances_itr = k_distances_vec.cbegin();

    for (; k_indices_itr != k_indices_vec.cend(); ++k_indices_itr, ++k_distances_itr)
    {
      auto& k_indices = *k_indices_itr;
      auto& k_distances = *k_distances_itr;
      
      for (int i = 0; i < k; ++i)
      {
        // Compare to ground truth values, computed independently
        static const int gt_indices[10] = {2, 7, 5, 1, 4, 6, 9, 0, 8, 3};
        static const float gt_distances[10] =
        {877.8f, 1674.7f, 1802.6f, 1937.5f, 2120.6f, 2228.8f, 3064.5f, 3199.7f, 3604.2f, 4344.8f};
        EXPECT_EQ (k_indices[i], gt_indices[i]);
        EXPECT_NEAR (k_distances[i], gt_distances[i], 0.1);
      }
    }
  }
  
  // Find k nearest neighbors with a different point representation
  {
    k_indices_vec.clear();
    k_distances_vec.clear();
    
    typename Tree::PointRepresentationConstPtr ptrep (new MyPointRepresentationXYZ);
    kdtree.setPointRepresentation (ptrep);
    kdtree.nearestKSearch (test_points, k, k_indices_vec, k_distances_vec);
    
    auto k_indices_itr = k_indices_vec.cbegin();
    auto k_distances_itr = k_distances_vec.cbegin();

    for (; k_indices_itr != k_indices_vec.cend(); ++k_indices_itr, ++k_distances_itr)
    {
      auto& k_indices = *k_indices_itr;
      auto& k_distances = *k_distances_itr;
      
      for (int i = 0; i < k; ++i)
      {
        // Compare to ground truth values, computed independently
        static const int gt_indices[10] = {2, 7, 5, 1, 4, 6, 9, 0, 8, 3};
        static const float gt_distances[10] =
        {877.8f, 1674.7f, 1802.6f, 1937.5f, 2120.6f, 2228.8f, 3064.5f, 3199.7f, 3604.2f, 4344.8f};
        EXPECT_EQ (k_indices[i], gt_indices[i]);
        EXPECT_NEAR (k_distances[i], gt_distances[i], 0.1);
      }
    }
  }

  // Go back to the default, this time with the values rescaled
  {
    DefaultPointRepresentation<MyPoint> point_rep;
    float alpha[3] = {1.0f, 2.0f, 3.0f};
    point_rep.setRescaleValues(alpha);
    kdtree.setPointRepresentation (point_rep.makeShared ());
    kdtree.nearestKSearch (test_points, k, k_indices_vec, k_distances_vec);

    auto k_indices_itr = k_indices_vec.cbegin();
    auto k_distances_itr = k_distances_vec.cbegin();

    for (; k_indices_itr != k_indices_vec.cend(); ++k_indices_itr, ++k_distances_itr)
    {
      auto& k_indices = *k_indices_itr;
      auto& k_distances = *k_distances_itr;

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
  }

  // Construct a kd-tree for points with 2 dimensions
  EXPECT_THROW (TreePointXY(), std::domain_error);

  // Set a point representation with 2 dimensions
  {
    typename Tree::PointRepresentationConstPtr ptrep (new MyPointRepresentationXY);
    EXPECT_THROW(kdtree.setPointRepresentation(ptrep), std::domain_error);
  }
}

TYPED_TEST (PCLCUDAKdTreeTestFixture, KdTree_copy)
{
  using Tree = typename TestFixture::TreeMyPoint;

  auto& cloud = this->cloud_;

  MyPoint test_point (0.01f, 0.01f, 0.01f);
  const int k = 10;
  const int max_nn = 10;
  unsigned int num_points = 4;

  PointVector test_points;
  test_points.reserve(num_points);
  for (int i = 0; i < num_points; ++i)
    test_points.push_back(test_point);

  Tree kdtree;
  kdtree.setInputCloud (cloud.makeShared ());
  
  std::vector<pcl::Indices> k_indices_vec;
  std::vector<std::vector<float>> k_distances_vec;
  kdtree.nearestKSearch (test_points, k, k_indices_vec, k_distances_vec);
  
  std::vector<pcl::Indices> indices_vec;
  std::vector<std::vector<float>> distances_vec;
  kdtree.radiusSearch (test_points, max_nn, indices_vec, distances_vec);
  
  Tree kdtree_copy (kdtree);
  
  std::vector<pcl::Indices> k_indices_vec_copy;
  std::vector<std::vector<float>> k_distances_vec_copy;
  kdtree_copy.nearestKSearch (test_points, k, k_indices_vec_copy, k_distances_vec_copy);

  std::vector<pcl::Indices> indices_vec_copy;
  std::vector<std::vector<float>> distances_vec_copy;
  kdtree_copy.radiusSearch (test_points, max_nn, indices_vec_copy, distances_vec_copy);

  EXPECT_EQ (k_indices_vec, k_indices_vec_copy);
  EXPECT_EQ (k_distances_vec, k_distances_vec_copy);
  EXPECT_EQ (indices_vec, indices_vec_copy);
  EXPECT_EQ (distances_vec, distances_vec_copy);
}

TYPED_TEST (PCLCUDAKdTreeTestFixture, KdTree_setMaxLeafSize)
{
  using Tree = typename TestFixture::TreeMyPoint;

  auto& cloud = this->cloud_;

  MyPoint test_point (0.01f, 0.01f, 0.01f);
  const int k = 10;
  unsigned int num_points = 4;

  PointVector test_points;
  test_points.reserve(num_points);
  for (int i = 0; i < num_points; ++i)
    test_points.push_back(test_point);

  Tree kdtree;
  kdtree.setInputCloud (cloud.makeShared ());
  
  std::vector<pcl::Indices> k_indices_default_vec;
  std::vector<std::vector<float>> k_distances_default_vec;
  kdtree.nearestKSearch (test_points, k, k_indices_default_vec, k_distances_default_vec);

  kdtree.setMaxLeafSize (50);
  
  std::vector<pcl::Indices> k_indices_changed_vec;
  std::vector<std::vector<float>> k_distances_changed_vec;
  kdtree.nearestKSearch (test_points, k, k_indices_changed_vec, k_distances_changed_vec);

  auto k_indices_default_itr = k_indices_default_vec.cbegin();
  auto k_indices_changed_itr = k_indices_changed_vec.cbegin();

  for (; k_indices_default_itr != k_indices_default_vec.cend(); ++k_indices_default_itr, ++k_indices_changed_itr)
  {
    const pcl::Indices& k_indices_default = *k_indices_default_itr;
    const pcl::Indices& k_indices_changed = *k_indices_changed_itr;

    std::unordered_set<pcl::index_t> k_indices_set_default, k_indices_set_changed;
  
    std::copy(
      k_indices_default.cbegin(),
      k_indices_default.cend(),
      std::inserter(
        k_indices_set_default,
        k_indices_set_default.end()
      )
    );
    
    std::copy(
      k_indices_changed.cbegin(),
      k_indices_changed.cend(),
      std::inserter(
        k_indices_set_changed,
        k_indices_set_changed.end()
      )
    );

    EXPECT_EQ (k_indices_set_default, k_indices_set_changed);
  }

  auto k_distances_default_itr = k_distances_default_vec.cbegin();
  auto k_distances_changed_itr = k_distances_changed_vec.cbegin();

  for (; k_distances_default_itr != k_distances_default_vec.cend(); ++k_distances_default_itr, ++k_distances_changed_itr)
  {
    const std::vector<float>& k_distances_default = *k_distances_default_itr;
    const std::vector<float>& k_distances_changed = *k_distances_changed_itr;

    EXPECT_EQ (k_distances_default, k_distances_changed);
  }
}

/* ---[ */
int
main (int argc, char** argv)
{
  // Load the standard PCD file from disk
  if (argc < 3)
  {
    std::cerr << "No test file given. Please download `sac_plane_test.pcd` and 'kdtree_unit_test_results.xml' pass them path to the test." << std::endl;
    return (-1);
  }

  // Load in the point clouds
  io::loadPCDFile (argv[1], *cloud_in);

  std::ifstream xml_file_input_stream (argv[2], std::ifstream::in);
  read_xml (xml_file_input_stream, xml_property_tree, boost::property_tree::xml_parser::trim_whitespace);

  testing::InitGoogleTest (&argc, argv);

  return (RUN_ALL_TESTS ());
}
/* ]--- */
