/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2025-, Open Perception Inc.
 *
 *  All rights reserved
 */

#define PCL_NO_PRECOMPILE 1 // for MyPoint instantiations
#include <pcl/common/distances.h>
#include <pcl/common/io.h> // for copyPointCloud
#include <pcl/common/time.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>           // for pcl::search::KdTree
#include <pcl/search/kdtree_nanoflann.h> // for pcl::search::KdTreeNanoflann
#include <pcl/test/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <iostream>

namespace pcl_tests {
inline double
squared_point_distance(const pcl::Narf36& p1, const pcl::Narf36& p2)
{
  double dist = 0.0;
  for (std::size_t i = 0; i < 36; ++i)
    dist += std::pow(static_cast<double>(p1.descriptor[i]) -
                         static_cast<double>(p2.descriptor[i]),
                     2);
  return dist;
}
} // namespace pcl_tests

using namespace pcl;

// many of the tests are copied and adapted form test/search/test_flann_search.cpp and
// test/kdtree/test_kdtree.cpp

PointCloud<PointXYZ> cloud, cloud_big, bun0_cloud;

void
init()
{
  float resolution = 0.1f;
  for (float z = -0.5f; z <= 0.5f; z += resolution)
    for (float y = -0.5f; y <= 0.5f; y += resolution)
      for (float x = -0.5f; x <= 0.5f; x += resolution)
        cloud.emplace_back(static_cast<float>(rand()) / (RAND_MAX + 1.0) - 0.5,
                           static_cast<float>(rand()) / (RAND_MAX + 1.0) - 0.5,
                           static_cast<float>(rand()) / (RAND_MAX + 1.0) - 0.5);
  cloud.width = cloud.size();
  cloud.height = 1;

  // Randomly create a new point cloud, use points.emplace_back
  cloud_big.width = 640 * 480;
  cloud_big.height = 1;
  for (std::size_t i = 0; i < cloud_big.width * cloud_big.height; ++i)
    cloud_big.points.emplace_back(static_cast<float>(1024 * rand() / (RAND_MAX + 1.0)),
                                  static_cast<float>(1024 * rand() / (RAND_MAX + 1.0)),
                                  static_cast<float>(1024 * rand() / (RAND_MAX + 1.0)));
}

TEST(PCL, KdTreeNanoflann_nearestKSearch)
{
  auto kdtree = pcl::make_shared<pcl::search::KdTreeNanoflann<PointXYZ>>();
  kdtree->setInputCloud(cloud.makeShared());
  EXPECT_EQ(kdtree->getEpsilon(), 0.0f);
  kdtree->setEpsilon(1.23f);
  EXPECT_EQ(kdtree->getEpsilon(), 1.23f);
  kdtree->setEpsilon(0.0f);
  PointXYZ test_point(0.01f, 0.01f, 0.01f);
  unsigned int no_of_neighbors = 20;
  std::multimap<float, int> sorted_brute_force_result;
  for (std::size_t i = 0; i < cloud.size(); ++i) {
    float distance = euclideanDistance(cloud[i], test_point);
    sorted_brute_force_result.insert(std::make_pair(distance, static_cast<int>(i)));
  }
  float max_dist = 0.0f;
  unsigned int counter = 0;
  for (auto it = sorted_brute_force_result.begin();
       it != sorted_brute_force_result.end() && counter < no_of_neighbors;
       ++it) {
    max_dist = std::max(max_dist, it->first);
    ++counter;
  }

  pcl::Indices k_indices;
  k_indices.resize(no_of_neighbors);
  std::vector<float> k_distances;
  k_distances.resize(no_of_neighbors);

  kdtree->nearestKSearch(test_point, no_of_neighbors, k_indices, k_distances);

  EXPECT_EQ(k_indices.size(), no_of_neighbors);

  // Check if all found neighbors have distance smaller than max_dist
  for (const auto& k_index : k_indices) {
    const PointXYZ& point = cloud[k_index];
    bool ok = euclideanDistance(test_point, point) <= max_dist;
    if (!ok)
      ok = (std::abs(euclideanDistance(test_point, point)) - max_dist) <= 1e-6;
    EXPECT_TRUE(ok);
  }

  kdtree->setUseRKNN(true);
  pcl::Indices k_indices_rknn;
  std::vector<float> k_distances_rknn;
  // first test with search radius bigger than farthest neighbor, results should be the
  // same
  kdtree->radiusSearch(test_point,
                       max_dist + std::numeric_limits<float>::epsilon(),
                       k_indices_rknn,
                       k_distances_rknn,
                       no_of_neighbors);
  EXPECT_EQ(k_indices_rknn.size(), k_indices.size());
  EXPECT_EQ(k_distances_rknn.size(), k_distances.size());
  for (std::size_t i = 0;
       i < std::min<std::size_t>(k_indices.size(), k_indices_rknn.size());
       ++i) {
    EXPECT_EQ(k_indices[i], k_indices_rknn[i]);
    EXPECT_EQ(k_distances[i], k_distances_rknn[i]);
  }
  // then test with search radius slightly smaller than farthest neighbor, should have
  // one neighbor less
  kdtree->radiusSearch(test_point,
                       max_dist - std::numeric_limits<float>::epsilon(),
                       k_indices_rknn,
                       k_distances_rknn,
                       no_of_neighbors);
  EXPECT_EQ(k_indices_rknn.size() + 1, k_indices.size());
  EXPECT_EQ(k_distances_rknn.size() + 1, k_distances.size());
  for (std::size_t i = 0;
       i < std::min<std::size_t>(k_indices.size(), k_indices_rknn.size());
       ++i) {
    EXPECT_EQ(k_indices[i], k_indices_rknn[i]);
    EXPECT_EQ(k_distances[i], k_distances_rknn[i]);
  }
}

/* Test the templated NN search (for different query point types) */
TEST(PCL, KdTreeNanoflannSearch_differentPointT)
{

  unsigned int no_of_neighbors = 20;

  pcl::search::KdTreeNanoflann<PointXYZ> kdtree;
  kdtree.setInputCloud(cloud_big.makeShared());

  PointCloud<PointXYZRGB> cloud_rgb;

  copyPointCloud(cloud_big, cloud_rgb);

  std::vector<std::vector<float>> dists;
  std::vector<pcl::Indices> indices;
  kdtree.nearestKSearchT(cloud_rgb, pcl::Indices(), no_of_neighbors, indices, dists);

  pcl::Indices k_indices;
  k_indices.resize(no_of_neighbors);
  std::vector<float> k_distances;
  k_distances.resize(no_of_neighbors);

  for (std::size_t i = 0; i < cloud_rgb.size(); ++i) {
    kdtree.nearestKSearch(cloud_big[i], no_of_neighbors, k_indices, k_distances);
    EXPECT_EQ(k_indices.size(), indices[i].size());
    EXPECT_EQ(k_distances.size(), dists[i].size());
    for (std::size_t j = 0; j < no_of_neighbors; j++) {
      EXPECT_TRUE(k_indices[j] == indices[i][j] || k_distances[j] == dists[i][j]);
    }
  }
}

/* Test for KdTreeNanoflann nearestKSearch with multiple query points */
TEST(PCL, KdTreeNanoflann_multipointKnnSearch)
{

  unsigned int no_of_neighbors = 20;

  pcl::search::KdTreeNanoflann<PointXYZ> kdtree;
  kdtree.setInputCloud(cloud_big.makeShared());

  std::vector<std::vector<float>> dists;
  std::vector<pcl::Indices> indices;
  kdtree.nearestKSearch(cloud_big, pcl::Indices(), no_of_neighbors, indices, dists);

  pcl::Indices k_indices;
  k_indices.resize(no_of_neighbors);
  std::vector<float> k_distances;
  k_distances.resize(no_of_neighbors);

  for (std::size_t i = 0; i < cloud_big.size(); ++i) {
    kdtree.nearestKSearch(cloud_big[i], no_of_neighbors, k_indices, k_distances);
    EXPECT_EQ(k_indices.size(), indices[i].size());
    EXPECT_EQ(k_distances.size(), dists[i].size());
    for (std::size_t j = 0; j < no_of_neighbors; j++) {
      EXPECT_TRUE(k_indices[j] == indices[i][j] || k_distances[j] == dists[i][j]);
    }
  }
}

/* Test for KdTreeNanoflann nearestKSearch with multiple query points */
TEST(PCL, KdTreeNanoflann_knnByIndex)
{

  unsigned int no_of_neighbors = 3;

  pcl::search::KdTreeNanoflann<PointXYZ> kdtree;
  kdtree.setInputCloud(cloud_big.makeShared());

  std::vector<std::vector<float>> dists;
  std::vector<pcl::Indices> indices;
  pcl::Indices query_indices;
  for (std::size_t i = 0; i < cloud_big.size(); i += 2) {
    query_indices.push_back(static_cast<int>(i));
  }
  kdtree.nearestKSearch(cloud_big, query_indices, no_of_neighbors, indices, dists);

  pcl::Indices k_indices;
  k_indices.resize(no_of_neighbors);
  std::vector<float> k_distances;
  k_distances.resize(no_of_neighbors);

  for (std::size_t i = 0; i < query_indices.size(); ++i) {
    kdtree.nearestKSearch(cloud_big[2 * i], no_of_neighbors, k_indices, k_distances);
    EXPECT_EQ(k_indices.size(), indices[i].size());
    EXPECT_EQ(k_distances.size(), dists[i].size());
    for (std::size_t j = 0; j < no_of_neighbors; j++) {
      EXPECT_TRUE(k_indices[j] == indices[i][j] || k_distances[j] == dists[i][j]);
    }
    kdtree.nearestKSearch(
        cloud_big, query_indices[i], no_of_neighbors, k_indices, k_distances);
    EXPECT_EQ(k_indices.size(), indices[i].size());
    EXPECT_EQ(k_distances.size(), dists[i].size());
    for (std::size_t j = 0; j < no_of_neighbors; j++) {
      EXPECT_TRUE(k_indices[j] == indices[i][j] || k_distances[j] == dists[i][j]);
    }
  }
}

TEST(PCL, KdTreeNanoflann_compareToKdTreeFlann)
{

  int no_of_neighbors = 3;
  pcl::Indices k_indices;
  k_indices.resize(no_of_neighbors);
  std::vector<float> k_distances;
  k_distances.resize(no_of_neighbors);

  pcl::search::Search<PointXYZ>*nanoflann_search, *kdtree_search;

  PointCloud<PointXYZ>::Ptr pc = cloud_big.makeShared();
  {
    ScopeTime scopeTime("KdTreeNanoflann build");
    nanoflann_search = new pcl::search::KdTreeNanoflann<PointXYZ>;
    nanoflann_search->setInputCloud(pc);
  }

  {
    ScopeTime scopeTime("kdtree build");
    kdtree_search = new pcl::search::KdTree<PointXYZ>();
    kdtree_search->setInputCloud(pc);
  }

  {
    ScopeTime scopeTime("KdTreeNanoflann nearestKSearch");
    for (const auto& point : cloud_big.points)
      nanoflann_search->nearestKSearch(point, no_of_neighbors, k_indices, k_distances);
  }
  {
    ScopeTime scopeTime("kd tree  nearestKSearch");
    for (const auto& point : cloud_big.points)
      kdtree_search->nearestKSearch(point, no_of_neighbors, k_indices, k_distances);
  }

  std::vector<pcl::Indices> indices_nanoflann;
  std::vector<std::vector<float>> dists_nanoflann;
  std::vector<pcl::Indices> indices_tree;
  std::vector<std::vector<float>> dists_tree;
  indices_nanoflann.resize(cloud_big.size());
  dists_nanoflann.resize(cloud_big.size());
  indices_tree.resize(cloud_big.size());
  dists_tree.resize(cloud_big.size());
  for (std::size_t i = 0; i < cloud_big.size(); ++i) {
    indices_nanoflann[i].resize(no_of_neighbors);
    dists_nanoflann[i].resize(no_of_neighbors);
    indices_tree[i].resize(no_of_neighbors);
    dists_tree[i].resize(no_of_neighbors);
  }

  {
    ScopeTime scopeTime("KdTreeNanoflann multi nearestKSearch");
    nanoflann_search->nearestKSearch(
        cloud_big, pcl::Indices(), no_of_neighbors, indices_nanoflann, dists_nanoflann);
  }
  {
    ScopeTime scopeTime("kd tree multi nearestKSearch");
    kdtree_search->nearestKSearch(
        cloud_big, pcl::Indices(), no_of_neighbors, indices_tree, dists_tree);
  }

  ASSERT_EQ(indices_nanoflann.size(), dists_nanoflann.size());
  ASSERT_EQ(indices_nanoflann.size(), indices_tree.size());
  ASSERT_EQ(indices_nanoflann.size(), dists_tree.size());

  for (std::size_t i = 0; i < indices_nanoflann.size(); i++) {
    ASSERT_EQ(indices_nanoflann[i].size(), no_of_neighbors);
    ASSERT_EQ(indices_tree[i].size(), no_of_neighbors);
    ASSERT_EQ(dists_nanoflann[i].size(), no_of_neighbors);
    ASSERT_EQ(dists_tree[i].size(), no_of_neighbors);
    for (int j = 0; j < no_of_neighbors; j++) {

      ASSERT_TRUE(indices_nanoflann[i][j] == indices_tree[i][j] ||
                  dists_nanoflann[i][j] == dists_tree[i][j]);
    }
  }

  pcl::Indices query_indices;
  for (std::size_t i = 0; i < cloud_big.size(); i += 2)
    query_indices.push_back(static_cast<int>(i));

  {
    ScopeTime scopeTime("KdTreeNanoflann multi nearestKSearch with indices");
    nanoflann_search->nearestKSearch(
        cloud_big, query_indices, no_of_neighbors, indices_nanoflann, dists_nanoflann);
  }
  {
    ScopeTime scopeTime("kd tree multi nearestKSearch with indices");
    kdtree_search->nearestKSearch(
        cloud_big, query_indices, no_of_neighbors, indices_tree, dists_tree);
  }
  ASSERT_EQ(indices_nanoflann.size(), dists_nanoflann.size());
  ASSERT_EQ(indices_nanoflann.size(), indices_tree.size());
  ASSERT_EQ(indices_nanoflann.size(), dists_tree.size());

  for (std::size_t i = 0; i < indices_nanoflann.size(); i++) {
    ASSERT_EQ(indices_nanoflann[i].size(), no_of_neighbors);
    ASSERT_EQ(indices_tree[i].size(), no_of_neighbors);
    ASSERT_EQ(dists_nanoflann[i].size(), no_of_neighbors);
    ASSERT_EQ(dists_tree[i].size(), no_of_neighbors);
    for (int j = 0; j < no_of_neighbors; j++) {
      ASSERT_TRUE(indices_nanoflann[i][j] == indices_tree[i][j] ||
                  dists_nanoflann[i][j] == dists_tree[i][j]);
    }
  }

  delete nanoflann_search;
  delete kdtree_search;
}

// test with FPFHSignature33
TEST(PCL, KdTreeNanoflann_features)
{
  using FeatureT = FPFHSignature33;

  // Create shared pointers
  pcl::PointCloud<PointXYZ>::Ptr cloud_source_ptr;
  cloud_source_ptr = bun0_cloud.makeShared();

  // first estimate normals
  PointCloud<Normal>::Ptr normals(new PointCloud<Normal>);
  pcl::NormalEstimation<PointXYZ, Normal> norm_est;
  norm_est.setRadiusSearch(0.05);
  norm_est.setInputCloud(cloud_source_ptr);
  norm_est.compute(*normals);

  // then estimate features
  PointCloud<FeatureT>::Ptr features_source(new PointCloud<FeatureT>);
  pcl::FPFHEstimation<PointXYZ, Normal, FeatureT> fpfh_est;
  fpfh_est.setRadiusSearch(0.05);
  fpfh_est.setInputCloud(cloud_source_ptr);
  fpfh_est.setInputNormals(normals);
  fpfh_est.compute(*features_source);

  pcl::search::KdTree<FeatureT> kdtree;
  kdtree.setInputCloud(features_source);

  pcl::search::KdTreeNanoflann<FeatureT, -1> kdtree_nanoflann;
  kdtree_nanoflann.setInputCloud(features_source);

  pcl::Indices indices, indices_nanoflann;
  std::vector<float> dists, dists_nanoflann;

  // test nearestKSearch
  ASSERT_TRUE(std::isfinite((*features_source)[10].histogram[0]));
  kdtree.nearestKSearch((*features_source)[10], 10, indices, dists);
  kdtree_nanoflann.nearestKSearch(
      (*features_source)[10], 10, indices_nanoflann, dists_nanoflann);
  ASSERT_EQ(indices.size(), indices_nanoflann.size());
  ASSERT_EQ(dists.size(), dists_nanoflann.size());
  ASSERT_EQ(indices.size(), dists.size());
  for (std::size_t i = 0; i < indices.size(); ++i) {
    EXPECT_EQ(indices[i], indices_nanoflann[i]);
    EXPECT_NEAR(dists[i], dists_nanoflann[i], 1e-4);
  }

  ASSERT_TRUE(std::isfinite((*features_source)[99].histogram[0]));
  kdtree.nearestKSearch((*features_source)[99], 5, indices, dists);
  kdtree_nanoflann.nearestKSearch(
      (*features_source)[99], 5, indices_nanoflann, dists_nanoflann);
  ASSERT_EQ(indices.size(), indices_nanoflann.size());
  ASSERT_EQ(dists.size(), dists_nanoflann.size());
  ASSERT_EQ(indices.size(), dists.size());
  for (std::size_t i = 0; i < indices.size(); ++i) {
    EXPECT_EQ(indices[i], indices_nanoflann[i]);
    EXPECT_NEAR(dists[i], dists_nanoflann[i], 1e-4);
  }
}

// test with Narf36, because it has a non-trivial point representation
TEST(PCL, KdTreeNanoflann_Narf36)
{
  using FeatureT = pcl::Narf36;
  auto feature_cloud = pcl::make_shared<pcl::PointCloud<FeatureT>>();
  feature_cloud->resize(1000);
  for (auto& feature : *feature_cloud) {
    feature.x = 20.0 * rand() / RAND_MAX - 10.0;
    feature.y = 20.0 * rand() / RAND_MAX - 10.0;
    feature.z = 20.0 * rand() / RAND_MAX - 10.0;
    feature.roll = M_PI * rand() / RAND_MAX;
    feature.pitch = M_PI * rand() / RAND_MAX;
    feature.yaw = M_PI * rand() / RAND_MAX;
    for (float& desc : feature.descriptor) {
      desc = 2.0 * rand() / RAND_MAX - 1.0;
    }
  }
  pcl::search::KdTree<FeatureT> kdtree;
  kdtree.setInputCloud(feature_cloud);

  pcl::search::KdTreeNanoflann<FeatureT, 36> kdtree_nanoflann;
  kdtree_nanoflann.setSortedResults(true);
  kdtree_nanoflann.setInputCloud(feature_cloud);

  pcl::Indices indices, indices_nanoflann;
  std::vector<float> dists, dists_nanoflann;

  // test nearestKSearch
  for (std::size_t j = 0; j < feature_cloud->size(); ++j) {
    const int k = static_cast<int>(99.0 * rand() / RAND_MAX + 1);
    kdtree.nearestKSearch((*feature_cloud)[j], k, indices, dists);
    kdtree_nanoflann.nearestKSearch(
        (*feature_cloud)[j], k, indices_nanoflann, dists_nanoflann);
    ASSERT_EQ(indices.size(), indices_nanoflann.size());
    ASSERT_EQ(dists.size(), dists_nanoflann.size());
    ASSERT_EQ(indices.size(), dists.size());
    ASSERT_EQ(indices_nanoflann.size(), dists_nanoflann.size());
    for (std::size_t i = 0; i < indices.size(); ++i) {
      // sometimes, two neighbors have almost the same distance to the query point, but
      // are switched due to limited floating point accuracy.
      if ((i + 1) < indices.size() && indices[i] != indices_nanoflann[i] &&
          indices[i + 1] != indices_nanoflann[i + 1] &&
          (indices[i] == indices_nanoflann[i + 1] ||
           indices[i + 1] == indices_nanoflann[i]) &&
          std::abs(dists[i] - dists_nanoflann[i + 1]) < 1e-4 &&
          std::abs(dists[i + 1] - dists_nanoflann[i]) < 1e-4) {
        std::swap(indices[i], indices[i + 1]);
        std::swap(dists[i], dists[i + 1]);
      }
    }
    if (!indices.empty() && indices.back() != indices_nanoflann.back() &&
        std::abs(dists.back() - dists_nanoflann.back()) < 1e-5 &&
        std::abs(pcl_tests::squared_point_distance((*feature_cloud)[j],
                                                   (*feature_cloud)[indices.back()]) -
                 pcl_tests::squared_point_distance(
                     (*feature_cloud)[j], (*feature_cloud)[indices_nanoflann.back()])) <
            1e-5) {
      indices_nanoflann.back() = indices.back();
    }
    for (std::size_t i = 0; i < indices.size(); ++i) {
      EXPECT_EQ(indices[i], indices_nanoflann[i]);
      EXPECT_NEAR(dists[i], dists_nanoflann[i], 1e-4);
    }
  }

  // test radiusSearch
  for (std::size_t i = 0; i < feature_cloud->size(); ++i) {
    const float radius = 3.5 + static_cast<float>(rand()) / RAND_MAX;
    kdtree.radiusSearch((*feature_cloud)[i], radius, indices, dists);
    kdtree_nanoflann.radiusSearch(
        (*feature_cloud)[i], radius, indices_nanoflann, dists_nanoflann);
    if (indices.size() > indices_nanoflann.size()) {
      if (std::abs(pcl_tests::squared_point_distance((*feature_cloud)[i],
                                                     (*feature_cloud)[indices.back()]) -
                   static_cast<double>(radius) * static_cast<double>(radius)) < 1e-5) {
        indices.pop_back();
        dists.pop_back();
      }
    }
    else if (indices.size() < indices_nanoflann.size()) {
      if (std::abs(
              pcl_tests::squared_point_distance(
                  (*feature_cloud)[i], (*feature_cloud)[indices_nanoflann.back()]) -
              static_cast<double>(radius) * static_cast<double>(radius)) < 1e-5) {
        indices_nanoflann.pop_back();
        dists_nanoflann.pop_back();
      }
    }
    ASSERT_EQ(indices.size(), indices_nanoflann.size());
    ASSERT_EQ(dists.size(), dists_nanoflann.size());
    ASSERT_EQ(indices.size(), dists.size());
    ASSERT_EQ(indices_nanoflann.size(), dists_nanoflann.size());
    for (std::size_t i = 0;
         i < std::min<std::size_t>(indices.size(), indices_nanoflann.size());
         ++i) {
      // sometimes, two neighbors have almost the same distance to the query point, but
      // are switched to due limited floating point accuracy.
      if ((i + 1) < indices.size() && indices[i] != indices_nanoflann[i] &&
          indices[i + 1] != indices_nanoflann[i + 1] &&
          indices[i] == indices_nanoflann[i + 1] &&
          indices[i + 1] == indices_nanoflann[i] &&
          std::abs(dists[i] - dists_nanoflann[i + 1]) < 1e-4 &&
          std::abs(dists[i + 1] - dists_nanoflann[i]) < 1e-4) {
        std::swap(indices[i], indices[i + 1]);
        std::swap(dists[i], dists[i + 1]);
      }
      EXPECT_EQ(indices[i], indices_nanoflann[i])
          << "i=" << i << ", dists[i]=" << dists[i]
          << ", dists_nanoflann[i]=" << dists_nanoflann[i] << std::endl;
      ;
      EXPECT_NEAR(dists[i], dists_nanoflann[i], 1e-4);
    }
  }
}

template <typename PointT>
double
manhattanDistance(const PointT& a, const PointT& b)
{
  return std::abs(static_cast<double>(a.x) - static_cast<double>(b.x)) +
         std::abs(static_cast<double>(a.y) - static_cast<double>(b.y)) +
         std::abs(static_cast<double>(a.z) - static_cast<double>(b.z));
}

// test with L1/manhattan distance instead of L2/euclidean distance
TEST(PCL, KdTreeNanoflann_nearestKSearch_L1)
{
  pcl::search::KdTreeNanoflann<PointXYZ, 3, pcl::search::L1_Adaptor> kdtree_nanoflann;
  kdtree_nanoflann.setInputCloud(bun0_cloud.makeShared());

  kdtree_nanoflann.setSortedResults(true);
  PointXYZ test_point = bun0_cloud[rand() % bun0_cloud.size()];
  const unsigned int no_of_neighbors = 20;
  std::multimap<double, int> sorted_brute_force_result;
  for (std::size_t i = 0; i < bun0_cloud.size(); ++i) {
    auto distance = manhattanDistance(bun0_cloud[i], test_point);
    sorted_brute_force_result.emplace(distance, i);
  }

  pcl::Indices k_indices;
  std::vector<float> k_distances;

  EXPECT_EQ(no_of_neighbors,
            kdtree_nanoflann.nearestKSearch(
                test_point, no_of_neighbors, k_indices, k_distances));
  EXPECT_EQ(k_indices.size(), no_of_neighbors);
  EXPECT_EQ(k_distances.size(), no_of_neighbors);
  unsigned int counter = 0;
  for (auto it = sorted_brute_force_result.begin();
       it != sorted_brute_force_result.end() && counter < no_of_neighbors;
       ++it, ++counter) {
    EXPECT_EQ(k_indices[counter], it->second);
    EXPECT_NEAR(k_distances[counter], it->first, 1e-6);
  }

  // test radius search
  const float search_radius = 10 * std::next(sorted_brute_force_result.begin())->first;
  kdtree_nanoflann.radiusSearch(test_point, search_radius, k_indices, k_distances);
  EXPECT_EQ(k_indices.size(), k_distances.size());
  counter = 0;
  for (auto it = sorted_brute_force_result.begin();
       it != sorted_brute_force_result.end() && it->first <= search_radius;
       ++it, ++counter) {
    // sometimes, two neighbors have almost the same distance to the query point, but
    // are switched to due limited floating point accuracy.
    if ((counter + 1) < k_indices.size() && k_indices[counter] != it->second &&
        k_indices[counter + 1] != std::next(it)->second &&
        k_indices[counter] == std::next(it)->second &&
        k_indices[counter + 1] == it->second &&
        std::abs(k_distances[counter] - std::next(it)->first) < 1e-6 &&
        std::abs(k_distances[counter + 1] - it->first) < 1e-6) {
      std::swap(k_indices[counter], k_indices[counter + 1]);
      std::swap(k_distances[counter], k_distances[counter + 1]);
    }
    EXPECT_EQ(k_indices[counter], it->second);
    EXPECT_NEAR(k_distances[counter], it->first, 1e-6);
  }
  EXPECT_EQ(k_indices.size(), counter);
  // test unsorted radius search
  kdtree_nanoflann.setSortedResults(false);
  k_indices.clear();
  k_distances.clear();
  kdtree_nanoflann.radiusSearch(test_point, search_radius, k_indices, k_distances);
  EXPECT_EQ(k_indices.size(), k_distances.size());
  for (auto it = sorted_brute_force_result.begin();
       it != sorted_brute_force_result.end() && it->first <= search_radius;
       ++it) {
    auto pos = std::find(k_indices.begin(), k_indices.end(), it->second);
    ASSERT_NE(pos, k_indices.end()); // must be found
    EXPECT_NEAR(it->first, k_distances[std::distance(k_indices.begin(), pos)], 1e-6);
  }
}

struct MyPoint : public PointXYZ {
  MyPoint(float x, float y, float z)
  {
    this->x = x;
    this->y = y;
    this->z = z;
  }
};

class MyPointRepresentationXY : public PointRepresentation<MyPoint> {
public:
  MyPointRepresentationXY() { this->nr_dimensions_ = 2; }

  void
  copyToFloatArray(const MyPoint& p, float* out) const override
  {
    out[0] = p.x;
    out[1] = p.y;
  }
};

TEST(PCL, KdTreeNanoflann_setPointRepresentation)
{
  PointCloud<MyPoint>::Ptr random_cloud(new PointCloud<MyPoint>());
  random_cloud->points.emplace_back(86.6f, 42.1f, 92.4f);
  random_cloud->points.emplace_back(63.1f, 18.4f, 22.3f);
  random_cloud->points.emplace_back(35.5f, 72.5f, 37.3f);
  random_cloud->points.emplace_back(99.7f, 37.0f, 8.7f);
  random_cloud->points.emplace_back(22.4f, 84.1f, 64.0f);
  random_cloud->points.emplace_back(65.2f, 73.4f, 18.0f);
  random_cloud->points.emplace_back(60.4f, 57.1f, 4.5f);
  random_cloud->points.emplace_back(38.7f, 17.6f, 72.3f);
  random_cloud->points.emplace_back(14.2f, 95.7f, 34.7f);
  random_cloud->points.emplace_back(2.5f, 26.5f, 66.0f);

  auto kdtree = pcl::make_shared<pcl::search::KdTreeNanoflann<MyPoint, -1>>();
  kdtree->setInputCloud(random_cloud);
  EXPECT_EQ(3, kdtree->getPointRepresentation()->getNumberOfDimensions());
  MyPoint p(50.0f, 50.0f, 50.0f);

  // Find k nearest neighbors
  constexpr int k = 10;
  pcl::Indices k_indices(k);
  std::vector<float> k_distances(k);
  kdtree->nearestKSearch(p, k, k_indices, k_distances);
  for (int i = 0; i < k; ++i) {
    // Compare to ground truth values, computed independently
    static const int gt_indices[10] = {2, 7, 5, 1, 4, 6, 9, 0, 8, 3};
    static const float gt_distances[10] = {877.8f,
                                           1674.7f,
                                           1802.6f,
                                           1937.5f,
                                           2120.6f,
                                           2228.8f,
                                           3064.5f,
                                           3199.7f,
                                           3604.2f,
                                           4344.8f};
    EXPECT_EQ(k_indices[i], gt_indices[i]);
    EXPECT_NEAR(k_distances[i], gt_distances[i], 0.1);
  }

  // Find k nearest neighbors with a different point representation
  pcl::search::KdTreeNanoflann<MyPoint, -1>::PointRepresentationConstPtr ptrep(
      new MyPointRepresentationXY);
  kdtree->setPointRepresentation(ptrep);
  EXPECT_EQ(2, kdtree->getPointRepresentation()->getNumberOfDimensions());
  kdtree->nearestKSearch(p, k, k_indices, k_distances);
  for (int i = 0; i < k; ++i) {
    // Compare to ground truth values, computed independently
    static const int gt_indices[10] = {6, 2, 5, 1, 7, 0, 4, 3, 9, 8};
    static const float gt_distances[10] = {158.6f,
                                           716.5f,
                                           778.6f,
                                           1170.2f,
                                           1177.5f,
                                           1402.0f,
                                           1924.6f,
                                           2639.1f,
                                           2808.5f,
                                           3370.1f};
    EXPECT_EQ(k_indices[i], gt_indices[i]);
    EXPECT_NEAR(k_distances[i], gt_distances[i], 0.1);
  }

  // Go back to the default, this time with the values rescaled
  DefaultPointRepresentation<MyPoint> point_rep;
  float alpha[3] = {1.0f, 2.0f, 3.0f};
  point_rep.setRescaleValues(alpha);
  kdtree->setPointRepresentation(point_rep.makeShared());
  EXPECT_EQ(3, kdtree->getPointRepresentation()->getNumberOfDimensions());
  kdtree->nearestKSearch(p, k, k_indices, k_distances);
  for (int i = 0; i < k; ++i) {
    // Compare to ground truth values, computed independently
    static const int gt_indices[10] = {2, 9, 4, 7, 1, 5, 8, 0, 3, 6};
    static const float gt_distances[10] = {3686.9f,
                                           6769.2f,
                                           7177.0f,
                                           8802.3f,
                                           11071.5f,
                                           11637.3f,
                                           11742.4f,
                                           17769.0f,
                                           18497.3f,
                                           18942.0f};
    EXPECT_EQ(k_indices[i], gt_indices[i]);
    EXPECT_NEAR(k_distances[i], gt_distances[i], 0.1);
  }
}

// test with nanoflann::KDTreeSingleIndexDynamicAdaptor (allows to dynamically
// add/remove points)
TEST(PCL, KdTreeNanoflann_dynamic)
{
  using Distance = pcl::search::L2_Simple_Adaptor;
  constexpr std::int32_t Dim = 3;
  pcl::search::KdTreeNanoflann<PointXYZ,
                               Dim,
                               Distance,
                               nanoflann::KDTreeSingleIndexDynamicAdaptor<
                                   Distance,
                                   pcl::search::internal::PointCloudAdaptor<float>,
                                   Dim,
                                   pcl::index_t>>
      kdtree_nanoflann;
  kdtree_nanoflann.setSortedResults(true);
  auto input_cloud = bun0_cloud.makeShared();
  input_cloud->reserve(input_cloud->size() + 1); // reserve space for one more point
  kdtree_nanoflann.setInputCloud(input_cloud);

  pcl::PointXYZ test_point = (*input_cloud)[rand() % input_cloud->size()];
  const unsigned int no_of_neighbors = 20;
  std::multimap<double, int> sorted_brute_force_result;
  for (std::size_t i = 0; i < input_cloud->size(); ++i) {
    auto distance = squaredEuclideanDistance((*input_cloud)[i], test_point);
    sorted_brute_force_result.emplace(distance, i);
  }

  pcl::Indices k_indices;
  std::vector<float> k_distances;

  EXPECT_EQ(no_of_neighbors,
            kdtree_nanoflann.nearestKSearch(
                test_point, no_of_neighbors, k_indices, k_distances));
  EXPECT_EQ(k_indices.size(), no_of_neighbors);
  EXPECT_EQ(k_distances.size(), no_of_neighbors);
  unsigned int counter = 0;
  for (auto it = sorted_brute_force_result.begin();
       it != sorted_brute_force_result.end() && counter < no_of_neighbors;
       ++it, ++counter) {
    EXPECT_EQ(k_indices[counter], it->second);
    EXPECT_NEAR(k_distances[counter], it->first, 1e-6);
  }

  kdtree_nanoflann.getNanoflannTree()->removePoint(k_indices[0]);
  input_cloud->push_back(test_point);
  kdtree_nanoflann.getNanoflannTree()->addPoints(input_cloud->size() - 1,
                                                 input_cloud->size() - 1);
  sorted_brute_force_result.erase(sorted_brute_force_result.begin());
  sorted_brute_force_result.emplace(0.0, input_cloud->size() - 1);

  EXPECT_EQ(no_of_neighbors,
            kdtree_nanoflann.nearestKSearch(
                test_point, no_of_neighbors, k_indices, k_distances));
  EXPECT_EQ(k_indices.size(), no_of_neighbors);
  EXPECT_EQ(k_distances.size(), no_of_neighbors);
  counter = 0;
  for (auto it = sorted_brute_force_result.begin();
       it != sorted_brute_force_result.end() && counter < no_of_neighbors;
       ++it, ++counter) {
    EXPECT_EQ(k_indices[counter], it->second);
    EXPECT_NEAR(k_distances[counter], it->first, 1e-6);
  }
}

int
main(int argc, char** argv)
{
  const auto seed = time(nullptr);
  std::cout << "seed=" << seed << std::endl;
  srand(seed);
  testing::InitGoogleTest(&argc, argv);
  init();

  if (argc < 2 || pcl::io::loadPCDFile(argv[1], bun0_cloud) < 0) {
    std::cerr << "Failed to read test file. Please download `bun0.pcd` and pass its "
                 "path to the test."
              << std::endl;
    return (-1);
  }

  return (RUN_ALL_TESTS());
}
