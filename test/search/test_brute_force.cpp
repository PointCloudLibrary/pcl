/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2026-, Open Perception Inc.
 *
 *  All rights reserved
 */

#define PCL_NO_PRECOMPILE 1
#include <pcl/point_representation.h>
#include <pcl/point_types.h>
#include <pcl/search/brute_force.h>
#include <pcl/test/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace
{
class PointXYRepresentation : public pcl::PointRepresentation<pcl::PointXYZ>
{
public:
  PointXYRepresentation ()
  {
    this->nr_dimensions_ = 2;
  }

  void
  copyToFloatArray (const pcl::PointXYZ& point, float* out) const override
  {
    out[0] = point.x;
    out[1] = point.y;
  }
};

float
descriptorDistance (const pcl::FPFHSignature33& lhs, const pcl::FPFHSignature33& rhs)
{
  float distance = 0.0f;
  for (std::size_t index = 0; index < 33; ++index)
  {
    const float diff = lhs.histogram[index] - rhs.histogram[index];
    distance += diff * diff;
  }
  return (distance);
}
} // namespace

TEST (PCL, BruteForce_setPointRepresentation)
{
  auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>> ();
  cloud->emplace_back (0.0f, 100.0f, 0.0f);
  cloud->emplace_back (1.0f, 0.0f, 100.0f);
  cloud->emplace_back (2.0f, 0.0f, 0.0f);

  pcl::search::BruteForce<pcl::PointXYZ> search (true);
  search.setInputCloud (cloud);
  search.setPointRepresentation (
      pcl::make_shared<PointXYRepresentation> ());

  pcl::Indices indices;
  std::vector<float> distances;
  EXPECT_EQ (3, search.nearestKSearch (pcl::PointXYZ (0.0f, 0.0f, 0.0f),
                                       3,
                                       indices,
                                       distances));

  ASSERT_EQ (3, indices.size ());
  ASSERT_EQ (3, distances.size ());
  EXPECT_EQ (1, indices[0]);
  EXPECT_EQ (2, indices[1]);
  EXPECT_EQ (0, indices[2]);
  EXPECT_FLOAT_EQ (1.0f, distances[0]);
  EXPECT_FLOAT_EQ (4.0f, distances[1]);
  EXPECT_FLOAT_EQ (10000.0f, distances[2]);
}

TEST (PCL, BruteForce_featurePointRepresentation)
{
  using FeatureT = pcl::FPFHSignature33;

  auto cloud = pcl::make_shared<pcl::PointCloud<FeatureT>> ();
  cloud->resize (4);
  for (std::size_t point_index = 0; point_index < cloud->size (); ++point_index)
  {
    for (std::size_t dimension = 0; dimension < 33; ++dimension)
      (*cloud)[point_index].histogram[dimension] =
          static_cast<float> (point_index * 10 + dimension);
  }
  (*cloud)[3].histogram[32] = 500.0f;

  FeatureT query{};
  for (std::size_t dimension = 0; dimension < 33; ++dimension)
    query.histogram[dimension] = static_cast<float> (10 + dimension);

  pcl::search::BruteForce<FeatureT> search (true);
  EXPECT_EQ (33, search.getPointRepresentation ()->getNumberOfDimensions ());
  search.setPointRepresentation (
      pcl::make_shared<pcl::DefaultFeatureRepresentation<FeatureT>> ());
  search.setInputCloud (cloud);

  pcl::Indices indices;
  std::vector<float> distances;
  EXPECT_EQ (3, search.nearestKSearch (query, 3, indices, distances));

  std::vector<std::pair<float, int>> expected;
  for (std::size_t index = 0; index < cloud->size (); ++index)
    expected.emplace_back (descriptorDistance ((*cloud)[index], query),
                           static_cast<int> (index));
  std::sort (expected.begin (), expected.end ());

  ASSERT_EQ (3, indices.size ());
  ASSERT_EQ (3, distances.size ());
  for (std::size_t i = 0; i < indices.size (); ++i)
  {
    EXPECT_EQ (expected[i].second, indices[i]);
    EXPECT_FLOAT_EQ (expected[i].first, distances[i]);
  }
}

TEST (PCL, BruteForce_sparseFeatureCloudSkipsInvalidDescriptors)
{
  using FeatureT = pcl::FPFHSignature33;

  auto cloud = pcl::make_shared<pcl::PointCloud<FeatureT>> ();
  cloud->is_dense = false;
  cloud->resize (3);
  for (std::size_t point_index = 0; point_index < cloud->size (); ++point_index)
  {
    for (std::size_t dimension = 0; dimension < 33; ++dimension)
      (*cloud)[point_index].histogram[dimension] =
          static_cast<float> (point_index + dimension);
  }
  (*cloud)[1].histogram[0] = std::numeric_limits<float>::quiet_NaN ();

  pcl::search::BruteForce<FeatureT> search (true);
  search.setPointRepresentation (
      pcl::make_shared<pcl::DefaultFeatureRepresentation<FeatureT>> ());
  search.setInputCloud (cloud);

  pcl::Indices indices;
  std::vector<float> distances;
  EXPECT_EQ (2, search.nearestKSearch ((*cloud)[0], 3, indices, distances));

  ASSERT_EQ (2, indices.size ());
  EXPECT_NE (1, indices[0]);
  EXPECT_NE (1, indices[1]);
}

int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
