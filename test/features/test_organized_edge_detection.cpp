/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2020-, Open Perception
 *
 *  All rights reserved
 */

#include <pcl/features/organized_edge_detection.h>
#include <pcl/test/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace {
class OrganizedPlaneDetectionTestFixture : public ::testing::Test {
protected:
  const int INNER_SQUARE_EDGE_LENGTH = 50;
  const int OUTER_SQUARE_EDGE_LENGTH = INNER_SQUARE_EDGE_LENGTH * 2;
  const float SYNTHETIC_CLOUD_BASE_DEPTH = 2.0;
  const float SYNTHETIC_CLOUD_DEPTH_DISCONTINUITY = .02f;
  const float SYNTHETIC_CLOUD_RESOLUTION = 0.01f;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
  pcl::PointIndices small_outer_perimeter_;
  pcl::PointIndices large_inner_perimeter_;

  void
  SetUp() override
  {
    cloud_ = generateSyntheticEdgeDetectionCloud();
  }

private:
  pcl::PointCloud<pcl::PointXYZ>::Ptr
  generateSyntheticEdgeDetectionCloud()
  {
    auto organized_test_cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>(
        OUTER_SQUARE_EDGE_LENGTH, OUTER_SQUARE_EDGE_LENGTH);

    auto occluded = std::set<int>{};

    // Draw a smaller square in front of a larger square both centered on the
    // view axis to generate synthetic occluding and occluded edges based on depth
    // discontinuity between neighboring pixels.  The base depth and resolution are
    // arbitrary and useful for visualizing the cloud.  The discontinuity of the
    // generated cloud must be greater than the threshold set when running the
    // organized edge detection algorithm.
    const auto outer_square_ctr = OUTER_SQUARE_EDGE_LENGTH / 2;
    const auto inner_square_ctr = INNER_SQUARE_EDGE_LENGTH / 2;
    const auto left_col = outer_square_ctr - inner_square_ctr;
    const auto right_col = outer_square_ctr + inner_square_ctr;
    const auto top_row = outer_square_ctr - inner_square_ctr;
    const auto bottom_row = outer_square_ctr + inner_square_ctr;

    for (auto row = 0; row < OUTER_SQUARE_EDGE_LENGTH; ++row) {
      for (auto col = 0; col < OUTER_SQUARE_EDGE_LENGTH; ++col) {
        float x = col - outer_square_ctr;
        float y = row - inner_square_ctr;

        auto depth = SYNTHETIC_CLOUD_BASE_DEPTH;

        // If pixels correspond to smaller box, then set depth and color appropriately
        if (col >= left_col && col < right_col) {
          if (row >= top_row && row < bottom_row) {

            depth = SYNTHETIC_CLOUD_BASE_DEPTH - SYNTHETIC_CLOUD_DEPTH_DISCONTINUITY;

            // Record outer perimeter points of small inner square that correspond to
            // the occluding edge points
            if ((col == outer_square_ctr - inner_square_ctr ||
                 col == outer_square_ctr + inner_square_ctr - 1) ||
                (row == outer_square_ctr - inner_square_ctr ||
                 row == outer_square_ctr + inner_square_ctr - 1)) {
              small_outer_perimeter_.indices.push_back(
                  row * organized_test_cloud->width + col);
            }
          }
        }

        // Record inner perimeter points of large outer square that correspond to the
        // occluded edge points
        if (row == top_row - 1 || row == bottom_row) {
          if (col >= left_col - 1 && col <= right_col) {
            large_inner_perimeter_.indices.push_back(row * organized_test_cloud->width +
                                                     col);
          }
        }
        else if (row >= top_row && row < bottom_row) {
          if (col == left_col - 1 || col == right_col) {
            large_inner_perimeter_.indices.push_back(row * organized_test_cloud->width +
                                                     col);
          }
        }

        organized_test_cloud->at(col, row).x = x * SYNTHETIC_CLOUD_RESOLUTION;
        organized_test_cloud->at(col, row).y = y * SYNTHETIC_CLOUD_RESOLUTION;
        organized_test_cloud->at(col, row).z = depth;
      }
    }

    return organized_test_cloud;
  }
};
} // namespace

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
This test is designed to ensure that the organized edge detection appropriately
classifies occluding and occluding edges and to specifically detect the type of
regression detailed in PR 4275 (https://github.com/PointCloudLibrary/pcl/pull/4275).
This test works by generating a synthetic cloud of one square slightly in front of
another square, so that occluding edges and occluded edges are generated.  The
regression introduced in PCL 1.10.1 was a logic bug that caused both occluding and
occluded edges to be miscategorized as occluding edges.  This test should catch
this and similar bugs.
*/
TEST_F(OrganizedPlaneDetectionTestFixture, OccludedAndOccludingEdges)
{
  const auto MAX_SEARCH_NEIGHBORS = 8;

  // The depth discontinuity check to determine whether an edge exists is linearly
  // dependent on the depth of the points in the cloud (not a fixed distance).  The
  // algorithm iterates through each point in the cloud and finding the neighboring
  // point with largest discontinuity value.  That value is compared against a threshold
  // multiplied by the actual depth value of the point. Therefore:
  // abs(SYNTHETIC_CLOUD_DEPTH_DISCONTINUITY) must be greater than
  // DEPTH_DISCONTINUITY_THRESHOLD * abs(SYNTHETIC_CLOUD_BASE_DEPTH)
  const auto DEPTH_DISCONTINUITY_THRESHOLD =
      SYNTHETIC_CLOUD_DEPTH_DISCONTINUITY / (SYNTHETIC_CLOUD_BASE_DEPTH * 1.1f);

  auto oed = pcl::OrganizedEdgeBase<pcl::PointXYZ, pcl::Label>();
  auto labels = pcl::PointCloud<pcl::Label>();
  auto label_indices = std::vector<pcl::PointIndices>();

  oed.setEdgeType(oed.EDGELABEL_OCCLUDING | oed.EDGELABEL_OCCLUDED);
  oed.setInputCloud(cloud_);
  oed.setDepthDisconThreshold(DEPTH_DISCONTINUITY_THRESHOLD);
  oed.setMaxSearchNeighbors(MAX_SEARCH_NEIGHBORS);
  oed.compute(labels, label_indices);

  EXPECT_EQ(label_indices[1].indices.size(), small_outer_perimeter_.indices.size());
  for (auto i = 0; i < small_outer_perimeter_.indices.size(); ++i) {
    EXPECT_EQ(label_indices[1].indices[i], small_outer_perimeter_.indices[i]);
  }

  EXPECT_EQ(label_indices[2].indices.size(), large_inner_perimeter_.indices.size());
  for (auto i = 0; i < large_inner_perimeter_.indices.size(); ++i) {
    EXPECT_EQ(label_indices[2].indices[i], large_inner_perimeter_.indices[i]);
  }
}

/* ---[ */
int
main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return (RUN_ALL_TESTS());
}
/* ]--- */
