/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2020-, Open Perception
 *
 *  All rights reserved
 */

#include <pcl/features/organized_edge_detection.h>
#include <pcl/point_cloud.h>
#include <pcl/test/gtest.h>
#include <pcl/io/pcd_io.h>

namespace {
class OrganizedPlaneDetectionTestFixture : public ::testing::Test {
protected:
  const int INNER_SQUARE_EDGE_LENGTH = 50;
  const int OUTER_SQUARE_EDGE_LENGTH = INNER_SQUARE_EDGE_LENGTH * 2;
  const float SYNTHETIC_CLOUD_BASE_DEPTH = 2.0;
  const float SYNTHETIC_CLOUD_DEPTH_DISCONTINUITY = .02f;
  const float SYNTHETIC_CLOUD_RESOLUTION = 0.01f;

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_;
  pcl::PointIndicesPtr indices_;

  void
  SetUp() override
  {
    cloud_ = generateSyntheticEdgeDetectionCloud();
  }
  void
  TearDown() override
  {}

private:
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr
  generateSyntheticEdgeDetectionCloud()
  {
    auto organized_test_cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>(
        OUTER_SQUARE_EDGE_LENGTH, OUTER_SQUARE_EDGE_LENGTH);

    // Draw a smaller red square in front of a larger green square both centered on the
    // view axis to generate synthetic occluding and occluded edges based on depth
    // discontinuity between neighboring pixels.  The base depth and resolution are
    // arbitrary and useful for visualizing the cloud.  The discontinuity of the
    // generated cloud must be greater than the threshold set when running the
    // organized edge detection algorithm.
    for (auto row = 0; row < OUTER_SQUARE_EDGE_LENGTH; ++row) {
      for (auto col = 0; col < OUTER_SQUARE_EDGE_LENGTH; ++col) {
        float x = col - (OUTER_SQUARE_EDGE_LENGTH / 2);
        float y = row - (OUTER_SQUARE_EDGE_LENGTH / 2);

        const auto outer_square_ctr = OUTER_SQUARE_EDGE_LENGTH / 2;
        const auto inner_square_ctr = INNER_SQUARE_EDGE_LENGTH / 2;

        auto depth = SYNTHETIC_CLOUD_BASE_DEPTH;
        auto r = 0;
        auto g = 255;

        // If pixels correspond to smaller box, then set depth and color appropriately
        if (col > outer_square_ctr - inner_square_ctr &&
            col < outer_square_ctr + inner_square_ctr) {
          if (row > outer_square_ctr - inner_square_ctr &&
              row < outer_square_ctr + inner_square_ctr) {
            depth = SYNTHETIC_CLOUD_BASE_DEPTH - SYNTHETIC_CLOUD_DEPTH_DISCONTINUITY;
            r = 255;
            g = 0;
          }
        }

        organized_test_cloud->at(col, row).x = x * SYNTHETIC_CLOUD_RESOLUTION;
        organized_test_cloud->at(col, row).y = y * SYNTHETIC_CLOUD_RESOLUTION;
        organized_test_cloud->at(col, row).z = depth;
        organized_test_cloud->at(col, row).r = r;
        organized_test_cloud->at(col, row).g = g;
        organized_test_cloud->at(col, row).b = 0;
        organized_test_cloud->at(col, row).a = 255;
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
  const auto MAX_SEARCH_NEIGHBORS = 50;
  const auto DEPTH_DISCONTINUITY_THRESHOLD = SYNTHETIC_CLOUD_DEPTH_DISCONTINUITY / 2.1f;

  // The expected number of occluded edge points is the number of pixels along the
  // perimeter of the inner square, subject to certain conditions including that the
  // inner square edge length is an even number.  The expected number of occluding edge
  // points is similar, but as empirically determined to be 8 less than the number
  // of perimeter points.
  const int EXPECTED_OCCLUDING_EDGE_POINTS = INNER_SQUARE_EDGE_LENGTH * 4 - 8;
  const int EXPECTED_OCCLUDED_EDGE_POINTS = INNER_SQUARE_EDGE_LENGTH * 4;

  auto oed = pcl::OrganizedEdgeFromRGB<pcl::PointXYZRGBA, pcl::Label>();
  auto labels = pcl::PointCloud<pcl::Label>();
  auto label_indices = std::vector<pcl::PointIndices>();

  oed.setEdgeType(oed.EDGELABEL_OCCLUDING | oed.EDGELABEL_OCCLUDED);
  oed.setInputCloud(cloud_);
  oed.setDepthDisconThreshold(DEPTH_DISCONTINUITY_THRESHOLD);
  oed.setMaxSearchNeighbors(MAX_SEARCH_NEIGHBORS);
  oed.compute(labels, label_indices);

  EXPECT_EQ(EXPECTED_OCCLUDING_EDGE_POINTS, label_indices[1].indices.size());
  EXPECT_EQ(EXPECTED_OCCLUDED_EDGE_POINTS, label_indices[2].indices.size());
}

/* ---[ */
int
main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return (RUN_ALL_TESTS());
}
/* ]--- */
