/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2020-, Open Perception
 *
 *  All rights reserved
 */

#include <pcl/test/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/features/organized_edge_detection.h>
#include <pcl/io/pcd_io.h>

namespace
{
  class OrganizedPlaneDetectionTestFixture : public ::testing::Test
  {
   protected:
     const int kSyntheticCloudInnerSquareEdgeLength = 124;
     const int kSyntheticCloudOuterSquareEdgeLength = kSyntheticCloudInnerSquareEdgeLength * 2;

     const int kExpectedOccludingEdgePoints = kSyntheticCloudInnerSquareEdgeLength * 4 - 8;  // Empirically determined
     const int kExpectedOccludedEdgePoints = kSyntheticCloudInnerSquareEdgeLength * 4;       // Each edge pixed of inner square should generate an occluded point

     const float kSyntheticCloudDisparity = .03f;
     const float kSyntheticCloudResolution = 0.01f;

     pcl::PointCloud <pcl::PointXYZRGBA>::Ptr cloud_;
     pcl::PointIndicesPtr indices_;

     OrganizedPlaneDetectionTestFixture() = default;
     virtual ~OrganizedPlaneDetectionTestFixture() = default;
     virtual void SetUp()
     {
       cloud_ = GenerateSyntheticEdgeDetectionCloud(kSyntheticCloudDisparity);
     }
     virtual void TearDown()
     {
       cloud_.reset();
     }

   private:
     pcl::PointCloud<pcl::PointXYZRGBA>::Ptr GenerateSyntheticEdgeDetectionCloud(const float &disparity)
     {
	auto organized_test_cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>(kSyntheticCloudOuterSquareEdgeLength, kSyntheticCloudOuterSquareEdgeLength);
        organized_test_cloud->is_dense = true;

        const auto kBaseDepth = 2.0f;

        // Draw a smaller red square in front of a larger green square both centered on the view axis to generate synthetic occluding and occluded edges based on depth disparity between neighboring pixels
	for(auto row = 0; row < kSyntheticCloudOuterSquareEdgeLength; row++)
	{
		for(auto col = 0; col < kSyntheticCloudOuterSquareEdgeLength; col++)
                {
                        float x = col - (kSyntheticCloudOuterSquareEdgeLength / 2);
                        float y = row - (kSyntheticCloudOuterSquareEdgeLength / 2);

                        const auto outer_square_ctr = kSyntheticCloudOuterSquareEdgeLength / 2;
                        const auto inner_square_ctr = kSyntheticCloudInnerSquareEdgeLength / 2;

                        auto depth = kBaseDepth + (disparity * 2);
                        auto r = 0;
                        auto g = 255;

                        // If pixels correspond to smaller box, then set depth and color appropriately
                        if (col > outer_square_ctr - inner_square_ctr && col < outer_square_ctr + inner_square_ctr)
                        {
                          if (row > outer_square_ctr - inner_square_ctr && row < outer_square_ctr + inner_square_ctr)
                          {
                            depth = kBaseDepth;
                            r = 255;
                            g = 0;
                          }
                        }

			organized_test_cloud->at(col, row).x = x * kSyntheticCloudResolution;
			organized_test_cloud->at(col, row).y = y * kSyntheticCloudResolution;
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
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
This test is designed to ensure that the organized edge detection appropriately classifies occluding and occluding 
edges and to speifically detect the type of regression detailed in PR 4275
(https://github.com/PointCloudLibrary/pcl/pull/4275).  This test works by generating a synthentic cloud of one 
square slightly in front of another square, so that occluding edges and occluded edges are generated.  The 
regression introduced in PCL 1.10.1 was a logic bug that caused both occluding and occluded edges to erroneously 
be categorized as occluding edges.  This test should catch this and similar bugs.
*/
TEST_F (OrganizedPlaneDetectionTestFixture, OccludedAndOccludingEdges)
{
  const auto kMaxSearchNeighbors = 50;
  const auto kDepthDiscontinuity = .02f;

  auto oed = pcl::OrganizedEdgeFromRGB<pcl::PointXYZRGBA, pcl::Label>();

  oed.setEdgeType(oed.EDGELABEL_OCCLUDING | oed.EDGELABEL_OCCLUDED);
  oed.setInputCloud(cloud_);
  oed.setDepthDisconThreshold(kDepthDiscontinuity);
  oed.setMaxSearchNeighbors(kMaxSearchNeighbors);

  auto labels = pcl::PointCloud<pcl::Label>();
  auto label_indices = std::vector<pcl::PointIndices>();

  oed.compute(labels, label_indices);

  EXPECT_EQ (kExpectedOccludingEdgePoints, label_indices[1].indices.size());  // Occluding Edges Indices Size should be 395 for synthetic cloud
  EXPECT_EQ (kExpectedOccludedEdgePoints, label_indices[2].indices.size());  // Occluded Edge Indices should be 401 for synthentic cloud
}


/* ---[ */
int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
