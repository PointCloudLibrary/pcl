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
     static const int kExpectedOccludingEdgePoints = 395;
     static const int kExpectedOccludedEdgePoints = 401;
     static const int kSyntheticCloudDisparity = .03;
     static const int kSyntheticCloudResolution = .005;
     static const int kSyntheticCloudInnerBoxWidth = 50;

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
	auto organized_test_cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();
	organized_test_cloud->width = kSyntheticCloudInnerBoxWidth * 4;
	organized_test_cloud->height = kSyntheticCloudInnerBoxWidth * 4;
	organized_test_cloud->is_dense = false;
	organized_test_cloud->points.resize(organized_test_cloud->height * organized_test_cloud->width);

	for(std::size_t i = 0; i < kSyntheticCloudInnerBoxWidth * 2; i++)
	{
		for(std::size_t j = 0; j < kSyntheticCloudInnerBoxWidth * 2; j++)
		{
			organized_test_cloud->at(j, i).x = i * kSyntheticCloudResolution - (kSyntheticCloudInnerBoxWidth / 2 * kSyntheticCloudResolution);
			organized_test_cloud->at(j, i).y = j * kSyntheticCloudResolution;
			organized_test_cloud->at(j, i).z = 2.0 + (disparity * 4);
			organized_test_cloud->at(j, i).r = 128;
			organized_test_cloud->at(j, i).g = 128;
			organized_test_cloud->at(j, i).b = 128;
			organized_test_cloud->at(j, i).a = 255;
		}
	}

	for(std::size_t i = kSyntheticCloudInnerBoxWidth / 2; i < kSyntheticCloudInnerBoxWidth / 2 + kSyntheticCloudInnerBoxWidth; i++)
	{
		for(std::size_t j = kSyntheticCloudInnerBoxWidth / 2; j < kSyntheticCloudInnerBoxWidth / 2 + kSyntheticCloudInnerBoxWidth; j++)
		{
			organized_test_cloud->at(j, i).x = i * kSyntheticCloudResolution - (kSyntheticCloudInnerBoxWidth / 2 * kSyntheticCloudResolution);
			organized_test_cloud->at(j, i).y = j * kSyntheticCloudResolution;
			organized_test_cloud->at(j, i).z = 2.0;
			organized_test_cloud->at(j, i).r = 192;
			organized_test_cloud->at(j, i).g = 192;
			organized_test_cloud->at(j, i).b = 192;
			organized_test_cloud->at(j, i).a = 255;
		}
	}
	return organized_test_cloud;
     }


  };
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST_F (OrganizedPlaneDetectionTestFixture, OccludedAndOccludingEdges)
{
  auto MaxSearchNeighbors = 50;
  auto DepthDiscontinuity = .02f;

  auto oed = pcl::OrganizedEdgeFromRGB<pcl::PointXYZRGBA, pcl::Label>();

  oed.setEdgeType(oed.EDGELABEL_OCCLUDING | oed.EDGELABEL_OCCLUDED);
  oed.setInputCloud(cloud_);
  oed.setDepthDisconThreshold(DepthDiscontinuity);
  oed.setMaxSearchNeighbors(MaxSearchNeighbors);

  auto labels = pcl::PointCloud<pcl::Label>();
  auto label_indices = std::vector<pcl::PointIndices>();

  oed.compute(labels, label_indices);

  EXPECT_EQ (395, label_indices[1].indices.size());  // Occluding Edges Indices Size should be 395 for synthetic cloud
  EXPECT_EQ (401, label_indices[2].indices.size());  // Occluded Edge Indices should be 401 for synthentic cloud
}


/* ---[ */
int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
