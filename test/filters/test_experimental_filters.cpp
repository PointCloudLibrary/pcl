/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2021-, Open Perception
 *
 *  All rights reserved
 */

#include <pcl/common/eigen.h>
#include <pcl/filters/experimental/crop_box.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/test/gtest.h>
#include <pcl/pcl_base.h> // for pcl::Indices
#include <pcl/pcl_tests.h>
#include <pcl/point_types.h>

using namespace pcl;
using namespace Eigen;

TEST(ExperimentalCropBox, Filters)
{
  // Create cloud with center point and corner points
  PointCloud<PointXYZ>::Ptr input(new PointCloud<PointXYZ>());

  input->push_back(PointXYZ(0.0f, 0.0f, 0.0f));
  input->push_back(PointXYZ(0.9f, 0.9f, 0.9f));
  input->push_back(PointXYZ(0.9f, 0.9f, -0.9f));
  input->push_back(PointXYZ(0.9f, -0.9f, 0.9f));
  input->push_back(PointXYZ(-0.9f, 0.9f, 0.9f));
  input->push_back(PointXYZ(0.9f, -0.9f, -0.9f));
  input->push_back(PointXYZ(-0.9f, -0.9f, 0.9f));
  input->push_back(PointXYZ(-0.9f, 0.9f, -0.9f));
  input->push_back(PointXYZ(-0.9f, -0.9f, -0.9f));

  // Create indices vector ( without 0 and 4)
  pcl::IndicesPtr idx(new pcl::Indices(7));
  (*idx)[0] = 1;
  (*idx)[1] = 2;
  (*idx)[2] = 3;
  (*idx)[3] = 5;
  (*idx)[4] = 6;
  (*idx)[5] = 7;
  (*idx)[6] = 8;

  // Define cropBox limit
  Eigen::Vector4f min_pt(-1.0f, -1.0f, -1.0f, 1.0f);
  Eigen::Vector4f max_pt(1.0f, 1.0f, 1.0f, 1.0f);

  // PointCloud without indices
  // -------------------------------------------------------------------------

  // Test the PointCloud<PointT> method
  experimental::CropBox<PointXYZ> cropBoxFilter(true);
  cropBoxFilter.setInputCloud(input);

  // Cropbox slightly bigger then bounding box of points
  cropBoxFilter.setMin(min_pt);
  cropBoxFilter.setMax(max_pt);

  // Indices
  pcl::Indices indices;
  cropBoxFilter.filter(indices);

  // Cloud
  PointCloud<PointXYZ> cloud_out;
  cropBoxFilter.filter(cloud_out);

  // Should contain all
  EXPECT_EQ(int(indices.size()), 9);
  EXPECT_EQ(int(cloud_out.size()), 9);
  EXPECT_EQ(int(cloud_out.width), 9);
  EXPECT_EQ(int(cloud_out.height), 1);

  IndicesConstPtr removed_indices;
  removed_indices = cropBoxFilter.getRemovedIndices();
  EXPECT_EQ(int(removed_indices->size()), 0);

  // Test setNegative
  PointCloud<PointXYZ> cloud_out_negative;
  cropBoxFilter.setNegative(true);
  cropBoxFilter.filter(cloud_out_negative);
  EXPECT_EQ(int(cloud_out_negative.size()), 0);

  cropBoxFilter.filter(indices);
  EXPECT_EQ(int(indices.size()), 0);
  cropBoxFilter.setNegative(false);
  cropBoxFilter.filter(cloud_out);

  // Translate crop box up by 1
  cropBoxFilter.setTranslation(Eigen::Vector3f(0, 1, 0));
  cropBoxFilter.filter(indices);
  cropBoxFilter.filter(cloud_out);

  EXPECT_EQ(int(indices.size()), 5);
  EXPECT_EQ(int(cloud_out.size()), 5);

  removed_indices = cropBoxFilter.getRemovedIndices();
  EXPECT_EQ(int(removed_indices->size()), 4);

  // Test setNegative
  cropBoxFilter.setNegative(true);
  cropBoxFilter.filter(cloud_out_negative);
  EXPECT_EQ(int(cloud_out_negative.size()), 4);

  cropBoxFilter.filter(indices);
  EXPECT_EQ(int(indices.size()), 4);

  cropBoxFilter.setNegative(false);
  cropBoxFilter.filter(cloud_out);

  // Rotate crop box up by 45
  cropBoxFilter.setRotation(Eigen::Vector3f(0.0f, 45.0f * float(M_PI) / 180.0f, 0.0f));
  cropBoxFilter.filter(indices);
  cropBoxFilter.filter(cloud_out);

  EXPECT_EQ(int(indices.size()), 1);
  EXPECT_EQ(int(cloud_out.size()), 1);
  EXPECT_EQ(int(cloud_out.width), 1);
  EXPECT_EQ(int(cloud_out.height), 1);

  removed_indices = cropBoxFilter.getRemovedIndices();
  EXPECT_EQ(int(removed_indices->size()), 8);

  // Test setNegative
  cropBoxFilter.setNegative(true);
  cropBoxFilter.filter(cloud_out_negative);
  EXPECT_EQ(int(cloud_out_negative.size()), 8);

  cropBoxFilter.filter(indices);
  EXPECT_EQ(int(indices.size()), 8);

  cropBoxFilter.setNegative(false);
  cropBoxFilter.filter(cloud_out);

  // Rotate point cloud by -45
  cropBoxFilter.setTransform(
      getTransformation(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -45.0f * float(M_PI) / 180.0f));
  cropBoxFilter.filter(indices);
  cropBoxFilter.filter(cloud_out);

  EXPECT_EQ(int(indices.size()), 3);
  EXPECT_EQ(int(cloud_out.size()), 3);
  EXPECT_EQ(int(cloud_out.width), 3);
  EXPECT_EQ(int(cloud_out.height), 1);

  removed_indices = cropBoxFilter.getRemovedIndices();
  EXPECT_EQ(int(removed_indices->size()), 6);

  // Test setNegative
  cropBoxFilter.setNegative(true);
  cropBoxFilter.filter(cloud_out_negative);
  EXPECT_EQ(int(cloud_out_negative.size()), 6);

  cropBoxFilter.filter(indices);
  EXPECT_EQ(int(indices.size()), 6);

  cropBoxFilter.setNegative(false);
  cropBoxFilter.filter(cloud_out);

  // Translate point cloud down by -1
  cropBoxFilter.setTransform(
      getTransformation(0, -1, 0, 0, 0, -45.0 * float(M_PI) / 180.0));
  cropBoxFilter.filter(indices);
  cropBoxFilter.filter(cloud_out);

  EXPECT_EQ(int(indices.size()), 2);
  EXPECT_EQ(int(cloud_out.size()), 2);
  EXPECT_EQ(int(cloud_out.width), 2);
  EXPECT_EQ(int(cloud_out.height), 1);

  removed_indices = cropBoxFilter.getRemovedIndices();
  EXPECT_EQ(int(removed_indices->size()), 7);

  // Test setNegative
  cropBoxFilter.setNegative(true);
  cropBoxFilter.filter(cloud_out_negative);
  EXPECT_EQ(int(cloud_out_negative.size()), 7);

  cropBoxFilter.filter(indices);
  EXPECT_EQ(int(indices.size()), 7);

  cropBoxFilter.setNegative(false);
  cropBoxFilter.filter(cloud_out);

  // Remove point cloud rotation
  cropBoxFilter.setTransform(getTransformation(0, -1, 0, 0, 0, 0));
  cropBoxFilter.filter(indices);
  cropBoxFilter.filter(cloud_out);

  EXPECT_EQ(int(indices.size()), 0);
  EXPECT_EQ(int(cloud_out.size()), 0);
  EXPECT_EQ(int(cloud_out.width), 0);
  EXPECT_EQ(int(cloud_out.height), 1);

  removed_indices = cropBoxFilter.getRemovedIndices();
  EXPECT_EQ(int(removed_indices->size()), 9);

  // Test setNegative
  cropBoxFilter.setNegative(true);
  cropBoxFilter.filter(cloud_out_negative);
  EXPECT_EQ(int(cloud_out_negative.size()), 9);

  cropBoxFilter.filter(indices);
  EXPECT_EQ(int(indices.size()), 9);

  // PointCloud with indices selection
  // -------------------------------------------------------------------------

  // Reset cropBox transformation
  cropBoxFilter.setNegative(false);
  cropBoxFilter.setTransform(getTransformation(0, 0, 0, 0, 0, 0));
  cropBoxFilter.setRotation(Eigen::Vector3f(0.0f, 0.0f, 0.0f));
  cropBoxFilter.setTranslation(Eigen::Vector3f(0.0f, 0.0f, 0.0f));

  // Setup input indices selection
  cropBoxFilter.setIndices(idx);

  // Indices
  cropBoxFilter.filter(indices);

  // Cloud
  cropBoxFilter.filter(cloud_out);

  // Should contain all
  EXPECT_EQ(int(indices.size()), 7);
  EXPECT_VECTOR_CONTAINS_ALL(pcl::Indices({1, 2, 3, 5, 6, 7, 8}), indices);
  EXPECT_EQ(int(cloud_out.size()), 7);
  EXPECT_EQ(int(cloud_out.width), 7);
  EXPECT_EQ(int(cloud_out.height), 1);

  removed_indices = cropBoxFilter.getRemovedIndices();
  EXPECT_EQ(int(removed_indices->size()), 0);

  // Test setNegative
  cropBoxFilter.setNegative(true);
  cropBoxFilter.filter(cloud_out_negative);
  EXPECT_EQ(int(cloud_out_negative.size()), 0);

  cropBoxFilter.filter(indices);
  EXPECT_EQ(int(indices.size()), 0);
  cropBoxFilter.setNegative(false);
  cropBoxFilter.filter(cloud_out);

  // Translate crop box up by 1
  cropBoxFilter.setTranslation(Eigen::Vector3f(0, 1, 0));
  cropBoxFilter.filter(indices);
  cropBoxFilter.filter(cloud_out);

  EXPECT_EQ(int(indices.size()), 3);
  EXPECT_VECTOR_CONTAINS_ALL(pcl::Indices({1, 2, 7}), indices);
  EXPECT_EQ(int(cloud_out.size()), 3);

  removed_indices = cropBoxFilter.getRemovedIndices();
  EXPECT_EQ(int(removed_indices->size()), 4);
  EXPECT_VECTOR_CONTAINS_ALL(pcl::Indices({3, 5, 6, 8}), *removed_indices);

  // Test setNegative
  cropBoxFilter.setNegative(true);
  cropBoxFilter.filter(cloud_out_negative);
  EXPECT_EQ(int(cloud_out_negative.size()), 4);

  cropBoxFilter.filter(indices);
  EXPECT_EQ(int(indices.size()), 4);
  EXPECT_VECTOR_CONTAINS_ALL(pcl::Indices({3, 5, 6, 8}), indices);

  cropBoxFilter.setNegative(false);
  cropBoxFilter.filter(cloud_out);

  // Rotate crop box up by 45
  cropBoxFilter.setRotation(Eigen::Vector3f(0.0f, 45.0f * float(M_PI) / 180.0f, 0.0f));
  cropBoxFilter.filter(indices);
  cropBoxFilter.filter(cloud_out);

  EXPECT_EQ(int(indices.size()), 0);
  EXPECT_EQ(int(cloud_out.size()), 0);
  EXPECT_EQ(int(cloud_out.width), 0);
  EXPECT_EQ(int(cloud_out.height), 1);

  removed_indices = cropBoxFilter.getRemovedIndices();
  EXPECT_EQ(int(removed_indices->size()), 7);
  EXPECT_VECTOR_DOES_NOT_CONTAIN(pcl::Indices({0, 4}), *removed_indices);
  // Test setNegative
  cropBoxFilter.setNegative(true);
  cropBoxFilter.filter(cloud_out_negative);
  EXPECT_EQ(int(cloud_out_negative.size()), 7);

  cropBoxFilter.filter(indices);
  EXPECT_EQ(int(indices.size()), 7);
  EXPECT_VECTOR_DOES_NOT_CONTAIN(pcl::Indices({0, 4}), indices);

  cropBoxFilter.setNegative(false);
  cropBoxFilter.filter(cloud_out);

  // Rotate point cloud by -45
  cropBoxFilter.setTransform(
      getTransformation(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -45.0f * float(M_PI) / 180.0f));
  cropBoxFilter.filter(indices);
  cropBoxFilter.filter(cloud_out);

  EXPECT_EQ(int(indices.size()), 1);
  EXPECT_VECTOR_CONTAINS_ALL(pcl::Indices({7}), indices);
  EXPECT_EQ(int(cloud_out.size()), 1);
  EXPECT_EQ(int(cloud_out.width), 1);
  EXPECT_EQ(int(cloud_out.height), 1);

  removed_indices = cropBoxFilter.getRemovedIndices();
  EXPECT_EQ(int(removed_indices->size()), 6);
  EXPECT_VECTOR_CONTAINS_ALL(pcl::Indices({1, 2, 3, 5, 6, 8}), *removed_indices);

  // Test setNegative
  cropBoxFilter.setNegative(true);
  cropBoxFilter.filter(cloud_out_negative);
  EXPECT_EQ(int(cloud_out_negative.size()), 6);

  cropBoxFilter.filter(indices);
  EXPECT_EQ(int(indices.size()), 6);
  EXPECT_VECTOR_CONTAINS_ALL(pcl::Indices({1, 2, 3, 5, 6, 8}), indices);

  cropBoxFilter.setNegative(false);
  cropBoxFilter.filter(cloud_out);

  // Translate point cloud down by -1
  cropBoxFilter.setTransform(
      getTransformation(0, -1, 0, 0, 0, -45.0 * float(M_PI) / 180.0));
  cropBoxFilter.filter(indices);
  cropBoxFilter.filter(cloud_out);

  EXPECT_EQ(int(indices.size()), 1);
  EXPECT_VECTOR_CONTAINS_ALL(indices, pcl::Indices({7}));
  EXPECT_EQ(int(cloud_out.size()), 1);
  EXPECT_EQ(int(cloud_out.width), 1);
  EXPECT_EQ(int(cloud_out.height), 1);

  removed_indices = cropBoxFilter.getRemovedIndices();
  EXPECT_EQ(int(removed_indices->size()), 6);
  EXPECT_VECTOR_CONTAINS_ALL(pcl::Indices({1, 2, 3, 5, 6, 8}), *removed_indices);

  // Test setNegative
  cropBoxFilter.setNegative(true);
  cropBoxFilter.filter(cloud_out_negative);
  EXPECT_EQ(int(cloud_out_negative.size()), 6);

  cropBoxFilter.filter(indices);
  EXPECT_EQ(int(indices.size()), 6);
  EXPECT_VECTOR_CONTAINS_ALL(pcl::Indices({1, 2, 3, 5, 6, 8}), indices);

  cropBoxFilter.setNegative(false);
  cropBoxFilter.filter(cloud_out);

  // Remove point cloud rotation
  cropBoxFilter.setTransform(getTransformation(0, -1, 0, 0, 0, 0));
  cropBoxFilter.filter(indices);
  cropBoxFilter.filter(cloud_out);

  EXPECT_EQ(int(indices.size()), 0);
  EXPECT_EQ(int(cloud_out.size()), 0);
  EXPECT_EQ(int(cloud_out.width), 0);
  EXPECT_EQ(int(cloud_out.height), 1);

  removed_indices = cropBoxFilter.getRemovedIndices();
  EXPECT_EQ(int(removed_indices->size()), 7);
  EXPECT_VECTOR_DOES_NOT_CONTAIN(pcl::Indices({0, 4}), *removed_indices);

  // Test setNegative
  cropBoxFilter.setNegative(true);
  cropBoxFilter.filter(cloud_out_negative);
  EXPECT_EQ(int(cloud_out_negative.size()), 7);

  cropBoxFilter.filter(indices);
  EXPECT_EQ(int(indices.size()), 7);
  EXPECT_VECTOR_DOES_NOT_CONTAIN(pcl::Indices({0, 4}), indices);
}

int
main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
