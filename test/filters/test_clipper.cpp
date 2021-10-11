/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009-2012, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
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
 * $Id$
 *
 */

#include <pcl/test/gtest.h>
#include <pcl/memory.h>  // for pcl::make_shared
#include <pcl/pcl_base.h>  // for pcl::Indices
#include <pcl/pcl_tests.h>
#include <pcl/point_types.h>
#include <pcl/filters/box_clipper3D.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/common/eigen.h>

using namespace pcl;
using namespace Eigen;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (BoxClipper3D, Filters)
{
  // PointCloud
  // -------------------------------------------------------------------------

  // Create cloud with center point and corner points
  PointCloud<PointXYZ>::Ptr input (new PointCloud<PointXYZ> ());
  input->push_back (PointXYZ (0.0f, 0.0f, 0.0f));
  input->push_back (PointXYZ (0.9f, 0.9f, 0.9f));
  input->push_back (PointXYZ (0.9f, 0.9f, -0.9f));
  input->push_back (PointXYZ (0.9f, -0.9f, 0.9f));
  input->push_back (PointXYZ (-0.9f, 0.9f, 0.9f));
  input->push_back (PointXYZ (0.9f, -0.9f, -0.9f));
  input->push_back (PointXYZ (-0.9f, -0.9f, 0.9f));
  input->push_back (PointXYZ (-0.9f, 0.9f, -0.9f));
  input->push_back (PointXYZ (-0.9f, -0.9f, -0.9f));

  ExtractIndices<PointXYZ> extract_indices;
  pcl::Indices indices;

  BoxClipper3D<PointXYZ> boxClipper3D (Affine3f::Identity ());
  boxClipper3D.clipPointCloud3D (*input, indices);

  PointCloud<PointXYZ> cloud_out;

  extract_indices.setInputCloud (input);
  extract_indices.setIndices (pcl::make_shared<pcl::Indices> (indices));
  extract_indices.filter (cloud_out);

  EXPECT_EQ (int (indices.size ()), 9);
  EXPECT_EQ (int (cloud_out.size ()), 9);
  EXPECT_EQ (int (cloud_out.width), 9);
  EXPECT_EQ (int (cloud_out.height), 1);

  // Translate points by 1 in Y-axis ...
  Affine3f t (Translation3f (0.0f, 1.0f, 0.0f));
  boxClipper3D.setTransformation (t);
  boxClipper3D.clipPointCloud3D (*input, indices);

  EXPECT_EQ (int (indices.size ()), 5);

  // ... then rotate points +45 in Y-Axis
  t.rotate (AngleAxisf (45.0f * float (M_PI) / 180.0f, Vector3f::UnitY ()));
  boxClipper3D.setTransformation (t);
  boxClipper3D.clipPointCloud3D (*input, indices);
  EXPECT_EQ (int (indices.size ()), 1);

  // ... then rotate points -45 in Z-axis
  t.rotate (AngleAxisf (-45.0f * float (M_PI) / 180.0f, Vector3f::UnitZ ()));
  boxClipper3D.setTransformation (t);
  boxClipper3D.clipPointCloud3D (*input, indices);
  EXPECT_EQ (int (indices.size ()), 3);

  // ... then scale points by 2
  t.scale (2.0f);
  boxClipper3D.setTransformation (t);
  boxClipper3D.clipPointCloud3D (*input, indices);
  EXPECT_EQ (int (indices.size ()), 1);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (CropBox, Filters)
{

  // PointT
  // -------------------------------------------------------------------------

  // Create cloud with center point and corner points
  PointCloud<PointXYZ>::Ptr input (new PointCloud<PointXYZ> ());

  input->push_back (PointXYZ (0.0f, 0.0f, 0.0f));
  input->push_back (PointXYZ (0.9f, 0.9f, 0.9f));
  input->push_back (PointXYZ (0.9f, 0.9f, -0.9f));
  input->push_back (PointXYZ (0.9f, -0.9f, 0.9f));
  input->push_back (PointXYZ (-0.9f, 0.9f, 0.9f));
  input->push_back (PointXYZ (0.9f, -0.9f, -0.9f));
  input->push_back (PointXYZ (-0.9f, -0.9f, 0.9f));
  input->push_back (PointXYZ (-0.9f, 0.9f, -0.9f));
  input->push_back (PointXYZ (-0.9f, -0.9f, -0.9f));

  // Create indices vector ( without 0 and 4)
  pcl::IndicesPtr idx (new pcl::Indices (7));
  (*idx)[0] = 1;
  (*idx)[1] = 2;
  (*idx)[2] = 3;
  (*idx)[3] = 5;
  (*idx)[4] = 6;
  (*idx)[5] = 7;
  (*idx)[6] = 8;

  // Define cropBox limit
  Eigen::Vector4f min_pt (-1.0f, -1.0f, -1.0f, 1.0f);
  Eigen::Vector4f max_pt (1.0f, 1.0f, 1.0f, 1.0f);

  // PointCloud without indices
  // -------------------------------------------------------------------------

  // Test the PointCloud<PointT> method
  CropBox<PointXYZ> cropBoxFilter (true);
  cropBoxFilter.setInputCloud (input);

  // Cropbox slightly bigger then bounding box of points
  cropBoxFilter.setMin (min_pt);
  cropBoxFilter.setMax (max_pt);

  // Indices
  pcl::Indices indices;
  cropBoxFilter.filter (indices);

  // Cloud
  PointCloud<PointXYZ> cloud_out;
  cropBoxFilter.filter (cloud_out);

  // Should contain all
  EXPECT_EQ (int (indices.size ()), 9);
  EXPECT_EQ (int (cloud_out.size ()), 9);
  EXPECT_EQ (int (cloud_out.width), 9);
  EXPECT_EQ (int (cloud_out.height), 1);

  IndicesConstPtr removed_indices;
  removed_indices = cropBoxFilter.getRemovedIndices ();
  EXPECT_EQ (int (removed_indices->size ()), 0);

  // Test setNegative
  PointCloud<PointXYZ> cloud_out_negative;
  cropBoxFilter.setNegative (true);
  cropBoxFilter.filter (cloud_out_negative);
  EXPECT_EQ (int (cloud_out_negative.size ()), 0);

  cropBoxFilter.filter (indices);
  EXPECT_EQ (int (indices.size ()), 0);
  cropBoxFilter.setNegative (false);
  cropBoxFilter.filter (cloud_out);

  // Translate crop box up by 1
  cropBoxFilter.setTranslation(Eigen::Vector3f(0, 1, 0));
  cropBoxFilter.filter (indices);
  cropBoxFilter.filter (cloud_out);

  EXPECT_EQ (int (indices.size ()), 5);
  EXPECT_EQ (int (cloud_out.size ()), 5);

  removed_indices = cropBoxFilter.getRemovedIndices ();
  EXPECT_EQ (int (removed_indices->size ()), 4);

  // Test setNegative
  cropBoxFilter.setNegative (true);
  cropBoxFilter.filter (cloud_out_negative);
  EXPECT_EQ (int (cloud_out_negative.size ()), 4);

  cropBoxFilter.filter (indices);
  EXPECT_EQ (int (indices.size ()), 4);

  cropBoxFilter.setNegative (false);
  cropBoxFilter.filter (cloud_out);

  // Rotate crop box up by 45
  cropBoxFilter.setRotation (Eigen::Vector3f (0.0f, 45.0f * float (M_PI) / 180.0f, 0.0f));
  cropBoxFilter.filter (indices);
  cropBoxFilter.filter (cloud_out);

  EXPECT_EQ (int (indices.size ()), 1);
  EXPECT_EQ (int (cloud_out.size ()), 1);
  EXPECT_EQ (int (cloud_out.width), 1);
  EXPECT_EQ (int (cloud_out.height), 1);

  removed_indices = cropBoxFilter.getRemovedIndices ();
  EXPECT_EQ (int (removed_indices->size ()), 8);

  // Test setNegative
  cropBoxFilter.setNegative (true);
  cropBoxFilter.filter (cloud_out_negative);
  EXPECT_EQ (int (cloud_out_negative.size ()), 8);

  cropBoxFilter.filter (indices);
  EXPECT_EQ (int (indices.size ()), 8);

  cropBoxFilter.setNegative (false);
  cropBoxFilter.filter (cloud_out);

  // Rotate point cloud by -45
  cropBoxFilter.setTransform (getTransformation (0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -45.0f * float (M_PI) / 180.0f));
  cropBoxFilter.filter (indices);
  cropBoxFilter.filter (cloud_out);

  EXPECT_EQ (int (indices.size ()), 3);
  EXPECT_EQ (int (cloud_out.size ()), 3);
  EXPECT_EQ (int (cloud_out.width), 3);
  EXPECT_EQ (int (cloud_out.height), 1);

  removed_indices = cropBoxFilter.getRemovedIndices ();
  EXPECT_EQ (int (removed_indices->size ()), 6);

  // Test setNegative
  cropBoxFilter.setNegative (true);
  cropBoxFilter.filter (cloud_out_negative);
  EXPECT_EQ (int (cloud_out_negative.size ()), 6);

  cropBoxFilter.filter (indices);
  EXPECT_EQ (int (indices.size ()), 6);

  cropBoxFilter.setNegative (false);
  cropBoxFilter.filter (cloud_out);

  // Translate point cloud down by -1
  cropBoxFilter.setTransform (getTransformation(0, -1, 0, 0, 0, -45.0 * float (M_PI) / 180.0));
  cropBoxFilter.filter (indices);
  cropBoxFilter.filter (cloud_out);

  EXPECT_EQ (int (indices.size ()), 2);
  EXPECT_EQ (int (cloud_out.size ()), 2);
  EXPECT_EQ (int (cloud_out.width), 2);
  EXPECT_EQ (int (cloud_out.height), 1);

  removed_indices = cropBoxFilter.getRemovedIndices ();
  EXPECT_EQ (int (removed_indices->size ()), 7);

  // Test setNegative
  cropBoxFilter.setNegative (true);
  cropBoxFilter.filter (cloud_out_negative);
  EXPECT_EQ (int (cloud_out_negative.size ()), 7);

  cropBoxFilter.filter (indices);
  EXPECT_EQ (int (indices.size ()), 7);

  cropBoxFilter.setNegative (false);
  cropBoxFilter.filter (cloud_out);

  // Remove point cloud rotation
  cropBoxFilter.setTransform (getTransformation(0, -1, 0, 0, 0, 0));
  cropBoxFilter.filter (indices);
  cropBoxFilter.filter (cloud_out);

  EXPECT_EQ (int (indices.size ()), 0);
  EXPECT_EQ (int (cloud_out.size ()), 0);
  EXPECT_EQ (int (cloud_out.width), 0);
  EXPECT_EQ (int (cloud_out.height), 1);

  removed_indices = cropBoxFilter.getRemovedIndices ();
  EXPECT_EQ (int (removed_indices->size ()), 9);

  // Test setNegative
  cropBoxFilter.setNegative (true);
  cropBoxFilter.filter (cloud_out_negative);
  EXPECT_EQ (int (cloud_out_negative.size ()), 9);

  cropBoxFilter.filter (indices);
  EXPECT_EQ (int (indices.size ()), 9);

  // PointCloud with indices selection
  // -------------------------------------------------------------------------

  // Reset cropBox transformation 
  cropBoxFilter.setNegative (false);
  cropBoxFilter.setTransform (getTransformation(0, 0, 0, 0, 0, 0));
  cropBoxFilter.setRotation (Eigen::Vector3f (0.0f, 0.0f, 0.0f));
  cropBoxFilter.setTranslation (Eigen::Vector3f (0.0f, 0.0f, 0.0f));

  // Setup input indices selection
  cropBoxFilter.setIndices(idx);

  // Indices
  cropBoxFilter.filter (indices);

  // Cloud
  cropBoxFilter.filter (cloud_out);

  // Should contain all
  EXPECT_EQ (int (indices.size ()), 7);
  EXPECT_VECTOR_CONTAINS_ALL (pcl::Indices({1, 2, 3, 5, 6, 7, 8}), indices);
  EXPECT_EQ (int (cloud_out.size ()), 7);
  EXPECT_EQ (int (cloud_out.width), 7);
  EXPECT_EQ (int (cloud_out.height), 1);

  removed_indices = cropBoxFilter.getRemovedIndices ();
  EXPECT_EQ (int (removed_indices->size ()), 0);

  // Test setNegative
  cropBoxFilter.setNegative (true);
  cropBoxFilter.filter (cloud_out_negative);
  EXPECT_EQ (int (cloud_out_negative.size ()), 0);

  cropBoxFilter.filter (indices);
  EXPECT_EQ (int (indices.size ()), 0);
  cropBoxFilter.setNegative (false);
  cropBoxFilter.filter (cloud_out);

  // Translate crop box up by 1
  cropBoxFilter.setTranslation(Eigen::Vector3f(0, 1, 0));
  cropBoxFilter.filter (indices);
  cropBoxFilter.filter (cloud_out);

  EXPECT_EQ (int (indices.size ()), 3);
  EXPECT_VECTOR_CONTAINS_ALL (pcl::Indices({1, 2, 7}), indices);
  EXPECT_EQ (int (cloud_out.size ()), 3);

  removed_indices = cropBoxFilter.getRemovedIndices ();
  EXPECT_EQ (int (removed_indices->size ()), 4);
  EXPECT_VECTOR_CONTAINS_ALL (pcl::Indices({3, 5, 6, 8}), *removed_indices);

  // Test setNegative
  cropBoxFilter.setNegative (true);
  cropBoxFilter.filter (cloud_out_negative);
  EXPECT_EQ (int (cloud_out_negative.size ()), 4);

  cropBoxFilter.filter (indices);
  EXPECT_EQ (int (indices.size ()), 4);
  EXPECT_VECTOR_CONTAINS_ALL (pcl::Indices({3, 5, 6, 8}), indices);

  cropBoxFilter.setNegative (false);
  cropBoxFilter.filter (cloud_out);

  // Rotate crop box up by 45
  cropBoxFilter.setRotation (Eigen::Vector3f (0.0f, 45.0f * float (M_PI) / 180.0f, 0.0f));
  cropBoxFilter.filter (indices);
  cropBoxFilter.filter (cloud_out);

  EXPECT_EQ (int (indices.size ()), 0);
  EXPECT_EQ (int (cloud_out.size ()), 0);
  EXPECT_EQ (int (cloud_out.width), 0);
  EXPECT_EQ (int (cloud_out.height), 1);

  removed_indices = cropBoxFilter.getRemovedIndices ();
  EXPECT_EQ (int (removed_indices->size ()), 7);
  EXPECT_VECTOR_DOES_NOT_CONTAIN (pcl::Indices({0, 4}), *removed_indices);
  // Test setNegative
  cropBoxFilter.setNegative (true);
  cropBoxFilter.filter (cloud_out_negative);
  EXPECT_EQ (int (cloud_out_negative.size ()), 7);

  cropBoxFilter.filter (indices);
  EXPECT_EQ (int (indices.size ()), 7);
  EXPECT_VECTOR_DOES_NOT_CONTAIN(pcl::Indices({0, 4}), indices);

  cropBoxFilter.setNegative (false);
  cropBoxFilter.filter (cloud_out);

  // Rotate point cloud by -45
  cropBoxFilter.setTransform (getTransformation (0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -45.0f * float (M_PI) / 180.0f));
  cropBoxFilter.filter (indices);
  cropBoxFilter.filter (cloud_out);

  EXPECT_EQ (int (indices.size ()), 1);
  EXPECT_VECTOR_CONTAINS_ALL (pcl::Indices({7}), indices);
  EXPECT_EQ (int (cloud_out.size ()), 1);
  EXPECT_EQ (int (cloud_out.width), 1);
  EXPECT_EQ (int (cloud_out.height), 1);

  removed_indices = cropBoxFilter.getRemovedIndices ();
  EXPECT_EQ (int (removed_indices->size ()), 6);
  EXPECT_VECTOR_CONTAINS_ALL (pcl::Indices({1, 2, 3, 5, 6, 8}), *removed_indices);

  // Test setNegative
  cropBoxFilter.setNegative (true);
  cropBoxFilter.filter (cloud_out_negative);
  EXPECT_EQ (int (cloud_out_negative.size ()), 6);

  cropBoxFilter.filter (indices);
  EXPECT_EQ (int (indices.size ()), 6);
  EXPECT_VECTOR_CONTAINS_ALL (pcl::Indices({1, 2, 3, 5, 6, 8}), indices);

  cropBoxFilter.setNegative (false);
  cropBoxFilter.filter (cloud_out);

  // Translate point cloud down by -1
  cropBoxFilter.setTransform (getTransformation(0, -1, 0, 0, 0, -45.0 * float (M_PI) / 180.0));
  cropBoxFilter.filter (indices);
  cropBoxFilter.filter (cloud_out);

  EXPECT_EQ (int (indices.size ()), 1);
  EXPECT_VECTOR_CONTAINS_ALL (indices, pcl::Indices({7}));
  EXPECT_EQ (int (cloud_out.size ()), 1);
  EXPECT_EQ (int (cloud_out.width), 1);
  EXPECT_EQ (int (cloud_out.height), 1);

  removed_indices = cropBoxFilter.getRemovedIndices ();
  EXPECT_EQ (int (removed_indices->size ()), 6);
  EXPECT_VECTOR_CONTAINS_ALL (pcl::Indices( {1, 2, 3, 5, 6, 8}), *removed_indices);

  // Test setNegative
  cropBoxFilter.setNegative (true);
  cropBoxFilter.filter (cloud_out_negative);
  EXPECT_EQ (int (cloud_out_negative.size ()), 6);

  cropBoxFilter.filter (indices);
  EXPECT_EQ (int (indices.size ()), 6);
  EXPECT_VECTOR_CONTAINS_ALL (pcl::Indices({1, 2, 3, 5, 6, 8}), indices);

  cropBoxFilter.setNegative (false);
  cropBoxFilter.filter (cloud_out);

  // Remove point cloud rotation
  cropBoxFilter.setTransform (getTransformation(0, -1, 0, 0, 0, 0));
  cropBoxFilter.filter (indices);
  cropBoxFilter.filter (cloud_out);

  EXPECT_EQ (int (indices.size ()), 0);
  EXPECT_EQ (int (cloud_out.size ()), 0);
  EXPECT_EQ (int (cloud_out.width), 0);
  EXPECT_EQ (int (cloud_out.height), 1);

  removed_indices = cropBoxFilter.getRemovedIndices ();
  EXPECT_EQ (int (removed_indices->size ()), 7);
  EXPECT_VECTOR_DOES_NOT_CONTAIN (pcl::Indices({0, 4}), *removed_indices);

  // Test setNegative
  cropBoxFilter.setNegative (true);
  cropBoxFilter.filter (cloud_out_negative);
  EXPECT_EQ (int (cloud_out_negative.size ()), 7);

  cropBoxFilter.filter (indices);
  EXPECT_EQ (int (indices.size ()), 7);
  EXPECT_VECTOR_DOES_NOT_CONTAIN (pcl::Indices({0, 4}), indices);

  // PCLPointCloud2 without indices
  // -------------------------------------------------------------------------

  // Create cloud with center point and corner points
  PCLPointCloud2::Ptr input2 (new PCLPointCloud2);
  pcl::toPCLPointCloud2 (*input, *input2);

  // Test the PointCloud<PointT> method
  CropBox<PCLPointCloud2> cropBoxFilter2(true);
  cropBoxFilter2.setInputCloud (input2);

  // Cropbox slightly bigger then bounding box of points
  cropBoxFilter2.setMin (min_pt);
  cropBoxFilter2.setMax (max_pt);

  // Indices
  pcl::Indices indices2;
  cropBoxFilter2.filter (indices2);

  // Cloud
  PCLPointCloud2 cloud_out2;
  cropBoxFilter2.filter (cloud_out2);

  // Should contain all
  EXPECT_EQ (int (indices2.size ()), 9);
  EXPECT_EQ (int (indices2.size ()), int (cloud_out2.width * cloud_out2.height));

  IndicesConstPtr removed_indices2;
  removed_indices2 = cropBoxFilter2.getRemovedIndices ();
  EXPECT_EQ (int (removed_indices2->size ()), 0);

  // Test setNegative
  PCLPointCloud2 cloud_out2_negative;
  cropBoxFilter2.setNegative (true);
  cropBoxFilter2.filter (cloud_out2_negative);
  EXPECT_EQ (int (cloud_out2_negative.width), 0);

  cropBoxFilter2.filter (indices2);
  EXPECT_EQ (int (indices2.size ()), 0);

  cropBoxFilter2.setNegative (false);
  cropBoxFilter2.filter (cloud_out2);

  // Translate crop box up by 1
  cropBoxFilter2.setTranslation (Eigen::Vector3f(0, 1, 0));
  cropBoxFilter2.filter (indices2);
  cropBoxFilter2.filter (cloud_out2);

  EXPECT_EQ (int (indices2.size ()), 5);
  EXPECT_EQ (int (indices2.size ()), int (cloud_out2.width * cloud_out2.height));

  removed_indices2 = cropBoxFilter2.getRemovedIndices ();
  EXPECT_EQ (int (removed_indices2->size ()), 4);

  // Test setNegative
  cropBoxFilter2.setNegative (true);
  cropBoxFilter2.filter (cloud_out2_negative);
  EXPECT_EQ (int (cloud_out2_negative.width), 4);

  cropBoxFilter2.filter (indices2);
  EXPECT_EQ (int (indices2.size ()), 4);

  cropBoxFilter2.setNegative (false);
  cropBoxFilter2.filter (cloud_out2);

  // Rotate crop box up by 45
  cropBoxFilter2.setRotation (Eigen::Vector3f (0.0f, 45.0f * float (M_PI) / 180.0f, 0.0f));
  cropBoxFilter2.filter (indices2);
  cropBoxFilter2.filter (cloud_out2);

  EXPECT_EQ (int (indices2.size ()), 1);
  EXPECT_EQ (int (indices2.size ()), int (cloud_out2.width * cloud_out2.height));

  // Rotate point cloud by -45
  cropBoxFilter2.setTransform (getTransformation (0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -45.0f * float (M_PI) / 180.0f));
  cropBoxFilter2.filter (indices2);
  cropBoxFilter2.filter (cloud_out2);

  EXPECT_EQ (int (indices2.size ()), 3);
  EXPECT_EQ (int (cloud_out2.width * cloud_out2.height), 3);

  removed_indices2 = cropBoxFilter2.getRemovedIndices ();
  EXPECT_EQ (int (removed_indices2->size ()), 6);

  // Test setNegative
  cropBoxFilter2.setNegative (true);
  cropBoxFilter2.filter (cloud_out2_negative);
  EXPECT_EQ (int (cloud_out2_negative.width), 6);

  cropBoxFilter2.filter (indices2);
  EXPECT_EQ (int (indices2.size ()), 6);

  cropBoxFilter2.setNegative (false);
  cropBoxFilter2.filter (cloud_out2);

  // Translate point cloud down by -1
  cropBoxFilter2.setTransform (getTransformation (0.0f, -1.0f, 0.0f, 0.0f, 0.0f, -45.0f * float (M_PI) / 180.0f));
  cropBoxFilter2.filter (indices2);
  cropBoxFilter2.filter (cloud_out2);

  EXPECT_EQ (int (indices2.size ()), 2);
  EXPECT_EQ (int (cloud_out2.width * cloud_out2.height), 2);

  removed_indices2 = cropBoxFilter2.getRemovedIndices ();
  EXPECT_EQ (int (removed_indices2->size ()), 7);

  // Test setNegative
  cropBoxFilter2.setNegative (true);
  cropBoxFilter2.filter (cloud_out2_negative);
  EXPECT_EQ (int (cloud_out2_negative.width), 7);

  cropBoxFilter2.filter (indices2);
  EXPECT_EQ (int (indices2.size ()), 7);

  cropBoxFilter2.setNegative (false);
  cropBoxFilter2.filter (cloud_out2);

  // Remove point cloud rotation
  cropBoxFilter2.setTransform (getTransformation(0, -1, 0, 0, 0, 0));
  cropBoxFilter2.filter (indices2);
  cropBoxFilter2.filter (cloud_out2);

  EXPECT_EQ (int (indices2.size ()), 0);
  EXPECT_EQ (int (cloud_out2.width * cloud_out2.height), 0);

  removed_indices2 = cropBoxFilter2.getRemovedIndices ();
  EXPECT_EQ (int (removed_indices2->size ()), 9);

  // Test setNegative
  cropBoxFilter2.setNegative (true);
  cropBoxFilter2.filter (cloud_out2_negative);
  EXPECT_EQ (int (cloud_out2_negative.width), 9);

  cropBoxFilter2.filter (indices2);
  EXPECT_EQ (int (indices2.size ()), 9);

  cropBoxFilter2.setNegative (false);
  cropBoxFilter2.filter (cloud_out2);

    // PointCloud2 with indices selection
    // -------------------------------------------------------------------------

    // Reset cropBox transformation
    cropBoxFilter2.setNegative (false);
    cropBoxFilter2.setTransform (getTransformation(0, 0, 0, 0, 0, 0));
    cropBoxFilter2.setRotation (Eigen::Vector3f (0.0f, 0.0f, 0.0f));
    cropBoxFilter2.setTranslation (Eigen::Vector3f (0.0f, 0.0f, 0.0f));

    // Setup input indices selection
    cropBoxFilter2.setIndices(idx);

    // Indices
    cropBoxFilter2.filter (indices2);

    // Cloud
    cropBoxFilter2.filter (cloud_out2);

    // Should contain all
    EXPECT_EQ (int (indices2.size ()), 7);
    EXPECT_VECTOR_CONTAINS_ALL (pcl::Indices({1, 2, 3, 5, 6, 7, 8}), indices2);
    EXPECT_EQ (int (cloud_out2.width), 7);
    EXPECT_EQ (int (cloud_out2.height), 1);

    removed_indices2 = cropBoxFilter2.getRemovedIndices ();
    EXPECT_EQ (int (removed_indices2->size ()), 0);

    // Test setNegative
    cropBoxFilter2.setNegative (true);
    cropBoxFilter2.filter (cloud_out2_negative);
    EXPECT_EQ (int (cloud_out2_negative.height*cloud_out2_negative.width), 0);

    cropBoxFilter2.filter (indices2);
    EXPECT_EQ (int (indices2.size ()), 0);
    cropBoxFilter2.setNegative (false);
    cropBoxFilter2.filter (cloud_out2);

    // Translate crop box up by 1
    cropBoxFilter2.setTranslation(Eigen::Vector3f(0, 1, 0));
    cropBoxFilter2.filter (indices2);
    cropBoxFilter2.filter (cloud_out2);

    EXPECT_EQ (int (indices2.size ()), 3);
    EXPECT_VECTOR_CONTAINS_ALL (pcl::Indices({1, 2, 7}), indices2);
    EXPECT_EQ (int (cloud_out2.width*cloud_out2.height), 3);

    removed_indices2 = cropBoxFilter2.getRemovedIndices ();
    EXPECT_EQ (int (removed_indices2->size ()), 4);
    EXPECT_VECTOR_CONTAINS_ALL (pcl::Indices({3, 5, 6, 8}), *removed_indices2);

    // Test setNegative
    cropBoxFilter2.setNegative (true);
    cropBoxFilter2.filter (cloud_out2_negative);
    EXPECT_EQ (int (cloud_out2_negative.width*cloud_out2_negative.height), 4);

    cropBoxFilter2.filter (indices2);
    EXPECT_EQ (int (indices2.size ()), 4);
    EXPECT_VECTOR_CONTAINS_ALL (pcl::Indices({3, 5, 6, 8}), indices2);

    cropBoxFilter2.setNegative (false);
    cropBoxFilter2.filter (cloud_out2);

    // Rotate crop box up by 45
    cropBoxFilter2.setRotation (Eigen::Vector3f (0.0f, 45.0f * float (M_PI) / 180.0f, 0.0f));
    cropBoxFilter2.filter (indices2);
    cropBoxFilter2.filter (cloud_out2);

    EXPECT_EQ (int (indices2.size ()), 0);
    EXPECT_EQ (int (cloud_out2.width), 0);
    EXPECT_EQ (int (cloud_out2.height), 1);

    removed_indices2 = cropBoxFilter2.getRemovedIndices ();
    EXPECT_EQ (int (removed_indices2->size ()), 7);

    EXPECT_VECTOR_DOES_NOT_CONTAIN (pcl::Indices({0, 4}), *removed_indices2);
    // Test setNegative
    cropBoxFilter2.setNegative (true);
    cropBoxFilter2.filter (cloud_out2_negative);
    EXPECT_EQ (int (cloud_out2_negative.width*cloud_out2_negative.height), 7);

    cropBoxFilter2.filter (indices2);
    EXPECT_EQ (int (indices2.size ()), 7);
    EXPECT_VECTOR_DOES_NOT_CONTAIN (pcl::Indices({0, 4}), indices2);

    cropBoxFilter2.setNegative (false);
    cropBoxFilter2.filter (cloud_out2);

    // Rotate point cloud by -45
    cropBoxFilter2.setTransform (getTransformation (0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -45.0f * float (M_PI) / 180.0f));
    cropBoxFilter2.filter (indices2);
    cropBoxFilter2.filter (cloud_out2);

    EXPECT_EQ (int (indices2.size ()), 1);
    EXPECT_VECTOR_CONTAINS_ALL (pcl::Indices({7}), indices2);
    EXPECT_EQ (int (cloud_out2.width), 1);
    EXPECT_EQ (int (cloud_out2.height), 1);

    removed_indices2 = cropBoxFilter2.getRemovedIndices ();
    EXPECT_EQ (int (removed_indices2->size ()), 6);
    EXPECT_VECTOR_CONTAINS_ALL (pcl::Indices({1, 2, 3, 5, 6, 8}), *removed_indices2);

    // Test setNegative
    cropBoxFilter2.setNegative (true);
    cropBoxFilter2.filter (cloud_out2_negative);
    EXPECT_EQ (int (cloud_out2_negative.width*cloud_out2_negative.height), 6);

    cropBoxFilter2.filter (indices2);
    EXPECT_EQ (int (indices2.size ()), 6);
    EXPECT_VECTOR_CONTAINS_ALL (pcl::Indices({1, 2, 3, 5, 6, 8}), indices2);

    cropBoxFilter2.setNegative (false);
    cropBoxFilter2.filter (cloud_out2);

    // Translate point cloud down by -1
    cropBoxFilter2.setTransform (getTransformation(0, -1, 0, 0, 0, -45.0 * float (M_PI) / 180.0));
    cropBoxFilter2.filter (indices2);
    cropBoxFilter2.filter (cloud_out2);

    EXPECT_EQ (int (indices2.size ()), 1);
    EXPECT_VECTOR_CONTAINS_ALL (pcl::Indices({7}), indices2);
    EXPECT_EQ (int (cloud_out2.width), 1);
    EXPECT_EQ (int (cloud_out2.height), 1);

    removed_indices = cropBoxFilter.getRemovedIndices ();
    EXPECT_EQ (int (removed_indices2->size ()), 6);
    EXPECT_VECTOR_CONTAINS_ALL (pcl::Indices({1, 2, 3, 5, 6, 8}), *removed_indices2);

    // Test setNegative
    cropBoxFilter2.setNegative (true);
    cropBoxFilter2.filter (cloud_out2_negative);
    EXPECT_EQ (int (cloud_out2_negative.width*cloud_out2_negative.height), 6);

    cropBoxFilter2.filter (indices2);
    EXPECT_EQ (int (indices2.size ()), 6);
    EXPECT_VECTOR_CONTAINS_ALL (pcl::Indices({1, 2, 3, 5, 6, 8}), indices2);

    cropBoxFilter2.setNegative (false);
    cropBoxFilter2.filter (cloud_out2);

    // Remove point cloud rotation
    cropBoxFilter2.setTransform (getTransformation(0, -1, 0, 0, 0, 0));
    cropBoxFilter2.filter (indices2);
    cropBoxFilter2.filter (cloud_out2);

    EXPECT_EQ (int (indices2.size ()), 0);
    EXPECT_EQ (int (cloud_out2.width), 0);
    EXPECT_EQ (int (cloud_out2.height), 1);

    removed_indices2 = cropBoxFilter2.getRemovedIndices ();
    EXPECT_EQ (int (removed_indices2->size ()), 7);
    EXPECT_VECTOR_DOES_NOT_CONTAIN (pcl::Indices({0, 4}), *removed_indices2);

    // Test setNegative
    cropBoxFilter2.setNegative (true);
    cropBoxFilter2.filter (cloud_out2_negative);
    EXPECT_EQ (int (cloud_out2_negative.height*cloud_out2_negative.width), 7);

    cropBoxFilter2.filter (indices2);
    EXPECT_EQ (int (indices2.size ()), 7);
    EXPECT_VECTOR_DOES_NOT_CONTAIN (pcl::Indices({0, 4}), indices2);

}

/* ---[ */
int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
