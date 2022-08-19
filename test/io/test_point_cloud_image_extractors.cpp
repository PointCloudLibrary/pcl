/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2013-, Open Perception, Inc.
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

#include <iostream>

#include <pcl/test/gtest.h>
#include <pcl/point_types.h>
#include <pcl/io/point_cloud_image_extractors.h>

using namespace pcl;
using namespace pcl::io;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, PointCloudImageExtractorFromNormalField)
{
  using PointT = PointNormal;
  PointCloud<PointT> cloud;
  cloud.width = 2;
  cloud.height = 2;
  cloud.is_dense = true;
  cloud.resize (cloud.width * cloud.height);
  for (auto &point : cloud.points)
  {
    point.normal_x = -1.0;
    point.normal_y =  0.0;
    point.normal_z =  1.0;
  }
  pcl::PCLImage image;
  PointCloudImageExtractorFromNormalField<PointT> pcie;

  ASSERT_TRUE (pcie.extract (cloud, image));

  EXPECT_EQ ("rgb8", image.encoding);
  EXPECT_EQ (cloud.width, image.width);
  EXPECT_EQ (cloud.height, image.height);

  EXPECT_EQ (  0, image.data[0 * 3 + 0]);
  EXPECT_EQ (  0, image.data[1 * 3 + 0]);
  EXPECT_EQ (  0, image.data[2 * 3 + 0]);
  EXPECT_EQ (  0, image.data[3 * 3 + 0]);
  EXPECT_EQ (127, image.data[0 * 3 + 1]);
  EXPECT_EQ (127, image.data[1 * 3 + 1]);
  EXPECT_EQ (127, image.data[2 * 3 + 1]);
  EXPECT_EQ (127, image.data[3 * 3 + 1]);
  EXPECT_EQ (254, image.data[0 * 3 + 2]);
  EXPECT_EQ (254, image.data[1 * 3 + 2]);
  EXPECT_EQ (254, image.data[2 * 3 + 2]);
  EXPECT_EQ (254, image.data[3 * 3 + 2]);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, PointCloudImageExtractorFromRGBField)
{
  using PointT = PointXYZRGB;
  PointCloud<PointT> cloud;
  cloud.width = 2;
  cloud.height = 2;
  cloud.is_dense = true;
  cloud.resize (cloud.width * cloud.height);
  for (auto &point : cloud.points)
  {
    point.r =   0;
    point.g = 127;
    point.b = 254;
  }
  pcl::PCLImage image;
  PointCloudImageExtractorFromRGBField<PointT> pcie;

  ASSERT_TRUE (pcie.extract (cloud, image));

  EXPECT_EQ ("rgb8", image.encoding);
  EXPECT_EQ (cloud.width, image.width);
  EXPECT_EQ (cloud.height, image.height);

  EXPECT_EQ (  0, image.data[0 * 3 + 0]);
  EXPECT_EQ (  0, image.data[1 * 3 + 0]);
  EXPECT_EQ (  0, image.data[2 * 3 + 0]);
  EXPECT_EQ (  0, image.data[3 * 3 + 0]);
  EXPECT_EQ (127, image.data[0 * 3 + 1]);
  EXPECT_EQ (127, image.data[1 * 3 + 1]);
  EXPECT_EQ (127, image.data[2 * 3 + 1]);
  EXPECT_EQ (127, image.data[3 * 3 + 1]);
  EXPECT_EQ (254, image.data[0 * 3 + 2]);
  EXPECT_EQ (254, image.data[1 * 3 + 2]);
  EXPECT_EQ (254, image.data[2 * 3 + 2]);
  EXPECT_EQ (254, image.data[3 * 3 + 2]);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, PointCloudImageExtractorFromRGBAField)
{
  using PointT = PointXYZRGBA;
  PointCloud<PointT> cloud;
  cloud.width = 2;
  cloud.height = 2;
  cloud.is_dense = true;
  cloud.resize (cloud.width * cloud.height);
  for (auto &point : cloud.points)
  {
    point.r =   0;
    point.g = 127;
    point.b = 254;
    point.a = 100;
  }
  pcl::PCLImage image;
  PointCloudImageExtractorFromRGBField<PointT> pcie;

  ASSERT_TRUE (pcie.extract (cloud, image));

  EXPECT_EQ ("rgb8", image.encoding);
  EXPECT_EQ (cloud.width, image.width);
  EXPECT_EQ (cloud.height, image.height);

  EXPECT_EQ (  0, image.data[0 * 3 + 0]);
  EXPECT_EQ (  0, image.data[1 * 3 + 0]);
  EXPECT_EQ (  0, image.data[2 * 3 + 0]);
  EXPECT_EQ (  0, image.data[3 * 3 + 0]);
  EXPECT_EQ (127, image.data[0 * 3 + 1]);
  EXPECT_EQ (127, image.data[1 * 3 + 1]);
  EXPECT_EQ (127, image.data[2 * 3 + 1]);
  EXPECT_EQ (127, image.data[3 * 3 + 1]);
  EXPECT_EQ (254, image.data[0 * 3 + 2]);
  EXPECT_EQ (254, image.data[1 * 3 + 2]);
  EXPECT_EQ (254, image.data[2 * 3 + 2]);
  EXPECT_EQ (254, image.data[3 * 3 + 2]);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, PointCloudImageExtractorFromLabelFieldMono)
{
  using PointT = PointXYZL;
  PointCloud<PointT> cloud;
  cloud.width = 2;
  cloud.height = 2;
  cloud.is_dense = true;
  cloud.resize (cloud.width * cloud.height);
  for (std::size_t i = 0; i < cloud.size (); i++)
    cloud[i].label = i;

  pcl::PCLImage image;
  PointCloudImageExtractorFromLabelField<PointT> pcie;
  pcie.setColorMode (pcie.COLORS_MONO);

  ASSERT_TRUE (pcie.extract (cloud, image));
  auto* data = reinterpret_cast<unsigned short*> (&image.data[0]);

  EXPECT_EQ ("mono16", image.encoding);
  EXPECT_EQ (cloud.width, image.width);
  EXPECT_EQ (cloud.height, image.height);

  for (std::size_t i = 0; i < cloud.size (); i++)
    EXPECT_EQ (i, data[i]);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, PointCloudImageExtractorFromLabelFieldRGB)
{
  using PointT = PointXYZL;
  PointCloud<PointT> cloud;
  cloud.width = 2;
  cloud.height = 2;
  cloud.is_dense = true;
  cloud.resize (cloud.width * cloud.height);
  for (std::size_t i = 0; i < cloud.size (); i++)
    cloud[i].label = i % 2;

  pcl::PCLImage image;
  PointCloudImageExtractorFromLabelField<PointT> pcie;
  pcie.setColorMode (pcie.COLORS_RGB_RANDOM);

  ASSERT_TRUE (pcie.extract (cloud, image));

  EXPECT_EQ ("rgb8", image.encoding);
  EXPECT_EQ (cloud.width, image.width);
  EXPECT_EQ (cloud.height, image.height);

  // Make sure same labels got the same random color
  std::uint8_t r0 = image.data[0 * 3 + 0];
  std::uint8_t g0 = image.data[0 * 3 + 1];
  std::uint8_t b0 = image.data[0 * 3 + 2];
  std::uint8_t r1 = image.data[1 * 3 + 0];
  std::uint8_t g1 = image.data[1 * 3 + 1];
  std::uint8_t b1 = image.data[1 * 3 + 2];

  EXPECT_EQ (r0, image.data[2 * 3 + 0]);
  EXPECT_EQ (g0, image.data[2 * 3 + 1]);
  EXPECT_EQ (b0, image.data[2 * 3 + 2]);
  EXPECT_EQ (r1, image.data[3 * 3 + 0]);
  EXPECT_EQ (g1, image.data[3 * 3 + 1]);
  EXPECT_EQ (b1, image.data[3 * 3 + 2]);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, PointCloudImageExtractorFromLabelFieldGlasbey)
{
  using PointT = PointXYZL;
  PointCloud<PointT> cloud;
  cloud.width = 2;
  cloud.height = 2;
  cloud.is_dense = true;
  cloud.resize (cloud.width * cloud.height);
  for (std::size_t i = 0; i < cloud.size (); i++)
    cloud[i].label = i % 2;

  pcl::PCLImage image;
  PointCloudImageExtractorFromLabelField<PointT> pcie;
  pcie.setColorMode (pcie.COLORS_RGB_GLASBEY);

  ASSERT_TRUE (pcie.extract (cloud, image));

  EXPECT_EQ ("rgb8", image.encoding);
  EXPECT_EQ (cloud.width, image.width);
  EXPECT_EQ (cloud.height, image.height);

  // Fill in different labels and extract another image
  for (std::size_t i = 0; i < cloud.size (); i++)
    cloud[i].label = i % 2 + 10;
  pcl::PCLImage image2;
  ASSERT_TRUE (pcie.extract (cloud, image2));

  // The first label should get the first Glasbey color
  EXPECT_EQ (GlasbeyLUT::data ()[0], image.data[0]);
  EXPECT_EQ (GlasbeyLUT::data ()[1], image.data[1]);
  EXPECT_EQ (GlasbeyLUT::data ()[2], image.data[2]);
  // Make sure the colors are the same
  for (std::size_t i = 0; i < 2 * 2 * 3; ++i)
    EXPECT_EQ (image2.data[i], image.data[i]);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, PointCloudImageExtractorFromZField)
{
  using PointT = PointXYZL;
  PointCloud<PointT> cloud;
  cloud.width = 2;
  cloud.height = 2;
  cloud.is_dense = true;
  cloud.resize (cloud.width * cloud.height);
  for (std::size_t i = 0; i < cloud.size (); i++)
    cloud[i].z = 1.0 + i;

  pcl::PCLImage image;
  PointCloudImageExtractorFromZField<PointT> pcie;

  ASSERT_TRUE (pcie.extract (cloud, image));
  auto* data = reinterpret_cast<unsigned short*> (&image.data[0]);

  EXPECT_EQ ("mono16", image.encoding);
  EXPECT_EQ (cloud.width, image.width);
  EXPECT_EQ (cloud.height, image.height);

  // by default Z field extractor scales with factor 10000
  for (std::size_t i = 0; i < cloud.size (); i++)
    EXPECT_EQ (10000 * (i + 1), data[i]);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, PointCloudImageExtractorFromCurvatureField)
{
  using PointT = PointNormal;
  PointCloud<PointT> cloud;
  cloud.width = 2;
  cloud.height = 2;
  cloud.is_dense = true;
  cloud.resize (cloud.width * cloud.height);

  cloud[0].curvature = 1.0;
  cloud[1].curvature = 2.0;
  cloud[2].curvature = 1.0;
  cloud[3].curvature = 2.0;

  pcl::PCLImage image;
  PointCloudImageExtractorFromCurvatureField<PointT> pcie;

  ASSERT_TRUE (pcie.extract (cloud, image));
  auto* data = reinterpret_cast<unsigned short*> (&image.data[0]);

  EXPECT_EQ ("mono16", image.encoding);
  EXPECT_EQ (cloud.width, image.width);
  EXPECT_EQ (cloud.height, image.height);

  // by default Curvature field extractor scales to full range of unsigned short
  EXPECT_EQ (0, data[0]);
  EXPECT_EQ (std::numeric_limits<unsigned short>::max () , data[1]);
  EXPECT_EQ (0, data[2]);
  EXPECT_EQ (std::numeric_limits<unsigned short>::max () , data[3]);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, PointCloudImageExtractorFromIntensityField)
{
  using PointT = PointXYZI;
  PointCloud<PointT> cloud;
  cloud.width = 2;
  cloud.height = 2;
  cloud.is_dense = true;
  cloud.resize (cloud.width * cloud.height);

  cloud[0].intensity = 10.0;
  cloud[1].intensity = 23.3;
  cloud[2].intensity = 28.9;
  cloud[3].intensity = 40.0;

  pcl::PCLImage image;
  PointCloudImageExtractorFromIntensityField<PointT> pcie;

  ASSERT_TRUE (pcie.extract (cloud, image));
  auto* data = reinterpret_cast<unsigned short*> (&image.data[0]);

  EXPECT_EQ ("mono16", image.encoding);
  EXPECT_EQ (cloud.width, image.width);
  EXPECT_EQ (cloud.height, image.height);

  // by default Intensity field extractor does not apply scaling
  for (std::size_t i = 0; i < cloud.size (); i++)
    EXPECT_EQ (static_cast<unsigned short> (cloud[i].intensity), data[i]);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, PointCloudImageExtractorBadInput)
{
  using PointT = PointXY; // none of point cloud image extractors support this
  PointCloud<PointT> cloud;
  cloud.width = 2;
  cloud.height = 2;
  cloud.is_dense = true;
  cloud.resize (cloud.width * cloud.height);

  pcl::PCLImage image;
  {
    PointCloudImageExtractorFromNormalField<PointT> pcie;
    EXPECT_FALSE (pcie.extract (cloud, image));
  }
  {
    PointCloudImageExtractorFromRGBField<PointT> pcie;
    EXPECT_FALSE (pcie.extract (cloud, image));
  }
  {
    PointCloudImageExtractorFromLabelField<PointT> pcie;
    EXPECT_FALSE (pcie.extract (cloud, image));
  }
  {
    PointCloudImageExtractorFromZField<PointT> pcie;
    EXPECT_FALSE (pcie.extract (cloud, image));
  }
  {
    PointCloudImageExtractorFromCurvatureField<PointT> pcie;
    EXPECT_FALSE (pcie.extract (cloud, image));
  }
  {
    PointCloudImageExtractorFromIntensityField<PointT> pcie;
    EXPECT_FALSE (pcie.extract (cloud, image));
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, PointCloudImageExtractorBlackNaNs)
{
  using PointT = PointNormal;
  PointCloud<PointT> cloud;
  cloud.width = 2;
  cloud.height = 2;
  cloud.is_dense = false;
  cloud.resize (cloud.width * cloud.height);

  cloud[0].curvature = 1.0;
  cloud[1].curvature = 2.0;
  cloud[2].curvature = 1.0;
  cloud[3].curvature = 2.0;
  cloud[3].z = std::numeric_limits<float>::quiet_NaN ();

  pcl::PCLImage image;

  PointCloudImageExtractorFromCurvatureField<PointT> pcie;

  ASSERT_TRUE (pcie.extract (cloud, image));

  {
    auto* data = reinterpret_cast<unsigned short*> (&image.data[0]);
    EXPECT_EQ (std::numeric_limits<unsigned short>::max (), data[3]);
  }

  pcie.setPaintNaNsWithBlack (true);

  ASSERT_TRUE (pcie.extract (cloud, image));

  {
    auto* data = reinterpret_cast<unsigned short*> (&image.data[0]);
    EXPECT_EQ (0, data[3]);
  }
}

int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}

