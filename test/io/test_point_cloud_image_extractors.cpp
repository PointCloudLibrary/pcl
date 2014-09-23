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

#include <gtest/gtest.h>
#include <pcl/point_types.h>
#include <pcl/io/point_cloud_image_extractors.h>

using namespace pcl;
using namespace pcl::io;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, PointCloudImageExtractorFromNormalField)
{
  typedef PointNormal PointT;
  PointCloud<PointT> cloud;
  cloud.width = 2;
  cloud.height = 2;
  cloud.is_dense = true;
  cloud.points.resize (cloud.width * cloud.height);
  for (size_t i = 0; i < cloud.points.size (); i++)
  {
    cloud.points[i].normal_x = -1.0;
    cloud.points[i].normal_y =  0.0;
    cloud.points[i].normal_z =  1.0;
  }
  pcl::PCLImage image;
  PointCloudImageExtractorFromNormalField<PointT> pcie;

  ASSERT_TRUE (pcie.extract(cloud, image));

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
  typedef PointXYZRGB PointT;
  PointCloud<PointT> cloud;
  cloud.width = 2;
  cloud.height = 2;
  cloud.is_dense = true;
  cloud.points.resize (cloud.width * cloud.height);
  for (size_t i = 0; i < cloud.points.size (); i++)
  {
    cloud.points[i].r =   0;
    cloud.points[i].g = 127;
    cloud.points[i].b = 254;
  }
  pcl::PCLImage image;
  PointCloudImageExtractorFromRGBField<PointT> pcie;

  ASSERT_TRUE (pcie.extract(cloud, image));

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
  typedef PointXYZRGBA PointT;
  PointCloud<PointT> cloud;
  cloud.width = 2;
  cloud.height = 2;
  cloud.is_dense = true;
  cloud.points.resize (cloud.width * cloud.height);
  for (size_t i = 0; i < cloud.points.size (); i++)
  {
    cloud.points[i].r =   0;
    cloud.points[i].g = 127;
    cloud.points[i].b = 254;
    cloud.points[i].a = 100;
  }
  pcl::PCLImage image;
  PointCloudImageExtractorFromRGBField<PointT> pcie;

  ASSERT_TRUE (pcie.extract(cloud, image));

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
  typedef PointXYZL PointT;
  PointCloud<PointT> cloud;
  cloud.width = 2;
  cloud.height = 2;
  cloud.is_dense = true;
  cloud.points.resize (cloud.width * cloud.height);
  for (size_t i = 0; i < cloud.points.size (); i++)
    cloud.points[i].label = i;

  pcl::PCLImage image;
  PointCloudImageExtractorFromLabelField<PointT> pcie;
  pcie.setColorMode (pcie.COLORS_MONO);

  ASSERT_TRUE (pcie.extract(cloud, image));
  unsigned short* data = reinterpret_cast<unsigned short*> (&image.data[0]);

  EXPECT_EQ ("mono16", image.encoding);
  EXPECT_EQ (cloud.width, image.width);
  EXPECT_EQ (cloud.height, image.height);

  for (size_t i = 0; i < cloud.points.size (); i++)
    EXPECT_EQ (i, data[i]);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, PointCloudImageExtractorFromLabelFieldRGB)
{
  typedef PointXYZL PointT;
  PointCloud<PointT> cloud;
  cloud.width = 2;
  cloud.height = 2;
  cloud.is_dense = true;
  cloud.points.resize (cloud.width * cloud.height);
  for (size_t i = 0; i < cloud.points.size (); i++)
    cloud.points[i].label = i % 2;

  pcl::PCLImage image;
  PointCloudImageExtractorFromLabelField<PointT> pcie;
  pcie.setColorMode (pcie.COLORS_RGB_RANDOM);

  ASSERT_TRUE (pcie.extract(cloud, image));

  EXPECT_EQ ("rgb8", image.encoding);
  EXPECT_EQ (cloud.width, image.width);
  EXPECT_EQ (cloud.height, image.height);

  // Make sure same labels got the same random color
  uint8_t r0 = image.data[0 * 3 + 0];
  uint8_t g0 = image.data[0 * 3 + 1];
  uint8_t b0 = image.data[0 * 3 + 2];
  uint8_t r1 = image.data[1 * 3 + 0];
  uint8_t g1 = image.data[1 * 3 + 1];
  uint8_t b1 = image.data[1 * 3 + 2];

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
  typedef PointXYZL PointT;
  PointCloud<PointT> cloud;
  cloud.width = 2;
  cloud.height = 2;
  cloud.is_dense = true;
  cloud.points.resize (cloud.width * cloud.height);
  for (size_t i = 0; i < cloud.points.size (); i++)
    cloud.points[i].label = i % 2;

  pcl::PCLImage image;
  PointCloudImageExtractorFromLabelField<PointT> pcie;
  pcie.setColorMode (pcie.COLORS_RGB_GLASBEY);

  ASSERT_TRUE (pcie.extract(cloud, image));

  EXPECT_EQ ("rgb8", image.encoding);
  EXPECT_EQ (cloud.width, image.width);
  EXPECT_EQ (cloud.height, image.height);

  // Fill in different labels and extract another image
  for (size_t i = 0; i < cloud.points.size (); i++)
    cloud.points[i].label = i % 2 + 10;
  pcl::PCLImage image2;
  ASSERT_TRUE (pcie.extract(cloud, image2));

  // The first color should be pure blue
  EXPECT_EQ (0, image.data[0]);
  EXPECT_EQ (0, image.data[1]);
  EXPECT_EQ (255, image.data[2]);
  // Make sure the colors are the same
  for (size_t i = 0; i < 2 * 2 * 3; ++i)
    EXPECT_EQ (image2.data[i], image.data[i]);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, PointCloudImageExtractorFromZField)
{
  typedef PointXYZL PointT;
  PointCloud<PointT> cloud;
  cloud.width = 2;
  cloud.height = 2;
  cloud.is_dense = true;
  cloud.points.resize (cloud.width * cloud.height);
  for (size_t i = 0; i < cloud.points.size (); i++)
    cloud.points[i].z = 1.0 + i;

  pcl::PCLImage image;
  PointCloudImageExtractorFromZField<PointT> pcie;

  ASSERT_TRUE (pcie.extract(cloud, image));
  unsigned short* data = reinterpret_cast<unsigned short*> (&image.data[0]);

  EXPECT_EQ ("mono16", image.encoding);
  EXPECT_EQ (cloud.width, image.width);
  EXPECT_EQ (cloud.height, image.height);

  // by default Z field extractor scales with factor 10000
  for (size_t i = 0; i < cloud.points.size (); i++)
    EXPECT_EQ (10000 * (i + 1), data[i]);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, PointCloudImageExtractorFromCurvatureField)
{
  typedef PointNormal PointT;
  PointCloud<PointT> cloud;
  cloud.width = 2;
  cloud.height = 2;
  cloud.is_dense = true;
  cloud.points.resize (cloud.width * cloud.height);

  cloud.points[0].curvature = 1.0;
  cloud.points[1].curvature = 2.0;
  cloud.points[2].curvature = 1.0;
  cloud.points[3].curvature = 2.0;

  pcl::PCLImage image;
  PointCloudImageExtractorFromCurvatureField<PointT> pcie;

  ASSERT_TRUE (pcie.extract(cloud, image));
  unsigned short* data = reinterpret_cast<unsigned short*> (&image.data[0]);

  EXPECT_EQ ("mono16", image.encoding);
  EXPECT_EQ (cloud.width, image.width);
  EXPECT_EQ (cloud.height, image.height);

  // by default Curvature field extractor scales to full range of unsigned short
  EXPECT_EQ (0, data[0]);
  EXPECT_EQ (std::numeric_limits<unsigned short>::max() , data[1]);
  EXPECT_EQ (0, data[2]);
  EXPECT_EQ (std::numeric_limits<unsigned short>::max() , data[3]);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, PointCloudImageExtractorFromIntensityField)
{
  typedef PointXYZI PointT;
  PointCloud<PointT> cloud;
  cloud.width = 2;
  cloud.height = 2;
  cloud.is_dense = true;
  cloud.points.resize (cloud.width * cloud.height);

  cloud.points[0].intensity = 10.0;
  cloud.points[1].intensity = 23.3;
  cloud.points[2].intensity = 28.9;
  cloud.points[3].intensity = 40.0;

  pcl::PCLImage image;
  PointCloudImageExtractorFromIntensityField<PointT> pcie;

  ASSERT_TRUE (pcie.extract(cloud, image));
  unsigned short* data = reinterpret_cast<unsigned short*> (&image.data[0]);

  EXPECT_EQ ("mono16", image.encoding);
  EXPECT_EQ (cloud.width, image.width);
  EXPECT_EQ (cloud.height, image.height);

  // by default Intensity field extractor does not apply scaling
  for (size_t i = 0; i < cloud.points.size (); i++)
    EXPECT_EQ (static_cast<unsigned short>(cloud.points[i].intensity), data[i]);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, PointCloudImageExtractorBadInput)
{
  typedef PointXY PointT; // none of point cloud image extractors support this
  PointCloud<PointT> cloud;
  cloud.width = 2;
  cloud.height = 2;
  cloud.is_dense = true;
  cloud.points.resize (cloud.width * cloud.height);

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
  typedef PointNormal PointT;
  PointCloud<PointT> cloud;
  cloud.width = 2;
  cloud.height = 2;
  cloud.is_dense = false;
  cloud.points.resize (cloud.width * cloud.height);

  cloud.points[0].curvature = 1.0;
  cloud.points[1].curvature = 2.0;
  cloud.points[2].curvature = 1.0;
  cloud.points[3].curvature = 2.0;
  cloud.points[3].z = std::numeric_limits<float>::quiet_NaN ();

  pcl::PCLImage image;

  PointCloudImageExtractorFromCurvatureField<PointT> pcie;

  ASSERT_TRUE (pcie.extract(cloud, image));

  {
    unsigned short* data = reinterpret_cast<unsigned short*> (&image.data[0]);
    EXPECT_EQ (std::numeric_limits<unsigned short>::max(), data[3]);
  }

  pcie.setPaintNaNsWithBlack (true);

  ASSERT_TRUE (pcie.extract(cloud, image));

  {
    unsigned short* data = reinterpret_cast<unsigned short*> (&image.data[0]);
    EXPECT_EQ (0, data[3]);
  }
}

int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}

