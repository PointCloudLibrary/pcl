/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
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
 */

#include <pcl/test/gtest.h>
#include <pcl/pcl_tests.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>

using namespace pcl;

using CloudXYZRGBA = PointCloud<PointXYZRGBA>;
using CloudXYZRGB = PointCloud<PointXYZRGB>;
using CloudXYZRGBNormal = PointCloud<PointXYZRGBNormal>;
using CloudXYZ = PointCloud<PointXYZ>;

PointXYZRGBA pt_xyz_rgba, pt_xyz_rgba2;
PointXYZRGB pt_xyz_rgb;
PointXYZ pt_xyz;

///////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, concatenateFields)
{
  bool status = isSamePointType<PointXYZ, PointXYZ> ();
  EXPECT_TRUE (status);
  status = isSamePointType<PointXYZ, PointXY> ();
  EXPECT_FALSE (status);
  status = isSamePointType<PointXY, PointXYZ> ();
  EXPECT_FALSE (status);
  status = isSamePointType<PointNormal, PointNormal> ();
  EXPECT_TRUE (status);
  status = isSamePointType<PointNormal, PointXYZRGBNormal> ();
  EXPECT_FALSE (status);
  status = isSamePointType<PointXYZRGB, PointXYZRGB> ();
  EXPECT_TRUE (status);
  
  // Even though it's the "same" type, rgb != rgba
  status = isSamePointType<PointXYZRGB, PointXYZRGBA> ();
  EXPECT_FALSE (status);
  status = isSamePointType<PointXYZRGBA, PointXYZRGB> ();
  EXPECT_FALSE (status);
}

///////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, copyPointCloud)
{
  CloudXYZRGBA cloud_xyz_rgba;
  cloud_xyz_rgba.push_back (pt_xyz_rgba);
  cloud_xyz_rgba.push_back (pt_xyz_rgba);
  cloud_xyz_rgba.push_back (pt_xyz_rgba);
  cloud_xyz_rgba.push_back (pt_xyz_rgba);
  cloud_xyz_rgba.push_back (pt_xyz_rgba);

  CloudXYZRGBNormal cloud_xyz_rgb_normal;
  pcl::copyPointCloud (cloud_xyz_rgba, cloud_xyz_rgb_normal);
  ASSERT_METADATA_EQ (cloud_xyz_rgba, cloud_xyz_rgb_normal);
  ASSERT_EQ (int (cloud_xyz_rgb_normal.size ()), 5);
  for (int i = 0; i < 5; ++i)
  {
    EXPECT_XYZ_EQ (cloud_xyz_rgba[i], cloud_xyz_rgb_normal[i]);
    EXPECT_RGB_EQ (cloud_xyz_rgba[i], cloud_xyz_rgb_normal[i]);
    EXPECT_EQ (cloud_xyz_rgba[i].rgba, cloud_xyz_rgb_normal[i].rgba);
  }

  Indices indices;
  indices.push_back (0); indices.push_back (1); 
  pcl::copyPointCloud (cloud_xyz_rgba, indices, cloud_xyz_rgb_normal);
  ASSERT_EQ (int (cloud_xyz_rgb_normal.size ()), 2);
  for (int i = 0; i < 2; ++i)
  {
    EXPECT_XYZ_EQ (cloud_xyz_rgba[i], cloud_xyz_rgb_normal[i]);
    EXPECT_RGB_EQ (cloud_xyz_rgba[i], cloud_xyz_rgb_normal[i]);
    EXPECT_EQ (cloud_xyz_rgba[i].rgba, cloud_xyz_rgb_normal[i].rgba);
  }

  IndicesAllocator< Eigen::aligned_allocator<pcl::index_t> > indices_aligned;
  indices_aligned.push_back (1); indices_aligned.push_back (2); indices_aligned.push_back (3); 
  pcl::copyPointCloud (cloud_xyz_rgba, indices_aligned, cloud_xyz_rgb_normal);
  ASSERT_EQ (int (cloud_xyz_rgb_normal.size ()), 3);
  for (int i = 0; i < 3; ++i)
  {
    EXPECT_XYZ_EQ (cloud_xyz_rgba[i], cloud_xyz_rgb_normal[i]);
    EXPECT_RGB_EQ (cloud_xyz_rgba[i], cloud_xyz_rgb_normal[i]);
    EXPECT_EQ (cloud_xyz_rgba[i].rgba, cloud_xyz_rgb_normal[i].rgba);
  }

  PointIndices pindices;
  pindices.indices.push_back (0); pindices.indices.push_back (2); pindices.indices.push_back (4);
  ASSERT_EQ (int (cloud_xyz_rgb_normal.size ()), 3);
  for (int i = 0; i < 3; ++i)
  {
    EXPECT_XYZ_EQ (cloud_xyz_rgba[i], cloud_xyz_rgb_normal[i]);
    EXPECT_RGB_EQ (cloud_xyz_rgba[i], cloud_xyz_rgb_normal[i]);
    EXPECT_EQ (cloud_xyz_rgba[i].rgba, cloud_xyz_rgb_normal[i].rgba);
  }
}

///////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, concatenatePointCloud2)
{
  CloudXYZRGBA cloud_xyz_rgba;
  cloud_xyz_rgba.push_back (pt_xyz_rgba);
  cloud_xyz_rgba.push_back (pt_xyz_rgba);
  cloud_xyz_rgba.push_back (pt_xyz_rgba);
  cloud_xyz_rgba.push_back (pt_xyz_rgba);
  cloud_xyz_rgba.push_back (pt_xyz_rgba);

  CloudXYZRGBA cloud_xyz_rgba2;
  cloud_xyz_rgba2.push_back (pt_xyz_rgba2);
  cloud_xyz_rgba2.push_back (pt_xyz_rgba2);

  pcl::PCLPointCloud2 cloud1, cloud2, cloud_out, cloud_out2, cloud_out3, cloud_out4;
  pcl::toPCLPointCloud2 (cloud_xyz_rgba, cloud1);
  pcl::toPCLPointCloud2 (cloud_xyz_rgba2, cloud2);

  // Regular
  EXPECT_TRUE (pcl::concatenate (cloud1, cloud2, cloud_out));

  CloudXYZRGBA cloud_all;
  pcl::fromPCLPointCloud2 (cloud_out, cloud_all);

  EXPECT_EQ (cloud_all.size (), cloud_xyz_rgba.size () + cloud_xyz_rgba2.size ());
  for (int i = 0; i < int (cloud_xyz_rgba.size ()); ++i)
  {
    EXPECT_XYZ_EQ (cloud_all[i], cloud_xyz_rgba[i]);
    EXPECT_RGBA_EQ (cloud_all[i], cloud_xyz_rgba[i]);
  }
  for (int i = 0; i < int (cloud_xyz_rgba2.size ()); ++i)
  {
    EXPECT_XYZ_EQ (cloud_all[cloud_xyz_rgba.size () + i], cloud_xyz_rgba2[i]);
    EXPECT_RGBA_EQ (cloud_all[cloud_xyz_rgba.size () + i], cloud_xyz_rgba2[i]);
  }

  // RGB != RGBA
  CloudXYZRGB cloud_xyz_rgb;
  cloud_xyz_rgb.push_back (pt_xyz_rgb);
  cloud_xyz_rgb.push_back (pt_xyz_rgb);

  pcl::toPCLPointCloud2 (cloud_xyz_rgb, cloud2);
  EXPECT_TRUE (pcl::concatenate (cloud1, cloud2, cloud_out2));

  pcl::fromPCLPointCloud2 (cloud_out2, cloud_all);

  EXPECT_EQ (cloud_all.size (), cloud_xyz_rgba.size () + cloud_xyz_rgba2.size ());
  for (int i = 0; i < int (cloud_xyz_rgba.size ()); ++i)
  {
    EXPECT_XYZ_EQ (cloud_all[i], cloud_xyz_rgba[i]);
    EXPECT_RGBA_EQ (cloud_all[i], cloud_xyz_rgba[i]);
  }
  for (int i = 0; i < int (cloud_xyz_rgb.size ()); ++i)
  {
    EXPECT_XYZ_EQ (cloud_all[cloud_xyz_rgba.size () + i], cloud_xyz_rgb[i]);
    EXPECT_RGBA_EQ (cloud_all[cloud_xyz_rgba.size () + i], cloud_xyz_rgb[i]);
  }

  // _ vs regular
  int rgb_idx = pcl::getFieldIndex (cloud1, "rgba");
  cloud1.fields[rgb_idx].name = "_";
  EXPECT_FALSE (pcl::concatenate (cloud1, cloud2, cloud_out3));
  cloud1.fields[rgb_idx].name = "rgba";

  // regular vs _
  rgb_idx = pcl::getFieldIndex (cloud2, "rgb");
  cloud2.fields[rgb_idx].name = "_";
  EXPECT_FALSE (pcl::concatenate (cloud1, cloud2, cloud_out4));

  // _ vs _
  rgb_idx = pcl::getFieldIndex (cloud1, "rgba");
  cloud1.fields[rgb_idx].name = "_";
  pcl::toPCLPointCloud2 (cloud_xyz_rgb, cloud2);
  rgb_idx = pcl::getFieldIndex (cloud2, "rgb");
  cloud2.fields[rgb_idx].name = "_";

  EXPECT_TRUE (pcl::concatenate (cloud1, cloud2, cloud_out3));

  pcl::fromPCLPointCloud2 (cloud_out3, cloud_all);

  EXPECT_EQ (cloud_all.size (), cloud_xyz_rgba.size () + cloud_xyz_rgb.size ());
  for (int i = 0; i < int (cloud_xyz_rgba.size ()); ++i)
  {
    EXPECT_XYZ_EQ (cloud_all[i], cloud_xyz_rgba[i]);
    // Data doesn't get modified
    EXPECT_RGBA_EQ (cloud_all[i], cloud_xyz_rgba[i]);
  }
  for (int i = 0; i < int (cloud_xyz_rgb.size ()); ++i)
  {
    EXPECT_XYZ_EQ (cloud_all[cloud_xyz_rgba.size () + i], cloud_xyz_rgb[i]);
    EXPECT_RGBA_EQ (cloud_all[cloud_xyz_rgba.size () + i], cloud_xyz_rgb[i]);
  }
  cloud1.fields[rgb_idx].name = "rgba";
  cloud2.fields[rgb_idx].name = "rgba";

  // _ vs regular
  rgb_idx = pcl::getFieldIndex (cloud1, "rgba");

  cloud1.fields[rgb_idx].name = "_";
  pcl::toPCLPointCloud2 (cloud_xyz_rgb, cloud2);
  EXPECT_FALSE (pcl::concatenate (cloud2, cloud1, cloud_out3));
  cloud1.fields[rgb_idx].name = "rgba";

  // regular vs _
  rgb_idx = pcl::getFieldIndex (cloud2, "rgb");
  cloud2.fields[rgb_idx].name = "_";
  EXPECT_FALSE (pcl::concatenate (cloud2, cloud1, cloud_out4));
}

TEST (PCL, CopyPointCloudWithIndicesAndRGBToRGBA)
{
  CloudXYZRGB cloud_xyz_rgb;
  CloudXYZRGBA cloud_xyz_rgba (5, 1, pt_xyz_rgba);

  Indices indices;
  indices.push_back (2);
  indices.push_back (3);

  pcl::copyPointCloud (cloud_xyz_rgba, indices, cloud_xyz_rgb);

  EXPECT_EQ (indices.size (), cloud_xyz_rgb.size ());
  for (std::size_t i = 0; i < indices.size (); ++i)
  {
    EXPECT_XYZ_EQ (cloud_xyz_rgb[i], cloud_xyz_rgba[indices[i]]);
    EXPECT_EQ (cloud_xyz_rgb[i].rgba, cloud_xyz_rgba[indices[i]].rgba);
  }
}

TEST (PCL, CopyPointCloudWithSameTypes)
{
  CloudXYZ cloud_in (5, 1, pt_xyz);
  CloudXYZ cloud_in_empty;
  CloudXYZ cloud_out;

  pcl::copyPointCloud (cloud_in, cloud_out);

  ASSERT_EQ (cloud_in.size (), cloud_out.size ());
  for (std::size_t i = 0; i < cloud_out.size (); ++i)
    EXPECT_XYZ_EQ (cloud_in[i], cloud_out[i]);

  pcl::copyPointCloud (cloud_in_empty, cloud_out);

  ASSERT_EQ (0, cloud_out.size ());
}

/* ---[ */
int
main (int argc, char** argv)
{
  pt_xyz_rgba.x = 5;
  pt_xyz_rgba.y = 2;
  pt_xyz_rgba.z = 3;
  pt_xyz_rgba.r = 112;
  pt_xyz_rgba.g = 100;
  pt_xyz_rgba.b = 255;
  pt_xyz_rgba.a = 135;
  
  pt_xyz_rgba2.x = 4;
  pt_xyz_rgba2.y = 1;
  pt_xyz_rgba2.z = 5;
  pt_xyz_rgba2.r = 11;
  pt_xyz_rgba2.g = 10;
  pt_xyz_rgba2.b = 0;
  pt_xyz_rgba2.a = 255;

  pt_xyz_rgb.x = 4;
  pt_xyz_rgb.y = 1;
  pt_xyz_rgb.z = 5;
  pt_xyz_rgb.r = 11;
  pt_xyz_rgb.g = 10;
  pt_xyz_rgb.b = 0;
  pt_xyz_rgb.a = 255;

  pt_xyz.x = 4;
  pt_xyz.y = 1;
  pt_xyz.z = 5;

  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
