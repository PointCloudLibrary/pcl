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

#include <gtest/gtest.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>

using namespace pcl;
using namespace std;

typedef PointCloud <PointXYZRGBA>      CloudXYZRGBA;
typedef PointCloud <PointXYZRGB>       CloudXYZRGB;
typedef PointCloud <PointXYZRGBNormal> CloudXYZRGBNormal;

PointXYZRGBA pt_xyz_rgba, pt_xyz_rgba2;
PointXYZRGB pt_xyz_rgb;

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
  EXPECT_EQ (int (cloud_xyz_rgb_normal.size ()), 5);
  for (int i = 0; i < 5; ++i)
  {
    EXPECT_FLOAT_EQ (cloud_xyz_rgba[i].x, cloud_xyz_rgb_normal[i].x);
    EXPECT_FLOAT_EQ (cloud_xyz_rgba[i].y, cloud_xyz_rgb_normal[i].y);
    EXPECT_FLOAT_EQ (cloud_xyz_rgba[i].z, cloud_xyz_rgb_normal[i].z);
    EXPECT_FLOAT_EQ (cloud_xyz_rgba[i].r, cloud_xyz_rgb_normal[i].r);
    EXPECT_FLOAT_EQ (cloud_xyz_rgba[i].g, cloud_xyz_rgb_normal[i].g);
    EXPECT_FLOAT_EQ (cloud_xyz_rgba[i].b, cloud_xyz_rgb_normal[i].b);
    EXPECT_EQ (cloud_xyz_rgba[i].rgba, cloud_xyz_rgb_normal[i].rgba);
  }

  vector<int> indices;
  indices.push_back (0); indices.push_back (1); 
  pcl::copyPointCloud (cloud_xyz_rgba, indices, cloud_xyz_rgb_normal);
  EXPECT_EQ (int (cloud_xyz_rgb_normal.size ()), 2);
  for (int i = 0; i < 2; ++i)
  {
    EXPECT_FLOAT_EQ (cloud_xyz_rgba[i].x, cloud_xyz_rgb_normal[i].x);
    EXPECT_FLOAT_EQ (cloud_xyz_rgba[i].y, cloud_xyz_rgb_normal[i].y);
    EXPECT_FLOAT_EQ (cloud_xyz_rgba[i].z, cloud_xyz_rgb_normal[i].z);
    EXPECT_FLOAT_EQ (cloud_xyz_rgba[i].r, cloud_xyz_rgb_normal[i].r);
    EXPECT_FLOAT_EQ (cloud_xyz_rgba[i].g, cloud_xyz_rgb_normal[i].g);
    EXPECT_FLOAT_EQ (cloud_xyz_rgba[i].b, cloud_xyz_rgb_normal[i].b);
    EXPECT_EQ (cloud_xyz_rgba[i].rgba, cloud_xyz_rgb_normal[i].rgba);
  }

  vector<int, Eigen::aligned_allocator<int> > indices_aligned;
  indices_aligned.push_back (1); indices_aligned.push_back (2); indices_aligned.push_back (3); 
  pcl::copyPointCloud (cloud_xyz_rgba, indices_aligned, cloud_xyz_rgb_normal);
  EXPECT_EQ (int (cloud_xyz_rgb_normal.size ()), 3);
  for (int i = 0; i < 3; ++i)
  {
    EXPECT_FLOAT_EQ (cloud_xyz_rgba[i].x, cloud_xyz_rgb_normal[i].x);
    EXPECT_FLOAT_EQ (cloud_xyz_rgba[i].y, cloud_xyz_rgb_normal[i].y);
    EXPECT_FLOAT_EQ (cloud_xyz_rgba[i].z, cloud_xyz_rgb_normal[i].z);
    EXPECT_FLOAT_EQ (cloud_xyz_rgba[i].r, cloud_xyz_rgb_normal[i].r);
    EXPECT_FLOAT_EQ (cloud_xyz_rgba[i].g, cloud_xyz_rgb_normal[i].g);
    EXPECT_FLOAT_EQ (cloud_xyz_rgba[i].b, cloud_xyz_rgb_normal[i].b);
    EXPECT_EQ (cloud_xyz_rgba[i].rgba, cloud_xyz_rgb_normal[i].rgba);
  }

  PointIndices pindices;
  pindices.indices.push_back (0); pindices.indices.push_back (2); pindices.indices.push_back (4);
  EXPECT_EQ (int (cloud_xyz_rgb_normal.size ()), 3);
  for (int i = 0; i < 3; ++i)
  {
    EXPECT_FLOAT_EQ (cloud_xyz_rgba[i].x, cloud_xyz_rgb_normal[i].x);
    EXPECT_FLOAT_EQ (cloud_xyz_rgba[i].y, cloud_xyz_rgb_normal[i].y);
    EXPECT_FLOAT_EQ (cloud_xyz_rgba[i].z, cloud_xyz_rgb_normal[i].z);
    EXPECT_FLOAT_EQ (cloud_xyz_rgba[i].r, cloud_xyz_rgb_normal[i].r);
    EXPECT_FLOAT_EQ (cloud_xyz_rgba[i].g, cloud_xyz_rgb_normal[i].g);
    EXPECT_FLOAT_EQ (cloud_xyz_rgba[i].b, cloud_xyz_rgb_normal[i].b);
    EXPECT_EQ (cloud_xyz_rgba[i].rgba, cloud_xyz_rgb_normal[i].rgba);
  }
}

///////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, concatenatePointCloud)
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
  pcl::toROSMsg (cloud_xyz_rgba, cloud1);
  pcl::toROSMsg (cloud_xyz_rgba2, cloud2);

  // Regular
  pcl::concatenatePointCloud (cloud1, cloud2, cloud_out);
  
  CloudXYZRGBA cloud_all;
  pcl::fromROSMsg (cloud_out, cloud_all);

  EXPECT_EQ (cloud_all.size (), cloud_xyz_rgba.size () + cloud_xyz_rgba2.size ());
  for (int i = 0; i < int (cloud_xyz_rgba.size ()); ++i)
  {
    EXPECT_FLOAT_EQ (cloud_all[i].x, cloud_xyz_rgba[i].x);
    EXPECT_FLOAT_EQ (cloud_all[i].y, cloud_xyz_rgba[i].y);
    EXPECT_FLOAT_EQ (cloud_all[i].z, cloud_xyz_rgba[i].z);
    EXPECT_FLOAT_EQ (cloud_all[i].r, cloud_xyz_rgba[i].r);
    EXPECT_FLOAT_EQ (cloud_all[i].g, cloud_xyz_rgba[i].g);
    EXPECT_FLOAT_EQ (cloud_all[i].b, cloud_xyz_rgba[i].b);
    EXPECT_EQ (cloud_all[i].rgba, cloud_xyz_rgba[i].rgba);
  }
  for (int i = 0; i < int (cloud_xyz_rgba2.size ()); ++i)
  {
    EXPECT_FLOAT_EQ (cloud_all[cloud_xyz_rgba.size () + i].x, cloud_xyz_rgba2[i].x);
    EXPECT_FLOAT_EQ (cloud_all[cloud_xyz_rgba.size () + i].y, cloud_xyz_rgba2[i].y);
    EXPECT_FLOAT_EQ (cloud_all[cloud_xyz_rgba.size () + i].z, cloud_xyz_rgba2[i].z);
    EXPECT_FLOAT_EQ (cloud_all[cloud_xyz_rgba.size () + i].r, cloud_xyz_rgba2[i].r);
    EXPECT_FLOAT_EQ (cloud_all[cloud_xyz_rgba.size () + i].g, cloud_xyz_rgba2[i].g);
    EXPECT_FLOAT_EQ (cloud_all[cloud_xyz_rgba.size () + i].b, cloud_xyz_rgba2[i].b);
    EXPECT_EQ (cloud_all[cloud_xyz_rgba.size () + i].rgba, cloud_xyz_rgba2[i].rgba);
  }

  // RGB != RGBA
  CloudXYZRGB cloud_xyz_rgb;
  cloud_xyz_rgb.push_back (pt_xyz_rgb);
  cloud_xyz_rgb.push_back (pt_xyz_rgb);

  pcl::toROSMsg (cloud_xyz_rgb, cloud2);
  pcl::concatenatePointCloud (cloud1, cloud2, cloud_out2);
  
  pcl::fromROSMsg (cloud_out2, cloud_all);

  EXPECT_EQ (cloud_all.size (), cloud_xyz_rgba.size () + cloud_xyz_rgba2.size ());
  for (int i = 0; i < int (cloud_xyz_rgba.size ()); ++i)
  {
    EXPECT_FLOAT_EQ (cloud_all[i].x, cloud_xyz_rgba[i].x);
    EXPECT_FLOAT_EQ (cloud_all[i].y, cloud_xyz_rgba[i].y);
    EXPECT_FLOAT_EQ (cloud_all[i].z, cloud_xyz_rgba[i].z);
    EXPECT_FLOAT_EQ (cloud_all[i].r, cloud_xyz_rgba[i].r);
    EXPECT_FLOAT_EQ (cloud_all[i].g, cloud_xyz_rgba[i].g);
    EXPECT_FLOAT_EQ (cloud_all[i].b, cloud_xyz_rgba[i].b);
    EXPECT_EQ (cloud_all[i].rgba, cloud_xyz_rgba[i].rgba);
  }
  for (int i = 0; i < int (cloud_xyz_rgb.size ()); ++i)
  {
    EXPECT_FLOAT_EQ (cloud_all[cloud_xyz_rgba.size () + i].x, cloud_xyz_rgb[i].x);
    EXPECT_FLOAT_EQ (cloud_all[cloud_xyz_rgba.size () + i].y, cloud_xyz_rgb[i].y);
    EXPECT_FLOAT_EQ (cloud_all[cloud_xyz_rgba.size () + i].z, cloud_xyz_rgb[i].z);
    EXPECT_FLOAT_EQ (cloud_all[cloud_xyz_rgba.size () + i].r, cloud_xyz_rgb[i].r);
    EXPECT_FLOAT_EQ (cloud_all[cloud_xyz_rgba.size () + i].g, cloud_xyz_rgb[i].g);
    EXPECT_FLOAT_EQ (cloud_all[cloud_xyz_rgba.size () + i].b, cloud_xyz_rgb[i].b);
    EXPECT_EQ (cloud_all[cloud_xyz_rgba.size () + i].rgba, cloud_xyz_rgb[i].rgba);
  }

  // _ vs regular
  int rgb_idx = pcl::getFieldIndex (cloud1, "rgba");
  cloud1.fields[rgb_idx].name = "_";
  pcl::concatenatePointCloud (cloud1, cloud2, cloud_out3);
  
  pcl::fromROSMsg (cloud_out3, cloud_all);

  EXPECT_EQ (cloud_all.size (), cloud_xyz_rgba.size () + cloud_xyz_rgba2.size ());
  for (int i = 0; i < int (cloud_xyz_rgba.size ()); ++i)
  {
    EXPECT_FLOAT_EQ (cloud_all[i].x, cloud_xyz_rgba[i].x);
    EXPECT_FLOAT_EQ (cloud_all[i].y, cloud_xyz_rgba[i].y);
    EXPECT_FLOAT_EQ (cloud_all[i].z, cloud_xyz_rgba[i].z);
    // Data doesn't get modified
    EXPECT_FLOAT_EQ (cloud_all[i].r, cloud_xyz_rgba[i].r);
    EXPECT_FLOAT_EQ (cloud_all[i].g, cloud_xyz_rgba[i].g);
    EXPECT_FLOAT_EQ (cloud_all[i].b, cloud_xyz_rgba[i].b);
    EXPECT_EQ (cloud_all[i].rgba, cloud_xyz_rgba[i].rgba);
  }
  for (int i = 0; i < int (cloud_xyz_rgb.size ()); ++i)
  {
    EXPECT_FLOAT_EQ (cloud_all[cloud_xyz_rgba.size () + i].x, cloud_xyz_rgb[i].x);
    EXPECT_FLOAT_EQ (cloud_all[cloud_xyz_rgba.size () + i].y, cloud_xyz_rgb[i].y);
    EXPECT_FLOAT_EQ (cloud_all[cloud_xyz_rgba.size () + i].z, cloud_xyz_rgb[i].z);
    EXPECT_FLOAT_EQ (cloud_all[cloud_xyz_rgba.size () + i].r, 0);
    EXPECT_FLOAT_EQ (cloud_all[cloud_xyz_rgba.size () + i].g, 0);
    EXPECT_FLOAT_EQ (cloud_all[cloud_xyz_rgba.size () + i].b, 0);
    EXPECT_EQ (cloud_all[cloud_xyz_rgba.size () + i].rgba, 0);
  }

  cloud1.fields[rgb_idx].name = "rgba";
  // regular vs _
  rgb_idx = pcl::getFieldIndex (cloud2, "rgb");
  cloud2.fields[rgb_idx].name = "_";
  pcl::concatenatePointCloud (cloud1, cloud2, cloud_out4);

  pcl::fromROSMsg (cloud_out4, cloud_all);

  EXPECT_EQ (cloud_all.size (), cloud_xyz_rgba.size () + cloud_xyz_rgba2.size ());
  for (int i = 0; i < int (cloud_xyz_rgba.size ()); ++i)
  {
    EXPECT_FLOAT_EQ (cloud_all[i].x, cloud_xyz_rgba[i].x);
    EXPECT_FLOAT_EQ (cloud_all[i].y, cloud_xyz_rgba[i].y);
    EXPECT_FLOAT_EQ (cloud_all[i].z, cloud_xyz_rgba[i].z);
    // Data doesn't get modified
    EXPECT_FLOAT_EQ (cloud_all[i].r, cloud_xyz_rgba[i].r);
    EXPECT_FLOAT_EQ (cloud_all[i].g, cloud_xyz_rgba[i].g);
    EXPECT_FLOAT_EQ (cloud_all[i].b, cloud_xyz_rgba[i].b);
    EXPECT_EQ (cloud_all[i].rgba, cloud_xyz_rgba[i].rgba);
  }
  for (int i = 0; i < int (cloud_xyz_rgb.size ()); ++i)
  {
    EXPECT_FLOAT_EQ (cloud_all[cloud_xyz_rgba.size () + i].x, cloud_xyz_rgb[i].x);
    EXPECT_FLOAT_EQ (cloud_all[cloud_xyz_rgba.size () + i].y, cloud_xyz_rgb[i].y);
    EXPECT_FLOAT_EQ (cloud_all[cloud_xyz_rgba.size () + i].z, cloud_xyz_rgb[i].z);
    EXPECT_FLOAT_EQ (cloud_all[cloud_xyz_rgba.size () + i].r, 0);
    EXPECT_FLOAT_EQ (cloud_all[cloud_xyz_rgba.size () + i].g, 0);
    EXPECT_FLOAT_EQ (cloud_all[cloud_xyz_rgba.size () + i].b, 0);
    EXPECT_EQ (cloud_all[cloud_xyz_rgba.size () + i].rgba, 0);
  }

  // _ vs _
  rgb_idx = pcl::getFieldIndex (cloud1, "rgba");
  cloud1.fields[rgb_idx].name = "_";
  pcl::toROSMsg (cloud_xyz_rgb, cloud2);
  rgb_idx = pcl::getFieldIndex (cloud2, "rgb");
  cloud2.fields[rgb_idx].name = "_";

  pcl::concatenatePointCloud (cloud1, cloud2, cloud_out3);
  
  pcl::fromROSMsg (cloud_out3, cloud_all);

  EXPECT_EQ (cloud_all.size (), cloud_xyz_rgba.size () + cloud_xyz_rgba2.size ());
  for (int i = 0; i < int (cloud_xyz_rgba.size ()); ++i)
  {
    EXPECT_FLOAT_EQ (cloud_all[i].x, cloud_xyz_rgba[i].x);
    EXPECT_FLOAT_EQ (cloud_all[i].y, cloud_xyz_rgba[i].y);
    EXPECT_FLOAT_EQ (cloud_all[i].z, cloud_xyz_rgba[i].z);
    // Data doesn't get modified
    EXPECT_FLOAT_EQ (cloud_all[i].r, cloud_xyz_rgba[i].r);
    EXPECT_FLOAT_EQ (cloud_all[i].g, cloud_xyz_rgba[i].g);
    EXPECT_FLOAT_EQ (cloud_all[i].b, cloud_xyz_rgba[i].b);
    EXPECT_EQ (cloud_all[i].rgba, cloud_xyz_rgba[i].rgba);
  }
  for (int i = 0; i < int (cloud_xyz_rgb.size ()); ++i)
  {
    EXPECT_FLOAT_EQ (cloud_all[cloud_xyz_rgba.size () + i].x, cloud_xyz_rgb[i].x);
    EXPECT_FLOAT_EQ (cloud_all[cloud_xyz_rgba.size () + i].y, cloud_xyz_rgb[i].y);
    EXPECT_FLOAT_EQ (cloud_all[cloud_xyz_rgba.size () + i].z, cloud_xyz_rgb[i].z);
    EXPECT_FLOAT_EQ (cloud_all[cloud_xyz_rgba.size () + i].r, 0);
    EXPECT_FLOAT_EQ (cloud_all[cloud_xyz_rgba.size () + i].g, 0);
    EXPECT_FLOAT_EQ (cloud_all[cloud_xyz_rgba.size () + i].b, 0);
    EXPECT_EQ (cloud_all[cloud_xyz_rgba.size () + i].rgba, 0);
  }

  cloud1.fields[rgb_idx].name = "rgba";
  // _ vs regular
  rgb_idx = pcl::getFieldIndex (cloud1, "rgba");

  cloud1.fields[rgb_idx].name = "_";
  pcl::toROSMsg (cloud_xyz_rgb, cloud2);
  pcl::concatenatePointCloud (cloud2, cloud1, cloud_out3);
  
  pcl::fromROSMsg (cloud_out3, cloud_all);

  EXPECT_EQ (cloud_all.size (), cloud_xyz_rgba.size () + cloud_xyz_rgba2.size ());
  for (int i = 0; i < int (cloud_xyz_rgb.size ()); ++i)
  {
    EXPECT_FLOAT_EQ (cloud_all[i].x, cloud_xyz_rgb[i].x);
    EXPECT_FLOAT_EQ (cloud_all[i].y, cloud_xyz_rgb[i].y);
    EXPECT_FLOAT_EQ (cloud_all[i].z, cloud_xyz_rgb[i].z);
    // Data doesn't get modified
    EXPECT_FLOAT_EQ (cloud_all[i].r, cloud_xyz_rgb[i].r);
    EXPECT_FLOAT_EQ (cloud_all[i].g, cloud_xyz_rgb[i].g);
    EXPECT_FLOAT_EQ (cloud_all[i].b, cloud_xyz_rgb[i].b);
    EXPECT_EQ (cloud_all[i].rgba, cloud_xyz_rgb[i].rgba);
  }
  for (int i = 0; i < int (cloud_xyz_rgba.size ()); ++i)
  {
    EXPECT_FLOAT_EQ (cloud_all[cloud_xyz_rgb.size () + i].x, cloud_xyz_rgba[i].x);
    EXPECT_FLOAT_EQ (cloud_all[cloud_xyz_rgb.size () + i].y, cloud_xyz_rgba[i].y);
    EXPECT_FLOAT_EQ (cloud_all[cloud_xyz_rgb.size () + i].z, cloud_xyz_rgba[i].z);
    EXPECT_FLOAT_EQ (cloud_all[cloud_xyz_rgb.size () + i].r, 0);
    EXPECT_FLOAT_EQ (cloud_all[cloud_xyz_rgb.size () + i].g, 0);
    EXPECT_FLOAT_EQ (cloud_all[cloud_xyz_rgb.size () + i].b, 0);
    EXPECT_EQ (cloud_all[cloud_xyz_rgb.size () + i].rgba, 0);
  }

  cloud1.fields[rgb_idx].name = "rgba";
  // regular vs _
  rgb_idx = pcl::getFieldIndex (cloud2, "rgb");
  cloud2.fields[rgb_idx].name = "_";
  pcl::concatenatePointCloud (cloud2, cloud1, cloud_out4);

  pcl::fromROSMsg (cloud_out4, cloud_all);

  EXPECT_EQ (cloud_all.size (), cloud_xyz_rgba.size () + cloud_xyz_rgba2.size ());
  for (int i = 0; i < int (cloud_xyz_rgb.size ()); ++i)
  {
    EXPECT_FLOAT_EQ (cloud_all[i].x, cloud_xyz_rgb[i].x);
    EXPECT_FLOAT_EQ (cloud_all[i].y, cloud_xyz_rgb[i].y);
    EXPECT_FLOAT_EQ (cloud_all[i].z, cloud_xyz_rgb[i].z);
    // Data doesn't get modified
    EXPECT_FLOAT_EQ (cloud_all[i].r, cloud_xyz_rgb[i].r);
    EXPECT_FLOAT_EQ (cloud_all[i].g, cloud_xyz_rgb[i].g);
    EXPECT_FLOAT_EQ (cloud_all[i].b, cloud_xyz_rgb[i].b);
    EXPECT_EQ (cloud_all[i].rgba, cloud_xyz_rgb[i].rgba);
  }
  for (int i = 0; i < int (cloud_xyz_rgba.size ()); ++i)
  {
    EXPECT_FLOAT_EQ (cloud_all[cloud_xyz_rgb.size () + i].x, cloud_xyz_rgba[i].x);
    EXPECT_FLOAT_EQ (cloud_all[cloud_xyz_rgb.size () + i].y, cloud_xyz_rgba[i].y);
    EXPECT_FLOAT_EQ (cloud_all[cloud_xyz_rgb.size () + i].z, cloud_xyz_rgba[i].z);
    EXPECT_FLOAT_EQ (cloud_all[cloud_xyz_rgb.size () + i].r, 0);
    EXPECT_FLOAT_EQ (cloud_all[cloud_xyz_rgb.size () + i].g, 0);
    EXPECT_FLOAT_EQ (cloud_all[cloud_xyz_rgb.size () + i].b, 0);
    EXPECT_EQ (cloud_all[cloud_xyz_rgb.size () + i].rgba, 0);
  }
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

  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
