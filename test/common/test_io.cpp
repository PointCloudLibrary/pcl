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
  for (int i = 0; i < static_cast<int>(cloud_xyz_rgba.size ()); ++i)
  {
    EXPECT_XYZ_EQ (cloud_all[i], cloud_xyz_rgba[i]);
    EXPECT_RGBA_EQ (cloud_all[i], cloud_xyz_rgba[i]);
  }
  for (int i = 0; i < static_cast<int>(cloud_xyz_rgba2.size ()); ++i)
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
  for (int i = 0; i < static_cast<int>(cloud_xyz_rgba.size ()); ++i)
  {
    EXPECT_XYZ_EQ (cloud_all[i], cloud_xyz_rgba[i]);
    EXPECT_RGBA_EQ (cloud_all[i], cloud_xyz_rgba[i]);
  }
  for (int i = 0; i < static_cast<int>(cloud_xyz_rgb.size ()); ++i)
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
  for (int i = 0; i < static_cast<int>(cloud_xyz_rgba.size ()); ++i)
  {
    EXPECT_XYZ_EQ (cloud_all[i], cloud_xyz_rgba[i]);
    // Data doesn't get modified
    EXPECT_RGBA_EQ (cloud_all[i], cloud_xyz_rgba[i]);
  }
  for (int i = 0; i < static_cast<int>(cloud_xyz_rgb.size ()); ++i)
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

///////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, concatenatePointCloud2FieldLayout)
{
  auto add_field = [](pcl::PCLPointCloud2& cloud, const std::string& name, const std::uint8_t datatype)
  {
    cloud.fields.emplace_back ();
    cloud.fields.back ().name = name;
    cloud.fields.back ().offset = cloud.point_step;
    cloud.fields.back ().datatype = datatype;
    cloud.fields.back ().count = 1;
    cloud.point_step += pcl::getFieldSize (datatype);
  };
  auto resize = [](pcl::PCLPointCloud2& cloud, const pcl::uindex_t width)
  {
    cloud.width = width;
    cloud.height = 1;
    cloud.row_step = cloud.width * cloud.point_step;
    cloud.data.resize (cloud.row_step);
  };

  // x, y and z without padding (point_step 12), as read from a file
  pcl::PCLPointCloud2 cloud_packed;
  add_field (cloud_packed, "x", pcl::PCLPointField::FLOAT32);
  add_field (cloud_packed, "y", pcl::PCLPointField::FLOAT32);
  add_field (cloud_packed, "z", pcl::PCLPointField::FLOAT32);
  resize (cloud_packed, 3);
  const std::vector<float> xyz_packed = {1.f, 2.f, 3.f, 4.f, 5.f, 6.f, 7.f, 8.f, 9.f};
  memcpy (cloud_packed.data.data (), xyz_packed.data (), cloud_packed.data.size ());

  // The same fields with padding (point_step 16)
  CloudXYZ cloud_xyz;
  cloud_xyz.emplace_back (10.f, 11.f, 12.f);
  cloud_xyz.emplace_back (13.f, 14.f, 15.f);
  pcl::PCLPointCloud2 cloud_padded;
  pcl::toPCLPointCloud2 (cloud_xyz, cloud_padded);
  ASSERT_GT (cloud_padded.point_step, cloud_packed.point_step);

  // The points of cloud_xyz followed by the points of cloud_packed
  auto expect_concatenated = [&](const pcl::PCLPointCloud2& cloud)
  {
    ASSERT_EQ (cloud_xyz.size () + 3, cloud.width * cloud.height);
    ASSERT_EQ (cloud.width * cloud.height * cloud.point_step, cloud.data.size ());
    CloudXYZ cloud_all;
    pcl::fromPCLPointCloud2 (cloud, cloud_all);
    for (std::size_t i = 0; i < cloud_xyz.size (); ++i)
      EXPECT_XYZ_EQ (cloud_all[i], cloud_xyz[i]);
    for (std::size_t i = 0; i < 3; ++i)
    {
      EXPECT_EQ (cloud_all[cloud_xyz.size () + i].x, xyz_packed[3 * i]);
      EXPECT_EQ (cloud_all[cloud_xyz.size () + i].y, xyz_packed[3 * i + 1]);
      EXPECT_EQ (cloud_all[cloud_xyz.size () + i].z, xyz_packed[3 * i + 2]);
    }
  };

  // Same field names, different point_step
  pcl::PCLPointCloud2 cloud_out;
  ASSERT_TRUE (pcl::concatenate (cloud_padded, cloud_packed, cloud_out));
  expect_concatenated (cloud_out);

  // Same field names and point_step, different offsets
  pcl::PCLPointCloud2 cloud_shifted, cloud_out_shifted;
  cloud_shifted.point_step = 4;
  add_field (cloud_shifted, "x", pcl::PCLPointField::FLOAT32);
  add_field (cloud_shifted, "y", pcl::PCLPointField::FLOAT32);
  add_field (cloud_shifted, "z", pcl::PCLPointField::FLOAT32);
  ASSERT_EQ (cloud_padded.point_step, cloud_shifted.point_step);
  resize (cloud_shifted, 3);
  for (std::size_t i = 0; i < 3; ++i)
    memcpy (&cloud_shifted.data[i * cloud_shifted.point_step + 4], &xyz_packed[3 * i], 3 * sizeof (float));
  ASSERT_TRUE (pcl::concatenate (cloud_padded, cloud_shifted, cloud_out_shifted));
  expect_concatenated (cloud_out_shifted);

  // Skip field in the middle of the first cloud only
  pcl::PCLPointCloud2 cloud_skip, cloud_out_skip;
  add_field (cloud_skip, "x", pcl::PCLPointField::FLOAT32);
  add_field (cloud_skip, "y", pcl::PCLPointField::FLOAT32);
  add_field (cloud_skip, "_", pcl::PCLPointField::FLOAT32);
  add_field (cloud_skip, "z", pcl::PCLPointField::FLOAT32);
  resize (cloud_skip, 2);
  ASSERT_TRUE (pcl::concatenate (cloud_skip, cloud_packed, cloud_out_skip));
  ASSERT_EQ (5u, cloud_out_skip.width * cloud_out_skip.height);
  ASSERT_EQ (cloud_out_skip.width * cloud_out_skip.height * cloud_out_skip.point_step, cloud_out_skip.data.size ());
  for (std::size_t i = 0; i < 3; ++i)
  {
    float xyz[4];
    memcpy (xyz, &cloud_out_skip.data[(2 + i) * cloud_out_skip.point_step], sizeof (xyz));
    EXPECT_EQ (xyz[0], xyz_packed[3 * i]);
    EXPECT_EQ (xyz[1], xyz_packed[3 * i + 1]);
    EXPECT_EQ (xyz[3], xyz_packed[3 * i + 2]);
  }

  // Field with a different size
  pcl::PCLPointCloud2 cloud_double;
  add_field (cloud_double, "x", pcl::PCLPointField::FLOAT32);
  add_field (cloud_double, "y", pcl::PCLPointField::FLOAT32);
  add_field (cloud_double, "z", pcl::PCLPointField::FLOAT64);
  resize (cloud_double, 3);
  EXPECT_FALSE (pcl::concatenate (cloud_padded, cloud_double, cloud_out));
  EXPECT_FALSE (pcl::concatenate (cloud_packed, cloud_double, cloud_out));
  cloud_double.fields[2].datatype = pcl::PCLPointField::FLOAT32;
  cloud_double.fields[2].count = 2;
  EXPECT_FALSE (pcl::concatenate (cloud_packed, cloud_double, cloud_out));

  // Field outside of the point
  pcl::PCLPointCloud2 cloud_short = cloud_packed;
  cloud_short.point_step = 8;
  resize (cloud_short, 3);
  EXPECT_FALSE (pcl::concatenate (cloud_padded, cloud_short, cloud_out));
  EXPECT_FALSE (pcl::concatenate (cloud_short, cloud_padded, cloud_out));

  // Data smaller than width * height * point_step
  cloud_short = cloud_packed;
  cloud_short.data.resize (cloud_short.data.size () - 1);
  EXPECT_FALSE (pcl::concatenate (cloud_padded, cloud_short, cloud_out));
  EXPECT_FALSE (pcl::concatenate (cloud_packed, cloud_short, cloud_out));
  EXPECT_FALSE (pcl::concatenate (cloud_short, cloud_packed, cloud_out));

  // Different names after skip fields
  pcl::PCLPointCloud2 cloud_skip_x, cloud_y;
  add_field (cloud_skip_x, "_", pcl::PCLPointField::FLOAT32);
  add_field (cloud_skip_x, "_", pcl::PCLPointField::FLOAT32);
  add_field (cloud_skip_x, "x", pcl::PCLPointField::FLOAT32);
  resize (cloud_skip_x, 2);
  add_field (cloud_y, "y", pcl::PCLPointField::FLOAT32);
  resize (cloud_y, 2);
  EXPECT_FALSE (pcl::concatenate (cloud_skip_x, cloud_y, cloud_out));
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

TEST (toPCLPointCloud2NoPadding, PointXYZI)
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  cloud.resize(static_cast<pcl::uindex_t>(2), static_cast<pcl::uindex_t>(2));
  cloud[0].x = 1.0; cloud[0].y = 2.0; cloud[0].z = 3.0; cloud[0].intensity = 123.0;
  cloud[1].x = -1.0; cloud[1].y = -2.0; cloud[1].z = -3.0; cloud[1].intensity = -123.0;
  cloud[2].x = 0.1; cloud[2].y = 0.2; cloud[2].z = 0.3; cloud[2].intensity = 12.3;
  cloud[3].x = 0.0; cloud[3].y = -1.7; cloud[3].z = 100.0; cloud[3].intensity = 3.14;
  pcl::PCLPointCloud2 msg;
  pcl::toPCLPointCloud2(cloud, msg, false);
  EXPECT_EQ (msg.height, cloud.height);
  EXPECT_EQ (msg.width, cloud.width);
  EXPECT_EQ (msg.fields.size(), 4);
  EXPECT_EQ (msg.fields[0].name, "x");
  EXPECT_EQ (msg.fields[0].offset, 0);
  EXPECT_EQ (msg.fields[0].datatype, pcl::PCLPointField::FLOAT32);
  EXPECT_EQ (msg.fields[0].count, 1);
  EXPECT_EQ (msg.fields[1].name, "y");
  EXPECT_EQ (msg.fields[1].offset, 4);
  EXPECT_EQ (msg.fields[1].datatype, pcl::PCLPointField::FLOAT32);
  EXPECT_EQ (msg.fields[1].count, 1);
  EXPECT_EQ (msg.fields[2].name, "z");
  EXPECT_EQ (msg.fields[2].offset, 8);
  EXPECT_EQ (msg.fields[2].datatype, pcl::PCLPointField::FLOAT32);
  EXPECT_EQ (msg.fields[2].count, 1);
  EXPECT_EQ (msg.fields[3].name, "intensity");
  EXPECT_EQ (msg.fields[3].offset, 12);
  EXPECT_EQ (msg.fields[3].datatype, pcl::PCLPointField::FLOAT32);
  EXPECT_EQ (msg.fields[3].count, 1);
  EXPECT_EQ (msg.point_step, 16);
  EXPECT_EQ (msg.row_step, 16*cloud.width);
  EXPECT_EQ (msg.data.size(), 16*cloud.width*cloud.height);
  EXPECT_EQ (msg.at<float>(0, 0), 1.0f);
  EXPECT_EQ (msg.at<float>(3, 4), -1.7f);
  EXPECT_EQ (msg.at<float>(1, 8), -3.0f);
  EXPECT_EQ (msg.at<float>(2, 12), 12.3f);
  pcl::PointCloud<pcl::PointXYZI> cloud2;
  pcl::fromPCLPointCloud2(msg, cloud2);
  for(std::size_t i=0; i<cloud2.size(); ++i) {
    EXPECT_EQ (cloud[i].x, cloud2[i].x);
    EXPECT_EQ (cloud[i].y, cloud2[i].y);
    EXPECT_EQ (cloud[i].z, cloud2[i].z);
    EXPECT_EQ (cloud[i].intensity, cloud2[i].intensity);
  }
}

TEST(PCL, fromPCLPointCloud2CastingXYZI)
{
  // test fromPCLPointCloud2, but in PCLPointCloud2 the fields have different types than in PointXYZI
  pcl::PCLPointCloud2 msg;
  msg.height = 2;
  msg.width = 2;
  msg.fields.resize(4);
  msg.fields[0].name = "x";
  msg.fields[0].offset = 0;
  msg.fields[0].datatype = pcl::PCLPointField::FLOAT64;
  msg.fields[0].count = 1;
  msg.fields[1].name = "y";
  msg.fields[1].offset = 8;
  msg.fields[1].datatype = pcl::PCLPointField::FLOAT64;
  msg.fields[1].count = 1;
  msg.fields[2].name = "z";
  msg.fields[2].offset = 16;
  msg.fields[2].datatype = pcl::PCLPointField::FLOAT64;
  msg.fields[2].count = 1;
  msg.fields[3].name = "intensity";
  msg.fields[3].offset = 24;
  msg.fields[3].datatype = pcl::PCLPointField::UINT16;
  msg.fields[3].count = 1;
  msg.point_step = 32;
  msg.row_step = 32*msg.width;
  msg.data.resize(32*msg.width*msg.height);
  for(std::size_t i=0; i<msg.width*msg.height; ++i) {
    msg.at<double>(i, 0) = 1.0*i;
    msg.at<double>(i, 8) = -1.6*i;
    msg.at<double>(i, 16) = -3.141*i;
    msg.at<std::uint16_t>(i, 24) = 123*i;
  }
  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::fromPCLPointCloud2(msg, cloud);
  for(std::size_t i=0; i<msg.width*msg.height; ++i) {
    EXPECT_EQ(cloud[i].x, 1.0f*i);
    EXPECT_EQ(cloud[i].y, -1.6f*i);
    EXPECT_EQ(cloud[i].z, -3.141f*i);
    EXPECT_EQ(cloud[i].intensity, 123.0f*i);
  }
}

TEST(PCL, fromPCLPointCloud2CastingSHOT352)
{
  // test whether casting also works if point type contains arrays
  pcl::PCLPointCloud2 msg;
  msg.height = 2;
  msg.width = 2;
  msg.fields.resize(2);
  msg.fields[0].name = "shot";
  msg.fields[0].offset = 0;
  msg.fields[0].datatype = pcl::PCLPointField::INT64;
  msg.fields[0].count = 352;
  msg.fields[1].name = "rf";
  msg.fields[1].offset = 352*8;
  msg.fields[1].datatype = pcl::PCLPointField::FLOAT64;
  msg.fields[1].count = 9;
  msg.point_step = (352*8+9*8);
  msg.row_step = msg.point_step*msg.width;
  msg.data.resize(msg.point_step*msg.width*msg.height);
  for(std::size_t i=0; i<msg.width*msg.height; ++i) {
    for(std::size_t j=0; j<352; ++j)
      msg.at<std::int64_t>(i, 8*j) = 1000*i+j;
    for(std::size_t j=0; j<9; ++j)
      msg.at<double>(i, 352*8+8*j) = (10*i+j)/10.0;
  }
  pcl::PointCloud<pcl::SHOT352> cloud;
  pcl::fromPCLPointCloud2(msg, cloud);
  for(std::size_t i=0; i<msg.width*msg.height; ++i) {
    for(std::size_t j=0; j<352; ++j)
      EXPECT_EQ(cloud[i].descriptor[j], 1000*i+j);
    for(std::size_t j=0; j<9; ++j)
      EXPECT_EQ(cloud[i].rf[j], (10*i+j)/10.0f);
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

  pt_xyz.x = 4;
  pt_xyz.y = 1;
  pt_xyz.z = 5;

  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
