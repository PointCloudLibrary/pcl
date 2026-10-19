/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2026-, Open Perception Inc.
 *
 *  All rights reserved
 */

#include <pcl/compression/libpng_wrapper.h>
#include <pcl/compression/organized_pointcloud_compression.h>
#include <pcl/test/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cstdint>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

using Compression = pcl::io::OrganizedPointCloudCompression<pcl::PointXYZRGB>;

namespace {
constexpr std::uint32_t width = 8;
constexpr std::uint32_t height = 6;
constexpr float focal_length = 525.0f;

// An organized cloud at depth 1
pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr
createOrganizedCloud()
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>(width, height));
  const int center_x = static_cast<int>(width / 2);
  const int center_y = static_cast<int>(height / 2);
  std::size_t i = 0;
  for (int y = -center_y; y < center_y; ++y)
    for (int x = -center_x; x < center_x; ++x, ++i) {
      auto& point = (*cloud)[i];
      point.x = static_cast<float>(x) / focal_length;
      point.y = static_cast<float>(y) / focal_length;
      point.z = 1.0f;
      point.r = static_cast<std::uint8_t>(i);
      point.g = static_cast<std::uint8_t>(2 * i);
      point.b = static_cast<std::uint8_t>(3 * i);
    }
  return cloud;
}

template <typename T>
void
write(std::ostream& stream, const T& value)
{
  stream.write(reinterpret_cast<const char*>(&value), sizeof(value));
}

// An encoded frame with the given header size, disparity shift and PNG images
std::string
encodeFrame(std::uint32_t frame_width,
            std::uint32_t frame_height,
            float disparity_shift,
            const std::vector<std::uint8_t>& disparity_png,
            const std::vector<std::uint8_t>& color_png)
{
  std::ostringstream frame;
  frame << "<PCL-ORG-COMPRESSED>";
  write(frame, frame_width);
  write(frame, frame_height);
  write(frame, 1.0f);           // max depth
  write(frame, focal_length);   // focal length
  write(frame, 1.0f);           // disparity scale
  write(frame, disparity_shift);
  write(frame, static_cast<std::uint32_t>(disparity_png.size()));
  frame.write(reinterpret_cast<const char*>(disparity_png.data()), disparity_png.size());
  write(frame, static_cast<std::uint32_t>(color_png.size()));
  frame.write(reinterpret_cast<const char*>(color_png.data()), color_png.size());
  return frame.str();
}

std::vector<std::uint8_t>
disparityPNG(std::uint32_t image_width, std::uint32_t image_height)
{
  std::vector<std::uint16_t> image(image_width * image_height, 500);
  std::vector<std::uint8_t> png;
  pcl::io::encodeMonoImageToPNG(image, image_width, image_height, png);
  return png;
}

std::vector<std::uint8_t>
colorPNG(std::uint32_t image_width, std::uint32_t image_height, bool mono)
{
  std::vector<std::uint8_t> image(image_width * image_height * (mono ? 1 : 3), 128);
  std::vector<std::uint8_t> png;
  if (mono)
    pcl::io::encodeMonoImageToPNG(image, image_width, image_height, png);
  else
    pcl::io::encodeRGBImageToPNG(image, image_width, image_height, png);
  return png;
}

bool
decode(const std::string& frame)
{
  Compression compression;
  std::istringstream input(frame);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGB>);
  return compression.decodePointCloud(input, cloud_out, false);
}
} // namespace

TEST(PCL, OrganizedPointCloudCompressionDecodePointCloud)
{
  const auto cloud = createOrganizedCloud();
  for (const bool convert_to_mono : {false, true}) {
    Compression compression;
    std::stringstream compressed_data;
    compression.encodePointCloud(cloud, compressed_data, true, convert_to_mono, false);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out(
        new pcl::PointCloud<pcl::PointXYZRGB>);
    ASSERT_TRUE(compression.decodePointCloud(compressed_data, cloud_out, false));
    ASSERT_EQ(cloud_out->size(), cloud->size());
    for (std::size_t i = 0; i < cloud->size(); ++i) {
      EXPECT_NEAR((*cloud_out)[i].z, (*cloud)[i].z, 1e-3);
      if (!convert_to_mono) {
        EXPECT_EQ((*cloud_out)[i].r, (*cloud)[i].r);
        EXPECT_EQ((*cloud_out)[i].g, (*cloud)[i].g);
        EXPECT_EQ((*cloud_out)[i].b, (*cloud)[i].b);
      }
    }
  }
}

TEST(PCL, OrganizedPointCloudCompressionDecodePointCloudHeaderSizeMismatch)
{
  // The header size must match the size of the encoded images, both when
  // decoding disparity (shift 0) and raw shift values (shift != 0)
  const std::vector<std::pair<std::uint32_t, std::uint32_t>> wrong_sizes = {
      {2 * width, height}, {width, 2 * height}, {width / 2, height}, {65536, 65536}};
  const auto disparity_png = disparityPNG(width, height);
  for (const float disparity_shift : {0.0f, 174.825f})
    for (const auto& color_png : {std::vector<std::uint8_t>(),
                                  colorPNG(width, height, false),
                                  colorPNG(width, height, true)}) {
      EXPECT_TRUE(decode(
          encodeFrame(width, height, disparity_shift, disparity_png, color_png)));
      for (const auto& size : wrong_sizes)
        EXPECT_FALSE(decode(encodeFrame(
            size.first, size.second, disparity_shift, disparity_png, color_png)))
            << "header size " << size.first << " x " << size.second;
    }
}

TEST(PCL, OrganizedPointCloudCompressionDecodePointCloudColorSizeMismatch)
{
  // The color image must have the size of the disparity image
  const auto disparity_png = disparityPNG(width, height);
  for (const float disparity_shift : {0.0f, 174.825f})
    for (const bool mono : {false, true}) {
      EXPECT_FALSE(decode(encodeFrame(width,
                                      height,
                                      disparity_shift,
                                      disparity_png,
                                      colorPNG(width / 2, height, mono))));
      EXPECT_FALSE(decode(encodeFrame(width,
                                      height,
                                      disparity_shift,
                                      disparity_png,
                                      colorPNG(width, height / 2, mono))));
    }
}

/* ---[ */
int
main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return (RUN_ALL_TESTS());
}
/* ]--- */
