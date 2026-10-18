/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2026-, Open Perception Inc.
 *
 *  All rights reserved
 */

#include <pcl/compression/libpng_wrapper.h>
#include <pcl/test/gtest.h>

#include <cstddef>
#include <cstdint>
#include <vector>

namespace {
constexpr std::size_t width = 16;
constexpr std::size_t height = 12;

template <typename T>
std::vector<T>
createImage(std::size_t channels)
{
  std::vector<T> image(width * height * channels);
  for (std::size_t i = 0; i < image.size(); ++i)
    image[i] = static_cast<T>(i * 7);
  return image;
}

// Decode into outputs that hold stale values, so that a failed decode must clear them
template <typename T>
void
decodeWithStaleOutputs(std::vector<std::uint8_t>& png,
                       std::vector<T>& image,
                       std::size_t& image_width,
                       std::size_t& image_height,
                       unsigned int& channels)
{
  image.assign(4, 1);
  image_width = 1;
  image_height = 1;
  channels = 1;
  pcl::io::decodePNGToImage(png, image, image_width, image_height, channels);
}

template <typename T>
void
expectEmptyOutputs(const std::vector<T>& image,
                   std::size_t image_width,
                   std::size_t image_height,
                   unsigned int channels)
{
  EXPECT_TRUE(image.empty());
  EXPECT_EQ(image_width, 0u);
  EXPECT_EQ(image_height, 0u);
  EXPECT_EQ(channels, 0u);
}
} // namespace

TEST(PCL, decodePNGToImageRoundTrip)
{
  std::vector<std::uint8_t> png;
  std::size_t image_width, image_height;
  unsigned int channels;

  const auto mono8 = createImage<std::uint8_t>(1);
  auto input8 = mono8;
  pcl::io::encodeMonoImageToPNG(input8, width, height, png);
  std::vector<std::uint8_t> decoded8;
  decodeWithStaleOutputs(png, decoded8, image_width, image_height, channels);
  EXPECT_EQ(image_width, width);
  EXPECT_EQ(image_height, height);
  EXPECT_EQ(channels, 1u);
  EXPECT_EQ(decoded8, mono8);

  const auto rgb8 = createImage<std::uint8_t>(3);
  input8 = rgb8;
  pcl::io::encodeRGBImageToPNG(input8, width, height, png);
  decodeWithStaleOutputs(png, decoded8, image_width, image_height, channels);
  EXPECT_EQ(image_width, width);
  EXPECT_EQ(image_height, height);
  EXPECT_EQ(channels, 3u);
  EXPECT_EQ(decoded8, rgb8);

  const auto mono16 = createImage<std::uint16_t>(1);
  auto input16 = mono16;
  pcl::io::encodeMonoImageToPNG(input16, width, height, png);
  std::vector<std::uint16_t> decoded16;
  decodeWithStaleOutputs(png, decoded16, image_width, image_height, channels);
  EXPECT_EQ(image_width, width);
  EXPECT_EQ(image_height, height);
  EXPECT_EQ(channels, 1u);
  EXPECT_EQ(decoded16, mono16);
}

TEST(PCL, decodePNGToImageTruncatedData)
{
  auto image = createImage<std::uint8_t>(3);
  std::vector<std::uint8_t> png;
  pcl::io::encodeRGBImageToPNG(image, width, height, png);

  // The encoded data ends with the 12 byte IEND chunk, which the decoder does not read.
  // Any shorter input ends inside the signature, the header or the image data.
  constexpr std::size_t iend_size = 12;
  ASSERT_GT(png.size(), iend_size);
  for (std::size_t size = 1; size < png.size() - iend_size; ++size) {
    std::vector<std::uint8_t> truncated(png.begin(), png.begin() + size);
    std::vector<std::uint8_t> decoded;
    std::size_t image_width, image_height;
    unsigned int channels;
    decodeWithStaleOutputs(truncated, decoded, image_width, image_height, channels);
    SCOPED_TRACE(size);
    expectEmptyOutputs(decoded, image_width, image_height, channels);
  }
}

TEST(PCL, decodePNGToImageBitDepthMismatch)
{
  std::size_t image_width, image_height;
  unsigned int channels;

  // 16-bit PNG decoded to an 8-bit image
  auto mono16 = createImage<std::uint16_t>(1);
  std::vector<std::uint8_t> png16;
  pcl::io::encodeMonoImageToPNG(mono16, width, height, png16);
  std::vector<std::uint8_t> decoded8;
  decodeWithStaleOutputs(png16, decoded8, image_width, image_height, channels);
  expectEmptyOutputs(decoded8, image_width, image_height, channels);

  // 8-bit PNG decoded to a 16-bit image
  auto mono8 = createImage<std::uint8_t>(1);
  std::vector<std::uint8_t> png8;
  pcl::io::encodeMonoImageToPNG(mono8, width, height, png8);
  std::vector<std::uint16_t> decoded16;
  decodeWithStaleOutputs(png8, decoded16, image_width, image_height, channels);
  expectEmptyOutputs(decoded16, image_width, image_height, channels);
}

int
main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
