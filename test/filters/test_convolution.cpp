/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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
#include <pcl/pcl_tests.h>
#include <pcl/filters/convolution.h>
#include <pcl/point_types.h>
#include <cmath>
#include <array>
#include <algorithm>

using namespace pcl;
using namespace pcl::common;
using namespace pcl::filters;
using namespace pcl::test;

std::array<RGB, 5> colormap {
  RGB(74, 19, 139),
  RGB(29, 135, 228),
  RGB(139, 194, 74),
  RGB(255, 235, 59),
  RGB(255, 0, 0),
};

RGB interpolate_color(float lower_bound, float upper_bound, float value)
{
  if (value <= lower_bound) return colormap[0];
  if (value >= upper_bound) return colormap.back();
  float step_size = (upper_bound - lower_bound) / static_cast<float>(colormap.size() - 1);
  std::size_t lower_index = static_cast<std::size_t>((value - lower_bound) / step_size);
  value -= (lower_bound + static_cast<float>(lower_index) * step_size);
  if (value == 0) return colormap[lower_index];
  auto interpolate = [](std::uint8_t lower, std::uint8_t upper, float step_size, float value) {
    return (lower == upper) ? lower : static_cast<std::uint8_t>(static_cast<float>(lower) + ((static_cast<float>(upper) - static_cast<float>(lower)) / step_size) * value);
  };
  return RGB(
    interpolate(colormap[lower_index].r, colormap[lower_index + 1].r, step_size, value),
    interpolate(colormap[lower_index].g, colormap[lower_index + 1].g, step_size, value),
    interpolate(colormap[lower_index].b, colormap[lower_index + 1].b, step_size, value)
  );
}

TEST (Convolution, convolveRowsXYZI)
{
  // input
  Eigen::ArrayXf filter(7);
  filter << 0.00443305, 0.0540056, 0.242036, 0.39905, 0.242036, 0.0540056, 0.00443305;
  auto input = pcl::make_shared<PointCloud<PointXYZI>>();
  input->width = 64;
  input->height = 48;
  input->resize (input->width * input->height);
  for (int r = 0; r < input->height; r++)
  {
    float y = 40.0f + 40.0f / 65.0f * static_cast<float>(r);
    float z = (r % 2 == 0) ? 1.0f : -1.0f;
    for(int c = 0; c < input->width; c++)
    {
      float x = 65.0f - 30.0f / 65.0f * static_cast<float>(c);
      z += (c % 2 == 0) ? 1.0f : -1.0f;
      (*input) (c,r).intensity = x + y + z;
    }
  }

  // filter
  auto output = pcl::make_shared<PointCloud<PointXYZI>>();
  pcl::filters::Convolution<PointXYZI, PointXYZI> convolve;
  convolve.setInputCloud(input);
  convolve.setKernel(filter);
  convolve.convolveRows(*output);

  // output rows
  // note: this is because of border policy ignore which sets the border of width = filter_size/2 as NaN for xyz and 0 for values
  std::array<float, 128> output_results {
    0.00000000f, 0.00000000f,
    0.00000000f, 0.00000000f,
    0.00000000f, 0.00000000f,
    105.10825348f, 132.03129578f,
    104.66084290f, 131.58390808f,
    104.18517303f, 131.10823059f,
    103.73776245f, 130.66082764f,
    103.26208496f, 130.18515015f,
    102.81467438f, 129.73773193f,
    102.33901978f, 129.26208496f,
    101.89160919f, 128.81466675f,
    101.41593933f, 128.33900452f,
    100.96853638f, 127.89158630f,
    100.49286652f, 127.41592407f,
    100.04545593f, 126.96851349f,
    99.56979370f, 126.49285126f,
    99.12238312f, 126.04543304f,
    98.64672089f, 125.56977844f,
    98.19929504f, 125.12236786f,
    97.72364044f, 124.64669800f,
    97.27622223f, 124.19929504f,
    96.80056000f, 123.72362518f,
    96.35314941f, 123.27621460f,
    95.87748718f, 122.80054474f,
    95.43006897f, 122.35313416f,
    94.95440674f, 121.87747192f,
    94.50699615f, 121.43005371f,
    94.03132629f, 120.95440674f,
    93.58392334f, 120.50697327f,
    93.10826111f, 120.03132629f,
    92.66084290f, 119.58390808f,
    92.18517303f, 119.10823822f,
    91.73777008f, 118.66082764f,
    91.26210785f, 118.18516541f,
    90.81468964f, 117.73775482f,
    90.33903503f, 117.26208496f,
    89.89160156f, 116.81467438f,
    89.41595459f, 116.33901215f,
    88.96853638f, 115.89160919f,
    88.49288177f, 115.41593170f,
    88.04545593f, 114.96851349f,
    87.56979370f, 114.49285889f,
    87.12238312f, 114.04544830f,
    86.64672089f, 113.56979370f,
    86.19931030f, 113.12236023f,
    85.72364044f, 112.64670563f,
    85.27622986f, 112.19928741f,
    84.80056763f, 111.72362518f,
    84.35315704f, 111.27622223f,
    83.87749481f, 110.80056000f,
    83.43008423f, 110.35313416f,
    82.95442200f, 109.87748718f,
    82.50699615f, 109.43006134f,
    82.03134155f, 108.95440674f,
    81.58392334f, 108.50699615f,
    81.10826874f, 108.03132629f,
    80.66085052f, 107.58391571f,
    80.18519592f, 107.10824585f,
    79.73777008f, 106.66084290f,
    79.26210785f, 106.18517303f,
    78.81469727f, 105.73775482f,
    0.00000000f, 0.00000000f,
    0.00000000f, 0.00000000f,
    0.00000000f, 0.00000000f,
  };

  // check result
  for (std::uint32_t i = 0; i < output->width ; ++i)
  {
    EXPECT_FLOAT_EQ ((*output) (i, 0).intensity, output_results[i * 2 + 0]);
    EXPECT_FLOAT_EQ ((*output) (i, 47).intensity, output_results[i * 2 + 1]);
  }
}

TEST (Convolution, convolveRowsRGB)
{
  // input
  Eigen::ArrayXf filter(7);
  filter << 0.00443305, 0.0540056, 0.242036, 0.39905, 0.242036, 0.0540056, 0.00443305;
  auto input = pcl::make_shared<PointCloud<RGB>>();
  input->width = 64;
  input->height = 48;
  input->resize(input->width * input->height);
  for (std::uint32_t r = 0; r < input->height; r++)
    for (std::uint32_t c = 0; c < input->width; c++)
    {
      float x1 = -2.0f + (4.0f / (float)input->width) * (float)c;
      float y1 = -2.0f + (4.0f / (float)input->height) * (float)r;
      float x2 = -M_PI + (2.0f * M_PI / (float)input->width) * (float)c;
      float y2 = -2.0f + (4.0f / (float)input->height) * (float)r;
      float z = x1 * exp(-(x1 * x1 + y1 * y1)) * 2.5f + sin(x2) * sin(y2);
      (*input) (c, r) = interpolate_color(-1.6f, 1.6f, z);
    }

  // filter
  auto output = pcl::make_shared<PointCloud<RGB>>();
  pcl::filters::Convolution<RGB, RGB> convolve;
  convolve.setInputCloud (input);
  convolve.setKernel (filter);
  convolve.convolveRows (*output);

  // output rows (first and last row)
  // note: this is because of border policy ignore which sets the border of width = filter_size/2 as NaN for xyz and 0 for rgb values
  std::array<std::uint8_t, 384> output_results {
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,
    175, 206, 68, 100, 173, 126,
    187, 210, 67, 88, 166, 143,
    199, 214, 65, 77, 160, 159,
    210, 218, 64, 66, 154, 175,
    220, 222, 63, 55, 149, 189,
    230, 225, 61, 46, 144, 202,
    238, 228, 60, 38, 139, 214,
    246, 231, 59, 32, 134, 221,
    251, 230, 58, 31, 128, 221,
    254, 225, 56, 33, 122, 218,
    254, 218, 54, 34, 117, 214,
    254, 213, 53, 36, 114, 212,
    254, 209, 52, 37, 112, 210,
    254, 208, 52, 37, 111, 209,
    254, 210, 52, 37, 111, 210,
    254, 214, 53, 36, 113, 211,
    254, 220, 55, 35, 116, 213,
    253, 226, 56, 33, 121, 217,
    250, 230, 58, 32, 126, 220,
    244, 230, 59, 32, 132, 221,
    237, 228, 60, 36, 137, 216,
    228, 225, 62, 44, 142, 205,
    219, 222, 63, 53, 148, 192,
    209, 218, 64, 64, 153, 177,
    198, 214, 65, 75, 159, 161,
    187, 210, 67, 87, 166, 145,
    175, 206, 68, 99, 172, 127,
    163, 202, 70, 112, 179, 110,
    151, 197, 72, 125, 186, 93,
    139, 192, 79, 138, 192, 79,
    127, 187, 91, 152, 198, 73,
    115, 181, 106, 165, 203, 70,
    103, 175, 122, 179, 207, 68,
    92, 169, 138, 192, 212, 66,
    81, 163, 153, 204, 216, 64,
    71, 157, 167, 216, 220, 63,
    61, 152, 180, 227, 224, 61,
    52, 147, 193, 238, 228, 60,
    44, 143, 204, 246, 229, 59,
    37, 139, 214, 252, 226, 57,
    33, 135, 220, 254, 218, 54,
    31, 131, 223, 254, 207, 51,
    31, 127, 221, 254, 198, 49,
    32, 124, 219, 254, 192, 48,
    33, 122, 218, 254, 188, 47,
    33, 121, 217, 254, 187, 46,
    33, 122, 218, 254, 189, 47,
    32, 123, 219, 254, 194, 48,
    31, 126, 221, 254, 201, 50,
    31, 130, 223, 254, 210, 52,
    32, 134, 221, 254, 220, 55,
    36, 138, 215, 251, 228, 57,
    43, 142, 206, 244, 229, 59,
    51, 146, 195, 235, 227, 61,
    60, 151, 182, 225, 224, 62,
    70, 156, 169, 214, 220, 63,
    80, 162, 154, 203, 216, 65,
    91, 168, 139, 191, 212, 66,
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,
  };

  // check result
  for (std::uint32_t i = 0; i < output->width ; ++i)
  {
    EXPECT_EQ ((*output) (i, 0).r, output_results[i * 6 + 0]);
    EXPECT_EQ ((*output) (i, 0).g, output_results[i * 6 + 1]);
    EXPECT_EQ ((*output) (i, 0).b, output_results[i * 6 + 2]);
    EXPECT_EQ ((*output) (i, 47).r, output_results[i * 6 + 3]);
    EXPECT_EQ ((*output) (i, 47).g, output_results[i * 6 + 4]);
    EXPECT_EQ ((*output) (i, 47).b, output_results[i * 6 + 5]);
  }
}

TEST (Convolution, convolveRowsXYZRGB)
{
  // input
  Eigen::ArrayXf filter(7);
  filter << 0.00443305, 0.0540056, 0.242036, 0.39905, 0.242036, 0.0540056, 0.00443305;
  auto input = pcl::make_shared<PointCloud<PointXYZRGB>>();
  input->width = 64;
  input->height = 48;
  input->resize(input->width * input->height);
  for (std::uint32_t r = 0; r < input->height; r++)
    for (std::uint32_t c = 0; c < input->width; c++)
    {
      float x1 = -2.0f + (4.0f / (float)input->width) * (float)c;
      float y1 = -2.0f + (4.0f / (float)input->height) * (float)r;
      float x2 = -M_PI + (2.0f * M_PI / (float)input->width) * (float)c;
      float y2 = -2.0f + (4.0f / (float)input->height) * (float)r;
      float z = x1 * exp(-(x1 * x1 + y1 * y1)) * 2.5f + sin(x2) * sin(y2);
      RGB color = interpolate_color(-1.6f, 1.6f, z);
      (*input) (c, r).x = x1;
      (*input) (c, r).y = y1;
      (*input) (c, r).z = z;
      (*input) (c, r).r = (uint8_t)(255.0f * color.r);
      (*input) (c, r).g = (uint8_t)(255.0f * color.g);
      (*input) (c, r).b = (uint8_t)(255.0f * color.b);
    }

  // filter
  auto output = pcl::make_shared<PointCloud<PointXYZRGB>>();
  pcl::filters::Convolution<PointXYZRGB, PointXYZRGB> convolve;
  convolve.setInputCloud (input);
  convolve.setKernel (filter);
  convolve.convolveRows (*output);

  // output rows (first and last row)
  // note: this is because of border policy ignore which sets the border of width = filter_size/2 as NaN for xyz and 0 for rgb values
  std::array<std::uint8_t, 384> output_results {
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,
    80, 49, 187, 155, 82, 129,
    68, 45, 188, 167, 89, 112,
    56, 41, 190, 178, 95, 96,
    45, 37, 191, 189, 101, 80,
    35, 33, 192, 200, 106, 66,
    25, 30, 194, 209, 111, 53,
    17, 27, 195, 217, 116, 41,
    9, 24, 196, 223, 121, 34,
    4, 25, 197, 224, 127, 34,
    1, 30, 199, 222, 133, 37,
    1, 37, 201, 221, 138, 41,
    1, 42, 202, 219, 141, 43,
    0, 46, 203, 218, 143, 45,
    0, 47, 203, 218, 144, 46,
    0, 45, 203, 218, 144, 45,
    1, 41, 202, 219, 142, 44,
    1, 35, 200, 220, 139, 42,
    2, 29, 199, 222, 134, 38,
    5, 25, 197, 223, 129, 35,
    11, 25, 196, 223, 123, 34,
    18, 27, 195, 219, 118, 39,
    27, 30, 193, 211, 113, 50,
    36, 33, 192, 202, 107, 63,
    46, 37, 191, 191, 102, 78,
    57, 41, 190, 180, 96, 94,
    68, 45, 188, 168, 89, 110,
    80, 49, 187, 156, 83, 128,
    92, 53, 185, 143, 76, 145,
    104, 58, 183, 130, 69, 162,
    116, 63, 176, 117, 63, 176,
    128, 68, 164, 103, 57, 182,
    140, 74, 149, 90, 52, 185,
    152, 80, 133, 76, 48, 187,
    163, 86, 117, 63, 43, 189,
    174, 92, 102, 51, 39, 191,
    184, 98, 88, 39, 35, 192,
    194, 103, 75, 28, 31, 193,
    203, 108, 62, 17, 27, 195,
    211, 112, 51, 9, 26, 196,
    218, 116, 41, 3, 29, 198,
    222, 120, 35, 1, 37, 201,
    224, 124, 32, 1, 48, 204,
    224, 128, 34, 0, 57, 206,
    223, 131, 36, 0, 63, 207,
    222, 133, 37, 0, 67, 208,
    222, 134, 38, 0, 68, 209,
    222, 133, 37, 0, 66, 208,
    223, 132, 36, 0, 61, 207,
    224, 129, 34, 1, 54, 205,
    224, 125, 32, 1, 45, 203,
    223, 121, 34, 1, 35, 200,
    219, 117, 40, 4, 27, 198,
    212, 113, 49, 11, 26, 196,
    204, 109, 60, 20, 28, 194,
    195, 104, 73, 30, 31, 193,
    185, 99, 86, 41, 35, 192,
    175, 93, 101, 52, 39, 190,
    164, 87, 116, 64, 43, 189,
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,
  };

  // check result
  for (std::uint32_t i = 0; i < output->width ; ++i)
  {
    EXPECT_EQ ((*output) (i, 0).r, output_results[i * 6 + 0]);
    EXPECT_EQ ((*output) (i, 0).g, output_results[i * 6 + 1]);
    EXPECT_EQ ((*output) (i, 0).b, output_results[i * 6 + 2]);
    EXPECT_EQ ((*output) (i, 47).r, output_results[i * 6 + 3]);
    EXPECT_EQ ((*output) (i, 47).g, output_results[i * 6 + 4]);
    EXPECT_EQ ((*output) (i, 47).b, output_results[i * 6 + 5]);
  }
}

int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
