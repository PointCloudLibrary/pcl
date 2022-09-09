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
  auto lower_index = static_cast<std::size_t>((value - lower_bound) / step_size);
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
  for (std::uint32_t r = 0; r < input->height; r++)
  {
    float y = 40.0f + 40.0f / 65.0f * static_cast<float>(r);
    float z = (r % 2 == 0) ? 1.0f : -1.0f;
    for(std::uint32_t c = 0; c < input->width; c++)
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
  std::array<float, 128> output_intensity {
    // row 0            row 47
      0.00000000f,     0.00000000f,
      0.00000000f,     0.00000000f,
      0.00000000f,     0.00000000f,
    105.10825348f,   132.03129578f,
    104.66084290f,   131.58390808f,
    104.18517303f,   131.10823059f,
    103.73776245f,   130.66082764f,
    103.26208496f,   130.18515015f,
    102.81467438f,   129.73773193f,
    102.33901978f,   129.26208496f,
    101.89160919f,   128.81466675f,
    101.41593933f,   128.33900452f,
    100.96853638f,   127.89158630f,
    100.49286652f,   127.41592407f,
    100.04545593f,   126.96851349f,
     99.56979370f,   126.49285126f,
     99.12238312f,   126.04543304f,
     98.64672089f,   125.56977844f,
     98.19929504f,   125.12236786f,
     97.72364044f,   124.64669800f,
     97.27622223f,   124.19929504f,
     96.80056000f,   123.72362518f,
     96.35314941f,   123.27621460f,
     95.87748718f,   122.80054474f,
     95.43006897f,   122.35313416f,
     94.95440674f,   121.87747192f,
     94.50699615f,   121.43005371f,
     94.03132629f,   120.95440674f,
     93.58392334f,   120.50697327f,
     93.10826111f,   120.03132629f,
     92.66084290f,   119.58390808f,
     92.18517303f,   119.10823822f,
     91.73777008f,   118.66082764f,
     91.26210785f,   118.18516541f,
     90.81468964f,   117.73775482f,
     90.33903503f,   117.26208496f,
     89.89160156f,   116.81467438f,
     89.41595459f,   116.33901215f,
     88.96853638f,   115.89160919f,
     88.49288177f,   115.41593170f,
     88.04545593f,   114.96851349f,
     87.56979370f,   114.49285889f,
     87.12238312f,   114.04544830f,
     86.64672089f,   113.56979370f,
     86.19931030f,   113.12236023f,
     85.72364044f,   112.64670563f,
     85.27622986f,   112.19928741f,
     84.80056763f,   111.72362518f,
     84.35315704f,   111.27622223f,
     83.87749481f,   110.80056000f,
     83.43008423f,   110.35313416f,
     82.95442200f,   109.87748718f,
     82.50699615f,   109.43006134f,
     82.03134155f,   108.95440674f,
     81.58392334f,   108.50699615f,
     81.10826874f,   108.03132629f,
     80.66085052f,   107.58391571f,
     80.18519592f,   107.10824585f,
     79.73777008f,   106.66084290f,
     79.26210785f,   106.18517303f,
     78.81469727f,   105.73775482f,
      0.00000000f,     0.00000000f,
      0.00000000f,     0.00000000f,
      0.00000000f,     0.00000000f,
  };

  // check result
  for (std::uint32_t i = 0; i < output->width ; ++i)
  {
    EXPECT_FLOAT_EQ ((*output) (i, 0).intensity, output_intensity[i * 2 + 0]);
    EXPECT_FLOAT_EQ ((*output) (i, 47).intensity, output_intensity[i * 2 + 1]);
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
      float z = x1 * std::exp(-(x1 * x1 + y1 * y1)) * 2.5f + std::sin(x2) * std::sin(y2);
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
  std::array<RGB, 128> output_results {
    //  row 0                 row 47
    RGB(0, 0, 0),         RGB(0, 0, 0),
    RGB(0, 0, 0),         RGB(0, 0, 0),
    RGB(0, 0, 0),         RGB(0, 0, 0),
    RGB(175, 206, 68),    RGB(100, 173, 126),
    RGB(187, 210, 67),    RGB(88, 166, 143),
    RGB(199, 214, 65),    RGB(77, 160, 159),
    RGB(210, 218, 64),    RGB(66, 154, 175),
    RGB(220, 222, 63),    RGB(55, 149, 189),
    RGB(230, 225, 61),    RGB(46, 144, 202),
    RGB(238, 228, 60),    RGB(38, 139, 214),
    RGB(246, 231, 59),    RGB(32, 134, 221),
    RGB(251, 230, 58),    RGB(31, 128, 221),
    RGB(254, 225, 56),    RGB(33, 122, 218),
    RGB(254, 218, 54),    RGB(34, 117, 214),
    RGB(254, 213, 53),    RGB(36, 114, 212),
    RGB(254, 209, 52),    RGB(37, 112, 210),
    RGB(254, 208, 52),    RGB(37, 111, 209),
    RGB(254, 210, 52),    RGB(37, 111, 210),
    RGB(254, 214, 53),    RGB(36, 113, 211),
    RGB(254, 220, 55),    RGB(35, 116, 213),
    RGB(253, 226, 56),    RGB(33, 121, 217),
    RGB(250, 230, 58),    RGB(32, 126, 220),
    RGB(244, 230, 59),    RGB(32, 132, 221),
    RGB(237, 228, 60),    RGB(36, 137, 216),
    RGB(228, 225, 62),    RGB(44, 142, 205),
    RGB(219, 222, 63),    RGB(53, 148, 192),
    RGB(209, 218, 64),    RGB(64, 153, 177),
    RGB(198, 214, 65),    RGB(75, 159, 161),
    RGB(187, 210, 67),    RGB(87, 166, 145),
    RGB(175, 206, 68),    RGB(99, 172, 127),
    RGB(163, 202, 70),    RGB(112, 179, 110),
    RGB(151, 197, 72),    RGB(125, 186, 93),
    RGB(139, 192, 79),    RGB(138, 192, 79),
    RGB(127, 187, 91),    RGB(152, 198, 73),
    RGB(115, 181, 106),   RGB(165, 203, 70),
    RGB(103, 175, 122),   RGB(179, 207, 68),
    RGB(92, 169, 138),    RGB(192, 212, 66),
    RGB(81, 163, 153),    RGB(204, 216, 64),
    RGB(71, 157, 167),    RGB(216, 220, 63),
    RGB(61, 152, 180),    RGB(227, 224, 61),
    RGB(52, 147, 193),    RGB(238, 228, 60),
    RGB(44, 143, 204),    RGB(246, 229, 59),
    RGB(37, 139, 214),    RGB(252, 226, 57),
    RGB(33, 135, 220),    RGB(254, 218, 54),
    RGB(31, 131, 223),    RGB(254, 207, 51),
    RGB(31, 127, 221),    RGB(254, 198, 49),
    RGB(32, 124, 219),    RGB(254, 192, 48),
    RGB(33, 122, 218),    RGB(254, 188, 47),
    RGB(33, 121, 217),    RGB(254, 187, 46),
    RGB(33, 122, 218),    RGB(254, 189, 47),
    RGB(32, 123, 219),    RGB(254, 194, 48),
    RGB(31, 126, 221),    RGB(254, 201, 50),
    RGB(31, 130, 223),    RGB(254, 210, 52),
    RGB(32, 134, 221),    RGB(254, 220, 55),
    RGB(36, 138, 215),    RGB(251, 228, 57),
    RGB(43, 142, 206),    RGB(244, 229, 59),
    RGB(51, 146, 195),    RGB(235, 227, 61),
    RGB(60, 151, 182),    RGB(225, 224, 62),
    RGB(70, 156, 169),    RGB(214, 220, 63),
    RGB(80, 162, 154),    RGB(203, 216, 65),
    RGB(91, 168, 139),    RGB(191, 212, 66),
    RGB(0, 0, 0),         RGB(0, 0, 0),
    RGB(0, 0, 0),         RGB(0, 0, 0),
    RGB(0, 0, 0),         RGB(0, 0, 0),
  };

  // check result
  for (std::uint32_t i = 0; i < output->width ; ++i)
  {
    EXPECT_EQ ((*output) (i, 0).r, output_results[i * 2 + 0].r);
    EXPECT_EQ ((*output) (i, 0).g, output_results[i * 2 + 0].g);
    EXPECT_EQ ((*output) (i, 0).b, output_results[i * 2 + 0].b);
    EXPECT_EQ ((*output) (i, 47).r, output_results[i * 2 + 1].r);
    EXPECT_EQ ((*output) (i, 47).g, output_results[i * 2 + 1].g);
    EXPECT_EQ ((*output) (i, 47).b, output_results[i * 2 + 1].b);
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
      float z = x1 * std::exp(-(x1 * x1 + y1 * y1)) * 2.5f + std::sin(x2) * std::sin(y2);
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
  std::array<RGB, 128> output_results {
    //  row 0                row 47
    RGB(0, 0, 0),        RGB(0, 0, 0),
    RGB(0, 0, 0),        RGB(0, 0, 0),
    RGB(0, 0, 0),        RGB(0, 0, 0),
    RGB(80, 49, 187),    RGB(155, 82, 129),
    RGB(68, 45, 188),    RGB(167, 89, 112),
    RGB(56, 41, 190),    RGB(178, 95, 96),
    RGB(45, 37, 191),    RGB(189, 101, 80),
    RGB(35, 33, 192),    RGB(200, 106, 66),
    RGB(25, 30, 194),    RGB(209, 111, 53),
    RGB(17, 27, 195),    RGB(217, 116, 41),
    RGB(9, 24, 196),     RGB(223, 121, 34),
    RGB(4, 25, 197),     RGB(224, 127, 34),
    RGB(1, 30, 199),     RGB(222, 133, 37),
    RGB(1, 37, 201),     RGB(221, 138, 41),
    RGB(1, 42, 202),     RGB(219, 141, 43),
    RGB(0, 46, 203),     RGB(218, 143, 45),
    RGB(0, 47, 203),     RGB(218, 144, 46),
    RGB(0, 45, 203),     RGB(218, 144, 45),
    RGB(1, 41, 202),     RGB(219, 142, 44),
    RGB(1, 35, 200),     RGB(220, 139, 42),
    RGB(2, 29, 199),     RGB(222, 134, 38),
    RGB(5, 25, 197),     RGB(223, 129, 35),
    RGB(11, 25, 196),    RGB(223, 123, 34),
    RGB(18, 27, 195),    RGB(219, 118, 39),
    RGB(27, 30, 193),    RGB(211, 113, 50),
    RGB(36, 33, 192),    RGB(202, 107, 63),
    RGB(46, 37, 191),    RGB(191, 102, 78),
    RGB(57, 41, 190),    RGB(180, 96, 94),
    RGB(68, 45, 188),    RGB(168, 89, 110),
    RGB(80, 49, 187),    RGB(156, 83, 128),
    RGB(92, 53, 185),    RGB(143, 76, 145),
    RGB(104, 58, 183),   RGB(130, 69, 162),
    RGB(116, 63, 176),   RGB(117, 63, 176),
    RGB(128, 68, 164),   RGB(103, 57, 182),
    RGB(140, 74, 149),   RGB(90, 52, 185),
    RGB(152, 80, 133),   RGB(76, 48, 187),
    RGB(163, 86, 117),   RGB(63, 43, 189),
    RGB(174, 92, 102),   RGB(51, 39, 191),
    RGB(184, 98, 88),    RGB(39, 35, 192),
    RGB(194, 103, 75),   RGB(28, 31, 193),
    RGB(203, 108, 62),   RGB(17, 27, 195),
    RGB(211, 112, 51),   RGB(9, 26, 196),
    RGB(218, 116, 41),   RGB(3, 29, 198),
    RGB(222, 120, 35),   RGB(1, 37, 201),
    RGB(224, 124, 32),   RGB(1, 48, 204),
    RGB(224, 128, 34),   RGB(0, 57, 206),
    RGB(223, 131, 36),   RGB(0, 63, 207),
    RGB(222, 133, 37),   RGB(0, 67, 208),
    RGB(222, 134, 38),   RGB(0, 68, 209),
    RGB(222, 133, 37),   RGB(0, 66, 208),
    RGB(223, 132, 36),   RGB(0, 61, 207),
    RGB(224, 129, 34),   RGB(1, 54, 205),
    RGB(224, 125, 32),   RGB(1, 45, 203),
    RGB(223, 121, 34),   RGB(1, 35, 200),
    RGB(219, 117, 40),   RGB(4, 27, 198),
    RGB(212, 113, 49),   RGB(11, 26, 196),
    RGB(204, 109, 60),   RGB(20, 28, 194),
    RGB(195, 104, 73),   RGB(30, 31, 193),
    RGB(185, 99, 86),    RGB(41, 35, 192),
    RGB(175, 93, 101),   RGB(52, 39, 190),
    RGB(164, 87, 116),   RGB(64, 43, 189),
    RGB(0, 0, 0),        RGB(0, 0, 0),
    RGB(0, 0, 0),        RGB(0, 0, 0),
    RGB(0, 0, 0),        RGB(0, 0, 0),
  };

  // check result
  for (std::uint32_t i = 0; i < output->width ; ++i)
  {
    EXPECT_EQ ((*output) (i, 0).r, output_results[i * 2 + 0].r);
    EXPECT_EQ ((*output) (i, 0).g, output_results[i * 2 + 0].g);
    EXPECT_EQ ((*output) (i, 0).b, output_results[i * 2 + 0].b);
    EXPECT_EQ ((*output) (i, 47).r, output_results[i * 2 + 1].r);
    EXPECT_EQ ((*output) (i, 47).g, output_results[i * 2 + 1].g);
    EXPECT_EQ ((*output) (i, 47).b, output_results[i * 2 + 1].b);
  }
}

int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
