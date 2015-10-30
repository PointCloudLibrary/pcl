/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception, Inc.
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

#include <pcl/point_types.h>
#include <pcl/common/colors.h>

/// Lookup table
static const unsigned char GLASBEY_LUT[] =
{
  77 , 175, 74 ,
  228, 26 , 28 ,
  55 , 126, 184,
  152, 78 , 163,
  255, 127, 0  ,
  255, 255, 51 ,
  166, 86 , 40 ,
  247, 129, 191,
  153, 153, 153,
  0  , 0  , 255,
  255, 0  , 255,
  0  , 255, 248,
  0  , 255, 0  ,
  0  , 101, 255,
  255, 255, 180,
  52 , 68 , 1  ,
  0  , 0  , 68 ,
  96 , 0  , 41 ,
  158, 147, 0  ,
  116, 0  , 185,
  255, 0  , 114,
  0  , 149, 125,
  209, 186, 255,
  255, 183, 156,
  240, 0  , 174,
  208, 255, 245,
  0  , 255, 176,
  170, 255, 93 ,
  0  , 207, 255,
  255, 190, 1  ,
  241, 117, 255,
  52 , 74 , 167,
  150, 166, 103,
  255, 114, 114,
  171, 100, 109,
  161, 0  , 41 ,
  160, 135, 255,
  105, 86 , 121,
  178, 21 , 105,
  0  , 3  , 123,
  255, 221, 236,
  160, 238, 173,
  237, 161, 77 ,
  0  , 141, 255,
  0  , 97 , 109,
  1  , 238, 98 ,
  81 , 0  , 78 ,
  128, 103, 66 ,
  0  , 108, 44 ,
  209, 224, 94 ,
  155, 0  , 255,
  0  , 45 , 223,
  88 , 28 , 0  ,
  166, 2  , 162,
  165, 205, 239,
  84 , 121, 0  ,
  76 , 109, 80 ,
  122, 180, 0  ,
  104, 204, 204,
  145, 95 , 255,
  214, 208, 177,
  185, 130, 176,
  130, 120, 194,
  128, 96 , 0  ,
  247, 161, 255,
  10 , 65 , 119,
  232, 102, 54 ,
  7  , 191, 131,
  105, 54 , 171,
  10 , 177, 0  ,
  215, 191, 105,
  198, 66 , 249,
  140, 255, 145,
  135, 60 , 105,
  254, 170, 191,
  130, 173, 255,
  161, 35 , 0  ,
  197, 255, 0  ,
  40 , 153, 180,
  215, 83 , 185,
  255, 77 , 161,
  128, 175, 147,
  216, 91 , 124,
  193, 144, 91 ,
  210, 196, 0  ,
  232, 39 , 73 ,
  76 , 52 , 116,
  159, 206, 110,
  138, 147, 187,
  140, 5  , 114,
  0  , 56 , 183,
  191, 105, 0  ,
  83 , 58 , 0  ,
  94 , 224, 0  ,
  121, 99 , 99 ,
  212, 123, 104,
  89 , 160, 99 ,
  146, 58 , 54 ,
  231, 46 , 215,
  93 , 245, 200,
  191, 147, 133,
  255, 211, 89 ,
  171, 77 , 214,
  0  , 121, 0  ,
  60 , 14 , 107,
  255, 82 , 1  ,
  112, 115, 43 ,
  0  , 172, 245,
  255, 184, 240,
  1  , 210, 111,
  203, 151, 0  ,
  95 , 114, 129,
  183, 215, 17 ,
  80 , 110, 231,
  201, 25 , 87 ,
  218, 250, 203,
  255, 148, 103,
  255, 217, 163,
  198, 172, 199,
  78 , 139, 135,
  197, 255, 134,
  38 , 0  , 165,
  197, 208, 211,
  193, 117, 225,
  111, 0  , 128,
  147, 255, 238,
  125, 62 , 254,
  112, 213, 78 ,
  198, 76 , 61 ,
  155, 48 , 82 ,
  0  , 199, 176,
  118, 6  , 0  ,
  2  , 106, 192,
  140, 167, 49 ,
  189, 81 , 145,
  254, 162, 38 ,
  134, 138, 106,
  0  , 68 , 17 ,
  122, 73 , 61 ,
  255, 251, 239,
  127, 94 , 193,
  181, 140, 48 ,
  66 , 235, 255,
  189, 140, 218,
  190, 0  , 138,
  132, 177, 185,
  90 , 54 , 202,
  0  , 35 , 131,
  251, 139, 149,
  74 , 0  , 225,
  0  , 106, 90 ,
  106, 199, 155,
  104, 169, 217,
  255, 239, 134,
  44 , 94 , 130,
  126, 0  , 78 ,
  196, 214, 145,
  160, 238, 255,
  222, 215, 255,
  255, 87 , 126,
  170, 161, 255,
  81 , 120, 60 ,
  255, 242, 91 ,
  168, 130, 145,
  158, 153, 64 ,
  211, 123, 156,
  255, 0  , 3  ,
  210, 118, 197,
  0  , 41 , 111,
  198, 178, 125,
  211, 255, 169,
  109, 215, 130,
  41 , 90 , 0  ,
  235, 193, 183,
  114, 58 , 0  ,
  140, 96 , 155,
  163, 223, 193,
  255, 142, 63 ,
  66 , 155, 1  ,
  83 , 96 , 152,
  106, 133, 230,
  255, 85 , 72 ,
  141, 216, 0  ,
  162, 102, 73 ,
  79 , 0  , 146,
  190, 0  , 30 ,
  165, 0  , 193,
  81 , 84 , 255,
  0  , 148, 74 ,
  203, 0  , 255,
  121, 54 , 71 ,
  215, 255, 97 ,
  163, 178, 0  ,
  111, 154, 68 ,
  120, 93 , 222,
  254, 187, 126,
  112, 0  , 27 ,
  204, 59 , 0  ,
  0  , 165, 167,
  151, 255, 0  ,
  104, 41 , 124,
  138, 89 , 113,
  255, 94 , 224,
  86 , 91 , 48 ,
  75 , 255, 76 ,
  204, 190, 67 ,
  255, 224, 0  ,
  49 , 126, 85 ,
  145, 196, 135,
  187, 64 , 79 ,
  255, 130, 233,
  205, 127, 68 ,
  105, 195, 223,
  161, 213, 81 ,
  165, 183, 240,
  125, 255, 180,
  126, 255, 111,
  67 , 255, 145,
  154, 138, 83 ,
  46 , 145, 231,
  118, 121, 0  ,
  154, 2  , 222,
  185, 119, 255,
  255, 0  , 62 ,
  131, 28 , 170,
  177, 70 , 183,
  217, 0  , 115,
  186, 196, 95 ,
  97 , 46 , 97 ,
  84 , 134, 167,
  81 , 54 , 145,
  107, 117, 107,
  51 , 15 , 80 ,
  215, 139, 143,
  255, 247, 203,
  255, 86 , 192,
  153, 91 , 0  ,
  255, 1  , 156,
  183, 79 , 19 ,
  235, 255, 146,
  211, 1  , 224,
  178, 167, 144,
  217, 97 , 0  ,
  134, 91 , 38 ,
  175, 151, 206,
  0  , 182, 63 ,
  210, 40 , 179,
  2  , 213, 42 ,
  94 , 84 , 160,
  136, 48 , 0  ,
  255, 110, 163,
  144, 121, 157,
  153, 61 , 225,
  237, 87 , 255,
  87 , 24 , 206,
  117, 143, 207,
};

/// Number of colors in Glasbey lookup table
static const unsigned int GLASBEY_LUT_SIZE = sizeof (GLASBEY_LUT) / (sizeof (GLASBEY_LUT[0]) * 3);

pcl::RGB
pcl::GlasbeyLUT::at (unsigned int color_id)
{
  assert (color_id < GLASBEY_LUT_SIZE);
  pcl::RGB color;
  color.r = GLASBEY_LUT[color_id * 3 + 0];
  color.g = GLASBEY_LUT[color_id * 3 + 1];
  color.b = GLASBEY_LUT[color_id * 3 + 2];
  return (color);
}

size_t
pcl::GlasbeyLUT::size ()
{
  return GLASBEY_LUT_SIZE;
}

const unsigned char*
pcl::GlasbeyLUT::data ()
{
  return GLASBEY_LUT;
}

pcl::RGB
pcl::getRandomColor (double min, double max)
{
  double sum;
  static unsigned stepRGBA = 100;
  double r, g, b;
  do
  {
    sum = 0;
    r = (rand () % stepRGBA) / static_cast<double> (stepRGBA);
    while ((g = (rand () % stepRGBA) / static_cast<double> (stepRGBA)) == r) {}
    while (((b = (rand () % stepRGBA) / static_cast<double> (stepRGBA)) == r) && (b == g)) {}
    sum = r + g + b;
  }
  while (sum <= min || sum >= max);
  pcl::RGB color;
  color.r = uint8_t (r * 255.0);
  color.g = uint8_t (g * 255.0);
  color.b = uint8_t (b * 255.0);
  return (color);
}
