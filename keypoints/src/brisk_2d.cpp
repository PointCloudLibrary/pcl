/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (C) 2011, The Autonomous Systems Lab (ASL), ETH Zurich, 
 *                      Stefan Leutenegger, Simon Lynen and Margarita Chli.
 *  Copyright (c) 2012-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and threshold_inary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in threshold_inary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may threshold_e used to endorse or promote products derived
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

#include <pcl/pcl_config.h>
#include <pcl/pcl_macros.h>
#include <pcl/keypoints/brisk_2d.h>
#include <pcl/point_types.h>
#include <pcl/impl/instantiate.hpp>
#if defined(__SSSE3__) && !defined(__i386__)
#include <tmmintrin.h>
#include <emmintrin.h>
#endif

/////////////////////////////////////////////////////////////////////////////////////////
// construct telling the octaves number:
pcl::keypoints::brisk::ScaleSpace::ScaleSpace (int octaves)
  : safety_factor_ (1.0)
  , basic_size_ (12.0)
{
  if (octaves == 0)
    layers_ = 1;
  else
    layers_ = uint8_t (2 * octaves);
}

/////////////////////////////////////////////////////////////////////////////////////////
pcl::keypoints::brisk::ScaleSpace::~ScaleSpace ()
{
}

/////////////////////////////////////////////////////////////////////////////////////////
// construct the image pyramids
void 
pcl::keypoints::brisk::ScaleSpace::constructPyramid (
    const std::vector<unsigned char>& image, int width, int height)
{
  // set correct size:
  pyramid_.clear ();

  // fill the pyramid
  pyramid_.push_back (pcl::keypoints::brisk::Layer (std::vector<unsigned char> (image), width, height));
  if (layers_ > 1)
    pyramid_.push_back (pcl::keypoints::brisk::Layer (pyramid_.back (), pcl::keypoints::brisk::Layer::CommonParams::TWOTHIRDSAMPLE));
  const int octaves2 = layers_;

  for (int i = 2; i < octaves2; i += 2)
  {
    pyramid_.push_back (pcl::keypoints::brisk::Layer (pyramid_[i-2], pcl::keypoints::brisk::Layer::CommonParams::HALFSAMPLE));
    pyramid_.push_back (pcl::keypoints::brisk::Layer (pyramid_[i-1], pcl::keypoints::brisk::Layer::CommonParams::HALFSAMPLE));
  }
}

/////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::keypoints::brisk::ScaleSpace::getKeypoints (
    const int threshold, 
    std::vector<pcl::PointWithScale, Eigen::aligned_allocator<pcl::PointWithScale> >& keypoints)
{
  // make sure keypoints is empty
  //keypoints.resize (0);
  keypoints.clear ();
  keypoints.reserve (2000);

  // assign thresholds
  threshold_ = uint8_t (threshold);
  safe_threshold_ = uint8_t (threshold_ * safety_factor_);
  std::vector<std::vector<pcl::PointUV, Eigen::aligned_allocator<pcl::PointUV> > > agast_points;
  agast_points.resize (layers_);

  // go through the octaves and intra layers and calculate fast corner scores:
  for (uint8_t i = 0; i < layers_; i++)
  {
    // call OAST16_9 without nms
    pcl::keypoints::brisk::Layer& l = pyramid_[i];
    l.getAgastPoints (safe_threshold_, agast_points[i]);
  }

  if (layers_ == 1)
  {
    // just do a simple 2d subpixel refinement...
    const int num = int (agast_points[0].size ());
    for (int n = 0; n < num; n++)
    {
      const pcl::PointUV& point = agast_points.at (0)[n];
      // first check if it is a maximum:
      if (!isMax2D (0, int (point.u), int (point.v)))
        continue;

      // let's do the subpixel and float scale refinement:
      pcl::keypoints::brisk::Layer& l = pyramid_[0];
      int s_0_0 = l.getAgastScore (point.u-1, point.v-1, 1);
      int s_1_0 = l.getAgastScore (point.u,   point.v-1, 1);
      int s_2_0 = l.getAgastScore (point.u+1, point.v-1, 1);
      int s_2_1 = l.getAgastScore (point.u+1, point.v,   1);
      int s_1_1 = l.getAgastScore (point.u,   point.v,   1);
      int s_0_1 = l.getAgastScore (point.u-1, point.v,   1);
      int s_0_2 = l.getAgastScore (point.u-1, point.v+1, 1);
      int s_1_2 = l.getAgastScore (point.u,   point.v+1, 1);
      int s_2_2 = l.getAgastScore (point.u+1, point.v+1, 1);
      float delta_x, delta_y;
      float max = subpixel2D (s_0_0, s_0_1, s_0_2,
                              s_1_0, s_1_1, s_1_2,
                              s_2_0, s_2_1, s_2_2,
                              delta_x, delta_y);

      // store:
      keypoints.push_back (pcl::PointWithScale (point.u + delta_x, point.v + delta_y, 0.0f, basic_size_, -1, max, 0));
    }
    return;
  }

  float x, y, scale, score;
  for (uint8_t i = 0; i < layers_; i++)
  {
    pcl::keypoints::brisk::Layer& l = pyramid_[i];
    const int num = int (agast_points[i].size ());
    
    if (i == layers_ - 1)
    {
      for (int n = 0; n < num; n++)
      {
        const pcl::PointUV& point = agast_points.at (i)[n];

        // consider only 2D maxima...
        if (!isMax2D (i, int (point.u), int (point.v)))
          continue;

        bool ismax;
        float dx, dy;
        getScoreMaxBelow (i, int (point.u), int (point.v),
                          l.getAgastScore (point.u, point.v, safe_threshold_), ismax,
                          dx, dy);
        if (!ismax)
          continue;

        // get the patch on this layer:
        int s_0_0 = l.getAgastScore (point.u-1, point.v-1, 1);
        int s_1_0 = l.getAgastScore (point.u,   point.v-1, 1);
        int s_2_0 = l.getAgastScore (point.u+1, point.v-1, 1);
        int s_2_1 = l.getAgastScore (point.u+1, point.v,   1);
        int s_1_1 = l.getAgastScore (point.u,   point.v,   1);
        int s_0_1 = l.getAgastScore (point.u-1, point.v,   1);
        int s_0_2 = l.getAgastScore (point.u-1, point.v+1, 1);
        int s_1_2 = l.getAgastScore (point.u,   point.v+1, 1);
        int s_2_2 = l.getAgastScore (point.u+1, point.v+1, 1);
        float delta_x, delta_y;
        float max = subpixel2D (s_0_0, s_0_1, s_0_2,
                                s_1_0, s_1_1, s_1_2,
                                s_2_0, s_2_1, s_2_2,
                                delta_x, delta_y);

        // store:
        keypoints.push_back (pcl::PointWithScale ((point.u + delta_x) * l.getScale () + l.getOffset (),     // x
                                                  (point.v + delta_y) * l.getScale () + l.getOffset (),     // y
                                                  0.0f,                                           // z
                                                  basic_size_ * l.getScale (),                         // size
                                                  -1,                                             // angle
                                                  max,                                            // response
                                                  i));                                            // octave
      }
    }
    else
    {
      // not the last layer:
      for (int n = 0; n < num; n++)
      {
        const pcl::PointUV& point = agast_points.at (i)[n];

        // first check if it is a maximum:
        if (!isMax2D (i, int (point.u), int (point.v)))
        {
          continue;
        }

        // let's do the subpixel and float scale refinement:
        bool ismax;
        score = refine3D (i, int (point.u), int (point.v), x, y, scale, ismax);

        if (!ismax)
          continue;

        // finally store the detected keypoint:
        if (score > float (threshold_))
        {
          keypoints.push_back (pcl::PointWithScale (x, y, 0.0f, basic_size_ * scale, -1, score, i));
        }
      }
    }
  }
}

/////////////////////////////////////////////////////////////////////////////////////////
// interpolated score access with recalculation when needed:
int 
pcl::keypoints::brisk::ScaleSpace::getScoreAbove (
    const uint8_t layer, const int x_layer, const int y_layer)
{
  assert (layer < layers_ - 1);
  pcl::keypoints::brisk::Layer& l = pyramid_[layer+1];
  if (layer % 2 == 0)
  { // octave
    const int sixths_x = 4 * x_layer - 1;
    const int x_above  = sixths_x / 6;
    const int sixths_y = 4 * y_layer - 1;
    const int y_above = sixths_y / 6;
    const int r_x = (sixths_x % 6);
    const int r_x_1 =6 - r_x;
    const int r_y = (sixths_y % 6);
    const int r_y_1 = 6 - r_y;
    uint8_t score = static_cast<uint8_t> (
                    0xFF & ((r_x_1 * r_y_1 * l.getAgastScore (x_above,     y_above,     1) +
                             r_x   * r_y_1 * l.getAgastScore (x_above + 1, y_above,     1) +
                             r_x_1 * r_y   * l.getAgastScore (x_above,     y_above + 1, 1) +
                             r_x   * r_y   * l.getAgastScore (x_above + 1, y_above + 1, 1) + 18) / 36));
 
    return (score);
  }
  else
  { // intra
    const int eighths_x = 6 * x_layer - 1;
    const int x_above = eighths_x / 8;
    const int eighths_y = 6 * y_layer - 1;
    const int y_above = eighths_y / 8;
    const int r_x = (eighths_x % 8);
    const int r_x_1 = 8 - r_x;
    const int r_y = (eighths_y % 8);
    const int r_y_1 = 8 - r_y;
    uint8_t score = static_cast<uint8_t> (
                    0xFF & ((r_x_1 * r_y_1 * l.getAgastScore (x_above,     y_above,     1) +
                             r_x   * r_y_1  * l.getAgastScore (x_above + 1, y_above,     1) +
                             r_x_1 * r_y    * l.getAgastScore (x_above ,    y_above + 1, 1) +
                             r_x   * r_y    * l.getAgastScore (x_above + 1, y_above + 1, 1) + 32) / 64));
    return (score);
  }
}

/////////////////////////////////////////////////////////////////////////////////////////
int 
pcl::keypoints::brisk::ScaleSpace::getScoreBelow (
    const uint8_t layer, const int x_layer, const int y_layer)
{
  assert (layer);
  pcl::keypoints::brisk::Layer& l = pyramid_[layer-1];
  int sixth_x;
  int quarter_x;
  float xf;
  int sixth_y;
  int quarter_y;
  float yf;

  // scaling:
  float offs;
  float area;
  int scaling;
  int scaling2;

  if (layer % 2 == 0)
  { // octave
    sixth_x = 8 * x_layer + 1;
    xf = float (sixth_x) / 6.0f;
    sixth_y = 8 * y_layer + 1;
    yf = float (sixth_y) / 6.0f;

    // scaling:
    offs = 2.0f / 3.0f;
    area = 4.0f * offs * offs;
    scaling  = static_cast<int> (4194304.0f / area);
    scaling2 = static_cast<int> (float (scaling) * area);
  }
  else
  {
    quarter_x = 6 * x_layer + 1;
    xf = float (quarter_x) / 4.0f;
    quarter_y = 6 * y_layer + 1;
    yf = float (quarter_y) / 4.0f;

    // scaling:
    offs = 3.0f / 4.0f;
    area = 4.0f * offs * offs;
    scaling  = static_cast<int> (4194304.0f / area);
    scaling2 = static_cast<int> (float (scaling) * area);
  }

  // calculate borders
  const float x_1 = xf - offs;
  const float x1  = xf + offs;
  const float y_1 = yf - offs;
  const float y1  = yf + offs;

  const int x_left   = int (x_1 + 0.5);
  const int y_top    = int (y_1 + 0.5);
  const int x_right  = int (x1  + 0.5);
  const int y_bottom = int (y1  + 0.5);

  // overlap area - multiplication factors:
  const float r_x_1 = float (x_left) - x_1 + 0.5f;
  const float r_y_1 = float (y_top) - y_1  + 0.5f;
  const float r_x1  = x1 - float (x_right) + 0.5f;
  const float r_y1  = y1 - float (y_bottom) + 0.5f;
  const int dx  = x_right - x_left - 1;
  const int dy = y_bottom - y_top - 1;
  const int A = static_cast<int> ((r_x_1 * r_y_1) * float (scaling));
  const int B = static_cast<int> ((r_x1  * r_y_1) * float (scaling));
  const int C = static_cast<int> ((r_x1  * r_y1)  * float (scaling));
  const int D = static_cast<int> ((r_x_1 * r_y1)  * float (scaling));
  const int r_x_1_i = static_cast<int> (r_x_1 * float (scaling));
  const int r_y_1_i = static_cast<int> (r_y_1 * float (scaling));
  const int r_x1_i  = static_cast<int> (r_x1  * float (scaling));
  const int r_y1_i  = static_cast<int> (r_y1  * float (scaling));

  // first row:
  int ret_val = A * int (l.getAgastScore (x_left, y_top, 1));
  for (int X = 1; X <= dx; X++)
    ret_val += r_y_1_i * int (l.getAgastScore (x_left + X, y_top, 1));

  ret_val += B * int (l.getAgastScore (x_left + dx + 1, y_top, 1));
  // middle ones:
  for (int Y = 1; Y <= dy; Y++)
  {
    ret_val += r_x_1_i * int (l.getAgastScore (x_left, y_top + Y, 1));

    for (int X = 1; X <= dx; X++)
      ret_val += int (l.getAgastScore (x_left + X, y_top + Y, 1)) * scaling;

    ret_val += r_x1_i * int (l.getAgastScore (x_left + dx + 1, y_top + Y, 1));
  }
  // last row:
  ret_val += D * int (l.getAgastScore (x_left, y_top + dy + 1, 1));
  for (int X = 1; X <= dx; X++)
    ret_val += r_y1_i * int (l.getAgastScore (x_left + X, y_top + dy + 1, 1));

  ret_val += C * int (l.getAgastScore (x_left + dx + 1, y_top + dy + 1, 1));

  return ((ret_val + scaling2 / 2) / scaling2);
}

/////////////////////////////////////////////////////////////////////////////////////////
bool 
pcl::keypoints::brisk::ScaleSpace::isMax2D (
    const uint8_t layer, const int x_layer, const int y_layer)
{
  const std::vector<unsigned char>& scores = pyramid_[layer].getScores ();
  const int scorescols = pyramid_[layer].getImageWidth ();
  const unsigned char* data = &scores[0] + y_layer * scorescols + x_layer;

  // decision tree:
  const unsigned char center = (*data);
  data--;
  const unsigned char s_10 = *data;
  if (center < s_10) return (false);
  data += 2;
  const unsigned char s10 = *data;
  if (center < s10) return (false);
  data -= (scorescols + 1);
  const unsigned char s0_1 = *data;
  if (center < s0_1) return (false);
  data += 2 * scorescols;
  const unsigned char s01 = *data;
  if (center < s01) return (false);
  data--;
  const unsigned char s_11 =* data;
  if (center < s_11) return (false);
  data += 2;
  const unsigned char s11 =* data;
  if (center < s11) return (false);
  data -= 2 * scorescols;
  const unsigned char s1_1 =* data;
  if (center < s1_1) return (false);
  data -= 2;
  const unsigned char s_1_1 =* data;
  if (center < s_1_1) return (false);

  // reject neighbor maxima
  std::vector<int> delta;
  // put together a list of 2d-offsets to where the maximum is also reached
  if (center == s_1_1) 
  {
    delta.push_back (-1);
    delta.push_back (-1);
  }
  if (center == s0_1) 
  {
    delta.push_back (0);
    delta.push_back (-1);
  }
  if (center == s1_1) 
  {
    delta.push_back (1);
    delta.push_back (-1);
  }
  if (center == s_10) 
  {
    delta.push_back (-1);
    delta.push_back (0);
  }
  if (center == s10) 
  {
    delta.push_back (1);
    delta.push_back (0);
  }
  if (center == s_11) 
  {
    delta.push_back (-1);
    delta.push_back (1);
  }
  if (center == s01) 
  {
    delta.push_back (0);
    delta.push_back (1);
  }
  if (center == s11) 
  {
    delta.push_back (1);
    delta.push_back (1);
  }
  
  unsigned int deltasize = static_cast<unsigned int> (delta.size ());
  if (deltasize != 0)
  {
    // in this case, we have to analyze the situation more carefully:
    // the values are gaussian blurred and then we really decide
    data = &scores[0] + y_layer * scorescols + x_layer;
    int smoothedcenter = 4 * center + 2 * (s_10 + s10 + s0_1 + s01) + s_1_1 + s1_1 + s_11 + s11;
    
    for (unsigned int i = 0; i < deltasize; i+= 2)
    {
      data = &scores[0] + (y_layer - 1 + delta[i+1]) * scorescols + x_layer + delta[i] - 1;

      int othercenter = *data;
      data++;
      othercenter += 2 * (*data);
      data++;
      othercenter += *data;
      data += scorescols;
      othercenter += 2 * (*data);
      data--;
      othercenter += 4 * (*data);
      data--;
      othercenter += 2 * (*data);
      data += scorescols;
      othercenter += *data;
      data++;
      othercenter += 2 * (*data);
      data++;
      othercenter += *data;
      if (othercenter > smoothedcenter) return (false);
    }
  }
  return (true);
}

/////////////////////////////////////////////////////////////////////////////////////////
// 3D maximum refinement centered around (x_layer,y_layer)
float 
pcl::keypoints::brisk::ScaleSpace::refine3D (
    const uint8_t layer, const int x_layer, const int y_layer,
    float& x, float& y, float& scale, bool& ismax)
{
  ismax = true;
  pcl::keypoints::brisk::Layer& this_layer = pyramid_[layer];
  const int center = this_layer.getAgastScore (x_layer, y_layer, 1);

  // check and get above maximum:
  float delta_x_above = 0, delta_y_above = 0;
  float max_above = getScoreMaxAbove (layer,x_layer, y_layer,
                                      center, ismax,
                                      delta_x_above, delta_y_above);

  if (!ismax) return (0.0);

  float max; // to be returned

  if (layer % 2 == 0)
  { // on octave
    // treat the patch below:
    float delta_x_below, delta_y_below;
    float max_below_float;
    unsigned char max_below_uchar = 0;
    if (layer == 0)
    {
      // guess the lower intra octave...
      pcl::keypoints::brisk::Layer& l = pyramid_[0];
      int s_0_0 = l.getAgastScore_5_8 (x_layer - 1, y_layer - 1, 1);
      max_below_uchar = static_cast<unsigned char> (s_0_0);
      int s_1_0 = l.getAgastScore_5_8 (x_layer, y_layer - 1, 1);

      if (s_1_0 > max_below_uchar) max_below_uchar = static_cast<unsigned char> (s_1_0);
      int s_2_0 = l.getAgastScore_5_8 (x_layer + 1, y_layer - 1, 1);
      if (s_2_0 > max_below_uchar) max_below_uchar = static_cast<unsigned char> (s_2_0);
      int s_2_1 = l.getAgastScore_5_8 (x_layer + 1, y_layer,   1);
      if (s_2_1 > max_below_uchar) max_below_uchar = static_cast<unsigned char> (s_2_1);
      int s_1_1 = l.getAgastScore_5_8 (x_layer,   y_layer,   1);
      if (s_1_1 > max_below_uchar) max_below_uchar = static_cast<unsigned char> (s_1_1);
      int s_0_1 = l.getAgastScore_5_8 (x_layer - 1, y_layer,   1);
      if (s_0_1 > max_below_uchar) max_below_uchar = static_cast<unsigned char> (s_0_1);
      int s_0_2 = l.getAgastScore_5_8 (x_layer - 1, y_layer + 1, 1);
      if (s_0_2 > max_below_uchar) max_below_uchar = static_cast<unsigned char> (s_0_2);
      int s_1_2 = l.getAgastScore_5_8 (x_layer,   y_layer + 1, 1);
      if (s_1_2 > max_below_uchar) max_below_uchar = static_cast<unsigned char> (s_1_2);
      int s_2_2 = l.getAgastScore_5_8 (x_layer + 1, y_layer +1 , 1);
      if (s_2_2 > max_below_uchar) max_below_uchar = static_cast<unsigned char> (s_2_2);

      max_below_float = subpixel2D (s_0_0, s_0_1, s_0_2,
                                    s_1_0, s_1_1, s_1_2,
                                    s_2_0, s_2_1, s_2_2,
                                    delta_x_below, delta_y_below);
      max_below_float = max_below_uchar;
    }
    else
    {
      max_below_float = getScoreMaxBelow (layer,x_layer, y_layer,
                                          center, ismax,
                                          delta_x_below, delta_y_below);
      if (!ismax) return (0);
    }

    // get the patch on this layer:
    int s_0_0 = this_layer.getAgastScore (x_layer - 1, y_layer - 1, 1);
    int s_1_0 = this_layer.getAgastScore (x_layer,     y_layer - 1, 1);
    int s_2_0 = this_layer.getAgastScore (x_layer + 1, y_layer - 1, 1);
    int s_2_1 = this_layer.getAgastScore (x_layer + 1, y_layer,     1);
    int s_1_1 = this_layer.getAgastScore (x_layer,     y_layer,     1);
    int s_0_1 = this_layer.getAgastScore (x_layer - 1, y_layer,     1);
    int s_0_2 = this_layer.getAgastScore (x_layer - 1, y_layer + 1, 1);
    int s_1_2 = this_layer.getAgastScore (x_layer,     y_layer + 1, 1);
    int s_2_2 = this_layer.getAgastScore (x_layer + 1, y_layer + 1, 1);
    float delta_x_layer, delta_y_layer;
    float max_layer = subpixel2D (s_0_0, s_0_1, s_0_2,
                                  s_1_0, s_1_1, s_1_2,
                                  s_2_0, s_2_1, s_2_2,
                                  delta_x_layer, delta_y_layer);

    // calculate the relative scale (1D maximum):
    if (layer == 0)
      scale = refine1D_2 (max_below_float, std::max (float (center), max_layer), max_above,max);
    else
      scale = refine1D (max_below_float, std::max (float (center), max_layer), max_above,max);

    if (scale > 1.0)
    {
      // interpolate the position:
      const float r0 = (1.5f - scale) / .5f;
      const float r1 = 1.0f - r0;
      x = (r0 * delta_x_layer + r1 * delta_x_above + float (x_layer))
          * this_layer.getScale () + this_layer.getOffset ();
      y = (r0 * delta_y_layer + r1 * delta_y_above + float (y_layer))
          * this_layer.getScale () + this_layer.getOffset ();
    }
    else
    {
      if (layer == 0)
      {
        // interpolate the position:
        const float r0 = (scale - 0.5f) / 0.5f;
        const float r_1 = 1.0f - r0;
        x = r0 * delta_x_layer + r_1 * delta_x_below + float (x_layer);
        y = r0 * delta_y_layer + r_1 * delta_y_below + float (y_layer);
      }
      else
      {
        // interpolate the position:
        const float r0 = (scale - 0.75f) / 0.25f;
        const float r_1 = 1.0f -r0;
        x = (r0 * delta_x_layer + r_1 * delta_x_below + float (x_layer))
            * this_layer.getScale () +this_layer.getOffset ();
        y = (r0 * delta_y_layer + r_1 * delta_y_below + float (y_layer))
            * this_layer.getScale () + this_layer.getOffset ();
      }
    }
  }
  else
  {
    // on intra
    // check the patch below:
    float delta_x_below, delta_y_below;
    float max_below = getScoreMaxBelow (layer, x_layer, y_layer,
                                        center, ismax,
                                        delta_x_below, delta_y_below);
    if (!ismax) return (0.0);

    // get the patch on this layer:
    int s_0_0 = this_layer.getAgastScore (x_layer - 1, y_layer - 1, 1);
    int s_1_0 = this_layer.getAgastScore (x_layer,     y_layer - 1, 1);
    int s_2_0 = this_layer.getAgastScore (x_layer + 1, y_layer - 1, 1);
    int s_2_1 = this_layer.getAgastScore (x_layer + 1, y_layer,     1);
    int s_1_1 = this_layer.getAgastScore (x_layer,     y_layer,     1);
    int s_0_1 = this_layer.getAgastScore (x_layer - 1, y_layer,     1);
    int s_0_2 = this_layer.getAgastScore (x_layer - 1, y_layer + 1, 1);
    int s_1_2 = this_layer.getAgastScore (x_layer,     y_layer + 1, 1);
    int s_2_2 = this_layer.getAgastScore (x_layer + 1, y_layer + 1, 1);
    float delta_x_layer, delta_y_layer;
    float max_layer = subpixel2D (s_0_0, s_0_1, s_0_2,
                                  s_1_0, s_1_1, s_1_2,
                                  s_2_0, s_2_1, s_2_2,
                                  delta_x_layer, delta_y_layer);

    // calculate the relative scale (1D maximum):
    scale = refine1D_1 (max_below, std::max (float (center),max_layer), max_above,max);
    if (scale > 1.0)
    {
      // interpolate the position:
      const float r0 = 4.0f - scale * 3.0f;
      const float r1 = 1.0f - r0;
      x = (r0 * delta_x_layer + r1 * delta_x_above + float (x_layer))
           * this_layer.getScale () + this_layer.getOffset ();
      y = (r0 * delta_y_layer + r1 * delta_y_above + float (y_layer))
          * this_layer.getScale () + this_layer.getOffset ();
    }
    else
    {
      // interpolate the position:
      const float r0 = scale * 3.0f - 2.0f;
      const float r_1 = 1.0f - r0;
      x = (r0 * delta_x_layer + r_1 * delta_x_below + float (x_layer))
           * this_layer.getScale () + this_layer.getOffset ();
      y = (r0 * delta_y_layer + r_1 * delta_y_below + float (y_layer))
           * this_layer.getScale () + this_layer.getOffset ();
    }
  }

  // calculate the absolute scale:
  scale *= this_layer.getScale ();

  // that's it, return the refined maximum:
  return (max);
}

/////////////////////////////////////////////////////////////////////////////////////////
// return the maximum of score patches above or below
float 
pcl::keypoints::brisk::ScaleSpace::getScoreMaxAbove (
    const uint8_t layer, const int x_layer, const int y_layer,
    const int threshold, bool& ismax, float& dx, float& dy)
{
  ismax = false;
  // relevant floating point coordinates
  float x_1;
  float x1;
  float y_1;
  float y1;

  // the layer above
  assert (layer + 1 < layers_);
  pcl::keypoints::brisk::Layer& layer_above = pyramid_[layer+1];

  if (layer % 2 == 0) 
  {
    // octave
    x_1  = float (4 * (x_layer) - 1 - 2) / 6.0f;
    x1   = float (4 * (x_layer) - 1 + 2) / 6.0f;
    y_1  = float (4 * (y_layer) - 1 - 2) / 6.0f;
    y1   = float (4 * (y_layer) - 1 + 2) / 6.0f;
  }
  else
  {
    // intra
    x_1 = float (6 * (x_layer) - 1 - 3) / 8.0f;
    x1  = float (6 * (x_layer) - 1 + 3) / 8.0f;
    y_1 = float (6 * (y_layer) - 1 - 3) / 8.0f;
    y1  = float (6 * (y_layer) - 1 + 3) / 8.0f;
  }

  // check the first row
  //int max_x = int (x_1) + 1;
  //int max_y = int (y_1) + 1;
  int max_x = int (x_1 + 1.0f);
  int max_y = int (y_1 + 1.0f);
  float tmp_max = 0;
  float max = layer_above.getAgastScore (x_1, y_1, 1,1.0f);

  if (max > threshold) return (0);
  //for (int x = int (x_1) + 1; x <= int (x1); x++)
  for (int x = int (x_1 + 1.0f); x <= int (x1); x++)
  {
    tmp_max = layer_above.getAgastScore (float (x), y_1, 1,1.0f);

    if (tmp_max > threshold) return (0);
    if (tmp_max > max)
    {
      max = tmp_max;
      max_x = x;
    }
  }
  tmp_max = layer_above.getAgastScore (x1, y_1, 1,1.0f);
  
  if (tmp_max > threshold) return (0);
  if (tmp_max > max)
  {
    max = tmp_max;
    max_x = int (x1);
  }

  // middle rows
  for (int y = int (y_1) + 1; y <= int (y1); y++)
  {
    tmp_max = layer_above.getAgastScore (x_1, float (y), 1);
    
    if (tmp_max > threshold) return (0);
    if (tmp_max > max)
    {
      max   = tmp_max;
      max_x = int (x_1 + 1);
      max_y = y;
    }
    for (int x = int (x_1) + 1; x <= int (x1); x++)
    {
      tmp_max = layer_above.getAgastScore (x, y, 1);

      if (tmp_max > threshold) return (0);
      if (tmp_max > max)
      {
        max   = tmp_max;
        max_x = x;
        max_y = y;
      }
    }
    tmp_max = layer_above.getAgastScore(x1,float(y),1);

    if (tmp_max > threshold) return 0;
    if (tmp_max > max)
    {
      max   = tmp_max;
      max_x = int (x1);
      max_y = y;
    }
  }

  // bottom row
  tmp_max = layer_above.getAgastScore (x_1, y1, 1);

  if (tmp_max > max)
  {
    max   = tmp_max;
    max_x = int (x_1 + 1);
    max_y = int (y1);
  }
  for (int x = int (x_1) + 1; x <= int (x1); x++)
  {
    tmp_max = layer_above.getAgastScore (float (x), y1, 1);

    if (tmp_max > max)
    {
      max   = tmp_max;
      max_x = x;
      max_y = int (y1);
    }
  }
  tmp_max = layer_above.getAgastScore (x1, y1, 1);

  if (tmp_max > max)
  {
    max   = tmp_max;
    max_x = int (x1);
    max_y = int (y1);
  }

  //find dx/dy:
  int s_0_0 = layer_above.getAgastScore (max_x - 1, max_y - 1, 1);
  int s_1_0 = layer_above.getAgastScore (max_x,     max_y - 1, 1);
  int s_2_0 = layer_above.getAgastScore (max_x + 1, max_y - 1, 1);
  int s_2_1 = layer_above.getAgastScore (max_x + 1, max_y,     1);
  int s_1_1 = layer_above.getAgastScore (max_x,     max_y,     1);
  int s_0_1 = layer_above.getAgastScore (max_x - 1, max_y,     1);
  int s_0_2 = layer_above.getAgastScore (max_x - 1, max_y + 1, 1);
  int s_1_2 = layer_above.getAgastScore (max_x,     max_y + 1, 1);
  int s_2_2 = layer_above.getAgastScore (max_x + 1, max_y + 1, 1);
  float dx_1, dy_1;
  float refined_max = subpixel2D (s_0_0, s_0_1, s_0_2,
                                  s_1_0, s_1_1, s_1_2,
                                  s_2_0, s_2_1, s_2_2,
                                  dx_1, dy_1);

  // calculate dx/dy in above coordinates
  float real_x = float (max_x) + dx_1;
  float real_y = float (max_y) + dy_1;
  bool returnrefined = true;
  if (layer % 2 == 0)
  {
    dx = (real_x * 6.0f + 1.0f) / 4.0f - float (x_layer);
    dy = (real_y * 6.0f + 1.0f) / 4.0f - float (y_layer);
  }
  else
  {
    dx = (real_x * 8.0f + 1.0f) / 6.0f - float (x_layer);
    dy = (real_y * 8.0f + 1.0f) / 6.0f - float (y_layer);
  }

  // saturate
  if (dx > 1.0f)  { dx = 1.0f;  returnrefined = false; }
  if (dx < -1.0f) { dx = -1.0f; returnrefined = false; }
  if (dy > 1.0f)  { dy = 1.0f;  returnrefined = false; }
  if (dy < -1.0f) { dy = -1.0f; returnrefined = false; }

  // done and ok.
  ismax = true;
  if (returnrefined)
    return (std::max (refined_max,max));
  return (max);
}

/////////////////////////////////////////////////////////////////////////////////////////
float 
pcl::keypoints::brisk::ScaleSpace::getScoreMaxBelow (
    const uint8_t layer, const int x_layer, const int y_layer,
    const int threshold, bool& ismax, float& dx, float& dy)
{
  ismax = false;

  // relevant floating point coordinates
  float x_1;
  float x1;
  float y_1;
  float y1;

  if (layer % 2 == 0)
  {
    // octave
    x_1 = float (8 * (x_layer) + 1 - 4) / 6.0f;
    x1  = float (8 * (x_layer) + 1 + 4) / 6.0f;
    y_1 = float (8 * (y_layer) + 1 - 4) / 6.0f;
    y1  = float (8 * (y_layer) + 1 + 4) / 6.0f;
  }
  else
  {
    x_1 = float (6 * (x_layer) + 1 - 3) / 4.0f;
    x1  = float (6 * (x_layer) + 1 + 3) / 4.0f;
    y_1 = float (6 * (y_layer) + 1 - 3) / 4.0f;
    y1  = float (6 * (y_layer) + 1 + 3) / 4.0f;
  }

  // the layer below
  assert (layer > 0);
  pcl::keypoints::brisk::Layer& layer_below = pyramid_[layer-1];

  // check the first row
  int max_x = int (x_1) + 1;
  int max_y = int (y_1) + 1;
  float tmp_max;
  float max = layer_below.getAgastScore (x_1, y_1, 1);
  if (max > threshold) return (0);
  for (int x = int (x_1) + 1; x <= int (x1); x++)
  {
    tmp_max = layer_below.getAgastScore (float (x), y_1, 1);
    if (tmp_max > threshold) return (0);
    if (tmp_max > max)
    {
      max   = tmp_max;
      max_x = x;
    }
  }
  tmp_max = layer_below.getAgastScore (x1, y_1, 1);
  if (tmp_max > threshold) return (0);
  if (tmp_max > max)
  {
    max   = tmp_max;
    max_x = int (x1);
  }

  // middle rows
  for (int y = int (y_1) + 1; y <= int (y1); y++)
  {
    tmp_max = layer_below.getAgastScore (x_1, float (y), 1);
    if (tmp_max > threshold) return (0);
    if (tmp_max > max)
    {
      max   = tmp_max;
      max_x = int (x_1 + 1);
      max_y = y;
    }
    for (int x = int (x_1) + 1; x <= int (x1); x++)
    {
      tmp_max = layer_below.getAgastScore (x, y, 1);
      if (tmp_max > threshold) return (0);
      if (tmp_max == max)
      {
        const int t1 = 2 * (
                            layer_below.getAgastScore    (x - 1, y,     1)
                            + layer_below.getAgastScore  (x + 1, y,     1)
                            + layer_below.getAgastScore  (x,     y + 1, 1)
                            + layer_below.getAgastScore  (x,     y - 1, 1))
                            + (layer_below.getAgastScore (x + 1, y + 1, 1)
                            + layer_below.getAgastScore  (x - 1, y + 1, 1)
                            + layer_below.getAgastScore  (x + 1, y - 1, 1)
                            + layer_below.getAgastScore  (x - 1, y - 1, 1));
        const int t2 = 2 * (
                            layer_below.getAgastScore    (max_x - 1, max_y,     1)
                            + layer_below.getAgastScore  (max_x + 1, max_y,     1)
                            + layer_below.getAgastScore  (max_x,     max_y + 1, 1)
                            + layer_below.getAgastScore  (max_x,     max_y - 1, 1))
                            + (layer_below.getAgastScore (max_x + 1, max_y + 1, 1)
                            + layer_below.getAgastScore  (max_x - 1, max_y + 1, 1)
                            + layer_below.getAgastScore  (max_x + 1, max_y - 1, 1)
                            + layer_below.getAgastScore  (max_x - 1, max_y - 1, 1));
        if (t1 > t2)
        {
          max_x = x;
          max_y = y;
        }
      }
      if (tmp_max > max)
      {
        max   = tmp_max;
        max_x = x;
        max_y = y;
      }
    }
    tmp_max = layer_below.getAgastScore (x1, float (y), 1);
    if (tmp_max > threshold) return (0);
    if (tmp_max > max)
    {
      max   = tmp_max;
      max_x = int (x1);
      max_y = y;
    }
  }

  // bottom row
  tmp_max = layer_below.getAgastScore (x_1, y1, 1);
  if (tmp_max > max)
  {
    max   = tmp_max;
    max_x = int (x_1 + 1);
    max_y = int (y1);
  }
  for (int x = int (x_1) + 1; x <= int (x1); x++)
  {
    tmp_max = layer_below.getAgastScore (float (x), y1, 1);
    if (tmp_max>max)
    {
      max   = tmp_max;
      max_x = x;
      max_y = int (y1);
    }
  }
  tmp_max = layer_below.getAgastScore (x1, y1, 1);
  if (tmp_max > max)
  {
    max   = tmp_max;
    max_x = int (x1);
    max_y = int (y1);
  }

  //find dx/dy:
  int s_0_0 = layer_below.getAgastScore (max_x - 1, max_y - 1, 1);
  int s_1_0 = layer_below.getAgastScore (max_x,     max_y - 1, 1);
  int s_2_0 = layer_below.getAgastScore (max_x + 1, max_y - 1, 1);
  int s_2_1 = layer_below.getAgastScore (max_x + 1, max_y,     1);
  int s_1_1 = layer_below.getAgastScore (max_x,     max_y,     1);
  int s_0_1 = layer_below.getAgastScore (max_x - 1, max_y,     1);
  int s_0_2 = layer_below.getAgastScore (max_x - 1, max_y + 1, 1);
  int s_1_2 = layer_below.getAgastScore (max_x,     max_y + 1, 1);
  int s_2_2 = layer_below.getAgastScore (max_x + 1, max_y + 1, 1);
  float dx_1, dy_1;
  float refined_max = subpixel2D (s_0_0, s_0_1, s_0_2,
                                  s_1_0, s_1_1, s_1_2,
                                  s_2_0, s_2_1, s_2_2,
                                  dx_1, dy_1);

  // calculate dx/dy in above coordinates
  float real_x = float (max_x) + dx_1;
  float real_y = float (max_y) + dy_1;
  bool returnrefined = true;
  if (layer % 2 == 0)
  {
    dx = (real_x * 6.0f + 1.0f) / 8.0f - float (x_layer);
    dy = (real_y * 6.0f + 1.0f) / 8.0f - float (y_layer);
  }
  else
  {
    dx = (real_x * 4.0f - 1.0f) / 6.0f - float (x_layer);
    dy = (real_y * 4.0f - 1.0f) / 6.0f - float (y_layer);
  }

  // saturate
  if (dx > 1.0f)  { dx = 1.0f;  returnrefined = false; }
  if (dx < -1.0f) { dx = -1.0f; returnrefined = false; }
  if (dy > 1.0f)  { dy = 1.0f;  returnrefined = false; }
  if (dy < -1.0f) { dy = -1.0f; returnrefined = false; }

  // done and ok.
  ismax = true;
  if (returnrefined)
    return (std::max (refined_max, max));

  return (max);
}

/////////////////////////////////////////////////////////////////////////////////////////
float 
pcl::keypoints::brisk::ScaleSpace::refine1D ( 
    const float s_05, const float s0, const float s05, float& max)
{
  int i_05 = int (1024.0 * s_05 + 0.5);
  int i0   = int (1024.0 * s0   + 0.5);
  int i05  = int (1024.0 * s05  + 0.5);

  //   16.0000  -24.0000    8.0000
  //  -40.0000   54.0000  -14.0000
  //   24.0000  -27.0000    6.0000

  int three_a = 16 * i_05 - 24 * i0 + 8 * i05;
  // second derivative must be negative:
  if (three_a >= 0)
  {
    if (s0 >= s_05 && s0 >= s05)
    {
      max = s0;
      return (1.0f);
    }
    if (s_05 >= s0 && s_05 >= s05)
    {
      max = s_05;
      return (0.75f);
    }
    if (s05 >= s0 && s05 >= s_05)
    {
      max = s05;
      return (1.5f);
    }
  }

  int three_b = -40 * i_05 + 54 * i0 - 14 * i05;
  // calculate max location:
  float ret_val = -float (three_b) / float (2 * three_a);
  // saturate and return
  if (ret_val < 0.75f)
    ret_val= 0.75f;
  else 
    if (ret_val > 1.5f) 
      ret_val= 1.5f; // allow to be slightly off bounds ...?
  int three_c = +24 * i_05  -27 * i0    +6 * i05;
  max = float (three_c) + float (three_a) * ret_val * ret_val + float (three_b) * ret_val;
  max /= 3072.0f;
  return (ret_val);
}

/////////////////////////////////////////////////////////////////////////////////////////
float 
pcl::keypoints::brisk::ScaleSpace::refine1D_1 (
    const float s_05, const float s0, const float s05, float& max)
{
  int i_05 = int (1024.0 *s_05 + 0.5);
  int i0   = int (1024.0 *s0   + 0.5);
  int i05  = int (1024.0 *s05  + 0.5);

    //  4.5000   -9.0000    4.5000
    //-10.5000   18.0000   -7.5000
    //  6.0000   -8.0000    3.0000

  int two_a = 9 * i_05 - 18 * i0 + 9 * i05;
  // second derivative must be negative:
  if (two_a >= 0)
  {
    if (s0 >= s_05 && s0 >= s05)
    {
      max = s0;
      return (1.0f);
    }
    if(s_05>=s0 && s_05>=s05)
    {
      max = s_05;
      return (0.6666666666666666666666666667f);
    }
    if (s05 >= s0 && s05 >= s_05)
    {
      max = s05;
      return (1.3333333333333333333333333333f);
    }
  }

  int two_b = -21 * i_05 + 36 * i0 - 15 * i05;
  // calculate max location:
  float ret_val = -float (two_b) / float (2 * two_a);
  // saturate and return
  if (ret_val < 0.6666666666666666666666666667f)
    ret_val = 0.666666666666666666666666667f;
  else 
    if (ret_val > 1.33333333333333333333333333f) 
      ret_val = 1.333333333333333333333333333f;
  int two_c = +12 * i_05  -16 * i0    +6 * i05;
  max = float (two_c) + float (two_a) * ret_val * ret_val + float (two_b) * ret_val;
  max /= 2048.0f;
  return (ret_val);
}

/////////////////////////////////////////////////////////////////////////////////////////
float 
pcl::keypoints::brisk::ScaleSpace::refine1D_2 (
    const float s_05, const float s0, const float s05, float& max)
{
  int i_05 = int (1024.0 * s_05 + 0.5);
  int i0   = int (1024.0 * s0   + 0.5);
  int i05  = int (1024.0 * s05  + 0.5);

  //   18.0000  -30.0000   12.0000
  //  -45.0000   65.0000  -20.0000
  //   27.0000  -30.0000    8.0000

  int a = 2 * i_05- 4 * i0 + 2 * i05;
  // second derivative must be negative:
  if (a >= 0)
  {
    if (s0 >= s_05 && s0 >= s05)
    {
      max = s0;
      return (1.0f);
    }
    if (s_05 >= s0 && s_05 >= s05)
    {
      max = s_05;
      return (0.7f);
    }
    if (s05 >= s0 && s05 >= s_05)
    {
      max = s05;
      return (1.5f);
    }
  }

  int b = -5 * i_05 + 8 * i0 - 3 * i05;
  // calculate max location:
  float ret_val = -float (b) / float (2 * a);
  // saturate and return
  if (ret_val < 0.7f) 
    ret_val = 0.7f;
  else 
    if (ret_val > 1.5f) 
      ret_val = 1.5f; // allow to be slightly off bounds ...?
  int c = +3 * i_05  -3 * i0    +1 * i05;
  max = float (c) + float(a) * ret_val * ret_val + float (b) * ret_val;
  max /= 1024.0f;
  return (ret_val);
}

/////////////////////////////////////////////////////////////////////////////////////////
float 
pcl::keypoints::brisk::ScaleSpace::subpixel2D (
    const int s_0_0, const int s_0_1, const int s_0_2,
    const int s_1_0, const int s_1_1, const int s_1_2,
    const int s_2_0, const int s_2_1, const int s_2_2,
    float& delta_x, float& delta_y)
{
  // the coefficients of the 2d quadratic function least-squares fit:
  int tmp1 =        s_0_0 + s_0_2 - 2*s_1_1 + s_2_0 + s_2_2;
  int coeff1 = 3 * (tmp1 + s_0_1 - ((s_1_0 + s_1_2) << 1) + s_2_1);
  int coeff2 = 3 * (tmp1 - ((s_0_1 + s_2_1) << 1) + s_1_0 + s_1_2);
  int tmp2 =                                  s_0_2 - s_2_0;
  int tmp3 =                         (s_0_0 + tmp2 - s_2_2);
  int tmp4 =                                   tmp3 -2 * tmp2;
  int coeff3 =                    -3 * (tmp3 + s_0_1 - s_2_1);
  int coeff4 =                    -3 * (tmp4 + s_1_0 - s_1_2);
  int coeff5 =            (s_0_0 - s_0_2 - s_2_0 + s_2_2) << 2;
  int coeff6 = -(s_0_0  + s_0_2 - ((s_1_0 + s_0_1 + s_1_2 + s_2_1) << 1) - 5 * s_1_1  + s_2_0  + s_2_2) << 1;


  // 2nd derivative test:
  int H_det = 4 * coeff1 * coeff2 - coeff5 * coeff5;

  if (H_det == 0)
  {
    delta_x = 0.0f;
    delta_y = 0.0f;
    return (float (coeff6) / 18.0f);
  }

  if (!(H_det > 0 && coeff1 < 0))
  {
    // The maximum must be at the one of the 4 patch corners.
    int tmp_max = coeff3 + coeff4 + coeff5;
    delta_x = 1.0f; delta_y = 1.0f;

    int tmp = -coeff3 + coeff4 - coeff5;
    if (tmp > tmp_max)
    {
      tmp_max = tmp;
      delta_x = -1.0f; delta_y = 1.0f;
    }
    tmp = coeff3 - coeff4 - coeff5;
    if (tmp > tmp_max)
    {
      tmp_max = tmp;
      delta_x = 1.0f; delta_y = -1.0f;
    }
    tmp = -coeff3 - coeff4 + coeff5;
    if (tmp > tmp_max)
    {
      tmp_max = tmp;
      delta_x = -1.0f; delta_y = -1.0f;
    }
    return (float (tmp_max + coeff1 + coeff2 + coeff6) / 18.0f);
  }

  // this is hopefully the normal outcome of the Hessian test
  delta_x = float (2 * coeff2 * coeff3 - coeff4 * coeff5) / float (-H_det);
  delta_y = float (2 * coeff1 * coeff4 - coeff3 * coeff5) / float (-H_det);
  // TODO: this is not correct, but easy, so perform a real boundary maximum search:
  bool tx = false; bool tx_ = false; bool ty = false; bool ty_ = false;
  if (delta_x > 1.0f) tx = true;
  else if (delta_x < -1.0f) tx_=true;
  if (delta_y > 1.0f) ty = true;
  if (delta_y < -1.0f) ty_ = true;

  if (tx || tx_ || ty || ty_)
  {
    // get two candidates:
    float delta_x1 = 0.0f, delta_x2 = 0.0f, delta_y1 = 0.0f, delta_y2 = 0.0f;
    if (tx) 
    {
      delta_x1 = 1.0f;
      delta_y1 = -float (coeff4 + coeff5) / float (2.0 * coeff2);
      if (delta_y1 > 1.0f) delta_y1 = 1.0f; else if (delta_y1 < -1.0f) delta_y1 = -1.0f;
    }
    else if (tx_) 
    {
      delta_x1 = -1.0f;
      delta_y1 = -float (coeff4 - coeff5) / float (2.0 * coeff2);
      if (delta_y1 > 1.0f) delta_y1 = 1.0f; else if (delta_y1 < -1.0f) delta_y1 = -1.0f;
    }
    if (ty) 
    {
      delta_y2 = 1.0f;
      delta_x2 = -float (coeff3 + coeff5) / float (2.0 * coeff1);
      if (delta_x2 > 1.0f) delta_x2 = 1.0f; else if (delta_x2 < -1.0f) delta_x2 = -1.0f;
    }
    else if (ty_) 
    {
      delta_y2 = -1.0f;
      delta_x2 = -float (coeff3 - coeff5) / float (2.0 * coeff1);
      if (delta_x2 > 1.0f) delta_x2 = 1.0f; else if (delta_x2 < -1.0f) delta_x2 = -1.0f;
    }
    // insert both options for evaluation which to pick
    float max1 = (float (coeff1) * delta_x1 * delta_x1 + float (coeff2) * delta_y1 * delta_y1
                 +float (coeff3) * delta_x1 + float (coeff4) * delta_y1
                 +float (coeff5) * delta_x1 * delta_y1
                 +float (coeff6)) / 18.0f;
    float max2 = (float (coeff1) * delta_x2 * delta_x2 + float (coeff2) * delta_y2 * delta_y2
                 +float (coeff3) * delta_x2 + float (coeff4) * delta_y2
                 +float (coeff5) * delta_x2 * delta_y2
                 +float (coeff6)) / 18.0f;
    if (max1 > max2) 
    {
      delta_x = delta_x1;
      delta_y = delta_x1;
      return (max1);
    }
    else
    {
      delta_x = delta_x2;
      delta_y = delta_x2;
      return (max2);
    }
  }

  // this is the case of the maximum inside the boundaries:
  return ((float (coeff1) * delta_x * delta_x + float (coeff2) * delta_y * delta_y
          +float (coeff3) * delta_x + float (coeff4) * delta_y
          +float (coeff5) * delta_x * delta_y
          +float (coeff6)) / 18.0f);
}

/////////////////////////////////////////////////////////////////////////////////////////
// construct a layer
pcl::keypoints::brisk::Layer::Layer (
    const std::vector<unsigned char>& img, 
    int width, int height,
    float scale, float offset) 
{
  img_width_ = width;
  img_height_ = height;
  img_ = img;
  scores_ = std::vector<unsigned char> (img_width_ * img_height_, 0);

  // attention: this means that the passed image reference must point to persistent memory
  scale_  = scale;
  offset_ = offset;

  // create an agast detector
  oast_detector_.reset (new pcl::keypoints::agast::OastDetector9_16 (img_width_, img_height_, 0));
  agast_detector_5_8_.reset (new pcl::keypoints::agast::AgastDetector5_8 (img_width_, img_height_, 0));
}

/////////////////////////////////////////////////////////////////////////////////////////
// derive a layer
pcl::keypoints::brisk::Layer::Layer (const pcl::keypoints::brisk::Layer& layer, int mode)
{
  if (mode == CommonParams::HALFSAMPLE)
  {
    img_width_ = layer.img_width_ / 2;
    img_height_ = layer.img_height_ / 2;
    img_.resize (img_width_ * img_height_);

    halfsample (layer.img_, layer.img_width_, layer.img_height_, img_, img_width_, img_height_);
    scale_  = layer.scale_ * 2.0f;
    offset_ = 0.5f * scale_ - 0.5f;
  }
  else 
  {
    img_width_ = 2 * layer.img_width_ / 3;
    img_height_ = 2 * layer.img_height_ / 3;
    img_.resize (img_width_ * img_height_);

    twothirdsample (layer.img_, layer.img_width_, layer.img_height_, img_, img_width_, img_height_);
    scale_  = layer.scale_ * 1.5f;
    offset_ = 0.5f * scale_ - 0.5f;
  }
 
  scores_ = std::vector<unsigned char> (img_width_ * img_height_, 0);

  // create an agast detector
  oast_detector_.reset (new pcl::keypoints::agast::OastDetector9_16 (img_width_, img_height_, 0));
  agast_detector_5_8_.reset (new pcl::keypoints::agast::AgastDetector5_8 (img_width_, img_height_, 0));
}

/////////////////////////////////////////////////////////////////////////////////////////
// Fast/Agast
// wraps the agast class
void 
pcl::keypoints::brisk::Layer::getAgastPoints (
    uint8_t threshold, std::vector<pcl::PointUV, Eigen::aligned_allocator<pcl::PointUV> > &keypoints)
{
  oast_detector_->setThreshold (threshold);
  oast_detector_->detect (&img_[0], keypoints);

  // also write scores
  const int num = int (keypoints.size ());
  const int imcols = img_width_;

  for (int i = 0; i < num; i++)
  {
    const int offs = int (keypoints[i].u + keypoints[i].v * float (imcols));
    *(&scores_[0] + offs) = static_cast<unsigned char> (oast_detector_->computeCornerScore (&img_[0] + offs));
  }
}

/////////////////////////////////////////////////////////////////////////////////////////
pcl::uint8_t 
pcl::keypoints::brisk::Layer::getAgastScore (int x, int y, uint8_t threshold)
{
  if (x < 3 || y < 3) 
  {
    return (0);
  }
  if (x >= img_width_ - 3 || y >= img_height_ - 3) 
  {
    return (0);
  }
  uint8_t& score = *(&scores_[0] + x + y * img_width_);
  if (score > 2) 
  {
    return (score);
  }
  oast_detector_->setThreshold (threshold - 1);
  score = uint8_t (oast_detector_->computeCornerScore (&img_[0] + x + y * img_width_));
  if (score < threshold) score = 0;
  return (score);
}

/////////////////////////////////////////////////////////////////////////////////////////
pcl::uint8_t 
pcl::keypoints::brisk::Layer::getAgastScore_5_8 (int x, int y, uint8_t threshold)
{
  if (x < 2 || y < 2)
  {
    return 0;
  }

  if (x >= img_width_ - 2 || y >= img_height_ - 2) 
  {
    return 0;
  }

  agast_detector_5_8_->setThreshold (threshold - 1);
  uint8_t score = uint8_t (agast_detector_5_8_->computeCornerScore (&img_[0] + x + y * img_width_));
  if (score < threshold) score = 0;
  return (score);
}

/////////////////////////////////////////////////////////////////////////////////////////
pcl::uint8_t 
pcl::keypoints::brisk::Layer::getAgastScore (float xf, float yf, uint8_t threshold, float scale)
{
  if (scale <= 1.0f)
  {
    // just do an interpolation inside the layer
    const int x = int (xf);
    const float rx1 = xf - float (x);
    const float rx = 1.0f - rx1;
    const int y = int (yf);
    const float ry1 = yf -float (y);
    const float ry  = 1.0f -ry1;

    const float value = rx  * ry  * getAgastScore (x,     y,     threshold)+
            rx1 * ry  * getAgastScore (x + 1, y,     threshold)+
            rx  * ry1 * getAgastScore (x,     y + 1, threshold)+
            rx1 * ry1 * getAgastScore (x + 1, y + 1, threshold);


    return (static_cast<uint8_t> (value));
  }
  else
  {
    // this means we overlap area smoothing
    const float halfscale = scale / 2.0f;
    // get the scores first:
    for (int x = int (xf - halfscale); x <= int (xf + halfscale + 1.0f); x++)
      for (int y = int (yf - halfscale); y <= int (yf + halfscale + 1.0f); y++)
        getAgastScore (x, y, threshold);
    // get the smoothed value
    return (getValue (scores_, img_width_, img_height_, xf, yf, scale));
  }
}

/////////////////////////////////////////////////////////////////////////////////////////
// access gray values (smoothed/interpolated)
pcl::uint8_t 
pcl::keypoints::brisk::Layer::getValue (
    const std::vector<unsigned char>& mat, 
    int width, int height,
    float xf, float yf, float scale)
{
  (void)height;
  assert (!mat.empty ());
  // get the position
  const int x = int (floor (xf));
  const int y = int (floor (yf));
  const std::vector<unsigned char>& image = mat;
  const int& imagecols = width;

  // get the sigma_half:
  const float sigma_half = scale / 2.0f;
  const float area = 4.0f * sigma_half * sigma_half;

  // calculate output:
  int ret_val;
  if (sigma_half < 0.5)
  {
    // interpolation multipliers:
		const int r_x   = static_cast<int> ((xf - float (x)) * 1024);
		const int r_y   = static_cast<int> ((yf - float (y)) * 1024);
    const int r_x_1 = (1024 - r_x);
    const int r_y_1 = (1024 - r_y);
    const unsigned char* ptr = &image[0] + x + y * imagecols;

    // just interpolate:
    ret_val = (r_x_1 * r_y_1 * int (*ptr));
    ptr++;
    ret_val += (r_x * r_y_1 * int (*ptr));
    ptr += imagecols;
    ret_val += (r_x * r_y * int (*ptr));
    ptr--;
    ret_val += (r_x_1 * r_y * int (*ptr));
    return (static_cast<uint8_t> (0xFF & ((ret_val + 512) / 1024 / 1024)));
  }

  // this is the standard case (simple, not speed optimized yet):

  // scaling:
  const int scaling  = static_cast<int> (4194304.0f / area);
  const int scaling2 = static_cast<int> (float (scaling) * area / 1024.0f);

  // calculate borders
  const float x_1 = xf - sigma_half;
  const float x1  = xf + sigma_half;
  const float y_1 = yf - sigma_half;
  const float y1  = yf + sigma_half;

  const int x_left   = int (x_1 + 0.5f);
  const int y_top    = int (y_1 + 0.5f);
  const int x_right  = int (x1 + 0.5f);
  const int y_bottom = int (y1 + 0.5f);

  // overlap area - multiplication factors:
  const float r_x_1 = float (x_left) - x_1 + 0.5f;
  const float r_y_1 = float (y_top)  - y_1 + 0.5f;
  const float r_x1  = x1 - float (x_right) + 0.5f;
  const float r_y1  = y1 - float (y_bottom) + 0.5f;
  const int dx = x_right  - x_left - 1;
  const int dy = y_bottom - y_top  - 1;
  const int A = static_cast<int> ((r_x_1 * r_y_1) * float (scaling));
  const int B = static_cast<int> ((r_x1  * r_y_1) * float (scaling));
  const int C = static_cast<int> ((r_x1  * r_y1)  * float (scaling));
  const int D = static_cast<int> ((r_x_1 * r_y1)  * float (scaling));
  const int r_x_1_i = static_cast<int> (r_x_1 * float (scaling));
  const int r_y_1_i = static_cast<int> (r_y_1 * float (scaling));
  const int r_x1_i  = static_cast<int> (r_x1  * float (scaling));
  const int r_y1_i  = static_cast<int> (r_y1  * float (scaling));

  // now the calculation:
  const unsigned char* ptr = &image[0] + x_left + imagecols * y_top;
  // first row:
  ret_val = A * int (*ptr);
  ptr++;
  const unsigned char* end1 = ptr + dx;
  for (; ptr < end1; ptr++)
    ret_val += r_y_1_i * int (*ptr);

  ret_val += B * int (*ptr);

  // middle ones:
  ptr += imagecols - dx - 1;
  const unsigned char* end_j = ptr + dy * imagecols;

  for (; ptr < end_j; ptr += imagecols - dx - 1)
  {
    ret_val += r_x_1_i * int (*ptr);
    ptr++;
    const unsigned char* end2 = ptr + dx;
    for (; ptr < end2; ptr++)
      ret_val += int (*ptr) * scaling;

    ret_val += r_x1_i * int (*ptr);
  }

  // last row:
  ret_val += D * int (*ptr);
  ptr++;
  const unsigned char* end3 = ptr + dx;
  for (; ptr < end3; ptr++)
    ret_val += r_y1_i * int (*ptr);

  ret_val += C * int (*ptr);

  return (static_cast<uint8_t> (0xFF & ((ret_val + scaling2 / 2) / scaling2 / 1024)));
}

/////////////////////////////////////////////////////////////////////////////////////////
// half sampling
inline void 
pcl::keypoints::brisk::Layer::halfsample (
    const std::vector<unsigned char>& srcimg, 
    int srcwidth, int srcheight,
    std::vector<unsigned char>& dstimg,
    int dstwidth, int dstheight)
{
  (void)dstheight;
#if defined(__SSSE3__) && !defined(__i386__)
  const unsigned short leftoverCols = static_cast<unsigned short> ((srcwidth % 16) / 2); // take care with border...
  const bool noleftover = (srcwidth % 16) == 0; // note: leftoverCols can be zero but this still false...

  // make sure the destination image is of the right size:
  assert (floor (double (srcwidth) / 2.0) == dstwidth);
  assert (floor (double (srcheight) / 2.0) == dstheight);

  // mask needed later:
  register __m128i mask = _mm_set_epi32 (0x00FF00FF, 0x00FF00FF, 0x00FF00FF, 0x00FF00FF);
  // to be added in order to make successive averaging correct:
  register __m128i ones = _mm_set_epi32 (0x11111111, 0x11111111, 0x11111111, 0x11111111);

  // data pointers:
  const __m128i* p1 = reinterpret_cast<const __m128i*> (&srcimg[0]);
  const __m128i* p2 = reinterpret_cast<const __m128i*> (&srcimg[0] + srcwidth);
  __m128i* p_dest = reinterpret_cast<__m128i*> (&dstimg[0]);
  unsigned char* p_dest_char;//=(unsigned char*)p_dest;

  // size:
  const unsigned int size = (srcwidth * srcheight) / 16;
  const unsigned int hsize = srcwidth / 16;
  const __m128i* p_end = p1 + size;
  unsigned int row = 0;
  const unsigned int end = hsize / 2;
  bool half_end;
  if (hsize % 2 == 0)
    half_end = false;
  else
    half_end = true;
  while (p2 < p_end)
  {
    for (unsigned int i = 0; i < end; i++)
    {
      // load the two blocks of memory:
      __m128i upper;
      __m128i lower;
      if (noleftover)
      {
        upper = _mm_load_si128 (p1);
        lower = _mm_load_si128 (p2);
      }
      else
      {
        upper = _mm_loadu_si128 (p1);
        lower = _mm_loadu_si128 (p2);
      }

      __m128i result1 = _mm_adds_epu8 (upper, ones);
      result1 = _mm_avg_epu8 (upper, lower);

      // increment the pointers:
      p1++;
      p2++;

      // load the two blocks of memory:
      upper = _mm_loadu_si128 (p1);
      lower = _mm_loadu_si128 (p2);
      __m128i result2 = _mm_adds_epu8 (upper, ones);
      result2 = _mm_avg_epu8 (upper, lower);
      // calculate the shifted versions:
      __m128i result1_shifted = _mm_srli_si128 (result1, 1);
      __m128i result2_shifted = _mm_srli_si128 (result2, 1);
      // pack:
      __m128i result = _mm_packus_epi16 (_mm_and_si128 (result1, mask), _mm_and_si128 (result2, mask));
      __m128i result_shifted = _mm_packus_epi16 (_mm_and_si128 (result1_shifted, mask),_mm_and_si128 (result2_shifted, mask));
      // average for the second time:
      result = _mm_avg_epu8 (result, result_shifted);

      // store to memory
      _mm_storeu_si128 (p_dest, result);

      // increment the pointers:
      p1++;
      p2++;
      p_dest++;
      //p_dest_char=(unsigned char*)p_dest;
    }
    // if we are not at the end of the row, do the rest:
    if (half_end)
    {
      // load the two blocks of memory:
      __m128i upper;
      __m128i lower;
      if (noleftover)
      {
        upper = _mm_load_si128 (p1);
        lower = _mm_load_si128 (p2);
      }
      else
      {
        upper = _mm_loadu_si128 (p1);
        lower = _mm_loadu_si128 (p2);
      }

      __m128i result1 = _mm_adds_epu8 (upper, ones);
      result1 = _mm_avg_epu8 (upper, lower);

      // increment the pointers:
      p1++;
      p2++;

      // compute horizontal pairwise average and store
      p_dest_char = reinterpret_cast<unsigned char*> (p_dest);
#ifdef __GNUC__
      typedef unsigned char __attribute__ ((__may_alias__)) UCHAR_ALIAS;
#endif
#ifdef _MSC_VER
      // Todo: find the equivalent to may_alias
      #define UCHAR_ALIAS unsigned char //__declspec(noalias)
#endif
      const UCHAR_ALIAS* result = reinterpret_cast<const UCHAR_ALIAS*> (&result1);
      for (unsigned int j = 0; j < 8; j++)
      {
        *(p_dest_char++) = static_cast<unsigned char> ((*(result + 2 * j) + *(result + 2 * j + 1)) / 2);
      }
      //p_dest_char=(unsigned char*)p_dest;
    }
    else
      p_dest_char = reinterpret_cast<unsigned char*> (p_dest);

    if (noleftover)
    {
      row++;
      p_dest = reinterpret_cast<__m128i*> (&dstimg[0] + row * dstwidth);
      p1 = reinterpret_cast<const __m128i*> (&srcimg[0] + 2 * row * srcwidth);
      //p2=(__m128i*)(srcimg.data+(2*row+1)*srcwidth);
      //p1+=hsize;
      p2 = p1 + hsize;
    }
    else
    {
      const unsigned char* p1_src_char = reinterpret_cast<const unsigned char*> (p1);
      const unsigned char* p2_src_char = reinterpret_cast<const unsigned char*> (p2);
      for (unsigned int k = 0; k < leftoverCols; k++)
      {
        unsigned short tmp = static_cast<unsigned short> (p1_src_char[k] + p1_src_char[k+1]+ p2_src_char[k] + p2_src_char[k+1]);
        *(p_dest_char++) = static_cast<unsigned char>(tmp / 4);
      }
      // done with the two rows:
      row++;
      p_dest = reinterpret_cast<__m128i*> (&dstimg[0] + row * dstwidth);
      p1 = reinterpret_cast<const __m128i*> (&srcimg[0] + 2 * row * srcwidth);
      p2 = reinterpret_cast<const __m128i*> (&srcimg[0] + (2 * row + 1) * srcwidth);
    }
  }
#else
  (void) (srcimg);
  (void) (srcwidth);
  (void) (srcheight);
  (void) (dstimg); 
  (void) (dstwidth);
  PCL_ERROR("brisk without SSSE3 support not implemented");
#endif
}

/////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::keypoints::brisk::Layer::twothirdsample (
    const std::vector<unsigned char>& srcimg, 
    int srcwidth, int srcheight,
    std::vector<unsigned char>& dstimg,
    int dstwidth, int dstheight)
{
  (void)dstheight;
#if defined(__SSSE3__) && !defined(__i386__)
  const unsigned short leftoverCols = static_cast<unsigned short> (((srcwidth / 3) * 3) % 15);// take care with border...

  // make sure the destination image is of the right size:
  assert (floor (double (srcwidth) / 3.0 * 2.0) == dstwidth);
  assert (floor (double (srcheight) / 3.0 * 2.0) == dstheight);

  // masks:
  register __m128i mask1 = _mm_set_epi8 (char(0x80),char(0x80),char(0x80),char(0x80),char(0x80),char(0x80),char(0x80),12,char(0x80),10,char(0x80),7,char(0x80),4,char(0x80),1);
  register __m128i mask2 = _mm_set_epi8 (char(0x80),char(0x80),char(0x80),char(0x80),char(0x80),char(0x80),12,char(0x80),10,char(0x80),7,char(0x80),4,char(0x80),1,char(0x80));
  register __m128i mask = _mm_set_epi8 (char(0x80),char(0x80),char(0x80),char(0x80),char(0x80),char(0x80),14,12,11,9,8,6,5,3,2,0);
  register __m128i store_mask = _mm_set_epi8 (0x0,0x0,0x0,0x0,0x0,0x0,char(0x80),char(0x80),char(0x80),char(0x80),char(0x80),char(0x80),char(0x80),char(0x80),char(0x80),char(0x80));

  // data pointers:
  const unsigned char* p1 = &srcimg[0];
  const unsigned char* p2 = p1 + srcwidth;
  const unsigned char* p3 = p2 + srcwidth;
  unsigned char* p_dest1 = &dstimg[0];
  unsigned char* p_dest2 = p_dest1 + dstwidth;
  const unsigned char* p_end = p1 + (srcwidth * srcheight);

  unsigned int row = 0;
  unsigned int row_dest = 0;
  int hsize = srcwidth / 15;
  while (p3 < p_end)
  {
    for (int i = 0; i < hsize; i++)
    {
      // load three rows
      const __m128i first = _mm_loadu_si128  (reinterpret_cast<const __m128i*> (p1));
      const __m128i second = _mm_loadu_si128 (reinterpret_cast<const __m128i*> (p2));
      const __m128i third = _mm_loadu_si128  (reinterpret_cast<const __m128i*> (p3));

      // upper row:
      __m128i upper = _mm_avg_epu8 (_mm_avg_epu8 (first,second),first);
      __m128i temp1_upper = _mm_or_si128 (_mm_shuffle_epi8 (upper,mask1), _mm_shuffle_epi8 (upper,mask2));
      __m128i temp2_upper  = _mm_shuffle_epi8 (upper,mask);
      __m128i result_upper = _mm_avg_epu8 (_mm_avg_epu8 (temp2_upper, temp1_upper), temp2_upper);

      // lower row:
      __m128i lower = _mm_avg_epu8 (_mm_avg_epu8 (third, second), third);
      __m128i temp1_lower  = _mm_or_si128 (_mm_shuffle_epi8 (lower, mask1), _mm_shuffle_epi8 (lower, mask2));
      __m128i temp2_lower  = _mm_shuffle_epi8 (lower,mask);
      __m128i result_lower = _mm_avg_epu8 (_mm_avg_epu8 (temp2_lower, temp1_lower), temp2_lower);

      // store:
      if (i * 10 + 16 > dstwidth)
      {
        _mm_maskmoveu_si128 (result_upper, store_mask, reinterpret_cast<char*> (p_dest1));
        _mm_maskmoveu_si128 (result_lower, store_mask, reinterpret_cast<char*> (p_dest2));
      }
      else
      {
        _mm_storeu_si128 (reinterpret_cast<__m128i*> (p_dest1), result_upper);
        _mm_storeu_si128 (reinterpret_cast<__m128i*> (p_dest2), result_lower);
      }

      // shift pointers:
      p1 += 15;
      p2 += 15;
      p3 += 15;
      p_dest1 += 10;
      p_dest2 += 10;
    }

    // fill the remainder:
    for (unsigned int j = 0; j < leftoverCols; j+= 3)
    {
      const unsigned short A1 = *(p1++);
      const unsigned short A2 = *(p1++);
      const unsigned short A3 = *(p1++);
      const unsigned short B1 = *(p2++);
      const unsigned short B2 = *(p2++);
      const unsigned short B3 = *(p2++);
      const unsigned short C1 = *(p3++);
      const unsigned short C2 = *(p3++);
      const unsigned short C3 = *(p3++);

      *(p_dest1++) = static_cast<unsigned char> (((4 * A1 + 2 * (A2 + B1) + B2) / 9) & 0x00FF);
      *(p_dest1++) = static_cast<unsigned char> (((4 * A3 + 2 * (A2 + B3) + B2) / 9) & 0x00FF);
      *(p_dest2++) = static_cast<unsigned char> (((4 * C1 + 2 * (C2 + B1) + B2) / 9) & 0x00FF);
      *(p_dest2++) = static_cast<unsigned char> (((4 * C3 + 2 * (C2 + B3) + B2) / 9) & 0x00FF);
    }

    // increment row counter:
    row += 3;
    row_dest += 2;

    // reset pointers
    p1 = &srcimg[0] + row * srcwidth;
    p2 = p1 + srcwidth;
    p3 = p2 + srcwidth;
    p_dest1 = &dstimg[0] + row_dest * dstwidth;
    p_dest2 = p_dest1 + dstwidth;
  }
#else
  (void) (srcimg);
  (void) (srcwidth);
  (void) (srcheight);
  (void) (dstimg); 
  (void) (dstwidth);
  PCL_ERROR("brisk without SSSE3 support not implemented");
#endif
}

