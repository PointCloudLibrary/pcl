/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
 */

#ifndef PCL_RECOGNITION_OCCLUSION_REASONING_HPP_
#define PCL_RECOGNITION_OCCLUSION_REASONING_HPP_

#include <pcl/recognition/hv/occlusion_reasoning.h>

#include <algorithm>

///////////////////////////////////////////////////////////////////////////////////////////
template<typename ModelT, typename SceneT>
pcl::occlusion_reasoning::ZBuffering<ModelT, SceneT>::ZBuffering (int resx, int resy, float f) :
  f_ (f), cx_ (resx), cy_ (resy), depth_ (nullptr)
{
}

///////////////////////////////////////////////////////////////////////////////////////////
template<typename ModelT, typename SceneT>
pcl::occlusion_reasoning::ZBuffering<ModelT, SceneT>::ZBuffering () :
  f_ (), cx_ (), cy_ (), depth_ (nullptr)
{
}

///////////////////////////////////////////////////////////////////////////////////////////
template<typename ModelT, typename SceneT>
pcl::occlusion_reasoning::ZBuffering<ModelT, SceneT>::~ZBuffering ()
{
  delete[] depth_;
}

///////////////////////////////////////////////////////////////////////////////////////////
template<typename ModelT, typename SceneT> void
pcl::occlusion_reasoning::ZBuffering<ModelT, SceneT>::filter (typename pcl::PointCloud<ModelT>::ConstPtr & model,
                                                              typename pcl::PointCloud<ModelT>::Ptr & filtered, float thres)
{
  pcl::Indices indices_to_keep;
  filter(model, indices_to_keep, thres);
  pcl::copyPointCloud (*model, indices_to_keep, *filtered);
}

///////////////////////////////////////////////////////////////////////////////////////////
template<typename ModelT, typename SceneT> void
pcl::occlusion_reasoning::ZBuffering<ModelT, SceneT>::filter (typename pcl::PointCloud<ModelT>::ConstPtr & model,
                                                                      pcl::Indices & indices_to_keep, float thres)
{

  float cx, cy;
  cx = static_cast<float> (cx_) / 2.f - 0.5f;
  cy = static_cast<float> (cy_) / 2.f - 0.5f;

  indices_to_keep.resize (model->size ());
  int keep = 0;
  for (std::size_t i = 0; i < model->size (); i++)
  {
    float x = (*model)[i].x;
    float y = (*model)[i].y;
    float z = (*model)[i].z;
    int u = static_cast<int> (f_ * x / z + cx);
    int v = static_cast<int> (f_ * y / z + cy);

    if (u >= cx_ || v >= cy_ || u < 0 || v < 0)
      continue;

    //Check if point depth (distance to camera) is greater than the (u,v) meaning that the point is not visible
    if ((z - thres) > depth_[u * cy_ + v] || !std::isfinite(depth_[u * cy_ + v]))
      continue;

    indices_to_keep[keep] = static_cast<int> (i);
    keep++;
  }

  indices_to_keep.resize (keep);
}

///////////////////////////////////////////////////////////////////////////////////////////
template<typename ModelT, typename SceneT> void
pcl::occlusion_reasoning::ZBuffering<ModelT, SceneT>::computeDepthMap (typename pcl::PointCloud<SceneT>::ConstPtr & scene, bool compute_focal,
                                                                       bool smooth, int wsize)
{
  float cx, cy;
  cx = static_cast<float> (cx_) / 2.f - 0.5f;
  cy = static_cast<float> (cy_) / 2.f - 0.5f;

  //compute the focal length
  if (compute_focal)
  {

    float max_u, max_v, min_u, min_v;
    max_u = max_v = std::numeric_limits<float>::max () * -1;
    min_u = min_v = std::numeric_limits<float>::max ();

    for (const auto& point: *scene)
    {
      float b_x = point.x / point.z;
      if (b_x > max_u)
        max_u = b_x;
      if (b_x < min_u)
        min_u = b_x;

      float b_y = point.y / point.z;
      if (b_y > max_v)
        max_v = b_y;
      if (b_y < min_v)
        min_v = b_y;
    }

    float maxC = std::max (std::max (std::abs (max_u), std::abs (max_v)), std::max (std::abs (min_u), std::abs (min_v)));
    f_ = (cx) / maxC;
  }

  depth_ = new float[cx_ * cy_];
  std::fill_n(depth_, cx * cy, std::numeric_limits<float>::quiet_NaN());

  for (const auto& point: *scene)
  {
    const float& x = point.x;
    const float& y = point.y;
    const float& z = point.z;
    const int u = static_cast<int> (f_ * x / z + cx);
    const int v = static_cast<int> (f_ * y / z + cy);

    if (u >= cx_ || v >= cy_ || u < 0 || v < 0)
      continue;

    if ((z < depth_[u * cy_ + v]) || (!std::isfinite(depth_[u * cy_ + v])))
      depth_[u * cx_ + v] = z;
  }

  if (smooth)
  {
    //Dilate and smooth the depth map
    int ws = wsize;
    int ws2 = int (std::floor (static_cast<float> (ws) / 2.f));
    float * depth_smooth = new float[cx_ * cy_];
    for (int i = 0; i < (cx_ * cy_); i++)
      depth_smooth[i] = std::numeric_limits<float>::quiet_NaN ();

    for (int u = ws2; u < (cx_ - ws2); u++)
    {
      for (int v = ws2; v < (cy_ - ws2); v++)
      {
        float min = std::numeric_limits<float>::max ();
        for (int j = (u - ws2); j <= (u + ws2); j++)
        {
          for (int i = (v - ws2); i <= (v + ws2); i++)
          {
            if (std::isfinite(depth_[j * cx_ + i]) && (depth_[j * cx_ + i] < min))
            {
              min = depth_[j * cx_ + i];
            }
          }
        }

        if (min < (std::numeric_limits<float>::max () - 0.1))
        {
          depth_smooth[u * cx_ + v] = min;
        }
      }
    }

    std::copy(depth_smooth, depth_smooth + cx_ * cy_, depth_);
    delete[] depth_smooth;
  }
}

#endif    // PCL_RECOGNITION_OCCLUSION_REASONING_HPP_
