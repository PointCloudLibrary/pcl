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

///////////////////////////////////////////////////////////////////////////////////////////
template<typename ModelT, typename SceneT>
pcl::occlusion_reasoning::ZBuffering<ModelT, SceneT>::ZBuffering (int resx, int resy, float f)
  : f_ (f)
  , cx_ (resx)
  , cy_ (resy)
  , depth_ (NULL)
{
}

///////////////////////////////////////////////////////////////////////////////////////////
template<typename ModelT, typename SceneT>
pcl::occlusion_reasoning::ZBuffering<ModelT, SceneT>::ZBuffering ()
  : f_ ()
  , cx_ ()
  , cy_ ()
  , depth_ (NULL)
{
}

///////////////////////////////////////////////////////////////////////////////////////////
template<typename ModelT, typename SceneT>
pcl::occlusion_reasoning::ZBuffering<ModelT, SceneT>::~ZBuffering ()
{
  if (depth_ != NULL)
    delete[] depth_;
}

///////////////////////////////////////////////////////////////////////////////////////////
template<typename ModelT, typename SceneT> void
pcl::occlusion_reasoning::ZBuffering<ModelT, SceneT>::filter (typename pcl::PointCloud<ModelT>::ConstPtr & model, typename pcl::PointCloud<ModelT>::Ptr & filtered,
                                         float thres)
{

  float cx, cy;
  cx = static_cast<float> (cx_) / 2.f - 0.5f;
  cy = static_cast<float> (cy_) / 2.f - 0.5f;

  std::vector<int> indices_to_keep;
  indices_to_keep.resize (model->points.size ());
  int keep = 0;
  for (size_t i = 0; i < model->points.size (); i++)
  {
    float x = model->points[i].x;
    float y = model->points[i].y;
    float z = model->points[i].z;
    int u = static_cast<int> (f_ * x / z + cx);
    int v = static_cast<int> (f_ * y / z + cy);

    if (u >= cx_ || v >= cy_ || u < 0 || v < 0)
      continue;

    //Check if point depth (distance to camera) is greater than the (u,v) meaning that the point is not visible
    if ((z - thres) > depth_[u * cy_ + v] || !pcl_isfinite(depth_[u * cy_ + v]))
      continue;

    indices_to_keep[keep] = static_cast<int> (i);
    keep++;
  }

  indices_to_keep.resize (keep);
  pcl::copyPointCloud (*model, indices_to_keep, *filtered);
}

///////////////////////////////////////////////////////////////////////////////////////////
template<typename ModelT, typename SceneT> void
pcl::occlusion_reasoning::ZBuffering<ModelT, SceneT>::computeDepthMap (typename pcl::PointCloud<SceneT>::ConstPtr & scene, bool compute_focal)
{
  //compute the focal length

  if (compute_focal)
  {

    float max_b = -1;
    for (size_t i = 0; i < scene->points.size (); i++)
    {
      float b_x = scene->points[i].x / scene->points[i].z;
      float b_y = scene->points[i].y / scene->points[i].z;
      float max_by = std::max (std::abs (b_x), std::abs (b_y));
      if (max_by > max_b)
        max_b = max_by;
    }

    f_ = (static_cast<float> (std::max (cx_, cy_)) / 2.f - 0.5f) / max_b;
  }

  float cx, cy;
  cx = static_cast<float> (cx_) / 2.f - 0.5f;
  cy = static_cast<float> (cy_) / 2.f - 0.5f;

  depth_ = new float[cx_ * cy_];
  for (int i = 0; i < (cx_ * cy_); i++)
    depth_[i] = std::numeric_limits<float>::quiet_NaN ();

  for (size_t i = 0; i < scene->points.size (); i++)
  {
    float x = scene->points[i].x;
    float y = scene->points[i].y;
    float z = scene->points[i].z;
    int u = static_cast<int> (f_ * x / z + cx);
    int v = static_cast<int> (f_ * y / z + cy);

    if (u >= cx_ || v >= cy_ || u < 0 || v < 0)
      continue;

    if ((z < depth_[u * cy_ + v]) || (!pcl_isfinite(depth_[u * cy_ + v])))
      depth_[u * cx_ + v] = z;
  }
}

#endif    // PCL_RECOGNITION_OCCLUSION_REASONING_HPP_

