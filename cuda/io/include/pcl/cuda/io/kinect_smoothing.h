/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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
 *
 * $Id$
 *
 */

#ifndef PCL_CUDA_KINECT_SMOOTHING_H_
#define PCL_CUDA_KINECT_SMOOTHING_H_

#include <pcl/io/openni_camera/openni_image.h>
#include <thrust/tuple.h>
#include <pcl/pcl_exports.h>

namespace pcl
{
  namespace cuda
  {

    struct DisparityBoundSmoothing
    {
      DisparityBoundSmoothing (int width, int height, int window_size, float focal_length, float baseline, float disparity_threshold, float *data, float *raw_data)
        : width_ (width), height_ (height), window_size_ (window_size)
        , focal_length_(focal_length), baseline_(baseline), disparity_threshold_(disparity_threshold)
        , data_(data), raw_data_(raw_data)
      {}
  
      int width_, height_;
      int window_size_;
      float focal_length_;
      float baseline_;
      float disparity_threshold_;
      float *data_;
      float *raw_data_;

      // helper function
      inline __host__ __device__ 
        float disparity2depth (float disparity)
      {
        return baseline_ * focal_length_ / disparity;
      }

      // helper function
      inline __host__ __device__
        float depth2disparity (float depth)
      {
        return baseline_ * focal_length_ / depth;
      }

      // clampToDisparityBounds
      inline __host__ __device__
        float clampToDisparityBounds (float avg_depth, float depth)
      {
        float disparity = depth2disparity (depth);
        float avg_disparity = depth2disparity (avg_depth);
        float min_disparity = disparity - disparity_threshold_;
        float max_disparity = disparity + disparity_threshold_;

        if (avg_disparity > max_disparity)
          return disparity2depth (max_disparity);
        if (avg_disparity < min_disparity)
          return disparity2depth (min_disparity);

        return avg_depth;
      }
  
      // actual kernel operator
      inline __host__ __device__
      float operator () (int idx)
      {
        float depth = data_ [idx];
#ifdef __CUDACC__        
        if (depth == 0 | isnan(depth) | isinf(depth))
          return 0;
#else
        if (depth == 0 | pcl_isnan(depth) | pcl_isinf(depth))
          return 0;
#endif
        int xIdx = idx % width_;
        int yIdx = idx / width_;
        // TODO: test median
        // This implements a fixed window size in image coordinates (pixels)
        int4 bounds = make_int4 (
            xIdx - window_size_,
            xIdx + window_size_,
            yIdx - window_size_,
            yIdx + window_size_
            );
        
        // clamp the coordinates to fit to depth image size
        bounds.x = clamp (bounds.x, 0, width_-1);
        bounds.y = clamp (bounds.y, 0, width_-1);
        bounds.z = clamp (bounds.z, 0, height_-1);
        bounds.w = clamp (bounds.w, 0, height_-1);
    
        float average_depth = depth;
        int counter = 1;
        // iterate over all pixels in the rectangular region
        for (int y = bounds.z; y <= bounds.w; ++y)
        {
          for (int x = bounds.x; x <= bounds.y; ++x)
          {
            // find index in point cloud from x,y pixel positions
            int otherIdx = ((int)y) * width_ + ((int)x);
            float otherDepth = data_[otherIdx];

            // ignore invalid points
            if (otherDepth == 0)
              continue;
            if (fabs(otherDepth - depth) > 200)
              continue;

            ++counter;
            average_depth += otherDepth;
          }
        }

        return clampToDisparityBounds (average_depth / counter, raw_data_[idx]);
      }
    };


    // This version requires a pre-computed map of float3 (nr_valid_points, min_allowable_depth, max_allowable_depth);
    struct DisparityClampedSmoothing
    {
      DisparityClampedSmoothing (float* data, float3* disparity_helper_map, int width, int height, int window_size) 
        : data_(data), disparity_helper_map_(disparity_helper_map), width_(width), height_(height), window_size_(window_size)
      {}

      float* data_;
      float3* disparity_helper_map_;
      int width_;
      int height_;
      int window_size_;

      template <typename Tuple>
      inline __host__ __device__
        float operator () (Tuple t)
      {
        float depth = thrust::get<0> (t);
        int idx = thrust::get<1> (t);
        float3 dhel = disparity_helper_map_[idx];
        int nr = (int) dhel.x;
        float min_d = dhel.y;
        float max_d = dhel.z;
#ifdef __CUDACC__        
        if (depth == 0 | isnan(depth) | isinf(depth))
          return 0.0f;
#else
        if (depth == 0 | pcl_isnan(depth) | pcl_isinf(depth))
          return 0.0f;
#endif
        int xIdx = idx % width_;
        int yIdx = idx / width_;

        // This implements a fixed window size in image coordinates (pixels)
        int4 bounds = make_int4 (
            xIdx - window_size_,
            xIdx + window_size_,
            yIdx - window_size_,
            yIdx + window_size_
            );
        
        // clamp the coordinates to fit to disparity image size
        bounds.x = clamp (bounds.x, 0, width_-1);
        bounds.y = clamp (bounds.y, 0, width_-1);
        bounds.z = clamp (bounds.z, 0, height_-1);
        bounds.w = clamp (bounds.w, 0, height_-1);

        // iterate over all pixels in the rectangular region
        for (int y = bounds.z; y <= bounds.w; ++y)
        {
          for (int x = bounds.x; x <= bounds.y; ++x)
          {
            // find index in point cloud from x,y pixel positions
            int otherIdx = ((int)y) * width_ + ((int)x);
            depth += data_[otherIdx];
          }
        }
      
        return clamp (depth / nr, min_d, max_d);
      }
    };

    struct DisparityHelperMap
    {
      DisparityHelperMap (float* data, int width, int height, int window_size, float baseline, float focal_length, float disp_thresh) 
        : data_(data), width_(width), height_(height), window_size_(window_size), baseline_(baseline), focal_length_(focal_length), disp_thresh_(disp_thresh)
      {}

      float* data_;
      int width_;
      int height_;
      int window_size_;
      float baseline_;
      float focal_length_;
      float disp_thresh_;

      // helper function
      inline __host__ __device__ 
        float disparity2depth (float disparity)
      {
        return baseline_ * focal_length_ / disparity;
      }

      // helper function
      inline __host__ __device__
        float depth2disparity (float depth)
      {
        return baseline_ * focal_length_ / depth;
      }

      inline __host__ __device__
        float3 operator () (int idx)
      {
        float disparity = depth2disparity (data_ [idx]);
#ifdef __CUDACC__         
        if (disparity == 0 | isnan(disparity) | isinf(disparity))
          return make_float3 (0,0,0);
#else
        if (disparity == 0 | pcl_isnan(disparity) | pcl_isinf(disparity))
          return make_float3 (0,0,0);
#endif
        int xIdx = idx % width_;
        int yIdx = idx / width_;

        // This implements a fixed window size in image coordinates (pixels)
        int4 bounds = make_int4 (
            xIdx - window_size_,
            xIdx + window_size_,
            yIdx - window_size_,
            yIdx + window_size_
            );
        
        // clamp the coordinates to fit to disparity image size
        bounds.x = clamp (bounds.x, 0, width_-1);
        bounds.y = clamp (bounds.y, 0, width_-1);
        bounds.z = clamp (bounds.z, 0, height_-1);
        bounds.w = clamp (bounds.w, 0, height_-1);

        int counter = 1;
        // iterate over all pixels in the rectangular region
        for (int y = bounds.z; y <= bounds.w; ++y)
        {
          for (int x = bounds.x; x <= bounds.y; ++x)
          {
            // find index in point cloud from x,y pixel positions
            int otherIdx = ((int)y) * width_ + ((int)x);
            float otherDepth = data_[otherIdx];

            // ignore invalid points
            if (otherDepth > 0)
              ++counter;
          }
        }
        
        return make_float3 ((float) counter, 
                            disparity2depth (disparity + disp_thresh_),
                            disparity2depth (disparity - disp_thresh_));
      }
    };


  } // namespace
} // namespace

#endif

