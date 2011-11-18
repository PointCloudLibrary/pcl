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

#ifndef PCL_CUDA_PREDICATE_H_
#define PCL_CUDA_PREDICATE_H_

//#if THRUST_DEVICE_COMPILER == THRUST_DEVICE_COMPILER_NVCC
//#undef __MMX__
#include <pcl/cuda/point_cloud.h>
#include <pcl/cuda/point_types.h>
//#else
//#endif


namespace pcl
{
namespace cuda
{
  template <class T>
  struct isNotZero
  {
      __inline__ __host__ __device__ bool 
      operator()(T x) { return (x != 0); }
  };

  struct isInlier
  {
      __inline__ __host__ __device__ bool 
      operator()(int x) { return (x != -1); }
  };

  struct isNotInlier
  {
      __inline__ __host__ __device__ bool 
      operator()(int x) { return (x == -1); }
  };

  struct SetColor
  {
    SetColor (const OpenNIRGB& color) : color_(color) {}
    __inline__ __host__ __device__ void 
       operator()(PointXYZRGB& point) { point.rgb.r = color_.r; point.rgb.g = color_.g; point.rgb.b = color_.b;}
    OpenNIRGB color_;
  };

  struct ChangeColor
  {
    ChangeColor (const OpenNIRGB& color) : color_(color) {}
    __inline__ __host__ __device__ PointXYZRGB&
       operator()(PointXYZRGB& point)
       {
         point.rgb.r = color_.r; point.rgb.g = color_.g; point.rgb.b = color_.b;
         return point;
       }
    OpenNIRGB color_;
  };

} // namespace
} // namespace

#endif  //#ifndef PCL_CUDA_PREDICATE_H_


