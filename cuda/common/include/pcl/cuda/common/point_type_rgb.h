/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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
 */

#pragma once

#include <cuda.h>
#include <pcl/cuda/cutil_math.h>

#ifdef RGB
# undef RGB
#endif

namespace pcl
{
namespace cuda
{

  /** \brief Default RGB structure, defined as a union over 4 chars. */
  union RGB
  {
    int rgb;
    struct
    {
      char r;
      char g;
      char b;
      char alpha;
    };

    inline __host__ __device__ RGB () {}
    inline __host__ __device__ RGB (int _rgb) : rgb(_rgb) {}
    inline __host__ __device__ RGB (char _r, char _g, char _b, char _alpha) :
                                       r(_r), g(_g), b(_b), alpha(_alpha) {}

    inline __host__ __device__ bool operator == (const RGB &rhs) const
    {
      return (r == rhs.r && g == rhs.g && b == rhs.b && alpha == rhs.alpha);
    }

    inline __host__ __device__ RGB operator - (RGB &rhs)
    {
      RGB res = *this;
      res -= rhs;
      return (res);
    }

    inline __host__ __device__ RGB& operator += (const RGB &rhs)
    {
      r += rhs.r;
      g += rhs.g;
      b += rhs.b;
      alpha += rhs.alpha;
      return (*this);
    }

    inline __host__ __device__ RGB& operator -= (const RGB &rhs)
    {
      r -= rhs.r;
      g -= rhs.g;
      b -= rhs.b;
      alpha -= rhs.alpha;
      return (*this);
    }

    inline __host__ __device__ RGB& operator *= (const RGB &rhs)
    {
      r *= rhs.r;
      g *= rhs.g;
      b *= rhs.b;
      alpha *= rhs.alpha;
      return (*this);
    }

    inline __host__ __device__ RGB& operator /= (const RGB &rhs)
    {
      r /= rhs.r;
      g /= rhs.g;
      b /= rhs.b;
      alpha /= rhs.alpha;
      return (*this);
    }
  };

} // namespace
} // namespace
