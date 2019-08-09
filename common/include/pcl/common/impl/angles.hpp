/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
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
 *  FOR a PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
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

#ifndef PCL_COMMON_ANGLES_IMPL_HPP_
#define PCL_COMMON_ANGLES_IMPL_HPP_

#include <cmath>
#include <pcl/pcl_macros.h>

namespace pcl
{
  inline float
  normAngle (float alpha)
  {
    return (alpha >= 0  ? 
        std::fmod (alpha + static_cast<float>(M_PI), 
               2.0f * static_cast<float>(M_PI)) 
        - static_cast<float>(M_PI) 
        : 
        -(std::fmod (static_cast<float>(M_PI) - alpha, 
                 2.0f * static_cast<float>(M_PI)) 
        - static_cast<float>(M_PI)));
  }

  inline float 
  rad2deg (float alpha)
  {
    return (alpha * 57.29578f);
  }

  inline float 
  deg2rad (float alpha)
  {
    return (alpha * 0.017453293f);
  }

  inline double 
  rad2deg (double alpha)
  {
    return (alpha * 57.29578);
  }

  inline double 
  deg2rad (double alpha)
  {
    return (alpha * 0.017453293);
  }
}

#endif  // PCL_COMMON_ANGLES_IMPL_HPP_

