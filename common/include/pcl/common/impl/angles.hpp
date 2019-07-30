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
    const float m_pi_float (static_cast<float> (M_PI));
    return (alpha >= 0.0f  ?
        fmodf (alpha + m_pi_float, 2.0f * m_pi_float) - m_pi_float
        : 
        -(fmodf (m_pi_float - alpha, 2.0f * m_pi_float) - m_pi_float));
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


  inline float
  getRotationAngle (const Eigen::Vector3f &point0,
                    const Eigen::Vector3f &point1,
                    const Eigen::Vector3f &center,
                    const Eigen::Vector4f &plane)
  {
    const Eigen::Vector3f vec0 = (point0 - center).normalized ();
    const Eigen::Vector3f vec1 = (point1 - center).normalized ();

    // Use vec0 as "X-axis" and the cross product of vec0 and -plane.normal as "Y-axis",
    // and use atan2 to get the unique angle (the range of atan2 is 2*pi).
    // Reverse the normal vector of plane should make the angle alpha to be 2*pi-alpha
    const float sin_val = vec0.cross (-Eigen::Vector3f (plane.segment (0, 3))).dot (vec1);
    const float cos_val = vec0.dot (vec1);
    float angle = std::atan2 (sin_val, cos_val);
    if (angle < 0.0f) // -pi~pi -> 0~2pi
    {
      angle += (float) (2.0 * M_PI);
    }
    return angle;
  }

}

#endif  // PCL_COMMON_ANGLES_IMPL_HPP_

