/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012-, Open Perception, Inc.
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

#pragma once

#include <pcl/common/eigen.h>
#include <pcl/range_image/range_image_spherical.h>
#include <pcl/pcl_macros.h>


namespace pcl
{

/////////////////////////////////////////////////////////////////////////
void
RangeImageSpherical::calculate3DPoint (float image_x, float image_y, float range, Eigen::Vector3f& point) const
{
  float angle_x, angle_y;
  getAnglesFromImagePoint (image_x, image_y, angle_x, angle_y);

  float cosY = std::cos (angle_y);
  point = Eigen::Vector3f (range * sinf (angle_x) * cosY, range * sinf (angle_y), range * std::cos (angle_x)*cosY);
  point = to_world_system_ * point;
}

/////////////////////////////////////////////////////////////////////////
inline void 
RangeImageSpherical::getImagePoint (const Eigen::Vector3f& point, float& image_x, float& image_y, float& range) const
{
  Eigen::Vector3f transformedPoint = to_range_image_system_ * point;
  range = transformedPoint.norm ();
  float angle_x = atan2LookUp (transformedPoint[0], transformedPoint[2]),
        angle_y = asinLookUp (transformedPoint[1]/range);
  getImagePointFromAngles (angle_x, angle_y, image_x, image_y);
}
/////////////////////////////////////////////////////////////////////////
void
RangeImageSpherical::getAnglesFromImagePoint (float image_x, float image_y, float& angle_x, float& angle_y) const
{
  angle_y = (image_y+static_cast<float> (image_offset_y_))*angular_resolution_y_ - 0.5f*static_cast<float> (M_PI);
  angle_x = ((image_x+ static_cast<float> (image_offset_x_))*angular_resolution_x_ - static_cast<float> (M_PI));
}
/////////////////////////////////////////////////////////////////////////
void
RangeImageSpherical::getImagePointFromAngles (float angle_x, float angle_y, float& image_x, float& image_y) const
{
  image_x = (angle_x + static_cast<float> (M_PI))*angular_resolution_x_reciprocal_ - static_cast<float> (image_offset_x_);
  image_y = (angle_y + 0.5f*static_cast<float> (M_PI))*angular_resolution_y_reciprocal_ - static_cast<float> (image_offset_y_);
}
}  // namespace pcl

