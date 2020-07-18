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
#include <pcl/range_image/range_image_planar.h>
#include <pcl/pcl_macros.h>

namespace pcl
{

/////////////////////////////////////////////////////////////////////////
template <typename PointCloudType> void
RangeImagePlanar::createFromPointCloudWithFixedSize (const PointCloudType& point_cloud,
                                                     int di_width, int di_height,
                                                     float di_center_x, float di_center_y,
                                                     float di_focal_length_x, float di_focal_length_y,
                                                     const Eigen::Affine3f& sensor_pose,
                                                     CoordinateFrame coordinate_frame, float noise_level,
                                                     float min_range)
{
  //std::cout << "Starting to create range image from "<<point_cloud.size ()<<" points.\n";

  width = di_width;
  height = di_height;
  center_x_ = di_center_x;
  center_y_ = di_center_y;
  focal_length_x_ = di_focal_length_x;
  focal_length_y_ = di_focal_length_y;
  focal_length_x_reciprocal_ = 1 / focal_length_x_;
  focal_length_y_reciprocal_ = 1 / focal_length_y_;

  is_dense = false;
  
  getCoordinateFrameTransformation (coordinate_frame, to_world_system_);
  to_world_system_ = sensor_pose * to_world_system_;

  to_range_image_system_ = to_world_system_.inverse (Eigen::Isometry);

  unsigned int size = width*height;
  points.clear ();
  points.resize (size, unobserved_point);

  int top=height, right=-1, bottom=-1, left=width;
  doZBuffer (point_cloud, noise_level, min_range, top, right, bottom, left);

  // Do not crop
  //cropImage (border_size, top, right, bottom, left);

  recalculate3DPointPositions ();
}


/////////////////////////////////////////////////////////////////////////
void
RangeImagePlanar::calculate3DPoint (float image_x, float image_y, float range, Eigen::Vector3f& point) const
{
  //std::cout << __PRETTY_FUNCTION__ << " called.\n";
  float delta_x = (image_x+static_cast<float> (image_offset_x_)-center_x_)*focal_length_x_reciprocal_,
        delta_y = (image_y+static_cast<float> (image_offset_y_)-center_y_)*focal_length_y_reciprocal_;
  point[2] = range / (std::sqrt (delta_x*delta_x + delta_y*delta_y + 1));
  point[0] = delta_x*point[2];
  point[1] = delta_y*point[2];
  point = to_world_system_ * point;
}

/////////////////////////////////////////////////////////////////////////
inline void 
RangeImagePlanar::getImagePoint (const Eigen::Vector3f& point, float& image_x, float& image_y, float& range) const 
{
  Eigen::Vector3f transformedPoint = to_range_image_system_ * point;
  if (transformedPoint[2]<=0)  // Behind the observer?
  {
    image_x = image_y = range = -1.0f;
    return;
  }
  range = transformedPoint.norm ();
  
  image_x = center_x_ + focal_length_x_*transformedPoint[0]/transformedPoint[2] - static_cast<float> (image_offset_x_);
  image_y = center_y_ + focal_length_y_*transformedPoint[1]/transformedPoint[2] - static_cast<float> (image_offset_y_);
}

}  // namespace pcl

