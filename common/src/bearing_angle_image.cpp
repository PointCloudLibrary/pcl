/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Intelligent Robotics Lab, DLUT.
 *  Author: Qinghua Li, Yan Zhuang, Fei Yan
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
 *   * Neither the name of Intelligent Robotics Lab, DLUT. nor the names
 *     of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
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

/**
  * \file bearing_angle_image.cpp
  * \created on: July 07, 2012
  * \author: Qinghua Li (qinghua__li@163.com)
  */

#include <pcl/range_image/bearing_angle_image.h>

namespace pcl
{
/////////////////////////////////////////////////////////
BearingAngleImage::BearingAngleImage () 
{
  reset ();
  unobserved_point_.x = unobserved_point_.y = unobserved_point_.z = 0.0;
  unobserved_point_.rgba = 255;
}

/////////////////////////////////////////////////////////
void
BearingAngleImage::reset ()
{
  width = height = 0;
  points.clear ();
}

/////////////////////////////////////////////////////////
double
BearingAngleImage::getAngle (const PointXYZ &point1, const PointXYZ &point2)
{
  double a, b, c;
  double theta;
  const Eigen::Vector3f& p1 = point1.getVector3fMap ();
  const Eigen::Vector3f& p2 = point2.getVector3fMap ();
  a = p1.squaredNorm ();
  b = (p1 - p2).squaredNorm ();
  c = p2.squaredNorm ();

  if (a != 0 && b != 0)
  {
    theta = std::acos ((a + b - c) / (2 * sqrt (a) * sqrt (b))) * 180 / M_PI;
  }
  else
  {
    theta = 0.0;
  }

  return theta;
}

/////////////////////////////////////////////////////////
void
BearingAngleImage::generateBAImage (PointCloud<PointXYZ>& point_cloud)
{
  width = point_cloud.width;
  height = point_cloud.height;
  unsigned int size = width * height;
  points.clear ();
  points.resize (size, unobserved_point_);

  double theta;
  std::uint8_t r, g, b, gray;

  // primary transformation process
  for (decltype(height) i = 0; i < height - 1; ++i)
  {
    for (decltype(width) j = 0; j < width - 1; ++j)
    {
      theta = getAngle (point_cloud.at (j, i + 1), point_cloud.at (j + 1, i));

      // based on the theta, calculate the gray value of every pixel point
      gray = theta * 255 / 180;
      r = gray;
      g = gray;
      b = gray;

      points[(i + 1) * width + j].x = point_cloud.at (j, i + 1).x;
      points[(i + 1) * width + j].y = point_cloud.at (j, i + 1).y;
      points[(i + 1) * width + j].z = point_cloud.at (j, i + 1).z;
      // set the gray value for every pixel point
      points[(i + 1) * width + j].rgba = ((int)r) << 24 | ((int)g) << 16 | ((int)b) << 8 | 0xff;
    }
  }
}

}  // namespace end
