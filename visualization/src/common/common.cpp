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
 * $Id$
 *
 */

#include <pcl/visualization/common/common.h>
#include <stdlib.h>

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::getRandomColors (double &r, double &g, double &b, double min, double max)
{
  double sum;
  static unsigned stepRGBA = 100;
  do
  {
    sum = 0;
    r = (rand () % stepRGBA) / static_cast<double> (stepRGBA);
    while ((g = (rand () % stepRGBA) / static_cast<double> (stepRGBA)) == r) {}
    while (((b = (rand () % stepRGBA) / static_cast<double> (stepRGBA)) == r) && (b == g)) {}
    sum = r + g + b;
  }
  while (sum <= min || sum >= max);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::getRandomColors (pcl::RGB &rgb, double min, double max)
{
  double sum;
  static unsigned stepRGBA = 100;
  double r, g, b;
  do
  {
    sum = 0;
    r = (rand () % stepRGBA) / static_cast<double> (stepRGBA);
    while ((g = (rand () % stepRGBA) / static_cast<double> (stepRGBA)) == r) {}
    while (((b = (rand () % stepRGBA) / static_cast<double> (stepRGBA)) == r) && (b == g)) {}
    sum = r + g + b;
  }
  while (sum <= min || sum >= max);
  rgb.r = uint8_t (r * 255.0);
  rgb.g = uint8_t (g * 255.0);
  rgb.b = uint8_t (b * 255.0);
}

/////////////////////////////////////////////////////////////////////////////////////////////

void
pcl::visualization::Camera::computeViewMatrix(Eigen::Matrix4d& view_mat) const
{
//constructs view matrix from camera pos, view up, and the point it is looking at
//this code is based off of gluLookAt http://www.opengl.org/wiki/GluLookAt_code
	Eigen::Vector3d focal_point (focal[0], focal[1], focal[2]);
	Eigen::Vector3d posv        (pos[0]  , pos[1]  , pos[2]);
	Eigen::Vector3d up          (view[0] , view[1] , view[2]);

	Eigen::Vector3d zAxis = (focal_point - posv).normalized();
  Eigen::Vector3d xAxis = zAxis.cross(up).normalized();
  // make sure the y-axis is orthogonal to the other two
  Eigen::Vector3d yAxis = xAxis.cross (zAxis);

	view_mat.block <1, 3> (0, 0) = xAxis;
	view_mat.block <1, 3> (1, 0) = yAxis;
	view_mat.block <1, 3> (2, 0) = -zAxis;
	view_mat.row (3) << 0, 0, 0, 1;

	view_mat.block <3, 1> (0, 3) = view_mat.topLeftCorner<3, 3> () * (-posv);
}

///////////////////////////////////////////////////////////////////////
void
pcl::visualization::Camera::computeProjectionMatrix (Eigen::Matrix4d& proj) const
{
  float top    = static_cast<float> (clip[0]) * tanf (0.5f * static_cast<float> (fovy));
  float left   = -top * static_cast<float> (window_size[0] / window_size[1]);
  float right  = -left;
  float bottom = -top;

  float temp1, temp2, temp3, temp4;
	temp1 = 2.0f * static_cast<float> (clip[0]);
	temp2 = 1.0f / (right - left);
	temp3 = 1.0f / (top - bottom);
	temp4 = 1.0f / static_cast<float> (clip[1] - clip[0]);

  proj.setZero ();

	proj(0,0) = temp1 * temp2;
	proj(1,1) = temp1 * temp3;
	proj(0,2) = (right + left) * temp2;
	proj(1,2) = (top + bottom) * temp3;
	proj(2,2) = (-clip[1] - clip[0]) * temp4;
	proj(3,2) = -1.0;
	proj(2,3) = (-temp1 * clip[1]) * temp4;
}
