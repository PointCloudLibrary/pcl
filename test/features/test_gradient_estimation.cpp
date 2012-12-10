/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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
 * $Id$
 *
 */

#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/intensity_gradient.h>

using namespace pcl;
using namespace pcl::io;
using namespace std;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, IntensityGradientEstimation)
{
  // Create a test cloud
  PointCloud<PointXYZI> cloud_xyzi;
  cloud_xyzi.height = 1;
  cloud_xyzi.is_dense = true;
  for (float x = -5.0f; x <= 5.0f; x += 0.1f)
  {
    for (float y = -5.0f; y <= 5.0f; y += 0.1f)
    {
      PointXYZI p;
      p.x = x;
      p.y = y;
      p.z = 0.1f * powf (x, 2.0f) + 0.5f * y + 1.0f;
      p.intensity = 0.1f * powf (x, 3.0f) + 0.2f * powf (y, 2.0f) + 1.0f * p.z + 20000.0f;

      cloud_xyzi.points.push_back (p);
    }
  }
  cloud_xyzi.width = static_cast<uint32_t> (cloud_xyzi.points.size ());
  PointCloud<PointXYZI>::ConstPtr cloud_ptr = cloud_xyzi.makeShared ();

  // Estimate surface normals
  PointCloud<Normal>::Ptr normals (new PointCloud<Normal> ());
  NormalEstimation<PointXYZI, Normal> norm_est;
  norm_est.setInputCloud (cloud_ptr);
  search::KdTree<PointXYZI>::Ptr treept1 (new search::KdTree<PointXYZI> (false));
  norm_est.setSearchMethod (treept1);
  norm_est.setRadiusSearch (0.25);
  norm_est.compute (*normals);

  // Estimate intensity gradient
  PointCloud<IntensityGradient> gradient;
  IntensityGradientEstimation<PointXYZI, Normal, IntensityGradient> grad_est;
  grad_est.setInputCloud (cloud_ptr);
  grad_est.setInputNormals (normals);
  search::KdTree<PointXYZI>::Ptr treept2 (new search::KdTree<PointXYZI> (false));
  grad_est.setSearchMethod (treept2);
  grad_est.setRadiusSearch (0.25);
  grad_est.compute (gradient);

  // Compare to gradient estimates to actual values
  for (size_t i = 0; i < cloud_ptr->points.size (); ++i)
  {
    const PointXYZI &p = cloud_ptr->points[i];

    // A pointer to the estimated gradient values
    const float * g_est = gradient.points[i].gradient;

    // Compute the surface normal analytically.
    float nx = -0.2f * p.x;
    float ny = -0.5f;
    float nz = 1.0f;
    float magnitude = sqrtf (nx * nx + ny * ny + nz * nz);
    nx /= magnitude;
    ny /= magnitude;
    nz /= magnitude;

    // Compute the intensity gradient analytically...
    float tmpx = 0.3f * powf (p.x, 2.0f);
    float tmpy = 0.4f * p.y;
    float tmpz = 1.0f;
    // ...and project the 3-D gradient vector onto the surface's tangent plane.
    float gx = (1 - nx * nx) * tmpx + (-nx * ny) * tmpy + (-nx * nz) * tmpz;
    float gy = (-ny * nx) * tmpx + (1 - ny * ny) * tmpy + (-ny * nz) * tmpz;
    float gz = (-nz * nx) * tmpx + (-nz * ny) * tmpy + (1 - nz * nz) * tmpz;

    // Compare the estimates to the derived values.
    const float tolerance = 0.11f;
    EXPECT_NEAR (g_est[0], gx, tolerance);
    EXPECT_NEAR (g_est[1], gy, tolerance);
    EXPECT_NEAR (g_est[2], gz, tolerance);
  }
}

/* ---[ */
int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
