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

#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/features/rift.h>

using namespace pcl;
using namespace std;

#ifndef PCL_ONLY_CORE_POINT_TYPES
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  TEST (PCL, RIFTEstimation)
  {
    // Generate a sample point cloud
    PointCloud<PointXYZI> cloud_xyzi;
    cloud_xyzi.height = 1;
    cloud_xyzi.is_dense = true;
    for (float x = -10.0; x <= 10.0; x += 1.0)
    {
      for (float y = -10.0; y <= 10.0; y += 1.0)
      {
        PointXYZI p;
        p.x = x;
        p.y = y;
        p.z = sqrt (400 - x * x - y * y);
        p.intensity = exp ((-pow (x - 3, 2) + pow (y + 2, 2)) / (2 * 25.0)) + exp ((-pow (x + 5, 2) + pow (y - 5, 2))
                                                                                   / (2 * 4.0));

        cloud_xyzi.points.push_back (p);
      }
    }
    cloud_xyzi.width = cloud_xyzi.points.size ();

    // Generate the intensity gradient data
    PointCloud<IntensityGradient> gradient;
    gradient.height = 1;
    gradient.width = cloud_xyzi.points.size ();
    gradient.is_dense = true;
    gradient.points.resize (gradient.width);
    for (size_t i = 0; i < cloud_xyzi.points.size (); ++i)
    {
      const PointXYZI &p = cloud_xyzi.points[i];

      // Compute the surface normal analytically.
      float nx = p.x;
      float ny = p.y;
      float nz = p.z;
      float magnitude = sqrt (nx * nx + ny * ny + nz * nz);
      nx /= magnitude;
      ny /= magnitude;
      nz /= magnitude;

      // Compute the intensity gradient analytically...
      float tmpx = -(p.x + 5) / 4.0 / exp ((pow (p.x + 5, 2) + pow (p.y - 5, 2)) / 8.0) - (p.x - 3) / 25.0
          / exp ((pow (p.x - 3, 2) + pow (p.y + 2, 2)) / 50.0);
      float tmpy = -(p.y - 5) / 4.0 / exp ((pow (p.x + 5, 2) + pow (p.y - 5, 2)) / 8.0) - (p.y + 2) / 25.0
          / exp ((pow (p.x - 3, 2) + pow (p.y + 2, 2)) / 50.0);
      float tmpz = 0;
      // ...and project the 3-D gradient vector onto the surface's tangent plane.
      float gx = (1 - nx * nx) * tmpx + (-nx * ny) * tmpy + (-nx * nz) * tmpz;
      float gy = (-ny * nx) * tmpx + (1 - ny * ny) * tmpy + (-ny * nz) * tmpz;
      float gz = (-nz * nx) * tmpx + (-nz * ny) * tmpy + (1 - nz * nz) * tmpz;

      gradient.points[i].gradient[0] = gx;
      gradient.points[i].gradient[1] = gy;
      gradient.points[i].gradient[2] = gz;
    }

    // Compute the RIFT features
    typedef Histogram<32> RIFTDescriptor;
    RIFTEstimation<PointXYZI, IntensityGradient, RIFTDescriptor> rift_est;
    search::KdTree<PointXYZI>::Ptr treept4 (new search::KdTree<PointXYZI> (false));
    rift_est.setSearchMethod (treept4);
    rift_est.setRadiusSearch (10.0);
    rift_est.setNrDistanceBins (4);
    rift_est.setNrGradientBins (8);

    rift_est.setInputCloud (cloud_xyzi.makeShared ());
    rift_est.setInputGradient (gradient.makeShared ());
    PointCloud<RIFTDescriptor> rift_output;
    rift_est.compute (rift_output);

    // Compare to independently verified values
    const RIFTDescriptor &rift = rift_output.points[220];
    const float correct_rift_feature_values[32] = {0.0187, 0.0349, 0.0647, 0.0881, 0.0042, 0.0131, 0.0346, 0.0030,
                                                   0.0076, 0.0218, 0.0463, 0.0030, 0.0087, 0.0288, 0.0920, 0.0472,
                                                   0.0076, 0.0420, 0.0726, 0.0669, 0.0090, 0.0901, 0.1274, 0.2185,
                                                   0.0147, 0.1222, 0.3568, 0.4348, 0.0149, 0.0806, 0.2787, 0.6864};
    for (int i = 0; i < 32; ++i)
      EXPECT_NEAR (rift.histogram[i], correct_rift_feature_values[i], 1e-4);
  }

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  TEST (PCL, RIFTEstimationEigen)
  {
    // Generate a sample point cloud
    PointCloud<PointXYZI> cloud_xyzi;
    cloud_xyzi.height = 1;
    cloud_xyzi.is_dense = true;
    for (float x = -10.0; x <= 10.0; x += 1.0)
    {
      for (float y = -10.0; y <= 10.0; y += 1.0)
      {
        PointXYZI p;
        p.x = x;
        p.y = y;
        p.z = sqrt (400 - x * x - y * y);
        p.intensity = exp ((-pow (x - 3, 2) + pow (y + 2, 2)) / (2 * 25.0)) + exp ((-pow (x + 5, 2) + pow (y - 5, 2))
                                                                                   / (2 * 4.0));

        cloud_xyzi.points.push_back (p);
      }
    }
    cloud_xyzi.width = cloud_xyzi.points.size ();

    // Generate the intensity gradient data
    PointCloud<IntensityGradient> gradient;
    gradient.height = 1;
    gradient.width = cloud_xyzi.points.size ();
    gradient.is_dense = true;
    gradient.points.resize (gradient.width);
    for (size_t i = 0; i < cloud_xyzi.points.size (); ++i)
    {
      const PointXYZI &p = cloud_xyzi.points[i];

      // Compute the surface normal analytically.
      float nx = p.x;
      float ny = p.y;
      float nz = p.z;
      float magnitude = sqrt (nx * nx + ny * ny + nz * nz);
      nx /= magnitude;
      ny /= magnitude;
      nz /= magnitude;

      // Compute the intensity gradient analytically...
      float tmpx = -(p.x + 5) / 4.0 / exp ((pow (p.x + 5, 2) + pow (p.y - 5, 2)) / 8.0) - (p.x - 3) / 25.0
          / exp ((pow (p.x - 3, 2) + pow (p.y + 2, 2)) / 50.0);
      float tmpy = -(p.y - 5) / 4.0 / exp ((pow (p.x + 5, 2) + pow (p.y - 5, 2)) / 8.0) - (p.y + 2) / 25.0
          / exp ((pow (p.x - 3, 2) + pow (p.y + 2, 2)) / 50.0);
      float tmpz = 0;
      // ...and project the 3-D gradient vector onto the surface's tangent plane.
      float gx = (1 - nx * nx) * tmpx + (-nx * ny) * tmpy + (-nx * nz) * tmpz;
      float gy = (-ny * nx) * tmpx + (1 - ny * ny) * tmpy + (-ny * nz) * tmpz;
      float gz = (-nz * nx) * tmpx + (-nz * ny) * tmpy + (1 - nz * nz) * tmpz;

      gradient.points[i].gradient[0] = gx;
      gradient.points[i].gradient[1] = gy;
      gradient.points[i].gradient[2] = gz;
    }

    // Compute the RIFT features
    RIFTEstimation<PointXYZI, IntensityGradient, Eigen::MatrixXf> rift_est;
    search::KdTree<PointXYZI>::Ptr treept4 (new search::KdTree<PointXYZI> (false));
    rift_est.setSearchMethod (treept4);
    rift_est.setRadiusSearch (10.0);
    rift_est.setNrDistanceBins (4);
    rift_est.setNrGradientBins (8);

    rift_est.setInputCloud (cloud_xyzi.makeShared ());
    rift_est.setInputGradient (gradient.makeShared ());
    PointCloud<Eigen::MatrixXf> rift_output;
    rift_est.computeEigen (rift_output);

    // Compare to independently verified values
    Eigen::VectorXf rift = rift_output.points.row (220);
    const float correct_rift_feature_values[32] = {0.0187, 0.0349, 0.0647, 0.0881, 0.0042, 0.0131, 0.0346, 0.0030,
                                                   0.0076, 0.0218, 0.0463, 0.0030, 0.0087, 0.0288, 0.0920, 0.0472,
                                                   0.0076, 0.0420, 0.0726, 0.0669, 0.0090, 0.0901, 0.1274, 0.2185,
                                                   0.0147, 0.1222, 0.3568, 0.4348, 0.0149, 0.0806, 0.2787, 0.6864};
    for (int i = 0; i < 32; ++i)
      ASSERT_NEAR (rift[i], correct_rift_feature_values[i], 1e-4);
  }
#endif

/* ---[ */
int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
