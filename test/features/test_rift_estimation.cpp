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

#include <pcl/test/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/features/rift.h>

using namespace pcl;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, RIFTEstimation)
{
  // Generate a sample point cloud
  PointCloud<PointXYZI> cloud_xyzi;
  cloud_xyzi.height = 1;
  cloud_xyzi.is_dense = true;
  for (float x = -10.0f; x <= 10.0f; x += 1.0f)
  {
    for (float y = -10.0f; y <= 10.0f; y += 1.0f)
    {
      PointXYZI p;
      p.x = x;
      p.y = y;
      p.z = std::sqrt (400 - x * x - y * y);
      p.intensity = std::exp ((-powf (x - 3.0f, 2.0f) + powf (y + 2.0f, 2.0f)) / (2.0f * 25.0f)) + std::exp ((-powf (x + 5.0f, 2.0f) + powf (y - 5.0f, 2.0f))
                                                                                 / (2.0f * 4.0f));

      cloud_xyzi.push_back (p);
    }
  }
  cloud_xyzi.width = cloud_xyzi.size ();

  // Generate the intensity gradient data
  PointCloud<IntensityGradient> gradient;
  gradient.height = 1;
  gradient.width = cloud_xyzi.size ();
  gradient.is_dense = true;
  gradient.resize (gradient.width);
  for (std::size_t i = 0; i < cloud_xyzi.size (); ++i)
  {
    const PointXYZI &p = cloud_xyzi[i];

    // Compute the surface normal analytically.
    float nx = p.x;
    float ny = p.y;
    float nz = p.z;
    float magnitude = std::sqrt (nx * nx + ny * ny + nz * nz);
    nx /= magnitude;
    ny /= magnitude;
    nz /= magnitude;

    // Compute the intensity gradient analytically...
    float tmpx = -(p.x + 5.0f) / 4.0f / std::exp ((powf (p.x + 5.0f, 2.0f) + powf (p.y - 5.0f, 2.0f)) / 8.0f) - (p.x - 3.0f) / 25.0f
        / std::exp ((powf (p.x - 3.0f, 2.0f) + powf (p.y + 2.0f, 2.0f)) / 50.0f);
    float tmpy = -(p.y - 5.0f) / 4.0f / std::exp ((powf (p.x + 5.0f, 2.0f) + powf (p.y - 5.0f, 2.0f)) / 8.0f) - (p.y + 2.0f) / 25.0f
        / std::exp ((powf (p.x - 3.0f, 2.0f) + powf (p.y + 2.0f, 2.0f)) / 50.0f);
    float tmpz = 0.0f;
    // ...and project the 3-D gradient vector onto the surface's tangent plane.
    float gx = (1 - nx * nx) * tmpx + (-nx * ny) * tmpy + (-nx * nz) * tmpz;
    float gy = (-ny * nx) * tmpx + (1 - ny * ny) * tmpy + (-ny * nz) * tmpz;
    float gz = (-nz * nx) * tmpx + (-nz * ny) * tmpy + (1 - nz * nz) * tmpz;

    gradient[i].gradient[0] = gx;
    gradient[i].gradient[1] = gy;
    gradient[i].gradient[2] = gz;
  }

  // Compute the RIFT features
  using RIFTDescriptor = Histogram<32>;
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
  const RIFTDescriptor &rift = rift_output[220];

  const float correct_rift_feature_values[32] =
     {0.0052f, 0.0349f, 0.0647f, 0.0881f, 0.0042f, 0.0131f, 0.0346f, 0.0030f,
      0.0076f, 0.0218f, 0.0463f, 0.0030f, 0.0087f, 0.0288f, 0.0920f, 0.0472f,
      0.0211f, 0.0420f, 0.0726f, 0.0669f, 0.0090f, 0.0901f, 0.1274f, 0.2185f,
      0.0147f, 0.1222f, 0.3568f, 0.4348f, 0.0149f, 0.0806f, 0.2787f, 0.6864f};
  for (int i = 0; i < 32; ++i)
    EXPECT_NEAR (rift.histogram[i], correct_rift_feature_values[i], 1e-4);
}

/* ---[ */
int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
