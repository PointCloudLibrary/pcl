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
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/spin_image.h>
#include <pcl/features/intensity_spin.h>

using namespace pcl;
using namespace pcl::io;

using KdTreePtr = search::KdTree<PointXYZ>::Ptr;

PointCloud<PointXYZ> cloud;
pcl::Indices indices;
KdTreePtr tree;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, SpinImageEstimation)
{
  // Estimate normals first
  double mr = 0.002;
  NormalEstimation<PointXYZ, Normal> n;
  PointCloud<Normal>::Ptr normals (new PointCloud<Normal> ());
  // set parameters
  n.setInputCloud (cloud.makeShared ());
  pcl::IndicesPtr indicesptr (new pcl::Indices (indices));
  n.setIndices (indicesptr);
  n.setSearchMethod (tree);
  n.setRadiusSearch (20 * mr);
  n.compute (*normals);

  EXPECT_NEAR ((*normals)[103].normal_x, 0.36683175, 1e-4);
  EXPECT_NEAR ((*normals)[103].normal_y, -0.44696972, 1e-4);
  EXPECT_NEAR ((*normals)[103].normal_z, -0.81587529, 1e-4);
  EXPECT_NEAR ((*normals)[200].normal_x, -0.71414840, 1e-4);
  EXPECT_NEAR ((*normals)[200].normal_y, -0.06002361, 1e-4);
  EXPECT_NEAR ((*normals)[200].normal_z, -0.69741613, 1e-4);

  EXPECT_NEAR ((*normals)[140].normal_x, -0.45109111, 1e-4);
  EXPECT_NEAR ((*normals)[140].normal_y, -0.19499126, 1e-4);
  EXPECT_NEAR ((*normals)[140].normal_z, -0.87091631, 1e-4);

  using SpinImage = Histogram<153>;
  SpinImageEstimation<PointXYZ, Normal, SpinImage> spin_est(8, 0.5, 16);
  // set parameters
  //spin_est.setInputWithNormals (cloud.makeShared (), normals);
  spin_est.setInputCloud (cloud.makeShared ());
  spin_est.setInputNormals (normals);
  spin_est.setIndices (indicesptr);
  spin_est.setSearchMethod (tree);
  spin_est.setRadiusSearch (40*mr);

  // Object
  PointCloud<SpinImage>::Ptr spin_images (new PointCloud<SpinImage> ());


  // radial SI
  spin_est.setRadialStructure();

  // estimate
  spin_est.compute (*spin_images);
  EXPECT_EQ (spin_images->size (), indices.size ());

  EXPECT_NEAR ((*spin_images)[100].histogram[0], 0, 1e-4);
  EXPECT_NEAR ((*spin_images)[100].histogram[12], 0, 1e-4);
  EXPECT_NEAR ((*spin_images)[100].histogram[24], 0.00233226, 1e-4);
  EXPECT_NEAR ((*spin_images)[100].histogram[36], 0, 1e-4);
  EXPECT_NEAR ((*spin_images)[100].histogram[48], 8.48662e-005, 1e-4);
  EXPECT_NEAR ((*spin_images)[100].histogram[60], 0.0266387, 1e-4);
  EXPECT_NEAR ((*spin_images)[100].histogram[72], 0, 1e-4);
  EXPECT_NEAR ((*spin_images)[100].histogram[84], 0, 1e-4);
  EXPECT_NEAR ((*spin_images)[100].histogram[96], 0.0414662, 1e-4);
  EXPECT_NEAR ((*spin_images)[100].histogram[108], 0, 1e-4);
  EXPECT_NEAR ((*spin_images)[100].histogram[120], 0, 1e-4);
  EXPECT_NEAR ((*spin_images)[100].histogram[132], 0, 1e-4);
  EXPECT_NEAR ((*spin_images)[100].histogram[144], 0.0128513, 1e-4);
  EXPECT_NEAR ((*spin_images)[300].histogram[0], 0, 1e-4);
  EXPECT_NEAR ((*spin_images)[300].histogram[12], 0, 1e-4);
  EXPECT_NEAR ((*spin_images)[300].histogram[24], 0.00932424, 1e-4);
  EXPECT_NEAR ((*spin_images)[300].histogram[36], 0, 1e-4);
  EXPECT_NEAR ((*spin_images)[300].histogram[48], 0, 1e-4);
  EXPECT_NEAR ((*spin_images)[300].histogram[60], 0.0145733, 1e-4);
  EXPECT_NEAR ((*spin_images)[300].histogram[72], 0, 1e-4);
  EXPECT_NEAR ((*spin_images)[300].histogram[84], 0, 1e-4);
  EXPECT_NEAR ((*spin_images)[300].histogram[96], 0.00034457, 1e-4);
  EXPECT_NEAR ((*spin_images)[300].histogram[108], 0, 1e-4);
  EXPECT_NEAR ((*spin_images)[300].histogram[120], 0, 1e-4);
  EXPECT_NEAR ((*spin_images)[300].histogram[132], 0, 1e-4);
  EXPECT_NEAR ((*spin_images)[300].histogram[144], 0.0121195, 1e-4);

  // radial SI, angular spin-images
  spin_est.setAngularDomain ();

  // estimate
  spin_est.compute (*spin_images);
  EXPECT_EQ (spin_images->size (), indices.size ());

  EXPECT_NEAR ((*spin_images)[100].histogram[0], 0, 1e-4);
  EXPECT_NEAR ((*spin_images)[100].histogram[12], 0, 1e-4);
  EXPECT_NEAR ((*spin_images)[100].histogram[24], 0.132139, 1e-4);
  EXPECT_NEAR ((*spin_images)[100].histogram[36], 0, 1e-4);
  EXPECT_NEAR ((*spin_images)[100].histogram[48], 0.908814, 1e-4);
  EXPECT_NEAR ((*spin_images)[100].histogram[60], 0.63875, 1e-4);
  EXPECT_NEAR ((*spin_images)[100].histogram[72], 0, 1e-4);
  EXPECT_NEAR ((*spin_images)[100].histogram[84], 0, 1e-4);
  EXPECT_NEAR ((*spin_images)[100].histogram[96], 0.550392, 1e-4);
  EXPECT_NEAR ((*spin_images)[100].histogram[108], 0, 1e-4);
  EXPECT_NEAR ((*spin_images)[100].histogram[120], 0, 1e-4);
  EXPECT_NEAR ((*spin_images)[100].histogram[132], 0, 1e-4);
  EXPECT_NEAR ((*spin_images)[100].histogram[144], 0.257136, 1e-4);
  EXPECT_NEAR ((*spin_images)[300].histogram[0], 0, 1e-4);
  EXPECT_NEAR ((*spin_images)[300].histogram[12], 0, 1e-4);
  EXPECT_NEAR ((*spin_images)[300].histogram[24], 0.230605, 1e-4);
  EXPECT_NEAR ((*spin_images)[300].histogram[36], 0, 1e-4);
  EXPECT_NEAR ((*spin_images)[300].histogram[48], 0, 1e-4);
  EXPECT_NEAR ((*spin_images)[300].histogram[60], 0.764872, 1e-4);
  EXPECT_NEAR ((*spin_images)[300].histogram[72], 0, 1e-4);
  EXPECT_NEAR ((*spin_images)[300].histogram[84], 0, 1e-4);
  EXPECT_NEAR ((*spin_images)[300].histogram[96], 1.02824, 1e-4);
  EXPECT_NEAR ((*spin_images)[300].histogram[108], 0, 1e-4);
  EXPECT_NEAR ((*spin_images)[300].histogram[120], 0, 1e-4);
  EXPECT_NEAR ((*spin_images)[300].histogram[132], 0, 1e-4);
  EXPECT_NEAR ((*spin_images)[300].histogram[144], 0.293567, 1e-4);

  // rectangular SI
  spin_est.setRadialStructure (false);
  spin_est.setAngularDomain (false);

  // estimate
  spin_est.compute (*spin_images);
  EXPECT_EQ (spin_images->size (), indices.size ());

  EXPECT_NEAR ((*spin_images)[100].histogram[0], 0, 1e-4);
  EXPECT_NEAR ((*spin_images)[100].histogram[12], 0, 1e-4);
  EXPECT_NEAR ((*spin_images)[100].histogram[24], 0.000889345, 1e-4);
  EXPECT_NEAR ((*spin_images)[100].histogram[36], 0, 1e-4);
  EXPECT_NEAR ((*spin_images)[100].histogram[48], 0, 1e-4);
  EXPECT_NEAR ((*spin_images)[100].histogram[60], 0.0489534, 1e-4);
  EXPECT_NEAR ((*spin_images)[100].histogram[72], 0, 1e-4);
  EXPECT_NEAR ((*spin_images)[100].histogram[84], 0, 1e-4);
  EXPECT_NEAR ((*spin_images)[100].histogram[96], 0.0747141, 1e-4);
  EXPECT_NEAR ((*spin_images)[100].histogram[108], 0, 1e-4);
  EXPECT_NEAR ((*spin_images)[100].histogram[120], 0, 1e-4);
  EXPECT_NEAR ((*spin_images)[100].histogram[132], 0.0173423, 1e-4);
  EXPECT_NEAR ((*spin_images)[100].histogram[144], 0, 1e-4);
  EXPECT_NEAR ((*spin_images)[300].histogram[0], 0, 1e-4);
  EXPECT_NEAR ((*spin_images)[300].histogram[12], 0, 1e-4);
  EXPECT_NEAR ((*spin_images)[300].histogram[24], 0.0267132, 1e-4);
  EXPECT_NEAR ((*spin_images)[300].histogram[36], 0, 1e-4);
  EXPECT_NEAR ((*spin_images)[300].histogram[48], 0, 1e-4);
  EXPECT_NEAR ((*spin_images)[300].histogram[60], 0, 1e-4);
  EXPECT_NEAR ((*spin_images)[300].histogram[72], 0, 1e-4);
  EXPECT_NEAR ((*spin_images)[300].histogram[84], 0, 1e-4);
  EXPECT_NEAR ((*spin_images)[300].histogram[96], 0, 1e-4);
  EXPECT_NEAR ((*spin_images)[300].histogram[108], 0.0209709, 1e-4);
  EXPECT_NEAR ((*spin_images)[300].histogram[120], 0, 1e-4);
  EXPECT_NEAR ((*spin_images)[300].histogram[132], 0, 1e-4);
  EXPECT_NEAR ((*spin_images)[300].histogram[144], 0.029372, 1e-4);

  // rectangular SI, angular spin-images
  spin_est.setAngularDomain ();

  // estimate
  spin_est.compute (*spin_images);
  EXPECT_EQ (spin_images->size (), indices.size ());

  EXPECT_NEAR ((*spin_images)[100].histogram[0], 0, 1e-4);
  EXPECT_NEAR ((*spin_images)[100].histogram[12], 0, 1e-4);
  EXPECT_NEAR ((*spin_images)[100].histogram[24], 0.132139, 1e-4);
  EXPECT_NEAR ((*spin_images)[100].histogram[36], 0, 1e-4);
  EXPECT_NEAR ((*spin_images)[100].histogram[48], 0, 1e-4);
  EXPECT_NEAR ((*spin_images)[100].histogram[60], 0.38800787925720215, 1e-4);
  EXPECT_NEAR ((*spin_images)[100].histogram[72], 0, 1e-4);
  EXPECT_NEAR ((*spin_images)[100].histogram[84], 0, 1e-4);
  EXPECT_NEAR ((*spin_images)[100].histogram[96], 0.468881, 1e-4);
  EXPECT_NEAR ((*spin_images)[100].histogram[108], 0, 1e-4);
  EXPECT_NEAR ((*spin_images)[100].histogram[120], 0, 1e-4);
  EXPECT_NEAR ((*spin_images)[100].histogram[132], 0.67901438474655151, 1e-4);
  EXPECT_NEAR ((*spin_images)[100].histogram[144], 0, 1e-4);
  EXPECT_NEAR ((*spin_images)[300].histogram[0], 0, 1e-4);
  EXPECT_NEAR ((*spin_images)[300].histogram[12], 0, 1e-4);
  EXPECT_NEAR ((*spin_images)[300].histogram[24], 0.143845, 1e-4);
  EXPECT_NEAR ((*spin_images)[300].histogram[36], 0, 1e-4);
  EXPECT_NEAR ((*spin_images)[300].histogram[48], 0, 1e-4);
  EXPECT_NEAR ((*spin_images)[300].histogram[60], 0, 1e-4);
  EXPECT_NEAR ((*spin_images)[300].histogram[72], 0, 1e-4);
  EXPECT_NEAR ((*spin_images)[300].histogram[84], 0, 1e-4);
  EXPECT_NEAR ((*spin_images)[300].histogram[96], 0, 1e-4);
  EXPECT_NEAR ((*spin_images)[300].histogram[108], 0.706084, 1e-4);
  EXPECT_NEAR ((*spin_images)[300].histogram[120], 0, 1e-4);
  EXPECT_NEAR ((*spin_images)[300].histogram[132], 0, 1e-4);
  EXPECT_NEAR ((*spin_images)[300].histogram[144], 0.272542, 1e-4);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, IntensitySpinEstimation)
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
      p.z = std::sqrt (400.0f - x * x - y * y);
      p.intensity = std::exp (-(powf (x - 3.0f, 2.0f) + powf (y + 2.0f, 2.0f)) / (2.0f * 25.0f)) + std::exp (-(powf (x + 5.0f, 2.0f) + powf (y - 5.0f, 2.0f))
                                                                                 / (2.0f * 4.0f));

      cloud_xyzi.push_back (p);
    }
  }
  cloud_xyzi.width = cloud_xyzi.size ();

  // Compute the intensity-domain spin features
  using IntensitySpin = Histogram<20>;
  IntensitySpinEstimation<PointXYZI, IntensitySpin> ispin_est;
  search::KdTree<PointXYZI>::Ptr treept3 (new search::KdTree<PointXYZI> (false));
  ispin_est.setSearchMethod (treept3);
  ispin_est.setRadiusSearch (10.0);
  ispin_est.setNrDistanceBins (4);
  ispin_est.setNrIntensityBins (5);

  ispin_est.setInputCloud (cloud_xyzi.makeShared ());
  PointCloud<IntensitySpin> ispin_output;
  ispin_est.compute (ispin_output);

  // Compare to independently verified values
  const IntensitySpin &ispin = ispin_output[220];
  const float correct_ispin_feature_values[20] = {2.4387f, 9.4737f, 21.3232f, 28.3025f, 22.5639f, 13.2426f, 35.7026f, 60.0755f,
                                                  66.9240f, 50.4225f, 42.7086f, 83.5818f, 105.4513f, 97.8454f, 67.3801f,
                                                  75.7127f, 119.4726f, 120.9649f, 93.4829f, 55.4045f};
  for (int i = 0; i < 20; ++i)
  {
    EXPECT_NEAR (ispin.histogram[i], correct_ispin_feature_values[i], 1e-4);
  }
}

/* ---[ */
int
main (int argc, char** argv)
{
  if (argc < 2)
  {
    std::cerr << "No test file given. Please download `bun0.pcd` and pass its path to the test." << std::endl;
    return (-1);
  }

  if (loadPCDFile<PointXYZ> (argv[1], cloud) < 0)
  {
    std::cerr << "Failed to read test file. Please download `bun0.pcd` and pass its path to the test." << std::endl;
    return (-1);
  }

  indices.resize (cloud.size ());
  for (std::size_t i = 0; i < indices.size (); ++i)
    indices[i] = static_cast<int> (i);

  tree.reset (new search::KdTree<PointXYZ> (false));
  tree->setInputCloud (cloud.makeShared ());

  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
