/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
 *  Copyright (c) 2014-, Open Perception, Inc.
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

#include <gtest/gtest.h>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/sample_consensus/sac_model_cone.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/sample_consensus/sac_model_circle.h>
#include <pcl/sample_consensus/sac_model_circle3d.h>
#include <pcl/sample_consensus/sac_model_normal_sphere.h>

using namespace pcl;

typedef SampleConsensusModelSphere<PointXYZ>::Ptr SampleConsensusModelSpherePtr;
typedef SampleConsensusModelCone<PointXYZ, Normal>::Ptr SampleConsensusModelConePtr;
typedef SampleConsensusModelCircle2D<PointXYZ>::Ptr SampleConsensusModelCircle2DPtr;
typedef SampleConsensusModelCircle3D<PointXYZ>::Ptr SampleConsensusModelCircle3DPtr;
typedef SampleConsensusModelCylinder<PointXYZ, Normal>::Ptr SampleConsensusModelCylinderPtr;
typedef SampleConsensusModelNormalSphere<PointXYZ, Normal>::Ptr SampleConsensusModelNormalSpherePtr;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (SampleConsensusModelSphere, RANSAC)
{
  srand (0);

  // Use a custom point cloud for these tests until we need something better
  PointCloud<PointXYZ> cloud;
  cloud.points.resize (10);
  cloud.points[0].getVector3fMap () << 1.7068f, 1.0684f, 2.2147f;
  cloud.points[1].getVector3fMap () << 2.4708f, 2.3081f, 1.1736f;
  cloud.points[2].getVector3fMap () << 2.7609f, 1.9095f, 1.3574f;
  cloud.points[3].getVector3fMap () << 2.8016f, 1.6704f, 1.5009f;
  cloud.points[4].getVector3fMap () << 1.8517f, 2.0276f, 1.0112f;
  cloud.points[5].getVector3fMap () << 1.8726f, 1.3539f, 2.7523f;
  cloud.points[6].getVector3fMap () << 2.5179f, 2.3218f, 1.2074f;
  cloud.points[7].getVector3fMap () << 2.4026f, 2.5114f, 2.7588f;
  cloud.points[8].getVector3fMap () << 2.6999f, 2.5606f, 1.5571f;
  cloud.points[9].getVector3fMap () << 0.0000f, 0.0000f, 0.0000f;

  // Create a shared sphere model pointer directly
  SampleConsensusModelSpherePtr model (new SampleConsensusModelSphere<PointXYZ> (cloud.makeShared ()));

  // Create the RANSAC object
  RandomSampleConsensus<PointXYZ> sac (model, 0.03);

  // Algorithm tests
  bool result = sac.computeModel ();
  ASSERT_TRUE (result);

  std::vector<int> sample;
  sac.getModel (sample);
  EXPECT_EQ (4, sample.size ());

  std::vector<int> inliers;
  sac.getInliers (inliers);
  EXPECT_EQ (9, inliers.size ());

  Eigen::VectorXf coeff;
  sac.getModelCoefficients (coeff);
  EXPECT_EQ (4, coeff.size ());
  EXPECT_NEAR (2, coeff[0] / coeff[3], 1e-2);
  EXPECT_NEAR (2, coeff[1] / coeff[3], 1e-2);
  EXPECT_NEAR (2, coeff[2] / coeff[3], 1e-2);

  Eigen::VectorXf coeff_refined;
  model->optimizeModelCoefficients (inliers, coeff, coeff_refined);
  EXPECT_EQ (4, coeff_refined.size ());
  EXPECT_NEAR (2, coeff_refined[0] / coeff_refined[3], 1e-2);
  EXPECT_NEAR (2, coeff_refined[1] / coeff_refined[3], 1e-2);
  EXPECT_NEAR (2, coeff_refined[2] / coeff_refined[3], 1e-2);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (SampleConsensusModelNormalSphere, RANSAC)
{
  srand (0);

  // Use a custom point cloud for these tests until we need something better
  PointCloud<PointXYZ> cloud;
  PointCloud<Normal> normals;
  cloud.points.resize (27); normals.points.resize (27);
  cloud.points[ 0].getVector3fMap () << -0.014695f,  0.009549f, 0.954775f;
  cloud.points[ 1].getVector3fMap () <<  0.014695f,  0.009549f, 0.954775f;
  cloud.points[ 2].getVector3fMap () << -0.014695f,  0.040451f, 0.954775f;
  cloud.points[ 3].getVector3fMap () <<  0.014695f,  0.040451f, 0.954775f;
  cloud.points[ 4].getVector3fMap () << -0.009082f, -0.015451f, 0.972049f;
  cloud.points[ 5].getVector3fMap () <<  0.009082f, -0.015451f, 0.972049f;
  cloud.points[ 6].getVector3fMap () << -0.038471f,  0.009549f, 0.972049f;
  cloud.points[ 7].getVector3fMap () <<  0.038471f,  0.009549f, 0.972049f;
  cloud.points[ 8].getVector3fMap () << -0.038471f,  0.040451f, 0.972049f;
  cloud.points[ 9].getVector3fMap () <<  0.038471f,  0.040451f, 0.972049f;
  cloud.points[10].getVector3fMap () << -0.009082f,  0.065451f, 0.972049f;
  cloud.points[11].getVector3fMap () <<  0.009082f,  0.065451f, 0.972049f;
  cloud.points[12].getVector3fMap () << -0.023776f, -0.015451f, 0.982725f;
  cloud.points[13].getVector3fMap () <<  0.023776f, -0.015451f, 0.982725f;
  cloud.points[14].getVector3fMap () << -0.023776f,  0.065451f, 0.982725f;
  cloud.points[15].getVector3fMap () <<  0.023776f,  0.065451f, 0.982725f;
  cloud.points[16].getVector3fMap () << -0.000000f, -0.025000f, 1.000000f;
  cloud.points[17].getVector3fMap () <<  0.000000f, -0.025000f, 1.000000f;
  cloud.points[18].getVector3fMap () << -0.029389f, -0.015451f, 1.000000f;
  cloud.points[19].getVector3fMap () <<  0.029389f, -0.015451f, 1.000000f;
  cloud.points[20].getVector3fMap () << -0.047553f,  0.009549f, 1.000000f;
  cloud.points[21].getVector3fMap () <<  0.047553f,  0.009549f, 1.000000f;
  cloud.points[22].getVector3fMap () << -0.047553f,  0.040451f, 1.000000f;
  cloud.points[23].getVector3fMap () <<  0.047553f,  0.040451f, 1.000000f;
  cloud.points[24].getVector3fMap () << -0.029389f,  0.065451f, 1.000000f;
  cloud.points[25].getVector3fMap () <<  0.029389f,  0.065451f, 1.000000f;
  cloud.points[26].getVector3fMap () <<  0.000000f,  0.075000f, 1.000000f;

  normals.points[ 0].getNormalVector3fMap () << -0.293893f, -0.309017f, -0.904509f;
  normals.points[ 1].getNormalVector3fMap () <<  0.293893f, -0.309017f, -0.904508f;
  normals.points[ 2].getNormalVector3fMap () << -0.293893f,  0.309017f, -0.904509f;
  normals.points[ 3].getNormalVector3fMap () <<  0.293893f,  0.309017f, -0.904508f;
  normals.points[ 4].getNormalVector3fMap () << -0.181636f, -0.809017f, -0.559017f;
  normals.points[ 5].getNormalVector3fMap () <<  0.181636f, -0.809017f, -0.559017f;
  normals.points[ 6].getNormalVector3fMap () << -0.769421f, -0.309017f, -0.559017f;
  normals.points[ 7].getNormalVector3fMap () <<  0.769421f, -0.309017f, -0.559017f;
  normals.points[ 8].getNormalVector3fMap () << -0.769421f,  0.309017f, -0.559017f;
  normals.points[ 9].getNormalVector3fMap () <<  0.769421f,  0.309017f, -0.559017f;
  normals.points[10].getNormalVector3fMap () << -0.181636f,  0.809017f, -0.559017f;
  normals.points[11].getNormalVector3fMap () <<  0.181636f,  0.809017f, -0.559017f;
  normals.points[12].getNormalVector3fMap () << -0.475528f, -0.809017f, -0.345491f;
  normals.points[13].getNormalVector3fMap () <<  0.475528f, -0.809017f, -0.345491f;
  normals.points[14].getNormalVector3fMap () << -0.475528f,  0.809017f, -0.345491f;
  normals.points[15].getNormalVector3fMap () <<  0.475528f,  0.809017f, -0.345491f;
  normals.points[16].getNormalVector3fMap () << -0.000000f, -1.000000f,  0.000000f;
  normals.points[17].getNormalVector3fMap () <<  0.000000f, -1.000000f,  0.000000f;
  normals.points[18].getNormalVector3fMap () << -0.587785f, -0.809017f,  0.000000f;
  normals.points[19].getNormalVector3fMap () <<  0.587785f, -0.809017f,  0.000000f;
  normals.points[20].getNormalVector3fMap () << -0.951057f, -0.309017f,  0.000000f;
  normals.points[21].getNormalVector3fMap () <<  0.951057f, -0.309017f,  0.000000f;
  normals.points[22].getNormalVector3fMap () << -0.951057f,  0.309017f,  0.000000f;
  normals.points[23].getNormalVector3fMap () <<  0.951057f,  0.309017f,  0.000000f;
  normals.points[24].getNormalVector3fMap () << -0.587785f,  0.809017f,  0.000000f;
  normals.points[25].getNormalVector3fMap () <<  0.587785f,  0.809017f,  0.000000f;
  normals.points[26].getNormalVector3fMap () <<  0.000000f,  1.000000f,  0.000000f;

  // Create a shared sphere model pointer directly
  SampleConsensusModelNormalSpherePtr model (new SampleConsensusModelNormalSphere<PointXYZ, Normal> (cloud.makeShared ()));
  model->setInputNormals (normals.makeShared ());

  // Create the RANSAC object
  RandomSampleConsensus<PointXYZ> sac (model, 0.03);

  // Algorithm tests
  bool result = sac.computeModel ();
  ASSERT_TRUE (result);

  std::vector<int> sample;
  sac.getModel (sample);
  EXPECT_EQ (4, sample.size ());

  std::vector<int> inliers;
  sac.getInliers (inliers);
  EXPECT_EQ (27, inliers.size ());

  Eigen::VectorXf coeff;
  sac.getModelCoefficients (coeff);
  EXPECT_EQ (4, coeff.size ());
  EXPECT_NEAR (0.000, coeff[0], 1e-2);
  EXPECT_NEAR (0.025, coeff[1], 1e-2);
  EXPECT_NEAR (1.000, coeff[2], 1e-2);
  EXPECT_NEAR (0.050, coeff[3], 1e-2);

  Eigen::VectorXf coeff_refined;
  model->optimizeModelCoefficients (inliers, coeff, coeff_refined);
  EXPECT_EQ (4, coeff_refined.size ());
  EXPECT_NEAR (0.000, coeff_refined[0], 1e-2);
  EXPECT_NEAR (0.025, coeff_refined[1], 1e-2);
  EXPECT_NEAR (1.000, coeff_refined[2], 1e-2);
  EXPECT_NEAR (0.050, coeff_refined[3], 1e-2);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (SampleConsensusModelCone, RANSAC)
{
  srand (0);

  // Use a custom point cloud for these tests until we need something better
  PointCloud<PointXYZ> cloud;
  PointCloud<Normal> normals;
  cloud.points.resize (31); normals.points.resize (31);

  cloud.points[ 0].getVector3fMap () << -0.011247f, 0.200000f, 0.965384f;
  cloud.points[ 1].getVector3fMap () <<  0.000000f, 0.200000f, 0.963603f;
  cloud.points[ 2].getVector3fMap () <<  0.011247f, 0.200000f, 0.965384f;
  cloud.points[ 3].getVector3fMap () << -0.016045f, 0.175000f, 0.977916f;
  cloud.points[ 4].getVector3fMap () << -0.008435f, 0.175000f, 0.974038f;
  cloud.points[ 5].getVector3fMap () <<  0.004218f, 0.175000f, 0.973370f;
  cloud.points[ 6].getVector3fMap () <<  0.016045f, 0.175000f, 0.977916f;
  cloud.points[ 7].getVector3fMap () << -0.025420f, 0.200000f, 0.974580f;
  cloud.points[ 8].getVector3fMap () <<  0.025420f, 0.200000f, 0.974580f;
  cloud.points[ 9].getVector3fMap () << -0.012710f, 0.150000f, 0.987290f;
  cloud.points[10].getVector3fMap () << -0.005624f, 0.150000f, 0.982692f;
  cloud.points[11].getVector3fMap () <<  0.002812f, 0.150000f, 0.982247f;
  cloud.points[12].getVector3fMap () <<  0.012710f, 0.150000f, 0.987290f;
  cloud.points[13].getVector3fMap () << -0.022084f, 0.175000f, 0.983955f;
  cloud.points[14].getVector3fMap () <<  0.022084f, 0.175000f, 0.983955f;
  cloud.points[15].getVector3fMap () << -0.034616f, 0.200000f, 0.988753f;
  cloud.points[16].getVector3fMap () <<  0.034616f, 0.200000f, 0.988753f;
  cloud.points[17].getVector3fMap () << -0.006044f, 0.125000f, 0.993956f;
  cloud.points[18].getVector3fMap () <<  0.004835f, 0.125000f, 0.993345f;
  cloud.points[19].getVector3fMap () << -0.017308f, 0.150000f, 0.994376f;
  cloud.points[20].getVector3fMap () <<  0.017308f, 0.150000f, 0.994376f;
  cloud.points[21].getVector3fMap () << -0.025962f, 0.175000f, 0.991565f;
  cloud.points[22].getVector3fMap () <<  0.025962f, 0.175000f, 0.991565f;
  cloud.points[23].getVector3fMap () << -0.009099f, 0.125000f, 1.000000f;
  cloud.points[24].getVector3fMap () <<  0.009099f, 0.125000f, 1.000000f;
  cloud.points[25].getVector3fMap () << -0.018199f, 0.150000f, 1.000000f;
  cloud.points[26].getVector3fMap () <<  0.018199f, 0.150000f, 1.000000f;
  cloud.points[27].getVector3fMap () << -0.027298f, 0.175000f, 1.000000f;
  cloud.points[28].getVector3fMap () <<  0.027298f, 0.175000f, 1.000000f;
  cloud.points[29].getVector3fMap () << -0.036397f, 0.200000f, 1.000000f;
  cloud.points[30].getVector3fMap () <<  0.036397f, 0.200000f, 1.000000f;

  normals.points[ 0].getNormalVector3fMap () << -0.290381f, -0.342020f, -0.893701f;
  normals.points[ 1].getNormalVector3fMap () <<  0.000000f, -0.342020f, -0.939693f;
  normals.points[ 2].getNormalVector3fMap () <<  0.290381f, -0.342020f, -0.893701f;
  normals.points[ 3].getNormalVector3fMap () << -0.552338f, -0.342020f, -0.760227f;
  normals.points[ 4].getNormalVector3fMap () << -0.290381f, -0.342020f, -0.893701f;
  normals.points[ 5].getNormalVector3fMap () <<  0.145191f, -0.342020f, -0.916697f;
  normals.points[ 6].getNormalVector3fMap () <<  0.552337f, -0.342020f, -0.760227f;
  normals.points[ 7].getNormalVector3fMap () << -0.656282f, -0.342020f, -0.656283f;
  normals.points[ 8].getNormalVector3fMap () <<  0.656282f, -0.342020f, -0.656283f;
  normals.points[ 9].getNormalVector3fMap () << -0.656283f, -0.342020f, -0.656282f;
  normals.points[10].getNormalVector3fMap () << -0.290381f, -0.342020f, -0.893701f;
  normals.points[11].getNormalVector3fMap () <<  0.145191f, -0.342020f, -0.916697f;
  normals.points[12].getNormalVector3fMap () <<  0.656282f, -0.342020f, -0.656282f;
  normals.points[13].getNormalVector3fMap () << -0.760228f, -0.342020f, -0.552337f;
  normals.points[14].getNormalVector3fMap () <<  0.760228f, -0.342020f, -0.552337f;
  normals.points[15].getNormalVector3fMap () << -0.893701f, -0.342020f, -0.290380f;
  normals.points[16].getNormalVector3fMap () <<  0.893701f, -0.342020f, -0.290380f;
  normals.points[17].getNormalVector3fMap () << -0.624162f, -0.342020f, -0.624162f;
  normals.points[18].getNormalVector3fMap () <<  0.499329f, -0.342020f, -0.687268f;
  normals.points[19].getNormalVector3fMap () << -0.893701f, -0.342020f, -0.290380f;
  normals.points[20].getNormalVector3fMap () <<  0.893701f, -0.342020f, -0.290380f;
  normals.points[21].getNormalVector3fMap () << -0.893701f, -0.342020f, -0.290381f;
  normals.points[22].getNormalVector3fMap () <<  0.893701f, -0.342020f, -0.290381f;
  normals.points[23].getNormalVector3fMap () << -0.939693f, -0.342020f,  0.000000f;
  normals.points[24].getNormalVector3fMap () <<  0.939693f, -0.342020f,  0.000000f;
  normals.points[25].getNormalVector3fMap () << -0.939693f, -0.342020f,  0.000000f;
  normals.points[26].getNormalVector3fMap () <<  0.939693f, -0.342020f,  0.000000f;
  normals.points[27].getNormalVector3fMap () << -0.939693f, -0.342020f,  0.000000f;
  normals.points[28].getNormalVector3fMap () <<  0.939693f, -0.342020f,  0.000000f;
  normals.points[29].getNormalVector3fMap () << -0.939693f, -0.342020f,  0.000000f;
  normals.points[30].getNormalVector3fMap () <<  0.939693f, -0.342020f,  0.000000f;

  // Create a shared cylinder model pointer directly
  SampleConsensusModelConePtr model (new SampleConsensusModelCone<PointXYZ, Normal> (cloud.makeShared ()));
  model->setInputNormals (normals.makeShared ());

  // Create the RANSAC object
  RandomSampleConsensus<PointXYZ> sac (model, 0.03);

  // Algorithm tests
  bool result = sac.computeModel ();
  ASSERT_TRUE (result);

  std::vector<int> sample;
  sac.getModel (sample);
  EXPECT_EQ (3, sample.size ());

  std::vector<int> inliers;
  sac.getInliers (inliers);
  EXPECT_EQ (31, inliers.size ());

  Eigen::VectorXf coeff;
  sac.getModelCoefficients (coeff);
  EXPECT_EQ (7, coeff.size ());
  EXPECT_NEAR (0.000000, coeff[0], 1e-2);
  EXPECT_NEAR (0.100000, coeff[1], 1e-2);
  EXPECT_NEAR (0.349066, coeff[6], 1e-2);

  Eigen::VectorXf coeff_refined;
  model->optimizeModelCoefficients (inliers, coeff, coeff_refined);
  EXPECT_EQ (7, coeff_refined.size ());
  EXPECT_NEAR (0.349066, coeff_refined[6], 1e-2);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (SampleConsensusModelCylinder, RANSAC)
{
  srand (0);

  // Use a custom point cloud for these tests until we need something better
  PointCloud<PointXYZ> cloud;
  PointCloud<Normal> normals;
  cloud.points.resize (20); normals.points.resize (20);

  cloud.points[ 0].getVector3fMap () << -0.499902f, 2.199701f, 0.000008f;
  cloud.points[ 1].getVector3fMap () << -0.875397f, 2.030177f, 0.050104f;
  cloud.points[ 2].getVector3fMap () << -0.995875f, 1.635973f, 0.099846f;
  cloud.points[ 3].getVector3fMap () << -0.779523f, 1.285527f, 0.149961f;
  cloud.points[ 4].getVector3fMap () << -0.373285f, 1.216488f, 0.199959f;
  cloud.points[ 5].getVector3fMap () << -0.052893f, 1.475973f, 0.250101f;
  cloud.points[ 6].getVector3fMap () << -0.036558f, 1.887591f, 0.299839f;
  cloud.points[ 7].getVector3fMap () << -0.335048f, 2.171994f, 0.350001f;
  cloud.points[ 8].getVector3fMap () << -0.745456f, 2.135528f, 0.400072f;
  cloud.points[ 9].getVector3fMap () << -0.989282f, 1.803311f, 0.449983f;
  cloud.points[10].getVector3fMap () << -0.900651f, 1.400701f, 0.500126f;
  cloud.points[11].getVector3fMap () << -0.539658f, 1.201468f, 0.550079f;
  cloud.points[12].getVector3fMap () << -0.151875f, 1.340951f, 0.599983f;
  cloud.points[13].getVector3fMap () << -0.000724f, 1.724373f, 0.649882f;
  cloud.points[14].getVector3fMap () << -0.188573f, 2.090983f, 0.699854f;
  cloud.points[15].getVector3fMap () << -0.587925f, 2.192257f, 0.749956f;
  cloud.points[16].getVector3fMap () << -0.927724f, 1.958846f, 0.800008f;
  cloud.points[17].getVector3fMap () << -0.976888f, 1.549655f, 0.849970f;
  cloud.points[18].getVector3fMap () << -0.702003f, 1.242707f, 0.899954f;
  cloud.points[19].getVector3fMap () << -0.289916f, 1.246296f, 0.950075f;

  normals.points[ 0].getNormalVector3fMap () <<  0.000098f,  1.000098f,  0.000008f;
  normals.points[ 1].getNormalVector3fMap () << -0.750891f,  0.660413f,  0.000104f;
  normals.points[ 2].getNormalVector3fMap () << -0.991765f, -0.127949f, -0.000154f;
  normals.points[ 3].getNormalVector3fMap () << -0.558918f, -0.829439f, -0.000039f;
  normals.points[ 4].getNormalVector3fMap () <<  0.253627f, -0.967447f, -0.000041f;
  normals.points[ 5].getNormalVector3fMap () <<  0.894105f, -0.447965f,  0.000101f;
  normals.points[ 6].getNormalVector3fMap () <<  0.926852f,  0.375543f, -0.000161f;
  normals.points[ 7].getNormalVector3fMap () <<  0.329948f,  0.943941f,  0.000001f;
  normals.points[ 8].getNormalVector3fMap () << -0.490966f,  0.871203f,  0.000072f;
  normals.points[ 9].getNormalVector3fMap () << -0.978507f,  0.206425f, -0.000017f;
  normals.points[10].getNormalVector3fMap () << -0.801227f, -0.598534f,  0.000126f;
  normals.points[11].getNormalVector3fMap () << -0.079447f, -0.996697f,  0.000079f;
  normals.points[12].getNormalVector3fMap () <<  0.696154f, -0.717889f, -0.000017f;
  normals.points[13].getNormalVector3fMap () <<  0.998685f,  0.048502f, -0.000118f;
  normals.points[14].getNormalVector3fMap () <<  0.622933f,  0.782133f, -0.000146f;
  normals.points[15].getNormalVector3fMap () << -0.175948f,  0.984480f, -0.000044f;
  normals.points[16].getNormalVector3fMap () << -0.855476f,  0.517824f,  0.000008f;
  normals.points[17].getNormalVector3fMap () << -0.953769f, -0.300571f, -0.000030f;
  normals.points[18].getNormalVector3fMap () << -0.404035f, -0.914700f, -0.000046f;
  normals.points[19].getNormalVector3fMap () <<  0.420154f, -0.907445f,  0.000075f;

  // Create a shared cylinder model pointer directly
  SampleConsensusModelCylinderPtr model (new SampleConsensusModelCylinder<PointXYZ, Normal> (cloud.makeShared ()));
  model->setInputNormals (normals.makeShared ());

  // Create the RANSAC object
  RandomSampleConsensus<PointXYZ> sac (model, 0.03);

  // Algorithm tests
  bool result = sac.computeModel ();
  ASSERT_TRUE (result);

  std::vector<int> sample;
  sac.getModel (sample);
  EXPECT_EQ (2, sample.size ());

  std::vector<int> inliers;
  sac.getInliers (inliers);
  EXPECT_EQ (20, inliers.size ());

  Eigen::VectorXf coeff;
  sac.getModelCoefficients (coeff);
  EXPECT_EQ (7, coeff.size ());
  EXPECT_NEAR (-0.5, coeff[0], 1e-3);
  EXPECT_NEAR ( 1.7, coeff[1], 1e-3);
  EXPECT_NEAR ( 0.5, coeff[6], 1e-3);

  Eigen::VectorXf coeff_refined;
  model->optimizeModelCoefficients (inliers, coeff, coeff_refined);
  EXPECT_EQ (7, coeff_refined.size ());
  EXPECT_NEAR (0.5, coeff_refined[6], 1e-3);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (SampleConsensusModelCircle2D, RANSAC)
{
  srand (0);

  // Use a custom point cloud for these tests until we need something better
  PointCloud<PointXYZ> cloud;
  cloud.points.resize (18);

  cloud.points[ 0].getVector3fMap () << 3.587751f, -4.190982f, 0.0f;
  cloud.points[ 1].getVector3fMap () << 3.808883f, -4.412265f, 0.0f;
  cloud.points[ 2].getVector3fMap () << 3.587525f, -5.809143f, 0.0f;
  cloud.points[ 3].getVector3fMap () << 2.999913f, -5.999980f, 0.0f;
  cloud.points[ 4].getVector3fMap () << 2.412224f, -5.809090f, 0.0f;
  cloud.points[ 5].getVector3fMap () << 2.191080f, -5.587682f, 0.0f;
  cloud.points[ 6].getVector3fMap () << 2.048941f, -5.309003f, 0.0f;
  cloud.points[ 7].getVector3fMap () << 2.000397f, -4.999944f, 0.0f;
  cloud.points[ 8].getVector3fMap () << 2.999953f, -6.000056f, 0.0f;
  cloud.points[ 9].getVector3fMap () << 2.691127f, -5.951136f, 0.0f;
  cloud.points[10].getVector3fMap () << 2.190892f, -5.587838f, 0.0f;
  cloud.points[11].getVector3fMap () << 2.048874f, -5.309052f, 0.0f;
  cloud.points[12].getVector3fMap () << 1.999990f, -5.000147f, 0.0f;
  cloud.points[13].getVector3fMap () << 2.049026f, -4.690918f, 0.0f;
  cloud.points[14].getVector3fMap () << 2.190956f, -4.412162f, 0.0f;
  cloud.points[15].getVector3fMap () << 2.412231f, -4.190918f, 0.0f;
  cloud.points[16].getVector3fMap () << 2.691027f, -4.049060f, 0.0f;
  cloud.points[17].getVector3fMap () << 2.000000f, -3.000000f, 0.0f;

  // Create a shared 2d circle model pointer directly
  SampleConsensusModelCircle2DPtr model (new SampleConsensusModelCircle2D<PointXYZ> (cloud.makeShared ()));

  // Create the RANSAC object
  RandomSampleConsensus<PointXYZ> sac (model, 0.03);

  // Algorithm tests
  bool result = sac.computeModel ();
  ASSERT_TRUE (result);

  std::vector<int> sample;
  sac.getModel (sample);
  EXPECT_EQ (3, sample.size ());

  std::vector<int> inliers;
  sac.getInliers (inliers);
  EXPECT_EQ (17, inliers.size ());

  Eigen::VectorXf coeff;
  sac.getModelCoefficients (coeff);
  EXPECT_EQ (3, coeff.size ());
  EXPECT_NEAR ( 3, coeff[0], 1e-3);
  EXPECT_NEAR (-5, coeff[1], 1e-3);
  EXPECT_NEAR ( 1, coeff[2], 1e-3);

  Eigen::VectorXf coeff_refined;
  model->optimizeModelCoefficients (inliers, coeff, coeff_refined);
  EXPECT_EQ (3, coeff_refined.size ());
  EXPECT_NEAR ( 3, coeff_refined[0], 1e-3);
  EXPECT_NEAR (-5, coeff_refined[1], 1e-3);
  EXPECT_NEAR ( 1, coeff_refined[2], 1e-3);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (SampleConsensusModelCircle3D, RANSAC)
{
  srand (0);

  // Use a custom point cloud for these tests until we need something better
  PointCloud<PointXYZ> cloud;
  cloud.points.resize (20);

  cloud.points[ 0].getVector3fMap () << 1.00000000f, 5.0000000f, -2.9000001f;
  cloud.points[ 1].getVector3fMap () << 1.03420200f, 5.0000000f, -2.9060307f;
  cloud.points[ 2].getVector3fMap () << 1.06427870f, 5.0000000f, -2.9233956f;
  cloud.points[ 3].getVector3fMap () << 1.08660260f, 5.0000000f, -2.9500000f;
  cloud.points[ 4].getVector3fMap () << 1.09848080f, 5.0000000f, -2.9826353f;
  cloud.points[ 5].getVector3fMap () << 1.09848080f, 5.0000000f, -3.0173647f;
  cloud.points[ 6].getVector3fMap () << 1.08660260f, 5.0000000f, -3.0500000f;
  cloud.points[ 7].getVector3fMap () << 1.06427870f, 5.0000000f, -3.0766044f;
  cloud.points[ 8].getVector3fMap () << 1.03420200f, 5.0000000f, -3.0939693f;
  cloud.points[ 9].getVector3fMap () << 1.00000000f, 5.0000000f, -3.0999999f;
  cloud.points[10].getVector3fMap () << 0.96579796f, 5.0000000f, -3.0939693f;
  cloud.points[11].getVector3fMap () << 0.93572122f, 5.0000000f, -3.0766044f;
  cloud.points[12].getVector3fMap () << 0.91339743f, 5.0000000f, -3.0500000f;
  cloud.points[13].getVector3fMap () << 0.90151924f, 5.0000000f, -3.0173647f;
  cloud.points[14].getVector3fMap () << 0.90151924f, 5.0000000f, -2.9826353f;
  cloud.points[15].getVector3fMap () << 0.91339743f, 5.0000000f, -2.9500000f;
  cloud.points[16].getVector3fMap () << 0.93572122f, 5.0000000f, -2.9233956f;
  cloud.points[17].getVector3fMap () << 0.96579796f, 5.0000000f, -2.9060307f;
  cloud.points[18].getVector3fMap () << 0.85000002f, 4.8499999f, -3.1500001f;
  cloud.points[19].getVector3fMap () << 1.15000000f, 5.1500001f, -2.8499999f;

  // Create a shared 3d circle model pointer directly
  SampleConsensusModelCircle3DPtr model (new SampleConsensusModelCircle3D<PointXYZ> (cloud.makeShared ()));

  // Create the RANSAC object
  RandomSampleConsensus<PointXYZ> sac (model, 0.03);

  // Algorithm tests
  bool result = sac.computeModel ();
  ASSERT_TRUE (result);

  std::vector<int> sample;
  sac.getModel (sample);
  EXPECT_EQ (3, sample.size ());

  std::vector<int> inliers;
  sac.getInliers (inliers);
  EXPECT_EQ (18, inliers.size ());

  Eigen::VectorXf coeff;
  sac.getModelCoefficients (coeff);
  EXPECT_EQ (7, coeff.size ());
  EXPECT_NEAR ( 1.0, coeff[0], 1e-3);
  EXPECT_NEAR ( 5.0, coeff[1], 1e-3);
  EXPECT_NEAR (-3.0, coeff[2], 1e-3);
  EXPECT_NEAR ( 0.1, coeff[3], 1e-3);
  EXPECT_NEAR ( 0.0, coeff[4], 1e-3);
  EXPECT_NEAR (-1.0, coeff[5], 1e-3);
  EXPECT_NEAR ( 0.0, coeff[6], 1e-3);

  Eigen::VectorXf coeff_refined;
  model->optimizeModelCoefficients (inliers, coeff, coeff_refined);
  EXPECT_EQ (7, coeff_refined.size ());
  EXPECT_NEAR ( 1.0, coeff_refined[0], 1e-3);
  EXPECT_NEAR ( 5.0, coeff_refined[1], 1e-3);
  EXPECT_NEAR (-3.0, coeff_refined[2], 1e-3);
  EXPECT_NEAR ( 0.1, coeff_refined[3], 1e-3);
  EXPECT_NEAR ( 0.0, coeff_refined[4], 1e-3);
  EXPECT_NEAR (-1.0, coeff_refined[5], 1e-3);
  EXPECT_NEAR ( 0.0, coeff_refined[6], 1e-3);
}

int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}

