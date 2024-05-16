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

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_circle.h>
#include <pcl/sample_consensus/sac_model_circle3d.h>
#include <pcl/sample_consensus/sac_model_cone.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/sample_consensus/sac_model_ellipse3d.h>
#include <pcl/sample_consensus/sac_model_normal_sphere.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/sample_consensus/sac_model_torus.h>
#include <pcl/test/gtest.h>
#include <pcl/pcl_tests.h> // for EXPECT_XYZ_NEAR

using namespace pcl;

using SampleConsensusModelSpherePtr = SampleConsensusModelSphere<PointXYZ>::Ptr;
using SampleConsensusModelConePtr = SampleConsensusModelCone<PointXYZ, Normal>::Ptr;
using SampleConsensusModelCircle2DPtr = SampleConsensusModelCircle2D<PointXYZ>::Ptr;
using SampleConsensusModelCircle3DPtr = SampleConsensusModelCircle3D<PointXYZ>::Ptr;
using SampleConsensusModelCylinderPtr =
    SampleConsensusModelCylinder<PointXYZ, Normal>::Ptr;
using SampleConsensusModelNormalSpherePtr =
    SampleConsensusModelNormalSphere<PointXYZ, Normal>::Ptr;
using SampleConsensusModelEllipse3DPtr = SampleConsensusModelEllipse3D<PointXYZ>::Ptr;
using SampleConsensusModelTorusPtr = SampleConsensusModelTorus<PointXYZ,Normal>::Ptr;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST(SampleConsensusModelSphere, RANSAC)
{
  srand(0);

  // Use a custom point cloud for these tests until we need something better
  PointCloud<PointXYZ> cloud;
  cloud.resize(10);
  cloud[0].getVector3fMap() << 1.7068f, 1.0684f, 2.2147f;
  cloud[1].getVector3fMap() << 2.4708f, 2.3081f, 1.1736f;
  cloud[2].getVector3fMap() << 2.7609f, 1.9095f, 1.3574f;
  cloud[3].getVector3fMap() << 2.8016f, 1.6704f, 1.5009f;
  cloud[4].getVector3fMap() << 1.8517f, 2.0276f, 1.0112f;
  cloud[5].getVector3fMap() << 1.8726f, 1.3539f, 2.7523f;
  cloud[6].getVector3fMap() << 2.5179f, 2.3218f, 1.2074f;
  cloud[7].getVector3fMap() << 2.4026f, 2.5114f, 2.7588f;
  cloud[8].getVector3fMap() << 2.6999f, 2.5606f, 1.5571f;
  cloud[9].getVector3fMap() << 0.0000f, 0.0000f, 0.0000f;

  // Create a shared sphere model pointer directly
  SampleConsensusModelSpherePtr model(
      new SampleConsensusModelSphere<PointXYZ>(cloud.makeShared()));

  // Create the RANSAC object
  RandomSampleConsensus<PointXYZ> sac(model, 0.03);

  // Algorithm tests
  bool result = sac.computeModel();
  ASSERT_TRUE(result);

  pcl::Indices sample;
  sac.getModel(sample);
  EXPECT_EQ(4, sample.size());

  pcl::Indices inliers;
  sac.getInliers(inliers);
  EXPECT_EQ(9, inliers.size());

  Eigen::VectorXf coeff;
  sac.getModelCoefficients(coeff);
  EXPECT_EQ(4, coeff.size());
  EXPECT_NEAR(2, coeff[0] / coeff[3], 1e-2);
  EXPECT_NEAR(2, coeff[1] / coeff[3], 1e-2);
  EXPECT_NEAR(2, coeff[2] / coeff[3], 1e-2);

  Eigen::VectorXf coeff_refined;
  model->optimizeModelCoefficients(inliers, coeff, coeff_refined);
  EXPECT_EQ(4, coeff_refined.size());
  EXPECT_NEAR(2, coeff_refined[0] / coeff_refined[3], 1e-2);
  EXPECT_NEAR(2, coeff_refined[1] / coeff_refined[3], 1e-2);
  EXPECT_NEAR(2, coeff_refined[2] / coeff_refined[3], 1e-2);
}

//////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
class SampleConsensusModelSphereTest : private SampleConsensusModelSphere<PointT> {
public:
  using SampleConsensusModelSphere<PointT>::SampleConsensusModelSphere;
  using SampleConsensusModelSphere<PointT>::countWithinDistanceStandard;
#if defined(__SSE__) && defined(__SSE2__) && defined(__SSE4_1__)
  using SampleConsensusModelSphere<PointT>::countWithinDistanceSSE;
#endif
#if defined(__AVX__) && defined(__AVX2__)
  using SampleConsensusModelSphere<PointT>::countWithinDistanceAVX;
#endif
};

TEST(SampleConsensusModelSphere,
     SIMD_countWithinDistance) // Test if all countWithinDistance implementations return
                               // the same value
{
  const auto seed = static_cast<unsigned>(std::time(nullptr));
  srand(seed);
  for (size_t i = 0; i < 100; i++) // Run as often as you like
  {
    // Generate a cloud with 1000 random points
    PointCloud<PointXYZ> cloud;
    pcl::Indices indices;
    cloud.resize(1000);
    for (std::size_t idx = 0; idx < cloud.size(); ++idx) {
      cloud[idx].x = 2.0 * static_cast<float>(rand()) / RAND_MAX - 1.0;
      cloud[idx].y = 2.0 * static_cast<float>(rand()) / RAND_MAX - 1.0;
      cloud[idx].z = 2.0 * static_cast<float>(rand()) / RAND_MAX - 1.0;
      if (rand() % 3 != 0) {
        indices.push_back(static_cast<int>(idx));
      }
    }
    SampleConsensusModelSphereTest<PointXYZ> model(cloud.makeShared(), indices, true);

    // Generate random sphere model parameters
    Eigen::VectorXf model_coefficients(4);
    model_coefficients << 2.0 * static_cast<float>(rand()) / RAND_MAX - 1.0,
        2.0 * static_cast<float>(rand()) / RAND_MAX - 1.0,
        2.0 * static_cast<float>(rand()) / RAND_MAX - 1.0,
        0.15 * static_cast<float>(rand()) / RAND_MAX; // center and radius

    const double threshold =
        0.15 * static_cast<double>(rand()) / RAND_MAX; // threshold in [0; 0.1]

    // The number of inliers is usually somewhere between 0 and 10
    const auto res_standard =
        model.countWithinDistanceStandard(model_coefficients, threshold); // Standard
    PCL_DEBUG(
        "seed=%lu, i=%lu, model=(%f, %f, %f, %f), threshold=%f, res_standard=%lu\n",
        seed,
        i,
        model_coefficients(0),
        model_coefficients(1),
        model_coefficients(2),
        model_coefficients(3),
        threshold,
        res_standard);
#if defined(__SSE__) && defined(__SSE2__) && defined(__SSE4_1__)
    const auto res_sse =
        model.countWithinDistanceSSE(model_coefficients, threshold); // SSE
    ASSERT_EQ(res_standard, res_sse);
#endif
#if defined(__AVX__) && defined(__AVX2__)
    const auto res_avx =
        model.countWithinDistanceAVX(model_coefficients, threshold); // AVX
    ASSERT_EQ(res_standard, res_avx);
#endif
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST(SampleConsensusModelNormalSphere, RANSAC)
{
  srand(0);

  // Use a custom point cloud for these tests until we need something better
  PointCloud<PointXYZ> cloud;
  PointCloud<Normal> normals;
  cloud.resize(27);
  normals.resize(27);
  cloud[0].getVector3fMap() << -0.014695f, 0.009549f, 0.954775f;
  cloud[1].getVector3fMap() << 0.014695f, 0.009549f, 0.954775f;
  cloud[2].getVector3fMap() << -0.014695f, 0.040451f, 0.954775f;
  cloud[3].getVector3fMap() << 0.014695f, 0.040451f, 0.954775f;
  cloud[4].getVector3fMap() << -0.009082f, -0.015451f, 0.972049f;
  cloud[5].getVector3fMap() << 0.009082f, -0.015451f, 0.972049f;
  cloud[6].getVector3fMap() << -0.038471f, 0.009549f, 0.972049f;
  cloud[7].getVector3fMap() << 0.038471f, 0.009549f, 0.972049f;
  cloud[8].getVector3fMap() << -0.038471f, 0.040451f, 0.972049f;
  cloud[9].getVector3fMap() << 0.038471f, 0.040451f, 0.972049f;
  cloud[10].getVector3fMap() << -0.009082f, 0.065451f, 0.972049f;
  cloud[11].getVector3fMap() << 0.009082f, 0.065451f, 0.972049f;
  cloud[12].getVector3fMap() << -0.023776f, -0.015451f, 0.982725f;
  cloud[13].getVector3fMap() << 0.023776f, -0.015451f, 0.982725f;
  cloud[14].getVector3fMap() << -0.023776f, 0.065451f, 0.982725f;
  cloud[15].getVector3fMap() << 0.023776f, 0.065451f, 0.982725f;
  cloud[16].getVector3fMap() << -0.000000f, -0.025000f, 1.000000f;
  cloud[17].getVector3fMap() << 0.000000f, -0.025000f, 1.000000f;
  cloud[18].getVector3fMap() << -0.029389f, -0.015451f, 1.000000f;
  cloud[19].getVector3fMap() << 0.029389f, -0.015451f, 1.000000f;
  cloud[20].getVector3fMap() << -0.047553f, 0.009549f, 1.000000f;
  cloud[21].getVector3fMap() << 0.047553f, 0.009549f, 1.000000f;
  cloud[22].getVector3fMap() << -0.047553f, 0.040451f, 1.000000f;
  cloud[23].getVector3fMap() << 0.047553f, 0.040451f, 1.000000f;
  cloud[24].getVector3fMap() << -0.029389f, 0.065451f, 1.000000f;
  cloud[25].getVector3fMap() << 0.029389f, 0.065451f, 1.000000f;
  cloud[26].getVector3fMap() << 0.000000f, 0.075000f, 1.000000f;

  normals[0].getNormalVector3fMap() << -0.293893f, -0.309017f, -0.904509f;
  normals[1].getNormalVector3fMap() << 0.293893f, -0.309017f, -0.904508f;
  normals[2].getNormalVector3fMap() << -0.293893f, 0.309017f, -0.904509f;
  normals[3].getNormalVector3fMap() << 0.293893f, 0.309017f, -0.904508f;
  normals[4].getNormalVector3fMap() << -0.181636f, -0.809017f, -0.559017f;
  normals[5].getNormalVector3fMap() << 0.181636f, -0.809017f, -0.559017f;
  normals[6].getNormalVector3fMap() << -0.769421f, -0.309017f, -0.559017f;
  normals[7].getNormalVector3fMap() << 0.769421f, -0.309017f, -0.559017f;
  normals[8].getNormalVector3fMap() << -0.769421f, 0.309017f, -0.559017f;
  normals[9].getNormalVector3fMap() << 0.769421f, 0.309017f, -0.559017f;
  normals[10].getNormalVector3fMap() << -0.181636f, 0.809017f, -0.559017f;
  normals[11].getNormalVector3fMap() << 0.181636f, 0.809017f, -0.559017f;
  normals[12].getNormalVector3fMap() << -0.475528f, -0.809017f, -0.345491f;
  normals[13].getNormalVector3fMap() << 0.475528f, -0.809017f, -0.345491f;
  normals[14].getNormalVector3fMap() << -0.475528f, 0.809017f, -0.345491f;
  normals[15].getNormalVector3fMap() << 0.475528f, 0.809017f, -0.345491f;
  normals[16].getNormalVector3fMap() << -0.000000f, -1.000000f, 0.000000f;
  normals[17].getNormalVector3fMap() << 0.000000f, -1.000000f, 0.000000f;
  normals[18].getNormalVector3fMap() << -0.587785f, -0.809017f, 0.000000f;
  normals[19].getNormalVector3fMap() << 0.587785f, -0.809017f, 0.000000f;
  normals[20].getNormalVector3fMap() << -0.951057f, -0.309017f, 0.000000f;
  normals[21].getNormalVector3fMap() << 0.951057f, -0.309017f, 0.000000f;
  normals[22].getNormalVector3fMap() << -0.951057f, 0.309017f, 0.000000f;
  normals[23].getNormalVector3fMap() << 0.951057f, 0.309017f, 0.000000f;
  normals[24].getNormalVector3fMap() << -0.587785f, 0.809017f, 0.000000f;
  normals[25].getNormalVector3fMap() << 0.587785f, 0.809017f, 0.000000f;
  normals[26].getNormalVector3fMap() << 0.000000f, 1.000000f, 0.000000f;

  // Create a shared sphere model pointer directly
  SampleConsensusModelNormalSpherePtr model(
      new SampleConsensusModelNormalSphere<PointXYZ, Normal>(cloud.makeShared()));
  model->setInputNormals(normals.makeShared());

  // Create the RANSAC object
  RandomSampleConsensus<PointXYZ> sac(model, 0.03);

  // Algorithm tests
  bool result = sac.computeModel();
  ASSERT_TRUE(result);

  pcl::Indices sample;
  sac.getModel(sample);
  EXPECT_EQ(4, sample.size());

  pcl::Indices inliers;
  sac.getInliers(inliers);
  EXPECT_EQ(27, inliers.size());

  Eigen::VectorXf coeff;
  sac.getModelCoefficients(coeff);
  EXPECT_EQ(4, coeff.size());
  EXPECT_NEAR(0.000, coeff[0], 1e-2);
  EXPECT_NEAR(0.025, coeff[1], 1e-2);
  EXPECT_NEAR(1.000, coeff[2], 1e-2);
  EXPECT_NEAR(0.050, coeff[3], 1e-2);

  Eigen::VectorXf coeff_refined;
  model->optimizeModelCoefficients(inliers, coeff, coeff_refined);
  EXPECT_EQ(4, coeff_refined.size());
  EXPECT_NEAR(0.000, coeff_refined[0], 1e-2);
  EXPECT_NEAR(0.025, coeff_refined[1], 1e-2);
  EXPECT_NEAR(1.000, coeff_refined[2], 1e-2);
  EXPECT_NEAR(0.050, coeff_refined[3], 1e-2);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST(SampleConsensusModelCone, RANSAC)
{
  srand(0);

  // Use a custom point cloud for these tests until we need something better
  PointCloud<PointXYZ> cloud;
  PointCloud<Normal> normals;
  cloud.resize(31);
  normals.resize(31);

  cloud[0].getVector3fMap() << -0.011247f, 0.200000f, 0.965384f;
  cloud[1].getVector3fMap() << 0.000000f, 0.200000f, 0.963603f;
  cloud[2].getVector3fMap() << 0.011247f, 0.200000f, 0.965384f;
  cloud[3].getVector3fMap() << -0.016045f, 0.175000f, 0.977916f;
  cloud[4].getVector3fMap() << -0.008435f, 0.175000f, 0.974038f;
  cloud[5].getVector3fMap() << 0.004218f, 0.175000f, 0.973370f;
  cloud[6].getVector3fMap() << 0.016045f, 0.175000f, 0.977916f;
  cloud[7].getVector3fMap() << -0.025420f, 0.200000f, 0.974580f;
  cloud[8].getVector3fMap() << 0.025420f, 0.200000f, 0.974580f;
  cloud[9].getVector3fMap() << -0.012710f, 0.150000f, 0.987290f;
  cloud[10].getVector3fMap() << -0.005624f, 0.150000f, 0.982692f;
  cloud[11].getVector3fMap() << 0.002812f, 0.150000f, 0.982247f;
  cloud[12].getVector3fMap() << 0.012710f, 0.150000f, 0.987290f;
  cloud[13].getVector3fMap() << -0.022084f, 0.175000f, 0.983955f;
  cloud[14].getVector3fMap() << 0.022084f, 0.175000f, 0.983955f;
  cloud[15].getVector3fMap() << -0.034616f, 0.200000f, 0.988753f;
  cloud[16].getVector3fMap() << 0.034616f, 0.200000f, 0.988753f;
  cloud[17].getVector3fMap() << -0.006044f, 0.125000f, 0.993956f;
  cloud[18].getVector3fMap() << 0.004835f, 0.125000f, 0.993345f;
  cloud[19].getVector3fMap() << -0.017308f, 0.150000f, 0.994376f;
  cloud[20].getVector3fMap() << 0.017308f, 0.150000f, 0.994376f;
  cloud[21].getVector3fMap() << -0.025962f, 0.175000f, 0.991565f;
  cloud[22].getVector3fMap() << 0.025962f, 0.175000f, 0.991565f;
  cloud[23].getVector3fMap() << -0.009099f, 0.125000f, 1.000000f;
  cloud[24].getVector3fMap() << 0.009099f, 0.125000f, 1.000000f;
  cloud[25].getVector3fMap() << -0.018199f, 0.150000f, 1.000000f;
  cloud[26].getVector3fMap() << 0.018199f, 0.150000f, 1.000000f;
  cloud[27].getVector3fMap() << -0.027298f, 0.175000f, 1.000000f;
  cloud[28].getVector3fMap() << 0.027298f, 0.175000f, 1.000000f;
  cloud[29].getVector3fMap() << -0.036397f, 0.200000f, 1.000000f;
  cloud[30].getVector3fMap() << 0.036397f, 0.200000f, 1.000000f;

  normals[0].getNormalVector3fMap() << -0.290381f, -0.342020f, -0.893701f;
  normals[1].getNormalVector3fMap() << 0.000000f, -0.342020f, -0.939693f;
  normals[2].getNormalVector3fMap() << 0.290381f, -0.342020f, -0.893701f;
  normals[3].getNormalVector3fMap() << -0.552338f, -0.342020f, -0.760227f;
  normals[4].getNormalVector3fMap() << -0.290381f, -0.342020f, -0.893701f;
  normals[5].getNormalVector3fMap() << 0.145191f, -0.342020f, -0.916697f;
  normals[6].getNormalVector3fMap() << 0.552337f, -0.342020f, -0.760227f;
  normals[7].getNormalVector3fMap() << -0.656282f, -0.342020f, -0.656283f;
  normals[8].getNormalVector3fMap() << 0.656282f, -0.342020f, -0.656283f;
  normals[9].getNormalVector3fMap() << -0.656283f, -0.342020f, -0.656282f;
  normals[10].getNormalVector3fMap() << -0.290381f, -0.342020f, -0.893701f;
  normals[11].getNormalVector3fMap() << 0.145191f, -0.342020f, -0.916697f;
  normals[12].getNormalVector3fMap() << 0.656282f, -0.342020f, -0.656282f;
  normals[13].getNormalVector3fMap() << -0.760228f, -0.342020f, -0.552337f;
  normals[14].getNormalVector3fMap() << 0.760228f, -0.342020f, -0.552337f;
  normals[15].getNormalVector3fMap() << -0.893701f, -0.342020f, -0.290380f;
  normals[16].getNormalVector3fMap() << 0.893701f, -0.342020f, -0.290380f;
  normals[17].getNormalVector3fMap() << -0.624162f, -0.342020f, -0.624162f;
  normals[18].getNormalVector3fMap() << 0.499329f, -0.342020f, -0.687268f;
  normals[19].getNormalVector3fMap() << -0.893701f, -0.342020f, -0.290380f;
  normals[20].getNormalVector3fMap() << 0.893701f, -0.342020f, -0.290380f;
  normals[21].getNormalVector3fMap() << -0.893701f, -0.342020f, -0.290381f;
  normals[22].getNormalVector3fMap() << 0.893701f, -0.342020f, -0.290381f;
  normals[23].getNormalVector3fMap() << -0.939693f, -0.342020f, 0.000000f;
  normals[24].getNormalVector3fMap() << 0.939693f, -0.342020f, 0.000000f;
  normals[25].getNormalVector3fMap() << -0.939693f, -0.342020f, 0.000000f;
  normals[26].getNormalVector3fMap() << 0.939693f, -0.342020f, 0.000000f;
  normals[27].getNormalVector3fMap() << -0.939693f, -0.342020f, 0.000000f;
  normals[28].getNormalVector3fMap() << 0.939693f, -0.342020f, 0.000000f;
  normals[29].getNormalVector3fMap() << -0.939693f, -0.342020f, 0.000000f;
  normals[30].getNormalVector3fMap() << 0.939693f, -0.342020f, 0.000000f;

  // Create a shared cylinder model pointer directly
  SampleConsensusModelConePtr model(
      new SampleConsensusModelCone<PointXYZ, Normal>(cloud.makeShared()));
  model->setInputNormals(normals.makeShared());

  // Create the RANSAC object
  RandomSampleConsensus<PointXYZ> sac(model, 0.03);

  // Algorithm tests
  bool result = sac.computeModel();
  ASSERT_TRUE(result);

  pcl::Indices sample;
  sac.getModel(sample);
  EXPECT_EQ(3, sample.size());

  pcl::Indices inliers;
  sac.getInliers(inliers);
  EXPECT_EQ(31, inliers.size());

  Eigen::VectorXf coeff;
  sac.getModelCoefficients(coeff);
  EXPECT_EQ(7, coeff.size());
  EXPECT_NEAR(0.000000, coeff[0], 1e-2);
  EXPECT_NEAR(0.100000, coeff[1], 1e-2);
  EXPECT_NEAR(0.349066, coeff[6], 1e-2);

  Eigen::VectorXf coeff_refined;
  model->optimizeModelCoefficients(inliers, coeff, coeff_refined);
  EXPECT_EQ(7, coeff_refined.size());
  EXPECT_NEAR(0.349066, coeff_refined[6], 1e-2);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST(SampleConsensusModelCylinder, RANSAC)
{
  srand(0);

  // Use a custom point cloud for these tests until we need something better
  PointCloud<PointXYZ> cloud;
  PointCloud<Normal> normals;
  cloud.resize(20);
  normals.resize(20);

  cloud[0].getVector3fMap() << -0.499902f, 2.199701f, 0.000008f;
  cloud[1].getVector3fMap() << -0.875397f, 2.030177f, 0.050104f;
  cloud[2].getVector3fMap() << -0.995875f, 1.635973f, 0.099846f;
  cloud[3].getVector3fMap() << -0.779523f, 1.285527f, 0.149961f;
  cloud[4].getVector3fMap() << -0.373285f, 1.216488f, 0.199959f;
  cloud[5].getVector3fMap() << -0.052893f, 1.475973f, 0.250101f;
  cloud[6].getVector3fMap() << -0.036558f, 1.887591f, 0.299839f;
  cloud[7].getVector3fMap() << -0.335048f, 2.171994f, 0.350001f;
  cloud[8].getVector3fMap() << -0.745456f, 2.135528f, 0.400072f;
  cloud[9].getVector3fMap() << -0.989282f, 1.803311f, 0.449983f;
  cloud[10].getVector3fMap() << -0.900651f, 1.400701f, 0.500126f;
  cloud[11].getVector3fMap() << -0.539658f, 1.201468f, 0.550079f;
  cloud[12].getVector3fMap() << -0.151875f, 1.340951f, 0.599983f;
  cloud[13].getVector3fMap() << -0.000724f, 1.724373f, 0.649882f;
  cloud[14].getVector3fMap() << -0.188573f, 2.090983f, 0.699854f;
  cloud[15].getVector3fMap() << -0.587925f, 2.192257f, 0.749956f;
  cloud[16].getVector3fMap() << -0.927724f, 1.958846f, 0.800008f;
  cloud[17].getVector3fMap() << -0.976888f, 1.549655f, 0.849970f;
  cloud[18].getVector3fMap() << -0.702003f, 1.242707f, 0.899954f;
  cloud[19].getVector3fMap() << -0.289916f, 1.246296f, 0.950075f;

  normals[0].getNormalVector3fMap() << 0.000098f, 1.000098f, 0.000008f;
  normals[1].getNormalVector3fMap() << -0.750891f, 0.660413f, 0.000104f;
  normals[2].getNormalVector3fMap() << -0.991765f, -0.127949f, -0.000154f;
  normals[3].getNormalVector3fMap() << -0.558918f, -0.829439f, -0.000039f;
  normals[4].getNormalVector3fMap() << 0.253627f, -0.967447f, -0.000041f;
  normals[5].getNormalVector3fMap() << 0.894105f, -0.447965f, 0.000101f;
  normals[6].getNormalVector3fMap() << 0.926852f, 0.375543f, -0.000161f;
  normals[7].getNormalVector3fMap() << 0.329948f, 0.943941f, 0.000001f;
  normals[8].getNormalVector3fMap() << -0.490966f, 0.871203f, 0.000072f;
  normals[9].getNormalVector3fMap() << -0.978507f, 0.206425f, -0.000017f;
  normals[10].getNormalVector3fMap() << -0.801227f, -0.598534f, 0.000126f;
  normals[11].getNormalVector3fMap() << -0.079447f, -0.996697f, 0.000079f;
  normals[12].getNormalVector3fMap() << 0.696154f, -0.717889f, -0.000017f;
  normals[13].getNormalVector3fMap() << 0.998685f, 0.048502f, -0.000118f;
  normals[14].getNormalVector3fMap() << 0.622933f, 0.782133f, -0.000146f;
  normals[15].getNormalVector3fMap() << -0.175948f, 0.984480f, -0.000044f;
  normals[16].getNormalVector3fMap() << -0.855476f, 0.517824f, 0.000008f;
  normals[17].getNormalVector3fMap() << -0.953769f, -0.300571f, -0.000030f;
  normals[18].getNormalVector3fMap() << -0.404035f, -0.914700f, -0.000046f;
  normals[19].getNormalVector3fMap() << 0.420154f, -0.907445f, 0.000075f;

  // Create a shared cylinder model pointer directly
  SampleConsensusModelCylinderPtr model(
      new SampleConsensusModelCylinder<PointXYZ, Normal>(cloud.makeShared()));
  model->setInputNormals(normals.makeShared());

  // Create the RANSAC object
  RandomSampleConsensus<PointXYZ> sac(model, 0.03);

  // Algorithm tests
  bool result = sac.computeModel();
  ASSERT_TRUE(result);

  pcl::Indices sample;
  sac.getModel(sample);
  EXPECT_EQ(2, sample.size());

  pcl::Indices inliers;
  sac.getInliers(inliers);
  EXPECT_EQ(20, inliers.size());

  Eigen::VectorXf coeff;
  sac.getModelCoefficients(coeff);
  EXPECT_EQ(7, coeff.size());
  EXPECT_NEAR(-0.5, coeff[0], 1e-3);
  EXPECT_NEAR(1.7, coeff[1], 1e-3);
  EXPECT_NEAR(0.5, coeff[6], 1e-3);

  Eigen::VectorXf coeff_refined;
  model->optimizeModelCoefficients(inliers, coeff, coeff_refined);
  EXPECT_EQ(7, coeff_refined.size());
  EXPECT_NEAR(0.5, coeff_refined[6], 1e-3);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST(SampleConsensusModelCircle2D, RANSAC)
{
  srand(0);

  // Use a custom point cloud for these tests until we need something better
  PointCloud<PointXYZ> cloud;
  cloud.resize(18);

  cloud[0].getVector3fMap() << 3.587751f, -4.190982f, 0.0f;
  cloud[1].getVector3fMap() << 3.808883f, -4.412265f, 0.0f;
  cloud[2].getVector3fMap() << 3.587525f, -5.809143f, 0.0f;
  cloud[3].getVector3fMap() << 2.999913f, -5.999980f, 0.0f;
  cloud[4].getVector3fMap() << 2.412224f, -5.809090f, 0.0f;
  cloud[5].getVector3fMap() << 2.191080f, -5.587682f, 0.0f;
  cloud[6].getVector3fMap() << 2.048941f, -5.309003f, 0.0f;
  cloud[7].getVector3fMap() << 2.000397f, -4.999944f, 0.0f;
  cloud[8].getVector3fMap() << 2.999953f, -6.000056f, 0.0f;
  cloud[9].getVector3fMap() << 2.691127f, -5.951136f, 0.0f;
  cloud[10].getVector3fMap() << 2.190892f, -5.587838f, 0.0f;
  cloud[11].getVector3fMap() << 2.048874f, -5.309052f, 0.0f;
  cloud[12].getVector3fMap() << 1.999990f, -5.000147f, 0.0f;
  cloud[13].getVector3fMap() << 2.049026f, -4.690918f, 0.0f;
  cloud[14].getVector3fMap() << 2.190956f, -4.412162f, 0.0f;
  cloud[15].getVector3fMap() << 2.412231f, -4.190918f, 0.0f;
  cloud[16].getVector3fMap() << 2.691027f, -4.049060f, 0.0f;
  cloud[17].getVector3fMap() << 2.000000f, -3.000000f, 0.0f;

  // Create a shared 2d circle model pointer directly
  SampleConsensusModelCircle2DPtr model(
      new SampleConsensusModelCircle2D<PointXYZ>(cloud.makeShared()));

  // Create the RANSAC object
  RandomSampleConsensus<PointXYZ> sac(model, 0.03);

  // Algorithm tests
  bool result = sac.computeModel();
  ASSERT_TRUE(result);

  pcl::Indices sample;
  sac.getModel(sample);
  EXPECT_EQ(3, sample.size());

  pcl::Indices inliers;
  sac.getInliers(inliers);
  EXPECT_EQ(17, inliers.size());

  Eigen::VectorXf coeff;
  sac.getModelCoefficients(coeff);
  EXPECT_EQ(3, coeff.size());
  EXPECT_NEAR(3, coeff[0], 1e-3);
  EXPECT_NEAR(-5, coeff[1], 1e-3);
  EXPECT_NEAR(1, coeff[2], 1e-3);

  Eigen::VectorXf coeff_refined;
  model->optimizeModelCoefficients(inliers, coeff, coeff_refined);
  EXPECT_EQ(3, coeff_refined.size());
  EXPECT_NEAR(3, coeff_refined[0], 1e-3);
  EXPECT_NEAR(-5, coeff_refined[1], 1e-3);
  EXPECT_NEAR(1, coeff_refined[2], 1e-3);
}

///////////////////////////////////////////////////////////////////////////////

TEST(SampleConsensusModelCircle2D, SampleValidationPointsValid)
{
  PointCloud<PointXYZ> cloud;
  cloud.resize(3);

  cloud[0].getVector3fMap() << 1.0f, 2.0f, 0.0f;
  cloud[1].getVector3fMap() << 0.0f, 1.0f, 0.0f;
  cloud[2].getVector3fMap() << 1.5f, 0.0f, 0.0f;

  // Create a shared line model pointer directly
  SampleConsensusModelCircle2DPtr model(
      new SampleConsensusModelCircle2D<PointXYZ>(cloud.makeShared()));

  // Algorithm tests
  pcl::Indices samples;
  int iterations = 0;
  model->getSamples(iterations, samples);
  EXPECT_EQ(samples.size(), 3);

  Eigen::VectorXf modelCoefficients;
  EXPECT_TRUE(model->computeModelCoefficients(samples, modelCoefficients));

  EXPECT_NEAR(modelCoefficients[0], 1.05f, 1e-3);             // center x
  EXPECT_NEAR(modelCoefficients[1], 0.95f, 1e-3);             // center y
  EXPECT_NEAR(modelCoefficients[2], std::sqrt(1.105f), 1e-3); // radius
}

using PointVector = std::vector<PointXYZ>;
class SampleConsensusModelCircle2DCollinearTest
: public testing::TestWithParam<PointVector> {
protected:
  void
  SetUp() override
  {
    pointCloud_ = make_shared<PointCloud<PointXYZ>>();
    for (const auto& point : GetParam()) {
      pointCloud_->emplace_back(point);
    }
  }

  PointCloud<PointXYZ>::Ptr pointCloud_ = nullptr;
};

// Parametric tests clearly show the input for which they failed and provide
// clearer feedback if something is changed in the future.
TEST_P(SampleConsensusModelCircle2DCollinearTest, SampleValidationPointsInvalid)
{
  ASSERT_NE(pointCloud_, nullptr);

  SampleConsensusModelCircle2DPtr model(
      new SampleConsensusModelCircle2D<PointXYZ>(pointCloud_));
  ASSERT_GE(model->getInputCloud()->size(), model->getSampleSize());

  // Algorithm tests
  // (Maybe) TODO: try to implement the "cheat point" trick from
  //               test_sample_consensus_line_models.cpp and friends here, too.
  // pcl::Indices samples;
  // int iterations = 0;
  // model->getSamples(iterations, samples);
  // EXPECT_EQ (samples.size(), 0);

  // Explicitly enforce sample order so they can act as designed
  pcl::Indices forcedSamples = {0, 1, 2};
  Eigen::VectorXf modelCoefficients;
  EXPECT_FALSE(model->computeModelCoefficients(forcedSamples, modelCoefficients));
}

INSTANTIATE_TEST_SUITE_P(
    CollinearInputs,
    SampleConsensusModelCircle2DCollinearTest,
    testing::Values( // Throw in some negative coordinates and "asymmetric" points to
                     // cover more cases
        PointVector{{0.0f, 0.0f, 0.0f},
                    {1.0f, 0.0f, 0.0f},
                    {2.0f, 0.0f, 0.0f}}, // collinear along x-axis
        PointVector{{-1.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {2.0f, 0.0f, 0.0f}},
        PointVector{{0.0f, -1.0f, 0.0f},
                    {0.0f, 1.0f, 0.0f},
                    {0.0f, 2.0f, 0.0f}}, // collinear along y-axis
        PointVector{{0.0f, -1.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 2.0f, 0.0f}},
        PointVector{{0.0f, 0.0f, 0.0f},
                    {1.0f, 1.0f, 0.0f},
                    {2.0f, 2.0f, 0.0f}}, // collinear along diagonal
        PointVector{{0.0f, 0.0f, 0.0f}, {-1.0f, -1.0f, 0.0f}, {-2.0f, -2.0f, 0.0f}},
        PointVector{{-3.0f, -6.5f, 0.0f},
                    {-2.0f, -0.5f, 0.0f},
                    {-1.5f, 2.5f, 0.0f}}, // other collinear input
        PointVector{{2.0f, 2.0f, 0.0f},
                    {0.0f, 0.0f, 0.0f},
                    {0.0f, 0.0f, 0.0f}}, // two points equal
        PointVector{{0.0f, 0.0f, 0.0f}, {2.0f, 2.0f, 0.0f}, {0.0f, 0.0f, 0.0f}},
        PointVector{{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, {2.0f, 2.0f, 0.0f}},
        PointVector{{1.0f, 1.0f, 0.0f}, {1.0f, 1.0f, 0.0f}, {1.0f, 1.0f, 0.0f}}
        // all points equal
        ));

//////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
class SampleConsensusModelCircle2DTest : private SampleConsensusModelCircle2D<PointT> {
public:
  using SampleConsensusModelCircle2D<PointT>::SampleConsensusModelCircle2D;
  using SampleConsensusModelCircle2D<PointT>::countWithinDistanceStandard;
#if defined(__SSE__) && defined(__SSE2__) && defined(__SSE4_1__)
  using SampleConsensusModelCircle2D<PointT>::countWithinDistanceSSE;
#endif
#if defined(__AVX__) && defined(__AVX2__)
  using SampleConsensusModelCircle2D<PointT>::countWithinDistanceAVX;
#endif
};

TEST(SampleConsensusModelCircle2D,
     SIMD_countWithinDistance) // Test if all countWithinDistance implementations return
                               // the same value
{
  const auto seed = static_cast<unsigned>(std::time(nullptr));
  srand(seed);
  for (size_t i = 0; i < 100; i++) // Run as often as you like
  {
    // Generate a cloud with 1000 random points
    PointCloud<PointXYZ> cloud;
    pcl::Indices indices;
    cloud.resize(1000);
    for (std::size_t idx = 0; idx < cloud.size(); ++idx) {
      cloud[idx].x = 2.0 * static_cast<float>(rand()) / RAND_MAX - 1.0;
      cloud[idx].y = 2.0 * static_cast<float>(rand()) / RAND_MAX - 1.0;
      cloud[idx].z = 2.0 * static_cast<float>(rand()) / RAND_MAX - 1.0;
      if (rand() % 2 == 0) {
        indices.push_back(static_cast<int>(idx));
      }
    }
    SampleConsensusModelCircle2DTest<PointXYZ> model(cloud.makeShared(), indices, true);

    // Generate random circle model parameters
    Eigen::VectorXf model_coefficients(3);
    model_coefficients << 2.0 * static_cast<float>(rand()) / RAND_MAX - 1.0,
        2.0 * static_cast<float>(rand()) / RAND_MAX - 1.0,
        0.1 * static_cast<float>(rand()) / RAND_MAX; // center and radius

    const double threshold =
        0.1 * static_cast<double>(rand()) / RAND_MAX; // threshold in [0; 0.1]

    // The number of inliers is usually somewhere between 0 and 20
    const auto res_standard =
        model.countWithinDistanceStandard(model_coefficients, threshold); // Standard
    PCL_DEBUG("seed=%lu, i=%lu, model=(%f, %f, %f), threshold=%f, res_standard=%lu\n",
              seed,
              i,
              model_coefficients(0),
              model_coefficients(1),
              model_coefficients(2),
              threshold,
              res_standard);
#if defined(__SSE__) && defined(__SSE2__) && defined(__SSE4_1__)
    const auto res_sse =
        model.countWithinDistanceSSE(model_coefficients, threshold); // SSE
    ASSERT_EQ(res_standard, res_sse);
#endif
#if defined(__AVX__) && defined(__AVX2__)
    const auto res_avx =
        model.countWithinDistanceAVX(model_coefficients, threshold); // AVX
    ASSERT_EQ(res_standard, res_avx);
#endif
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST(SampleConsensusModelCircle3D, RANSAC)
{
  srand(0);

  // Use a custom point cloud for these tests until we need something better
  PointCloud<PointXYZ> cloud;
  cloud.resize(20);

  cloud[0].getVector3fMap() << 1.00000000f, 5.0000000f, -2.9000001f;
  cloud[1].getVector3fMap() << 1.03420200f, 5.0000000f, -2.9060307f;
  cloud[2].getVector3fMap() << 1.06427870f, 5.0000000f, -2.9233956f;
  cloud[3].getVector3fMap() << 1.08660260f, 5.0000000f, -2.9500000f;
  cloud[4].getVector3fMap() << 1.09848080f, 5.0000000f, -2.9826353f;
  cloud[5].getVector3fMap() << 1.09848080f, 5.0000000f, -3.0173647f;
  cloud[6].getVector3fMap() << 1.08660260f, 5.0000000f, -3.0500000f;
  cloud[7].getVector3fMap() << 1.06427870f, 5.0000000f, -3.0766044f;
  cloud[8].getVector3fMap() << 1.03420200f, 5.0000000f, -3.0939693f;
  cloud[9].getVector3fMap() << 1.00000000f, 5.0000000f, -3.0999999f;
  cloud[10].getVector3fMap() << 0.96579796f, 5.0000000f, -3.0939693f;
  cloud[11].getVector3fMap() << 0.93572122f, 5.0000000f, -3.0766044f;
  cloud[12].getVector3fMap() << 0.91339743f, 5.0000000f, -3.0500000f;
  cloud[13].getVector3fMap() << 0.90151924f, 5.0000000f, -3.0173647f;
  cloud[14].getVector3fMap() << 0.90151924f, 5.0000000f, -2.9826353f;
  cloud[15].getVector3fMap() << 0.91339743f, 5.0000000f, -2.9500000f;
  cloud[16].getVector3fMap() << 0.93572122f, 5.0000000f, -2.9233956f;
  cloud[17].getVector3fMap() << 0.96579796f, 5.0000000f, -2.9060307f;
  cloud[18].getVector3fMap() << 0.85000002f, 4.8499999f, -3.1500001f;
  cloud[19].getVector3fMap() << 1.15000000f, 5.1500001f, -2.8499999f;

  // Create a shared 3d circle model pointer directly
  SampleConsensusModelCircle3DPtr model(
      new SampleConsensusModelCircle3D<PointXYZ>(cloud.makeShared()));

  // Create the RANSAC object
  RandomSampleConsensus<PointXYZ> sac(model, 0.03);

  // Algorithm tests
  bool result = sac.computeModel();
  ASSERT_TRUE(result);

  pcl::Indices sample;
  sac.getModel(sample);
  EXPECT_EQ(3, sample.size());

  pcl::Indices inliers;
  sac.getInliers(inliers);
  EXPECT_EQ(18, inliers.size());

  Eigen::VectorXf coeff;
  sac.getModelCoefficients(coeff);
  EXPECT_EQ(7, coeff.size());
  EXPECT_NEAR(1.0, coeff[0], 1e-3);
  EXPECT_NEAR(5.0, coeff[1], 1e-3);
  EXPECT_NEAR(-3.0, coeff[2], 1e-3);
  EXPECT_NEAR(0.1, coeff[3], 1e-3);
  EXPECT_NEAR(0.0, coeff[4], 1e-3);
  // Use abs in y component because both variants are valid normal vectors
  EXPECT_NEAR(1.0, std::abs(coeff[5]), 1e-3);
  EXPECT_NEAR(0.0, coeff[6], 1e-3);

  Eigen::VectorXf coeff_refined;
  model->optimizeModelCoefficients(inliers, coeff, coeff_refined);
  EXPECT_EQ(7, coeff_refined.size());
  EXPECT_NEAR(1.0, coeff_refined[0], 1e-3);
  EXPECT_NEAR(5.0, coeff_refined[1], 1e-3);
  EXPECT_NEAR(-3.0, coeff_refined[2], 1e-3);
  EXPECT_NEAR(0.1, coeff_refined[3], 1e-3);
  EXPECT_NEAR(0.0, coeff_refined[4], 1e-3);
  EXPECT_NEAR(1.0, std::abs(coeff_refined[5]), 1e-3);
  EXPECT_NEAR(0.0, coeff_refined[6], 1e-3);
}

TEST(SampleConsensusModelSphere, projectPoints)
{
  Eigen::VectorXf model_coefficients(4);
  model_coefficients << -0.32, -0.89, 0.37, 0.12; // center and radius

  pcl::PointCloud<pcl::PointXYZ>::Ptr input(new pcl::PointCloud<pcl::PointXYZ>);
  input->emplace_back(-0.259754, -0.950873, 0.318377);  // inlier, dist from center=0.10
  input->emplace_back(0.595892, 0.455094, 0.025545);    // outlier, not projected
  input->emplace_back(-0.221871, -0.973718, 0.353817);  // inlier, dist from center=0.13
  input->emplace_back(-0.332269, -0.848851, 0.437499);  // inlier, dist from center=0.08
  input->emplace_back(-0.242308, -0.561036, -0.365535); // outlier, not projected
  input->emplace_back(-0.327668, -0.800009, 0.290988);  // inlier, dist from center=0.12
  input->emplace_back(-0.173948, -0.883831, 0.403625);  // inlier, dist from center=0.15
  input->emplace_back(-0.033891, 0.624537, -0.606994);  // outlier, not projected

  pcl::SampleConsensusModelSphere<pcl::PointXYZ> model(input);
  pcl::Indices inliers = {0, 2, 3, 5, 6};

  pcl::PointCloud<pcl::PointXYZ> projected_truth;
  projected_truth.emplace_back(-0.247705, -0.963048, 0.308053);
  projected_truth.emplace_back(-0.229419, -0.967278, 0.355062);
  projected_truth.emplace_back(-0.338404, -0.828276, 0.471249);
  projected_truth.emplace_back(-0.327668, -0.800009, 0.290988);
  projected_truth.emplace_back(-0.203158, -0.885065, 0.396900);

  pcl::PointCloud<pcl::PointXYZ> projected_points;
  model.projectPoints(inliers, model_coefficients, projected_points, false);
  EXPECT_EQ(projected_points.size(), 5);
  for (int i = 0; i < 5; ++i)
    EXPECT_XYZ_NEAR(projected_points[i], projected_truth[i], 1e-5);

  pcl::PointCloud<pcl::PointXYZ> projected_points_all;
  model.projectPoints(inliers, model_coefficients, projected_points_all, true);
  EXPECT_EQ(projected_points_all.size(), 8);
  EXPECT_XYZ_NEAR(projected_points_all[0], projected_truth[0], 1e-5);
  EXPECT_XYZ_NEAR(projected_points_all[1], (*input)[1], 1e-5);
  EXPECT_XYZ_NEAR(projected_points_all[2], projected_truth[1], 1e-5);
  EXPECT_XYZ_NEAR(projected_points_all[3], projected_truth[2], 1e-5);
  EXPECT_XYZ_NEAR(projected_points_all[4], (*input)[4], 1e-5);
  EXPECT_XYZ_NEAR(projected_points_all[5], projected_truth[3], 1e-5);
  EXPECT_XYZ_NEAR(projected_points_all[6], projected_truth[4], 1e-5);
  EXPECT_XYZ_NEAR(projected_points_all[7], (*input)[7], 1e-5);
}

TEST(SampleConsensusModelCylinder, projectPoints)
{
  Eigen::VectorXf model_coefficients(7);
  model_coefficients << -0.59, 0.85, -0.80, 0.22, -0.70, 0.68, 0.18;

  pcl::PointCloud<pcl::PointXYZ>::Ptr input(new pcl::PointCloud<pcl::PointXYZ>);
  input->emplace_back(-0.616358, 0.713315, -0.885120);  // inlier, dist from axis=0.16
  input->emplace_back(0.595892, 0.455094, 0.025545);    // outlier, not projected
  input->emplace_back(-0.952749, 1.450040, -1.305640);  // inlier, dist from axis=0.19
  input->emplace_back(-0.644947, 1.421240, -1.421170);  // inlier, dist from axis=0.14
  input->emplace_back(-0.242308, -0.561036, -0.365535); // outlier, not projected
  input->emplace_back(-0.502111, 0.694671, -0.878344);  // inlier, dist from axis=0.18
  input->emplace_back(-0.664129, 0.557583, -0.750060);  // inlier, dist from axis=0.21
  input->emplace_back(-0.033891, 0.624537, -0.606994);  // outlier, not projected

  pcl::SampleConsensusModelCylinder<pcl::PointXYZ, pcl::Normal> model(input);
  // not necessary to set normals for model here because projectPoints does not use them
  pcl::Indices inliers = {0, 2, 3, 5, 6};

  pcl::PointCloud<pcl::PointXYZ> projected_truth;
  projected_truth.emplace_back(-0.620532, 0.699027, -0.898478);
  projected_truth.emplace_back(-0.943418, 1.449510, -1.309200);
  projected_truth.emplace_back(-0.608243, 1.417710, -1.436680);
  projected_truth.emplace_back(-0.502111, 0.694671, -0.878344);
  projected_truth.emplace_back(-0.646557, 0.577140, -0.735612);

  pcl::PointCloud<pcl::PointXYZ> projected_points;
  model.projectPoints(inliers, model_coefficients, projected_points, false);
  EXPECT_EQ(projected_points.size(), 5);
  for (int i = 0; i < 5; ++i)
    EXPECT_XYZ_NEAR(projected_points[i], projected_truth[i], 1e-5);

  pcl::PointCloud<pcl::PointXYZ> projected_points_all;
  model.projectPoints(inliers, model_coefficients, projected_points_all, true);
  EXPECT_EQ(projected_points_all.size(), 8);
  EXPECT_XYZ_NEAR(projected_points_all[0], projected_truth[0], 1e-5);
  EXPECT_XYZ_NEAR(projected_points_all[1], (*input)[1], 1e-5);
  EXPECT_XYZ_NEAR(projected_points_all[2], projected_truth[1], 1e-5);
  EXPECT_XYZ_NEAR(projected_points_all[3], projected_truth[2], 1e-5);
  EXPECT_XYZ_NEAR(projected_points_all[4], (*input)[4], 1e-5);
  EXPECT_XYZ_NEAR(projected_points_all[5], projected_truth[3], 1e-5);
  EXPECT_XYZ_NEAR(projected_points_all[6], projected_truth[4], 1e-5);
  EXPECT_XYZ_NEAR(projected_points_all[7], (*input)[7], 1e-5);
}

TEST(SampleConsensusModelEllipse3D, RANSAC)
{
  srand(0);

  // Using a custom point cloud on a tilted plane
  PointCloud<PointXYZ> cloud;
  cloud.resize(22);

  cloud[0].getVector3fMap() << 1.000000, 5.000000, 3.000000;
  cloud[1].getVector3fMap() << 0.690983, 5.000000, 2.902110;
  cloud[2].getVector3fMap() << 0.412215, 5.000000, 2.618030;
  cloud[3].getVector3fMap() << 0.190983, 5.000000, 2.175570;
  cloud[4].getVector3fMap() << 0.048944, 5.000000, 1.618030;
  cloud[5].getVector3fMap() << 0.000000, 5.000000, 1.000000;
  cloud[6].getVector3fMap() << 0.048944, 5.000000, 0.381966;
  cloud[7].getVector3fMap() << 0.190983, 5.000000, -0.175571;
  cloud[8].getVector3fMap() << 0.412215, 5.000000, -0.618034;
  cloud[9].getVector3fMap() << 0.690983, 5.000000, -0.902113;
  cloud[10].getVector3fMap() << 1.000000, 5.000000, -1.000000;
  cloud[11].getVector3fMap() << 1.309020, 5.000000, -0.902113;
  cloud[12].getVector3fMap() << 1.587790, 5.000000, -0.618034;
  cloud[13].getVector3fMap() << 1.809020, 5.000000, -0.175571;
  cloud[14].getVector3fMap() << 1.951060, 5.000000, 0.381966;
  cloud[15].getVector3fMap() << 2.000000, 5.000000, 1.000000;
  cloud[16].getVector3fMap() << 1.951060, 5.000000, 1.618030;
  cloud[17].getVector3fMap() << 1.809020, 5.000000, 2.175570;
  cloud[18].getVector3fMap() << 1.587790, 5.000000, 2.618030;
  cloud[19].getVector3fMap() << 1.309020, 5.000000, 2.902110;

  cloud[20].getVector3fMap() << 0.85000002f, 4.8499999f, -3.1500001f;
  cloud[21].getVector3fMap() << 1.15000000f, 5.1500001f, -2.8499999f;

  // Create a shared 3d ellipse model pointer directly
  SampleConsensusModelEllipse3DPtr model(
      new SampleConsensusModelEllipse3D<PointXYZ>(cloud.makeShared()));

  // Create the RANSAC object
  RandomSampleConsensus<PointXYZ> sac(model, 0.0011);

  // Algorithm tests
  bool result = sac.computeModel();
  ASSERT_TRUE(result);

  pcl::Indices sample;
  sac.getModel(sample);
  EXPECT_EQ(6, sample.size());

  pcl::Indices inliers;
  sac.getInliers(inliers);
  EXPECT_EQ(20, inliers.size());

  Eigen::VectorXf coeff;
  sac.getModelCoefficients(coeff);
  EXPECT_EQ(11, coeff.size());
  EXPECT_NEAR(1.0, coeff[0], 1e-3);
  EXPECT_NEAR(5.0, coeff[1], 1e-3);
  EXPECT_NEAR(1.0, coeff[2], 1e-3);

  EXPECT_NEAR(2.0, coeff[3], 1e-3);
  EXPECT_NEAR(1.0, coeff[4], 1e-3);

  EXPECT_NEAR(0.0, coeff[5], 1e-3);
  // Use abs in y component because both variants are valid normal vectors
  EXPECT_NEAR(1.0, std::abs(coeff[6]), 1e-3);
  EXPECT_NEAR(0.0, coeff[7], 1e-3);

  EXPECT_NEAR(0.0, coeff[8], 1e-3);
  EXPECT_NEAR(0.0, coeff[9], 1e-3);
  // Use abs in z component because both variants are valid local vectors
  EXPECT_NEAR(1.0, std::abs(coeff[10]), 1e-3);

  Eigen::VectorXf coeff_refined;
  model->optimizeModelCoefficients(inliers, coeff, coeff_refined);
  EXPECT_EQ(11, coeff_refined.size());
  EXPECT_NEAR(1.0, coeff_refined[0], 1e-3);
  EXPECT_NEAR(5.0, coeff_refined[1], 1e-3);
  EXPECT_NEAR(1.0, coeff_refined[2], 1e-3);

  EXPECT_NEAR(2.0, coeff_refined[3], 1e-3);
  EXPECT_NEAR(1.0, coeff_refined[4], 1e-3);

  EXPECT_NEAR(0.0, coeff_refined[5], 1e-3);
  // Use abs in y component because both variants are valid normal vectors
  EXPECT_NEAR(1.0, std::abs(coeff_refined[6]), 1e-3);
  EXPECT_NEAR(0.0, coeff_refined[7], 1e-3);

  EXPECT_NEAR(0.0, coeff_refined[8], 1e-3);
  EXPECT_NEAR(0.0, coeff_refined[9], 1e-3);
  // Use abs in z component because both variants are valid local vectors
  EXPECT_NEAR(1.0, std::abs(coeff_refined[10]), 1e-3);
}

// Heavy oclusion, all points on a 30 degree segment on the major radius
//  and 90 degrees on the minor
TEST(SampleConsensusModelTorusOclusion, RANSAC)
{

  srand(0);

  PointCloud<PointXYZ> cloud;
  PointCloud<Normal> normals;

  cloud.resize(50);
  normals.resize(50);

  cloud[0].getVector3fMap() << 1.4386461277601734, -1.0569588316537601,
      3.6109405500068648;
  cloud[1].getVector3fMap() << 1.5892272151771987, -1.0107131751656608,
      3.7524801792294786;
  cloud[2].getVector3fMap() << 0.779450850990528, -1.1095381992814901,
      2.1362474566704086;
  cloud[3].getVector3fMap() << 2.2332764042301116, -1.0712898227325813,
      4.104289150491388;
  cloud[4].getVector3fMap() << 1.4625240977021328, -1.0262438455563425,
      3.6408698702774327;
  cloud[5].getVector3fMap() << 1.107288574597542, -1.038213986002474,
      3.2111695308737334;
  cloud[6].getVector3fMap() << 1.634136176426644, -1.0586849054858922,
      3.7791937450863844;
  cloud[7].getVector3fMap() << 2.8081039281494284, -1.124052124218725,
      4.208730485774282;
  cloud[8].getVector3fMap() << 2.7325382847100004, -1.0968720291167913,
      4.214374570675481;
  cloud[9].getVector3fMap() << 1.1897810394404515, -1.0861469920822655,
      3.3103205876091675;
  cloud[10].getVector3fMap() << 0.8242772124713484, -1.0935505936537508,
      2.4973185149793924;
  cloud[11].getVector3fMap() << 2.9589166075430366, -1.0656763334810868,
      4.240842449620676;
  cloud[12].getVector3fMap() << 1.7930324882302902, -1.0629548347911097,
      3.8893227292858175;
  cloud[13].getVector3fMap() << 0.7810401372209808, -1.0705732580035723,
      2.305065186549668;
  cloud[14].getVector3fMap() << 0.9062603270739178, -1.0815748767074063,
      2.785726514647834;
  cloud[15].getVector3fMap() << 1.3832157146170436, -1.0790593653653633,
      3.546265907749163;
  cloud[16].getVector3fMap() << 2.040614544849421, -1.0918678466867353,
      4.015855816881193;
  cloud[17].getVector3fMap() << 2.274098663746168, -1.0273778393320356,
      4.128098505334945;
  cloud[18].getVector3fMap() << 1.2518457008499417, -1.0912889870169762,
      3.38890936771287;
  cloud[19].getVector3fMap() << 0.8773148573186607, -1.026298817514791,
      2.7419351335271855;
  cloud[20].getVector3fMap() << 2.460972277763233, -1.0874470683716413,
      4.168209147029958;
  cloud[21].getVector3fMap() << 2.326091552875379, -1.0983984335719184,
      4.125546904328003;
  cloud[22].getVector3fMap() << 2.0996991277329786, -1.0707210059905774,
      4.050880542671691;
  cloud[23].getVector3fMap() << 0.95831333743683, -1.061687690479444,
      2.9269785200505813;
  cloud[24].getVector3fMap() << 2.0588703194976024, -1.0025516869353353,
      4.043701622831673;
  normals[0].getNormalVector3fMap() << -0.6776646502188018, -0.2278353266150415,
      0.6991864456566977;
  normals[1].getNormalVector3fMap() << -0.6264981002776666, -0.04285270066264479,
      0.7782440339600377;
  normals[2].getNormalVector3fMap() << -0.8972132050327152, -0.4381527971259608,
      0.05505080458648829;
  normals[3].getNormalVector3fMap() << -0.32813125795357256, -0.2851592909303272,
      0.9005631884270638;
  normals[4].getNormalVector3fMap() << -0.6799645795137096, -0.10497538222537117,
      0.7256916285402369;
  normals[5].getNormalVector3fMap() << -0.8324065340358171, -0.15285594400989705,
      0.5326672718267207;
  normals[6].getNormalVector3fMap() << -0.5919262688649901, -0.2347396219435707,
      0.7710516209161101;
  normals[7].getNormalVector3fMap() << -0.07514704519204393, -0.4962084968749015,
      0.8649451134193752;
  normals[8].getNormalVector3fMap() << -0.11054456443635059, -0.387488116467167,
      0.9152228465626854;
  normals[9].getNormalVector3fMap() << -0.7604417087668234, -0.34458796832906313,
      0.5504430394242827;
  normals[10].getNormalVector3fMap() << -0.9040312337559508, -0.3742023746150035,
      0.20664005232816324;
  normals[11].getNormalVector3fMap() << -0.0176869745485768, -0.2627053339243488,
      0.964713987904713;
  normals[12].getNormalVector3fMap() << -0.5210086952913671, -0.25181933916444066,
      0.8155592926658195;
  normals[13].getNormalVector3fMap() << -0.950388588906301, -0.2822930320142894,
      0.13066053020277188;
  normals[14].getNormalVector3fMap() << -0.8850007317473024, -0.32629950682962533,
      0.3321179559275326;
  normals[15].getNormalVector3fMap() << -0.6856032449538655, -0.31623746146145426,
      0.65569967094482;
  normals[16].getNormalVector3fMap() << -0.3996678191380136, -0.36747138674694285,
      0.8397799796778576;
  normals[17].getNormalVector3fMap() << -0.3208968621888316, -0.10951135732814456,
      0.9407616416784378;
  normals[18].getNormalVector3fMap() << -0.728898292732006, -0.36515594806790613,
      0.5791100175640165;
  normals[19].getNormalVector3fMap() << -0.9387598943114375, -0.1051952700591645,
      0.3281216481574453;
  normals[20].getNormalVector3fMap() << -0.22602052518599647, -0.34978827348656716,
      0.9091550395427244;
  normals[21].getNormalVector3fMap() << -0.27783106746442193, -0.3935937342876755,
      0.8762955382067529;
  normals[22].getNormalVector3fMap() << -0.38553965278262686, -0.2828840239623112,
      0.878257254521215;
  normals[23].getNormalVector3fMap() << -0.8823896524250601, -0.24675076191777665,
      0.4006277109564169;
  normals[24].getNormalVector3fMap() << -0.4182604905252856, -0.010206747741342384,
      0.908269775103241;

  // 50% noise uniform [-2,2]
  //
  cloud[25].getVector3fMap() << 0.25023241635877147, 0.27654549396527894,
      1.07955881369387;
  cloud[26].getVector3fMap() << 1.5449856383148206, -0.46768009897289264,
      -2.062172100500517;
  cloud[27].getVector3fMap() << 2.068709384697231, -0.8995244010670893,
      0.4472750119304405;
  cloud[28].getVector3fMap() << -1.9703101501142217, 1.1677926799358453,
      -1.0951161775500093;
  cloud[29].getVector3fMap() << 1.5128012164196942, -0.3784790741317119,
      1.9953141538660382;
  cloud[30].getVector3fMap() << -1.7035274240520712, -0.040343373432154106,
      -0.13506114362465782;
  cloud[31].getVector3fMap() << 1.390301434734198, -1.0836155740357354,
      1.3817400889837255;
  cloud[32].getVector3fMap() << 0.6973526735174085, 1.4609265623041212,
      0.3991283042562106;
  cloud[33].getVector3fMap() << 0.4585644490692351, 1.8056826118986748,
      1.1908087822224616;
  cloud[34].getVector3fMap() << -1.899161354377058, -1.2250806902713103,
      1.5135509588271026;
  cloud[35].getVector3fMap() << 0.05728241071603746, -1.3140082682155136,
      -1.6804780212669348;
  cloud[36].getVector3fMap() << -0.5371089158049953, -0.02542717526439331,
      -0.6188539490393421;
  cloud[37].getVector3fMap() << -0.21842672967261145, 0.6528285340670843,
      1.937369474575887;
  cloud[38].getVector3fMap() << 1.6906916394191258, 1.6029527944840072,
      1.3312465637845015;
  cloud[39].getVector3fMap() << 0.3871457304584722, -0.7014470556575774,
      -1.3686189094260588;
  cloud[40].getVector3fMap() << 1.1287360826333366, -1.8859435547052814,
      -0.1392786225318703;
  cloud[41].getVector3fMap() << -0.8284092960915028, 1.0112260700590863,
      -1.1937340633604672;
  cloud[42].getVector3fMap() << 1.8440270354564277, -0.3703200026464992,
      -1.5917391524525757;
  cloud[43].getVector3fMap() << 0.02671922592530418, 1.7827062803768543,
      0.22852714632858673;
  cloud[44].getVector3fMap() << -1.5132468082647963, -1.3357890987499987,
      -1.158617245205414;
  cloud[45].getVector3fMap() << -1.1450583549521511, 1.45432498632732,
      -2.095300144813141;
  cloud[46].getVector3fMap() << 0.18809078359436793, -1.6008222007566066,
      -1.9699784955663424;
  cloud[47].getVector3fMap() << -1.1753993948548627, 1.5857927603987902,
      0.14497327864750886;
  cloud[48].getVector3fMap() << 1.121788740853686, -0.27095183911320286,
      -0.12199102154089814;
  cloud[49].getVector3fMap() << 0.768999145889063, -2.0309651709863434,
      0.7930530394403963;
  normals[25].getNormalVector3fMap() << -0.5835940349115277, 0.1757014210775822,
      0.7928095692201251;
  normals[26].getNormalVector3fMap() << -0.6960838602866861, -0.42094642891496487,
      -0.5816110069729798;
  normals[27].getNormalVector3fMap() << 0.255914777841287, 0.6279839361250196,
      -0.7349447614966528;
  normals[28].getNormalVector3fMap() << -0.6075736135140413, 0.1336509980609126,
      -0.7829378742140479;
  normals[29].getNormalVector3fMap() << -0.4983181083004855, 0.6669454154651717,
      -0.5539520518328415;
  normals[30].getNormalVector3fMap() << -0.7745671302471588, 0.5084406300820161,
      -0.37620989676307437;
  normals[31].getNormalVector3fMap() << -0.424778132583581, -0.3243720781494619,
      0.8451900928168792;
  normals[32].getNormalVector3fMap() << -0.5821055941507861, 0.35171580987235973,
      0.73310917764286;
  normals[33].getNormalVector3fMap() << 0.8396655225180351, -0.48303927894460474,
      0.2482637011147448;
  normals[34].getNormalVector3fMap() << 0.256742174797301, 0.7352345686595317,
      0.6273066114177216;
  normals[35].getNormalVector3fMap() << -0.0652239383938407, -0.5360244339035914,
      0.8416790624214975;
  normals[36].getNormalVector3fMap() << 0.6702382164209467, -0.3031905309377628,
      0.6773892789220579;
  normals[37].getNormalVector3fMap() << -0.6040272704362459, -0.10302003040831528,
      -0.7902771222197995;
  normals[38].getNormalVector3fMap() << 0.9983521281387145, 0.041967677271189614,
      -0.03913747954788317;
  normals[39].getNormalVector3fMap() << -0.573664090993926, 0.46793032429526715,
      0.6722728034875713;
  normals[40].getNormalVector3fMap() << -0.5945467180061245, -0.48897233716525434,
      -0.6382948014791401;
  normals[41].getNormalVector3fMap() << 0.11334045761805764, -0.6164053590067436,
      0.7792293462483921;
  normals[42].getNormalVector3fMap() << -0.766256491311007, 0.13240541094009678,
      -0.6287446196012567;
  normals[43].getNormalVector3fMap() << 0.43564165550696804, 0.7816025130800787,
      0.4464458080596722;
  normals[44].getNormalVector3fMap() << 0.7597220695940338, -0.5120511261307517,
      -0.4007817625591101;
  normals[45].getNormalVector3fMap() << -0.6597147170804349, -0.27171235425320656,
      -0.7006774497681952;
  normals[46].getNormalVector3fMap() << -0.14344953607996272, 0.06349058786868034,
      -0.9876189426345229;
  normals[47].getNormalVector3fMap() << 0.2696193746529791, 0.8928064202811087,
      -0.36083526534496174;
  normals[48].getNormalVector3fMap() << 0.5473019047514905, 0.29388155846326774,
      -0.7836416621457739;
  normals[49].getNormalVector3fMap() << 0.053697689135186716, 0.05924709269452209,
      -0.9967980438327452;

  SampleConsensusModelTorus<PointXYZ, Normal>::Ptr model(
      new SampleConsensusModelTorus<PointXYZ, Normal>(cloud.makeShared()));

  model->setInputNormals(normals.makeShared());

  // Create the RANSAC object
  // Small threshold to filter out the numerous outliers
  RandomSampleConsensus<PointXYZ> sac(model, 0.001);

  // Algorithm tests
  bool result = sac.computeModel();
  ASSERT_TRUE(result);

  pcl::Indices sample;
  sac.getModel(sample);
  EXPECT_EQ(4, sample.size());
  pcl::Indices inliers;
  sac.getInliers(inliers);
  EXPECT_EQ(25, inliers.size());

  Eigen::VectorXf coeff;
  sac.getModelCoefficients(coeff);

  EXPECT_EQ(8, coeff.size());

  EXPECT_NEAR(coeff[0], 2, 1e-2);
  EXPECT_NEAR(coeff[1], 0.25, 1e-2);
  EXPECT_NEAR(coeff[2], 3, 1e-2);
  EXPECT_NEAR(coeff[3], -1, 1e-2);
  EXPECT_NEAR(coeff[4], 2, 1e-2);
  EXPECT_NEAR(coeff[5], 0.0, 1e-2);
  EXPECT_NEAR(std::abs(coeff[6]), 1.0, 1e-2);
  EXPECT_NEAR(coeff[7], 0.0, 1e-2);
  Eigen::VectorXf coeff_refined;
  model->optimizeModelCoefficients(inliers, coeff, coeff_refined);
  EXPECT_EQ(8, coeff.size());

  EXPECT_NEAR(coeff[0], 2, 1e-2);
  EXPECT_NEAR(coeff[1], 0.25, 1e-2);
  EXPECT_NEAR(coeff[2], 3, 1e-2);
  EXPECT_NEAR(coeff[3], -1, 1e-2);
  EXPECT_NEAR(coeff[4], 2, 1e-2);
  EXPECT_NEAR(coeff[5], 0.0, 1e-2);
  EXPECT_NEAR(std::abs(coeff[6]), 1.0, 1e-2);
  EXPECT_NEAR(coeff[7], 2.220446049250313e-16, 1e-2);
}

// A horn shaped torus
TEST(SampleConsensusModelTorusHorn, RANSAC)
{

  srand(0);

  PointCloud<PointXYZ> cloud;
  PointCloud<Normal> normals;

  cloud.resize(7);
  normals.resize(7);

  cloud[0].getVector3fMap() << 3.000501648262739, -1.0005866141772064,
      2.0196299263944386;
  cloud[1].getVector3fMap() << 2.9306387358067587, -0.9355559306559758,
      1.804104008927194;
  cloud[2].getVector3fMap() << 3.1148967392352143, -1.3928055353556932,
      2.1927039488583757;
  cloud[3].getVector3fMap() << 3.0736420787608285, -0.8370133320562925,
      1.7603380061176133;
  cloud[4].getVector3fMap() << 2.88008899080742, -1.245300517665885, 1.7510639730129496;
  cloud[5].getVector3fMap() << 3.0000040500927305, -1.0005041529688534,
      2.0158691660694794;
  cloud[6].getVector3fMap() << 2.983210284063484, -0.5044792022516073,
      2.0456050860401795;
  normals[0].getNormalVector3fMap() << -0.6479150922982518, 0.7576547294171206,
      0.07851970557775474;
  normals[1].getNormalVector3fMap() << 0.45515258767393824, -0.4228856734855979,
      -0.783583964291224;
  normals[2].getNormalVector3fMap() << 0.17884740355312917, -0.6114381536611204,
      0.7708157954335032;
  normals[3].getNormalVector3fMap() << -0.11718185523562125, -0.2593500950773666,
      -0.958647975529547;
  normals[4].getNormalVector3fMap() << -0.04047436729113163, -0.08279792919404502,
      -0.995744107948202;
  normals[5].getNormalVector3fMap() << -0.008017000458096018, 0.9979511214462377,
      0.06347666427791779;
  normals[6].getNormalVector3fMap() << -0.03329532756428898, 0.9826567250055698,
      0.18242034416071792;

  SampleConsensusModelTorus<PointXYZ, Normal>::Ptr model(
      new SampleConsensusModelTorus<PointXYZ, Normal>(cloud.makeShared()));

  model->setInputNormals(normals.makeShared());

  // Create the RANSAC object
  RandomSampleConsensus<PointXYZ> sac(model, 0.2);

  // Algorithm tests
  bool result = sac.computeModel();
  ASSERT_TRUE(result);

  pcl::Indices sample;
  sac.getModel(sample);
  EXPECT_EQ(4, sample.size());
  pcl::Indices inliers;
  sac.getInliers(inliers);
  EXPECT_EQ(7, inliers.size());

  Eigen::VectorXf coeff;
  sac.getModelCoefficients(coeff);

  EXPECT_EQ(8, coeff.size());

  EXPECT_NEAR(coeff[0], 0.25, 1e-2);
  EXPECT_NEAR(coeff[1], 0.25, 1e-2);
  EXPECT_NEAR(coeff[2], 3, 1e-2);
  EXPECT_NEAR(coeff[3], -1, 1e-2);
  EXPECT_NEAR(coeff[4], 2, 1e-2);
  EXPECT_NEAR(coeff[5], 0.0, 1e-2);
  EXPECT_NEAR(coeff[6], 0.0, 1e-2);
  EXPECT_NEAR(std::abs(coeff[7]), 1.0, 1e-2);
  return;

  Eigen::VectorXf coeff_refined;
  model->optimizeModelCoefficients(inliers, coeff, coeff_refined);
  EXPECT_EQ(8, coeff.size());

  EXPECT_NEAR(coeff[0], 0.25, 1e-2);
  EXPECT_NEAR(coeff[1], 0.25, 1e-2);
  EXPECT_NEAR(coeff[2], 3, 1e-2);
  EXPECT_NEAR(coeff[3], -1, 1e-2);
  EXPECT_NEAR(coeff[4], 2, 1e-2);
  EXPECT_NEAR(coeff[5], 0.0, 1e-2);
  EXPECT_NEAR(coeff[6], 0.0, 1e-2);
  EXPECT_NEAR(coeff[7], 1.0, 1e-2);
}

TEST(SampleConsensusModelTorusRefine, RANSAC)
{

  srand(0);

  PointCloud<PointXYZ> cloud;
  PointCloud<Normal> normals;

  cloud.resize(12);
  normals.resize(12);

  cloud[0].getVector3fMap() << 2.052800042174215, -1.499473956010903, 2.5922393000558;
  cloud[1].getVector3fMap() << 3.388588815443773, -0.2804951689253241,
      2.016023579560368;
  cloud[2].getVector3fMap() << 2.1062433380708585, -1.9174254209231951,
      2.2138169934854175;
  cloud[3].getVector3fMap() << 2.9741032000482, -1.0699765160210948, 1.2784833859363935;
  cloud[4].getVector3fMap() << 3.9945837837405858, -0.24398838472758466,
      1.994969222832288;
  cloud[5].getVector3fMap() << 3.29052359025732, -0.7052701711244429,
      1.4026501046485196;
  cloud[6].getVector3fMap() << 3.253762467235399, -1.2666426752546665,
      1.2533731806961965;
  cloud[7].getVector3fMap() << 2.793231427168476, -1.406941876180895, 2.914835409806976;
  cloud[8].getVector3fMap() << 3.427656537026421, -0.3921726018138755,
      1.1167321991754167;
  cloud[9].getVector3fMap() << 3.45310885872988, -1.187857062974888, 0.9128847947344318;

  normals[0].getNormalVector3fMap() << -0.9655752892034741, 0.13480487505578329,
      0.22246798992399325;
  normals[1].getNormalVector3fMap() << -0.9835035116470829, -0.02321732676535275,
      -0.17939286026965295;
  normals[2].getNormalVector3fMap() << -0.6228348353863176, -0.7614744633300792,
      0.17953665231775656;
  normals[3].getNormalVector3fMap() << -0.3027649706212169, 0.4167626949130777,
      0.8571127281131243;
  normals[4].getNormalVector3fMap() << 0.9865410652838972, 0.13739803967452247,
      0.08864821037173687;
  normals[5].getNormalVector3fMap() << -0.723213640950708, -0.05078427284613152,
      0.688754663994597;
  normals[6].getNormalVector3fMap() << 0.4519195477489684, -0.4187464441250127,
      0.7876675300499734;
  normals[7].getNormalVector3fMap() << 0.7370319397802214, -0.6656659398898118,
      0.11693064702813241;
  normals[8].getNormalVector3fMap() << -0.4226770542031876, 0.7762818780175667,
      -0.4676863839279862;
  normals[9].getNormalVector3fMap() << 0.720025487985072, -0.5768131803911037,
      -0.38581064212766236;

  // Uniform noise between -0.1 and 0.1
  cloud[0].getVector3fMap() += Eigen::Vector3f(-0.02519484, 0.03325529, 0.09188957);
  cloud[1].getVector3fMap() += Eigen::Vector3f(0.06969781, -0.06921317, -0.07229406);
  cloud[2].getVector3fMap() += Eigen::Vector3f(-0.00064637, -0.00231905, -0.0080026);
  cloud[3].getVector3fMap() += Eigen::Vector3f(0.05039557, -0.0229141, 0.0594657);
  cloud[4].getVector3fMap() += Eigen::Vector3f(-0.05717322, -0.09670288, 0.00176189);
  cloud[5].getVector3fMap() += Eigen::Vector3f(0.02668492, -0.06824032, 0.05790168);
  cloud[6].getVector3fMap() += Eigen::Vector3f(0.07829713, 0.06426746, 0.04172692);
  cloud[7].getVector3fMap() += Eigen::Vector3f(0.0006326, -0.02518951, -0.00927858);
  cloud[8].getVector3fMap() += Eigen::Vector3f(-0.04975343, 0.09912357, -0.04233801);
  cloud[9].getVector3fMap() += Eigen::Vector3f(-0.04810247, 0.03382804, 0.07958129);

  // Outliers
  cloud[10].getVector3fMap() << 5, 1, 1;
  cloud[11].getVector3fMap() << 5, 2, 1;

  normals[10].getNormalVector3fMap() << 1, 0, 0;
  normals[11].getNormalVector3fMap() << 1, 0, 0;

  SampleConsensusModelTorus<PointXYZ, Normal>::Ptr model(
      new SampleConsensusModelTorus<PointXYZ, Normal>(cloud.makeShared()));

  model->setInputNormals(normals.makeShared());

  // Create the RANSAC object
  RandomSampleConsensus<PointXYZ> sac(model, 0.2);

  // Algorithm tests
  bool result = sac.computeModel();
  ASSERT_TRUE(result);

  pcl::Indices sample;
  sac.getModel(sample);
  EXPECT_EQ(4, sample.size());
  pcl::Indices inliers;
  sac.getInliers(inliers);
  EXPECT_EQ(10, inliers.size());

  Eigen::VectorXf coeff;
  sac.getModelCoefficients(coeff);

  EXPECT_EQ(8, coeff.size());

  EXPECT_NEAR(coeff[0], 1, 2e-1);
  EXPECT_NEAR(coeff[1], 0.3, 2e-1);
  EXPECT_NEAR(coeff[2], 3, 2e-1);
  EXPECT_NEAR(coeff[3], -1, 2e-1);
  EXPECT_NEAR(coeff[4], 2, 2e-1);

  if (coeff[5] < 0){
    coeff[5] *= -1.0;
    coeff[6] *= -1.0;
    coeff[7] *= -1.0;
  }

  EXPECT_NEAR(coeff[5], 0.7071067811865476, 2e-1);
  EXPECT_NEAR(coeff[6], -0.6830127018922194, 2e-1);
  EXPECT_NEAR(coeff[7], 0.1830127018922194, 2e-1);

  Eigen::VectorXf coeff_refined(8);
  return;

  // Optimize goes from 2e-1 to 1e-1
  model->optimizeModelCoefficients(inliers, coeff, coeff_refined);
  EXPECT_EQ(8, coeff.size());

  EXPECT_NEAR(coeff[0], 1, 1e-1);
  EXPECT_NEAR(coeff[1], 0.3, 1e-1);
  EXPECT_NEAR(coeff[2], 3, 1e-1);
  EXPECT_NEAR(coeff[3], -1, 1e-1);
  EXPECT_NEAR(coeff[4], 2, 1e-1);
  EXPECT_NEAR(coeff[5], 0.7071067811865476, 1e-1);
  EXPECT_NEAR(coeff[6], -0.6830127018922194, 1e-1);
  EXPECT_NEAR(coeff[7], 0.1830127018922194, 1e-1);
}

TEST(SampleConsensusModelTorus, RANSAC)
{
  srand(0);

  PointCloud<PointXYZ> cloud;
  PointCloud<Normal> normals;

  cloud.resize(4);
  normals.resize(4);

  cloud[0].getVector3fMap() << 8.359341574088198, 7.22552693060636, 5.4049978066219575;
  cloud[1].getVector3fMap() << 7.777710489873524, 6.9794499622227635, 5.96264148630509;
  cloud[2].getVector3fMap() << 7.578062528900397, 8.466627338184125, 6.764936180563802;
  cloud[3].getVector3fMap() << 6.8073801963063225, 6.950495936581675,
      6.5682651621988140636;

  normals[0].getNormalVector3fMap() << 0.78726775, -0.60899961, -0.09658657;
  normals[1].getNormalVector3fMap() << 0.66500173, 0.11532684, 0.73788374;
  normals[2].getNormalVector3fMap() << -0.58453172, 0.0233942, -0.81103353;
  normals[3].getNormalVector3fMap() << -0.92017329, -0.39125533, 0.01415573;

  // Create a shared 3d torus model pointer directly
  SampleConsensusModelTorus<PointXYZ, Normal>::Ptr model(
      new SampleConsensusModelTorus<PointXYZ, Normal>(cloud.makeShared()));
  model->setInputNormals(normals.makeShared());

  // Create the RANSAC object
  RandomSampleConsensus<PointXYZ> sac(model, 0.11);

  // Algorithm tests
  bool result = sac.computeModel();
  ASSERT_TRUE(result);

  pcl::Indices sample;
  sac.getModel(sample);
  EXPECT_EQ(4, sample.size());
  pcl::Indices inliers;
  sac.getInliers(inliers);
  EXPECT_EQ(4, inliers.size());

  Eigen::VectorXf coeff;
  sac.getModelCoefficients(coeff);

  EXPECT_EQ(8, coeff.size());

  EXPECT_NEAR(coeff[0], 1, 1e-2);
  EXPECT_NEAR(coeff[1], 0.3, 1e-2);

  EXPECT_NEAR(coeff[2], 7.758357590948854, 1e-2);
  EXPECT_NEAR(coeff[3], 7.756009480304242, 1e-2);
  EXPECT_NEAR(coeff[4], 6.297666724054506, 1e-2);

  EXPECT_NEAR(coeff[5], 0.7071067811865475, 1e-2);
  EXPECT_NEAR(coeff[6], -0.5, 1e-2);
  EXPECT_NEAR(coeff[7], 0.5, 1e-2);

  Eigen::VectorXf coeff_refined;
  model->optimizeModelCoefficients(inliers, coeff, coeff_refined);
  EXPECT_EQ(8, coeff.size());

  EXPECT_NEAR(coeff[0], 1, 1e-2);
  EXPECT_NEAR(coeff[1], 0.3, 1e-2);

  EXPECT_NEAR(coeff[2], 7.758357590948854, 1e-2);
  EXPECT_NEAR(coeff[3], 7.756009480304242, 1e-2);
  EXPECT_NEAR(coeff[4], 6.297666724054506, 1e-2);

  if (coeff[5] < 0){
    coeff[5] *= -1.0;
    coeff[6] *= -1.0;
    coeff[7] *= -1.0;
  }

  EXPECT_NEAR(coeff[5], 0.7071067811865475, 1e-2);
  EXPECT_NEAR(coeff[6], -0.5, 1e-2);
  EXPECT_NEAR(coeff[7], 0.5, 1e-2);
}

TEST(SampleConsensusModelTorusSelfIntersectSpindle, RANSAC)
{
  srand(0);

  PointCloud<PointXYZ> cloud;
  PointCloud<Normal> normals;

  cloud.resize(15);
  normals.resize(15);

  cloud[0].getVector3fMap() << 4.197443296388562, -2.244178951074561,
      2.6544058976891054;
  cloud[1].getVector3fMap() << 3.9469472011448596, -2.2109428683077716,
      1.8207364262436627;
  cloud[2].getVector3fMap() << 4.036997360721013, -1.9837065514073593, 2.88062697126334;
  cloud[3].getVector3fMap() << 4.554241490476904, -1.8200324440442106,
      2.242830580711606;
  cloud[4].getVector3fMap() << 3.9088172351876764, -1.7982235216273337,
      2.268412990083617;
  cloud[5].getVector3fMap() << 3.842480541291084, -1.8025598756948282,
      2.2527669926394016;
  cloud[6].getVector3fMap() << 4.3496726790753755, -2.2441275500858833,
      2.239055754148472;
  cloud[7].getVector3fMap() << 4.52235090070925, -1.7373785296059916, 2.466177376096933;
  cloud[8].getVector3fMap() << 3.7710839801895077, -1.7082589118588576,
      2.393963290485919;
  cloud[9].getVector3fMap() << 4.068180803015269, -1.3503789464621836,
      2.3423326381708147;
  cloud[10].getVector3fMap() << 3.873973067071098, -1.7135016016729774,
      2.2124191518411247;
  cloud[11].getVector3fMap() << 4.390758174759903, -2.171435874256942,
      2.214023036691005;
  cloud[12].getVector3fMap() << 4.324106983802413, -1.3650663408422128,
      2.453692863759843;
  cloud[13].getVector3fMap() << 4.345401269961894, -2.0286148560415813,
      2.456689045210522;
  cloud[14].getVector3fMap() << 4.31528844186095, -2.0083582606580292,
      2.367720270538135;
  normals[0].getNormalVector3fMap() << 0.6131882081326164, -0.6055955130301103,
      0.5072024211347066;
  normals[1].getNormalVector3fMap() << -0.37431625455633444, -0.44239562607541916,
      -0.8149683746037361;
  normals[2].getNormalVector3fMap() << 0.03426300530560111, -0.20348141751799728,
      0.9784791051383237;
  normals[3].getNormalVector3fMap() << 0.940856493913579, -0.2960809146555105,
      0.1646971458083096;
  normals[4].getNormalVector3fMap() << -0.6286552509632886, 0.5146645236295431,
      0.5830205858745127;
  normals[5].getNormalVector3fMap() << -0.3871040511550286, 0.724048220462587,
      -0.5708805724705703;
  normals[6].getNormalVector3fMap() << 0.8666661368649906, -0.43068753570421703,
      0.25178970153789454;
  normals[7].getNormalVector3fMap() << 0.9362467937507173, -0.17430389316386408,
      0.30505752575444156;
  normals[8].getNormalVector3fMap() << -0.8229596731853971, 0.3855286394843701,
      -0.4172589656890726;
  normals[9].getNormalVector3fMap() << -0.36653063101311856, 0.9297937437536523,
      -0.033747453321582466;
  normals[10].getNormalVector3fMap() << -0.9907072431684454, -0.07717115427192464,
      -0.11199897893247729;
  normals[11].getNormalVector3fMap() << 0.8661927924780622, -0.3669697390799624,
      0.3391802719184018;
  normals[12].getNormalVector3fMap() << 0.3744106777049837, 0.8911051835726027,
      0.25641411082569643;
  normals[13].getNormalVector3fMap() << 0.8809879144326921, -0.4062669413039098,
      -0.2425025093212462;
  normals[14].getNormalVector3fMap() << 0.8117975834319093, -0.31266565300418725,
      -0.49317833789165666;

  // Create a shared 3d torus model pointer directly
  SampleConsensusModelTorus<PointXYZ, Normal>::Ptr model(
      new SampleConsensusModelTorus<PointXYZ, Normal>(cloud.makeShared()));
  model->setInputNormals(normals.makeShared());

  // Create the RANSAC object
  RandomSampleConsensus<PointXYZ> sac(model, 0.11);

  // Algorithm tests
  bool result = sac.computeModel();
  ASSERT_TRUE(result);

  pcl::Indices sample;
  sac.getModel(sample);
  EXPECT_EQ(4, sample.size());
  pcl::Indices inliers;
  sac.getInliers(inliers);
  EXPECT_EQ(15, inliers.size());

  Eigen::VectorXf coeff;
  sac.getModelCoefficients(coeff);

  EXPECT_EQ(8, coeff.size());
  EXPECT_NEAR(coeff[0], 0.25, 1e-2);
  EXPECT_NEAR(coeff[1], 0.35, 1e-2);
  EXPECT_NEAR(coeff[2], 4.1, 1e-2);
  EXPECT_NEAR(coeff[3], -1.9, 1e-2);
  EXPECT_NEAR(coeff[4], 2.3, 1e-2);
  EXPECT_NEAR(coeff[5], 0.8660254037844385, 1e-2);
  EXPECT_NEAR(coeff[6], -0.4330127018922194, 1e-2);
  EXPECT_NEAR(coeff[7], 0.25000000000000017, 1e-2);

  Eigen::VectorXf coeff_refined;
  model->optimizeModelCoefficients(inliers, coeff, coeff_refined);
  EXPECT_EQ(8, coeff.size());
  EXPECT_NEAR(coeff[0], 0.25, 1e-2);
  EXPECT_NEAR(coeff[1], 0.35, 1e-2);
  EXPECT_NEAR(coeff[2], 4.1, 1e-2);
  EXPECT_NEAR(coeff[3], -1.9, 1e-2);
  EXPECT_NEAR(coeff[4], 2.3, 1e-2);

  if (coeff[5] < 0){
    coeff[5] *= -1.0;
    coeff[6] *= -1.0;
    coeff[7] *= -1.0;
  }

  EXPECT_NEAR(coeff[5], 0.8660254037844385, 1e-2);
  EXPECT_NEAR(coeff[6], -0.4330127018922194, 1e-2);
  EXPECT_NEAR(coeff[7], 0.25000000000000017, 1e-2);
}

int
main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return (RUN_ALL_TESTS());
}
