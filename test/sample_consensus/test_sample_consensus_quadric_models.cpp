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

#include <pcl/test/gtest.h>
#include <pcl/pcl_tests.h> // for EXPECT_XYZ_NEAR

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/sample_consensus/sac_model_cone.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/sample_consensus/sac_model_circle.h>
#include <pcl/sample_consensus/sac_model_circle3d.h>
#include <pcl/sample_consensus/sac_model_normal_sphere.h>
#include <pcl/sample_consensus/sac_model_ellipse3d.h>

using namespace pcl;

using SampleConsensusModelSpherePtr = SampleConsensusModelSphere<PointXYZ>::Ptr;
using SampleConsensusModelConePtr = SampleConsensusModelCone<PointXYZ, Normal>::Ptr;
using SampleConsensusModelCircle2DPtr = SampleConsensusModelCircle2D<PointXYZ>::Ptr;
using SampleConsensusModelCircle3DPtr = SampleConsensusModelCircle3D<PointXYZ>::Ptr;
using SampleConsensusModelCylinderPtr = SampleConsensusModelCylinder<PointXYZ, Normal>::Ptr;
using SampleConsensusModelNormalSpherePtr = SampleConsensusModelNormalSphere<PointXYZ, Normal>::Ptr;
using SampleConsensusModelEllipse3DPtr = SampleConsensusModelEllipse3D<PointXYZ>::Ptr;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (SampleConsensusModelSphere, RANSAC)
{
  srand (0);

  // Use a custom point cloud for these tests until we need something better
  PointCloud<PointXYZ> cloud;
  cloud.resize (10);
  cloud[0].getVector3fMap () << 1.7068f, 1.0684f, 2.2147f;
  cloud[1].getVector3fMap () << 2.4708f, 2.3081f, 1.1736f;
  cloud[2].getVector3fMap () << 2.7609f, 1.9095f, 1.3574f;
  cloud[3].getVector3fMap () << 2.8016f, 1.6704f, 1.5009f;
  cloud[4].getVector3fMap () << 1.8517f, 2.0276f, 1.0112f;
  cloud[5].getVector3fMap () << 1.8726f, 1.3539f, 2.7523f;
  cloud[6].getVector3fMap () << 2.5179f, 2.3218f, 1.2074f;
  cloud[7].getVector3fMap () << 2.4026f, 2.5114f, 2.7588f;
  cloud[8].getVector3fMap () << 2.6999f, 2.5606f, 1.5571f;
  cloud[9].getVector3fMap () << 0.0000f, 0.0000f, 0.0000f;

  // Create a shared sphere model pointer directly
  SampleConsensusModelSpherePtr model (new SampleConsensusModelSphere<PointXYZ> (cloud.makeShared ()));

  // Create the RANSAC object
  RandomSampleConsensus<PointXYZ> sac (model, 0.03);

  // Algorithm tests
  bool result = sac.computeModel ();
  ASSERT_TRUE (result);

  pcl::Indices sample;
  sac.getModel (sample);
  EXPECT_EQ (4, sample.size ());

  pcl::Indices inliers;
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

//////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
class SampleConsensusModelSphereTest : private SampleConsensusModelSphere<PointT>
{
  public:
    using SampleConsensusModelSphere<PointT>::SampleConsensusModelSphere;
    using SampleConsensusModelSphere<PointT>::countWithinDistanceStandard;
#if defined (__SSE__) && defined (__SSE2__) && defined (__SSE4_1__)
    using SampleConsensusModelSphere<PointT>::countWithinDistanceSSE;
#endif
#if defined (__AVX__) && defined (__AVX2__)
    using SampleConsensusModelSphere<PointT>::countWithinDistanceAVX;
#endif
};

TEST (SampleConsensusModelSphere, SIMD_countWithinDistance) // Test if all countWithinDistance implementations return the same value
{
  const auto seed = static_cast<unsigned> (std::time (nullptr));
  srand (seed);
  for (size_t i=0; i<100; i++) // Run as often as you like
  {
    // Generate a cloud with 1000 random points
    PointCloud<PointXYZ> cloud;
    pcl::Indices indices;
    cloud.resize (1000);
    for (std::size_t idx = 0; idx < cloud.size (); ++idx)
    {
      cloud[idx].x = 2.0 * static_cast<float> (rand ()) / RAND_MAX - 1.0;
      cloud[idx].y = 2.0 * static_cast<float> (rand ()) / RAND_MAX - 1.0;
      cloud[idx].z = 2.0 * static_cast<float> (rand ()) / RAND_MAX - 1.0;
      if (rand () % 3 != 0)
      {
        indices.push_back (static_cast<int> (idx));
      }
    }
    SampleConsensusModelSphereTest<PointXYZ> model (cloud.makeShared (), indices, true);

    // Generate random sphere model parameters
    Eigen::VectorXf model_coefficients(4);
    model_coefficients << 2.0 * static_cast<float> (rand ()) / RAND_MAX - 1.0,
                          2.0 * static_cast<float> (rand ()) / RAND_MAX - 1.0,
                          2.0 * static_cast<float> (rand ()) / RAND_MAX - 1.0,
                          0.15 * static_cast<float> (rand ()) / RAND_MAX; // center and radius

    const double threshold = 0.15 * static_cast<double> (rand ()) / RAND_MAX; // threshold in [0; 0.1]

    // The number of inliers is usually somewhere between 0 and 10
    const auto res_standard = model.countWithinDistanceStandard (model_coefficients, threshold); // Standard
    PCL_DEBUG ("seed=%lu, i=%lu, model=(%f, %f, %f, %f), threshold=%f, res_standard=%lu\n", seed, i,
               model_coefficients(0), model_coefficients(1), model_coefficients(2), model_coefficients(3), threshold, res_standard);
#if defined (__SSE__) && defined (__SSE2__) && defined (__SSE4_1__)
    const auto res_sse      = model.countWithinDistanceSSE (model_coefficients, threshold); // SSE
    ASSERT_EQ (res_standard, res_sse);
#endif
#if defined (__AVX__) && defined (__AVX2__)
    const auto res_avx      = model.countWithinDistanceAVX (model_coefficients, threshold); // AVX
    ASSERT_EQ (res_standard, res_avx);
#endif
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (SampleConsensusModelNormalSphere, RANSAC)
{
  srand (0);

  // Use a custom point cloud for these tests until we need something better
  PointCloud<PointXYZ> cloud;
  PointCloud<Normal> normals;
  cloud.resize (27); normals.resize (27);
  cloud[ 0].getVector3fMap () << -0.014695f,  0.009549f, 0.954775f;
  cloud[ 1].getVector3fMap () <<  0.014695f,  0.009549f, 0.954775f;
  cloud[ 2].getVector3fMap () << -0.014695f,  0.040451f, 0.954775f;
  cloud[ 3].getVector3fMap () <<  0.014695f,  0.040451f, 0.954775f;
  cloud[ 4].getVector3fMap () << -0.009082f, -0.015451f, 0.972049f;
  cloud[ 5].getVector3fMap () <<  0.009082f, -0.015451f, 0.972049f;
  cloud[ 6].getVector3fMap () << -0.038471f,  0.009549f, 0.972049f;
  cloud[ 7].getVector3fMap () <<  0.038471f,  0.009549f, 0.972049f;
  cloud[ 8].getVector3fMap () << -0.038471f,  0.040451f, 0.972049f;
  cloud[ 9].getVector3fMap () <<  0.038471f,  0.040451f, 0.972049f;
  cloud[10].getVector3fMap () << -0.009082f,  0.065451f, 0.972049f;
  cloud[11].getVector3fMap () <<  0.009082f,  0.065451f, 0.972049f;
  cloud[12].getVector3fMap () << -0.023776f, -0.015451f, 0.982725f;
  cloud[13].getVector3fMap () <<  0.023776f, -0.015451f, 0.982725f;
  cloud[14].getVector3fMap () << -0.023776f,  0.065451f, 0.982725f;
  cloud[15].getVector3fMap () <<  0.023776f,  0.065451f, 0.982725f;
  cloud[16].getVector3fMap () << -0.000000f, -0.025000f, 1.000000f;
  cloud[17].getVector3fMap () <<  0.000000f, -0.025000f, 1.000000f;
  cloud[18].getVector3fMap () << -0.029389f, -0.015451f, 1.000000f;
  cloud[19].getVector3fMap () <<  0.029389f, -0.015451f, 1.000000f;
  cloud[20].getVector3fMap () << -0.047553f,  0.009549f, 1.000000f;
  cloud[21].getVector3fMap () <<  0.047553f,  0.009549f, 1.000000f;
  cloud[22].getVector3fMap () << -0.047553f,  0.040451f, 1.000000f;
  cloud[23].getVector3fMap () <<  0.047553f,  0.040451f, 1.000000f;
  cloud[24].getVector3fMap () << -0.029389f,  0.065451f, 1.000000f;
  cloud[25].getVector3fMap () <<  0.029389f,  0.065451f, 1.000000f;
  cloud[26].getVector3fMap () <<  0.000000f,  0.075000f, 1.000000f;

  normals[ 0].getNormalVector3fMap () << -0.293893f, -0.309017f, -0.904509f;
  normals[ 1].getNormalVector3fMap () <<  0.293893f, -0.309017f, -0.904508f;
  normals[ 2].getNormalVector3fMap () << -0.293893f,  0.309017f, -0.904509f;
  normals[ 3].getNormalVector3fMap () <<  0.293893f,  0.309017f, -0.904508f;
  normals[ 4].getNormalVector3fMap () << -0.181636f, -0.809017f, -0.559017f;
  normals[ 5].getNormalVector3fMap () <<  0.181636f, -0.809017f, -0.559017f;
  normals[ 6].getNormalVector3fMap () << -0.769421f, -0.309017f, -0.559017f;
  normals[ 7].getNormalVector3fMap () <<  0.769421f, -0.309017f, -0.559017f;
  normals[ 8].getNormalVector3fMap () << -0.769421f,  0.309017f, -0.559017f;
  normals[ 9].getNormalVector3fMap () <<  0.769421f,  0.309017f, -0.559017f;
  normals[10].getNormalVector3fMap () << -0.181636f,  0.809017f, -0.559017f;
  normals[11].getNormalVector3fMap () <<  0.181636f,  0.809017f, -0.559017f;
  normals[12].getNormalVector3fMap () << -0.475528f, -0.809017f, -0.345491f;
  normals[13].getNormalVector3fMap () <<  0.475528f, -0.809017f, -0.345491f;
  normals[14].getNormalVector3fMap () << -0.475528f,  0.809017f, -0.345491f;
  normals[15].getNormalVector3fMap () <<  0.475528f,  0.809017f, -0.345491f;
  normals[16].getNormalVector3fMap () << -0.000000f, -1.000000f,  0.000000f;
  normals[17].getNormalVector3fMap () <<  0.000000f, -1.000000f,  0.000000f;
  normals[18].getNormalVector3fMap () << -0.587785f, -0.809017f,  0.000000f;
  normals[19].getNormalVector3fMap () <<  0.587785f, -0.809017f,  0.000000f;
  normals[20].getNormalVector3fMap () << -0.951057f, -0.309017f,  0.000000f;
  normals[21].getNormalVector3fMap () <<  0.951057f, -0.309017f,  0.000000f;
  normals[22].getNormalVector3fMap () << -0.951057f,  0.309017f,  0.000000f;
  normals[23].getNormalVector3fMap () <<  0.951057f,  0.309017f,  0.000000f;
  normals[24].getNormalVector3fMap () << -0.587785f,  0.809017f,  0.000000f;
  normals[25].getNormalVector3fMap () <<  0.587785f,  0.809017f,  0.000000f;
  normals[26].getNormalVector3fMap () <<  0.000000f,  1.000000f,  0.000000f;

  // Create a shared sphere model pointer directly
  SampleConsensusModelNormalSpherePtr model (new SampleConsensusModelNormalSphere<PointXYZ, Normal> (cloud.makeShared ()));
  model->setInputNormals (normals.makeShared ());

  // Create the RANSAC object
  RandomSampleConsensus<PointXYZ> sac (model, 0.03);

  // Algorithm tests
  bool result = sac.computeModel ();
  ASSERT_TRUE (result);

  pcl::Indices sample;
  sac.getModel (sample);
  EXPECT_EQ (4, sample.size ());

  pcl::Indices inliers;
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
  cloud.resize (31); normals.resize (31);

  cloud[ 0].getVector3fMap () << -0.011247f, 0.200000f, 0.965384f;
  cloud[ 1].getVector3fMap () <<  0.000000f, 0.200000f, 0.963603f;
  cloud[ 2].getVector3fMap () <<  0.011247f, 0.200000f, 0.965384f;
  cloud[ 3].getVector3fMap () << -0.016045f, 0.175000f, 0.977916f;
  cloud[ 4].getVector3fMap () << -0.008435f, 0.175000f, 0.974038f;
  cloud[ 5].getVector3fMap () <<  0.004218f, 0.175000f, 0.973370f;
  cloud[ 6].getVector3fMap () <<  0.016045f, 0.175000f, 0.977916f;
  cloud[ 7].getVector3fMap () << -0.025420f, 0.200000f, 0.974580f;
  cloud[ 8].getVector3fMap () <<  0.025420f, 0.200000f, 0.974580f;
  cloud[ 9].getVector3fMap () << -0.012710f, 0.150000f, 0.987290f;
  cloud[10].getVector3fMap () << -0.005624f, 0.150000f, 0.982692f;
  cloud[11].getVector3fMap () <<  0.002812f, 0.150000f, 0.982247f;
  cloud[12].getVector3fMap () <<  0.012710f, 0.150000f, 0.987290f;
  cloud[13].getVector3fMap () << -0.022084f, 0.175000f, 0.983955f;
  cloud[14].getVector3fMap () <<  0.022084f, 0.175000f, 0.983955f;
  cloud[15].getVector3fMap () << -0.034616f, 0.200000f, 0.988753f;
  cloud[16].getVector3fMap () <<  0.034616f, 0.200000f, 0.988753f;
  cloud[17].getVector3fMap () << -0.006044f, 0.125000f, 0.993956f;
  cloud[18].getVector3fMap () <<  0.004835f, 0.125000f, 0.993345f;
  cloud[19].getVector3fMap () << -0.017308f, 0.150000f, 0.994376f;
  cloud[20].getVector3fMap () <<  0.017308f, 0.150000f, 0.994376f;
  cloud[21].getVector3fMap () << -0.025962f, 0.175000f, 0.991565f;
  cloud[22].getVector3fMap () <<  0.025962f, 0.175000f, 0.991565f;
  cloud[23].getVector3fMap () << -0.009099f, 0.125000f, 1.000000f;
  cloud[24].getVector3fMap () <<  0.009099f, 0.125000f, 1.000000f;
  cloud[25].getVector3fMap () << -0.018199f, 0.150000f, 1.000000f;
  cloud[26].getVector3fMap () <<  0.018199f, 0.150000f, 1.000000f;
  cloud[27].getVector3fMap () << -0.027298f, 0.175000f, 1.000000f;
  cloud[28].getVector3fMap () <<  0.027298f, 0.175000f, 1.000000f;
  cloud[29].getVector3fMap () << -0.036397f, 0.200000f, 1.000000f;
  cloud[30].getVector3fMap () <<  0.036397f, 0.200000f, 1.000000f;

  normals[ 0].getNormalVector3fMap () << -0.290381f, -0.342020f, -0.893701f;
  normals[ 1].getNormalVector3fMap () <<  0.000000f, -0.342020f, -0.939693f;
  normals[ 2].getNormalVector3fMap () <<  0.290381f, -0.342020f, -0.893701f;
  normals[ 3].getNormalVector3fMap () << -0.552338f, -0.342020f, -0.760227f;
  normals[ 4].getNormalVector3fMap () << -0.290381f, -0.342020f, -0.893701f;
  normals[ 5].getNormalVector3fMap () <<  0.145191f, -0.342020f, -0.916697f;
  normals[ 6].getNormalVector3fMap () <<  0.552337f, -0.342020f, -0.760227f;
  normals[ 7].getNormalVector3fMap () << -0.656282f, -0.342020f, -0.656283f;
  normals[ 8].getNormalVector3fMap () <<  0.656282f, -0.342020f, -0.656283f;
  normals[ 9].getNormalVector3fMap () << -0.656283f, -0.342020f, -0.656282f;
  normals[10].getNormalVector3fMap () << -0.290381f, -0.342020f, -0.893701f;
  normals[11].getNormalVector3fMap () <<  0.145191f, -0.342020f, -0.916697f;
  normals[12].getNormalVector3fMap () <<  0.656282f, -0.342020f, -0.656282f;
  normals[13].getNormalVector3fMap () << -0.760228f, -0.342020f, -0.552337f;
  normals[14].getNormalVector3fMap () <<  0.760228f, -0.342020f, -0.552337f;
  normals[15].getNormalVector3fMap () << -0.893701f, -0.342020f, -0.290380f;
  normals[16].getNormalVector3fMap () <<  0.893701f, -0.342020f, -0.290380f;
  normals[17].getNormalVector3fMap () << -0.624162f, -0.342020f, -0.624162f;
  normals[18].getNormalVector3fMap () <<  0.499329f, -0.342020f, -0.687268f;
  normals[19].getNormalVector3fMap () << -0.893701f, -0.342020f, -0.290380f;
  normals[20].getNormalVector3fMap () <<  0.893701f, -0.342020f, -0.290380f;
  normals[21].getNormalVector3fMap () << -0.893701f, -0.342020f, -0.290381f;
  normals[22].getNormalVector3fMap () <<  0.893701f, -0.342020f, -0.290381f;
  normals[23].getNormalVector3fMap () << -0.939693f, -0.342020f,  0.000000f;
  normals[24].getNormalVector3fMap () <<  0.939693f, -0.342020f,  0.000000f;
  normals[25].getNormalVector3fMap () << -0.939693f, -0.342020f,  0.000000f;
  normals[26].getNormalVector3fMap () <<  0.939693f, -0.342020f,  0.000000f;
  normals[27].getNormalVector3fMap () << -0.939693f, -0.342020f,  0.000000f;
  normals[28].getNormalVector3fMap () <<  0.939693f, -0.342020f,  0.000000f;
  normals[29].getNormalVector3fMap () << -0.939693f, -0.342020f,  0.000000f;
  normals[30].getNormalVector3fMap () <<  0.939693f, -0.342020f,  0.000000f;

  // Create a shared cylinder model pointer directly
  SampleConsensusModelConePtr model (new SampleConsensusModelCone<PointXYZ, Normal> (cloud.makeShared ()));
  model->setInputNormals (normals.makeShared ());

  // Create the RANSAC object
  RandomSampleConsensus<PointXYZ> sac (model, 0.03);

  // Algorithm tests
  bool result = sac.computeModel ();
  ASSERT_TRUE (result);

  pcl::Indices sample;
  sac.getModel (sample);
  EXPECT_EQ (3, sample.size ());

  pcl::Indices inliers;
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
  cloud.resize (20); normals.resize (20);

  cloud[ 0].getVector3fMap () << -0.499902f, 2.199701f, 0.000008f;
  cloud[ 1].getVector3fMap () << -0.875397f, 2.030177f, 0.050104f;
  cloud[ 2].getVector3fMap () << -0.995875f, 1.635973f, 0.099846f;
  cloud[ 3].getVector3fMap () << -0.779523f, 1.285527f, 0.149961f;
  cloud[ 4].getVector3fMap () << -0.373285f, 1.216488f, 0.199959f;
  cloud[ 5].getVector3fMap () << -0.052893f, 1.475973f, 0.250101f;
  cloud[ 6].getVector3fMap () << -0.036558f, 1.887591f, 0.299839f;
  cloud[ 7].getVector3fMap () << -0.335048f, 2.171994f, 0.350001f;
  cloud[ 8].getVector3fMap () << -0.745456f, 2.135528f, 0.400072f;
  cloud[ 9].getVector3fMap () << -0.989282f, 1.803311f, 0.449983f;
  cloud[10].getVector3fMap () << -0.900651f, 1.400701f, 0.500126f;
  cloud[11].getVector3fMap () << -0.539658f, 1.201468f, 0.550079f;
  cloud[12].getVector3fMap () << -0.151875f, 1.340951f, 0.599983f;
  cloud[13].getVector3fMap () << -0.000724f, 1.724373f, 0.649882f;
  cloud[14].getVector3fMap () << -0.188573f, 2.090983f, 0.699854f;
  cloud[15].getVector3fMap () << -0.587925f, 2.192257f, 0.749956f;
  cloud[16].getVector3fMap () << -0.927724f, 1.958846f, 0.800008f;
  cloud[17].getVector3fMap () << -0.976888f, 1.549655f, 0.849970f;
  cloud[18].getVector3fMap () << -0.702003f, 1.242707f, 0.899954f;
  cloud[19].getVector3fMap () << -0.289916f, 1.246296f, 0.950075f;

  normals[ 0].getNormalVector3fMap () <<  0.000098f,  1.000098f,  0.000008f;
  normals[ 1].getNormalVector3fMap () << -0.750891f,  0.660413f,  0.000104f;
  normals[ 2].getNormalVector3fMap () << -0.991765f, -0.127949f, -0.000154f;
  normals[ 3].getNormalVector3fMap () << -0.558918f, -0.829439f, -0.000039f;
  normals[ 4].getNormalVector3fMap () <<  0.253627f, -0.967447f, -0.000041f;
  normals[ 5].getNormalVector3fMap () <<  0.894105f, -0.447965f,  0.000101f;
  normals[ 6].getNormalVector3fMap () <<  0.926852f,  0.375543f, -0.000161f;
  normals[ 7].getNormalVector3fMap () <<  0.329948f,  0.943941f,  0.000001f;
  normals[ 8].getNormalVector3fMap () << -0.490966f,  0.871203f,  0.000072f;
  normals[ 9].getNormalVector3fMap () << -0.978507f,  0.206425f, -0.000017f;
  normals[10].getNormalVector3fMap () << -0.801227f, -0.598534f,  0.000126f;
  normals[11].getNormalVector3fMap () << -0.079447f, -0.996697f,  0.000079f;
  normals[12].getNormalVector3fMap () <<  0.696154f, -0.717889f, -0.000017f;
  normals[13].getNormalVector3fMap () <<  0.998685f,  0.048502f, -0.000118f;
  normals[14].getNormalVector3fMap () <<  0.622933f,  0.782133f, -0.000146f;
  normals[15].getNormalVector3fMap () << -0.175948f,  0.984480f, -0.000044f;
  normals[16].getNormalVector3fMap () << -0.855476f,  0.517824f,  0.000008f;
  normals[17].getNormalVector3fMap () << -0.953769f, -0.300571f, -0.000030f;
  normals[18].getNormalVector3fMap () << -0.404035f, -0.914700f, -0.000046f;
  normals[19].getNormalVector3fMap () <<  0.420154f, -0.907445f,  0.000075f;

  // Create a shared cylinder model pointer directly
  SampleConsensusModelCylinderPtr model (new SampleConsensusModelCylinder<PointXYZ, Normal> (cloud.makeShared ()));
  model->setInputNormals (normals.makeShared ());

  // Create the RANSAC object
  RandomSampleConsensus<PointXYZ> sac (model, 0.03);

  // Algorithm tests
  bool result = sac.computeModel ();
  ASSERT_TRUE (result);

  pcl::Indices sample;
  sac.getModel (sample);
  EXPECT_EQ (2, sample.size ());

  pcl::Indices inliers;
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
  cloud.resize (18);

  cloud[ 0].getVector3fMap () << 3.587751f, -4.190982f, 0.0f;
  cloud[ 1].getVector3fMap () << 3.808883f, -4.412265f, 0.0f;
  cloud[ 2].getVector3fMap () << 3.587525f, -5.809143f, 0.0f;
  cloud[ 3].getVector3fMap () << 2.999913f, -5.999980f, 0.0f;
  cloud[ 4].getVector3fMap () << 2.412224f, -5.809090f, 0.0f;
  cloud[ 5].getVector3fMap () << 2.191080f, -5.587682f, 0.0f;
  cloud[ 6].getVector3fMap () << 2.048941f, -5.309003f, 0.0f;
  cloud[ 7].getVector3fMap () << 2.000397f, -4.999944f, 0.0f;
  cloud[ 8].getVector3fMap () << 2.999953f, -6.000056f, 0.0f;
  cloud[ 9].getVector3fMap () << 2.691127f, -5.951136f, 0.0f;
  cloud[10].getVector3fMap () << 2.190892f, -5.587838f, 0.0f;
  cloud[11].getVector3fMap () << 2.048874f, -5.309052f, 0.0f;
  cloud[12].getVector3fMap () << 1.999990f, -5.000147f, 0.0f;
  cloud[13].getVector3fMap () << 2.049026f, -4.690918f, 0.0f;
  cloud[14].getVector3fMap () << 2.190956f, -4.412162f, 0.0f;
  cloud[15].getVector3fMap () << 2.412231f, -4.190918f, 0.0f;
  cloud[16].getVector3fMap () << 2.691027f, -4.049060f, 0.0f;
  cloud[17].getVector3fMap () << 2.000000f, -3.000000f, 0.0f;

  // Create a shared 2d circle model pointer directly
  SampleConsensusModelCircle2DPtr model (new SampleConsensusModelCircle2D<PointXYZ> (cloud.makeShared ()));

  // Create the RANSAC object
  RandomSampleConsensus<PointXYZ> sac (model, 0.03);

  // Algorithm tests
  bool result = sac.computeModel ();
  ASSERT_TRUE (result);

  pcl::Indices sample;
  sac.getModel (sample);
  EXPECT_EQ (3, sample.size ());

  pcl::Indices inliers;
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

///////////////////////////////////////////////////////////////////////////////

TEST (SampleConsensusModelCircle2D, SampleValidationPointsValid)
{
  PointCloud<PointXYZ> cloud;
  cloud.resize (3);

  cloud[0].getVector3fMap () <<  1.0f,  2.0f,  0.0f;
  cloud[1].getVector3fMap () <<  0.0f,  1.0f,  0.0f;
  cloud[2].getVector3fMap () <<  1.5f,  0.0f,  0.0f;

  // Create a shared line model pointer directly
  SampleConsensusModelCircle2DPtr model (
    new SampleConsensusModelCircle2D<PointXYZ> (cloud.makeShared ()));

  // Algorithm tests
  pcl::Indices samples;
  int iterations = 0;
  model->getSamples(iterations, samples);
  EXPECT_EQ (samples.size(), 3);

  Eigen::VectorXf modelCoefficients;
  EXPECT_TRUE (model->computeModelCoefficients (samples, modelCoefficients));

  EXPECT_NEAR (modelCoefficients[0], 1.05f, 1e-3);   // center x
  EXPECT_NEAR (modelCoefficients[1], 0.95f, 1e-3);   // center y
  EXPECT_NEAR (modelCoefficients[2], std::sqrt (1.105f), 1e-3);  // radius
}

using PointVector = std::vector<PointXYZ>;
class SampleConsensusModelCircle2DCollinearTest
  : public testing::TestWithParam<PointVector> {
  protected:
    void SetUp() override {
      pointCloud_ = make_shared<PointCloud<PointXYZ>>();
      for(const auto& point : GetParam()) {
        pointCloud_->emplace_back(point);
      }
    }

    PointCloud<PointXYZ>::Ptr pointCloud_ = nullptr;
};

// Parametric tests clearly show the input for which they failed and provide
// clearer feedback if something is changed in the future.
TEST_P (SampleConsensusModelCircle2DCollinearTest, SampleValidationPointsInvalid)
{
  ASSERT_NE (pointCloud_, nullptr);

  SampleConsensusModelCircle2DPtr model (
      new SampleConsensusModelCircle2D<PointXYZ> (pointCloud_));
  ASSERT_GE (model->getInputCloud ()->size (), model->getSampleSize ());

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
  EXPECT_FALSE (model->computeModelCoefficients (forcedSamples, modelCoefficients));
}

INSTANTIATE_TEST_SUITE_P (CollinearInputs, SampleConsensusModelCircle2DCollinearTest,
  testing::Values( // Throw in some negative coordinates and "asymmetric" points to cover more cases
    PointVector {{ 0.0f,  0.0f, 0.0f}, { 1.0f,  0.0f, 0.0f}, { 2.0f,  0.0f, 0.0f}}, // collinear along x-axis
    PointVector {{-1.0f,  0.0f, 0.0f}, { 1.0f,  0.0f, 0.0f}, { 2.0f,  0.0f, 0.0f}},
    PointVector {{ 0.0f, -1.0f, 0.0f}, { 0.0f,  1.0f, 0.0f}, { 0.0f,  2.0f, 0.0f}}, // collinear along y-axis
    PointVector {{ 0.0f, -1.0f, 0.0f}, { 0.0f,  1.0f, 0.0f}, { 0.0f,  2.0f, 0.0f}},
    PointVector {{ 0.0f,  0.0f, 0.0f}, { 1.0f,  1.0f, 0.0f}, { 2.0f,  2.0f, 0.0f}}, // collinear along diagonal
    PointVector {{ 0.0f,  0.0f, 0.0f}, {-1.0f, -1.0f, 0.0f}, {-2.0f, -2.0f, 0.0f}},
    PointVector {{-3.0f, -6.5f, 0.0f}, {-2.0f, -0.5f, 0.0f}, {-1.5f,  2.5f, 0.0f}}, // other collinear input
    PointVector {{ 2.0f,  2.0f, 0.0f}, { 0.0f,  0.0f, 0.0f}, { 0.0f,  0.0f, 0.0f}}, // two points equal
    PointVector {{ 0.0f,  0.0f, 0.0f}, { 2.0f,  2.0f, 0.0f}, { 0.0f,  0.0f, 0.0f}},
    PointVector {{ 0.0f,  0.0f, 0.0f}, { 0.0f,  0.0f, 0.0f}, { 2.0f,  2.0f, 0.0f}},
    PointVector {{ 1.0f,  1.0f, 0.0f}, { 1.0f,  1.0f, 0.0f}, { 1.0f,  1.0f, 0.0f}}  // all points equal
  )
);

//////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
class SampleConsensusModelCircle2DTest : private SampleConsensusModelCircle2D<PointT>
{
  public:
    using SampleConsensusModelCircle2D<PointT>::SampleConsensusModelCircle2D;
    using SampleConsensusModelCircle2D<PointT>::countWithinDistanceStandard;
#if defined (__SSE__) && defined (__SSE2__) && defined (__SSE4_1__)
    using SampleConsensusModelCircle2D<PointT>::countWithinDistanceSSE;
#endif
#if defined (__AVX__) && defined (__AVX2__)
    using SampleConsensusModelCircle2D<PointT>::countWithinDistanceAVX;
#endif
};

TEST (SampleConsensusModelCircle2D, SIMD_countWithinDistance) // Test if all countWithinDistance implementations return the same value
{
  const auto seed = static_cast<unsigned> (std::time (nullptr));
  srand (seed);
  for (size_t i=0; i<100; i++) // Run as often as you like
  {
    // Generate a cloud with 1000 random points
    PointCloud<PointXYZ> cloud;
    pcl::Indices indices;
    cloud.resize (1000);
    for (std::size_t idx = 0; idx < cloud.size (); ++idx)
    {
      cloud[idx].x = 2.0 * static_cast<float> (rand ()) / RAND_MAX - 1.0;
      cloud[idx].y = 2.0 * static_cast<float> (rand ()) / RAND_MAX - 1.0;
      cloud[idx].z = 2.0 * static_cast<float> (rand ()) / RAND_MAX - 1.0;
      if (rand () % 2 == 0)
      {
        indices.push_back (static_cast<int> (idx));
      }
    }
    SampleConsensusModelCircle2DTest<PointXYZ> model (cloud.makeShared (), indices, true);

    // Generate random circle model parameters
    Eigen::VectorXf model_coefficients(3);
    model_coefficients << 2.0 * static_cast<float> (rand ()) / RAND_MAX - 1.0,
                          2.0 * static_cast<float> (rand ()) / RAND_MAX - 1.0,
                          0.1 * static_cast<float> (rand ()) / RAND_MAX; // center and radius

    const double threshold = 0.1 * static_cast<double> (rand ()) / RAND_MAX; // threshold in [0; 0.1]

    // The number of inliers is usually somewhere between 0 and 20
    const auto res_standard = model.countWithinDistanceStandard (model_coefficients, threshold); // Standard
    PCL_DEBUG ("seed=%lu, i=%lu, model=(%f, %f, %f), threshold=%f, res_standard=%lu\n", seed, i,
               model_coefficients(0), model_coefficients(1), model_coefficients(2), threshold, res_standard);
#if defined (__SSE__) && defined (__SSE2__) && defined (__SSE4_1__)
    const auto res_sse      = model.countWithinDistanceSSE (model_coefficients, threshold); // SSE
    ASSERT_EQ (res_standard, res_sse);
#endif
#if defined (__AVX__) && defined (__AVX2__)
    const auto res_avx      = model.countWithinDistanceAVX (model_coefficients, threshold); // AVX
    ASSERT_EQ (res_standard, res_avx);
#endif
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (SampleConsensusModelCircle3D, RANSAC)
{
  srand (0);

  // Use a custom point cloud for these tests until we need something better
  PointCloud<PointXYZ> cloud;
  cloud.resize (20);

  cloud[ 0].getVector3fMap () << 1.00000000f, 5.0000000f, -2.9000001f;
  cloud[ 1].getVector3fMap () << 1.03420200f, 5.0000000f, -2.9060307f;
  cloud[ 2].getVector3fMap () << 1.06427870f, 5.0000000f, -2.9233956f;
  cloud[ 3].getVector3fMap () << 1.08660260f, 5.0000000f, -2.9500000f;
  cloud[ 4].getVector3fMap () << 1.09848080f, 5.0000000f, -2.9826353f;
  cloud[ 5].getVector3fMap () << 1.09848080f, 5.0000000f, -3.0173647f;
  cloud[ 6].getVector3fMap () << 1.08660260f, 5.0000000f, -3.0500000f;
  cloud[ 7].getVector3fMap () << 1.06427870f, 5.0000000f, -3.0766044f;
  cloud[ 8].getVector3fMap () << 1.03420200f, 5.0000000f, -3.0939693f;
  cloud[ 9].getVector3fMap () << 1.00000000f, 5.0000000f, -3.0999999f;
  cloud[10].getVector3fMap () << 0.96579796f, 5.0000000f, -3.0939693f;
  cloud[11].getVector3fMap () << 0.93572122f, 5.0000000f, -3.0766044f;
  cloud[12].getVector3fMap () << 0.91339743f, 5.0000000f, -3.0500000f;
  cloud[13].getVector3fMap () << 0.90151924f, 5.0000000f, -3.0173647f;
  cloud[14].getVector3fMap () << 0.90151924f, 5.0000000f, -2.9826353f;
  cloud[15].getVector3fMap () << 0.91339743f, 5.0000000f, -2.9500000f;
  cloud[16].getVector3fMap () << 0.93572122f, 5.0000000f, -2.9233956f;
  cloud[17].getVector3fMap () << 0.96579796f, 5.0000000f, -2.9060307f;
  cloud[18].getVector3fMap () << 0.85000002f, 4.8499999f, -3.1500001f;
  cloud[19].getVector3fMap () << 1.15000000f, 5.1500001f, -2.8499999f;

  // Create a shared 3d circle model pointer directly
  SampleConsensusModelCircle3DPtr model (new SampleConsensusModelCircle3D<PointXYZ> (cloud.makeShared ()));

  // Create the RANSAC object
  RandomSampleConsensus<PointXYZ> sac (model, 0.03);

  // Algorithm tests
  bool result = sac.computeModel ();
  ASSERT_TRUE (result);

  pcl::Indices sample;
  sac.getModel (sample);
  EXPECT_EQ (3, sample.size ());

  pcl::Indices inliers;
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
  // Use abs in y component because both variants are valid normal vectors
  EXPECT_NEAR ( 1.0, std::abs (coeff[5]), 1e-3);
  EXPECT_NEAR ( 0.0, coeff[6], 1e-3);

  Eigen::VectorXf coeff_refined;
  model->optimizeModelCoefficients (inliers, coeff, coeff_refined);
  EXPECT_EQ (7, coeff_refined.size ());
  EXPECT_NEAR ( 1.0, coeff_refined[0], 1e-3);
  EXPECT_NEAR ( 5.0, coeff_refined[1], 1e-3);
  EXPECT_NEAR (-3.0, coeff_refined[2], 1e-3);
  EXPECT_NEAR ( 0.1, coeff_refined[3], 1e-3);
  EXPECT_NEAR ( 0.0, coeff_refined[4], 1e-3);
  EXPECT_NEAR ( 1.0, std::abs (coeff_refined[5]), 1e-3);
  EXPECT_NEAR ( 0.0, coeff_refined[6], 1e-3);
}

TEST (SampleConsensusModelSphere, projectPoints)
{
  Eigen::VectorXf model_coefficients(4);
  model_coefficients << -0.32, -0.89, 0.37, 0.12; // center and radius

  pcl::PointCloud<pcl::PointXYZ>::Ptr input(new pcl::PointCloud<pcl::PointXYZ>);
  input->emplace_back(-0.259754, -0.950873,  0.318377); // inlier, dist from center=0.10
  input->emplace_back( 0.595892,  0.455094,  0.025545); // outlier, not projected
  input->emplace_back(-0.221871, -0.973718,  0.353817); // inlier, dist from center=0.13
  input->emplace_back(-0.332269, -0.848851,  0.437499); // inlier, dist from center=0.08
  input->emplace_back(-0.242308, -0.561036, -0.365535); // outlier, not projected
  input->emplace_back(-0.327668, -0.800009,  0.290988); // inlier, dist from center=0.12
  input->emplace_back(-0.173948, -0.883831,  0.403625); // inlier, dist from center=0.15
  input->emplace_back(-0.033891,  0.624537, -0.606994); // outlier, not projected

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
  for(int i=0; i<5; ++i)
    EXPECT_XYZ_NEAR(projected_points[i], projected_truth[i], 1e-5);

  pcl::PointCloud<pcl::PointXYZ> projected_points_all;
  model.projectPoints(inliers, model_coefficients, projected_points_all, true);
  EXPECT_EQ(projected_points_all.size(), 8);
  EXPECT_XYZ_NEAR(projected_points_all[0], projected_truth[0], 1e-5);
  EXPECT_XYZ_NEAR(projected_points_all[1],        (*input)[1], 1e-5);
  EXPECT_XYZ_NEAR(projected_points_all[2], projected_truth[1], 1e-5);
  EXPECT_XYZ_NEAR(projected_points_all[3], projected_truth[2], 1e-5);
  EXPECT_XYZ_NEAR(projected_points_all[4],        (*input)[4], 1e-5);
  EXPECT_XYZ_NEAR(projected_points_all[5], projected_truth[3], 1e-5);
  EXPECT_XYZ_NEAR(projected_points_all[6], projected_truth[4], 1e-5);
  EXPECT_XYZ_NEAR(projected_points_all[7],        (*input)[7], 1e-5);
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
  EXPECT_XYZ_NEAR(projected_points_all[1],        (*input)[1], 1e-5);
  EXPECT_XYZ_NEAR(projected_points_all[2], projected_truth[1], 1e-5);
  EXPECT_XYZ_NEAR(projected_points_all[3], projected_truth[2], 1e-5);
  EXPECT_XYZ_NEAR(projected_points_all[4],        (*input)[4], 1e-5);
  EXPECT_XYZ_NEAR(projected_points_all[5], projected_truth[3], 1e-5);
  EXPECT_XYZ_NEAR(projected_points_all[6], projected_truth[4], 1e-5);
  EXPECT_XYZ_NEAR(projected_points_all[7],        (*input)[7], 1e-5);
}

TEST(SampleConsensusModelEllipse3D, RANSAC)
{
  srand(0);

  // Using a custom point cloud on a tilted plane
  PointCloud<PointXYZ> cloud;
  cloud.resize(22);

  cloud[ 0].getVector3fMap() << 1.000000, 5.000000, 3.000000;
  cloud[ 1].getVector3fMap() << 0.690983, 5.000000, 2.902110;
  cloud[ 2].getVector3fMap() << 0.412215, 5.000000, 2.618030;
  cloud[ 3].getVector3fMap() << 0.190983, 5.000000, 2.175570;
  cloud[ 4].getVector3fMap() << 0.048944, 5.000000, 1.618030;
  cloud[ 5].getVector3fMap() << 0.000000, 5.000000, 1.000000;
  cloud[ 6].getVector3fMap() << 0.048944, 5.000000, 0.381966;
  cloud[ 7].getVector3fMap() << 0.190983, 5.000000, -0.175571;
  cloud[ 8].getVector3fMap() << 0.412215, 5.000000, -0.618034;
  cloud[ 9].getVector3fMap() << 0.690983, 5.000000, -0.902113;
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
  SampleConsensusModelEllipse3DPtr model( new SampleConsensusModelEllipse3D<PointXYZ>(cloud.makeShared()));

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

int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}

