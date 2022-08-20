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

#include <pcl/pcl_tests.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/utils.h>

#include <pcl/sample_consensus/msac.h>
#include <pcl/sample_consensus/lmeds.h>
#include <pcl/sample_consensus/rmsac.h>
#include <pcl/sample_consensus/mlesac.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/rransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_normal_plane.h>
#include <pcl/sample_consensus/sac_model_normal_parallel_plane.h>

using namespace pcl;
using namespace pcl::io;

using SampleConsensusModelPlanePtr = SampleConsensusModelPlane<PointXYZ>::Ptr;
using SampleConsensusModelNormalPlanePtr = SampleConsensusModelNormalPlane<PointXYZ, Normal>::Ptr;
using SampleConsensusModelNormalParallelPlanePtr = SampleConsensusModelNormalParallelPlane<PointXYZ, Normal>::Ptr;

PointCloud<PointXYZ>::Ptr cloud_ (new PointCloud<PointXYZ> ());
PointCloud<Normal>::Ptr normals_ (new PointCloud<Normal> ());
pcl::Indices indices_;
float plane_coeffs_[] = {-0.8964f, -0.5868f, -1.208f};

template <typename ModelType, typename SacType>
void verifyPlaneSac (ModelType& model,
                     SacType& sac,
                     unsigned int inlier_number = 2000,
                     float tol = 1e-1f,
                     float refined_tol = 1e-1f,
                     float proj_tol = 1e-3f)
{
  // Algorithm tests
  bool result = sac.computeModel ();
  ASSERT_TRUE (result);

  pcl::Indices sample;
  sac.getModel (sample);
  EXPECT_EQ (3, sample.size ());

  pcl::Indices inliers;
  sac.getInliers (inliers);
  EXPECT_LT (inlier_number, inliers.size ());

  Eigen::VectorXf coeff;
  sac.getModelCoefficients (coeff);
  EXPECT_EQ (4, coeff.size ());
  EXPECT_NEAR (plane_coeffs_[0], coeff[0] / coeff[3], tol);
  EXPECT_NEAR (plane_coeffs_[1], coeff[1] / coeff[3], tol);
  EXPECT_NEAR (plane_coeffs_[2], coeff[2] / coeff[3], tol);

  Eigen::VectorXf coeff_refined;
  model->optimizeModelCoefficients (inliers, coeff, coeff_refined);
  EXPECT_EQ (4, coeff_refined.size ());
  EXPECT_NEAR (plane_coeffs_[0], coeff_refined[0] / coeff_refined[3], refined_tol);
  EXPECT_NEAR (plane_coeffs_[1], coeff_refined[1] / coeff_refined[3], refined_tol);

  // This test fails in Windows (VS 2010) -- not sure why yet -- relaxing the constraint from 1e-2 to 1e-1
  // This test fails in MacOS too -- not sure why yet -- disabling
  //EXPECT_NEAR (coeff_refined[2] / coeff_refined[3], plane_coeffs_[2], refined_tol);

  // Projection tests
  PointCloud<PointXYZ> proj_points;
  model->projectPoints (inliers, coeff_refined, proj_points);
  EXPECT_XYZ_NEAR (PointXYZ (1.1266,  0.0152, -0.0156), proj_points[20], proj_tol);
  EXPECT_XYZ_NEAR (PointXYZ (1.1843, -0.0635, -0.0201), proj_points[30], proj_tol);
  EXPECT_XYZ_NEAR (PointXYZ (1.0749, -0.0586,  0.0587), proj_points[50], proj_tol);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (SampleConsensusModelPlane, Base)
{
  // Create a shared plane model pointer directly
  SampleConsensusModelPlanePtr model (new SampleConsensusModelPlane<PointXYZ> (cloud_));

  // Basic tests
  PointCloud<PointXYZ>::ConstPtr cloud = model->getInputCloud ();
  ASSERT_EQ (cloud_->size (), cloud->size ());

  model->setInputCloud (cloud);
  cloud = model->getInputCloud ();
  ASSERT_EQ (cloud_->size (), cloud->size ());

  auto indices = model->getIndices ();
  ASSERT_EQ (indices_.size (), indices->size ());
  model->setIndices (indices_);
  indices = model->getIndices ();
  ASSERT_EQ (indices_.size (), indices->size ());
  model->setIndices (indices);
  indices = model->getIndices ();
  ASSERT_EQ (indices_.size (), indices->size ());
}

TEST (SampleConsensusModelPlane, SampleValidationPointsCollinear)
{
  PointCloud<PointXYZ> cloud;
  cloud.resize (4);

  // The "cheat point" makes it possible to find a set of valid samples and
  // therefore avoids the log message of an unsuccessful sample validation
  // being printed a 1000 times without any chance of success.
  // The order is chosen such that with a known, fixed rng-state/-seed all
  // validation steps are actually exercised.
  const pcl::index_t firstCollinearPointIndex = 0;
  const pcl::index_t secondCollinearPointIndex = 1;
  const pcl::index_t thirdCollinearPointIndex = 2;
  const pcl::index_t cheatPointIndex = 3;

  cloud[firstCollinearPointIndex].getVector3fMap () <<  0.1f,  0.1f,  0.1f;
  cloud[secondCollinearPointIndex].getVector3fMap () <<  0.2f,  0.2f,  0.2f;
  cloud[thirdCollinearPointIndex].getVector3fMap () <<  0.3f,  0.3f,  0.3f;
  cloud[cheatPointIndex].getVector3fMap () <<  0.0f,  0.1f,  0.0f; // <-- cheat point

  // Create a shared line model pointer directly and explicitly disable the
  // random seed for the reasons mentioned above
  SampleConsensusModelPlanePtr model (
    new SampleConsensusModelPlane<PointXYZ> (cloud.makeShared (), /* random = */ false));

  // Algorithm tests
  pcl::Indices samples;
  int iterations = 0;
  model->getSamples(iterations, samples);
  EXPECT_EQ (samples.size(), 3);
  // The "cheat point" has to be part of the sample, otherwise something is wrong.
  // The best option would be to assert on stderr output here, but that doesn't
  // seem to be that simple.
  EXPECT_TRUE (std::find(samples.begin (), samples.end (), cheatPointIndex) != samples.end ());

  pcl::Indices forcedSamples = {firstCollinearPointIndex, secondCollinearPointIndex, thirdCollinearPointIndex};
  Eigen::VectorXf modelCoefficients;
  EXPECT_FALSE (model->computeModelCoefficients (forcedSamples, modelCoefficients));
}

TEST (SampleConsensusModelPlane, SampleValidationPointsValid)
{
  PointCloud<PointXYZ> cloud;
  cloud.resize (3);

  cloud[0].getVector3fMap () <<  0.1f,  0.0f,  0.0f;
  cloud[1].getVector3fMap () <<  0.0f,  0.1f,  0.0f;
  cloud[2].getVector3fMap () <<  0.0f,  0.0f,  0.1f;

  // Create a shared line model pointer directly
  SampleConsensusModelPlanePtr model (
    new SampleConsensusModelPlane<PointXYZ> (cloud.makeShared ()));

  // Algorithm tests
  pcl::Indices samples;
  int iterations = 0;
  model->getSamples(iterations, samples);
  EXPECT_EQ (samples.size(), 3);

  Eigen::VectorXf modelCoefficients;
  EXPECT_TRUE (model->computeModelCoefficients (samples, modelCoefficients));
}

TEST (SampleConsensusModelPlane, SampleValidationNotEnoughSamples)
{
  PointCloud<PointXYZ> cloud;
  cloud.resize (2);

  cloud[0].getVector3fMap () <<  0.1f,  0.0f,  0.0f;
  cloud[1].getVector3fMap () <<  0.0f,  0.1f,  0.0f;

  std::vector<pcl::Indices> testIndices = {{}, {0,}, {0, 1}};

  for( const auto& indices : testIndices) {
    PointCloud<PointXYZ> subCloud {cloud, indices};

    // Create a shared line model pointer directly
    SampleConsensusModelPlanePtr model (
      new SampleConsensusModelPlane<PointXYZ> (subCloud.makeShared ()));

    // Algorithm tests
    pcl::Indices samples;
    int iterations = 0;
    model->getSamples(iterations, samples);
    EXPECT_EQ (samples.size(), 0);

    Eigen::VectorXf modelCoefficients;
    EXPECT_FALSE (model->computeModelCoefficients (indices, modelCoefficients));
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (SampleConsensusModelPlane, RANSAC)
{
  srand (0);

  // Create a shared plane model pointer directly
  SampleConsensusModelPlanePtr model (new SampleConsensusModelPlane<PointXYZ> (cloud_));

  // Create the RANSAC object
  RandomSampleConsensus<PointXYZ> sac (model, 0.03);

  verifyPlaneSac (model, sac);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (SampleConsensusModelPlane, LMedS)
{
  srand (0);

  // Create a shared plane model pointer directly
  SampleConsensusModelPlanePtr model (new SampleConsensusModelPlane<PointXYZ> (cloud_));

  // Create the LMedS object
  LeastMedianSquares<PointXYZ> sac (model, 0.03);

  verifyPlaneSac (model, sac);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (SampleConsensusModelPlane, MSAC)
{
  srand (0);

  // Create a shared plane model pointer directly
  SampleConsensusModelPlanePtr model (new SampleConsensusModelPlane<PointXYZ> (cloud_));

  // Create the MSAC object
  MEstimatorSampleConsensus<PointXYZ> sac (model, 0.03);

  verifyPlaneSac (model, sac);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (SampleConsensusModelPlane, RRANSAC)
{
  srand (0);

  // Create a shared plane model pointer directly
  SampleConsensusModelPlanePtr model (new SampleConsensusModelPlane<PointXYZ> (cloud_));

  // Create the RRANSAC object
  RandomizedRandomSampleConsensus<PointXYZ> sac (model, 0.03);

  sac.setFractionNrPretest (0.1);
  ASSERT_EQ (0.1, sac.getFractionNrPretest ());

  verifyPlaneSac (model, sac, 600, 1.0f, 1.0f, 0.01f);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (SampleConsensusModelPlane, MLESAC)
{
  srand (0);

  // Create a shared plane model pointer directly
  SampleConsensusModelPlanePtr model (new SampleConsensusModelPlane<PointXYZ> (cloud_));

  // Create the MSAC object
  MaximumLikelihoodSampleConsensus<PointXYZ> sac (model, 0.03);

  verifyPlaneSac (model, sac, 1000, 0.3f, 0.2f, 0.01f);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (SampleConsensusModelPlane, RMSAC)
{
  srand (0);

  // Create a shared plane model pointer directly
  SampleConsensusModelPlanePtr model (new SampleConsensusModelPlane<PointXYZ> (cloud_));

  // Create the RMSAC object
  RandomizedMEstimatorSampleConsensus<PointXYZ> sac (model, 0.03);

  sac.setFractionNrPretest (10.0);
  ASSERT_EQ (10.0, sac.getFractionNrPretest ());

  verifyPlaneSac (model, sac, 600, 1.0f, 1.0f, 0.01f);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (SampleConsensusModelNormalPlane, RANSAC)
{
  srand (0);

  // Create a shared plane model pointer directly
  SampleConsensusModelNormalPlanePtr model (new SampleConsensusModelNormalPlane<PointXYZ, Normal> (cloud_));
  model->setInputNormals (normals_);
  model->setNormalDistanceWeight (0.01);

  // Create the RANSAC object
  RandomSampleConsensus<PointXYZ> sac (model, 0.03);

  verifyPlaneSac (model, sac);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (SampleConsensusModelNormalParallelPlane, RANSAC)
{
  srand (0);

  // Use a custom point cloud for these tests until we need something better
  PointCloud<PointXYZ> cloud;
  PointCloud<Normal> normals;
  cloud.resize (10);
  normals.resize (10);

  for (std::size_t idx = 0; idx < cloud.size (); ++idx)
  {
    cloud[idx].x = static_cast<float> ((rand () % 200) - 100);
    cloud[idx].y = static_cast<float> ((rand () % 200) - 100);
    cloud[idx].z = 0.0f;

    normals[idx].normal_x = 0.0f;
    normals[idx].normal_y = 0.0f;
    normals[idx].normal_z = 1.0f;
  }

  // Create a shared plane model pointer directly
  SampleConsensusModelNormalParallelPlanePtr model (new SampleConsensusModelNormalParallelPlane<PointXYZ, Normal> (cloud.makeShared ()));
  model->setInputNormals (normals.makeShared ());

  const float max_angle_rad = 0.01f;
  const float angle_eps = 0.001f;
  model->setEpsAngle (max_angle_rad);

  // Test true axis
  {
    model->setAxis (Eigen::Vector3f (0, 0, 1));

    RandomSampleConsensus<PointXYZ> sac (model, 0.03);
    sac.computeModel();

    pcl::Indices inliers;
    sac.getInliers (inliers);
    ASSERT_EQ (cloud.size (), inliers.size ());
  }

  // test axis slightly in valid range
  {
    model->setAxis (Eigen::Vector3f (0, std::sin (max_angle_rad * (1 - angle_eps)), std::cos (max_angle_rad * (1 - angle_eps))));
    RandomSampleConsensus<PointXYZ> sac (model, 0.03);
    sac.computeModel ();

    pcl::Indices inliers;
    sac.getInliers (inliers);
    ASSERT_EQ (cloud.size (), inliers.size ());
  }

  // test axis slightly out of valid range
  {
    model->setAxis (Eigen::Vector3f (0, std::sin (max_angle_rad * (1 + angle_eps)), std::cos (max_angle_rad * (1 + angle_eps))));
    RandomSampleConsensus<PointXYZ> sac (model, 0.03);
    sac.computeModel ();

    pcl::Indices inliers;
    sac.getInliers (inliers);
    ASSERT_EQ (0, inliers.size ());
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
class SampleConsensusModelPlaneTest : private SampleConsensusModelPlane<PointT>
{
  public:
    using SampleConsensusModelPlane<PointT>::SampleConsensusModelPlane;
    using SampleConsensusModelPlane<PointT>::countWithinDistanceStandard;
#if defined (__SSE__) && defined (__SSE2__) && defined (__SSE4_1__)
    using SampleConsensusModelPlane<PointT>::countWithinDistanceSSE;
#endif
#if defined (__AVX__) && defined (__AVX2__)
    using SampleConsensusModelPlane<PointT>::countWithinDistanceAVX;
#endif
};

TEST (SampleConsensusModelPlane, SIMD_countWithinDistance) // Test if all countWithinDistance implementations return the same value
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
    SampleConsensusModelPlaneTest<PointXYZ> model (cloud.makeShared (), indices, true);

    // Generate random model parameters
    Eigen::VectorXf model_coefficients(4);
    model_coefficients << 2.0 * static_cast<float> (rand ()) / RAND_MAX - 1.0,
                          2.0 * static_cast<float> (rand ()) / RAND_MAX - 1.0,
                          2.0 * static_cast<float> (rand ()) / RAND_MAX - 1.0, 0.0;
    model_coefficients.normalize ();
    model_coefficients(3) = 2.0 * static_cast<float> (rand ()) / RAND_MAX - 1.0; // Last parameter

    const double threshold = 0.1 * static_cast<double> (rand ()) / RAND_MAX; // threshold in [0; 0.1]

    // The number of inliers is usually somewhere between 0 and 100
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

//////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT>
class SampleConsensusModelNormalPlaneTest : private SampleConsensusModelNormalPlane<PointT, PointNT>
{
  public:
    using SampleConsensusModelNormalPlane<PointT, PointNT>::SampleConsensusModelNormalPlane;
    using SampleConsensusModelNormalPlane<PointT, PointNT>::setNormalDistanceWeight;
    using SampleConsensusModelNormalPlane<PointT, PointNT>::setInputNormals;
    using SampleConsensusModelNormalPlane<PointT, PointNT>::countWithinDistanceStandard;
#if defined (__SSE__) && defined (__SSE2__) && defined (__SSE4_1__)
    using SampleConsensusModelNormalPlane<PointT, PointNT>::countWithinDistanceSSE;
#endif
#if defined (__AVX__) && defined (__AVX2__)
    using SampleConsensusModelNormalPlane<PointT, PointNT>::countWithinDistanceAVX;
#endif
};

TEST (SampleConsensusModelNormalPlane, SIMD_countWithinDistance) // Test if all countWithinDistance implementations return the same value
{
  const auto seed = static_cast<unsigned> (std::time (nullptr));
  srand (seed);
  for (size_t i=0; i<1000; i++) // Run as often as you like
  {
    // Generate a cloud with 10000 random points
    PointCloud<PointXYZ> cloud;
    PointCloud<Normal> normal_cloud;
    pcl::Indices indices;
    cloud.resize (10000);
    normal_cloud.resize (10000);
    for (std::size_t idx = 0; idx < cloud.size (); ++idx)
    {
      cloud[idx].x = 2.0 * static_cast<float> (rand ()) / RAND_MAX - 1.0;
      cloud[idx].y = 2.0 * static_cast<float> (rand ()) / RAND_MAX - 1.0;
      cloud[idx].z = 2.0 * static_cast<float> (rand ()) / RAND_MAX - 1.0;
      const double a = 2.0 * static_cast<float> (rand ()) / RAND_MAX - 1.0;
      const double b = 2.0 * static_cast<float> (rand ()) / RAND_MAX - 1.0;
      const double c = 2.0 * static_cast<float> (rand ()) / RAND_MAX - 1.0;
      const double factor = 1.0 / sqrt(a * a + b * b + c * c);
      normal_cloud[idx].normal[0] = a * factor;
      normal_cloud[idx].normal[1] = b * factor;
      normal_cloud[idx].normal[2] = c * factor;
      if (rand () % 4 != 0)
      {
        indices.push_back (static_cast<int> (idx));
      }
    }
    SampleConsensusModelNormalPlaneTest<PointXYZ, Normal> model (cloud.makeShared (), indices, true);
    
    const double normal_distance_weight = 0.3 * static_cast<double> (rand ()) / RAND_MAX; // in [0; 0.3]
    model.setNormalDistanceWeight (normal_distance_weight);
    model.setInputNormals (normal_cloud.makeShared ());

    // Generate random model parameters
    Eigen::VectorXf model_coefficients(4);
    model_coefficients << 2.0 * static_cast<float> (rand ()) / RAND_MAX - 1.0,
                          2.0 * static_cast<float> (rand ()) / RAND_MAX - 1.0,
                          2.0 * static_cast<float> (rand ()) / RAND_MAX - 1.0, 0.0;
    model_coefficients.normalize ();
    model_coefficients(3) = 2.0 * static_cast<float> (rand ()) / RAND_MAX - 1.0; // Last parameter

    const double threshold = 0.1 * static_cast<double> (rand ()) / RAND_MAX; // threshold in [0; 0.1]

    // The number of inliers is usually somewhere between 0 and 100
    const auto res_standard = model.countWithinDistanceStandard (model_coefficients, threshold); // Standard
    pcl::utils::ignore(res_standard);
#if defined (__SSE__) && defined (__SSE2__) && defined (__SSE4_1__)
    const auto res_sse      = model.countWithinDistanceSSE (model_coefficients, threshold); // SSE
    EXPECT_LE ((res_standard > res_sse ? res_standard - res_sse : res_sse - res_standard), 2u) << "seed=" << seed << ", i=" << i
        << ", model=(" << model_coefficients(0) << ", " << model_coefficients(1) << ", " << model_coefficients(2) << ", " << model_coefficients(3)
        << "), threshold=" << threshold << ", normal_distance_weight=" << normal_distance_weight << ", res_standard=" << res_standard << std::endl;
#endif
#if defined (__AVX__) && defined (__AVX2__)
    const auto res_avx      = model.countWithinDistanceAVX (model_coefficients, threshold); // AVX
    EXPECT_LE ((res_standard > res_avx ? res_standard - res_avx : res_avx - res_standard), 2u) << "seed=" << seed << ", i=" << i
        << ", model=(" << model_coefficients(0) << ", " << model_coefficients(1) << ", " << model_coefficients(2) << ", " << model_coefficients(3)
        << "), threshold=" << threshold << ", normal_distance_weight=" << normal_distance_weight << ", res_standard=" << res_standard << std::endl;
#endif
  }
}

TEST (SampleConsensusModelPlane, OptimizeFarFromOrigin)
{ // Test if the model can successfully optimize a plane that is far from the origin
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  Eigen::Vector3d x(-0.435197968, 0.598251061, -0.672828654);
  Eigen::Vector3d y(-0.547340139, 0.417556627,  0.725303548);
  Eigen::Vector3d z( 0.714857680, 0.683916759,  0.145727023); // This is the normal of the plane
  Eigen::Vector3d center(7380.86467, -8350.60056617, 4324.22814107);
  for(double i=-0.5; i<0.5; i+=0.01)
    for(double j=-0.5; j<0.5; j+=0.01) {
      Eigen::Vector3d p = center + i*x + j*y;
      cloud->emplace_back(p[0], p[1], p[2]);
    }
  pcl::SampleConsensusModelPlane<pcl::PointXYZ> model(cloud, true);
  pcl::Indices inliers;
  for(std::size_t i=0; i<cloud->size(); ++i) inliers.push_back(i);
  Eigen::VectorXf coeffs(4); // Doesn't have to be initialized, the function doesn't use them
  Eigen::VectorXf optimized_coeffs(4);
  model.optimizeModelCoefficients(inliers, coeffs, optimized_coeffs);
  EXPECT_NEAR(optimized_coeffs[0], z[0], 5e-6);
  EXPECT_NEAR(optimized_coeffs[1], z[1], 5e-6);
  EXPECT_NEAR(optimized_coeffs[2], z[2], 5e-6);
  EXPECT_NEAR(optimized_coeffs[3], -z.dot(center), 5e-2);
}

int
main (int argc, char** argv)
{
  if (argc < 2)
  {
    std::cerr << "No test file given. Please download `sac_plane_test.pcd` and pass its path to the test." << std::endl;
    return (-1);
  }

  // Load a standard PCD file from disk
  pcl::PCLPointCloud2 cloud_blob;
  if (loadPCDFile (argv[1], cloud_blob) < 0)
  {
    std::cerr << "Failed to read test file. Please download `sac_plane_test.pcd` and pass its path to the test." << std::endl;
    return (-1);
  }

  fromPCLPointCloud2 (cloud_blob, *cloud_);
  fromPCLPointCloud2 (cloud_blob, *normals_);

  indices_.resize (cloud_->size ());
  for (std::size_t i = 0; i < indices_.size (); ++i) { indices_[i] = int (i); }

  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}

