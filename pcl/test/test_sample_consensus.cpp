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
 * $Id: test_sample_consensus.cpp 35810 2011-02-08 00:03:46Z rusu $
 *
 */
/** \author Radu Bogdan Rusu */

#include <gtest/gtest.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/sample_consensus/sac.h>
#include <pcl/sample_consensus/lmeds.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/rransac.h>
#include <pcl/sample_consensus/msac.h>
#include <pcl/sample_consensus/rmsac.h>
#include <pcl/sample_consensus/mlesac.h>
#include <pcl/sample_consensus/sac_model.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/sample_consensus/sac_model_circle.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/sac_model_normal_plane.h>
#include <pcl/sample_consensus/sac_model_parallel_plane.h>

using namespace pcl;
using namespace pcl::io;
using namespace std;

typedef SampleConsensusModel<PointXYZ>::Ptr SampleConsensusModelPtr;
typedef SampleConsensusModelPlane<PointXYZ>::Ptr SampleConsensusModelPlanePtr;
typedef SampleConsensusModelSphere<PointXYZ>::Ptr SampleConsensusModelSpherePtr;
typedef SampleConsensusModelCylinder<PointXYZ, Normal>::Ptr SampleConsensusModelCylinderPtr;
typedef SampleConsensusModelCircle2D<PointXYZ>::Ptr SampleConsensusModelCircle2DPtr;
typedef SampleConsensusModelLine<PointXYZ>::Ptr SampleConsensusModelLinePtr;
typedef SampleConsensusModelNormalPlane<PointXYZ, Normal>::Ptr SampleConsensusModelNormalPlanePtr;
typedef SampleConsensusModelParallelPlane<PointXYZ>::Ptr SampleConsensusModelParallelPlanePtr;

PointCloud<PointXYZ>::Ptr cloud_ (new PointCloud<PointXYZ> ());
PointCloud<Normal>::Ptr normals_ (new PointCloud<Normal> ());
vector<int> indices_;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (SampleConsensusModelPlane, Base)
{
  // Create a shared plane model pointer directly
  SampleConsensusModelPlanePtr model (new SampleConsensusModelPlane<PointXYZ> (cloud_));

  // Basic tests
  PointCloud<PointXYZ>::ConstPtr cloud = model->getInputCloud ();
  ASSERT_EQ (cloud->points.size (), cloud_->points.size ());

  model->setInputCloud (cloud);
  cloud = model->getInputCloud ();
  ASSERT_EQ (cloud->points.size (), cloud_->points.size ());

  boost::shared_ptr<vector<int> > indices = model->getIndices ();
  ASSERT_EQ (indices->size (), indices_.size ());
  model->setIndices (indices_);
  indices = model->getIndices ();
  ASSERT_EQ (indices->size (), indices_.size ());
  model->setIndices (indices);
  indices = model->getIndices ();
  ASSERT_EQ (indices->size (), indices_.size ());
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (RANSAC, Base)
{
  // Create a shared plane model pointer directly
  SampleConsensusModelPlanePtr model (new SampleConsensusModelPlane<PointXYZ> (cloud_));

  // Create the RANSAC object
  RandomSampleConsensus<PointXYZ> sac (model, 0.03);

  // Basic tests
  ASSERT_EQ (sac.getDistanceThreshold (), 0.03);
  sac.setDistanceThreshold (0.03);
  ASSERT_EQ (sac.getDistanceThreshold (), 0.03);

  sac.setProbability (0.99);
  ASSERT_EQ (sac.getProbability (), 0.99);

  sac.setMaxIterations (10000);
  ASSERT_EQ (sac.getMaxIterations (), 10000);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (RANSAC, SampleConsensusModelPlane)
{
  srand (0);
  // Create a shared plane model pointer directly
  SampleConsensusModelPlanePtr model (new SampleConsensusModelPlane<PointXYZ> (cloud_));

  // Create the RANSAC object
  RandomSampleConsensus<PointXYZ> sac (model, 0.03);

  // Algorithm tests
  bool result = sac.computeModel ();
  ASSERT_EQ (result, true);

  std::vector<int> sample;
  sac.getModel (sample);
  EXPECT_EQ ((int)sample.size (), 3);
  EXPECT_EQ (sample[0], 1818);
  EXPECT_EQ (sample[1], 1567);
  EXPECT_EQ (sample[2], 2064);

  std::vector<int> inliers;
  sac.getInliers (inliers);
  EXPECT_EQ ((int)inliers.size (), 2525);

  Eigen::VectorXf coeff;
  sac.getModelCoefficients (coeff);
  EXPECT_EQ ((int)coeff.size (), 4);
  EXPECT_NEAR (coeff[0],  0.5590537786, 1e-4);
  EXPECT_NEAR (coeff[1],  0.3575230539, 1e-4);
  EXPECT_NEAR (coeff[2],  0.7480883598, 1e-4);
  EXPECT_NEAR (coeff[3], -0.622878551,  1e-4);

  Eigen::VectorXf coeff_refined;
  model->optimizeModelCoefficients (inliers, coeff, coeff_refined);
  EXPECT_EQ ((int)coeff_refined.size (), 4);
  EXPECT_NEAR (coeff_refined[0],  0.5554245114, 1e-4);
  EXPECT_NEAR (coeff_refined[1],  0.3644647598, 1e-4);
  EXPECT_NEAR (coeff_refined[2],  0.747441709,  1e-4);
  EXPECT_NEAR (coeff_refined[3], -0.619598031,  1e-4);

  // Projection tests
  PointCloud<PointXYZ> proj_points;
  model->projectPoints (inliers, coeff_refined, proj_points);
  EXPECT_NEAR (proj_points.points[20].x,  1.12661,   1e-4);
  EXPECT_NEAR (proj_points.points[20].y,  0.0152829, 1e-4);
  EXPECT_NEAR (proj_points.points[20].z, -0.0156815, 1e-4);

  EXPECT_NEAR (proj_points.points[30].x,  1.18438,   1e-4);
  EXPECT_NEAR (proj_points.points[30].y, -0.0635465, 1e-4);
  EXPECT_NEAR (proj_points.points[30].z, -0.0201715, 1e-4);

  EXPECT_NEAR (proj_points.points[50].x,  1.07499,   1e-4);
  EXPECT_NEAR (proj_points.points[50].y, -0.0586441, 1e-4);
  EXPECT_NEAR (proj_points.points[50].z,  0.0587273, 1e-4);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (LMedS, SampleConsensusModelPlane)
{
  srand (0);
  // Create a shared plane model pointer directly
  SampleConsensusModelPlanePtr model (new SampleConsensusModelPlane<PointXYZ> (cloud_));

  // Create the LMedS object
  LeastMedianSquares<PointXYZ> sac (model, 0.03);

  // Algorithm tests
  bool result = sac.computeModel ();
  ASSERT_EQ (result, true);

  std::vector<int> sample;
  sac.getModel (sample);
  EXPECT_EQ ((int)sample.size (), 3);
  EXPECT_EQ (sample[0], 338);
  EXPECT_EQ (sample[1], 413);
  EXPECT_EQ (sample[2], 1626);

  std::vector<int> inliers;
  sac.getInliers (inliers);
  EXPECT_EQ ((int)inliers.size (), 2490);

  Eigen::VectorXf coeff;
  sac.getModelCoefficients (coeff);
  EXPECT_EQ ((int)coeff.size (), 4);
  EXPECT_NEAR (coeff[0], -0.5413796306, 1e-4);
  EXPECT_NEAR (coeff[1], -0.3658693433, 1e-4);
  EXPECT_NEAR (coeff[2], -0.756999135,  1e-4);
  EXPECT_NEAR (coeff[3],  0.606127798,  1e-4);

  Eigen::VectorXf coeff_refined;
  model->optimizeModelCoefficients (inliers, coeff, coeff_refined);
  EXPECT_EQ ((int)coeff_refined.size (), 4);
  EXPECT_NEAR (coeff_refined[0],  0.5517195463, 1e-4);
  EXPECT_NEAR (coeff_refined[1],  0.3651787937, 1e-4);
  EXPECT_NEAR (coeff_refined[2],  0.74983340,   1e-4);
  EXPECT_NEAR (coeff_refined[3], -0.6159575582, 1e-4);

  // Projection tests
  PointCloud<PointXYZ> proj_points;
  model->projectPoints (inliers, coeff_refined, proj_points);

  EXPECT_NEAR (proj_points.points[20].x,  1.12694,   1e-4);
  EXPECT_NEAR (proj_points.points[20].y,  0.0154845, 1e-4);
  EXPECT_NEAR (proj_points.points[20].z, -0.0152713, 1e-4);

  EXPECT_NEAR (proj_points.points[30].x,  1.18486,   1e-4);
  EXPECT_NEAR (proj_points.points[30].y, -0.0632408, 1e-4);
  EXPECT_NEAR (proj_points.points[30].z, -0.0195459, 1e-4);

  EXPECT_NEAR (proj_points.points[50].x,  1.07512,   1e-4);
  EXPECT_NEAR (proj_points.points[50].y, -0.0585533, 1e-4);
  EXPECT_NEAR (proj_points.points[50].z,  0.058916,  1e-4);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (MSAC, SampleConsensusModelPlane)
{
  srand (0);
  // Create a shared plane model pointer directly
  SampleConsensusModelPlanePtr model (new SampleConsensusModelPlane<PointXYZ> (cloud_));

  // Create the MSAC object
  MEstimatorSampleConsensus<PointXYZ> sac (model, 0.03);

  // Algorithm tests
  bool result = sac.computeModel ();
  ASSERT_EQ (result, true);

  std::vector<int> sample;
  sac.getModel (sample);
  EXPECT_EQ ((int)sample.size (), 3);
  EXPECT_EQ (sample[0], 464);
  EXPECT_EQ (sample[1], 1992);
  EXPECT_EQ (sample[2], 53);

  std::vector<int> inliers;
  sac.getInliers (inliers);
  EXPECT_EQ ((int)inliers.size (), 2505);

  Eigen::VectorXf coeff;
  sac.getModelCoefficients (coeff);
  EXPECT_EQ ((int)coeff.size (), 4);

  EXPECT_NEAR (coeff[0], -0.5485954285, 1e-4);
  EXPECT_NEAR (coeff[1], -0.3597415686, 1e-4);
  EXPECT_NEAR (coeff[2], -0.7547376752, 1e-4);
  EXPECT_NEAR (coeff[3],  0.6130523086, 1e-4);

  Eigen::VectorXf coeff_refined;
  model->optimizeModelCoefficients (inliers, coeff, coeff_refined);
  EXPECT_EQ ((int)coeff_refined.size (), 4);
  EXPECT_NEAR (coeff_refined[0],  0.5532695055, 1e-4);
  EXPECT_NEAR (coeff_refined[1],  0.3648152649, 1e-4);
  EXPECT_NEAR (coeff_refined[2],  0.7488676906, 1e-4);
  EXPECT_NEAR (coeff_refined[3], -0.617486834,  1e-4);

  // Projection tests
  PointCloud<PointXYZ> proj_points;
  model->projectPoints (inliers, coeff_refined, proj_points);

  EXPECT_NEAR (proj_points.points[20].x,  1.12681,   1e-4);
  EXPECT_NEAR (proj_points.points[20].y,  0.0154031, 1e-4);
  EXPECT_NEAR (proj_points.points[20].z, -0.0154375, 1e-4);

  EXPECT_NEAR (proj_points.points[30].x,  1.18466,   1e-4);
  EXPECT_NEAR (proj_points.points[30].y, -0.0633676, 1e-4);
  EXPECT_NEAR (proj_points.points[30].z, -0.0198059, 1e-4);

  EXPECT_NEAR (proj_points.points[50].x,  1.07506,   1e-4);
  EXPECT_NEAR (proj_points.points[50].y, -0.0585913, 1e-4);
  EXPECT_NEAR (proj_points.points[50].z,  0.0588374, 1e-4);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (RRANSAC, SampleConsensusModelPlane)
{
  srand (0);
  // Create a shared plane model pointer directly
  SampleConsensusModelPlanePtr model (new SampleConsensusModelPlane<PointXYZ> (cloud_));

  // Create the RRANSAC object
  RandomizedRandomSampleConsensus<PointXYZ> sac (model, 0.03);

  sac.setFractionNrPretest (10.0);
  ASSERT_EQ (sac.getFractionNrPretest (), 10.0);

  // Algorithm tests
  bool result = sac.computeModel ();
  ASSERT_EQ (result, true);

  std::vector<int> sample;
  sac.getModel (sample);
  EXPECT_EQ ((int)sample.size (), 3);
  EXPECT_EQ (sample[0], 2758);
  EXPECT_EQ (sample[1], 1294);
  EXPECT_EQ (sample[2], 2570);

  std::vector<int> inliers;
  sac.getInliers (inliers);
  EXPECT_EQ ((int)inliers.size (), 2482);

  Eigen::VectorXf coeff;
  sac.getModelCoefficients (coeff);
  EXPECT_EQ ((int)coeff.size (), 4);
  EXPECT_NEAR (coeff[0], -0.5901594758, 1e-4);
  EXPECT_NEAR (coeff[1], -0.3539851904, 1e-4);
  EXPECT_NEAR (coeff[2], -0.725538671,  1e-4);
  EXPECT_NEAR (coeff[3],  0.6641977429, 1e-4);

  Eigen::VectorXf coeff_refined;
  model->optimizeModelCoefficients (inliers, coeff, coeff_refined);
  EXPECT_EQ ((int)coeff_refined.size (), 4);
  EXPECT_NEAR (fabs (coeff_refined[0]), 0.5598492622, 1e-4);
  EXPECT_NEAR (fabs (coeff_refined[1]), 0.3632659912, 1e-4);
  EXPECT_NEAR (fabs (coeff_refined[2]), 0.7447191477, 1e-4);
  EXPECT_NEAR (fabs (coeff_refined[3]), 0.6246083975, 1e-4);

  // Projection tests
  PointCloud<PointXYZ> proj_points;
  model->projectPoints (inliers, coeff_refined, proj_points);

  EXPECT_NEAR (proj_points.points[20].x,  1.1266,    1e-4);
  EXPECT_NEAR (proj_points.points[20].y,  0.0152881, 1e-4);
  EXPECT_NEAR (proj_points.points[20].z, -0.0156696, 1e-4);

  EXPECT_NEAR (proj_points.points[30].x,  1.18417,   1e-4);
  EXPECT_NEAR (proj_points.points[30].y, -0.0636751, 1e-4);
  EXPECT_NEAR (proj_points.points[30].z, -0.0204345, 1e-4);

  EXPECT_NEAR (proj_points.points[50].x,  1.07519,   1e-4);
  EXPECT_NEAR (proj_points.points[50].y, -0.0585223, 1e-4);
  EXPECT_NEAR (proj_points.points[50].z,  0.0589761, 1e-4);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (RMSAC, SampleConsensusModelPlane)
{
  srand (0);
  // Create a shared plane model pointer directly
  SampleConsensusModelPlanePtr model (new SampleConsensusModelPlane<PointXYZ> (cloud_));

  // Create the RMSAC object
  RandomizedMEstimatorSampleConsensus<PointXYZ> sac (model, 0.03);

  sac.setFractionNrPretest (10.0);
  ASSERT_EQ (sac.getFractionNrPretest (), 10.0);

  // Algorithm tests
  bool result = sac.computeModel ();
  ASSERT_EQ (result, true);

  std::vector<int> sample;
  sac.getModel (sample);
  EXPECT_EQ ((int)sample.size (), 3);
  EXPECT_EQ (sample[0], 2758);
  EXPECT_EQ (sample[1], 1294);
  EXPECT_EQ (sample[2], 2570);

  std::vector<int> inliers;
  sac.getInliers (inliers);
  EXPECT_EQ ((int)inliers.size (), 2482);

  Eigen::VectorXf coeff;
  sac.getModelCoefficients (coeff);
  EXPECT_EQ ((int)coeff.size (), 4);
  EXPECT_NEAR (coeff[0], -0.5901594758, 1e-4);
  EXPECT_NEAR (coeff[1], -0.3539851904, 1e-4);
  EXPECT_NEAR (coeff[2], -0.725538671,  1e-4);
  EXPECT_NEAR (coeff[3],  0.6641977429, 1e-4);

  Eigen::VectorXf coeff_refined;
  model->optimizeModelCoefficients (inliers, coeff, coeff_refined);
  EXPECT_EQ ((int)coeff_refined.size (), 4);
  EXPECT_NEAR (fabs (coeff_refined[0]), 0.5598492622, 1e-4);
  EXPECT_NEAR (fabs (coeff_refined[1]), 0.3632659912, 1e-4);
  EXPECT_NEAR (fabs (coeff_refined[2]), 0.7447191477, 1e-4);
  EXPECT_NEAR (fabs (coeff_refined[3]), 0.6246083975, 1e-4);

  // Projection tests
  PointCloud<PointXYZ> proj_points;
  model->projectPoints (inliers, coeff_refined, proj_points);

  EXPECT_NEAR (proj_points.points[20].x,  1.1266,    1e-4);
  EXPECT_NEAR (proj_points.points[20].y,  0.0152881, 1e-4);
  EXPECT_NEAR (proj_points.points[20].z, -0.0156696, 1e-4);

  EXPECT_NEAR (proj_points.points[30].x,  1.18417,   1e-4);
  EXPECT_NEAR (proj_points.points[30].y, -0.0636751, 1e-4);
  EXPECT_NEAR (proj_points.points[30].z, -0.0204345, 1e-4);

  EXPECT_NEAR (proj_points.points[50].x,  1.07519,   1e-4);
  EXPECT_NEAR (proj_points.points[50].y, -0.0585223, 1e-4);
  EXPECT_NEAR (proj_points.points[50].z,  0.0589761, 1e-4);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (MLESAC, SampleConsensusModelPlane)
{
  srand (0);
  // Create a shared plane model pointer directly
  SampleConsensusModelPlanePtr model (new SampleConsensusModelPlane<PointXYZ> (cloud_));

  // Create the MSAC object
  MaximumLikelihoodSampleConsensus<PointXYZ> sac (model, 0.03);

  // Algorithm tests
  bool result = sac.computeModel ();
  ASSERT_EQ (result, true);

  std::vector<int> sample;
  sac.getModel (sample);
  EXPECT_EQ ((int)sample.size (), 3);
  EXPECT_EQ (sample[0], 2758);
  EXPECT_EQ (sample[1], 1294);
  EXPECT_EQ (sample[2], 2570);

  std::vector<int> inliers;
  sac.getInliers (inliers);
  EXPECT_EQ ((int)inliers.size (), 2214);

  Eigen::VectorXf coeff;
  sac.getModelCoefficients (coeff);
  EXPECT_EQ ((int)coeff.size (), 4);

  EXPECT_NEAR (coeff[0], -0.5901594758, 1e-4);
  EXPECT_NEAR (coeff[1], -0.3539851904, 1e-4);
  EXPECT_NEAR (coeff[2], -0.725538671,  1e-4);
  EXPECT_NEAR (coeff[3],  0.664197742,  1e-4);

  Eigen::VectorXf coeff_refined;
  model->optimizeModelCoefficients (inliers, coeff, coeff_refined);
  EXPECT_EQ ((int)coeff_refined.size (), 4);
  EXPECT_NEAR (fabs (coeff_refined[0]), 0.5599190593, 1e-4);
  EXPECT_NEAR (fabs (coeff_refined[1]), 0.3627234101, 1e-4);
  EXPECT_NEAR (fabs (coeff_refined[2]), 0.7449311614, 1e-4);
  EXPECT_NEAR (fabs (coeff_refined[3]), 0.625774502,  1e-4);

  // Projection tests
  PointCloud<PointXYZ> proj_points;
  model->projectPoints (inliers, coeff_refined, proj_points);

  EXPECT_NEAR (proj_points.points[20].x,  1.12721,   1e-4);
  EXPECT_NEAR (proj_points.points[20].y,  0.01568,   1e-4);
  EXPECT_NEAR (proj_points.points[20].z, -0.014851,  1e-4);

  EXPECT_NEAR (proj_points.points[30].x,  1.18476,   1e-4);
  EXPECT_NEAR (proj_points.points[30].y, -0.063291,  1e-4);
  EXPECT_NEAR (proj_points.points[30].z, -0.019650,  1e-4);

  EXPECT_NEAR (proj_points.points[50].x,  1.07578,   1e-4);
  EXPECT_NEAR (proj_points.points[50].y, -0.058144, 1e-4);
  EXPECT_NEAR (proj_points.points[50].z,  0.059756, 1e-4);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (RANSAC, SampleConsensusModelSphere)
{
  srand (0);

  // Use a custom point cloud for these tests until we need something better
  PointCloud<PointXYZ> cloud;
  cloud.points.resize (10);
  cloud.points[0].x = 1.7068; cloud.points[0].y = 1.0684; cloud.points[0].z = 2.2147;
  cloud.points[1].x = 2.4708; cloud.points[1].y = 2.3081; cloud.points[1].z = 1.1736;
  cloud.points[2].x = 2.7609; cloud.points[2].y = 1.9095; cloud.points[2].z = 1.3574;
  cloud.points[3].x = 2.8016; cloud.points[3].y = 1.6704; cloud.points[3].z = 1.5009;
  cloud.points[4].x = 1.8517; cloud.points[4].y = 2.0276; cloud.points[4].z = 1.0112;
  cloud.points[5].x = 1.8726; cloud.points[5].y = 1.3539; cloud.points[5].z = 2.7523;
  cloud.points[6].x = 2.5179; cloud.points[6].y = 2.3218; cloud.points[6].z = 1.2074;
  cloud.points[7].x = 2.4026; cloud.points[7].y = 2.5114; cloud.points[7].z = 2.7588;
  cloud.points[8].x = 2.6999; cloud.points[8].y = 2.5606; cloud.points[8].z = 1.5571;
  cloud.points[9].x = 0;      cloud.points[9].y = 0;      cloud.points[9].z = 0;

  // Create a shared sphere model pointer directly
  SampleConsensusModelSpherePtr model (new SampleConsensusModelSphere<PointXYZ> (cloud.makeShared ()));

  // Create the RANSAC object
  RandomSampleConsensus<PointXYZ> sac (model, 0.03);

  // Algorithm tests
  bool result = sac.computeModel ();
  ASSERT_EQ (result, true);

  std::vector<int> sample;
  sac.getModel (sample);
  EXPECT_EQ ((int)sample.size (), 4);
  EXPECT_EQ (sample[0], 1);
  EXPECT_EQ (sample[1], 3);
  EXPECT_EQ (sample[2], 7);
  EXPECT_EQ (sample[3], 2);

  std::vector<int> inliers;
  sac.getInliers (inliers);
  EXPECT_EQ ((int)inliers.size (), 9);

  Eigen::VectorXf coeff;
  sac.getModelCoefficients (coeff);
  EXPECT_EQ ((int)coeff.size (), 4);
  EXPECT_NEAR (coeff[0], 2.00052,  1e-4);
  EXPECT_NEAR (coeff[1], 1.9997,   1e-4);
  EXPECT_NEAR (coeff[2], 2.00003,  1e-4);
  EXPECT_NEAR (coeff[3], 0.999565, 1e-4);

  Eigen::VectorXf coeff_refined;
  model->optimizeModelCoefficients (inliers, coeff, coeff_refined);
  EXPECT_EQ ((int)coeff_refined.size (), 4);
  EXPECT_NEAR (coeff_refined[0], 2.00023,  1e-3);
  EXPECT_NEAR (coeff_refined[1], 1.99979,  1e-3);
  EXPECT_NEAR (coeff_refined[2], 1.99979,  1e-3);
  EXPECT_NEAR (coeff_refined[3], 0.999888, 1e-3);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (RANSAC, SampleConsensusModelCylinder)
{
  srand (0);

  // Use a custom point cloud for these tests until we need something better
  PointCloud<PointXYZ> cloud;
  PointCloud<Normal> normals;
  cloud.points.resize (20); normals.points.resize (20);

  cloud.points[0].x =  -0.499902; cloud.points[0].y =  2.199701; cloud.points[0].z =  0.000008;
  cloud.points[1].x =  -0.875397; cloud.points[1].y =  2.030177; cloud.points[1].z =  0.050104;
  cloud.points[2].x =  -0.995875; cloud.points[2].y =  1.635973; cloud.points[2].z =  0.099846;
  cloud.points[3].x =  -0.779523; cloud.points[3].y =  1.285527; cloud.points[3].z =  0.149961;
  cloud.points[4].x =  -0.373285; cloud.points[4].y =  1.216488; cloud.points[4].z =  0.199959;
  cloud.points[5].x =  -0.052893; cloud.points[5].y =  1.475973; cloud.points[5].z =  0.250101;
  cloud.points[6].x =  -0.036558; cloud.points[6].y =  1.887591; cloud.points[6].z =  0.299839;
  cloud.points[7].x =  -0.335048; cloud.points[7].y =  2.171994; cloud.points[7].z =  0.350001;
  cloud.points[8].x =  -0.745456; cloud.points[8].y =  2.135528; cloud.points[8].z =  0.400072;
  cloud.points[9].x =  -0.989282; cloud.points[9].y =  1.803311; cloud.points[9].z =  0.449983;
  cloud.points[10].x = -0.900651; cloud.points[10].y = 1.400701; cloud.points[10].z = 0.500126;
  cloud.points[11].x = -0.539658; cloud.points[11].y = 1.201468; cloud.points[11].z = 0.550079;
  cloud.points[12].x = -0.151875; cloud.points[12].y = 1.340951; cloud.points[12].z = 0.599983;
  cloud.points[13].x = -0.000724; cloud.points[13].y = 1.724373; cloud.points[13].z = 0.649882;
  cloud.points[14].x = -0.188573; cloud.points[14].y = 2.090983; cloud.points[14].z = 0.699854;
  cloud.points[15].x = -0.587925; cloud.points[15].y = 2.192257; cloud.points[15].z = 0.749956;
  cloud.points[16].x = -0.927724; cloud.points[16].y = 1.958846; cloud.points[16].z = 0.800008;
  cloud.points[17].x = -0.976888; cloud.points[17].y = 1.549655; cloud.points[17].z = 0.849970;
  cloud.points[18].x = -0.702003; cloud.points[18].y = 1.242707; cloud.points[18].z = 0.899954;
  cloud.points[19].x = -0.289916; cloud.points[19].y = 1.246296; cloud.points[19].z = 0.950075;

  normals.points[0].normal[0] =   0.000098; normals.points[0].normal[1] =   1.000098; normals.points[0].normal[2] =   0.000008;
  normals.points[1].normal[0] =  -0.750891; normals.points[1].normal[1] =   0.660413; normals.points[1].normal[2] =   0.000104;
  normals.points[2].normal[0] =  -0.991765; normals.points[2].normal[1] =  -0.127949; normals.points[2].normal[2] =  -0.000154;
  normals.points[3].normal[0] =  -0.558918; normals.points[3].normal[1] =  -0.829439; normals.points[3].normal[2] =  -0.000039;
  normals.points[4].normal[0] =   0.253627; normals.points[4].normal[1] =  -0.967447; normals.points[4].normal[2] =  -0.000041;
  normals.points[5].normal[0] =   0.894105; normals.points[5].normal[1] =  -0.447965; normals.points[5].normal[2] =   0.000101;
  normals.points[6].normal[0] =   0.926852; normals.points[6].normal[1] =   0.375543; normals.points[6].normal[2] =  -0.000161;
  normals.points[7].normal[0] =   0.329948; normals.points[7].normal[1] =   0.943941; normals.points[7].normal[2] =   0.000001;
  normals.points[8].normal[0] =  -0.490966; normals.points[8].normal[1] =   0.871203; normals.points[8].normal[2] =   0.000072;
  normals.points[9].normal[0] =  -0.978507; normals.points[9].normal[1] =   0.206425; normals.points[9].normal[2] =  -0.000017;
  normals.points[10].normal[0] = -0.801227; normals.points[10].normal[1] = -0.598534; normals.points[10].normal[2] =  0.000126;
  normals.points[11].normal[0] = -0.079447; normals.points[11].normal[1] = -0.996697; normals.points[11].normal[2] =  0.000079;
  normals.points[12].normal[0] =  0.696154; normals.points[12].normal[1] = -0.717889; normals.points[12].normal[2] = -0.000017;
  normals.points[13].normal[0] =  0.998685; normals.points[13].normal[1] =  0.048502; normals.points[13].normal[2] = -0.000118;
  normals.points[14].normal[0] =  0.622933; normals.points[14].normal[1] =  0.782133; normals.points[14].normal[2] = -0.000146;
  normals.points[15].normal[0] = -0.175948; normals.points[15].normal[1] =  0.984480; normals.points[15].normal[2] = -0.000044;
  normals.points[16].normal[0] = -0.855476; normals.points[16].normal[1] =  0.517824; normals.points[16].normal[2] =  0.000008;
  normals.points[17].normal[0] = -0.953769; normals.points[17].normal[1] = -0.300571; normals.points[17].normal[2] = -0.000030;
  normals.points[18].normal[0] = -0.404035; normals.points[18].normal[1] = -0.914700; normals.points[18].normal[2] = -0.000046;
  normals.points[19].normal[0] =  0.420154; normals.points[19].normal[1] = -0.907445; normals.points[19].normal[2] =  0.000075;

  // Create a shared cylinder model pointer directly
  SampleConsensusModelCylinderPtr model (new SampleConsensusModelCylinder<PointXYZ, Normal> (cloud.makeShared ()));
  model->setInputNormals (normals.makeShared ());

  // Create the RANSAC object
  RandomSampleConsensus<PointXYZ> sac (model, 0.03);

  // Algorithm tests
  bool result = sac.computeModel ();
  ASSERT_EQ (result, true);

  std::vector<int> sample;
  sac.getModel (sample);
  EXPECT_EQ ((int)sample.size (), 2);
  EXPECT_EQ (sample[0], 16);
  EXPECT_EQ (sample[1], 7);

  std::vector<int> inliers;
  sac.getInliers (inliers);
  EXPECT_EQ ((int)inliers.size (), 20);

  Eigen::VectorXf coeff;
  sac.getModelCoefficients (coeff);
  EXPECT_EQ ((int)coeff.size (), 7);
  EXPECT_NEAR (coeff[0], -0.500038, 1e-4);
  EXPECT_NEAR (coeff[1],  1.69997,  1e-4);
  EXPECT_NEAR (coeff[6],  0.499934, 1e-4);

  Eigen::VectorXf coeff_refined;
  model->optimizeModelCoefficients (inliers, coeff, coeff_refined);
  EXPECT_EQ ((int)coeff_refined.size (), 7);
  EXPECT_NEAR (coeff_refined[6], 0.499966, 1e-4);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (RANSAC, SampleConsensusModelCircle2D)
{
  srand (0);

  // Use a custom point cloud for these tests until we need something better
  PointCloud<PointXYZ> cloud;
  cloud.points.resize (18);

  cloud.points[0].x = 3.587751;  cloud.points[0].y = -4.190982;  cloud.points[0].z = 0;
  cloud.points[1].x = 3.808883;  cloud.points[1].y = -4.412265;  cloud.points[1].z = 0;
  cloud.points[2].x = 3.587525;  cloud.points[2].y = -5.809143;  cloud.points[2].z = 0;
  cloud.points[3].x = 2.999913;  cloud.points[3].y = -5.999980;  cloud.points[3].z = 0;
  cloud.points[4].x = 2.412224;  cloud.points[4].y = -5.809090;  cloud.points[4].z = 0;
  cloud.points[5].x = 2.191080;  cloud.points[5].y = -5.587682;  cloud.points[5].z = 0;
  cloud.points[6].x = 2.048941;  cloud.points[6].y = -5.309003;  cloud.points[6].z = 0;
  cloud.points[7].x = 2.000397;  cloud.points[7].y = -4.999944;  cloud.points[7].z = 0;
  cloud.points[8].x = 2.999953;  cloud.points[8].y = -6.000056;  cloud.points[8].z = 0;
  cloud.points[9].x = 2.691127;  cloud.points[9].y = -5.951136;  cloud.points[9].z = 0;
  cloud.points[10].x = 2.190892; cloud.points[10].y = -5.587838; cloud.points[10].z = 0;
  cloud.points[11].x = 2.048874; cloud.points[11].y = -5.309052; cloud.points[11].z = 0;
  cloud.points[12].x = 1.999990; cloud.points[12].y = -5.000147; cloud.points[12].z = 0;
  cloud.points[13].x = 2.049026; cloud.points[13].y = -4.690918; cloud.points[13].z = 0;
  cloud.points[14].x = 2.190956; cloud.points[14].y = -4.412162; cloud.points[14].z = 0;
  cloud.points[15].x = 2.412231; cloud.points[15].y = -4.190918; cloud.points[15].z = 0;
  cloud.points[16].x = 2.691027; cloud.points[16].y = -4.049060; cloud.points[16].z = 0;
  cloud.points[17].x = 2;        cloud.points[17].y = -3;        cloud.points[17].z = 0;

  // Create a shared 2d circle model pointer directly
  SampleConsensusModelCircle2DPtr model (new SampleConsensusModelCircle2D<PointXYZ> (cloud.makeShared ()));

  // Create the RANSAC object
  RandomSampleConsensus<PointXYZ> sac (model, 0.03);

  // Algorithm tests
  bool result = sac.computeModel ();
  ASSERT_EQ (result, true);

  std::vector<int> sample;
  sac.getModel (sample);
  EXPECT_EQ ((int)sample.size (), 3);
  EXPECT_EQ (sample[0], 15);
  EXPECT_EQ (sample[1], 7);
  EXPECT_EQ (sample[2], 14);

  std::vector<int> inliers;
  sac.getInliers (inliers);
  EXPECT_EQ ((int)inliers.size (), 17);

  Eigen::VectorXf coeff;
  sac.getModelCoefficients (coeff);
  EXPECT_EQ ((int)coeff.size (), 3);
  EXPECT_NEAR (coeff[0],  2.9988,   1e-4);
  EXPECT_NEAR (coeff[1], -4.99885,  1e-4);
  EXPECT_NEAR (coeff[2],  0.998406, 1e-4);

  Eigen::VectorXf coeff_refined;
  model->optimizeModelCoefficients (inliers, coeff, coeff_refined);
  EXPECT_EQ ((int)coeff_refined.size (), 3);
  EXPECT_NEAR (coeff_refined[0],  2.99999,  1e-4);
  EXPECT_NEAR (coeff_refined[1], -5.00004,  1e-4);
  EXPECT_NEAR (coeff_refined[2],  0.999962, 1e-4);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (RANSAC, SampleConsensusModelLine)
{
  srand (0);

  // Use a custom point cloud for these tests until we need something better
  PointCloud<PointXYZ> cloud;
  cloud.points.resize (10);

  cloud.points[0].x = 1;  cloud.points[0].y = 2;    cloud.points[0].z = 3;
  cloud.points[1].x = 4;  cloud.points[1].y = 5;    cloud.points[1].z = 6;
  cloud.points[2].x = 7;  cloud.points[2].y = 8;    cloud.points[2].z = 9;
  cloud.points[3].x = 10; cloud.points[3].y = 11;   cloud.points[3].z = 12;
  cloud.points[4].x = 13; cloud.points[4].y = 14;   cloud.points[4].z = 15;
  cloud.points[5].x = 16; cloud.points[5].y = 17;   cloud.points[5].z = 18;
  cloud.points[6].x = 19; cloud.points[6].y = 20;   cloud.points[6].z = 21;
  cloud.points[7].x = 22; cloud.points[7].y = 23;   cloud.points[7].z = 24;
  cloud.points[8].x = -5; cloud.points[8].y = 1.57; cloud.points[8].z = 0.75;
  cloud.points[9].x = 4;  cloud.points[9].y = 2;    cloud.points[9].z = 3;

  // Create a shared line model pointer directly
  SampleConsensusModelLinePtr model (new SampleConsensusModelLine<PointXYZ> (cloud.makeShared ()));

  // Create the RANSAC object
  RandomSampleConsensus<PointXYZ> sac (model, 0.001);

  // Algorithm tests
  bool result = sac.computeModel ();
  ASSERT_EQ (result, true);

  std::vector<int> sample;
  sac.getModel (sample);
  EXPECT_EQ ((int)sample.size (), 2);
  EXPECT_EQ (sample[0], 1);
  EXPECT_EQ (sample[1], 3);

  std::vector<int> inliers;
  sac.getInliers (inliers);
  EXPECT_EQ ((int)inliers.size (), 8);

  Eigen::VectorXf coeff;
  sac.getModelCoefficients (coeff);
  EXPECT_EQ ((int)coeff.size (), 6);
  EXPECT_NEAR (coeff[3], 0.57735, 1e-4);
  EXPECT_NEAR (coeff[4], 0.57735, 1e-4);
  EXPECT_NEAR (coeff[5], 0.57735, 1e-4);

  Eigen::VectorXf coeff_refined;
  model->optimizeModelCoefficients (inliers, coeff, coeff_refined);
  EXPECT_EQ ((int)coeff_refined.size (), 6);
  EXPECT_NEAR (coeff_refined[3], 0.57735, 1e-4);
  EXPECT_NEAR (coeff_refined[4], 0.57735, 1e-4);
  EXPECT_NEAR (coeff_refined[5], 0.57735, 1e-4);

  // Projection tests
  PointCloud<PointXYZ> proj_points;
  model->projectPoints (inliers, coeff_refined, proj_points);

  EXPECT_NEAR (proj_points.points[2].x, 7.0, 1e-4);
  EXPECT_NEAR (proj_points.points[2].y, 8.0, 1e-4);
  EXPECT_NEAR (proj_points.points[2].z, 9.0, 1e-4);

  EXPECT_NEAR (proj_points.points[3].x, 10.0, 1e-4);
  EXPECT_NEAR (proj_points.points[3].y, 11.0, 1e-4);
  EXPECT_NEAR (proj_points.points[3].z, 12.0, 1e-4);

  EXPECT_NEAR (proj_points.points[5].x, 16.0, 1e-4);
  EXPECT_NEAR (proj_points.points[5].y, 17.0, 1e-4);
  EXPECT_NEAR (proj_points.points[5].z, 18.0, 1e-4);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (RANSAC, SampleConsensusModelNormalPlane)
{
  srand (0);
  // Create a shared plane model pointer directly
  SampleConsensusModelNormalPlanePtr model (new SampleConsensusModelNormalPlane<PointXYZ, Normal> (cloud_));
  model->setInputNormals (normals_);
  model->setNormalDistanceWeight (0.01);
  // Create the RANSAC object
  RandomSampleConsensus<PointXYZ> sac (model, 0.03);

  // Algorithm tests
  bool result = sac.computeModel ();
  ASSERT_EQ (result, true);

  std::vector<int> sample;
  sac.getModel (sample);
  EXPECT_EQ ((int)sample.size (), 3);
  EXPECT_EQ (sample[0], 1818);
  EXPECT_EQ (sample[1], 1567);
  EXPECT_EQ (sample[2], 2064);

  std::vector<int> inliers;
  sac.getInliers (inliers);
  EXPECT_EQ ((int)inliers.size (), 2440);

  Eigen::VectorXf coeff;
  sac.getModelCoefficients (coeff);
  EXPECT_EQ ((int)coeff.size (), 4);
  EXPECT_NEAR (coeff[0],  0.5590537786, 1e-4);
  EXPECT_NEAR (coeff[1],  0.3575230538, 1e-4);
  EXPECT_NEAR (coeff[2],  0.7480884194, 1e-4);
  EXPECT_NEAR (coeff[3], -0.6228785514, 1e-4);

  Eigen::VectorXf coeff_refined;
  model->optimizeModelCoefficients (inliers, coeff, coeff_refined);
  EXPECT_EQ ((int)coeff_refined.size (), 4);
  EXPECT_NEAR (fabs (coeff_refined[0]), 0.552499, 1e-4);
  EXPECT_NEAR (fabs (coeff_refined[1]), 0.364361, 1e-4);
  EXPECT_NEAR (fabs (coeff_refined[2]), 0.749656, 1e-4);
  EXPECT_NEAR (fabs (coeff_refined[3]), 0.617206, 1e-4);

  // Projection tests
  PointCloud<PointXYZ> proj_points;
  model->projectPoints (inliers, coeff_refined, proj_points);

  EXPECT_NEAR (proj_points.points[20].x,  1.12707,   1e-4);
  EXPECT_NEAR (proj_points.points[20].y,  0.0155766, 1e-4);
  EXPECT_NEAR (proj_points.points[20].z, -0.0149861, 1e-4);

  EXPECT_NEAR (proj_points.points[30].x,  1.185,    1e-4);
  EXPECT_NEAR (proj_points.points[30].y, -0.063224, 1e-4);
  EXPECT_NEAR (proj_points.points[30].z, -0.019343, 1e-4);

  EXPECT_NEAR (proj_points.points[50].x,  1.07528,   1e-4);
  EXPECT_NEAR (proj_points.points[50].y, -0.0584513, 1e-4);
  EXPECT_NEAR (proj_points.points[50].z,  0.0591309, 1e-4);
}

/* ---[ */
int
  main (int argc, char** argv)
{
  // Load a standard PCD file from disk
  sensor_msgs::PointCloud2 cloud_blob;
  loadPCDFile ("./test/sac_plane_test.pcd", cloud_blob);
  fromROSMsg (cloud_blob, *cloud_);

  indices_.resize (cloud_->points.size ());
  for (size_t i = 0; i < indices_.size (); ++i) { indices_[i] = i; }

  // Estimate surface normals
  NormalEstimation<PointXYZ, Normal> n;
  KdTree<PointXYZ>::Ptr tree = boost::make_shared<KdTreeFLANN<PointXYZ> > ();
  tree->setInputCloud (cloud_);
  n.setInputCloud (cloud_);
  n.setIndices (boost::make_shared <vector<int> > (indices_));
  n.setSearchMethod (tree);
  n.setRadiusSearch (0.02);    // Use 2cm radius to estimate the normals
  n.compute (*normals_);

  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
