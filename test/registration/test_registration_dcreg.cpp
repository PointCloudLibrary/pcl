/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012-, Open Perception, Inc.
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

#include <pcl/registration/correspondence_types.h>
#include <pcl/registration/transformation_estimation_point_to_plane_dcreg.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>
#include <pcl/test/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cmath>
#include <cstdint>
#include <string>

namespace {

using PointT = pcl::PointNormal;
using CloudT = pcl::PointCloud<PointT>;
using DCRegEstimator =
    pcl::registration::TransformationEstimationPointToPlaneDCReg<PointT, PointT, float>;

PointT
makePoint(float x, float y, float z, float nx, float ny, float nz)
{
  PointT point;
  point.x = x;
  point.y = y;
  point.z = z;
  point.normal_x = nx;
  point.normal_y = ny;
  point.normal_z = nz;
  return (point);
}

void
finishCloud(CloudT& cloud)
{
  cloud.width = static_cast<std::uint32_t>(cloud.size());
  cloud.height = 1;
  cloud.is_dense = true;
}

pcl::Correspondences
makeIdentityCorrespondences(std::size_t size)
{
  pcl::Correspondences correspondences;
  correspondences.reserve(size);
  for (std::size_t i = 0; i < size; ++i)
    correspondences.emplace_back(static_cast<int>(i), static_cast<int>(i), 0.0f);
  return (correspondences);
}

CloudT
makeTranslatedSource(const CloudT& target, const Eigen::Vector3f& translation)
{
  CloudT source;
  source.reserve(target.size());
  for (const auto& point : target) {
    PointT source_point = point;
    source_point.x -= translation.x();
    source_point.y -= translation.y();
    source_point.z -= translation.z();
    source.push_back(source_point);
  }
  finishCloud(source);
  return (source);
}

CloudT
makeCubeTarget()
{
  CloudT target;
  const float values[2] = {-0.5f, 0.5f};
  for (const float y : values) {
    for (const float z : values) {
      target.push_back(makePoint(-0.5f, y, z, -1.0f, 0.0f, 0.0f));
      target.push_back(makePoint(0.5f, y, z, 1.0f, 0.0f, 0.0f));
    }
  }
  for (const float x : values) {
    for (const float z : values) {
      target.push_back(makePoint(x, -0.5f, z, 0.0f, -1.0f, 0.0f));
      target.push_back(makePoint(x, 0.5f, z, 0.0f, 1.0f, 0.0f));
    }
  }
  for (const float x : values) {
    for (const float y : values) {
      target.push_back(makePoint(x, y, -0.5f, 0.0f, 0.0f, -1.0f));
      target.push_back(makePoint(x, y, 0.5f, 0.0f, 0.0f, 1.0f));
    }
  }
  finishCloud(target);
  return (target);
}

CloudT
makePlaneTarget()
{
  CloudT target;
  for (int ix = -2; ix <= 2; ++ix) {
    for (int iy = -2; iy <= 2; ++iy) {
      target.push_back(makePoint(0.1f * static_cast<float>(ix),
                                 0.1f * static_cast<float>(iy),
                                 0.0f,
                                 0.0f,
                                 0.0f,
                                 1.0f));
    }
  }
  finishCloud(target);
  return (target);
}

CloudT
makeCylinderTarget()
{
  CloudT target;
  constexpr int kAngleSteps = 36;
  constexpr double kPi = 3.14159265358979323846;
  const float heights[3] = {-0.5f, 0.0f, 0.5f};
  for (const float z : heights) {
    for (int i = 0; i < kAngleSteps; ++i) {
      const float angle =
          static_cast<float>(2.0 * kPi * i / static_cast<double>(kAngleSteps));
      const float x = std::cos(angle);
      const float y = std::sin(angle);
      target.push_back(makePoint(x, y, z, x, y, 0.0f));
    }
  }
  finishCloud(target);
  return (target);
}

void
expectFiniteTransform(const Eigen::Matrix4f& transformation)
{
  for (int row = 0; row < 4; ++row)
    for (int col = 0; col < 4; ++col)
      EXPECT_TRUE(std::isfinite(transformation(row, col)));
}

} // namespace

TEST(PCL, TransformationEstimationPointToPlaneDCRegParameters)
{
  DCRegEstimator estimator;
  EXPECT_DOUBLE_EQ(estimator.getDegeneracyConditionThreshold(), 10.0);
  EXPECT_DOUBLE_EQ(estimator.getKappaTarget(), 10.0);
  EXPECT_DOUBLE_EQ(estimator.getPcgTolerance(), 1e-6);
  EXPECT_EQ(estimator.getPcgMaxIterations(), 10);

  estimator.setDegeneracyConditionThreshold(20.0);
  estimator.setKappaTarget(15.0);
  estimator.setPcgTolerance(1e-8);
  estimator.setPcgMaxIterations(25);

  EXPECT_DOUBLE_EQ(estimator.getDegeneracyConditionThreshold(), 20.0);
  EXPECT_DOUBLE_EQ(estimator.getKappaTarget(), 15.0);
  EXPECT_DOUBLE_EQ(estimator.getPcgTolerance(), 1e-8);
  EXPECT_EQ(estimator.getPcgMaxIterations(), 25);
  EXPECT_EQ(estimator.getLastDegeneracyAnalysis().solver_type, "invalid");
}

TEST(PCL, TransformationEstimationPointToPlaneDCRegEmptyCorrespondences)
{
  DCRegEstimator estimator;
  CloudT source;
  CloudT target;
  pcl::Correspondences correspondences;
  Eigen::Matrix4f transformation = Eigen::Matrix4f::Zero();

  estimator.estimateRigidTransformation(
      source, target, correspondences, transformation);

  EXPECT_TRUE(transformation.isApprox(Eigen::Matrix4f::Identity()));
  const auto& analysis = estimator.getLastDegeneracyAnalysis();
  EXPECT_FALSE(analysis.has_correspondences);
  EXPECT_EQ(analysis.solver_type, "no_correspondences");
}

TEST(PCL, TransformationEstimationPointToPlaneDCRegMatchesLLS)
{
  const CloudT target = makeCubeTarget();
  const Eigen::Vector3f translation(0.02f, -0.03f, 0.04f);
  const CloudT source = makeTranslatedSource(target, translation);
  const pcl::Correspondences correspondences =
      makeIdentityCorrespondences(target.size());

  DCRegEstimator estimator;
  pcl::registration::TransformationEstimationPointToPlaneLLS<PointT, PointT, float>
      lls_estimator;
  Eigen::Matrix4f dcreg_transformation;
  Eigen::Matrix4f lls_transformation;

  estimator.estimateRigidTransformation(
      source, target, correspondences, dcreg_transformation);
  lls_estimator.estimateRigidTransformation(
      source, target, correspondences, lls_transformation);

  EXPECT_TRUE(dcreg_transformation.isApprox(lls_transformation, 1e-4f));
  EXPECT_TRUE((dcreg_transformation.block<3, 1>(0, 3).isApprox(translation, 1e-4f)));
  const auto& analysis = estimator.getLastDegeneracyAnalysis();
  EXPECT_TRUE(analysis.has_correspondences);
  EXPECT_TRUE(std::isfinite(analysis.condition_number_full));
  EXPECT_TRUE(std::isfinite(analysis.condition_number_rotation));
  EXPECT_TRUE(std::isfinite(analysis.condition_number_translation));
}

TEST(PCL, TransformationEstimationPointToPlaneDCRegPlaneDegeneracy)
{
  const CloudT target = makePlaneTarget();
  const CloudT source =
      makeTranslatedSource(target, Eigen::Vector3f(0.0f, 0.0f, 0.05f));
  const pcl::Correspondences correspondences =
      makeIdentityCorrespondences(target.size());

  DCRegEstimator estimator;
  Eigen::Matrix4f transformation;
  estimator.estimateRigidTransformation(
      source, target, correspondences, transformation);

  expectFiniteTransform(transformation);
  EXPECT_LT((transformation.block<3, 1>(0, 3).norm()), 1.0f);
  const auto& analysis = estimator.getLastDegeneracyAnalysis();
  EXPECT_TRUE(analysis.has_correspondences);
  EXPECT_TRUE(analysis.is_degenerate);
  EXPECT_TRUE(analysis.is_rank_deficient || analysis.weak_translation_axes.sum() > 0);
  EXPECT_TRUE(std::isfinite(analysis.condition_number_full));
}

TEST(PCL, TransformationEstimationPointToPlaneDCRegCylinderDegeneracy)
{
  const CloudT target = makeCylinderTarget();
  const CloudT source = target;
  const pcl::Correspondences correspondences =
      makeIdentityCorrespondences(target.size());

  DCRegEstimator estimator;
  Eigen::Matrix4f transformation;
  estimator.estimateRigidTransformation(
      source, target, correspondences, transformation);

  expectFiniteTransform(transformation);
  EXPECT_TRUE(transformation.isApprox(Eigen::Matrix4f::Identity(), 1e-4f));
  const auto& analysis = estimator.getLastDegeneracyAnalysis();
  EXPECT_TRUE(analysis.has_correspondences);
  EXPECT_TRUE(analysis.is_degenerate);
  EXPECT_EQ(analysis.weak_translation_axes(2), 1);
  EXPECT_TRUE(analysis.is_rank_deficient ||
              analysis.condition_number_translation >
                  estimator.getDegeneracyConditionThreshold());
  EXPECT_FALSE(analysis.solver_type.empty());
}

int
main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return (RUN_ALL_TESTS());
}
