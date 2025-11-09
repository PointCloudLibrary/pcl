/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-, Open Perception, Inc.
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

#pragma once

#include <pcl/registration/anderson_acceleration.h>
#include <pcl/registration/icp.h>
#include <pcl/search/kdtree.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>

#include <Eigen/Core>

#include <vector>

namespace pcl {
/**
 * \brief FastRobustIterativeClosestPoint implements the FRICP variant described in
 *        "Fast and Robust Iterative Closest Point", Zhang et al., 2022.
 *
 * The solver relies on Welsch reweighting for robustness and optional Anderson
 * acceleration for faster convergence.
 *
 * \ingroup registration
 */
template <typename PointSource, typename PointTarget, typename Scalar = float>
class FastRobustIterativeClosestPoint
: public IterativeClosestPoint<PointSource, PointTarget, Scalar> {
public:
  using Base = IterativeClosestPoint<PointSource, PointTarget, Scalar>;
  using typename Base::Matrix4;
  using typename Base::PointCloudSource;
  using typename Base::PointCloudTarget;

  enum class RobustFunction { NONE, WELSCH };

  FastRobustIterativeClosestPoint();

  void
  setRobustFunction(RobustFunction f);

  [[nodiscard]] RobustFunction
  getRobustFunction() const;

  void
  setUseAndersonAcceleration(bool enabled);

  [[nodiscard]] bool
  getUseAndersonAcceleration() const;

  void
  setAndersonHistorySize(std::size_t history);

  [[nodiscard]] std::size_t
  getAndersonHistorySize() const;

  void
  setDynamicWelschBeginRatio(double ratio);

  void
  setDynamicWelschEndRatio(double ratio);

  void
  setDynamicWelschDecay(double ratio);

protected:
  void
  computeTransformation(PointCloudSource& output, const Matrix4& guess) override;

private:
  using Matrix4d = Eigen::Matrix<double, 4, 4>;
  using Matrix3Xd = Eigen::Matrix<double, 3, Eigen::Dynamic>;
  using VectorXd = Eigen::VectorXd;
  using Vector3d = Eigen::Vector3d;
  using AndersonAccelerationType = registration::AndersonAcceleration;

  Matrix4d
  convertGuessToCentered(const Matrix4& guess,
                         const Vector3d& source_mean,
                         const Vector3d& target_mean) const;

  Matrix4d
  convertCenteredToActual(const Matrix4d& transform,
                          const Vector3d& source_mean,
                          const Vector3d& target_mean) const;

  bool
  updateCorrespondences(const Matrix4d& transform,
                        const Matrix3Xd& source,
                        const Matrix3Xd& target,
                        pcl::search::KdTree<pcl::PointXYZ>& tree,
                        Matrix3Xd& matched_targets,
                        VectorXd& residuals) const;

  double
  computeEnergy(const VectorXd& residuals, double nu) const;

  VectorXd
  computeWeights(const VectorXd& residuals, double nu) const;

  Matrix4d
  computeWeightedRigidTransform(const Matrix3Xd& source,
                                const Matrix3Xd& target,
                                const VectorXd& weights) const;

  double
  findKNearestMedian(const pcl::PointCloud<pcl::PointXYZ>& cloud,
                     pcl::search::KdTree<pcl::PointXYZ>& tree,
                     int neighbors) const;

  double
  computeMedian(const VectorXd& values) const;

  double
  computeMedian(std::vector<double> values) const;

  Matrix4d
  matrixLog(const Matrix4d& transform) const;

  RobustFunction robust_function_;
  bool use_anderson_;
  std::size_t anderson_history_;
  double nu_begin_ratio_;
  double nu_end_ratio_;
  double nu_decay_ratio_;

  static constexpr double same_threshold_ = 1e-6;

  AndersonAccelerationType anderson_;

  PCL_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace pcl

#include <pcl/registration/impl/fricp.hpp>
