/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-, Open Perception, Inc.
 *
 *  All rights reserved.
 */

#pragma once

#include <pcl/registration/anderson_acceleration.h>
#include <pcl/registration/icp.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/search.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>

#include <Eigen/Core>

#include <cmath>
#include <vector>

namespace pcl {
/**
 * \brief FastRobustIterativeClosestPoint implements the FRICP variant described in
 *        "Fast and Robust Iterative Closest Point", Zhang et al., 2021.
 *
 * The solver relies on Welsch reweighting for robustness and optional Anderson
 * acceleration for faster convergence.
 *
 * \code
 * pcl::FastRobustIterativeClosestPoint<PointT, PointT> reg;
 * reg.setInputSource (src); // src and tgt are clouds that must be created before
 * reg.setInputTarget (tgt);
 * reg.setMaximumIterations (60); // parameters may have to be tuned, depending on the point clouds
 * reg.setTransformationEpsilon (1e-8);
 * pcl::PointCloud<PointT> output;
 * reg.align (output);
 * \endcode
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

  /** \brief Enable or disable Anderson acceleration in the FRICP optimization loop.
   *
   * When enabled, convergence can be faster on some datasets but may become less
   * stable. The default is disabled to keep behavior predictable.
   */
  void
  setUseAndersonAcceleration(bool enabled);

  [[nodiscard]] bool
  getUseAndersonAcceleration() const;

  /** \brief Set the history size used by Anderson acceleration.
   *
   * Larger values may improve acceleration quality but can increase instability and
   * memory usage. Values smaller than 1 are clamped to 1.
   */
  void
  setAndersonHistorySize(std::size_t history);

  [[nodiscard]] std::size_t
  getAndersonHistorySize() const;

  /** \brief Set the initial Welsch scale ratio used in dynamic robust weighting.
   *
   * Larger values start with weaker down-weighting of outliers. Values are clamped
   * to a small positive threshold.
   */
  void
  setDynamicWelschBeginRatio(double ratio);

  /** \brief Set the final Welsch scale ratio used in dynamic robust weighting.
   *
   * Smaller values end with stronger outlier suppression. Values are clamped to a
   * small positive threshold.
   */
  void
  setDynamicWelschEndRatio(double ratio);

  /** \brief Set the multiplicative decay applied to the dynamic Welsch scale.
   *
   * Valid range is [0, 1]. Smaller values reduce the scale faster per outer
   * iteration, while larger values keep it closer to the current value.
   */
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
                        pcl::search::Search<pcl::PointXYZ>& tree,
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
                     pcl::search::Search<pcl::PointXYZ>& tree,
                     int neighbors) const;

  Matrix4d
  matrixLog(const Matrix4d& transform) const;

  RobustFunction robust_function_;
  bool use_anderson_ = false;
  std::size_t anderson_history_ = 5;
  double nu_begin_ratio_ = 3.0;
  double nu_end_ratio_ = 1.0 / (3.0 * std::sqrt(3.0));
  double nu_decay_ratio_ = 0.5;

  static constexpr double same_threshold_ = 1e-6;

  AndersonAccelerationType anderson_;

  PCL_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace pcl

#include <pcl/registration/impl/fricp.hpp>
