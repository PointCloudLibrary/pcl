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
#include <pcl/memory.h>
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
 * \note FRICP uses its own robust correspondence mechanism based on the Welsch
 * function. The following inherited ICP settings have no effect:
 *   - setMaxCorrespondenceDistance() 閳?FRICP controls outlier rejection via
 *     the Welsch scale parameter instead.
 *   - Correspondence estimators, rejectors, and reciprocal correspondences
 *     are not used by this class.
 *   - setEuclideanFitnessEpsilon() and setRotationEpsilon() are not evaluated.
 *
 * \note setMaximumIterations() controls the number of inner-loop iterations
 * per Welsch scale stage. The outer loop may run multiple stages as the
 * scale decays, so the total number of iterations can exceed this value.
 *
 * \note Input point clouds must not contain NaN or Inf values. Non-finite
 * points are not handled and will produce incorrect results.
 *
 * \code
 * pcl::FastRobustIterativeClosestPoint<PointT, PointT> reg;
 * reg.setInputSource (src); // src and tgt are clouds that must be created before
 * reg.setInputTarget (tgt);
 * // parameters may have to be tuned, depending on the point clouds
 * reg.setMaximumIterations (60);
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
  using Ptr =
      shared_ptr<FastRobustIterativeClosestPoint<PointSource, PointTarget, Scalar>>;
  using ConstPtr = shared_ptr<
      const FastRobustIterativeClosestPoint<PointSource, PointTarget, Scalar>>;

  using Base = IterativeClosestPoint<PointSource, PointTarget, Scalar>;
  using typename Base::Matrix4;
  using typename Base::PointCloudSource;
  using typename Base::PointCloudTarget;

  enum class RobustFunction { NONE, WELSCH };
  enum class ConvergenceTrigger {
    NONE = 0,
    DEFAULT_CRITERIA,
    FRICP_STOP_THRESHOLD,
    ITERATION_LIMIT,
    NO_CORRESPONDENCES
  };

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
   * Valid range is (0, 1]. A value of 0 or a very small positive number is
   * silently replaced by the default (0.5). Smaller values reduce the scale
   * faster per outer iteration, while larger values keep it closer to the
   * current value.
   */
  void
  setDynamicWelschDecay(double ratio);

  [[nodiscard]] ConvergenceTrigger
  getLastConvergenceTrigger() const;

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
                        VectorXd& residuals,
                        pcl::Correspondences* correspondences = nullptr) const;

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

  RobustFunction robust_function_ = RobustFunction::WELSCH;
  bool use_anderson_ = false;
  std::size_t anderson_history_ = 5;
  double nu_begin_ratio_ = 3.0;
  double nu_end_ratio_ = 1.0 / (3.0 * std::sqrt(3.0));
  double nu_decay_ratio_ = 0.5;
  ConvergenceTrigger last_convergence_trigger_ = ConvergenceTrigger::NONE;

  static constexpr double same_threshold_ = 1e-6;

  AndersonAccelerationType anderson_;

  PCL_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace pcl

#include <pcl/registration/impl/fricp.hpp>
