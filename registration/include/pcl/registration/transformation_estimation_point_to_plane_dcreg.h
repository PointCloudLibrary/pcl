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

#pragma once

#include <pcl/registration/transformation_estimation.h>
#include <pcl/cloud_iterator.h>

#include <Eigen/Core>

#include <limits>
#include <string>

namespace pcl {
namespace registration {

/** \brief Diagnostic data produced by TransformationEstimationPointToPlaneDCReg.
 *
 * The weak-axis fields use physical x/y/z order. The rotation axes describe
 * small-angle roll/pitch/yaw increments, and the translation axes describe x/y/z
 * increments in the point-to-plane linearization frame.
 *
 * \ingroup registration
 */
struct DCRegDegeneracyAnalysis {
  bool has_correspondences{false};
  bool is_rank_deficient{false};
  bool schur_factorization_ok{false};
  bool is_degenerate{false};

  double condition_number_full{std::numeric_limits<double>::quiet_NaN()};
  double condition_number_rotation{std::numeric_limits<double>::quiet_NaN()};
  double condition_number_translation{std::numeric_limits<double>::quiet_NaN()};

  Eigen::Vector3d schur_eigenvalues_rotation{
      Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())};
  Eigen::Vector3d schur_eigenvalues_translation{
      Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())};
  Eigen::Vector3d axis_aligned_eigenvalues_rotation{
      Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())};
  Eigen::Vector3d axis_aligned_eigenvalues_translation{
      Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())};

  Eigen::Vector3i weak_rotation_axes{Eigen::Vector3i::Zero()};
  Eigen::Vector3i weak_translation_axes{Eigen::Vector3i::Zero()};

  std::string solver_type{"invalid"};
  bool pcg_converged{false};
  int pcg_iterations{0};
};

/** \brief DCReg-aware point-to-plane linear least-squares transformation estimation.
 *
 * This estimator keeps the same point-to-plane residual model as
 * TransformationEstimationPointToPlaneLLS, but solves the resulting 6D normal
 * equation with DCReg-style Schur-complement degeneracy analysis and a
 * preconditioned conjugate-gradient solve.
 *
 * The implementation adapts the core solver idea from:
 * Xiangcheng Hu et al., "DCReg: Decoupled Characterization for Efficient
 * Degenerate LiDAR Registration", https://arxiv.org/abs/2509.06285 .
 * Reference implementation: https://github.com/JokerJohn/DCReg .
 *
 * \note PointTarget must provide normal_x, normal_y, and normal_z fields.
 * \ingroup registration
 */
template <typename PointSource, typename PointTarget, typename Scalar = float>
class TransformationEstimationPointToPlaneDCReg
: public TransformationEstimation<PointSource, PointTarget, Scalar> {
public:
  using Ptr = shared_ptr<
      TransformationEstimationPointToPlaneDCReg<PointSource, PointTarget, Scalar>>;
  using ConstPtr =
      shared_ptr<const TransformationEstimationPointToPlaneDCReg<PointSource,
                                                                 PointTarget,
                                                                 Scalar>>;
  using Matrix4 =
      typename TransformationEstimation<PointSource, PointTarget, Scalar>::Matrix4;

  TransformationEstimationPointToPlaneDCReg() = default;
  ~TransformationEstimationPointToPlaneDCReg() override = default;

  inline void
  estimateRigidTransformation(const pcl::PointCloud<PointSource>& cloud_src,
                              const pcl::PointCloud<PointTarget>& cloud_tgt,
                              Matrix4& transformation_matrix) const override;

  inline void
  estimateRigidTransformation(const pcl::PointCloud<PointSource>& cloud_src,
                              const pcl::Indices& indices_src,
                              const pcl::PointCloud<PointTarget>& cloud_tgt,
                              Matrix4& transformation_matrix) const override;

  inline void
  estimateRigidTransformation(const pcl::PointCloud<PointSource>& cloud_src,
                              const pcl::Indices& indices_src,
                              const pcl::PointCloud<PointTarget>& cloud_tgt,
                              const pcl::Indices& indices_tgt,
                              Matrix4& transformation_matrix) const override;

  inline void
  estimateRigidTransformation(const pcl::PointCloud<PointSource>& cloud_src,
                              const pcl::PointCloud<PointTarget>& cloud_tgt,
                              const pcl::Correspondences& correspondences,
                              Matrix4& transformation_matrix) const override;

  /** \brief Set the Schur condition number above which an axis is considered weak. */
  inline void
  setDegeneracyConditionThreshold(double threshold)
  {
    degeneracy_condition_threshold_ = threshold;
  }

  inline double
  getDegeneracyConditionThreshold() const
  {
    return (degeneracy_condition_threshold_);
  }

  /** \brief Set the target condition number used when clamping weak eigenvalues. */
  inline void
  setKappaTarget(double kappa_target)
  {
    kappa_target_ = kappa_target;
  }

  inline double
  getKappaTarget() const
  {
    return (kappa_target_);
  }

  /** \brief Set the relative stopping tolerance for the PCG solve. */
  inline void
  setPcgTolerance(double tolerance)
  {
    pcg_tolerance_ = tolerance;
  }

  inline double
  getPcgTolerance() const
  {
    return (pcg_tolerance_);
  }

  /** \brief Set the maximum number of PCG iterations. */
  inline void
  setPcgMaxIterations(int max_iterations)
  {
    pcg_max_iterations_ = max_iterations;
  }

  inline int
  getPcgMaxIterations() const
  {
    return (pcg_max_iterations_);
  }

  /** \brief Return diagnostics from the last transformation estimation call. */
  inline const DCRegDegeneracyAnalysis&
  getLastDegeneracyAnalysis() const
  {
    return (last_degeneracy_analysis_);
  }

protected:
  void
  estimateRigidTransformation(ConstCloudIterator<PointSource>& source_it,
                              ConstCloudIterator<PointTarget>& target_it,
                              Matrix4& transformation_matrix) const;

  inline void
  constructTransformationMatrix(const double& alpha,
                                const double& beta,
                                const double& gamma,
                                const double& tx,
                                const double& ty,
                                const double& tz,
                                Matrix4& transformation_matrix) const;

  double degeneracy_condition_threshold_{10.0};
  double kappa_target_{10.0};
  double pcg_tolerance_{1e-6};
  int pcg_max_iterations_{10};
  mutable DCRegDegeneracyAnalysis last_degeneracy_analysis_;
};

} // namespace registration
} // namespace pcl

#include <pcl/registration/impl/transformation_estimation_point_to_plane_dcreg.hpp>
