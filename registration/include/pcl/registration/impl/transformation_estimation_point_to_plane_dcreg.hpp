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

#ifndef PCL_REGISTRATION_TRANSFORMATION_ESTIMATION_POINT_TO_PLANE_DCREG_HPP_
#define PCL_REGISTRATION_TRANSFORMATION_ESTIMATION_POINT_TO_PLANE_DCREG_HPP_

#include <pcl/console/print.h>
#include <pcl/cloud_iterator.h>

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <Eigen/LU>
#include <Eigen/QR>
#include <Eigen/SVD>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace pcl {
namespace registration {
namespace detail {

using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;

constexpr double min_pivot = 1e-12;
constexpr double min_eigenvalue = 1e-9;

struct DCRegLinearSystem {
  Matrix6d hessian{Matrix6d::Zero()};
  Vector6d rhs{Vector6d::Zero()};
  int valid_correspondences{0};
};

struct DCRegDetection {
  bool factorization_ok{false};
  Eigen::Vector3d lambda_rot{
      Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())};
  Eigen::Vector3d lambda_trans{
      Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())};
  Eigen::Matrix3d basis_rot{Eigen::Matrix3d::Identity()};
  Eigen::Matrix3d basis_trans{Eigen::Matrix3d::Identity()};
};

struct DCRegCharacterization {
  bool factorization_ok{false};
  Eigen::Matrix<double, 6, 6> preconditioner{Matrix6d::Identity()};
  double condition_number_rot{std::numeric_limits<double>::quiet_NaN()};
  double condition_number_trans{std::numeric_limits<double>::quiet_NaN()};
  Eigen::Vector3d aligned_lambda_rot{
      Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())};
  Eigen::Vector3d aligned_lambda_trans{
      Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())};
  Eigen::Vector3i weak_rot_axes{Eigen::Vector3i::Zero()};
  Eigen::Vector3i weak_trans_axes{Eigen::Vector3i::Zero()};
};

struct DCRegSolverResult {
  Vector6d update{Vector6d::Zero()};
  std::string solver_type{"dense"};
  bool pcg_converged{false};
  int pcg_iterations{0};
};

inline double
conditionNumber(const Eigen::Vector3d& lambda)
{
  return (lambda.maxCoeff() / std::max(lambda.minCoeff(), min_pivot));
}

inline double
fullConditionNumber(const Matrix6d& hessian)
{
  const Eigen::JacobiSVD<Matrix6d> svd(hessian);
  const Vector6d singular_values = svd.singularValues();
  return (singular_values(0) / std::max(singular_values(5), min_pivot));
}

inline bool
eigenDecomposeSymmetric(const Eigen::Matrix3d& matrix,
                        Eigen::Vector3d& eigenvalues,
                        Eigen::Matrix3d& eigenvectors)
{
  const Eigen::Matrix3d symmetric = 0.5 * (matrix + matrix.transpose());
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(symmetric);
  if (solver.info() != Eigen::Success)
    return (false);

  eigenvalues = solver.eigenvalues();
  eigenvectors = solver.eigenvectors();
  return (eigenvalues.allFinite() && eigenvectors.allFinite());
}

inline DCRegDetection
detectSchurDegeneracy(const Matrix6d& hessian)
{
  DCRegDetection detection;
  const Eigen::Matrix3d h_rr = hessian.block<3, 3>(0, 0);
  const Eigen::Matrix3d h_rt = hessian.block<3, 3>(0, 3);
  const Eigen::Matrix3d h_tr = hessian.block<3, 3>(3, 0);
  const Eigen::Matrix3d h_tt = hessian.block<3, 3>(3, 3);

  const Eigen::FullPivLU<Eigen::Matrix3d> lu_rr(h_rr);
  const Eigen::FullPivLU<Eigen::Matrix3d> lu_tt(h_tt);
  if (!lu_rr.isInvertible() || !lu_tt.isInvertible())
    return (detection);

  const Eigen::Matrix3d schur_rot = h_rr - h_rt * lu_tt.solve(h_tr);
  const Eigen::Matrix3d schur_trans = h_tt - h_tr * lu_rr.solve(h_rt);
  if (!eigenDecomposeSymmetric(schur_rot, detection.lambda_rot, detection.basis_rot) ||
      !eigenDecomposeSymmetric(
          schur_trans, detection.lambda_trans, detection.basis_trans))
    return (detection);

  detection.factorization_ok = true;
  return (detection);
}

inline DCRegDetection
detectBlockFallback(const Matrix6d& hessian)
{
  DCRegDetection detection;
  if (!eigenDecomposeSymmetric(
          hessian.block<3, 3>(0, 0), detection.lambda_rot, detection.basis_rot) ||
      !eigenDecomposeSymmetric(
          hessian.block<3, 3>(3, 3), detection.lambda_trans, detection.basis_trans))
    return (detection);

  detection.factorization_ok = true;
  return (detection);
}

inline bool
alignEigenBasisToAxes(const Eigen::Matrix3d& raw_basis,
                      Eigen::Matrix3d& aligned_basis,
                      std::array<int, 3>& original_indices)
{
  const std::array<Eigen::Vector3d, 3> reference_axes = {
      Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitZ()};
  std::array<bool, 3> used = {false, false, false};
  original_indices.fill(-1);
  aligned_basis.setZero();

  for (int axis = 0; axis < 3; ++axis) {
    double best_score = -1.0;
    int best_index = -1;
    for (int candidate = 0; candidate < 3; ++candidate) {
      if (used[candidate])
        continue;
      const double score = std::abs(reference_axes[axis].dot(raw_basis.col(candidate)));
      if (score > best_score) {
        best_score = score;
        best_index = candidate;
      }
    }
    if (best_index < 0)
      return (false);
    used[best_index] = true;
    original_indices[axis] = best_index;
    aligned_basis.col(axis) = raw_basis.col(best_index);
    if (reference_axes[axis].dot(aligned_basis.col(axis)) < 0.0)
      aligned_basis.col(axis) = -aligned_basis.col(axis);
  }
  return (true);
}

inline Eigen::Vector3i
weakAxes(const Eigen::Vector3d& aligned_lambda, double condition_threshold)
{
  Eigen::Vector3i weak_axes = Eigen::Vector3i::Zero();
  const double lambda_max = std::max(aligned_lambda.maxCoeff(), min_eigenvalue);
  const double threshold = std::max(condition_threshold, 1.0);
  for (int axis = 0; axis < 3; ++axis) {
    const double condition = lambda_max / std::max(aligned_lambda(axis), min_pivot);
    if (condition > threshold)
      weak_axes(axis) = 1;
  }
  return (weak_axes);
}

inline Eigen::Vector3d
clampWeakEigenvalues(const Eigen::Vector3d& aligned_lambda,
                     const Eigen::Vector3i& weak_axes,
                     double kappa_target)
{
  Eigen::Vector3d clamped = aligned_lambda;
  const double lambda_max = std::max(aligned_lambda.maxCoeff(), min_eigenvalue);
  const double weak_lambda =
      std::max(lambda_max / std::max(kappa_target, 1.0), min_eigenvalue);
  for (int axis = 0; axis < 3; ++axis) {
    if (weak_axes(axis))
      clamped(axis) = weak_lambda;
  }
  return (clamped);
}

inline DCRegCharacterization
characterizeDegeneracy(const DCRegDetection& detection,
                       double condition_threshold,
                       double kappa_target)
{
  DCRegCharacterization characterization;
  characterization.factorization_ok = detection.factorization_ok;
  if (!detection.factorization_ok)
    return (characterization);

  Eigen::Matrix3d aligned_rot_basis;
  Eigen::Matrix3d aligned_trans_basis;
  std::array<int, 3> rot_indices;
  std::array<int, 3> trans_indices;
  if (!alignEigenBasisToAxes(detection.basis_rot, aligned_rot_basis, rot_indices) ||
      !alignEigenBasisToAxes(
          detection.basis_trans, aligned_trans_basis, trans_indices)) {
    characterization.factorization_ok = false;
    return (characterization);
  }

  for (int axis = 0; axis < 3; ++axis) {
    characterization.aligned_lambda_rot(axis) = detection.lambda_rot(rot_indices[axis]);
    characterization.aligned_lambda_trans(axis) =
        detection.lambda_trans(trans_indices[axis]);
  }

  characterization.condition_number_rot =
      conditionNumber(characterization.aligned_lambda_rot);
  characterization.condition_number_trans =
      conditionNumber(characterization.aligned_lambda_trans);
  characterization.weak_rot_axes =
      weakAxes(characterization.aligned_lambda_rot, condition_threshold);
  characterization.weak_trans_axes =
      weakAxes(characterization.aligned_lambda_trans, condition_threshold);

  const Eigen::Vector3d clamped_rot =
      clampWeakEigenvalues(characterization.aligned_lambda_rot,
                           characterization.weak_rot_axes,
                           kappa_target);
  const Eigen::Vector3d clamped_trans =
      clampWeakEigenvalues(characterization.aligned_lambda_trans,
                           characterization.weak_trans_axes,
                           kappa_target);

  characterization.preconditioner.setZero();
  characterization.preconditioner.block<3, 3>(0, 0) =
      aligned_rot_basis *
      clamped_rot.cwiseMax(min_eigenvalue).cwiseInverse().asDiagonal() *
      aligned_rot_basis.transpose();
  characterization.preconditioner.block<3, 3>(3, 3) =
      aligned_trans_basis *
      clamped_trans.cwiseMax(min_eigenvalue).cwiseInverse().asDiagonal() *
      aligned_trans_basis.transpose();
  return (characterization);
}

inline bool
solveRankDeficientMinimumNorm(const DCRegLinearSystem& system, Vector6d& update)
{
  const Matrix6d hessian = 0.5 * (system.hessian + system.hessian.transpose());
  const Eigen::SelfAdjointEigenSolver<Matrix6d> solver(hessian);
  if (solver.info() != Eigen::Success)
    return (false);

  const Vector6d eigenvalues = solver.eigenvalues();
  const double lambda_max = eigenvalues.cwiseAbs().maxCoeff();
  if (lambda_max < min_pivot) {
    update.setZero();
    return (true);
  }

  const double nullspace_threshold = std::max(lambda_max * 1e-10, min_pivot);
  bool rank_deficient = false;
  Vector6d inverse_eigenvalues = Vector6d::Zero();
  for (int i = 0; i < 6; ++i) {
    if (eigenvalues(i) <= nullspace_threshold) {
      rank_deficient = true;
      continue;
    }
    inverse_eigenvalues(i) = 1.0 / eigenvalues(i);
  }
  if (!rank_deficient)
    return (false);

  update = solver.eigenvectors() * inverse_eigenvalues.asDiagonal() *
           solver.eigenvectors().transpose() * system.rhs;
  return (update.allFinite());
}

inline DCRegSolverResult
solveDenseFallback(const DCRegLinearSystem& system)
{
  DCRegSolverResult result;
  const Matrix6d hessian = 0.5 * (system.hessian + system.hessian.transpose());
  const Eigen::LDLT<Matrix6d> ldlt(hessian);
  if (ldlt.info() == Eigen::Success && ldlt.isPositive()) {
    result.update = ldlt.solve(system.rhs);
    if (result.update.allFinite()) {
      result.solver_type = "dense";
      return (result);
    }
  }

  result.solver_type = "qr_fallback";
  result.update = system.hessian.colPivHouseholderQr().solve(system.rhs);
  if (!result.update.allFinite())
    result.update.setZero();
  return (result);
}

inline DCRegSolverResult
solvePreconditioned(const DCRegLinearSystem& system,
                    const DCRegCharacterization& characterization,
                    double pcg_tolerance,
                    int pcg_max_iterations)
{
  Vector6d minimum_norm_update;
  if (solveRankDeficientMinimumNorm(system, minimum_norm_update)) {
    DCRegSolverResult result;
    result.update = minimum_norm_update;
    result.solver_type = "minimum_norm";
    return (result);
  }

  if (!characterization.factorization_ok ||
      !characterization.preconditioner.allFinite())
    return (solveDenseFallback(system));

  const Matrix6d hessian = 0.5 * (system.hessian + system.hessian.transpose());
  const double rhs_norm = system.rhs.norm();
  if (rhs_norm < min_pivot) {
    DCRegSolverResult result;
    result.solver_type = "zero_rhs";
    result.pcg_converged = true;
    return (result);
  }

  Vector6d delta = Vector6d::Zero();
  Vector6d residual = system.rhs;
  Vector6d z = characterization.preconditioner * residual;
  Vector6d direction = z;
  double rz_old = residual.dot(z);
  if (!z.allFinite() || !std::isfinite(rz_old) || std::abs(rz_old) < min_pivot)
    return (solveDenseFallback(system));

  const double tolerance = pcg_tolerance > 0.0 ? pcg_tolerance : 1e-6;
  const double target_residual = tolerance * std::max(1.0, rhs_norm);
  const int max_iterations = std::max(1, pcg_max_iterations);
  int iterations = 0;
  for (int i = 0; i < max_iterations; ++i) {
    iterations = i + 1;
    const Vector6d hessian_direction = hessian * direction;
    const double denom = direction.dot(hessian_direction);
    if (!std::isfinite(denom) || std::abs(denom) < min_pivot)
      break;

    const double alpha = rz_old / denom;
    if (!std::isfinite(alpha))
      break;

    delta += alpha * direction;
    residual -= alpha * hessian_direction;
    if (!delta.allFinite() || !residual.allFinite())
      break;
    if (residual.norm() <= target_residual) {
      DCRegSolverResult result;
      result.update = delta;
      result.solver_type = "pcg";
      result.pcg_converged = true;
      result.pcg_iterations = iterations;
      return (result);
    }

    z = characterization.preconditioner * residual;
    const double rz_new = residual.dot(z);
    if (!z.allFinite() || !std::isfinite(rz_new) || std::abs(rz_old) < min_pivot)
      break;

    const double beta = rz_new / rz_old;
    if (!std::isfinite(beta))
      break;
    direction = z + beta * direction;
    rz_old = rz_new;
  }

  DCRegSolverResult result = solveDenseFallback(system);
  result.pcg_iterations = iterations;
  return (result);
}

} // namespace detail

template <typename PointSource, typename PointTarget, typename Scalar>
inline void
TransformationEstimationPointToPlaneDCReg<PointSource, PointTarget, Scalar>::
    estimateRigidTransformation(const pcl::PointCloud<PointSource>& cloud_src,
                                const pcl::PointCloud<PointTarget>& cloud_tgt,
                                Matrix4& transformation_matrix) const
{
  if (cloud_tgt.size() != cloud_src.size()) {
    PCL_ERROR("[pcl::TransformationEstimationPointToPlaneDCReg::"
              "estimateRigidTransformation] Number or points in source (%zu) differs "
              "than target (%zu)!\n",
              static_cast<std::size_t>(cloud_src.size()),
              static_cast<std::size_t>(cloud_tgt.size()));
    return;
  }

  ConstCloudIterator<PointSource> source_it(cloud_src);
  ConstCloudIterator<PointTarget> target_it(cloud_tgt);
  estimateRigidTransformation(source_it, target_it, transformation_matrix);
}

template <typename PointSource, typename PointTarget, typename Scalar>
inline void
TransformationEstimationPointToPlaneDCReg<PointSource, PointTarget, Scalar>::
    estimateRigidTransformation(const pcl::PointCloud<PointSource>& cloud_src,
                                const pcl::Indices& indices_src,
                                const pcl::PointCloud<PointTarget>& cloud_tgt,
                                Matrix4& transformation_matrix) const
{
  if (cloud_tgt.size() != indices_src.size()) {
    PCL_ERROR("[pcl::TransformationEstimationPointToPlaneDCReg::"
              "estimateRigidTransformation] Number or points in source (%zu) differs "
              "than target (%zu)!\n",
              indices_src.size(),
              static_cast<std::size_t>(cloud_tgt.size()));
    return;
  }

  ConstCloudIterator<PointSource> source_it(cloud_src, indices_src);
  ConstCloudIterator<PointTarget> target_it(cloud_tgt);
  estimateRigidTransformation(source_it, target_it, transformation_matrix);
}

template <typename PointSource, typename PointTarget, typename Scalar>
inline void
TransformationEstimationPointToPlaneDCReg<PointSource, PointTarget, Scalar>::
    estimateRigidTransformation(const pcl::PointCloud<PointSource>& cloud_src,
                                const pcl::Indices& indices_src,
                                const pcl::PointCloud<PointTarget>& cloud_tgt,
                                const pcl::Indices& indices_tgt,
                                Matrix4& transformation_matrix) const
{
  if (indices_tgt.size() != indices_src.size()) {
    PCL_ERROR("[pcl::TransformationEstimationPointToPlaneDCReg::"
              "estimateRigidTransformation] Number or points in source (%zu) differs "
              "than target (%zu)!\n",
              indices_src.size(),
              indices_tgt.size());
    return;
  }

  ConstCloudIterator<PointSource> source_it(cloud_src, indices_src);
  ConstCloudIterator<PointTarget> target_it(cloud_tgt, indices_tgt);
  estimateRigidTransformation(source_it, target_it, transformation_matrix);
}

template <typename PointSource, typename PointTarget, typename Scalar>
inline void
TransformationEstimationPointToPlaneDCReg<PointSource, PointTarget, Scalar>::
    estimateRigidTransformation(const pcl::PointCloud<PointSource>& cloud_src,
                                const pcl::PointCloud<PointTarget>& cloud_tgt,
                                const pcl::Correspondences& correspondences,
                                Matrix4& transformation_matrix) const
{
  ConstCloudIterator<PointSource> source_it(cloud_src, correspondences, true);
  ConstCloudIterator<PointTarget> target_it(cloud_tgt, correspondences, false);
  estimateRigidTransformation(source_it, target_it, transformation_matrix);
}

template <typename PointSource, typename PointTarget, typename Scalar>
inline void
TransformationEstimationPointToPlaneDCReg<PointSource, PointTarget, Scalar>::
    constructTransformationMatrix(const double& alpha,
                                  const double& beta,
                                  const double& gamma,
                                  const double& tx,
                                  const double& ty,
                                  const double& tz,
                                  Matrix4& transformation_matrix) const
{
  transformation_matrix = Matrix4::Zero();
  transformation_matrix(0, 0) = static_cast<Scalar>(std::cos(gamma) * std::cos(beta));
  transformation_matrix(0, 1) =
      static_cast<Scalar>(-std::sin(gamma) * std::cos(alpha) +
                          std::cos(gamma) * std::sin(beta) * std::sin(alpha));
  transformation_matrix(0, 2) =
      static_cast<Scalar>(std::sin(gamma) * std::sin(alpha) +
                          std::cos(gamma) * std::sin(beta) * std::cos(alpha));
  transformation_matrix(1, 0) = static_cast<Scalar>(std::sin(gamma) * std::cos(beta));
  transformation_matrix(1, 1) =
      static_cast<Scalar>(std::cos(gamma) * std::cos(alpha) +
                          std::sin(gamma) * std::sin(beta) * std::sin(alpha));
  transformation_matrix(1, 2) =
      static_cast<Scalar>(-std::cos(gamma) * std::sin(alpha) +
                          std::sin(gamma) * std::sin(beta) * std::cos(alpha));
  transformation_matrix(2, 0) = static_cast<Scalar>(-std::sin(beta));
  transformation_matrix(2, 1) = static_cast<Scalar>(std::cos(beta) * std::sin(alpha));
  transformation_matrix(2, 2) = static_cast<Scalar>(std::cos(beta) * std::cos(alpha));
  transformation_matrix(0, 3) = static_cast<Scalar>(tx);
  transformation_matrix(1, 3) = static_cast<Scalar>(ty);
  transformation_matrix(2, 3) = static_cast<Scalar>(tz);
  transformation_matrix(3, 3) = static_cast<Scalar>(1);
}

template <typename PointSource, typename PointTarget, typename Scalar>
void
TransformationEstimationPointToPlaneDCReg<PointSource, PointTarget, Scalar>::
    estimateRigidTransformation(ConstCloudIterator<PointSource>& source_it,
                                ConstCloudIterator<PointTarget>& target_it,
                                Matrix4& transformation_matrix) const
{
  detail::DCRegLinearSystem system;
  transformation_matrix = Matrix4::Identity();
  last_degeneracy_analysis_ = DCRegDegeneracyAnalysis();

  while (source_it.isValid() && target_it.isValid()) {
    if (!std::isfinite(source_it->x) || !std::isfinite(source_it->y) ||
        !std::isfinite(source_it->z) || !std::isfinite(target_it->x) ||
        !std::isfinite(target_it->y) || !std::isfinite(target_it->z) ||
        !std::isfinite(target_it->normal_x) || !std::isfinite(target_it->normal_y) ||
        !std::isfinite(target_it->normal_z)) {
      ++source_it;
      ++target_it;
      continue;
    }

    const double sx = source_it->x;
    const double sy = source_it->y;
    const double sz = source_it->z;
    const double dx = target_it->x;
    const double dy = target_it->y;
    const double dz = target_it->z;
    const double nx = target_it->normal[0];
    const double ny = target_it->normal[1];
    const double nz = target_it->normal[2];

    detail::Vector6d jacobian;
    jacobian << nz * sy - ny * sz, nx * sz - nz * sx, ny * sx - nx * sy, nx, ny, nz;
    const double rhs = nx * dx + ny * dy + nz * dz - nx * sx - ny * sy - nz * sz;
    system.hessian.noalias() += jacobian * jacobian.transpose();
    system.rhs.noalias() += jacobian * rhs;
    ++system.valid_correspondences;

    ++source_it;
    ++target_it;
  }

  if (system.valid_correspondences == 0) {
    last_degeneracy_analysis_.solver_type = "no_correspondences";
    return;
  }

  last_degeneracy_analysis_.has_correspondences = true;
  last_degeneracy_analysis_.condition_number_full =
      detail::fullConditionNumber(system.hessian);

  detail::Vector6d minimum_norm_update;
  last_degeneracy_analysis_.is_rank_deficient =
      detail::solveRankDeficientMinimumNorm(system, minimum_norm_update);

  const detail::DCRegDetection schur_detection =
      detail::detectSchurDegeneracy(system.hessian);
  const detail::DCRegDetection detection =
      schur_detection.factorization_ok ? schur_detection
                                       : detail::detectBlockFallback(system.hessian);
  const detail::DCRegCharacterization characterization = detail::characterizeDegeneracy(
      detection, degeneracy_condition_threshold_, kappa_target_);

  last_degeneracy_analysis_.schur_factorization_ok = schur_detection.factorization_ok;
  if (detection.factorization_ok) {
    last_degeneracy_analysis_.schur_eigenvalues_rotation = detection.lambda_rot;
    last_degeneracy_analysis_.schur_eigenvalues_translation = detection.lambda_trans;
  }
  if (characterization.factorization_ok) {
    last_degeneracy_analysis_.condition_number_rotation =
        characterization.condition_number_rot;
    last_degeneracy_analysis_.condition_number_translation =
        characterization.condition_number_trans;
    last_degeneracy_analysis_.axis_aligned_eigenvalues_rotation =
        characterization.aligned_lambda_rot;
    last_degeneracy_analysis_.axis_aligned_eigenvalues_translation =
        characterization.aligned_lambda_trans;
    last_degeneracy_analysis_.weak_rotation_axes = characterization.weak_rot_axes;
    last_degeneracy_analysis_.weak_translation_axes = characterization.weak_trans_axes;
  }
  last_degeneracy_analysis_.is_degenerate =
      last_degeneracy_analysis_.is_rank_deficient ||
      last_degeneracy_analysis_.weak_rotation_axes.any() ||
      last_degeneracy_analysis_.weak_translation_axes.any();

  const detail::DCRegSolverResult result = detail::solvePreconditioned(
      system, characterization, pcg_tolerance_, pcg_max_iterations_);
  last_degeneracy_analysis_.solver_type = result.solver_type;
  last_degeneracy_analysis_.pcg_converged = result.pcg_converged;
  last_degeneracy_analysis_.pcg_iterations = result.pcg_iterations;

  constructTransformationMatrix(result.update(0),
                                result.update(1),
                                result.update(2),
                                result.update(3),
                                result.update(4),
                                result.update(5),
                                transformation_matrix);
}

} // namespace registration
} // namespace pcl

#endif // PCL_REGISTRATION_TRANSFORMATION_ESTIMATION_POINT_TO_PLANE_DCREG_HPP_
