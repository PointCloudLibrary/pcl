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
 *   * Redistributions of source code must reproduce the above
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

#ifndef PCL_REGISTRATION_IMPL_FRICP_HPP_
#define PCL_REGISTRATION_IMPL_FRICP_HPP_

#include <pcl/common/transforms.h>
#include <pcl/types.h>

#include <Eigen/Eigenvalues>
#include <Eigen/SVD>
#include <unsupported/Eigen/MatrixFunctions>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>
#include <utility>
#include <vector>
namespace pcl {

template <typename PointSource, typename PointTarget, typename Scalar>
FastRobustIterativeClosestPoint<PointSource, PointTarget, Scalar>::
    FastRobustIterativeClosestPoint()
: robust_function_(RobustFunction::WELSCH)
, nu_end_ratio_(1.0 / (3.0 * std::sqrt(3.0)))
{
  this->reg_name_ = "FastRobustIterativeClosestPoint";
  this->max_iterations_ = 50;
  this->transformation_epsilon_ = static_cast<Scalar>(1e-6);
  this->min_number_correspondences_ = 4;
}

template <typename PointSource, typename PointTarget, typename Scalar>
void
FastRobustIterativeClosestPoint<PointSource, PointTarget, Scalar>::setRobustFunction(
    RobustFunction f)
{
  robust_function_ = f;
}

template <typename PointSource, typename PointTarget, typename Scalar>
typename FastRobustIterativeClosestPoint<PointSource,
                                         PointTarget,
                                         Scalar>::RobustFunction
FastRobustIterativeClosestPoint<PointSource, PointTarget, Scalar>::getRobustFunction()
    const
{
  return robust_function_;
}

template <typename PointSource, typename PointTarget, typename Scalar>
void
FastRobustIterativeClosestPoint<PointSource, PointTarget, Scalar>::
    setUseAndersonAcceleration(bool enabled)
{
  use_anderson_ = enabled;
}

template <typename PointSource, typename PointTarget, typename Scalar>
bool
FastRobustIterativeClosestPoint<PointSource, PointTarget, Scalar>::
    getUseAndersonAcceleration() const
{
  return use_anderson_;
}

template <typename PointSource, typename PointTarget, typename Scalar>
void
FastRobustIterativeClosestPoint<PointSource, PointTarget, Scalar>::
    setAndersonHistorySize(std::size_t history)
{
  anderson_history_ = std::max<std::size_t>(1, history);
}

template <typename PointSource, typename PointTarget, typename Scalar>
std::size_t
FastRobustIterativeClosestPoint<PointSource, PointTarget, Scalar>::
    getAndersonHistorySize() const
{
  return anderson_history_;
}

template <typename PointSource, typename PointTarget, typename Scalar>
void
FastRobustIterativeClosestPoint<PointSource, PointTarget, Scalar>::
    setDynamicWelschBeginRatio(double ratio)
{
  nu_begin_ratio_ = std::max(ratio, same_threshold_);
}

template <typename PointSource, typename PointTarget, typename Scalar>
void
FastRobustIterativeClosestPoint<PointSource, PointTarget, Scalar>::
    setDynamicWelschEndRatio(double ratio)
{
  nu_end_ratio_ = std::max(ratio, same_threshold_);
}

template <typename PointSource, typename PointTarget, typename Scalar>
void
FastRobustIterativeClosestPoint<PointSource, PointTarget, Scalar>::
    setDynamicWelschDecay(double ratio)
{
  nu_decay_ratio_ = std::max(0.0, std::min(1.0, ratio));
  if (nu_decay_ratio_ < same_threshold_)
    nu_decay_ratio_ = 0.5;
}

template <typename PointSource, typename PointTarget, typename Scalar>
void
FastRobustIterativeClosestPoint<PointSource, PointTarget, Scalar>::
    computeTransformation(PointCloudSource& output, const Matrix4& guess)
{
  pcl::IterativeClosestPoint<PointSource, PointTarget, Scalar>::initComputeReciprocal();

  if (!this->input_ || !this->target_) {
    PCL_ERROR("[pcl::%s::computeTransformation] Invalid input clouds.\n",
              this->getClassName().c_str());
    return;
  }

  if (this->input_->empty() || this->target_->empty()) {
    PCL_ERROR("[pcl::%s::computeTransformation] Empty input point clouds.\n",
              this->getClassName().c_str());
    return;
  }

  std::vector<int> source_indices;
  if (this->indices_ && !this->indices_->empty())
    source_indices.assign(this->indices_->begin(), this->indices_->end());
  else {
    source_indices.resize(this->input_->size());
    std::iota(source_indices.begin(), source_indices.end(), 0);
  }

  if (source_indices.size() <
      static_cast<std::size_t>(this->min_number_correspondences_)) {
    PCL_ERROR("[pcl::%s::computeTransformation] Not enough source points (%zu).\n",
              this->getClassName().c_str(),
              source_indices.size());
    return;
  }

  const std::size_t source_size = source_indices.size();
  const std::size_t target_size = this->target_->size();

  if (target_size < static_cast<std::size_t>(this->min_number_correspondences_)) {
    PCL_ERROR("[pcl::%s::computeTransformation] Not enough target points (%zu).\n",
              this->getClassName().c_str(),
              target_size);
    return;
  }

  Matrix3Xd source_mat(3, source_size);
  for (std::size_t i = 0; i < source_size; ++i) {
    const auto& pt = (*this->input_)[source_indices[i]];
    source_mat.col(i) = pt.getVector3fMap().template cast<double>();
  }
  Vector3d source_mean = source_mat.rowwise().mean();
  source_mat.colwise() -= source_mean;

  Matrix3Xd target_mat(3, target_size);
  for (std::size_t i = 0; i < target_size; ++i) {
    const auto& pt = (*this->target_)[i];
    target_mat.col(i) = pt.getVector3fMap().template cast<double>();
  }
  Vector3d target_mean = target_mat.rowwise().mean();
  target_mat.colwise() -= target_mean;

  pcl::PointCloud<pcl::PointXYZ>::Ptr target_centered(
      new pcl::PointCloud<pcl::PointXYZ>);
  target_centered->resize(target_size);
  for (std::size_t i = 0; i < target_size; ++i) {
    (*target_centered)[i].x = static_cast<float>(target_mat(0, i));
    (*target_centered)[i].y = static_cast<float>(target_mat(1, i));
    (*target_centered)[i].z = static_cast<float>(target_mat(2, i));
  }

  pcl::search::KdTree<pcl::PointXYZ> tree;
  tree.setInputCloud(target_centered);

  Matrix4d transform_centered = convertGuessToCentered(guess, source_mean, target_mean);
  Matrix4d svd_transform = transform_centered;
  Matrix4d previous_transform = transform_centered;

  Matrix3Xd matched_targets(3, source_size);
  VectorXd residuals(source_size);
  if (!updateCorrespondences(transform_centered,
                             source_mat,
                             target_mat,
                             tree,
                             matched_targets,
                             residuals)) {
    PCL_ERROR(
        "[pcl::%s::computeTransformation] Failed to initialize correspondences.\n",
        this->getClassName().c_str());
    return;
  }

  const bool use_welsch = (robust_function_ == RobustFunction::WELSCH);
  double nu_limit = 1.0;
  double nu_current = 1.0;
  if (use_welsch) {
    const double neighbor_med = findKNearestMedian(*target_centered, tree, 7);
    const double residual_med = computeMedian(residuals);
    nu_limit = std::max(nu_end_ratio_ * neighbor_med, same_threshold_);
    nu_current = std::max(nu_begin_ratio_ * residual_med, nu_limit);
  }

  const double stop_threshold = (this->transformation_epsilon_ > 0)
                                    ? static_cast<double>(this->transformation_epsilon_)
                                    : 1e-6;

  this->nr_iterations_ = 0;
  this->converged_ = false;

  double last_energy = std::numeric_limits<double>::max();
  if (use_anderson_) {
    const Matrix4d log_state = matrixLog(transform_centered);
    anderson_.init(anderson_history_, 16, log_state.data());
  }

  bool outer_done = false;
  bool converged = false;

  while (!outer_done) {
    for (int iter = 0; iter < this->max_iterations_; ++iter) {
      double energy =
          use_welsch ? computeEnergy(residuals, nu_current) : residuals.squaredNorm();
      if (use_anderson_) {
        if (energy <= last_energy) {
          last_energy = energy;
        }
        else {
          transform_centered = svd_transform;
          const Matrix4d log_state = matrixLog(transform_centered);
          anderson_.replace(log_state.data());
          if (!updateCorrespondences(transform_centered,
                                     source_mat,
                                     target_mat,
                                     tree,
                                     matched_targets,
                                     residuals)) {
            PCL_ERROR("[pcl::%s::computeTransformation] Unable to recompute "
                      "correspondences during fallback.\n",
                      this->getClassName().c_str());
            return;
          }
          energy = use_welsch ? computeEnergy(residuals, nu_current)
                              : residuals.squaredNorm();
          last_energy = energy;
        }
      }
      else {
        last_energy = energy;
      }

      VectorXd weights = use_welsch ? computeWeights(residuals, nu_current)
                                    : VectorXd::Ones(residuals.size());
      Matrix4d candidate =
          computeWeightedRigidTransform(source_mat, matched_targets, weights);
      svd_transform = candidate;
      transform_centered = candidate;

      if (use_anderson_) {
        const Matrix4d log_matrix = matrixLog(transform_centered);
        const Eigen::VectorXd& accelerated = anderson_.compute(log_matrix.data());
        transform_centered = Eigen::Map<const Matrix4d>(accelerated.data()).exp();
      }

      if (!updateCorrespondences(transform_centered,
                                 source_mat,
                                 target_mat,
                                 tree,
                                 matched_targets,
                                 residuals)) {
        PCL_ERROR("[pcl::%s::computeTransformation] Failed to update "
                  "correspondences.\n",
                  this->getClassName().c_str());
        return;
      }

      const double delta = (transform_centered - previous_transform).norm();
      previous_transform = transform_centered;
      ++this->nr_iterations_;

      if (delta < stop_threshold) {
        converged = true;
        break;
      }
    }

    if (!use_welsch || (std::abs(nu_current - nu_limit) < same_threshold_)) {
      outer_done = true;
    }
    else {
      nu_current = std::max(nu_current * nu_decay_ratio_, nu_limit);
      last_energy = std::numeric_limits<double>::max();
      if (use_anderson_) {
        const Matrix4d log_state = matrixLog(transform_centered);
        anderson_.reset(log_state.data());
      }
    }
  }

  this->converged_ = converged;

  const Matrix4d final_transform =
      convertCenteredToActual(transform_centered, source_mean, target_mean);
  this->final_transformation_ = final_transform.template cast<Scalar>();
  this->transformation_ = this->final_transformation_;
  this->previous_transformation_ = this->final_transformation_;

  pcl::transformPointCloud(*this->input_, output, this->final_transformation_);
}

template <typename PointSource, typename PointTarget, typename Scalar>
typename FastRobustIterativeClosestPoint<PointSource, PointTarget, Scalar>::Matrix4d
FastRobustIterativeClosestPoint<PointSource, PointTarget, Scalar>::
    convertGuessToCentered(const Matrix4& guess,
                           const Vector3d& source_mean,
                           const Vector3d& target_mean) const
{
  Matrix4d centered = Matrix4d::Identity();
  centered.block<3, 3>(0, 0) = guess.template block<3, 3>(0, 0).template cast<double>();
  Vector3d translation = guess.template block<3, 1>(0, 3).template cast<double>();
  centered.block<3, 1>(0, 3) =
      centered.block<3, 3>(0, 0) * source_mean + translation - target_mean;
  return centered;
}

template <typename PointSource, typename PointTarget, typename Scalar>
typename FastRobustIterativeClosestPoint<PointSource, PointTarget, Scalar>::Matrix4d
FastRobustIterativeClosestPoint<PointSource, PointTarget, Scalar>::
    convertCenteredToActual(const Matrix4d& transform,
                            const Vector3d& source_mean,
                            const Vector3d& target_mean) const
{
  Matrix4d actual = transform;
  actual.block<3, 1>(0, 3) = transform.block<3, 1>(0, 3) -
                             transform.block<3, 3>(0, 0) * source_mean + target_mean;
  return actual;
}

template <typename PointSource, typename PointTarget, typename Scalar>
bool
FastRobustIterativeClosestPoint<PointSource, PointTarget, Scalar>::
    updateCorrespondences(const Matrix4d& transform,
                          const Matrix3Xd& source,
                          const Matrix3Xd& target,
                          pcl::search::KdTree<pcl::PointXYZ>& tree,
                          Matrix3Xd& matched_targets,
                          VectorXd& residuals) const
{
  const Eigen::Matrix3d R = transform.block<3, 3>(0, 0);
  const Eigen::Vector3d t = transform.block<3, 1>(0, 3);
  pcl::PointXYZ query;
  pcl::Indices nn_indices(1);
  std::vector<float> nn_sqr_dists(1);

  for (int i = 0; i < source.cols(); ++i) {
    const Eigen::Vector3d current = R * source.col(i) + t;
    query.x = static_cast<float>(current.x());
    query.y = static_cast<float>(current.y());
    query.z = static_cast<float>(current.z());
    if (tree.nearestKSearch(query, 1, nn_indices, nn_sqr_dists) != 1)
      return false;
    const auto idx = nn_indices[0];
    matched_targets.col(i) = target.col(static_cast<int>(idx));
    residuals[i] = (current - matched_targets.col(i)).norm();
  }
  return true;
}

template <typename PointSource, typename PointTarget, typename Scalar>
double
FastRobustIterativeClosestPoint<PointSource, PointTarget, Scalar>::computeEnergy(
    const VectorXd& residuals, double nu) const
{
  if (nu < same_threshold_)
    nu = same_threshold_;
  const double denom = 2.0 * nu * nu;
  double energy = 0.0;
  for (int i = 0; i < residuals.size(); ++i) {
    const double dist2 = residuals[i] * residuals[i];
    energy += 1.0 - std::exp(-dist2 / denom);
  }
  return energy;
}

template <typename PointSource, typename PointTarget, typename Scalar>
typename FastRobustIterativeClosestPoint<PointSource, PointTarget, Scalar>::VectorXd
FastRobustIterativeClosestPoint<PointSource, PointTarget, Scalar>::computeWeights(
    const VectorXd& residuals, double nu) const
{
  if (nu < same_threshold_)
    nu = same_threshold_;
  const double denom = 2.0 * nu * nu;
  VectorXd weights(residuals.size());
  Eigen::Index idx = 0;
  for (const auto residual : residuals.array()) {
    const double dist2 = static_cast<double>(residual) * static_cast<double>(residual);
    weights[idx++] = std::exp(-dist2 / denom);
  }
  return weights;
}

template <typename PointSource, typename PointTarget, typename Scalar>
typename FastRobustIterativeClosestPoint<PointSource, PointTarget, Scalar>::Matrix4d
FastRobustIterativeClosestPoint<PointSource, PointTarget, Scalar>::
    computeWeightedRigidTransform(const Matrix3Xd& source,
                                  const Matrix3Xd& target,
                                  const VectorXd& weights) const
{
  Matrix4d transform = Matrix4d::Identity();
  VectorXd normalized = weights;
  const double sum = normalized.sum();
  if (sum <= same_threshold_) {
    normalized.setOnes();
    normalized /= static_cast<double>(normalized.size());
  }
  else {
    normalized /= sum;
  }

  const Vector3d source_mean = source * normalized;
  const Vector3d target_mean = target * normalized;
  const Matrix3Xd source_centered = source.colwise() - source_mean;
  const Matrix3Xd target_centered = target.colwise() - target_mean;
  Eigen::Matrix3d sigma =
      source_centered * normalized.asDiagonal() * target_centered.transpose();
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(sigma,
                                        Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d R = svd.matrixV() * svd.matrixU().transpose();
  if (R.determinant() < 0.0) {
    Eigen::Matrix3d V = svd.matrixV();
    V.col(2) *= -1.0;
    R = V * svd.matrixU().transpose();
  }
  transform.block<3, 3>(0, 0) = R;
  transform.block<3, 1>(0, 3) = target_mean - R * source_mean;
  return transform;
}

template <typename PointSource, typename PointTarget, typename Scalar>
double
FastRobustIterativeClosestPoint<PointSource, PointTarget, Scalar>::findKNearestMedian(
    const pcl::PointCloud<pcl::PointXYZ>& cloud,
    pcl::search::KdTree<pcl::PointXYZ>& tree,
    int neighbors) const
{
  if (cloud.empty() || neighbors < 2)
    return 0.0;

  const int k = std::min<int>(neighbors, static_cast<int>(cloud.size()));
  if (k < 2)
    return 0.0;

  std::vector<double> local_medians(cloud.size(), 0.0);
  pcl::Indices nn_indices(k);
  std::vector<float> nn_sqr_dists(k);
  std::vector<double> dists;
  dists.reserve(k - 1);

  for (std::size_t i = 0; i < cloud.size(); ++i) {
    if (tree.nearestKSearch(cloud[i], k, nn_indices, nn_sqr_dists) != k)
      continue;
    dists.clear();
    for (int j = 1; j < k; ++j)
      dists.push_back(std::sqrt(nn_sqr_dists[j]));
    local_medians[i] = computeMedian(dists);
  }

  return computeMedian(std::move(local_medians));
}

template <typename PointSource, typename PointTarget, typename Scalar>
double
FastRobustIterativeClosestPoint<PointSource, PointTarget, Scalar>::computeMedian(
    const VectorXd& values) const
{
  std::vector<double> buffer(static_cast<std::size_t>(values.size()));
  for (int i = 0; i < values.size(); ++i)
    buffer[static_cast<std::size_t>(i)] = values[i];
  return computeMedian(std::move(buffer));
}

template <typename PointSource, typename PointTarget, typename Scalar>
double
FastRobustIterativeClosestPoint<PointSource, PointTarget, Scalar>::computeMedian(
    std::vector<double> values) const
{
  if (values.empty())
    return 0.0;
  auto mid = values.begin() + values.size() / 2;
  std::nth_element(values.begin(), mid, values.end());
  if (values.size() % 2 == 1)
    return *mid;
  auto mid_prev = values.begin() + values.size() / 2 - 1;
  std::nth_element(values.begin(), mid_prev, values.end());
  return (*mid + *mid_prev) * 0.5;
}

template <typename PointSource, typename PointTarget, typename Scalar>
typename FastRobustIterativeClosestPoint<PointSource, PointTarget, Scalar>::Matrix4d
FastRobustIterativeClosestPoint<PointSource, PointTarget, Scalar>::matrixLog(
    const Matrix4d& transform) const
{
  Eigen::RealSchur<Matrix4d> schur(transform);
  const Matrix4d U = schur.matrixU();
  const Matrix4d R = schur.matrixT();
  std::array<bool, 3> selected{{true, true, true}};
  Eigen::Matrix3d B = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d V = Eigen::Matrix3d::Identity();

  for (int i = 0; i < 3; ++i) {
    if (!selected[i])
      continue;
    if (std::abs(R(i, i) - 1.0) <= same_threshold_)
      continue;
    int partner = -1;
    for (int j = i + 1; j < 3; ++j) {
      if (std::abs(R(j, j) - R(i, i)) < same_threshold_) {
        partner = j;
        selected[j] = false;
        break;
      }
    }
    if (partner > 0) {
      selected[i] = false;
      const double diag = std::max(-1.0, std::min(1.0, static_cast<double>(R(i, i))));
      double theta = std::acos(diag);
      if (R(i, partner) < 0.0)
        theta = -theta;
      B(i, partner) += theta;
      B(partner, i) -= theta;
      V(i, partner) -= theta / 2.0;
      V(partner, i) += theta / 2.0;
      const double denom = 2.0 * (1.0 - R(i, i));
      if (std::abs(denom) > same_threshold_) {
        const double coeff = 1.0 - (theta * R(i, partner)) / denom;
        V(i, i) -= coeff;
        V(partner, partner) -= coeff;
      }
    }
  }

  Matrix4d trimmed = Matrix4d::Zero();
  trimmed.block<3, 3>(0, 0) = B;
  trimmed.block<3, 1>(0, 3) = V * R.block<3, 1>(0, 3);
  return U * trimmed * U.transpose();
}

} // namespace pcl

#endif // PCL_REGISTRATION_IMPL_FRICP_HPP_
