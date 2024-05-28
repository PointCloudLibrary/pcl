/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010, Willow Garage, Inc.
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
 * $Id$
 *
 */

#ifndef PCL_REGISTRATION_IMPL_GICP_HPP_
#define PCL_REGISTRATION_IMPL_GICP_HPP_

#include <pcl/registration/exceptions.h>

namespace pcl {

template <typename PointSource, typename PointTarget, typename Scalar>
void
GeneralizedIterativeClosestPoint<PointSource, PointTarget, Scalar>::setNumberOfThreads(
    unsigned int nr_threads)
{
#ifdef _OPENMP
  if (nr_threads == 0)
    threads_ = omp_get_num_procs();
  else
    threads_ = nr_threads;
  PCL_DEBUG("[pcl::GeneralizedIterativeClosestPoint::setNumberOfThreads] Setting "
            "number of threads to %u.\n",
            threads_);
#else
  threads_ = 1;
  if (nr_threads != 1)
    PCL_WARN("[pcl::GeneralizedIterativeClosestPoint::setNumberOfThreads] "
             "Parallelization is requested, but OpenMP is not available! Continuing "
             "without parallelization.\n");
#endif // _OPENMP
}

template <typename PointSource, typename PointTarget, typename Scalar>
template <typename PointT>
void
GeneralizedIterativeClosestPoint<PointSource, PointTarget, Scalar>::computeCovariances(
    typename pcl::PointCloud<PointT>::ConstPtr cloud,
    const typename pcl::search::KdTree<PointT>::Ptr kdtree,
    MatricesVector& cloud_covariances)
{
  if (k_correspondences_ > static_cast<int>(cloud->size())) {
    PCL_ERROR("[pcl::GeneralizedIterativeClosestPoint::computeCovariances] Number or "
              "points in cloud (%lu) is less than k_correspondences_ (%lu)!\n",
              cloud->size(),
              k_correspondences_);
    return;
  }

  Eigen::Vector3d mean;
  Eigen::Matrix3d cov;
  pcl::Indices nn_indices(k_correspondences_);
  std::vector<float> nn_dist_sq(k_correspondences_);

  // We should never get there but who knows
  if (cloud_covariances.size() < cloud->size())
    cloud_covariances.resize(cloud->size());

#pragma omp parallel for default(none) num_threads(threads_) schedule(dynamic, 32)     \
    shared(cloud, cloud_covariances, kdtree)                                           \
        firstprivate(mean, cov, nn_indices, nn_dist_sq)
  for (std::ptrdiff_t i = 0; i < static_cast<std::ptrdiff_t>(cloud->size()); ++i) {
    const PointT& query_point = (*cloud)[i];
    // Zero out the cov and mean
    cov.setZero();
    mean.setZero();

    // Search for the K nearest neighbours
    kdtree->nearestKSearch(query_point, k_correspondences_, nn_indices, nn_dist_sq);

    // Find the covariance matrix
    for (int j = 0; j < k_correspondences_; j++) {
      // de-mean neighbourhood to avoid inaccuracies when far away from origin
      const double ptx = (*cloud)[nn_indices[j]].x - query_point.x,
                   pty = (*cloud)[nn_indices[j]].y - query_point.y,
                   ptz = (*cloud)[nn_indices[j]].z - query_point.z;

      mean[0] += ptx;
      mean[1] += pty;
      mean[2] += ptz;

      cov(0, 0) += ptx * ptx;

      cov(1, 0) += pty * ptx;
      cov(1, 1) += pty * pty;

      cov(2, 0) += ptz * ptx;
      cov(2, 1) += ptz * pty;
      cov(2, 2) += ptz * ptz;
    }

    mean /= static_cast<double>(k_correspondences_);
    // Get the actual covariance
    for (int k = 0; k < 3; k++)
      for (int l = 0; l <= k; l++) {
        cov(k, l) /= static_cast<double>(k_correspondences_);
        cov(k, l) -= mean[k] * mean[l];
        cov(l, k) = cov(k, l);
      }

    // Compute the SVD (covariance matrix is symmetric so U = V')
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(cov, Eigen::ComputeFullU);
    cov.setZero();
    Eigen::Matrix3d U = svd.matrixU();
    // Reconstitute the covariance matrix with modified singular values using the column
    // // vectors in V.
    for (int k = 0; k < 3; k++) {
      Eigen::Vector3d col = U.col(k);
      double v = 1.; // biggest 2 singular values replaced by 1
      if (k == 2)    // smallest singular value replaced by gicp_epsilon
        v = gicp_epsilon_;
      cov += v * col * col.transpose();
    }
    cloud_covariances[i] = cov;
  }
}

template <typename PointSource, typename PointTarget, typename Scalar>
void
GeneralizedIterativeClosestPoint<PointSource, PointTarget, Scalar>::getRDerivatives(
    double phi,
    double theta,
    double psi,
    Eigen::Matrix3d& dR_dPhi,
    Eigen::Matrix3d& dR_dTheta,
    Eigen::Matrix3d& dR_dPsi) const
{
  const double cphi = std::cos(phi), sphi = std::sin(phi);
  const double ctheta = std::cos(theta), stheta = std::sin(theta);
  const double cpsi = std::cos(psi), spsi = std::sin(psi);
  dR_dPhi(0, 0) = 0.;
  dR_dPhi(1, 0) = 0.;
  dR_dPhi(2, 0) = 0.;

  dR_dPhi(0, 1) = sphi * spsi + cphi * cpsi * stheta;
  dR_dPhi(1, 1) = -cpsi * sphi + cphi * spsi * stheta;
  dR_dPhi(2, 1) = cphi * ctheta;

  dR_dPhi(0, 2) = cphi * spsi - cpsi * sphi * stheta;
  dR_dPhi(1, 2) = -cphi * cpsi - sphi * spsi * stheta;
  dR_dPhi(2, 2) = -ctheta * sphi;

  dR_dTheta(0, 0) = -cpsi * stheta;
  dR_dTheta(1, 0) = -spsi * stheta;
  dR_dTheta(2, 0) = -ctheta;

  dR_dTheta(0, 1) = cpsi * ctheta * sphi;
  dR_dTheta(1, 1) = ctheta * sphi * spsi;
  dR_dTheta(2, 1) = -sphi * stheta;

  dR_dTheta(0, 2) = cphi * cpsi * ctheta;
  dR_dTheta(1, 2) = cphi * ctheta * spsi;
  dR_dTheta(2, 2) = -cphi * stheta;

  dR_dPsi(0, 0) = -ctheta * spsi;
  dR_dPsi(1, 0) = cpsi * ctheta;
  dR_dPsi(2, 0) = 0.;

  dR_dPsi(0, 1) = -cphi * cpsi - sphi * spsi * stheta;
  dR_dPsi(1, 1) = -cphi * spsi + cpsi * sphi * stheta;
  dR_dPsi(2, 1) = 0.;

  dR_dPsi(0, 2) = cpsi * sphi - cphi * spsi * stheta;
  dR_dPsi(1, 2) = sphi * spsi + cphi * cpsi * stheta;
  dR_dPsi(2, 2) = 0.;
}

template <typename PointSource, typename PointTarget, typename Scalar>
void
GeneralizedIterativeClosestPoint<PointSource, PointTarget, Scalar>::computeRDerivative(
    const Vector6d& x, const Eigen::Matrix3d& dCost_dR_T, Vector6d& g) const
{
  Eigen::Matrix3d dR_dPhi;
  Eigen::Matrix3d dR_dTheta;
  Eigen::Matrix3d dR_dPsi;
  getRDerivatives(x[3], x[4], x[5], dR_dPhi, dR_dTheta, dR_dPsi);

  g[3] = (dR_dPhi * dCost_dR_T).trace();
  g[4] = (dR_dTheta * dCost_dR_T).trace();
  g[5] = (dR_dPsi * dCost_dR_T).trace();
}

template <typename PointSource, typename PointTarget, typename Scalar>
void
GeneralizedIterativeClosestPoint<PointSource, PointTarget, Scalar>::getR2ndDerivatives(
    double phi,
    double theta,
    double psi,
    Eigen::Matrix3d& ddR_dPhi_dPhi,
    Eigen::Matrix3d& ddR_dPhi_dTheta,
    Eigen::Matrix3d& ddR_dPhi_dPsi,
    Eigen::Matrix3d& ddR_dTheta_dTheta,
    Eigen::Matrix3d& ddR_dTheta_dPsi,
    Eigen::Matrix3d& ddR_dPsi_dPsi) const
{
  const double sphi = std::sin(phi), stheta = std::sin(theta), spsi = std::sin(psi);
  const double cphi = std::cos(phi), ctheta = std::cos(theta), cpsi = std::cos(psi);
  ddR_dPhi_dPhi(0, 0) = 0.0;
  ddR_dPhi_dPhi(1, 0) = 0.0;
  ddR_dPhi_dPhi(2, 0) = 0.0;
  ddR_dPhi_dPhi(0, 1) = -cpsi * stheta * sphi + spsi * cphi;
  ddR_dPhi_dPhi(1, 1) = -cpsi * cphi - spsi * stheta * sphi;
  ddR_dPhi_dPhi(2, 1) = -ctheta * sphi;
  ddR_dPhi_dPhi(0, 2) = -spsi * sphi - cpsi * stheta * cphi;
  ddR_dPhi_dPhi(1, 2) = -spsi * stheta * cphi + cpsi * sphi;
  ddR_dPhi_dPhi(2, 2) = -ctheta * cphi;

  ddR_dPhi_dTheta(0, 0) = 0.0;
  ddR_dPhi_dTheta(1, 0) = 0.0;
  ddR_dPhi_dTheta(2, 0) = 0.0;
  ddR_dPhi_dTheta(0, 1) = cpsi * ctheta * cphi;
  ddR_dPhi_dTheta(1, 1) = spsi * ctheta * cphi;
  ddR_dPhi_dTheta(2, 1) = -stheta * cphi;
  ddR_dPhi_dTheta(0, 2) = -cpsi * ctheta * sphi;
  ddR_dPhi_dTheta(1, 2) = -spsi * ctheta * sphi;
  ddR_dPhi_dTheta(2, 2) = stheta * sphi;

  ddR_dPhi_dPsi(0, 0) = 0.0;
  ddR_dPhi_dPsi(1, 0) = 0.0;
  ddR_dPhi_dPsi(2, 0) = 0.0;
  ddR_dPhi_dPsi(0, 1) = -spsi * stheta * cphi + cpsi * sphi;
  ddR_dPhi_dPsi(1, 1) = spsi * sphi + cpsi * stheta * cphi;
  ddR_dPhi_dPsi(2, 1) = 0.0;
  ddR_dPhi_dPsi(0, 2) = cpsi * cphi + spsi * stheta * sphi;
  ddR_dPhi_dPsi(1, 2) = -cpsi * stheta * sphi + spsi * cphi;
  ddR_dPhi_dPsi(2, 2) = 0.0;

  ddR_dTheta_dTheta(0, 0) = -cpsi * ctheta;
  ddR_dTheta_dTheta(1, 0) = -spsi * ctheta;
  ddR_dTheta_dTheta(2, 0) = stheta;
  ddR_dTheta_dTheta(0, 1) = -cpsi * stheta * sphi;
  ddR_dTheta_dTheta(1, 1) = -spsi * stheta * sphi;
  ddR_dTheta_dTheta(2, 1) = -ctheta * sphi;
  ddR_dTheta_dTheta(0, 2) = -cpsi * stheta * cphi;
  ddR_dTheta_dTheta(1, 2) = -spsi * stheta * cphi;
  ddR_dTheta_dTheta(2, 2) = -ctheta * cphi;

  ddR_dTheta_dPsi(0, 0) = spsi * stheta;
  ddR_dTheta_dPsi(1, 0) = -cpsi * stheta;
  ddR_dTheta_dPsi(2, 0) = 0.0;
  ddR_dTheta_dPsi(0, 1) = -spsi * ctheta * sphi;
  ddR_dTheta_dPsi(1, 1) = cpsi * ctheta * sphi;
  ddR_dTheta_dPsi(2, 1) = 0.0;
  ddR_dTheta_dPsi(0, 2) = -spsi * ctheta * cphi;
  ddR_dTheta_dPsi(1, 2) = cpsi * ctheta * cphi;
  ddR_dTheta_dPsi(2, 2) = 0.0;

  ddR_dPsi_dPsi(0, 0) = -cpsi * ctheta;
  ddR_dPsi_dPsi(1, 0) = -spsi * ctheta;
  ddR_dPsi_dPsi(2, 0) = 0.0;
  ddR_dPsi_dPsi(0, 1) = -cpsi * stheta * sphi + spsi * cphi;
  ddR_dPsi_dPsi(1, 1) = -cpsi * cphi - spsi * stheta * sphi;
  ddR_dPsi_dPsi(2, 1) = 0.0;
  ddR_dPsi_dPsi(0, 2) = -spsi * sphi - cpsi * stheta * cphi;
  ddR_dPsi_dPsi(1, 2) = -spsi * stheta * cphi + cpsi * sphi;
  ddR_dPsi_dPsi(2, 2) = 0.0;
}

template <typename PointSource, typename PointTarget, typename Scalar>
void
GeneralizedIterativeClosestPoint<PointSource, PointTarget, Scalar>::
    estimateRigidTransformationBFGS(const PointCloudSource& cloud_src,
                                    const pcl::Indices& indices_src,
                                    const PointCloudTarget& cloud_tgt,
                                    const pcl::Indices& indices_tgt,
                                    Matrix4& transformation_matrix)
{
  // need at least min_number_correspondences_ samples
  if (indices_src.size() < min_number_correspondences_) {
    PCL_THROW_EXCEPTION(
        NotEnoughPointsException,
        "[pcl::GeneralizedIterativeClosestPoint::estimateRigidTransformationBFGS] Need "
        "at least "
            << min_number_correspondences_
            << " points to estimate a transform! "
               "Source and target have "
            << indices_src.size() << " points!");
    return;
  }
  // Set the initial solution
  Vector6d x = Vector6d::Zero();
  // translation part
  x[0] = transformation_matrix(0, 3);
  x[1] = transformation_matrix(1, 3);
  x[2] = transformation_matrix(2, 3);
  // rotation part (Z Y X euler angles convention)
  // see: https://en.wikipedia.org/wiki/Rotation_matrix#General_rotations
  x[3] = std::atan2(transformation_matrix(2, 1), transformation_matrix(2, 2));
  x[4] = asin(-transformation_matrix(2, 0));
  x[5] = std::atan2(transformation_matrix(1, 0), transformation_matrix(0, 0));

  // Set temporary pointers
  tmp_src_ = &cloud_src;
  tmp_tgt_ = &cloud_tgt;
  tmp_idx_src_ = &indices_src;
  tmp_idx_tgt_ = &indices_tgt;

  // Optimize using BFGS
  OptimizationFunctorWithIndices functor(this);
  BFGS<OptimizationFunctorWithIndices> bfgs(functor);
  bfgs.parameters.sigma = 0.01;
  bfgs.parameters.rho = 0.01;
  bfgs.parameters.tau1 = 9;
  bfgs.parameters.tau2 = 0.05;
  bfgs.parameters.tau3 = 0.5;
  bfgs.parameters.order = 3;

  int inner_iterations_ = 0;
  int result = bfgs.minimizeInit(x);
  result = BFGSSpace::Running;
  do {
    inner_iterations_++;
    result = bfgs.minimizeOneStep(x);
    if (result) {
      break;
    }
    result = bfgs.testGradient();
  } while (result == BFGSSpace::Running && inner_iterations_ < max_inner_iterations_);
  if (result == BFGSSpace::NoProgress || result == BFGSSpace::Success ||
      inner_iterations_ == max_inner_iterations_) {
    PCL_DEBUG("[pcl::registration::TransformationEstimationBFGS::"
              "estimateRigidTransformation]");
    PCL_DEBUG("BFGS solver finished with exit code %i \n", result);
    transformation_matrix.setIdentity();
    applyState(transformation_matrix, x);
  }
  else
    PCL_THROW_EXCEPTION(
        SolverDidntConvergeException,
        "[pcl::" << getClassName()
                 << "::TransformationEstimationBFGS::estimateRigidTransformation] BFGS "
                    "solver didn't converge!");
}

template <typename PointSource, typename PointTarget, typename Scalar>
void
GeneralizedIterativeClosestPoint<PointSource, PointTarget, Scalar>::
    estimateRigidTransformationNewton(const PointCloudSource& cloud_src,
                                      const pcl::Indices& indices_src,
                                      const PointCloudTarget& cloud_tgt,
                                      const pcl::Indices& indices_tgt,
                                      Matrix4& transformation_matrix)
{
  //  need at least min_number_correspondences_ samples
  if (indices_src.size() < min_number_correspondences_) {
    PCL_THROW_EXCEPTION(NotEnoughPointsException,
                        "[pcl::GeneralizedIterativeClosestPoint::"
                        "estimateRigidTransformationNewton] Need "
                        "at least "
                            << min_number_correspondences_
                            << " points to estimate a transform! "
                               "Source and target have "
                            << indices_src.size() << " points!");
    return;
  }
  // Set the initial solution
  Vector6d x = Vector6d::Zero();
  // translation part
  x[0] = transformation_matrix(0, 3);
  x[1] = transformation_matrix(1, 3);
  x[2] = transformation_matrix(2, 3);
  // rotation part (Z Y X euler angles convention)
  // see: https://en.wikipedia.org/wiki/Rotation_matrix#General_rotations
  x[3] = std::atan2(transformation_matrix(2, 1), transformation_matrix(2, 2));
  x[4] = std::asin(
      std::min<double>(1.0, std::max<double>(-1.0, -transformation_matrix(2, 0))));
  x[5] = std::atan2(transformation_matrix(1, 0), transformation_matrix(0, 0));

  // Set temporary pointers
  tmp_src_ = &cloud_src;
  tmp_tgt_ = &cloud_tgt;
  tmp_idx_src_ = &indices_src;
  tmp_idx_tgt_ = &indices_tgt;

  // Optimize using Newton
  OptimizationFunctorWithIndices functor(this);
  Eigen::Matrix<double, 6, 6> hessian;
  Eigen::Matrix<double, 6, 1> gradient;
  double current_x_value = functor(x);
  functor.dfddf(x, gradient, hessian);
  Eigen::Matrix<double, 6, 1> delta;
  int inner_iterations_ = 0;
  do {
    ++inner_iterations_;
    // compute descent direction from hessian and gradient. Take special measures if
    // hessian is not positive-definite (positive Eigenvalues)
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> eigensolver(hessian);
    Eigen::Matrix<double, 6, 6> inverted_eigenvalues =
        Eigen::Matrix<double, 6, 6>::Zero();
    for (int i = 0; i < 6; ++i) {
      const double ev = eigensolver.eigenvalues()[i];
      if (ev < 0)
        inverted_eigenvalues(i, i) = 1.0 / eigensolver.eigenvalues()[5];
      else
        inverted_eigenvalues(i, i) = 1.0 / ev;
    }
    delta = eigensolver.eigenvectors() * inverted_eigenvalues *
            eigensolver.eigenvectors().transpose() * gradient;

    // simple line search to guarantee a decrease in the function value
    double alpha = 1.0;
    double candidate_x_value;
    bool improvement_found = false;
    for (int i = 0; i < 10; ++i, alpha /= 2) {
      Vector6d candidate_x = x - alpha * delta;
      candidate_x_value = functor(candidate_x);
      if (candidate_x_value < current_x_value) {
        PCL_DEBUG("[estimateRigidTransformationNewton] Using stepsize=%g, function "
                  "value previously: %g, now: %g, "
                  "improvement: %g\n",
                  alpha,
                  current_x_value,
                  candidate_x_value,
                  current_x_value - candidate_x_value);
        x = candidate_x;
        current_x_value = candidate_x_value;
        improvement_found = true;
        break;
      }
    }
    if (!improvement_found) {
      PCL_DEBUG("[estimateRigidTransformationNewton] finishing because no progress\n");
      break;
    }
    functor.dfddf(x, gradient, hessian);
    if (gradient.head<3>().norm() < translation_gradient_tolerance_ &&
        gradient.tail<3>().norm() < rotation_gradient_tolerance_) {
      PCL_DEBUG("[estimateRigidTransformationNewton] finishing because gradient below "
                "threshold: translation: %g<%g, rotation: %g<%g\n",
                gradient.head<3>().norm(),
                translation_gradient_tolerance_,
                gradient.tail<3>().norm(),
                rotation_gradient_tolerance_);
      break;
    }
  } while (inner_iterations_ < max_inner_iterations_);
  PCL_DEBUG("[estimateRigidTransformationNewton] solver finished after %i iterations "
            "(of max %i)\n",
            inner_iterations_,
            max_inner_iterations_);
  transformation_matrix.setIdentity();
  applyState(transformation_matrix, x);
}

template <typename PointSource, typename PointTarget, typename Scalar>
inline double
GeneralizedIterativeClosestPoint<PointSource, PointTarget, Scalar>::
    OptimizationFunctorWithIndices::operator()(const Vector6d& x)
{
  Matrix4 transformation_matrix = gicp_->base_transformation_;
  gicp_->applyState(transformation_matrix, x);
  double f = 0;
  int m = static_cast<int>(gicp_->tmp_idx_src_->size());
  for (int i = 0; i < m; ++i) {
    // The last coordinate, p_src[3] is guaranteed to be set to 1.0 in registration.hpp
    Vector4fMapConst p_src =
        (*gicp_->tmp_src_)[(*gicp_->tmp_idx_src_)[i]].getVector4fMap();
    // The last coordinate, p_tgt[3] is guaranteed to be set to 1.0 in registration.hpp
    Vector4fMapConst p_tgt =
        (*gicp_->tmp_tgt_)[(*gicp_->tmp_idx_tgt_)[i]].getVector4fMap();
    Eigen::Vector4f p_trans_src(transformation_matrix.template cast<float>() * p_src);
    // Estimate the distance (cost function)
    // The last coordinate is still guaranteed to be set to 1.0
    // The d here is the negative of the d in the paper
    Eigen::Vector3d d(p_trans_src[0] - p_tgt[0],
                      p_trans_src[1] - p_tgt[1],
                      p_trans_src[2] - p_tgt[2]);
    Eigen::Vector3d Md(gicp_->mahalanobis((*gicp_->tmp_idx_src_)[i]) * d);
    // increment= d'*Md/num_matches = d'*M*d/num_matches (we postpone
    // 1/num_matches after the loop closes)
    f += static_cast<double>(d.transpose() * Md);
  }
  return f / m;
}

template <typename PointSource, typename PointTarget, typename Scalar>
inline void
GeneralizedIterativeClosestPoint<PointSource, PointTarget, Scalar>::
    OptimizationFunctorWithIndices::df(const Vector6d& x, Vector6d& g)
{
  Matrix4 transformation_matrix = gicp_->base_transformation_;
  gicp_->applyState(transformation_matrix, x);
  // Zero out g
  g.setZero();
  // Eigen::Vector3d g_t = g.head<3> ();
  // the transpose of the derivative of the cost function w.r.t rotation matrix
  Eigen::Matrix3d dCost_dR_T = Eigen::Matrix3d::Zero();
  int m = static_cast<int>(gicp_->tmp_idx_src_->size());
  for (int i = 0; i < m; ++i) {
    // The last coordinate, p_src[3] is guaranteed to be set to 1.0 in registration.hpp
    Vector4fMapConst p_src =
        (*gicp_->tmp_src_)[(*gicp_->tmp_idx_src_)[i]].getVector4fMap();
    // The last coordinate, p_tgt[3] is guaranteed to be set to 1.0 in registration.hpp
    Vector4fMapConst p_tgt =
        (*gicp_->tmp_tgt_)[(*gicp_->tmp_idx_tgt_)[i]].getVector4fMap();

    Eigen::Vector4f p_trans_src(transformation_matrix.template cast<float>() * p_src);
    // The last coordinate is still guaranteed to be set to 1.0
    // The d here is the negative of the d in the paper
    Eigen::Vector3d d(p_trans_src[0] - p_tgt[0],
                      p_trans_src[1] - p_tgt[1],
                      p_trans_src[2] - p_tgt[2]);
    // Md = M*d
    Eigen::Vector3d Md(gicp_->mahalanobis((*gicp_->tmp_idx_src_)[i]) * d);
    // Increment translation gradient
    // g.head<3> ()+= 2*M*d/num_matches (we postpone 2/num_matches after the loop
    // closes)
    g.head<3>() += Md;
    // Increment rotation gradient
    p_trans_src = gicp_->base_transformation_.template cast<float>() * p_src;
    Eigen::Vector3d p_base_src(p_trans_src[0], p_trans_src[1], p_trans_src[2]);
    dCost_dR_T += p_base_src * Md.transpose();
  }
  g.head<3>() *= 2.0 / m;
  dCost_dR_T *= 2.0 / m;
  gicp_->computeRDerivative(x, dCost_dR_T, g);
}

template <typename PointSource, typename PointTarget, typename Scalar>
inline void
GeneralizedIterativeClosestPoint<PointSource, PointTarget, Scalar>::
    OptimizationFunctorWithIndices::fdf(const Vector6d& x, double& f, Vector6d& g)
{
  Matrix4 transformation_matrix = gicp_->base_transformation_;
  gicp_->applyState(transformation_matrix, x);
  f = 0;
  g.setZero();
  // the transpose of the derivative of the cost function w.r.t rotation matrix
  Eigen::Matrix3d dCost_dR_T = Eigen::Matrix3d::Zero();
  const int m = static_cast<int>(gicp_->tmp_idx_src_->size());
  for (int i = 0; i < m; ++i) {
    // The last coordinate, p_src[3] is guaranteed to be set to 1.0 in registration.hpp
    Vector4fMapConst p_src =
        (*gicp_->tmp_src_)[(*gicp_->tmp_idx_src_)[i]].getVector4fMap();
    // The last coordinate, p_tgt[3] is guaranteed to be set to 1.0 in registration.hpp
    Vector4fMapConst p_tgt =
        (*gicp_->tmp_tgt_)[(*gicp_->tmp_idx_tgt_)[i]].getVector4fMap();
    Eigen::Vector4f p_trans_src(transformation_matrix.template cast<float>() * p_src);
    // The last coordinate is still guaranteed to be set to 1.0
    // The d here is the negative of the d in the paper
    Eigen::Vector3d d(p_trans_src[0] - p_tgt[0],
                      p_trans_src[1] - p_tgt[1],
                      p_trans_src[2] - p_tgt[2]);
    // Md = M*d
    Eigen::Vector3d Md(gicp_->mahalanobis((*gicp_->tmp_idx_src_)[i]) * d);
    // Increment total error
    f += static_cast<double>(d.transpose() * Md);
    // Increment translation gradient
    // g.head<3> ()+= 2*M*d/num_matches (we postpone 2/num_matches after the loop
    // closes)
    g.head<3>() += Md;
    p_trans_src = gicp_->base_transformation_.template cast<float>() * p_src;
    Eigen::Vector3d p_base_src(p_trans_src[0], p_trans_src[1], p_trans_src[2]);
    // Increment rotation gradient
    dCost_dR_T += p_base_src * Md.transpose();
  }
  f /= static_cast<double>(m);
  g.head<3>() *= (2.0 / m);
  dCost_dR_T *= 2.0 / m;
  gicp_->computeRDerivative(x, dCost_dR_T, g);
}

template <typename PointSource, typename PointTarget, typename Scalar>
inline void
GeneralizedIterativeClosestPoint<PointSource, PointTarget, Scalar>::
    OptimizationFunctorWithIndices::dfddf(const Vector6d& x,
                                          Vector6d& gradient,
                                          Matrix6d& hessian)
{
  Matrix4 transformation_matrix = gicp_->base_transformation_;
  gicp_->applyState(transformation_matrix, x);
  const Eigen::Matrix4f transformation_matrix_float =
      transformation_matrix.template cast<float>();
  const Eigen::Matrix4f base_transformation_float =
      gicp_->base_transformation_.template cast<float>();
  // Zero out gradient and hessian
  gradient.setZero();
  hessian.setZero();
  // Helper matrices
  Eigen::Matrix3d dR_dPhi;
  Eigen::Matrix3d dR_dTheta;
  Eigen::Matrix3d dR_dPsi;
  gicp_->getRDerivatives(x[3], x[4], x[5], dR_dPhi, dR_dTheta, dR_dPsi);
  Eigen::Matrix3d ddR_dPhi_dPhi;
  Eigen::Matrix3d ddR_dPhi_dTheta;
  Eigen::Matrix3d ddR_dPhi_dPsi;
  Eigen::Matrix3d ddR_dTheta_dTheta;
  Eigen::Matrix3d ddR_dTheta_dPsi;
  Eigen::Matrix3d ddR_dPsi_dPsi;
  gicp_->getR2ndDerivatives(x[3],
                            x[4],
                            x[5],
                            ddR_dPhi_dPhi,
                            ddR_dPhi_dTheta,
                            ddR_dPhi_dPsi,
                            ddR_dTheta_dTheta,
                            ddR_dTheta_dPsi,
                            ddR_dPsi_dPsi);
  Eigen::Matrix3d dCost_dR_T = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d dCost_dR_T1 = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d dCost_dR_T2 = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d dCost_dR_T3 = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d dCost_dR_T1b = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d dCost_dR_T2b = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d dCost_dR_T3b = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d hessian_rot_phi = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d hessian_rot_theta = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d hessian_rot_psi = Eigen::Matrix3d::Zero();
  Eigen::Matrix<double, 9, 6> hessian_rot_tmp = Eigen::Matrix<double, 9, 6>::Zero();

  int m = static_cast<int>(gicp_->tmp_idx_src_->size());
  for (int i = 0; i < m; ++i) {
    // The last coordinate, p_src[3] is guaranteed to be set to 1.0 in registration.hpp
    const auto& src_idx = (*gicp_->tmp_idx_src_)[i];
    Vector4fMapConst p_src = (*gicp_->tmp_src_)[src_idx].getVector4fMap();
    // The last coordinate, p_tgt[3] is guaranteed to be set to 1.0 in registration.hpp
    Vector4fMapConst p_tgt =
        (*gicp_->tmp_tgt_)[(*gicp_->tmp_idx_tgt_)[i]].getVector4fMap();
    Eigen::Vector4f p_trans_src(transformation_matrix_float * p_src);
    // The last coordinate is still guaranteed to be set to 1.0
    // The d here is the negative of the d in the paper
    const Eigen::Vector3d d(p_trans_src[0] - p_tgt[0],
                            p_trans_src[1] - p_tgt[1],
                            p_trans_src[2] - p_tgt[2]);
    const Eigen::Matrix3d& M = gicp_->mahalanobis(src_idx);
    const Eigen::Vector3d Md(M * d);    // Md = M*d
    gradient.head<3>() += Md;           // translation gradient
    hessian.topLeftCorner<3, 3>() += M; // translation-translation hessian
    p_trans_src = base_transformation_float * p_src;
    const Eigen::Vector3d p_base_src(p_trans_src[0], p_trans_src[1], p_trans_src[2]);
    dCost_dR_T.noalias() += p_base_src * Md.transpose();
    dCost_dR_T1b += p_base_src[0] * M;
    dCost_dR_T2b += p_base_src[1] * M;
    dCost_dR_T3b += p_base_src[2] * M;
    hessian_rot_tmp.noalias() +=
        Eigen::Map<const Eigen::Matrix<double, 9, 1>>{M.data()} *
        (Eigen::Matrix<double, 1, 6>() << p_base_src[0] * p_base_src[0],
         p_base_src[0] * p_base_src[1],
         p_base_src[0] * p_base_src[2],
         p_base_src[1] * p_base_src[1],
         p_base_src[1] * p_base_src[2],
         p_base_src[2] * p_base_src[2])
            .finished();
  }
  gradient.head<3>() *= 2.0 / m; // translation gradient
  dCost_dR_T *= 2.0 / m;
  gicp_->computeRDerivative(x, dCost_dR_T, gradient); // rotation gradient
  hessian.topLeftCorner<3, 3>() *= 2.0 / m;           // translation-translation hessian
  // translation-rotation hessian
  dCost_dR_T1.row(0) = dCost_dR_T1b.col(0);
  dCost_dR_T1.row(1) = dCost_dR_T2b.col(0);
  dCost_dR_T1.row(2) = dCost_dR_T3b.col(0);
  dCost_dR_T2.row(0) = dCost_dR_T1b.col(1);
  dCost_dR_T2.row(1) = dCost_dR_T2b.col(1);
  dCost_dR_T2.row(2) = dCost_dR_T3b.col(1);
  dCost_dR_T3.row(0) = dCost_dR_T1b.col(2);
  dCost_dR_T3.row(1) = dCost_dR_T2b.col(2);
  dCost_dR_T3.row(2) = dCost_dR_T3b.col(2);
  dCost_dR_T1 *= 2.0 / m;
  dCost_dR_T2 *= 2.0 / m;
  dCost_dR_T3 *= 2.0 / m;
  hessian(3, 0) = (dR_dPhi * dCost_dR_T1).trace();
  hessian(4, 0) = (dR_dTheta * dCost_dR_T1).trace();
  hessian(5, 0) = (dR_dPsi * dCost_dR_T1).trace();
  hessian(3, 1) = (dR_dPhi * dCost_dR_T2).trace();
  hessian(4, 1) = (dR_dTheta * dCost_dR_T2).trace();
  hessian(5, 1) = (dR_dPsi * dCost_dR_T2).trace();
  hessian(3, 2) = (dR_dPhi * dCost_dR_T3).trace();
  hessian(4, 2) = (dR_dTheta * dCost_dR_T3).trace();
  hessian(5, 2) = (dR_dPsi * dCost_dR_T3).trace();
  hessian.block<3, 3>(0, 3) = hessian.block<3, 3>(3, 0).transpose();
  // rotation-rotation hessian
  int lookup[3][3] = {{0, 1, 2}, {1, 3, 4}, {2, 4, 5}};
  for (int l = 0; l < 3; ++l) {
    for (int i = 0; i < 3; ++i) {
      double phi_tmp = 0.0, theta_tmp = 0.0, psi_tmp = 0.0;
      for (int j = 0; j < 3; ++j) {
        for (int k = 0; k < 3; ++k) {
          phi_tmp += hessian_rot_tmp(3 * j + i, lookup[l][k]) * dR_dPhi(j, k);
          theta_tmp += hessian_rot_tmp(3 * j + i, lookup[l][k]) * dR_dTheta(j, k);
          psi_tmp += hessian_rot_tmp(3 * j + i, lookup[l][k]) * dR_dPsi(j, k);
        }
      }
      hessian_rot_phi(i, l) = phi_tmp;
      hessian_rot_theta(i, l) = theta_tmp;
      hessian_rot_psi(i, l) = psi_tmp;
    }
  }
  hessian_rot_phi *= 2.0 / m;
  hessian_rot_theta *= 2.0 / m;
  hessian_rot_psi *= 2.0 / m;
  hessian(3, 3) = (dR_dPhi.transpose() * hessian_rot_phi).trace() +
                  (ddR_dPhi_dPhi * dCost_dR_T).trace();
  hessian(3, 4) = (dR_dPhi.transpose() * hessian_rot_theta).trace() +
                  (ddR_dPhi_dTheta * dCost_dR_T).trace();
  hessian(3, 5) = (dR_dPhi.transpose() * hessian_rot_psi).trace() +
                  (ddR_dPhi_dPsi * dCost_dR_T).trace();
  hessian(4, 4) = (dR_dTheta.transpose() * hessian_rot_theta).trace() +
                  (ddR_dTheta_dTheta * dCost_dR_T).trace();
  hessian(4, 5) = (dR_dTheta.transpose() * hessian_rot_psi).trace() +
                  (ddR_dTheta_dPsi * dCost_dR_T).trace();
  hessian(5, 5) = (dR_dPsi.transpose() * hessian_rot_psi).trace() +
                  (ddR_dPsi_dPsi * dCost_dR_T).trace();
  hessian(4, 3) = hessian(3, 4);
  hessian(5, 3) = hessian(3, 5);
  hessian(5, 4) = hessian(4, 5);
}

template <typename PointSource, typename PointTarget, typename Scalar>
inline BFGSSpace::Status
GeneralizedIterativeClosestPoint<PointSource, PointTarget, Scalar>::
    OptimizationFunctorWithIndices::checkGradient(const Vector6d& g)
{
  auto translation_epsilon = gicp_->translation_gradient_tolerance_;
  auto rotation_epsilon = gicp_->rotation_gradient_tolerance_;

  if ((translation_epsilon < 0.) || (rotation_epsilon < 0.))
    return BFGSSpace::NegativeGradientEpsilon;

  // express translation gradient as norm of translation parameters
  auto translation_grad = g.head<3>().norm();

  // express rotation gradient as a norm of rotation parameters
  auto rotation_grad = g.tail<3>().norm();

  if ((translation_grad < translation_epsilon) && (rotation_grad < rotation_epsilon))
    return BFGSSpace::Success;

  return BFGSSpace::Running;
}

template <typename PointSource, typename PointTarget, typename Scalar>
inline void
GeneralizedIterativeClosestPoint<PointSource, PointTarget, Scalar>::
    computeTransformation(PointCloudSource& output, const Matrix4& guess)
{
  pcl::IterativeClosestPoint<PointSource, PointTarget, Scalar>::initComputeReciprocal();
  // Difference between consecutive transforms
  double delta = 0;
  // Get the size of the source point cloud
  const std::size_t N = indices_->size();
  // Set the mahalanobis matrices to identity
  mahalanobis_.resize(N, Eigen::Matrix3d::Identity());
  // Compute target cloud covariance matrices
  if ((!target_covariances_) || (target_covariances_->empty())) {
    target_covariances_.reset(new MatricesVector);
    computeCovariances<PointTarget>(target_, tree_, *target_covariances_);
  }
  // Compute input cloud covariance matrices
  if ((!input_covariances_) || (input_covariances_->empty())) {
    input_covariances_.reset(new MatricesVector);
    computeCovariances<PointSource>(input_, tree_reciprocal_, *input_covariances_);
  }

  base_transformation_ = Matrix4::Identity();
  nr_iterations_ = 0;
  converged_ = false;
  double dist_threshold = corr_dist_threshold_ * corr_dist_threshold_;
  pcl::Indices nn_indices(1);
  std::vector<float> nn_dists(1);

  pcl::transformPointCloud(output, output, guess);

  while (!converged_) {
    std::size_t cnt = 0;
    pcl::Indices source_indices(indices_->size());
    pcl::Indices target_indices(indices_->size());

    // guess corresponds to base_t and transformation_ to t
    Eigen::Matrix4d transform_R = Eigen::Matrix4d::Zero();
    for (std::size_t i = 0; i < 4; i++)
      for (std::size_t j = 0; j < 4; j++)
        for (std::size_t k = 0; k < 4; k++)
          transform_R(i, j) += static_cast<double>(transformation_(i, k)) *
                               static_cast<double>(guess(k, j));

    Eigen::Matrix3d R = transform_R.topLeftCorner<3, 3>();

    for (std::size_t i = 0; i < N; i++) {
      PointSource query = output[i];
      query.getVector4fMap() =
          transformation_.template cast<float>() * query.getVector4fMap();

      if (!searchForNeighbors(query, nn_indices, nn_dists)) {
        PCL_ERROR("[pcl::%s::computeTransformation] Unable to find a nearest neighbor "
                  "in the target dataset for point %d in the source!\n",
                  getClassName().c_str(),
                  (*indices_)[i]);
        return;
      }

      // Check if the distance to the nearest neighbor is smaller than the user imposed
      // threshold
      if (nn_dists[0] < dist_threshold) {
        Eigen::Matrix3d& C1 = (*input_covariances_)[i];
        Eigen::Matrix3d& C2 = (*target_covariances_)[nn_indices[0]];
        Eigen::Matrix3d& M = mahalanobis_[i];
        // M = R*C1
        M = R * C1;
        // temp = M*R' + C2 = R*C1*R' + C2
        Eigen::Matrix3d temp = M * R.transpose();
        temp += C2;
        // M = temp^-1
        M = temp.inverse();
        source_indices[cnt] = static_cast<int>(i);
        target_indices[cnt] = nn_indices[0];
        cnt++;
      }
    }
    // Resize to the actual number of valid correspondences
    source_indices.resize(cnt);
    target_indices.resize(cnt);
    /* optimize transformation using the current assignment and Mahalanobis metrics*/
    previous_transformation_ = transformation_;
    // optimization right here
    try {
      rigid_transformation_estimation_(
          output, source_indices, *target_, target_indices, transformation_);
      /* compute the delta from this iteration */
      delta = 0.;
      for (int k = 0; k < 4; k++) {
        for (int l = 0; l < 4; l++) {
          double ratio = 1;
          if (k < 3 && l < 3) // rotation part of the transform
            ratio = 1. / rotation_epsilon_;
          else
            ratio = 1. / transformation_epsilon_;
          double c_delta =
              ratio * std::abs(previous_transformation_(k, l) - transformation_(k, l));
          if (c_delta > delta)
            delta = c_delta;
        }
      }
    } catch (PCLException& e) {
      PCL_DEBUG("[pcl::%s::computeTransformation] Optimization issue %s\n",
                getClassName().c_str(),
                e.what());
      break;
    }
    nr_iterations_++;

    if (update_visualizer_ != nullptr) {
      PointCloudSourcePtr input_transformed(new PointCloudSource);
      pcl::transformPointCloud(output, *input_transformed, transformation_);
      update_visualizer_(*input_transformed, source_indices, *target_, target_indices);
    }

    // Check for convergence
    if (nr_iterations_ >= max_iterations_ || delta < 1) {
      converged_ = true;
      PCL_DEBUG("[pcl::%s::computeTransformation] Convergence reached. Number of "
                "iterations: %d out of %d. Transformation difference: %f\n",
                getClassName().c_str(),
                nr_iterations_,
                max_iterations_,
                (transformation_ - previous_transformation_).array().abs().sum());
      previous_transformation_ = transformation_;
    }
    else
      PCL_DEBUG("[pcl::%s::computeTransformation] Convergence failed\n",
                getClassName().c_str());
  }
  final_transformation_ = previous_transformation_ * guess;

  PCL_DEBUG("Transformation "
            "is:\n\t%5f\t%5f\t%5f\t%5f\n\t%5f\t%5f\t%5f\t%5f\n\t%5f\t%5f\t%5f\t%5f\n\t%"
            "5f\t%5f\t%5f\t%5f\n",
            final_transformation_(0, 0),
            final_transformation_(0, 1),
            final_transformation_(0, 2),
            final_transformation_(0, 3),
            final_transformation_(1, 0),
            final_transformation_(1, 1),
            final_transformation_(1, 2),
            final_transformation_(1, 3),
            final_transformation_(2, 0),
            final_transformation_(2, 1),
            final_transformation_(2, 2),
            final_transformation_(2, 3),
            final_transformation_(3, 0),
            final_transformation_(3, 1),
            final_transformation_(3, 2),
            final_transformation_(3, 3));

  // Transform the point cloud
  pcl::transformPointCloud(*input_, output, final_transformation_);
}

template <typename PointSource, typename PointTarget, typename Scalar>
void
GeneralizedIterativeClosestPoint<PointSource, PointTarget, Scalar>::applyState(
    Matrix4& t, const Vector6d& x) const
{
  // Z Y X euler angles convention
  Matrix3 R = (AngleAxis(static_cast<Scalar>(x[5]), Vector3::UnitZ()) *
               AngleAxis(static_cast<Scalar>(x[4]), Vector3::UnitY()) *
               AngleAxis(static_cast<Scalar>(x[3]), Vector3::UnitX()))
                  .toRotationMatrix();
  Matrix4 T = Matrix4::Identity();
  T.template block<3, 3>(0, 0) = R;
  T.template block<3, 1>(0, 3) = Vector3(
      static_cast<Scalar>(x[0]), static_cast<Scalar>(x[1]), static_cast<Scalar>(x[2]));
  t = T * t;
}

} // namespace pcl

#endif // PCL_REGISTRATION_IMPL_GICP_HPP_
