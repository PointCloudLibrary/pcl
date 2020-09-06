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

#include <pcl/common/projection_matrix.h>
#include <pcl/console/print.h> // for PCL_ERROR
#include <pcl/cloud_iterator.h>

#include <Eigen/Eigenvalues> // for SelfAdjointEigenSolver

namespace pcl
{

namespace common
{

namespace internal
{

template <typename MatrixT> void
makeSymmetric (MatrixT& matrix, bool use_upper_triangular = true)
{
  if (use_upper_triangular && (MatrixT::Flags & Eigen::RowMajorBit))
  {
    matrix.coeffRef (4) = matrix.coeff (1);
    matrix.coeffRef (8) = matrix.coeff (2);
    matrix.coeffRef (9) = matrix.coeff (6);
    matrix.coeffRef (12) = matrix.coeff (3);
    matrix.coeffRef (13) = matrix.coeff (7);
    matrix.coeffRef (14) = matrix.coeff (11);
  }
  else
  {
    matrix.coeffRef (1) = matrix.coeff (4);
    matrix.coeffRef (2) = matrix.coeff (8);
    matrix.coeffRef (6) = matrix.coeff (9);
    matrix.coeffRef (3) = matrix.coeff (12);
    matrix.coeffRef (7) = matrix.coeff (13);
    matrix.coeffRef (11) = matrix.coeff (14);
  }
}

} // namespace internal
} // namespace common


template <typename PointT> double
estimateProjectionMatrix (
    typename pcl::PointCloud<PointT>::ConstPtr cloud,
    Eigen::Matrix<float, 3, 4, Eigen::RowMajor>& projection_matrix,
    const Indices& indices)
{
  // internally we calculate with double but store the result into float matrices.
  using Scalar = double;
  projection_matrix.setZero ();
  if (cloud->height == 1 || cloud->width == 1)
  {
    PCL_ERROR ("[pcl::estimateProjectionMatrix] Input dataset is not organized!\n");
    return (-1.0);
  }

  Eigen::Matrix<Scalar, 4, 4, Eigen::RowMajor> A = Eigen::Matrix<Scalar, 4, 4, Eigen::RowMajor>::Zero ();
  Eigen::Matrix<Scalar, 4, 4, Eigen::RowMajor> B = Eigen::Matrix<Scalar, 4, 4, Eigen::RowMajor>::Zero ();
  Eigen::Matrix<Scalar, 4, 4, Eigen::RowMajor> C = Eigen::Matrix<Scalar, 4, 4, Eigen::RowMajor>::Zero ();
  Eigen::Matrix<Scalar, 4, 4, Eigen::RowMajor> D = Eigen::Matrix<Scalar, 4, 4, Eigen::RowMajor>::Zero ();

  pcl::ConstCloudIterator <PointT> pointIt (*cloud, indices);

  while (pointIt)
  {
    unsigned yIdx = pointIt.getCurrentPointIndex () / cloud->width;
    unsigned xIdx = pointIt.getCurrentPointIndex () % cloud->width;

    const PointT& point = *pointIt;
    if (std::isfinite (point.x))
    {
      Scalar xx = point.x * point.x;
      Scalar xy = point.x * point.y;
      Scalar xz = point.x * point.z;
      Scalar yy = point.y * point.y;
      Scalar yz = point.y * point.z;
      Scalar zz = point.z * point.z;
      Scalar xx_yy = xIdx * xIdx + yIdx * yIdx;

      A.coeffRef (0) += xx;
      A.coeffRef (1) += xy;
      A.coeffRef (2) += xz;
      A.coeffRef (3) += point.x;

      A.coeffRef (5) += yy;
      A.coeffRef (6) += yz;
      A.coeffRef (7) += point.y;

      A.coeffRef (10) += zz;
      A.coeffRef (11) += point.z;
      A.coeffRef (15) += 1.0;

      B.coeffRef (0) -= xx * xIdx;
      B.coeffRef (1) -= xy * xIdx;
      B.coeffRef (2) -= xz * xIdx;
      B.coeffRef (3) -= point.x * static_cast<double>(xIdx);

      B.coeffRef (5) -= yy * xIdx;
      B.coeffRef (6) -= yz * xIdx;
      B.coeffRef (7) -= point.y * static_cast<double>(xIdx);

      B.coeffRef (10) -= zz * xIdx;
      B.coeffRef (11) -= point.z * static_cast<double>(xIdx);

      B.coeffRef (15) -= xIdx;

      C.coeffRef (0) -= xx * yIdx;
      C.coeffRef (1) -= xy * yIdx;
      C.coeffRef (2) -= xz * yIdx;
      C.coeffRef (3) -= point.x * static_cast<double>(yIdx);

      C.coeffRef (5) -= yy * yIdx;
      C.coeffRef (6) -= yz * yIdx;
      C.coeffRef (7) -= point.y * static_cast<double>(yIdx);

      C.coeffRef (10) -= zz * yIdx;
      C.coeffRef (11) -= point.z * static_cast<double>(yIdx);

      C.coeffRef (15) -= yIdx;

      D.coeffRef (0) += xx * xx_yy;
      D.coeffRef (1) += xy * xx_yy;
      D.coeffRef (2) += xz * xx_yy;
      D.coeffRef (3) += point.x * xx_yy;

      D.coeffRef (5) += yy * xx_yy;
      D.coeffRef (6) += yz * xx_yy;
      D.coeffRef (7) += point.y * xx_yy;

      D.coeffRef (10) += zz * xx_yy;
      D.coeffRef (11) += point.z * xx_yy;

      D.coeffRef (15) += xx_yy;
    }

    ++pointIt;
  } // while

  common::internal::makeSymmetric (A);
  common::internal::makeSymmetric (B);
  common::internal::makeSymmetric (C);
  common::internal::makeSymmetric (D);

  Eigen::Matrix<Scalar, 12, 12, Eigen::RowMajor> X = Eigen::Matrix<Scalar, 12, 12, Eigen::RowMajor>::Zero ();
  X.topLeftCorner<4,4> ().matrix () = A;
  X.block<4,4> (0, 8).matrix () = B;
  X.block<4,4> (8, 0).matrix () = B;
  X.block<4,4> (4, 4).matrix () = A;
  X.block<4,4> (4, 8).matrix () = C;
  X.block<4,4> (8, 4).matrix () = C;
  X.block<4,4> (8, 8).matrix () = D;

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<Scalar, 12, 12, Eigen::RowMajor> > ei_symm (X);
  Eigen::Matrix<Scalar, 12, 12, Eigen::RowMajor> eigen_vectors = ei_symm.eigenvectors ();

  // check whether the residual MSE is low. If its high, the cloud was not captured from a projective device.
  Eigen::Matrix<Scalar, 1, 1> residual_sqr = eigen_vectors.col (0).transpose () * X *  eigen_vectors.col (0);

  double residual = residual_sqr.coeff (0);

  projection_matrix.coeffRef (0) = static_cast <float> (eigen_vectors.coeff (0));
  projection_matrix.coeffRef (1) = static_cast <float> (eigen_vectors.coeff (12));
  projection_matrix.coeffRef (2) = static_cast <float> (eigen_vectors.coeff (24));
  projection_matrix.coeffRef (3) = static_cast <float> (eigen_vectors.coeff (36));
  projection_matrix.coeffRef (4) = static_cast <float> (eigen_vectors.coeff (48));
  projection_matrix.coeffRef (5) = static_cast <float> (eigen_vectors.coeff (60));
  projection_matrix.coeffRef (6) = static_cast <float> (eigen_vectors.coeff (72));
  projection_matrix.coeffRef (7) = static_cast <float> (eigen_vectors.coeff (84));
  projection_matrix.coeffRef (8) = static_cast <float> (eigen_vectors.coeff (96));
  projection_matrix.coeffRef (9) = static_cast <float> (eigen_vectors.coeff (108));
  projection_matrix.coeffRef (10) = static_cast <float> (eigen_vectors.coeff (120));
  projection_matrix.coeffRef (11) = static_cast <float> (eigen_vectors.coeff (132));

  if (projection_matrix.coeff (0) < 0)
    projection_matrix *= -1.0;

  return (residual);
}

} // namespace pcl

