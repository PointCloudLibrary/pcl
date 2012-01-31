/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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
 * $Id$
 *
 */

#ifndef PCL_SEARCH_IMPL_ORGANIZED_NEIGHBOR_SEARCH_H_
#define PCL_SEARCH_IMPL_ORGANIZED_NEIGHBOR_SEARCH_H_

#include "pcl/search/organized.h"
#include <pcl/common/eigen.h>
#include "pcl/common/time.h"
#include <eigen3/Eigen/Eigenvalues>
//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT> int
pcl::search::OrganizedNeighbor<PointT>::radiusSearch (int                 index,
                                                       const double        radius,
                                                       std::vector<int>    &k_indices,
                                                       std::vector<float>  &k_sqr_distances,
                                                       unsigned int        max_nn) const
{
  const PointT& searchPoint = input_->points [index];
  return (radiusSearch (searchPoint, radius, k_indices, k_sqr_distances, max_nn));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT> int
pcl::search::OrganizedNeighbor<PointT>::radiusSearch (const               PointT &p_q,
                                                       const double        radius,
                                                       std::vector<int>    &k_indices,
                                                       std::vector<float>  &k_sqr_distances,
                                                       unsigned int        max_nn) const
{
  if (projection_matrix_.coeffRef (0) == 0)
  {
    PCL_ERROR ("[pcl::%s::radiusSearch] Invalid projection matrix. Probably input dataset was not organized!\n", getName ().c_str ());
    return (0);
  }

  // NAN test
  if (!pcl_isfinite (p_q.x) || !pcl_isfinite (p_q.y) || !pcl_isfinite (p_q.z))
    return (0);

  // search window
  unsigned left, right, top, bottom;
  //unsigned x, y, idx;
  float squared_distance, squared_radius;

  k_indices.clear ();
  k_sqr_distances.clear ();

  squared_radius = radius * radius;

  this->getProjectedRadiusSearchBox (p_q, squared_radius, left, right, top, bottom);

  // iterate over search box
  if (max_nn == 0 || max_nn >= (unsigned int)input_->points.size ())
    max_nn = input_->points.size ();

  k_indices.reserve (max_nn);
  k_sqr_distances.reserve (max_nn);

  unsigned yEnd  = (bottom + 1) * input_->width + right + 1;
  register unsigned idx  = top * input_->width + left;
  unsigned skip = input_->width - right + left - 1;
  unsigned xEnd = idx - left + right + 1;

  for (; xEnd != yEnd; idx += skip, xEnd += input_->width)
  {
    for (; idx < xEnd; ++idx)
    {
      squared_distance = (input_->points[idx].getVector3fMap () - p_q.getVector3fMap ()).squaredNorm ();
      if (squared_distance <= squared_radius)
      {
        k_indices.push_back (idx);
        k_sqr_distances.push_back (squared_distance);
        // already done ?
        if (k_indices.size () == max_nn)
          return max_nn;
      }
    }
  }
  return (k_indices.size ());
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT> int
pcl::search::OrganizedNeighbor<PointT>::nearestKSearch (const pcl::PointCloud<PointT> &cloud,
                                                        int                           index,
                                                        int                           k,
                                                        std::vector<int>              &k_indices,
                                                        std::vector<float>            &k_sqr_distances) const
{
  return (nearestKSearch (index, k, k_indices, k_sqr_distances));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT> int
pcl::search::OrganizedNeighbor<PointT>::nearestKSearch (int index,
                                                        int k,
                                                        std::vector<int> &k_indices,
                                                        std::vector<float> &k_distances) const
{
  PCL_ERROR ("[pcl::search::OrganizedNeighbor::nearestKSearch] Method not implemented!\n");
  return (0);
}


////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT> void
pcl::search::OrganizedNeighbor<PointT>::getProjectedRadiusSearchBox (const PointT& point,
                                                                      float squared_radius,
                                                                      unsigned &minX,
                                                                      unsigned &maxX,
                                                                      unsigned &minY,
                                                                      unsigned &maxY) const
{
  Eigen::Vector3f q = KR_ * point.getVector3fMap () + projection_matrix_.block <3, 1> (0, 3);

  float a = squared_radius * KR_KRT_.coeff (8) - q [2] * q [2];
  float b = squared_radius * KR_KRT_.coeff (7) - q [1] * q [2];
  float c = squared_radius * KR_KRT_.coeff (4) - q [1] * q [1];

  // a and c are multiplied by two already => - 4ac -> - ac
  float det = b * b - a * c;
  if (det < 0)
  {
    minY = 0;
    maxY = input_->height - 1;
  }
  else
  {
    minY = (unsigned) std::max (0, (int) floor ((b + sqrt (det)) / a));
    maxY = (unsigned) std::min ((int) input_->height - 1, (int) ceil ((b - sqrt (det)) / a));
  }

  b = squared_radius * KR_KRT_.coeff (6) - q [0] * q [2];
  c = squared_radius * KR_KRT_.coeff (0) - q [0] * q [0];

  det = b * b - a * c;
  if (det < 0)
  {
    minX = 0;
    maxX = input_->width - 1;
  }
  else
  {
    minX = (unsigned) std::max (0, (int) floor ((b + sqrt (det)) / a));
    maxX = (unsigned) std::min ((int)input_->width - 1, (int) ceil ((b - sqrt (det)) / a));
  }
  std::cout << "window2: " << minX << " : " << minY << " - " << maxX << " : " << maxY << std::endl;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT> template <typename MatrixType> void
pcl::search::OrganizedNeighbor<PointT>::makeSymmetric (MatrixType& matrix, bool use_upper_triangular) const
{
  if (use_upper_triangular)
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

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT> void
pcl::search::OrganizedNeighbor<PointT>::estimateProjectionMatrix ()
{
  // internally we calculate with double but store the result into float matrices.
  typedef double Scalar;
  projection_matrix_.setZero ();
  if (input_->height == 1 || input_->width == 1)
  {
    PCL_ERROR ("[pcl::%s::estimateProjectionMatrix] Input dataset is not organized!\n", getName ().c_str ());
    return;
  }

  // we just want to use every 16th column and row -> skip = 2^4
  const unsigned int skip = input_->width >> 4;
  Eigen::Matrix<Scalar, 4, 4> A = Eigen::Matrix<Scalar, 4, 4>::Zero ();
  Eigen::Matrix<Scalar, 4, 4> B = Eigen::Matrix<Scalar, 4, 4>::Zero ();
  Eigen::Matrix<Scalar, 4, 4> C = Eigen::Matrix<Scalar, 4, 4>::Zero ();
  Eigen::Matrix<Scalar, 4, 4> D = Eigen::Matrix<Scalar, 4, 4>::Zero ();

  for (unsigned yIdx = 0, idx = 0; yIdx < input_->height; yIdx += skip, idx += input_->width * (skip-1))
  {
    for (unsigned xIdx = 0; xIdx < input_->width; xIdx += skip, idx += skip)
    {
      const PointT& point = input_->points[idx];
      if (isFinite (point))
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
        B.coeffRef (3) -= point.x * xIdx;

        B.coeffRef (5) -= yy * xIdx;
        B.coeffRef (6) -= yz * xIdx;
        B.coeffRef (7) -= point.y * xIdx;

        B.coeffRef (10) -= zz * xIdx;
        B.coeffRef (11) -= point.z * xIdx;

        B.coeffRef (15) -= xIdx;

        C.coeffRef (0) -= xx * yIdx;
        C.coeffRef (1) -= xy * yIdx;
        C.coeffRef (2) -= xz * yIdx;
        C.coeffRef (3) -= point.x * yIdx;

        C.coeffRef (5) -= yy * yIdx;
        C.coeffRef (6) -= yz * yIdx;
        C.coeffRef (7) -= point.y * yIdx;

        C.coeffRef (10) -= zz * yIdx;
        C.coeffRef (11) -= point.z * yIdx;

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
    }
  }

  makeSymmetric(A);
  makeSymmetric(B);
  makeSymmetric(C);
  makeSymmetric(D);

  Eigen::Matrix<Scalar, 12, 12> X = Eigen::Matrix<Scalar, 12, 12>::Zero ();
  X.topLeftCorner<4,4> () = A;
  X.block<4,4> (0, 8) = B;
  X.block<4,4> (8, 0) = B;
  X.block<4,4> (4, 4) = A;
  X.block<4,4> (4, 8) = C;
  X.block<4,4> (8, 4) = C;
  X.block<4,4> (8, 8) = D;

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<Scalar, 12, 12> > ei_symm(X);
  Eigen::Matrix<Scalar, 12, 12> eigen_vectors = ei_symm.eigenvectors();

  // check whether the residual MSE is low. If its high, the cloud was not captured from a projective device.
  Eigen::Matrix<Scalar, 1, 1> residual_sqr = eigen_vectors.col (0).transpose () * X *  eigen_vectors.col (0);
  if ( residual_sqr.coeff (0) > eps_ * A.coeff (15))
  {
    PCL_ERROR ("[pcl::%s::radiusSearch] Input dataset is not from a projective device!\n", getName ().c_str ());
    return;
  }

  projection_matrix_.coeffRef (0) = eigen_vectors.coeff (0);
  projection_matrix_.coeffRef (1) = eigen_vectors.coeff (12);
  projection_matrix_.coeffRef (2) = eigen_vectors.coeff (24);
  projection_matrix_.coeffRef (3) = eigen_vectors.coeff (36);
  projection_matrix_.coeffRef (4) = eigen_vectors.coeff (48);
  projection_matrix_.coeffRef (5) = eigen_vectors.coeff (60);
  projection_matrix_.coeffRef (6) = eigen_vectors.coeff (72);
  projection_matrix_.coeffRef (7) = eigen_vectors.coeff (84);
  projection_matrix_.coeffRef (8) = eigen_vectors.coeff (96);
  projection_matrix_.coeffRef (9) = eigen_vectors.coeff (108);
  projection_matrix_.coeffRef (10) = eigen_vectors.coeff (120);
  projection_matrix_.coeffRef (11) = eigen_vectors.coeff (132);

  if (projection_matrix_.coeff (0) < 0)
    projection_matrix_ *= -1.0;

  // get left 3x3 sub matrix, which contains K * R, with K = camera matrix = [[fx s cx] [0 fy cy] [0 0 1]]
  // and R being the rotation matrix
  KR_ = projection_matrix_.topLeftCorner <3, 3> ();

  // precalculate KR * KR^T needed by calculations during nn-search
  KR_KRT_ = KR_ * KR_.transpose ();
}

#define PCL_INSTANTIATE_OrganizedNeighbor(T) template class PCL_EXPORTS pcl::search::OrganizedNeighbor<T>;

#endif
