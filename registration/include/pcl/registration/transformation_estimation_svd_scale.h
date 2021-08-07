/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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

#pragma once

#include <pcl/registration/transformation_estimation_svd.h>

namespace pcl {
namespace registration {
/** @b TransformationEstimationSVD implements SVD-based estimation of
 * the transformation aligning the given correspondences.
 * Optionally the scale is estimated. Note that the similarity transform might not be
 * optimal for the underlying Frobenius Norm.
 *
 * \note The class is templated on the source and target point types as well as on the
 * output scalar of the transformation matrix (i.e., float or double). Default: float.
 * \author Suat Gedikli
 * \ingroup registration
 */
template <typename PointSource, typename PointTarget, typename Scalar = float>
class TransformationEstimationSVDScale
: public TransformationEstimationSVD<PointSource, PointTarget, Scalar> {
public:
  using Ptr =
      shared_ptr<TransformationEstimationSVDScale<PointSource, PointTarget, Scalar>>;
  using ConstPtr = shared_ptr<
      const TransformationEstimationSVDScale<PointSource, PointTarget, Scalar>>;

  using Matrix4 =
      typename TransformationEstimationSVD<PointSource, PointTarget, Scalar>::Matrix4;

  /** \brief Inherits from TransformationEstimationSVD, but forces it to not use the
   * Umeyama method */
  TransformationEstimationSVDScale()
  : TransformationEstimationSVD<PointSource, PointTarget, Scalar>(false)
  {}

protected:
  /** \brief Obtain a 4x4 rigid transformation matrix from a correlation matrix H = src
   * * tgt' \param[in] cloud_src_demean the input source cloud, demeaned, in Eigen
   * format \param[in] centroid_src the input source centroid, in Eigen format
   * \param[in] cloud_tgt_demean the input target cloud, demeaned, in Eigen format
   * \param[in] centroid_tgt the input target cloud, in Eigen format
   * \param[out] transformation_matrix the resultant 4x4 rigid transformation matrix
   */
  void
  getTransformationFromCorrelation(
      const Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>& cloud_src_demean,
      const Eigen::Matrix<Scalar, 4, 1>& centroid_src,
      const Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>& cloud_tgt_demean,
      const Eigen::Matrix<Scalar, 4, 1>& centroid_tgt,
      Matrix4& transformation_matrix) const;
};

} // namespace registration
} // namespace pcl

#include <pcl/registration/impl/transformation_estimation_svd_scale.hpp>
