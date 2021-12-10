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

#include <pcl/common/transforms.h>
#include <pcl/features/feature.h>
#include <pcl/registration/correspondence_types.h>
#include <pcl/correspondence.h>

namespace pcl {
namespace registration {
/** \brief TransformationValidation represents the base class for methods
 * that validate the correctness of a transformation found through \ref
 * TransformationEstimation.
 *
 * The inputs for a validation estimation can take any or all of the following:
 *
 *   - source point cloud
 *   - target point cloud
 *   - estimated transformation between source and target
 *
 * The output is in the form of a score or a confidence measure.
 *
 * \note The class is templated on the source and target point types as well as on the
 * output scalar of the transformation matrix (i.e., float or double). Default: float.
 * \author Radu B. Rusu
 * \ingroup registration
 */
template <typename PointSource, typename PointTarget, typename Scalar = float>
class TransformationValidation {
public:
  using Matrix4 = Eigen::Matrix<Scalar, 4, 4>;
  using Ptr = shared_ptr<TransformationValidation<PointSource, PointTarget, Scalar>>;
  using ConstPtr =
      shared_ptr<const TransformationValidation<PointSource, PointTarget, Scalar>>;

  using PointCloudSource = pcl::PointCloud<PointSource>;
  using PointCloudSourcePtr = typename PointCloudSource::Ptr;
  using PointCloudSourceConstPtr = typename PointCloudSource::ConstPtr;

  using PointCloudTarget = pcl::PointCloud<PointTarget>;
  using PointCloudTargetPtr = typename PointCloudTarget::Ptr;
  using PointCloudTargetConstPtr = typename PointCloudTarget::ConstPtr;

  TransformationValidation(){};
  virtual ~TransformationValidation(){};

  /** \brief Validate the given transformation with respect to the input cloud data, and
   * return a score. Pure virtual.
   *
   * \param[in] cloud_src the source point cloud dataset
   * \param[in] cloud_tgt the target point cloud dataset
   * \param[out] transformation_matrix the transformation matrix
   *
   * \return the score or confidence measure for the given
   * transformation_matrix with respect to the input data
   */
  virtual double
  validateTransformation(const PointCloudSourceConstPtr& cloud_src,
                         const PointCloudTargetConstPtr& cloud_tgt,
                         const Matrix4& transformation_matrix) const = 0;

  /** \brief Comparator function for deciding which score is better after running the
   * validation on multiple transforms. Pure virtual.
   *
   * \note For example, for Euclidean distances smaller is better, for inliers the
   * opposite.
   *
   * \param[in] score1 the first value
   * \param[in] score2 the second value
   *
   * \return true if score1 is better than score2
   */
  virtual bool
  operator()(const double& score1, const double& score2) const = 0;

  /** \brief Check if the score is valid for a specific transformation. Pure virtual.
   *
   * \param[in] cloud_src the source point cloud dataset
   * \param[in] cloud_tgt the target point cloud dataset
   * \param[out] transformation_matrix the transformation matrix
   *
   * \return true if the transformation is valid, false otherwise.
   */
  virtual bool
  isValid(const PointCloudSourceConstPtr& cloud_src,
          const PointCloudTargetConstPtr& cloud_tgt,
          const Matrix4& transformation_matrix) const = 0;
};
} // namespace registration
} // namespace pcl
