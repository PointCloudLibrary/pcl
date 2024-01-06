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
 *
 */

#pragma once

#include <pcl/registration/transformation_estimation.h>
#include <pcl/cloud_iterator.h>

namespace pcl {
namespace registration {
/** @b TransformationEstimationDualQuaternion implements dual quaternion based
 * estimation of the transformation aligning the given correspondences.
 *
 * \note The class is templated on the source and target point types as well as on the
 * output scalar of the transformation matrix (i.e., float or double). Default: float.
 * \author Sergey Zagoruyko
 * \ingroup registration
 */
template <typename PointSource, typename PointTarget, typename Scalar = float>
class TransformationEstimationDualQuaternion
: public TransformationEstimation<PointSource, PointTarget, Scalar> {
public:
  using Ptr = shared_ptr<
      TransformationEstimationDualQuaternion<PointSource, PointTarget, Scalar>>;
  using ConstPtr = shared_ptr<
      const TransformationEstimationDualQuaternion<PointSource, PointTarget, Scalar>>;

  using Matrix4 =
      typename TransformationEstimation<PointSource, PointTarget, Scalar>::Matrix4;

  TransformationEstimationDualQuaternion() = default;
  ~TransformationEstimationDualQuaternion() override = default;

  /** \brief Estimate a rigid rotation transformation between a source and a target
   * point cloud using dual quaternion optimization \param[in] cloud_src the source
   * point cloud dataset \param[in] cloud_tgt the target point cloud dataset \param[out]
   * transformation_matrix the resultant transformation matrix
   */
  inline void
  estimateRigidTransformation(const pcl::PointCloud<PointSource>& cloud_src,
                              const pcl::PointCloud<PointTarget>& cloud_tgt,
                              Matrix4& transformation_matrix) const override;

  /** \brief Estimate a rigid rotation transformation between a source and a target
   * point cloud using dual quaternion optimization \param[in] cloud_src the source
   * point cloud dataset \param[in] indices_src the vector of indices describing the
   * points of interest in \a cloud_src
   * \param[in] cloud_tgt the target point cloud dataset
   * \param[out] transformation_matrix the resultant transformation matrix
   */
  inline void
  estimateRigidTransformation(const pcl::PointCloud<PointSource>& cloud_src,
                              const pcl::Indices& indices_src,
                              const pcl::PointCloud<PointTarget>& cloud_tgt,
                              Matrix4& transformation_matrix) const override;

  /** \brief Estimate a rigid rotation transformation between a source and a target
   * point cloud using dual quaternion optimization \param[in] cloud_src the source
   * point cloud dataset \param[in] indices_src the vector of indices describing the
   * points of interest in \a cloud_src
   * \param[in] cloud_tgt the target point cloud dataset
   * \param[in] indices_tgt the vector of indices describing the correspondences of the
   * interest points from \a indices_src
   * \param[out] transformation_matrix the resultant transformation matrix
   */
  inline void
  estimateRigidTransformation(const pcl::PointCloud<PointSource>& cloud_src,
                              const pcl::Indices& indices_src,
                              const pcl::PointCloud<PointTarget>& cloud_tgt,
                              const pcl::Indices& indices_tgt,
                              Matrix4& transformation_matrix) const override;

  /** \brief Estimate a rigid rotation transformation between a source and a target
   * point cloud using dual quaternion optimization \param[in] cloud_src the source
   * point cloud dataset \param[in] cloud_tgt the target point cloud dataset \param[in]
   * correspondences the vector of correspondences between source and target point cloud
   * \param[out] transformation_matrix the resultant transformation matrix
   */
  void
  estimateRigidTransformation(const pcl::PointCloud<PointSource>& cloud_src,
                              const pcl::PointCloud<PointTarget>& cloud_tgt,
                              const pcl::Correspondences& correspondences,
                              Matrix4& transformation_matrix) const override;

protected:
  /** \brief Estimate a rigid rotation transformation between a source and a target
   * \param[in] source_it an iterator over the source point cloud dataset
   * \param[in] target_it an iterator over the target point cloud dataset
   * \param[out] transformation_matrix the resultant transformation matrix
   */
  void
  estimateRigidTransformation(ConstCloudIterator<PointSource>& source_it,
                              ConstCloudIterator<PointTarget>& target_it,
                              Matrix4& transformation_matrix) const;
};

} // namespace registration
} // namespace pcl

#include <pcl/registration/impl/transformation_estimation_dual_quaternion.hpp>
