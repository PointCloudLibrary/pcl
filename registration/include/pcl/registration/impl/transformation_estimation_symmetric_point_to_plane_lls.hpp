/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2019-, Open Perception, Inc.
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

#include <pcl/cloud_iterator.h>

namespace pcl {

namespace registration {

template <typename PointSource, typename PointTarget, typename Scalar>
inline void
TransformationEstimationSymmetricPointToPlaneLLS<PointSource, PointTarget, Scalar>::
    estimateRigidTransformation(const pcl::PointCloud<PointSource>& cloud_src,
                                const pcl::PointCloud<PointTarget>& cloud_tgt,
                                Matrix4& transformation_matrix) const
{
  const auto nr_points = cloud_src.size();
  if (cloud_tgt.size() != nr_points) {
    PCL_ERROR("[pcl::TransformationEstimationSymmetricPointToPlaneLLS::"
              "estimateRigidTransformation] Number or points in source (%zu) differs "
              "from target (%zu)!\n",
              static_cast<std::size_t>(nr_points),
              static_cast<std::size_t>(cloud_tgt.size()));
    return;
  }

  ConstCloudIterator<PointSource> source_it(cloud_src);
  ConstCloudIterator<PointTarget> target_it(cloud_tgt);
  estimateRigidTransformation(source_it, target_it, transformation_matrix);
}

template <typename PointSource, typename PointTarget, typename Scalar>
void
TransformationEstimationSymmetricPointToPlaneLLS<PointSource, PointTarget, Scalar>::
    estimateRigidTransformation(const pcl::PointCloud<PointSource>& cloud_src,
                                const pcl::Indices& indices_src,
                                const pcl::PointCloud<PointTarget>& cloud_tgt,
                                Matrix4& transformation_matrix) const
{
  const auto nr_points = indices_src.size();
  if (cloud_tgt.size() != nr_points) {
    PCL_ERROR("[pcl::TransformationEstimationSymmetricPointToPlaneLLS::"
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
TransformationEstimationSymmetricPointToPlaneLLS<PointSource, PointTarget, Scalar>::
    estimateRigidTransformation(const pcl::PointCloud<PointSource>& cloud_src,
                                const pcl::Indices& indices_src,
                                const pcl::PointCloud<PointTarget>& cloud_tgt,
                                const pcl::Indices& indices_tgt,
                                Matrix4& transformation_matrix) const
{
  const auto nr_points = indices_src.size();
  if (indices_tgt.size() != nr_points) {
    PCL_ERROR("[pcl::TransformationEstimationSymmetricPointToPlaneLLS::"
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
TransformationEstimationSymmetricPointToPlaneLLS<PointSource, PointTarget, Scalar>::
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
TransformationEstimationSymmetricPointToPlaneLLS<PointSource, PointTarget, Scalar>::
    constructTransformationMatrix(const Vector6& parameters,
                                  Matrix4& transformation_matrix) const
{
  // Construct the transformation matrix from rotation and translation
  const Eigen::AngleAxis<Scalar> rotation_z(parameters(2),
                                            Eigen::Matrix<Scalar, 3, 1>::UnitZ());
  const Eigen::AngleAxis<Scalar> rotation_y(parameters(1),
                                            Eigen::Matrix<Scalar, 3, 1>::UnitY());
  const Eigen::AngleAxis<Scalar> rotation_x(parameters(0),
                                            Eigen::Matrix<Scalar, 3, 1>::UnitX());
  const Eigen::Translation<Scalar, 3> translation(
      parameters(3), parameters(4), parameters(5));
  const Eigen::Transform<Scalar, 3, Eigen::Affine> transform =
      rotation_z * rotation_y * rotation_x * translation * rotation_z * rotation_y *
      rotation_x;
  transformation_matrix = transform.matrix();
}

template <typename PointSource, typename PointTarget, typename Scalar>
inline void
TransformationEstimationSymmetricPointToPlaneLLS<PointSource, PointTarget, Scalar>::
    estimateRigidTransformation(ConstCloudIterator<PointSource>& source_it,
                                ConstCloudIterator<PointTarget>& target_it,
                                Matrix4& transformation_matrix) const
{
  using Matrix6 = Eigen::Matrix<Scalar, 6, 6>;
  using Vector3 = Eigen::Matrix<Scalar, 3, 1>;

  Matrix6 ATA;
  Vector6 ATb;
  ATA.setZero();
  ATb.setZero();
  auto M = ATA.template selfadjointView<Eigen::Upper>();

  // Approximate as a linear least squares problem
  source_it.reset();
  target_it.reset();
  for (; source_it.isValid() && target_it.isValid(); ++source_it, ++target_it) {
    const Vector3 p(source_it->x, source_it->y, source_it->z);
    const Vector3 q(target_it->x, target_it->y, target_it->z);
    const Vector3 n1(source_it->getNormalVector3fMap().template cast<Scalar>());
    const Vector3 n2(target_it->getNormalVector3fMap().template cast<Scalar>());
    Vector3 n;
    if (enforce_same_direction_normals_) {
      if (n1.dot(n2) >= 0.)
        n = n1 + n2;
      else
        n = n1 - n2;
    }
    else {
      n = n1 + n2;
    }

    if (!p.array().isFinite().all() || !q.array().isFinite().all() ||
        !n.array().isFinite().all()) {
      continue;
    }

    Vector6 v;
    v << (p + q).cross(n), n;
    M.rankUpdate(v);

    ATb += v * (q - p).dot(n);
  }

  // Solve A*x = b
  const Vector6 x = M.ldlt().solve(ATb);

  // Construct the transformation matrix from x
  constructTransformationMatrix(x, transformation_matrix);
}

template <typename PointSource, typename PointTarget, typename Scalar>
inline void
TransformationEstimationSymmetricPointToPlaneLLS<PointSource, PointTarget, Scalar>::
    setEnforceSameDirectionNormals(bool enforce_same_direction_normals)
{
  enforce_same_direction_normals_ = enforce_same_direction_normals;
}

template <typename PointSource, typename PointTarget, typename Scalar>
inline bool
TransformationEstimationSymmetricPointToPlaneLLS<PointSource, PointTarget, Scalar>::
    getEnforceSameDirectionNormals()
{
  return enforce_same_direction_normals_;
}

} // namespace registration
} // namespace pcl
