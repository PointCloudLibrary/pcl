/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception, Inc.
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
#ifndef PCL_REGISTRATION_IMPL_TRANSFORMATION_ESTIMATION_3POINT_H_
#define PCL_REGISTRATION_IMPL_TRANSFORMATION_ESTIMATION_3POINT_H_

#include <pcl/common/eigen.h>
#include <pcl/registration/transformation_estimation_3point.h>

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename Scalar>
inline void
pcl::registration::TransformationEstimation3Point<PointSource, PointTarget, Scalar>::
    estimateRigidTransformation(const pcl::PointCloud<PointSource>& cloud_src,
                                const pcl::PointCloud<PointTarget>& cloud_tgt,
                                Matrix4& transformation_matrix) const
{
  if (cloud_src.size() != 3 || cloud_tgt.size() != 3) {
    PCL_ERROR("[pcl::TransformationEstimation3Point::estimateRigidTransformation] "
              "Number of points in source (%zu) and target (%zu) must be 3!\n",
              static_cast<std::size_t>(cloud_src.size()),
              static_cast<std::size_t>(cloud_tgt.size()));
    return;
  }

  ConstCloudIterator<PointSource> source_it(cloud_src);
  ConstCloudIterator<PointTarget> target_it(cloud_tgt);
  estimateRigidTransformation(source_it, target_it, transformation_matrix);
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename Scalar>
void
pcl::registration::TransformationEstimation3Point<PointSource, PointTarget, Scalar>::
    estimateRigidTransformation(const pcl::PointCloud<PointSource>& cloud_src,
                                const pcl::Indices& indices_src,
                                const pcl::PointCloud<PointTarget>& cloud_tgt,
                                Matrix4& transformation_matrix) const
{
  if (indices_src.size() != 3 || cloud_tgt.size() != 3) {
    PCL_ERROR(
        "[pcl::TransformationEstimation3Point::estimateRigidTransformation] Number of "
        "indices in source (%zu) and points in target (%zu) must be 3!\n",
        indices_src.size(),
        static_cast<std::size_t>(cloud_tgt.size()));
    return;
  }

  ConstCloudIterator<PointSource> source_it(cloud_src, indices_src);
  ConstCloudIterator<PointTarget> target_it(cloud_tgt);
  estimateRigidTransformation(source_it, target_it, transformation_matrix);
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename Scalar>
inline void
pcl::registration::TransformationEstimation3Point<PointSource, PointTarget, Scalar>::
    estimateRigidTransformation(const pcl::PointCloud<PointSource>& cloud_src,
                                const pcl::Indices& indices_src,
                                const pcl::PointCloud<PointTarget>& cloud_tgt,
                                const pcl::Indices& indices_tgt,
                                Matrix4& transformation_matrix) const
{
  if (indices_src.size() != 3 || indices_tgt.size() != 3) {
    PCL_ERROR("[pcl::TransformationEstimation3Point::estimateRigidTransformation] "
              "Number of indices in source (%lu) and target (%lu) must be 3!\n",
              indices_src.size(),
              indices_tgt.size());
    return;
  }

  ConstCloudIterator<PointSource> source_it(cloud_src, indices_src);
  ConstCloudIterator<PointTarget> target_it(cloud_tgt, indices_tgt);
  estimateRigidTransformation(source_it, target_it, transformation_matrix);
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename Scalar>
void
pcl::registration::TransformationEstimation3Point<PointSource, PointTarget, Scalar>::
    estimateRigidTransformation(const pcl::PointCloud<PointSource>& cloud_src,
                                const pcl::PointCloud<PointTarget>& cloud_tgt,
                                const pcl::Correspondences& correspondences,
                                Matrix4& transformation_matrix) const
{
  if (correspondences.size() != 3) {
    PCL_ERROR("[pcl::TransformationEstimation3Point::estimateRigidTransformation] "
              "Number of correspondences (%lu) must be 3!\n",
              correspondences.size());
    return;
  }

  ConstCloudIterator<PointSource> source_it(cloud_src, correspondences, true);
  ConstCloudIterator<PointTarget> target_it(cloud_tgt, correspondences, false);
  estimateRigidTransformation(source_it, target_it, transformation_matrix);
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename Scalar>
inline void
pcl::registration::TransformationEstimation3Point<PointSource, PointTarget, Scalar>::
    estimateRigidTransformation(ConstCloudIterator<PointSource>& source_it,
                                ConstCloudIterator<PointTarget>& target_it,
                                Matrix4& transformation_matrix) const
{
  transformation_matrix.setIdentity();
  source_it.reset();
  target_it.reset();

  Eigen::Matrix<Scalar, 4, 1> source_mean, target_mean;
  pcl::compute3DCentroid(source_it, source_mean);
  pcl::compute3DCentroid(target_it, target_mean);

  source_it.reset();
  target_it.reset();

  Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> source_demean, target_demean;
  pcl::demeanPointCloud(source_it, source_mean, source_demean, 3);
  pcl::demeanPointCloud(target_it, target_mean, target_demean, 3);

  source_it.reset();
  target_it.reset();

  Eigen::Matrix<Scalar, 3, 1> s1 =
      source_demean.col(1).head(3) - source_demean.col(0).head(3);
  s1.normalize();

  Eigen::Matrix<Scalar, 3, 1> s2 =
      source_demean.col(2).head(3) - source_demean.col(0).head(3);
  s2 -= s2.dot(s1) * s1;
  s2.normalize();

  Eigen::Matrix<Scalar, 3, 3> source_rot;
  source_rot.col(0) = s1;
  source_rot.col(1) = s2;
  source_rot.col(2) = s1.cross(s2);

  Eigen::Matrix<Scalar, 3, 1> t1 =
      target_demean.col(1).head(3) - target_demean.col(0).head(3);
  t1.normalize();

  Eigen::Matrix<Scalar, 3, 1> t2 =
      target_demean.col(2).head(3) - target_demean.col(0).head(3);
  t2 -= t2.dot(t1) * t1;
  t2.normalize();

  Eigen::Matrix<Scalar, 3, 3> target_rot;
  target_rot.col(0) = t1;
  target_rot.col(1) = t2;
  target_rot.col(2) = t1.cross(t2);

  // Eigen::Matrix <Scalar, 3, 3> R = source_rot * target_rot.transpose ();
  Eigen::Matrix<Scalar, 3, 3> R = target_rot * source_rot.transpose();
  transformation_matrix.topLeftCorner(3, 3) = R;
  // transformation_matrix.block (0, 3, 3, 1) = source_mean.head (3) - R *
  // target_mean.head (3);
  transformation_matrix.block(0, 3, 3, 1) =
      target_mean.head(3) - R * source_mean.head(3);
}

//#define PCL_INSTANTIATE_TransformationEstimation3Point(T,U) template class PCL_EXPORTS
// pcl::registration::TransformationEstimation3Point<T,U>;

#endif // PCL_REGISTRATION_IMPL_TRANSFORMATION_ESTIMATION_3POINT_H_
