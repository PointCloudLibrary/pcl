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
 *
 */

#ifndef PCL_REGISTRATION_TRANSFORMATION_ESTIMATION_DQ_HPP_
#define PCL_REGISTRATION_TRANSFORMATION_ESTIMATION_DQ_HPP_

#include <pcl/common/eigen.h>

#include <Eigen/Eigenvalues> // for EigenSolver

namespace pcl {

namespace registration {

template <typename PointSource, typename PointTarget, typename Scalar>
inline void
TransformationEstimationDualQuaternion<PointSource, PointTarget, Scalar>::
    estimateRigidTransformation(const pcl::PointCloud<PointSource>& cloud_src,
                                const pcl::PointCloud<PointTarget>& cloud_tgt,
                                Matrix4& transformation_matrix) const
{
  const auto nr_points = cloud_src.size();
  if (cloud_tgt.size() != nr_points) {
    PCL_ERROR(
        "[pcl::TransformationEstimationDualQuaternion::estimateRigidTransformation] "
        "Number or points in source (%zu) differs than target (%zu)!\n",
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
TransformationEstimationDualQuaternion<PointSource, PointTarget, Scalar>::
    estimateRigidTransformation(const pcl::PointCloud<PointSource>& cloud_src,
                                const pcl::Indices& indices_src,
                                const pcl::PointCloud<PointTarget>& cloud_tgt,
                                Matrix4& transformation_matrix) const
{
  if (indices_src.size() != cloud_tgt.size()) {
    PCL_ERROR("[pcl::TransformationDQ::estimateRigidTransformation] Number or points "
              "in source (%zu) differs than target (%zu)!\n",
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
TransformationEstimationDualQuaternion<PointSource, PointTarget, Scalar>::
    estimateRigidTransformation(const pcl::PointCloud<PointSource>& cloud_src,
                                const pcl::Indices& indices_src,
                                const pcl::PointCloud<PointTarget>& cloud_tgt,
                                const pcl::Indices& indices_tgt,
                                Matrix4& transformation_matrix) const
{
  if (indices_src.size() != indices_tgt.size()) {
    PCL_ERROR(
        "[pcl::TransformationEstimationDualQuaternion::estimateRigidTransformation] "
        "Number or points in source (%lu) differs than target (%lu)!\n",
        indices_src.size(),
        indices_tgt.size());
    return;
  }

  ConstCloudIterator<PointSource> source_it(cloud_src, indices_src);
  ConstCloudIterator<PointTarget> target_it(cloud_tgt, indices_tgt);
  estimateRigidTransformation(source_it, target_it, transformation_matrix);
}

template <typename PointSource, typename PointTarget, typename Scalar>
void
TransformationEstimationDualQuaternion<PointSource, PointTarget, Scalar>::
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
TransformationEstimationDualQuaternion<PointSource, PointTarget, Scalar>::
    estimateRigidTransformation(ConstCloudIterator<PointSource>& source_it,
                                ConstCloudIterator<PointTarget>& target_it,
                                Matrix4& transformation_matrix) const
{
  const int npts = static_cast<int>(source_it.size());

  transformation_matrix.setIdentity();

  // dual quaternion optimization
  Eigen::Matrix<double, 4, 4> C1 = Eigen::Matrix<double, 4, 4>::Zero();
  Eigen::Matrix<double, 4, 4> C2 = Eigen::Matrix<double, 4, 4>::Zero();
  double* c1 = C1.data();
  double* c2 = C2.data();

  for (int i = 0; i < npts; ++i) {
    const PointSource& a = *source_it;
    const PointTarget& b = *target_it;
    const double axbx = a.x * b.x;
    const double ayby = a.y * b.y;
    const double azbz = a.z * b.z;
    const double axby = a.x * b.y;
    const double aybx = a.y * b.x;
    const double axbz = a.x * b.z;
    const double azbx = a.z * b.x;
    const double aybz = a.y * b.z;
    const double azby = a.z * b.y;
    c1[0] += axbx - azbz - ayby;
    c1[5] += ayby - azbz - axbx;
    c1[10] += azbz - axbx - ayby;
    c1[15] += axbx + ayby + azbz;
    c1[1] += axby + aybx;
    c1[2] += axbz + azbx;
    c1[3] += aybz - azby;
    c1[6] += azby + aybz;
    c1[7] += azbx - axbz;
    c1[11] += axby - aybx;

    c2[1] += a.z + b.z;
    c2[2] -= a.y + b.y;
    c2[3] += a.x - b.x;
    c2[6] += a.x + b.x;
    c2[7] += a.y - b.y;
    c2[11] += a.z - b.z;
    ++source_it;
    ++target_it;
  }

  c1[4] = c1[1];
  c1[8] = c1[2];
  c1[9] = c1[6];
  c1[12] = c1[3];
  c1[13] = c1[7];
  c1[14] = c1[11];
  c2[4] = -c2[1];
  c2[8] = -c2[2];
  c2[12] = -c2[3];
  c2[9] = -c2[6];
  c2[13] = -c2[7];
  c2[14] = -c2[11];

  C1 *= -2.0;
  C2 *= 2.0;

  const Eigen::Matrix<double, 4, 4> A =
      (0.25 / double(npts)) * C2.transpose() * C2 - C1;

  const Eigen::EigenSolver<Eigen::Matrix<double, 4, 4>> es(A);

  ptrdiff_t i;
  es.eigenvalues().real().maxCoeff(&i);
  const Eigen::Matrix<double, 4, 1> qmat = es.eigenvectors().col(i).real();
  const Eigen::Matrix<double, 4, 1> smat = -(0.5 / double(npts)) * C2 * qmat;

  const Eigen::Quaternion<double> q(qmat(3), qmat(0), qmat(1), qmat(2));
  const Eigen::Quaternion<double> s(smat(3), smat(0), smat(1), smat(2));

  const Eigen::Quaternion<double> t = s * q.conjugate();

  const Eigen::Matrix<double, 3, 3> R(q.toRotationMatrix());

  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      transformation_matrix(i, j) = R(i, j);

  transformation_matrix(0, 3) = -t.x();
  transformation_matrix(1, 3) = -t.y();
  transformation_matrix(2, 3) = -t.z();
}

} // namespace registration
} // namespace pcl

#endif /* PCL_REGISTRATION_TRANSFORMATION_ESTIMATION_DQ_HPP_ */
