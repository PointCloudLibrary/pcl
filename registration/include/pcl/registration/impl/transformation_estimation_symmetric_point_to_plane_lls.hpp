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

#ifndef PCL_REGISTRATION_TRANSFORMATION_ESTIMATION_SYMMETRIC_POINT_TO_PLANE_LLS_HPP_
#define PCL_REGISTRATION_TRANSFORMATION_ESTIMATION_SYMMETRIC_POINT_TO_PLANE_LLS_HPP_
#include <pcl/cloud_iterator.h>

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename Scalar> inline void
pcl::registration::TransformationEstimationSymmetricPointToPlaneLLS<PointSource, PointTarget, Scalar>::
estimateRigidTransformation (const pcl::PointCloud<PointSource> &cloud_src,
                             const pcl::PointCloud<PointTarget> &cloud_tgt,
                             Matrix4 &transformation_matrix) const
{
  const auto nr_points = cloud_src.points.size ();
  if (cloud_tgt.points.size () != nr_points)
  {
    PCL_ERROR ("[pcl::TransformationEstimationSymmetricPointToPlaneLLS::estimateRigidTransformation] Number or points in source (%lu) differs than target (%lu)!\n", nr_points, cloud_tgt.points.size ());
    return;
  }

  ConstCloudIterator<PointSource> source_it (cloud_src);
  ConstCloudIterator<PointTarget> target_it (cloud_tgt);
  estimateRigidTransformation (source_it, target_it, transformation_matrix);  
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename Scalar> void
pcl::registration::TransformationEstimationSymmetricPointToPlaneLLS<PointSource, PointTarget, Scalar>::
estimateRigidTransformation (const pcl::PointCloud<PointSource> &cloud_src,
                             const std::vector<int> &indices_src,
                             const pcl::PointCloud<PointTarget> &cloud_tgt,
                             Matrix4 &transformation_matrix) const
{
  const auto nr_points = indices_src.size ();
  if (cloud_tgt.points.size () != nr_points)
  {
    PCL_ERROR ("[pcl::TransformationEstimationSymmetricPointToPlaneLLS::estimateRigidTransformation] Number or points in source (%lu) differs than target (%lu)!\n", indices_src.size (), cloud_tgt.points.size ());
    return;
  }

  ConstCloudIterator<PointSource> source_it (cloud_src, indices_src);
  ConstCloudIterator<PointTarget> target_it (cloud_tgt);
  estimateRigidTransformation (source_it, target_it, transformation_matrix);  
}


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename Scalar> inline void
pcl::registration::TransformationEstimationSymmetricPointToPlaneLLS<PointSource, PointTarget, Scalar>::
estimateRigidTransformation (const pcl::PointCloud<PointSource> &cloud_src,
                             const std::vector<int> &indices_src,
                             const pcl::PointCloud<PointTarget> &cloud_tgt,
                             const std::vector<int> &indices_tgt,
                             Matrix4 &transformation_matrix) const
{
  const auto nr_points = indices_src.size ();
  if (indices_tgt.size () != nr_points)
  {
    PCL_ERROR ("[pcl::TransformationEstimationSymmetricPointToPlaneLLS::estimateRigidTransformation] Number or points in source (%lu) differs than target (%lu)!\n", indices_src.size (), indices_tgt.size ());
    return;
  }

  ConstCloudIterator<PointSource> source_it (cloud_src, indices_src);
  ConstCloudIterator<PointTarget> target_it (cloud_tgt, indices_tgt);
  estimateRigidTransformation (source_it, target_it, transformation_matrix);  
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename Scalar> inline void
pcl::registration::TransformationEstimationSymmetricPointToPlaneLLS<PointSource, PointTarget, Scalar>::
estimateRigidTransformation (const pcl::PointCloud<PointSource> &cloud_src,
                             const pcl::PointCloud<PointTarget> &cloud_tgt,
                             const pcl::Correspondences &correspondences,
                             Matrix4 &transformation_matrix) const
{
  ConstCloudIterator<PointSource> source_it (cloud_src, correspondences, true);
  ConstCloudIterator<PointTarget> target_it (cloud_tgt, correspondences, false);
  estimateRigidTransformation (source_it, target_it, transformation_matrix);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename Scalar> inline void
pcl::registration::TransformationEstimationSymmetricPointToPlaneLLS<PointSource, PointTarget, Scalar>::
constructTransformationMatrix (const Vector6d &parameters,
                               Matrix4 &transformation_matrix) const
{
  // Construct the transformation matrix from rotation and translation 
  Eigen::Matrix<Scalar, 3, 3> rotation_matrix;
  rotation_matrix = Eigen::AngleAxis<Scalar> (parameters (2), Eigen::Matrix<Scalar, 3, 1>::UnitZ ()) * 
                    Eigen::AngleAxis<Scalar> (parameters (1), Eigen::Matrix<Scalar, 3, 1>::UnitY ()) *
                    Eigen::AngleAxis<Scalar> (parameters (0), Eigen::Matrix<Scalar, 3, 1>::UnitX ());
  Eigen::Matrix<Scalar, 3, 1> translation_vector;
  translation_vector << static_cast<Scalar> (parameters (3)), static_cast<Scalar> (parameters (4)), static_cast<Scalar> (parameters (5));
  transformation_matrix.setIdentity();
  transformation_matrix.template block<3, 3> (0, 0) = rotation_matrix;
  transformation_matrix.template block<3, 1> (0, 3) = translation_vector;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename Scalar> inline void
pcl::registration::TransformationEstimationSymmetricPointToPlaneLLS<PointSource, PointTarget, Scalar>::
estimateRigidTransformation (ConstCloudIterator<PointSource>& source_it, ConstCloudIterator<PointTarget>& target_it, Matrix4 &transformation_matrix) const
{
  using Matrix6d = Eigen::Matrix<double, 6, 6>;

  Matrix6d ATA;
  Vector6d ATb;
  ATA.setZero ();
  ATb.setZero ();
  auto U = ATA.triangularView<Eigen::Upper> ();

  // Approximate as a linear least squares problem
  for (; source_it.isValid () && target_it.isValid (); ++source_it, ++target_it)
  {
    if (!isFinite(*source_it) ||
        !isFinite(*target_it))
    {
      continue;
    }

    const float & sx = source_it->x;
    const float & sy = source_it->y;
    const float & sz = source_it->z;
    const float & dx = target_it->x;
    const float & dy = target_it->y;
    const float & dz = target_it->z;
    const float & nx = source_it->normal[0] + target_it->normal[0];
    const float & ny = source_it->normal[1] + target_it->normal[1];
    const float & nz = source_it->normal[2] + target_it->normal[2];

    double a = nz*sy - ny*sz;
    double b = nx*sz - nz*sx; 
    double c = ny*sx - nx*sy;
   
    U.coeffRef (0, 0) += a * a;
    U.coeffRef (0, 1) += a * b;
    U.coeffRef (0, 2) += a * c;
    U.coeffRef (0, 3) += a * nx;
    U.coeffRef (0, 4) += a * ny;
    U.coeffRef (0, 5) += a * nz;
    U.coeffRef (1, 1) += b * b;
    U.coeffRef (1, 2) += b * c;
    U.coeffRef (1, 3) += b * nx;
    U.coeffRef (1, 4) += b * ny;
    U.coeffRef (1, 5) += b * nz;
    U.coeffRef (2, 2) += c * c;
    U.coeffRef (2, 3) += c * nx;
    U.coeffRef (2, 4) += c * ny;
    U.coeffRef (2, 5) += c * nz;
    U.coeffRef (3, 3) += nx * nx;
    U.coeffRef (3, 4) += nx * ny;
    U.coeffRef (3, 5) += nx * nz;
    U.coeffRef (4, 4) += ny * ny;
    U.coeffRef (4, 5) += ny * nz;
    U.coeffRef (5, 5) += nz * nz;

    double d = nx*dx + ny*dy + nz*dz - nx*sx - ny*sy - nz*sz;
    ATb.coeffRef (0) += a * d;
    ATb.coeffRef (1) += b * d;
    ATb.coeffRef (2) += c * d;
    ATb.coeffRef (3) += nx * d;
    ATb.coeffRef (4) += ny * d;
    ATb.coeffRef (5) += nz * d;
  }

  // Solve A*x = b
  Vector6d x = static_cast<Vector6d> (ATA.selfadjointView<Eigen::Upper> ().ldlt ().solve (ATb));
  
  // Construct the transformation matrix from x
  constructTransformationMatrix (x, transformation_matrix);
}
#endif /* PCL_REGISTRATION_TRANSFORMATION_ESTIMATION_SYMMETRIC_POINT_TO_PLANE_LLS_HPP_ */
