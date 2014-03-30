/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012-, Open Perception Inc.
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
#ifndef PCL_REGISTRATION_TRANSFORMATION_ESTIMATION_2D_HPP_
#define PCL_REGISTRATION_TRANSFORMATION_ESTIMATION_2D_HPP_

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename Scalar> inline void
pcl::registration::TransformationEstimation2D<PointSource, PointTarget, Scalar>::estimateRigidTransformation (
    const pcl::PointCloud<PointSource> &cloud_src,
    const pcl::PointCloud<PointTarget> &cloud_tgt,
    Matrix4 &transformation_matrix) const
{
  size_t nr_points = cloud_src.points.size ();
  if (cloud_tgt.points.size () != nr_points)
  {
    PCL_ERROR ("[pcl::TransformationEstimation2D::estimateRigidTransformation] Number or points in source (%lu) differs than target (%lu)!\n", nr_points, cloud_tgt.points.size ());
    return;
  }

  ConstCloudIterator<PointSource> source_it (cloud_src);
  ConstCloudIterator<PointTarget> target_it (cloud_tgt);
  estimateRigidTransformation (source_it, target_it, transformation_matrix);
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename Scalar> void
pcl::registration::TransformationEstimation2D<PointSource, PointTarget, Scalar>::estimateRigidTransformation (
    const pcl::PointCloud<PointSource> &cloud_src,
    const std::vector<int> &indices_src,
    const pcl::PointCloud<PointTarget> &cloud_tgt,
    Matrix4 &transformation_matrix) const
{
  if (indices_src.size () != cloud_tgt.points.size ())
  {
    PCL_ERROR ("[pcl::Transformation2D::estimateRigidTransformation] Number or points in source (%lu) differs than target (%lu)!\n", indices_src.size (), cloud_tgt.points.size ());
    return;
  }

  ConstCloudIterator<PointSource> source_it (cloud_src, indices_src);
  ConstCloudIterator<PointTarget> target_it (cloud_tgt);
  estimateRigidTransformation (source_it, target_it, transformation_matrix);
}


///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename Scalar> inline void
pcl::registration::TransformationEstimation2D<PointSource, PointTarget, Scalar>::estimateRigidTransformation (
    const pcl::PointCloud<PointSource> &cloud_src,
    const std::vector<int> &indices_src,
    const pcl::PointCloud<PointTarget> &cloud_tgt,
    const std::vector<int> &indices_tgt,
    Matrix4 &transformation_matrix) const
{
  if (indices_src.size () != indices_tgt.size ())
  {
    PCL_ERROR ("[pcl::TransformationEstimation2D::estimateRigidTransformation] Number or points in source (%lu) differs than target (%lu)!\n", indices_src.size (), indices_tgt.size ());
    return;
  }

  ConstCloudIterator<PointSource> source_it (cloud_src, indices_src);
  ConstCloudIterator<PointTarget> target_it (cloud_tgt, indices_tgt);
  estimateRigidTransformation (source_it, target_it, transformation_matrix);
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename Scalar> void
pcl::registration::TransformationEstimation2D<PointSource, PointTarget, Scalar>::estimateRigidTransformation (
    const pcl::PointCloud<PointSource> &cloud_src,
    const pcl::PointCloud<PointTarget> &cloud_tgt,
    const pcl::Correspondences &correspondences,
    Matrix4 &transformation_matrix) const
{
  ConstCloudIterator<PointSource> source_it (cloud_src, correspondences, true);
  ConstCloudIterator<PointTarget> target_it (cloud_tgt, correspondences, false);
  estimateRigidTransformation (source_it, target_it, transformation_matrix);
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename Scalar> inline void
pcl::registration::TransformationEstimation2D<PointSource, PointTarget, Scalar>::estimateRigidTransformation (
    ConstCloudIterator<PointSource>& source_it,
    ConstCloudIterator<PointTarget>& target_it,
    Matrix4 &transformation_matrix) const
{
  source_it.reset (); target_it.reset ();

  Eigen::Matrix<Scalar, 4, 1> centroid_src, centroid_tgt;
  // Estimate the centroids of source, target
  compute3DCentroid (source_it, centroid_src);
  compute3DCentroid (target_it, centroid_tgt);
  source_it.reset (); target_it.reset ();

  // ignore z component
  centroid_src[2] = 0.0f;
  centroid_tgt[2] = 0.0f;
  // Subtract the centroids from source, target
  Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> cloud_src_demean, cloud_tgt_demean;
  demeanPointCloud (source_it, centroid_src, cloud_src_demean);
  demeanPointCloud (target_it, centroid_tgt, cloud_tgt_demean);

  getTransformationFromCorrelation (cloud_src_demean, centroid_src, cloud_tgt_demean, centroid_tgt, transformation_matrix);
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename Scalar> void
pcl::registration::TransformationEstimation2D<PointSource, PointTarget, Scalar>::getTransformationFromCorrelation (
    const Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> &cloud_src_demean,
    const Eigen::Matrix<Scalar, 4, 1> &centroid_src,
    const Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> &cloud_tgt_demean,
    const Eigen::Matrix<Scalar, 4, 1> &centroid_tgt,
    Matrix4 &transformation_matrix) const
{
  transformation_matrix.setIdentity ();

  // Assemble the correlation matrix H = source * target'
  Eigen::Matrix<Scalar, 3, 3> H = (cloud_src_demean * cloud_tgt_demean.transpose ()).topLeftCorner (3, 3);
  
  float angle = atan2 ((H (0, 1) - H (1, 0)), (H(0, 0) + H (1, 1)));
  
  Eigen::Matrix<Scalar, 3, 3> R (Eigen::Matrix<Scalar, 3, 3>::Identity ());
  R (0, 0) = R (1, 1) = cos (angle);
  R (0, 1) = -sin (angle);
  R (1, 0) = sin (angle);

  // Return the correct transformation
  transformation_matrix.topLeftCorner (3, 3).matrix () = R;
  const Eigen::Matrix<Scalar, 3, 1> Rc (R * centroid_src.head (3).matrix ());
  transformation_matrix.block (0, 3, 3, 1).matrix () = centroid_tgt.head (3) - Rc;
}

#endif    // PCL_REGISTRATION_TRANSFORMATION_ESTIMATION_2D_HPP_
