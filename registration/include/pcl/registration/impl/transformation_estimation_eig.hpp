/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Jin Wu, Ming Liu, Yilong Zhu
 *                      Hong Kong University of Science and Technology (HKUST)
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

#ifndef PCL_REGISTRATION_TRANSFORMATION_ESTIMATION_EIG_HPP_
#define PCL_REGISTRATION_TRANSFORMATION_ESTIMATION_EIG_HPP_

#include <pcl/common/eigen.h>

 ///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename Scalar> inline void
pcl::registration::TransformationEstimationEIG<PointSource, PointTarget, Scalar>::estimateRigidTransformation (
  const pcl::PointCloud<PointSource> &cloud_src,
  const pcl::PointCloud<PointTarget> &cloud_tgt,
  Matrix4 &transformation_matrix) const
{
  size_t nr_points = cloud_src.points.size ();
  if ( cloud_tgt.points.size () != nr_points )
  {
    PCL_ERROR ( "[pcl::TransformationEstimationEIG::estimateRigidTransformation] Number or points in source (%lu) differs than target (%lu)!\n", nr_points, cloud_tgt.points.size () );
    return;
  }

  ConstCloudIterator<PointSource> source_it ( cloud_src );
  ConstCloudIterator<PointTarget> target_it ( cloud_tgt );
  estimateRigidTransformation ( source_it, target_it, transformation_matrix);
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename Scalar> void
pcl::registration::TransformationEstimationEIG<PointSource, PointTarget, Scalar>::estimateRigidTransformation (
  const pcl::PointCloud<PointSource> &cloud_src,
  const std::vector<int> &indices_src,
  const pcl::PointCloud<PointTarget> &cloud_tgt,
  Matrix4 &transformation_matrix) const
{
  if ( indices_src.size () != cloud_tgt.points.size () )
  {
    PCL_ERROR ( "[pcl::TransformationEIG::estimateRigidTransformation] Number or points in source (%lu) differs than target (%lu)!\n", indices_src.size (), cloud_tgt.points.size () );
    return;
  }

  ConstCloudIterator<PointSource> source_it ( cloud_src, indices_src );
  ConstCloudIterator<PointTarget> target_it ( cloud_tgt );
  estimateRigidTransformation ( source_it, target_it, transformation_matrix);
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename Scalar> inline void
pcl::registration::TransformationEstimationEIG<PointSource, PointTarget, Scalar>::estimateRigidTransformation (
  const pcl::PointCloud<PointSource> &cloud_src,
  const std::vector<int> &indices_src,
  const pcl::PointCloud<PointTarget> &cloud_tgt,
  const std::vector<int> &indices_tgt,
  Matrix4 &transformation_matrix) const
{
  if ( indices_src.size () != indices_tgt.size () )
  {
    PCL_ERROR ( "[pcl::TransformationEstimationEIG::estimateRigidTransformation] Number or points in source (%lu) differs than target (%lu)!\n", indices_src.size (), indices_tgt.size () );
    return;
  }

  ConstCloudIterator<PointSource> source_it ( cloud_src, indices_src );
  ConstCloudIterator<PointTarget> target_it ( cloud_tgt, indices_tgt );
  estimateRigidTransformation ( source_it, target_it, transformation_matrix);
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename Scalar> void
pcl::registration::TransformationEstimationEIG<PointSource, PointTarget, Scalar>::estimateRigidTransformation (
  const pcl::PointCloud<PointSource> &cloud_src,
  const pcl::PointCloud<PointTarget> &cloud_tgt,
  const pcl::Correspondences &correspondences,
  Matrix4 &transformation_matrix) const
{
  ConstCloudIterator<PointSource> source_it ( cloud_src, correspondences, true );
  ConstCloudIterator<PointTarget> target_it ( cloud_tgt, correspondences, false );
  estimateRigidTransformation ( source_it, target_it, transformation_matrix);
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename Scalar> inline void
pcl::registration::TransformationEstimationEIG<PointSource, PointTarget, Scalar>::estimateRigidTransformation (
  ConstCloudIterator<PointSource> &source_it,
  ConstCloudIterator<PointTarget> &target_it,
  Matrix4 &transformation_matrix) const
{
  // Convert to Eigen format
  const int npts = static_cast <int> ( source_it.size () );

  Eigen::Matrix3d sigma;
  Eigen::Vector3d mean_s, mean_t;

  mean_s.setZero ();
  mean_t.setZero ();

  for ( int i = 0; i < npts; ++i )
  {
    mean_s = mean_s + Eigen::Vector3d ( (double)source_it->x, (double)source_it->y, (double)source_it->z );
    mean_t = mean_t + Eigen::Vector3d ( (double)target_it->x, (double)target_it->y, (double)target_it->z );
    source_it++;
    target_it++;
  }
  source_it.reset ();
  target_it.reset ();

  mean_s = mean_s / npts;
  mean_t = mean_t / npts;

  sigma.setZero ();

  for ( int i = 0; i < npts; ++i )
  {
    Eigen::Vector3d source_v ( (double)source_it->x, (double)source_it->y, (double)source_it->z );
    Eigen::Vector3d target_v ( (double)target_it->x, (double)target_it->y, (double)target_it->z );
    sigma = sigma + ( target_v - mean_t ) * ( ( source_v - mean_s ).transpose () );
    source_it++;
    target_it++;
  }
  source_it.reset ();
  target_it.reset ();
  sigma = sigma / npts;

  Eigen::Matrix3d A = ( sigma ) - sigma.transpose ();
  Eigen::Matrix3d tmp;
  Eigen::Vector3d D ( A ( 1, 2 ), A ( 2, 0 ), A ( 0, 1 ) );
  Eigen::Matrix4d QQ;
  QQ ( 0, 0 ) = ( sigma )( 0, 0 ) + ( sigma )( 1, 1 ) + ( sigma )( 2, 2 );
  tmp = ( sigma ) + sigma.transpose ();
  tmp ( 0, 0 ) -= QQ ( 0, 0 );    tmp ( 1, 1 ) -= QQ ( 0, 0 );    tmp ( 2, 2 ) -= QQ ( 0, 0 );
  QQ ( 0, 1 ) = D.x ();     QQ ( 0, 2 ) = D.y ();     QQ ( 0, 3 ) = D.z ();
  QQ ( 1, 0 ) = D.x ();     QQ ( 2, 0 ) = D.y ();     QQ ( 3, 0 ) = D.z ();

  QQ ( 1, 1 ) = tmp ( 0, 0 ); QQ ( 1, 2 ) = tmp ( 0, 1 ); QQ ( 1, 3 ) = tmp ( 0, 2 );
  QQ ( 2, 1 ) = tmp ( 1, 0 ); QQ ( 2, 2 ) = tmp ( 1, 1 ); QQ ( 2, 3 ) = tmp ( 1, 2 );
  QQ ( 3, 1 ) = tmp ( 2, 0 ); QQ ( 3, 2 ) = tmp ( 2, 1 ); QQ ( 3, 3 ) = tmp ( 2, 2 );
  
  Eigen::Quaterniond qRes = Eigen::Quaterniond(1, 0, 0, 0);
  if(use_fs3r)
  {
    double c = QQ.determinant ();
    double b = -8.0 * sigma.determinant ();
    double a = -2.0 * ( ( sigma )( 0, 0 ) * ( sigma )( 0, 0 ) + ( sigma )( 0, 1 ) * ( sigma )( 0, 1 ) + ( sigma )( 0, 2 ) * ( sigma )( 0, 2 ) +
                        ( sigma )( 1, 0 ) * ( sigma )( 1, 0 ) + ( sigma )( 1, 1 ) * ( sigma )( 1, 1 ) + ( sigma )( 1, 2 ) * ( sigma )( 1, 2 ) +
                        ( sigma )( 2, 0 ) * ( sigma )( 2, 0 ) + ( sigma )( 2, 1 ) * ( sigma )( 2, 1 ) + ( sigma )( 2, 2 ) * ( sigma )( 2, 2 ) );

    double T0 = 2.0 * a * a * a + 27.0 * b * b - 72.0 * a * c;
    double tt = a * a + 12.0 * c;
    double theta = atan2 ( sqrt ( 4.0 * tt * tt * tt - T0 * T0 ), T0 );
    double aT1 = 1.259921049894873 * sqrt ( tt ) * cos ( theta * 0.333333333333333333 );
    double T2 = sqrt ( -4.0 * a + 3.174802103936399 * aT1 );
    double lambda = 0.204124145231932 * ( T2 + sqrt ( -T2 * T2 - 12.0 * a - 29.393876913398135 * b / T2 ) );

    double G11 = QQ ( 0, 0 ) - lambda, G12 = QQ ( 0, 1 ), G13 = QQ ( 0, 2 ), G14 = QQ ( 0, 3 );
    double G22 = QQ ( 1, 1 ) - lambda, G23 = QQ ( 1, 2 ), G24 = QQ ( 1, 3 );
    double G33 = QQ ( 2, 2 ) - lambda, G34 = QQ ( 2, 3 );

    qRes = Eigen::Quaterniond (
          G14 * G23 * G23 - G13 * G23 * G24 - G14 * G22 * G33 + G12 * G24 * G33 + G13 * G22 * G34 - G12 * G23 * G34,
          G13 * G13 * G24 + G12 * G14 * G33 - G11 * G24 * G33 + G11 * G23 * G34 - G13 * G14 * G23 - G13 * G12 * G34,
          G13 * G14 * G22 - G12 * G14 * G23 - G12 * G13 * G24 + G11 * G23 * G24 + G12 * G12 * G34 - G11 * G22 * G34,
        -( G13 * G13 * G22 - 2 * G12 * G13 * G23 + G11 * G23 * G23 + G12 * G12 * G33 - G11 * G22 * G33 ) );
  }
  else
  {
    Eigen::EigenSolver<Eigen::Matrix4d> es(QQ);
    Eigen::Matrix4d eVecs = es.eigenvectors().real(); // Keep only the real part of complex matrix
    Eigen::Vector4d eVals = es.eigenvalues().real(); // Keep only the real part of complex matrix

    // Sort by ascending eigenvalues:
    std::vector<std::pair<double, int>> D;
    D.reserve(eVals.size());
    for (int i = 0;i < eVals.size(); i++)
        D.push_back(std::make_pair(eVals.coeff(i, 0), i));
    std::sort(D.begin(), D.end());
    Eigen::Matrix4d sortedEigs;
    sortedEigs.resizeLike(eVecs);
    for (int i = 0; i < eVals.size(); i++)
    {
        eVals.coeffRef(i, 0) = D[i].first;
        sortedEigs.col(i) = eVecs.col(D[i].second);
    }
    eVecs = sortedEigs;
  
    Eigen::Vector4d qq(eVecs.col(3));
    Eigen::Quaterniond qqq(qq(0), qq(1), qq(2), qq(3));
    qRes = qqq;
  }
  
  qRes.normalize ();

  Eigen::Matrix3d rRes = qRes.toRotationMatrix ().transpose();
  Eigen::Vector3d tRes = mean_t - rRes * mean_s;

  transformation_matrix = Matrix4::Identity ();
  transformation_matrix(0, 0) = rRes(0, 0); transformation_matrix(0, 1) = rRes(0, 1); transformation_matrix(0, 2) = rRes(0, 2);
  transformation_matrix(1, 0) = rRes(1, 0); transformation_matrix(1, 1) = rRes(1, 1); transformation_matrix(1, 2) = rRes(1, 2);
  transformation_matrix(2, 0) = rRes(2, 0); transformation_matrix(2, 1) = rRes(2, 1); transformation_matrix(2, 2) = rRes(2, 2);
  transformation_matrix(0, 3) = tRes(0); transformation_matrix(1, 3) = tRes(1); transformation_matrix(2, 3) = tRes(2);
}

//#define PCL_INSTANTIATE_TransformationEstimationEIG(T,U) template class PCL_EXPORTS pcl::registration::TransformationEstimationEIG<T,U>;

#endif /* PCL_REGISTRATION_TRANSFORMATION_ESTIMATION_EIG_HPP_ */
