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
 */

#ifndef PCL_PCA_IMPL_HPP
#define PCL_PCA_IMPL_HPP

#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/exceptions.h>

/////////////////////////////////////////////////////////////////////////////////////////
/** \brief Constructor with direct computation
  * \param[in] cloud input m*n matrix (ie n vectors of R(m))
  * \param[in] basis_only flag to compute only the PCA basis
  */
template<typename PointT>
pcl::PCA<PointT>::PCA (const pcl::PointCloud<PointT> &cloud, bool basis_only)
{
  Base ();
  basis_only_ = basis_only;
  setInputCloud (cloud.makeShared ());
  compute_done_ = initCompute ();
}

/////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT> bool
pcl::PCA<PointT>::initCompute () 
{
  if(!Base::initCompute ())
  {
    PCL_THROW_EXCEPTION (InitFailedException, "[pcl::PCA::initCompute] failed");
    return (false);
  }
  if(indices_->size () < 3)
  {
    PCL_THROW_EXCEPTION (InitFailedException, "[pcl::PCA::initCompute] number of points < 3");
    return (false);
  }
  
  // Compute mean
  mean_ = Eigen::Vector4f::Zero ();
  compute3DCentroid (*input_, *indices_, mean_);  
  // Compute demeanished cloud
  Eigen::MatrixXf cloud_demean;
  demeanPointCloud (*input_, *indices_, mean_, cloud_demean);
  assert (cloud_demean.cols () == int (indices_->size ()));
  // Compute the product cloud_demean * cloud_demean^T
  Eigen::Matrix3f alpha = static_cast<Eigen::Matrix3f> (cloud_demean.topRows<3> () * cloud_demean.topRows<3> ().transpose ());
  
  // Compute eigen vectors and values
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> evd (alpha);
  // Organize eigenvectors and eigenvalues in ascendent order
  for (int i = 0; i < 3; ++i)
  {
    eigenvalues_[i] = evd.eigenvalues () [2-i];
    eigenvectors_.col (i) = evd.eigenvectors ().col (2-i);
  }
  // If not basis only then compute the coefficients

  if (!basis_only_)
    coefficients_ = eigenvectors_.transpose() * cloud_demean.topRows<3> ();
  compute_done_ = true;
  return (true);
}

/////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT> inline void 
pcl::PCA<PointT>::update (const PointT& input_point, FLAG flag) 
{
  if (!compute_done_)
    initCompute ();
  if (!compute_done_)
    PCL_THROW_EXCEPTION (InitFailedException, "[pcl::PCA::update] PCA initCompute failed");

  Eigen::Vector3f input (input_point.x, input_point.y, input_point.z);
  const size_t n = eigenvectors_.cols ();// number of eigen vectors
  Eigen::VectorXf meanp = (float(n) * (mean_.head<3>() + input)) / float(n + 1);
  Eigen::VectorXf a = eigenvectors_.transpose() * (input - mean_.head<3>());
  Eigen::VectorXf y = (eigenvectors_ * a) + mean_.head<3>();
  Eigen::VectorXf h = y - input;
  if (h.norm() > 0) 
    h.normalize ();
  else
    h.setZero ();
  float gamma = h.dot(input - mean_.head<3>());
  Eigen::MatrixXf D = Eigen::MatrixXf::Zero (a.size() + 1, a.size() + 1);
  D.block(0,0,n,n) = a * a.transpose();
  D /=  float(n)/float((n+1) * (n+1));
  for(std::size_t i=0; i < a.size(); i++) {
    D(i,i)+= float(n)/float(n+1)*eigenvalues_(i);
    D(D.rows()-1,i) = float(n) / float((n+1) * (n+1)) * gamma * a(i);
    D(i,D.cols()-1) = D(D.rows()-1,i);
    D(D.rows()-1,D.cols()-1) = float(n)/float((n+1) * (n+1)) * gamma * gamma;
  }

  Eigen::MatrixXf R(D.rows(), D.cols());
  Eigen::EigenSolver<Eigen::MatrixXf> D_evd (D, false);
  Eigen::VectorXf alphap = D_evd.eigenvalues().real();
  eigenvalues_.resize(eigenvalues_.size() +1);
  for(std::size_t i=0;i<eigenvalues_.size();i++) {
    eigenvalues_(i) = alphap(eigenvalues_.size()-i-1);
    R.col(i) = D.col(D.cols()-i-1);
  }
  Eigen::MatrixXf Up = Eigen::MatrixXf::Zero(eigenvectors_.rows(), eigenvectors_.cols()+1);
  Up.topLeftCorner(eigenvectors_.rows(),eigenvectors_.cols()) = eigenvectors_;
  Up.rightCols<1>() = h;
  eigenvectors_ = Up*R;
  if (!basis_only_) {
    Eigen::Vector3f etha = Up.transpose() * (mean_.head<3>() - meanp);
    coefficients_.resize(coefficients_.rows()+1,coefficients_.cols()+1);
    for(std::size_t i=0; i < (coefficients_.cols() - 1); i++) {
      coefficients_(coefficients_.rows()-1,i) = 0;
      coefficients_.col(i) = (R.transpose() * coefficients_.col(i)) + etha;
    }
    a.resize(a.size()+1);
    a(a.size()-1) = 0;
    coefficients_.col(coefficients_.cols()-1) = (R.transpose() * a) + etha;
  }
  mean_.head<3>() = meanp;
  switch (flag) 
  {
    case increase:
      if (eigenvectors_.rows() >= eigenvectors_.cols())
        break;
    case preserve:
      if (!basis_only_)
        coefficients_ = coefficients_.topRows(coefficients_.rows() - 1);
      eigenvectors_ = eigenvectors_.leftCols(eigenvectors_.cols() - 1);
      eigenvalues_.resize(eigenvalues_.size()-1);
      break;
    default:
      PCL_ERROR("[pcl::PCA] unknown flag\n");
  }
}

/////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT> inline void
pcl::PCA<PointT>::project (const PointT& input, PointT& projection)
{
  if(!compute_done_)
    initCompute ();
  if (!compute_done_)
    PCL_THROW_EXCEPTION (InitFailedException, "[pcl::PCA::project] PCA initCompute failed");
  
  Eigen::Vector3f demean_input = input.getVector3fMap () - mean_.head<3> ();
  projection.getVector3fMap () = eigenvectors_.transpose() * demean_input;
}

/////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT> inline void
pcl::PCA<PointT>::project (const PointCloud& input, PointCloud& projection)
{
  if(!compute_done_)
    initCompute ();
  if (!compute_done_)
    PCL_THROW_EXCEPTION (InitFailedException, "[pcl::PCA::project] PCA initCompute failed");
  if (input.is_dense)
  {
    projection.resize (input.size ());
    for (size_t i = 0; i < input.size (); ++i)
      project (input[i], projection[i]);
  }
  else
  {
    PointT p;
    for (size_t i = 0; i < input.size (); ++i)
    {
      if (!pcl_isfinite (input[i].x) || 
          !pcl_isfinite (input[i].y) ||
          !pcl_isfinite (input[i].z))
        continue;
      project (input[i], p);
      projection.push_back (p);
    }
  }
}

/////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT> inline void
pcl::PCA<PointT>::reconstruct (const PointT& projection, PointT& input)
{
  if(!compute_done_)
    initCompute ();
  if (!compute_done_)
    PCL_THROW_EXCEPTION (InitFailedException, "[pcl::PCA::reconstruct] PCA initCompute failed");

  input.getVector3fMap ()= eigenvectors_ * projection.getVector3fMap ();
  input.getVector3fMap ()+= mean_.head<3> ();
}

/////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT> inline void
pcl::PCA<PointT>::reconstruct (const PointCloud& projection, PointCloud& input)
{
  if(!compute_done_)
    initCompute ();
  if (!compute_done_)
    PCL_THROW_EXCEPTION (InitFailedException, "[pcl::PCA::reconstruct] PCA initCompute failed");
  if (input.is_dense)
  {
    input.resize (projection.size ());
    for (size_t i = 0; i < projection.size (); ++i)
      reconstruct (projection[i], input[i]);
  }
  else
  {
    PointT p;
    for (size_t i = 0; i < input.size (); ++i)
    {
      if (!pcl_isfinite (input[i].x) || 
          !pcl_isfinite (input[i].y) ||
          !pcl_isfinite (input[i].z))
        continue;
      reconstruct (projection[i], p);
      input.push_back (p);
    }
  }
}

#endif
