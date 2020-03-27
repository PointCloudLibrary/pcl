/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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

#include <pcl/common/eigen.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
inline void
pcl::TransformationFromCorrespondences::reset ()
{
  no_of_samples_ = 0;
  accumulated_weight_ = 0.0;
  mean1_.fill(0);
  mean2_.fill(0);
  covariance_.fill(0);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
inline void
pcl::TransformationFromCorrespondences::add (const Eigen::Vector3f& point, const Eigen::Vector3f& corresponding_point,
                                             float weight)
{
  if (weight==0.0f)
    return;
  
  ++no_of_samples_;
  accumulated_weight_ += weight;
  float alpha = weight/accumulated_weight_;
  
  Eigen::Vector3f diff1 = point - mean1_, diff2 = corresponding_point - mean2_;
  covariance_ = (1.0f-alpha)*(covariance_ + alpha * (diff2 * diff1.transpose()));
  
  mean1_ += alpha*(diff1);
  mean2_ += alpha*(diff2);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
inline Eigen::Affine3f
pcl::TransformationFromCorrespondences::getTransformation ()
{
  //Eigen::JacobiSVD<Eigen::Matrix<float, 3, 3> > svd (covariance_, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::JacobiSVD<Eigen::Matrix<float, 3, 3> > svd (covariance_, Eigen::ComputeFullU | Eigen::ComputeFullV);
  const Eigen::Matrix<float, 3, 3>& u = svd.matrixU(),
                                   & v = svd.matrixV();
  Eigen::Matrix<float, 3, 3> s;
  s.setIdentity();
  if (u.determinant()*v.determinant() < 0.0f)
    s(2,2) = -1.0f;
  
  Eigen::Matrix<float, 3, 3> r = u * s * v.transpose();
  Eigen::Vector3f t = mean2_ - r*mean1_;
  
  Eigen::Affine3f ret;
  ret(0,0)=r(0,0); ret(0,1)=r(0,1); ret(0,2)=r(0,2); ret(0,3)=t(0);
  ret(1,0)=r(1,0); ret(1,1)=r(1,1); ret(1,2)=r(1,2); ret(1,3)=t(1);
  ret(2,0)=r(2,0); ret(2,1)=r(2,1); ret(2,2)=r(2,2); ret(2,3)=t(2);
  ret(3,0)=0.0f;   ret(3,1)=0.0f;   ret(3,2)=0.0f;   ret(3,3)=1.0f;
  
  return (ret);
}
