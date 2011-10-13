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
#ifndef PCL_REGISTRATION_TRANSFORMATION_ESTIMATION_LM_HPP_
#define PCL_REGISTRATION_TRANSFORMATION_ESTIMATION_LM_HPP_

#include "pcl/registration/warp_point_rigid.h"
#include "pcl/registration/warp_point_rigid_6d.h"
#include "pcl/registration/distances.h"
#include <unsupported/Eigen/NonLinearOptimization>
//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget> void
pcl::registration::TransformationEstimationLM<PointSource, PointTarget>::estimateRigidTransformation (
    const pcl::PointCloud<PointSource> &cloud_src,
    const pcl::PointCloud<PointTarget> &cloud_tgt,
    Eigen::Matrix4f &transformation_matrix)
{

  // <cloud_src,cloud_src> is the source dataset
  if (cloud_src.points.size () != cloud_tgt.points.size ())
  {
    PCL_ERROR ("[pcl::registration::TransformationEstimationLM::estimateRigidTransformation] ");
    PCL_ERROR ("Number or points in source (%lu) differs than target (%lu)!\n", 
               (unsigned long)cloud_src.points.size (), (unsigned long)cloud_tgt.points.size ());
    return;
  }
  if (cloud_src.points.size () < 4)     // need at least 4 samples
  {
    PCL_ERROR ("[pcl::registration::TransformationEstimationLM::estimateRigidTransformation] ");
    PCL_ERROR ("Need at least 4 points to estimate a transform! Source and target have %lu points!\n", 
               (unsigned long)cloud_src.points.size ());
    return;
  }

  // If no warp function has been set, use the default (WarpPointRigid6D)
  if (!warp_point_)
    warp_point_.reset (new WarpPointRigid6D<PointSource, PointTarget>);

  int n_unknowns = warp_point_->getDimension ();
  int m = cloud_src.points.size ();
  Eigen::VectorXd x(n_unknowns);
  x.setZero ();
  
  // Set temporary pointers
  tmp_src_ = &cloud_src;
  tmp_tgt_ = &cloud_tgt;

  OptimizationFunctor functor(n_unknowns, m, this, pcl::distances::l2SqrFunctionPtr);
  Eigen::NumericalDiff<OptimizationFunctor> num_diff(functor);
  Eigen::LevenbergMarquardt<Eigen::NumericalDiff<OptimizationFunctor> > lm(num_diff);
  int info = lm.minimize(x);

  // Compute the norm of the residuals
  PCL_DEBUG ("[pcl::registration::TransformationEstimationLM::estimateRigidTransformation]");
  PCL_DEBUG ("LM solver finished with exit code %i, having a residual norm of %g. \n", info, lm.fvec.norm ());
  if(n_unknowns == 7)
    PCL_DEBUG ("Final solution: [%f %f %f %f] [%f %f %f]\n", x[0], x[1], x[2], x[3], x[4], x[5], x[6]);
  else
    PCL_DEBUG ("Final solution: [%f %f %f] [%f %f %f]\n", x[0], x[1], x[2], x[3], x[4], x[5]);
  
  // Return the correct transformation
  transformation_matrix.setZero ();

  Eigen::Quaternionf q (0, x[3], x[4], x[5]);
  q.w () = sqrt (1 - q.dot (q));   // Compute w from the unit quaternion
  transformation_matrix.topLeftCorner<3, 3> () = q.toRotationMatrix ();

  Eigen::Vector4f t (x[0], x[1], x[2], 1.0);
  transformation_matrix.block <4, 1> (0, 3) = t;

  tmp_src_ = tmp_tgt_ = NULL;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget> void
pcl::registration::TransformationEstimationLM<PointSource, PointTarget>::estimateRigidTransformation (
    const pcl::PointCloud<PointSource> &cloud_src,
    const std::vector<int> &indices_src,
    const pcl::PointCloud<PointTarget> &cloud_tgt,
    Eigen::Matrix4f &transformation_matrix)
{
  if (indices_src.size () != cloud_tgt.points.size ())
  {
    PCL_ERROR ("[pcl::registration::TransformationEstimationLM::estimateRigidTransformation] Number or points in source (%lu) differs than target (%lu)!\n", (unsigned long)indices_src.size (), (unsigned long)cloud_tgt.points.size ());
    return;
  }

  // <cloud_src,cloud_src> is the source dataset
  transformation_matrix.setIdentity ();

  const int nr_correspondences = (int)cloud_tgt.points.size ();
  std::vector<int> indices_tgt;
  indices_tgt.resize(nr_correspondences);
  for (int i = 0; i < nr_correspondences; ++i)
    indices_tgt[i] = i;

  estimateRigidTransformation(cloud_src, indices_src, cloud_tgt, indices_tgt, transformation_matrix);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget> inline void
pcl::registration::TransformationEstimationLM<PointSource, PointTarget>::estimateRigidTransformation (
    const pcl::PointCloud<PointSource> &cloud_src,
    const std::vector<int> &indices_src,
    const pcl::PointCloud<PointTarget> &cloud_tgt,
    const std::vector<int> &indices_tgt,
    Eigen::Matrix4f &transformation_matrix)
{
  if (indices_src.size () != indices_tgt.size ())
  {
    PCL_ERROR ("[pcl::registration::TransformationEstimationLM::estimateRigidTransformation] Number or points in source (%lu) differs than target (%lu)!\n", (unsigned long)indices_src.size (), (unsigned long)indices_tgt.size ());
    return;
  }

  if (indices_src.size () < 4)     // need at least 4 samples
  {
    PCL_ERROR ("[pcl::IterativeClosestPointNonLinear::estimateRigidTransformationLM] ");
    PCL_ERROR ("Need at least 4 points to estimate a transform! Source and target have %lu points!",
               (unsigned long)indices_src.size ());
    return;
  }

  // If no warp function has been set, use the default (WarpPointRigid6D)
  if (!warp_point_)
    warp_point_.reset (new WarpPointRigid6D<PointSource, PointTarget>);

  int n_unknowns = warp_point_->getDimension ();  // get dimension of unknown space
  int m = indices_src.size ();
  Eigen::VectorXd x(n_unknowns);
  x.setConstant (n_unknowns, 0);

  // Set temporary pointers
  tmp_src_ = &cloud_src;
  tmp_tgt_ = &cloud_tgt;
  tmp_idx_src_ = &indices_src;
  tmp_idx_tgt_ = &indices_tgt;

  OptimizationFunctorWithIndices functor(n_unknowns, m, this, pcl::distances::l2SqrFunctionPtr);
  Eigen::NumericalDiff<OptimizationFunctorWithIndices> num_diff(functor);
  Eigen::LevenbergMarquardt<Eigen::NumericalDiff<OptimizationFunctorWithIndices> > lm(num_diff);
  int info = lm.minimize(x);

  // Compute the norm of the residuals
  PCL_DEBUG ("[pcl::registration::TransformationEstimationLM::estimateRigidTransformation]");
  PCL_DEBUG ("LM solver finished with exit code %i, having a residual norm of %g. \n", info, lm.fvec.norm ());
  PCL_DEBUG ("Final solution: [%f %f %f] [%f %f %f]\n", x[0], x[1], x[2], x[3], x[4], x[5]);

  // Return the correct transformation
  transformation_matrix.setZero ();

  Eigen::Quaternionf q (0, x[3], x[4], x[5]);
  q.w () = sqrt (1 - q.dot (q));   // Compute w from the unit quaternion
  transformation_matrix.topLeftCorner<3, 3> () = q.toRotationMatrix ();

  Eigen::Vector4f t (x[0], x[1], x[2], 1.0);
  transformation_matrix.block <4, 1> (0, 3) = t;

  tmp_src_ = tmp_tgt_ = NULL;
  tmp_idx_src_ = tmp_idx_tgt_ = NULL;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget> inline void
pcl::registration::TransformationEstimationLM<PointSource, PointTarget>::estimateRigidTransformation (
    const pcl::PointCloud<PointSource> &cloud_src,
    const pcl::PointCloud<PointTarget> &cloud_tgt,
    const pcl::Correspondences &correspondences,
    Eigen::Matrix4f &transformation_matrix)
{
  const int nr_correspondences = (int)correspondences.size();
  std::vector<int> indices_src(nr_correspondences);
  std::vector<int> indices_tgt(nr_correspondences);
  for (int i = 0; i < nr_correspondences; ++i)
  {
    indices_src[i] = correspondences[i].index_query;
    indices_tgt[i] = correspondences[i].index_match;
  }

  estimateRigidTransformation(cloud_src, indices_src, cloud_tgt, indices_tgt, transformation_matrix);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget> int 
pcl::registration::TransformationEstimationLM<PointSource, PointTarget>::OptimizationFunctor::operator() (const Eigen::VectorXd &x, Eigen::VectorXd &fvec) const
{
  // Copy the rotation and translation components
  Eigen::Vector4f t(x[0], x[1], x[2], 1.0);
  
  // Compute w from the unit quaternion
  Eigen::Quaternionf q (0, x[3], x[4], x[5]);
  q.w () = sqrt (1 - q.dot (q));
  
  // If the quaternion is used to rotate several points (>1) then it is much more efficient to first convert it
  // to a 3x3 Matrix. Comparison of the operation cost for n transformations:
  // * Quaternion: 30n
  // * Via a Matrix3: 24 + 15n
  Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Zero ();
  transformation_matrix.topLeftCorner<3, 3> () = q.toRotationMatrix ();
  transformation_matrix.block <4, 1> (0, 3) = t;
  
  //double sigma = estimator_->getMaxCorrespondenceDistance ();
  for (int i = 0; i < m_values; i++)
  {
    // The last coordinate, p_src[3] is guaranteed to be set to 1.0 in registration.hpp
    Vector4fMapConst p_src = estimator_->tmp_src_->points[i].getVector4fMap ();
    // The last coordinate, p_tgt[3] is guaranteed to be set to 1.0 in registration.hpp
    Vector4fMapConst p_tgt = estimator_->tmp_tgt_->points[i].getVector4fMap ();
    
    Eigen::Vector4f pp = transformation_matrix * p_src;
    
    // Estimate the distance (cost function)
    fvec[i] = distance_ (p_tgt, pp);
  }
  return 0;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget> int
pcl::registration::TransformationEstimationLM<PointSource, PointTarget>::OptimizationFunctorWithIndices::operator() (const Eigen::VectorXd &x, Eigen::VectorXd &fvec) const
{
  Eigen::VectorXf params = x.cast<float> ();
  estimator_->warp_point_->setParam (params);
  
  //double sigma = estimator_->getMaxCorrespondenceDistance ();
  for (int i = 0; i < m_values; ++i)
  {
    // The last coordinate, p_src[3] is guaranteed to be set to 1.0 in registration.hpp
    const PointSource &p_src = estimator_->tmp_src_->points[(*estimator_->tmp_idx_src_)[i]];
    // The last coordinate, p_tgt[3] is guaranteed to be set to 1.0 in registration.hpp
    Vector4fMapConst p_tgt = estimator_->tmp_tgt_->points[(*estimator_->tmp_idx_tgt_)[i]].getVector4fMap ();
    
    PointTarget pp;
    estimator_->warp_point_->warpPoint (p_src, pp);
    
    // Estimate the distance (cost function)
    fvec[i] =  distance_ (p_tgt, pp.getVector4fMap());
  }
  return (0);
}

//#define PCL_INSTANTIATE_TransformationEstimationLM(T,U) template class PCL_EXPORTS pcl::registration::TransformationEstimationLM<T,U>;

#endif /* PCL_REGISTRATION_TRANSFORMATION_ESTIMATION_LM_HPP_ */
