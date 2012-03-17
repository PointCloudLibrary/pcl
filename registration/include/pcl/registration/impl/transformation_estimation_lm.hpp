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

#include <pcl/registration/warp_point_rigid.h>
#include <pcl/registration/warp_point_rigid_6d.h>
#include <pcl/registration/distances.h>
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
    PCL_ERROR ("Number or points in source (%zu) differs than target (%zu)!\n", 
               cloud_src.points.size (), cloud_tgt.points.size ());
    return;
  }
  if (cloud_src.points.size () < 4)     // need at least 4 samples
  {
    PCL_ERROR ("[pcl::registration::TransformationEstimationLM::estimateRigidTransformation] ");
    PCL_ERROR ("Need at least 4 points to estimate a transform! Source and target have %zu points!\n", 
               cloud_src.points.size ());
    return;
  }

  // If no warp function has been set, use the default (WarpPointRigid6D)
  if (!warp_point_)
    warp_point_.reset (new WarpPointRigid6D<PointSource, PointTarget>);

  int n_unknowns = warp_point_->getDimension ();
  Eigen::VectorXd x (n_unknowns);
  x.setZero ();
  
  // Set temporary pointers
  tmp_src_ = &cloud_src;
  tmp_tgt_ = &cloud_tgt;

  OptimizationFunctor functor (static_cast<int> (cloud_src.points.size ()), this);
  Eigen::NumericalDiff<OptimizationFunctor> num_diff (functor);
  Eigen::LevenbergMarquardt<Eigen::NumericalDiff<OptimizationFunctor>, double> lm (num_diff);
  int info = lm.minimize (x);

  // Compute the norm of the residuals
  PCL_DEBUG ("[pcl::registration::TransformationEstimationLM::estimateRigidTransformation]");
  PCL_DEBUG ("LM solver finished with exit code %i, having a residual norm of %g. \n", info, lm.fvec.norm ());
  PCL_DEBUG ("Final solution: [%f", x[0]);
  for (int i = 1; i < n_unknowns; ++i) 
    PCL_DEBUG (" %f", x[i]);
  PCL_DEBUG ("]\n");

  // Return the correct transformation
  Eigen::VectorXf params = x.cast<float> ();
  warp_point_->setParam (params);
  transformation_matrix = warp_point_->getTransform ();

  tmp_src_ = NULL;
  tmp_tgt_ = NULL;
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
    PCL_ERROR ("[pcl::registration::TransformationEstimationLM::estimateRigidTransformation] Number or points in source (%zu) differs than target (%zu)!\n", indices_src.size (), cloud_tgt.points.size ());
    return;
  }

  // <cloud_src,cloud_src> is the source dataset
  transformation_matrix.setIdentity ();

  const int nr_correspondences = static_cast<const int> (cloud_tgt.points.size ());
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
    PCL_ERROR ("[pcl::registration::TransformationEstimationLM::estimateRigidTransformation] Number or points in source (%zu) differs than target (%zu)!\n", indices_src.size (), indices_tgt.size ());
    return;
  }

  if (indices_src.size () < 4)     // need at least 4 samples
  {
    PCL_ERROR ("[pcl::IterativeClosestPointNonLinear::estimateRigidTransformationLM] ");
    PCL_ERROR ("Need at least 4 points to estimate a transform! Source and target have %zu points!",
               indices_src.size ());
    return;
  }

  // If no warp function has been set, use the default (WarpPointRigid6D)
  if (!warp_point_)
    warp_point_.reset (new WarpPointRigid6D<PointSource, PointTarget>);

  int n_unknowns = warp_point_->getDimension ();  // get dimension of unknown space
  Eigen::VectorXd x (n_unknowns);
  x.setConstant (n_unknowns, 0);

  // Set temporary pointers
  tmp_src_ = &cloud_src;
  tmp_tgt_ = &cloud_tgt;
  tmp_idx_src_ = &indices_src;
  tmp_idx_tgt_ = &indices_tgt;

  OptimizationFunctorWithIndices functor (static_cast<int> (indices_src.size ()), this);
  Eigen::NumericalDiff<OptimizationFunctorWithIndices> num_diff (functor);
  Eigen::LevenbergMarquardt<Eigen::NumericalDiff<OptimizationFunctorWithIndices> > lm (num_diff);
  int info = lm.minimize (x);

  // Compute the norm of the residuals
  PCL_DEBUG ("[pcl::registration::TransformationEstimationLM::estimateRigidTransformation]");
  PCL_DEBUG ("LM solver finished with exit code %i, having a residual norm of %g. \n", info, lm.fvec.norm ());
  PCL_DEBUG ("Final solution: [%f", x[0]);
  for (int i = 1; i < n_unknowns; ++i) 
    PCL_DEBUG (" %f", x[i]);
  PCL_DEBUG ("]\n");

  // Return the correct transformation
  Eigen::VectorXf params = x.cast<float> ();
  warp_point_->setParam (params);
  transformation_matrix = warp_point_->getTransform ();

  tmp_src_ = NULL;
  tmp_tgt_ = NULL;
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
  const int nr_correspondences = static_cast<const int> (correspondences.size ());
  std::vector<int> indices_src (nr_correspondences);
  std::vector<int> indices_tgt (nr_correspondences);
  for (int i = 0; i < nr_correspondences; ++i)
  {
    indices_src[i] = correspondences[i].index_query;
    indices_tgt[i] = correspondences[i].index_match;
  }

  estimateRigidTransformation(cloud_src, indices_src, cloud_tgt, indices_tgt, transformation_matrix);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget> int 
pcl::registration::TransformationEstimationLM<PointSource, PointTarget>::OptimizationFunctor::operator () (
    const Eigen::VectorXd &x, Eigen::VectorXd &fvec) const
{
  const PointCloud<PointSource> & src_points = *estimator_->tmp_src_;
  const PointCloud<PointTarget> & tgt_points = *estimator_->tmp_tgt_;

  // Initialize the warp function with the given parameters
  Eigen::VectorXf params = x.cast<float> ();
  estimator_->warp_point_->setParam (params);

  // Transform each source point and compute its distance to the corresponding target point
  for (int i = 0; i < values (); ++i)
  {
    const PointSource & p_src = src_points.points[i];
    const PointTarget & p_tgt = tgt_points.points[i];

    // Transform the source point based on the current warp parameters
    PointSource p_src_warped;
    estimator_->warp_point_->warpPoint (p_src, p_src_warped);
    
    // Estimate the distance (cost function)
    fvec[i] = estimator_->computeDistance (p_src_warped, p_tgt);
  }
  return (0);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget> int
pcl::registration::TransformationEstimationLM<PointSource, PointTarget>::OptimizationFunctorWithIndices::operator() (
    const Eigen::VectorXd &x, Eigen::VectorXd &fvec) const
{
  const PointCloud<PointSource> & src_points = *estimator_->tmp_src_;
  const PointCloud<PointTarget> & tgt_points = *estimator_->tmp_tgt_;
  const std::vector<int> & src_indices = *estimator_->tmp_idx_src_;
  const std::vector<int> & tgt_indices = *estimator_->tmp_idx_tgt_;

  // Initialize the warp function with the given parameters
  Eigen::VectorXf params = x.cast<float> ();
  estimator_->warp_point_->setParam (params);

  // Transform each source point and compute its distance to the corresponding target point
  for (int i = 0; i < values (); ++i)
  {
    const PointSource & p_src = src_points.points[src_indices[i]];
    const PointTarget & p_tgt = tgt_points.points[tgt_indices[i]];

    // Transform the source point based on the current warp parameters
    PointSource p_src_warped;
    estimator_->warp_point_->warpPoint (p_src, p_src_warped);
    
    // Estimate the distance (cost function)
    fvec[i] = estimator_->computeDistance (p_src_warped, p_tgt);
  }
  return (0);
}
/*
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget> inline double 
pcl::registration::TransformationEstimationLM<PointSource, PointTarget>::computeMedian (double *fvec, int m)
{
  double median;
  // Copy the values to vectors for faster sorting
  std::vector<double> data (m);
  memcpy (&data[0], fvec, sizeof (double) * m);

  std::sort (data.begin (), data.end ());

  int mid = data.size () / 2;
  if (data.size () % 2 == 0)
    median = (data[mid-1] + data[mid]) / 2.0;
  else
    median = data[mid];
  return (median);
}
*/

//#define PCL_INSTANTIATE_TransformationEstimationLM(T,U) template class PCL_EXPORTS pcl::registration::TransformationEstimationLM<T,U>;

#endif /* PCL_REGISTRATION_TRANSFORMATION_ESTIMATION_LM_HPP_ */
