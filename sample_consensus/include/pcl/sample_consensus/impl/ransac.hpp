/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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
 * $Id: ransac.hpp 34393 2010-11-30 23:02:08Z rusu $
 *
 */

#ifndef PCL_SAMPLE_CONSENSUS_IMPL_RANSAC_H_
#define PCL_SAMPLE_CONSENSUS_IMPL_RANSAC_H_

#include "pcl/sample_consensus/ransac.h"

//////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::RandomSampleConsensus<PointT>::computeModel (int debug_verbosity_level)
{
  // Warn and exit if no threshold was set
  if (threshold_ == DBL_MAX)
  {
    ROS_ERROR ("[pcl::RandomSampleConsensus::computeModel] No threshold set!");
    return (false);
  }

  iterations_ = 0;
  int n_best_inliers_count = -INT_MAX;
  double k = 1.0;

  std::vector<int> inliers;
  std::vector<int> selection;
  Eigen::VectorXf model_coefficients;

  int n_inliers_count = 0;

  // Iterate
  while (iterations_ < k)
  {
    // Get X samples which satisfy the model criteria
    sac_model_->getSamples (iterations_, selection);

    if (selection.empty ()) 
    {
      ROS_ERROR ("[pcl::RandomSampleConsensus::computeModel] No samples could be selected!");
      break;
    }

    // Search for inliers in the point cloud for the current plane model M
    if (!sac_model_->computeModelCoefficients (selection, model_coefficients))
    {
      //++iterations_;
      continue;
    }

    // Select the inliers that are within threshold_ from the model
    sac_model_->selectWithinDistance (model_coefficients, threshold_, inliers);
    //if (inliers.empty () && k > 1.0)
    //  continue;

    n_inliers_count = inliers.size ();

    // Better match ?
    if (n_inliers_count > n_best_inliers_count)
    {
      n_best_inliers_count = n_inliers_count;

      // Save the current model/inlier/coefficients selection as being the best so far
      inliers_            = inliers;
      model_              = selection;
      model_coefficients_ = model_coefficients;

      // Compute the k parameter (k=log(z)/log(1-w^n))
      double w = (double)((double)n_best_inliers_count / (double)sac_model_->getIndices ()->size ());
      double p_no_outliers = 1.0 - pow (w, (double)selection.size ());
      p_no_outliers = (std::max) (std::numeric_limits<double>::epsilon (), p_no_outliers);       // Avoid division by -Inf
      p_no_outliers = (std::min) (1.0 - std::numeric_limits<double>::epsilon (), p_no_outliers);   // Avoid division by 0.
      k = log (1.0 - probability_) / log (p_no_outliers);
    }

    ++iterations_;
    if (debug_verbosity_level > 1)
      ROS_DEBUG ("[pcl::RandomSampleConsensus::computeModel] Trial %d out of %f: %d inliers (best is: %d so far).", iterations_, k, n_inliers_count, n_best_inliers_count);
    if (iterations_ > max_iterations_)
    {
      if (debug_verbosity_level > 0)
        ROS_DEBUG ("[pcl::RandomSampleConsensus::computeModel] RANSAC reached the maximum number of trials.");
      break;
    }
  }

  if (debug_verbosity_level > 0)
    ROS_DEBUG ("[pcl::RandomSampleConsensus::computeModel] Model: %zu size, %d inliers.", model_.size (), n_best_inliers_count);

  if (model_.empty ())
  {
    inliers_.clear ();
    return (false);
  }

  // Get the set of inliers that correspond to the best model found so far
  //sac_model_->selectWithinDistance (model_coefficients_, threshold_, inliers_);
  return (true);
}

#define PCL_INSTANTIATE_RandomSampleConsensus(T) template class pcl::RandomSampleConsensus<T>;

#endif    // PCL_SAMPLE_CONSENSUS_IMPL_RANSAC_H_

