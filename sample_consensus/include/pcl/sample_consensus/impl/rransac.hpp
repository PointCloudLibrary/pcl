/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009, Willow Garage, Inc.
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
 * $Id$
 *
 */

#ifndef PCL_SAMPLE_CONSENSUS_IMPL_RRANSAC_H_
#define PCL_SAMPLE_CONSENSUS_IMPL_RRANSAC_H_

#include <pcl/sample_consensus/rransac.h>

//////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::RandomizedRandomSampleConsensus<PointT>::computeModel (int debug_verbosity_level)
{
  // Warn and exit if no threshold was set
  if (threshold_ == std::numeric_limits<double>::max())
  {
    PCL_ERROR ("[pcl::RandomizedRandomSampleConsensus::computeModel] No threshold set!\n");
    return (false);
  }

  iterations_ = 0;
  int n_best_inliers_count = -INT_MAX;
  double k = 1.0;

  std::vector<int> selection;
  Eigen::VectorXf model_coefficients;
  std::set<int> indices_subset;

  int n_inliers_count = 0;
  unsigned skipped_count = 0;
  // supress infinite loops by just allowing 10 x maximum allowed iterations for invalid model parameters!
  const unsigned max_skip = max_iterations_ * 10;

  // Number of samples to try randomly
  size_t fraction_nr_points = pcl_lrint (static_cast<double>(sac_model_->getIndices ()->size ()) * fraction_nr_pretest_ / 100.0);

  // Iterate
  while (iterations_ < k && skipped_count < max_skip)
  {
    // Get X samples which satisfy the model criteria
    sac_model_->getSamples (iterations_, selection);

    if (selection.empty ()) break;

    // Search for inliers in the point cloud for the current plane model M
    if (!sac_model_->computeModelCoefficients (selection, model_coefficients))
    {
      //iterations_++;
      ++ skipped_count;
      continue;
    }

    // RRANSAC addon: verify a random fraction of the data
    // Get X random samples which satisfy the model criterion
    this->getRandomSamples (sac_model_->getIndices (), fraction_nr_points, indices_subset);
    if (!sac_model_->doSamplesVerifyModel (indices_subset, model_coefficients, threshold_))
    {
      // Unfortunately we cannot "continue" after the first iteration, because k might not be set, while iterations gets incremented
      if (k > 1.0)
      {
        ++iterations_;
        continue;
      }
    }

    // Select the inliers that are within threshold_ from the model
    n_inliers_count = sac_model_->countWithinDistance (model_coefficients, threshold_);

    // Better match ?
    if (n_inliers_count > n_best_inliers_count)
    {
      n_best_inliers_count = n_inliers_count;

      // Save the current model/inlier/coefficients selection as being the best so far
      model_              = selection;
      model_coefficients_ = model_coefficients;

      // Compute the k parameter (k=log(z)/log(1-w^n))
      double w = static_cast<double> (n_inliers_count) / static_cast<double> (sac_model_->getIndices ()->size ());
      double p_no_outliers = 1 - pow (w, static_cast<double> (selection.size ()));
      p_no_outliers = (std::max) (std::numeric_limits<double>::epsilon (), p_no_outliers);       // Avoid division by -Inf
      p_no_outliers = (std::min) (1 - std::numeric_limits<double>::epsilon (), p_no_outliers);   // Avoid division by 0.
      k = log (1 - probability_) / log (p_no_outliers);
    }

    ++iterations_;

    if (debug_verbosity_level > 1)
      PCL_DEBUG ("[pcl::RandomizedRandomSampleConsensus::computeModel] Trial %d out of %d: %d inliers (best is: %d so far).\n", iterations_, static_cast<int> (ceil (k)), n_inliers_count, n_best_inliers_count);
    if (iterations_ > max_iterations_)
    {
      if (debug_verbosity_level > 0)
        PCL_DEBUG ("[pcl::RandomizedRandomSampleConsensus::computeModel] RRANSAC reached the maximum number of trials.\n");
      break;
    }
  }

  if (debug_verbosity_level > 0)
    PCL_DEBUG ("[pcl::RandomizedRandomSampleConsensus::computeModel] Model: %lu size, %d inliers.\n", model_.size (), n_best_inliers_count);

  if (model_.empty ())
  {
    inliers_.clear ();
    return (false);
  }

  // Get the set of inliers that correspond to the best model found so far
  sac_model_->selectWithinDistance (model_coefficients_, threshold_, inliers_);
  return (true);
}

#define PCL_INSTANTIATE_RandomizedRandomSampleConsensus(T) template class PCL_EXPORTS pcl::RandomizedRandomSampleConsensus<T>;

#endif    // PCL_SAMPLE_CONSENSUS_IMPL_RRANSAC_H_

