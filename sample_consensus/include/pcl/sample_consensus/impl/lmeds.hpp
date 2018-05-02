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

#ifndef PCL_SAMPLE_CONSENSUS_IMPL_LMEDS_H_
#define PCL_SAMPLE_CONSENSUS_IMPL_LMEDS_H_

#include <pcl/sample_consensus/lmeds.h>

//////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::LeastMedianSquares<PointT>::computeModel (int debug_verbosity_level)
{
  // Warn and exit if no threshold was set
  if (threshold_ == std::numeric_limits<double>::max())
  {
    PCL_ERROR ("[pcl::LeastMedianSquares::computeModel] No threshold set!\n");
    return (false);
  }

  iterations_ = 0;
  double d_best_penalty = std::numeric_limits<double>::max();

  std::vector<int> best_model;
  std::vector<int> selection;
  Eigen::VectorXf model_coefficients;
  std::vector<double> distances;

  int n_inliers_count = 0;

  unsigned skipped_count = 0;
  // suppress infinite loops by just allowing 10 x maximum allowed iterations for invalid model parameters!
  const unsigned max_skip = max_iterations_ * 10;
  
  // Iterate
  while (iterations_ < max_iterations_ && skipped_count < max_skip)
  {
    // Get X samples which satisfy the model criteria
    sac_model_->getSamples (iterations_, selection);

    if (selection.empty ()) break;

    // Search for inliers in the point cloud for the current plane model M
    if (!sac_model_->computeModelCoefficients (selection, model_coefficients))
    {
      //iterations_++;
      ++skipped_count;
      continue;
    }

    double d_cur_penalty = 0;
    // d_cur_penalty = sum (min (dist, threshold))

    // Iterate through the 3d points and calculate the distances from them to the model
    sac_model_->getDistancesToModel (model_coefficients, distances);
    
    // No distances? The model must not respect the user given constraints
    if (distances.empty ())
    {
      //iterations_++;
      ++skipped_count;
      continue;
    }

    std::sort (distances.begin (), distances.end ());
    // d_cur_penalty = median (distances)
    size_t mid = sac_model_->getIndices ()->size () / 2;
    if (mid >= distances.size ())
    {
      //iterations_++;
      ++skipped_count;
      continue;
    }

    // Do we have a "middle" point or should we "estimate" one ?
    if (sac_model_->getIndices ()->size () % 2 == 0)
      d_cur_penalty = (sqrt (distances[mid-1]) + sqrt (distances[mid])) / 2;
    else
      d_cur_penalty = sqrt (distances[mid]);

    // Better match ?
    if (d_cur_penalty < d_best_penalty)
    {
      d_best_penalty = d_cur_penalty;

      // Save the current model/coefficients selection as being the best so far
      model_              = selection;
      model_coefficients_ = model_coefficients;
    }

    ++iterations_;
    if (debug_verbosity_level > 1)
      PCL_DEBUG ("[pcl::LeastMedianSquares::computeModel] Trial %d out of %d. Best penalty is %f.\n", iterations_, max_iterations_, d_best_penalty);
  }

  if (model_.empty ())
  {
    if (debug_verbosity_level > 0)
      PCL_DEBUG ("[pcl::LeastMedianSquares::computeModel] Unable to find a solution!\n");
    return (false);
  }

  // Classify the data points into inliers and outliers
  // Sigma = 1.4826 * (1 + 5 / (n-d)) * sqrt (M)
  // @note: See "Robust Regression Methods for Computer Vision: A Review"
  //double sigma = 1.4826 * (1 + 5 / (sac_model_->getIndices ()->size () - best_model.size ())) * sqrt (d_best_penalty);
  //double threshold = 2.5 * sigma;

  // Iterate through the 3d points and calculate the distances from them to the model again
  sac_model_->getDistancesToModel (model_coefficients_, distances);
  // No distances? The model must not respect the user given constraints
  if (distances.empty ())
  {
    PCL_ERROR ("[pcl::LeastMedianSquares::computeModel] The model found failed to verify against the given constraints!\n");
    return (false);
  }

  std::vector<int> &indices = *sac_model_->getIndices ();

  if (distances.size () != indices.size ())
  {
    PCL_ERROR ("[pcl::LeastMedianSquares::computeModel] Estimated distances (%lu) differs than the normal of indices (%lu).\n", distances.size (), indices.size ());
    return (false);
  }

  inliers_.resize (distances.size ());
  // Get the inliers for the best model found
  n_inliers_count = 0;
  for (size_t i = 0; i < distances.size (); ++i)
    if (distances[i] <= threshold_)
      inliers_[n_inliers_count++] = indices[i];

  // Resize the inliers vector
  inliers_.resize (n_inliers_count);

  if (debug_verbosity_level > 0)
    PCL_DEBUG ("[pcl::LeastMedianSquares::computeModel] Model: %lu size, %d inliers.\n", model_.size (), n_inliers_count);

  return (true);
}

#define PCL_INSTANTIATE_LeastMedianSquares(T) template class PCL_EXPORTS pcl::LeastMedianSquares<T>;

#endif    // PCL_SAMPLE_CONSENSUS_IMPL_LMEDS_H_

