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

#ifndef PCL_SAMPLE_CONSENSUS_IMPL_MLESAC_H_
#define PCL_SAMPLE_CONSENSUS_IMPL_MLESAC_H_

#include <pcl/sample_consensus/mlesac.h>
#include <cfloat> // for FLT_MAX
#include <pcl/common/common.h> // for computeMedian

//////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::MaximumLikelihoodSampleConsensus<PointT>::computeModel (int debug_verbosity_level)
{
  // Warn and exit if no threshold was set
  if (threshold_ == std::numeric_limits<double>::max())
  {
    PCL_ERROR ("[pcl::MaximumLikelihoodSampleConsensus::computeModel] No threshold set!\n");
    return (false);
  }

  iterations_ = 0;
  double d_best_penalty = std::numeric_limits<double>::max();
  double k = 1.0;

  Indices selection;
  Eigen::VectorXf model_coefficients (sac_model_->getModelSize ());
  std::vector<double> distances;

  // Compute sigma - remember to set threshold_ correctly !
  sigma_ = computeMedianAbsoluteDeviation (sac_model_->getInputCloud (), sac_model_->getIndices (), threshold_);
  const double dist_scaling_factor = -1.0 / (2.0 * sigma_ * sigma_); // Precompute since this does not change
  const double normalization_factor = 1.0 / (sqrt (2 * M_PI) * sigma_);
  if (debug_verbosity_level > 1)
    PCL_DEBUG ("[pcl::MaximumLikelihoodSampleConsensus::computeModel] Estimated sigma value: %f.\n", sigma_);

  // Compute the bounding box diagonal: V = sqrt (sum (max(pointCloud) - min(pointCloud)^2))
  Eigen::Vector4f min_pt, max_pt;
  getMinMax (sac_model_->getInputCloud (), sac_model_->getIndices (), min_pt, max_pt);
  max_pt -= min_pt;
  double v = sqrt (max_pt.dot (max_pt));

  int n_inliers_count = 0;
  std::size_t indices_size;
  unsigned skipped_count = 0;
  // suppress infinite loops by just allowing 10 x maximum allowed iterations for invalid model parameters!
  const unsigned max_skip = max_iterations_ * 10;
  
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

    // Iterate through the 3d points and calculate the distances from them to the model
    sac_model_->getDistancesToModel (model_coefficients, distances);

    if (distances.empty ())
    {
      //iterations_++;
      ++skipped_count;
      continue;
    }
    
    // Use Expectation-Maximization to find out the right value for d_cur_penalty
    // ---[ Initial estimate for the gamma mixing parameter = 1/2
    double gamma = 0.5;
    double p_outlier_prob = 0;

    indices_size = sac_model_->getIndices ()->size ();
    std::vector<double> p_inlier_prob (indices_size);
    for (int j = 0; j < iterations_EM_; ++j)
    {
      const double weighted_normalization_factor = gamma * normalization_factor;
      // Likelihood of a datum given that it is an inlier
      for (std::size_t i = 0; i < indices_size; ++i)
        p_inlier_prob[i] = weighted_normalization_factor * std::exp ( dist_scaling_factor * distances[i] * distances[i] );

      // Likelihood of a datum given that it is an outlier
      p_outlier_prob = (1 - gamma) / v;

      gamma = 0;
      for (std::size_t i = 0; i < indices_size; ++i)
        gamma += p_inlier_prob [i] / (p_inlier_prob[i] + p_outlier_prob);
      gamma /= static_cast<double>(sac_model_->getIndices ()->size ());
    }

    // Find the std::log likelihood of the model -L = -sum [std::log (pInlierProb + pOutlierProb)]
    double d_cur_penalty = 0;
    for (std::size_t i = 0; i < indices_size; ++i)
      d_cur_penalty += std::log (p_inlier_prob[i] + p_outlier_prob);
    d_cur_penalty = - d_cur_penalty;

    // Better match ?
    if (d_cur_penalty < d_best_penalty)
    {
      d_best_penalty = d_cur_penalty;

      // Save the current model/coefficients selection as being the best so far
      model_              = selection;
      model_coefficients_ = model_coefficients;

      n_inliers_count = 0;
      // Need to compute the number of inliers for this model to adapt k
      for (const double &distance : distances)
        if (distance <= 2 * sigma_)
          n_inliers_count++;

      // Compute the k parameter (k=std::log(z)/std::log(1-w^n))
      double w = static_cast<double> (n_inliers_count) / static_cast<double> (sac_model_->getIndices ()->size ());
      double p_no_outliers = 1 - std::pow (w, static_cast<double> (selection.size ()));
      p_no_outliers = (std::max) (std::numeric_limits<double>::epsilon (), p_no_outliers);       // Avoid division by -Inf
      p_no_outliers = (std::min) (1 - std::numeric_limits<double>::epsilon (), p_no_outliers);   // Avoid division by 0.
      k = std::log (1 - probability_) / std::log (p_no_outliers);
    }

    ++iterations_;
    if (debug_verbosity_level > 1)
      PCL_DEBUG ("[pcl::MaximumLikelihoodSampleConsensus::computeModel] Trial %d out of %d. Best penalty is %f.\n", iterations_, static_cast<int> (std::ceil (k)), d_best_penalty);
    if (iterations_ > max_iterations_)
    {
      if (debug_verbosity_level > 0)
        PCL_DEBUG ("[pcl::MaximumLikelihoodSampleConsensus::computeModel] MLESAC reached the maximum number of trials.\n");
      break;
    }
  }

  if (model_.empty ())
  {
    if (debug_verbosity_level > 0)
      PCL_DEBUG ("[pcl::MaximumLikelihoodSampleConsensus::computeModel] Unable to find a solution!\n");
    return (false);
  }

  // Iterate through the 3d points and calculate the distances from them to the model again
  sac_model_->getDistancesToModel (model_coefficients_, distances);
  Indices &indices = *sac_model_->getIndices ();
  if (distances.size () != indices.size ())
  {
    PCL_ERROR ("[pcl::MaximumLikelihoodSampleConsensus::computeModel] Estimated distances (%lu) differs than the normal of indices (%lu).\n", distances.size (), indices.size ());
    return (false);
  }

  inliers_.resize (distances.size ());
  // Get the inliers for the best model found
  n_inliers_count = 0;
  for (std::size_t i = 0; i < distances.size (); ++i)
    if (distances[i] <= 2 * sigma_)
      inliers_[n_inliers_count++] = indices[i];

  // Resize the inliers vector
  inliers_.resize (n_inliers_count);

  if (debug_verbosity_level > 0)
    PCL_DEBUG ("[pcl::MaximumLikelihoodSampleConsensus::computeModel] Model: %lu size, %d inliers.\n", model_.size (), n_inliers_count);

  return (true);
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> double
pcl::MaximumLikelihoodSampleConsensus<PointT>::computeMedianAbsoluteDeviation (
    const PointCloudConstPtr &cloud, 
    const IndicesPtr &indices,
    double sigma) const
{
  std::vector<double> distances (indices->size ());

  Eigen::Vector4f median;
  // median (dist (x - median (x)))
  computeMedian (cloud, indices, median);

  for (std::size_t i = 0; i < indices->size (); ++i)
  {
    pcl::Vector4fMapConst pt = (*cloud)[(*indices)[i]].getVector4fMap ();
    Eigen::Vector4f ptdiff = pt - median;
    ptdiff[3] = 0;
    distances[i] = ptdiff.dot (ptdiff);
  }

  const double result = pcl::computeMedian (distances.begin (), distances.end (), static_cast<double(*)(double)>(std::sqrt));
  return (sigma * result);
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::MaximumLikelihoodSampleConsensus<PointT>::getMinMax (
    const PointCloudConstPtr &cloud, 
    const IndicesPtr &indices,
    Eigen::Vector4f &min_p, 
    Eigen::Vector4f &max_p) const
{
  min_p.setConstant (FLT_MAX);
  max_p.setConstant (-FLT_MAX);
  min_p[3] = max_p[3] = 0;

  for (std::size_t i = 0; i < indices->size (); ++i)
  {
    if ((*cloud)[(*indices)[i]].x < min_p[0]) min_p[0] = (*cloud)[(*indices)[i]].x;
    if ((*cloud)[(*indices)[i]].y < min_p[1]) min_p[1] = (*cloud)[(*indices)[i]].y;
    if ((*cloud)[(*indices)[i]].z < min_p[2]) min_p[2] = (*cloud)[(*indices)[i]].z;

    if ((*cloud)[(*indices)[i]].x > max_p[0]) max_p[0] = (*cloud)[(*indices)[i]].x;
    if ((*cloud)[(*indices)[i]].y > max_p[1]) max_p[1] = (*cloud)[(*indices)[i]].y;
    if ((*cloud)[(*indices)[i]].z > max_p[2]) max_p[2] = (*cloud)[(*indices)[i]].z;
  }
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::MaximumLikelihoodSampleConsensus<PointT>::computeMedian (
    const PointCloudConstPtr &cloud, 
    const IndicesPtr &indices,
    Eigen::Vector4f &median) const
{
  // Copy the values to vectors for faster sorting
  std::vector<float> x (indices->size ());
  std::vector<float> y (indices->size ());
  std::vector<float> z (indices->size ());
  for (std::size_t i = 0; i < indices->size (); ++i)
  {
    x[i] = (*cloud)[(*indices)[i]].x;
    y[i] = (*cloud)[(*indices)[i]].y;
    z[i] = (*cloud)[(*indices)[i]].z;
  }

  median[0] = pcl::computeMedian (x.begin(), x.end());
  median[1] = pcl::computeMedian (y.begin(), y.end());
  median[2] = pcl::computeMedian (z.begin(), z.end());
  median[3] = 0;
}

#define PCL_INSTANTIATE_MaximumLikelihoodSampleConsensus(T) template class PCL_EXPORTS pcl::MaximumLikelihoodSampleConsensus<T>;

#endif    // PCL_SAMPLE_CONSENSUS_IMPL_MLESAC_H_

