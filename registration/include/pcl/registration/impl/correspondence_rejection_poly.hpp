/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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
 */
#ifndef PCL_REGISTRATION_IMPL_CORRESPONDENCE_REJECTION_POLY_HPP_
#define PCL_REGISTRATION_IMPL_CORRESPONDENCE_REJECTION_POLY_HPP_

///////////////////////////////////////////////////////////////////////////////////////////
template <typename SourceT, typename TargetT> void 
pcl::registration::CorrespondenceRejectorPoly<SourceT, TargetT>::getRemainingCorrespondences (
    const pcl::Correspondences& original_correspondences, 
    pcl::Correspondences& remaining_correspondences)
{
  // This is reset after all the checks below
  remaining_correspondences = original_correspondences;
  
  // Check source/target
  if (!input_)
  {
    PCL_ERROR ("[pcl::registration::%s::getRemainingCorrespondences] No source was input! Returning all input correspondences.\n",
               getClassName ().c_str ());
    return;
  }

  if (!target_)
  {
    PCL_ERROR ("[pcl::registration::%s::getRemainingCorrespondences] No target was input! Returning all input correspondences.\n",
               getClassName ().c_str ());
    return;
  }
  
  // Check cardinality
  if (cardinality_ < 2)
  {
    PCL_ERROR ("[pcl::registration::%s::getRemainingCorrespondences] Polygon cardinality too low!. Returning all input correspondences.\n",
               getClassName ().c_str() );
    return;
  }
  
  // Number of input correspondences
  const int nr_correspondences = static_cast<int> (original_correspondences.size ());

  // Not enough correspondences for polygonal rejections
  if (cardinality_ >= nr_correspondences)
  {
    PCL_ERROR ("[pcl::registration::%s::getRemainingCorrespondences] Number of correspondences smaller than polygon cardinality! Returning all input correspondences.\n",
               getClassName ().c_str() );
    return;
  }
  
  // Check similarity
  if (similarity_threshold_ < 0.0f || similarity_threshold_ > 1.0f)
  {
    PCL_ERROR ("[pcl::registration::%s::getRemainingCorrespondences] Invalid edge length similarity - must be in [0,1]!. Returning all input correspondences.\n",
               getClassName ().c_str() );
    return;
  }
  
  // Similarity, squared
  similarity_threshold_squared_ = similarity_threshold_ * similarity_threshold_;

  // Initialization of result
  remaining_correspondences.clear ();
  remaining_correspondences.reserve (nr_correspondences);
  
  // Number of times a correspondence is sampled and number of times it was accepted
  std::vector<int> num_samples (nr_correspondences, 0);
  std::vector<int> num_accepted (nr_correspondences, 0);
  
  // Main loop
  for (int i = 0; i < iterations_; ++i)
  {
    // Sample cardinality_ correspondences without replacement
    const std::vector<int> idx = getUniqueRandomIndices (nr_correspondences, cardinality_);
    
    // Verify the polygon similarity
    if (thresholdPolygon (original_correspondences, idx))
    {
      // Increment sample counter and accept counter
      for (int j = 0; j < cardinality_; ++j)
      {
        ++num_samples[ idx[j] ];
        ++num_accepted[ idx[j] ];
      }
    }
    else
    {
      // Not accepted, only increment sample counter
      for (int j = 0; j < cardinality_; ++j)
        ++num_samples[ idx[j] ];
    }
  }
  
  // Now calculate the acceptance rate of each correspondence
  std::vector<float> accept_rate (nr_correspondences, 0.0f);
  for (int i = 0; i < nr_correspondences; ++i)
  {
    const int numsi = num_samples[i];
    if (numsi == 0)
      accept_rate[i] = 0.0f;
    else
      accept_rate[i] = static_cast<float> (num_accepted[i]) / static_cast<float> (numsi);
  }
  
  // Compute a histogram in range [0,1] for acceptance rates
  const int hist_size = nr_correspondences / 2; // TODO: Optimize this
  const std::vector<int> histogram = computeHistogram (accept_rate, 0.0f, 1.0f, hist_size);
  
  // Find the cut point between outliers and inliers using Otsu's thresholding method
  const int cut_idx = findThresholdOtsu (histogram);
  const float cut = static_cast<float> (cut_idx) / static_cast<float> (hist_size);
  
  // Threshold
  for (int i = 0; i < nr_correspondences; ++i)
    if (accept_rate[i] > cut)
      remaining_correspondences.push_back (original_correspondences[i]);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename SourceT, typename TargetT> std::vector<int> 
pcl::registration::CorrespondenceRejectorPoly<SourceT, TargetT>::computeHistogram (const std::vector<float>& data,
                                                                         float lower, float upper, int bins)
{
  // Result
  std::vector<int> result (bins, 0);
  
  // Last index into result and increment factor from data value --> index
  const int last_idx = bins - 1;
  const float idx_per_val = static_cast<float> (bins) / (upper - lower);
  
  // Accumulate
  for (std::vector<float>::const_iterator it = data.begin (); it != data.end (); ++it)
     ++result[ std::min (last_idx, int ((*it)*idx_per_val)) ];
  
  return (result);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename SourceT, typename TargetT> int 
pcl::registration::CorrespondenceRejectorPoly<SourceT, TargetT>::findThresholdOtsu (const std::vector<int>& histogram)
{
  // Precision
  const double eps = std::numeric_limits<double>::epsilon();
  
  // Histogram dimension
  const int nbins = static_cast<int> (histogram.size ());
  
  // Mean and inverse of the number of data points
  double mean = 0.0;
  double sum_inv = 0.0;
  for (int i = 0; i < nbins; ++i)
  {
    mean += static_cast<double> (i * histogram[i]);
    sum_inv += static_cast<double> (histogram[i]);
  }
  sum_inv = 1.0/sum_inv;
  mean *= sum_inv;
  
  // Probability and mean of class 1 (data to the left of threshold)
  double class_mean1 = 0.0;
  double class_prob1 = 0.0;
  double class_prob2 = 1.0;
  
  // Maximized between class variance and associated bin value
  double between_class_variance_max = 0.0;
  int result = 0;
  
  // Loop over all bin values
  for (int i = 0; i < nbins; ++i)
  {
    class_mean1 *= class_prob1;
    
    // Probability of bin i
    const double prob_i = static_cast<double> (histogram[i]) * sum_inv;
    
    // Class probability 1: sum of probabilities from 0 to i
    class_prob1 += prob_i;
    
    // Class probability 2: sum of probabilities from i+1 to nbins-1
    class_prob2 -= prob_i;
    
    // Avoid division by zero below
    if (std::min (class_prob1,class_prob2) < eps || std::max (class_prob1,class_prob2) > 1.0-eps)
      continue;
    
    // Class mean 1: sum of probabilities from 0 to i, weighted by bin value
    class_mean1 = (class_mean1 + static_cast<double> (i) * prob_i) / class_prob1;
    
    // Class mean 2: sum of probabilities from i+1 to nbins-1, weighted by bin value
    const double class_mean2 = (mean - class_prob1*class_mean1) / class_prob2;
    
    // Between class variance
    const double between_class_variance = class_prob1 * class_prob2
                                          * (class_mean1 - class_mean2)
                                          * (class_mean1 - class_mean2);
    
    // If between class variance is maximized, update result
    if (between_class_variance > between_class_variance_max)
    {
      between_class_variance_max = between_class_variance;
      result = i;
    }
  }
  
  return (result);
}

#endif    // PCL_REGISTRATION_IMPL_CORRESPONDENCE_REJECTION_POLY_HPP_
