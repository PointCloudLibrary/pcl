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
 * $Id$
 *
 */
#ifndef PCL_REGISTRATION_IMPL_CORRESPONDENCE_REJECTION_POLY_HPP_
#define PCL_REGISTRATION_IMPL_CORRESPONDENCE_REJECTION_POLY_HPP_

#include <limits>

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void 
pcl::registration::CorrespondenceRejectorPoly<PointT>::getRemainingCorrespondences (
    const pcl::Correspondences& original_correspondences, 
    pcl::Correspondences& remaining_correspondences)
{
  // This is reset after all the checks below
  remaining_correspondences = original_correspondences;
  
  // Get source/target
  boost::shared_ptr<DataContainer<PointT> > data_container_ptr = boost::static_pointer_cast<DataContainer<PointT> > (data_container_);
  typename pcl::PointCloud<PointT>::ConstPtr source = data_container_ptr->getInputSource();
  typename pcl::PointCloud<PointT>::ConstPtr target = data_container_ptr->getInputTarget();
  
  // Check source/target
  if (!source)
  {
    PCL_ERROR ("[pcl::registration::%s::getRemainingCorrespondences] No source was input! Returning all input correspondences.\n", getClassName ().c_str ());
    return;
  }

  if (!target)
  {
    PCL_ERROR ("[pcl::registration::%s::getRemainingCorrespondences] No target was input! Returning all input correspondences.\n", getClassName ().c_str ());
    return;
  }
  
  // Check cardinality
  if (cardinality_ < 2)
  {
    PCL_ERROR ("[pcl::registration::%s::getRemainingCorrespondences] Polygon cardinality too low!. Returning all input correspondences.\n", getClassName ().c_str() );
    return;
  }
  
  // Number of input correspondences
  const int nr_correspondences = static_cast<int> (original_correspondences.size ());

  // Not enough correspondences for polygonal rejections
  if (cardinality_ >= nr_correspondences)
  {
    PCL_ERROR ("[pcl::registration::%s::getRemainingCorrespondences] Number of correspondences smaller than polygon cardinality! Returning all input correspondences.\n", getClassName ().c_str() );
    return;
  }
  
  // Check similarity
  if (similarity_threshold_ < 0.0f || similarity_threshold_ > 1.0f)
  {
    PCL_ERROR ("[pcl::registration::%s::getRemainingCorrespondences] Invalid edge length similarity - must be in [0,1]!. Returning all input correspondences.\n", getClassName ().c_str() );
    return;
  }
  
  // Similarity, squared
  const float simsq = similarity_threshold_*similarity_threshold_;

  // Initialization of result
  remaining_correspondences.clear ();
  remaining_correspondences.reserve (nr_correspondences);
  
  // Number of times a correspondence is sampled and number of times it was accepted
  std::vector<int> numSamples (nr_correspondences, 0);
  std::vector<int> numAccepted (nr_correspondences, 0);
  
  // Main loop
  for (int i = 0; i < iterations_; ++i)
  {
    // Sample cardinality_ correspondences without replacement
    const std::vector<int> idx = getUniqueRandomIndices (nr_correspondences, cardinality_);
    
    // Verify the polygon similarity
    if (thresholdPolygon(source, target, original_correspondences, idx, simsq))
    {
      // Increment sample counter and accept counter
      for (int j = 0; j < cardinality_; ++j)
      {
        ++numSamples[ idx[j] ];
        ++numAccepted[ idx[j] ];
      }
    } else {
      // Not accepted, only increment sample counter
      for (int j = 0; j < cardinality_; ++j)
        ++numSamples[ idx[j] ];
    }
  }
  
  // Now calculate the acceptance rate of each correspondence
  std::vector<float> acceptRate (nr_correspondences, 0.0f);
  for (int i = 0; i < nr_correspondences; ++i)
  {
    const int numsi = numSamples[i];
    if (numsi == 0)
      acceptRate[i] = 0.0f;
    else
      acceptRate[i] = float (numAccepted[i]) / float (numsi);
  }
  
  // Compute a histogram in range [0,1] for acceptance rates
  const int hist_size = 256;
  const std::vector<int> histogram = computeHistogram (acceptRate, 0.0f, 1.0f, hist_size);
  
  // Find the cut point between outliers and inliers using Otsu's thresholding method
  const int cut_idx = findThresholdOtsu (histogram);
  const float cut = float(cut_idx) / float(hist_size);
  
  // Threshold
  for (int i = 0; i < nr_correspondences; ++i)
    if (acceptRate[i] > cut)
      remaining_correspondences.push_back (original_correspondences[i]);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> std::vector<int> 
pcl::registration::CorrespondenceRejectorPoly<PointT>::computeHistogram (const std::vector<float>& data, float lower, float upper, int bins)
{
  // Result
  std::vector<int> result (bins,0);
  
  // Last index into result and increment factor from data value --> index
  const int last_idx = bins - 1;
  const float idx_per_val = float (bins) / (upper - lower);
  
  // Accumulate
  for (std::vector<float>::const_iterator it = data.begin (); it != data.end (); ++it)
     ++result[ std::min (last_idx, int ((*it)*idx_per_val)) ];
  
  return (result);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> int 
pcl::registration::CorrespondenceRejectorPoly<PointT>::findThresholdOtsu (const std::vector<int>& histogram)
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
    mean += double (i*histogram[i]);
    sum_inv += double (histogram[i]);
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
    class_mean1 = (class_mean1 + static_cast<double> (i) * prob_i)/class_prob1;
    
    // Class mean 2: sum of probabilities from i+1 to nbins-1, weighted by bin value
    const double class_mean2 = (mean - class_prob1*class_mean1)/class_prob2;
    
    // Between class variance
    const double between_class_variance = class_prob1*class_prob2*(class_mean1-class_mean2)*(class_mean1-class_mean2);
    
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
