/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Alexandru-Eugen Ichim
 *                      Willow Garage, Inc
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
 */

#ifndef PCL_REGISTRATION_IMPL_PYRAMID_FEATURE_MATCHING_H_
#define PCL_REGISTRATION_IMPL_PYRAMID_FEATURE_MATCHING_H_

#include <pcl/win32_macros.h>

#include "pcl/registration/pyramid_feature_matching.h"

/** \brief Helper function to calculate the binary logarithm
  * \param n_arg: some value
  * \return binary logarithm (log2) of argument n_arg
  */
__inline double
Log2 (double n_arg)
{
  return log (n_arg) / M_LN2;
}


template <typename PointFeature> void
pcl::PyramidFeatureMatching<PointFeature>::initialize ()
{
  float D = 0.0f;
  for (std::vector<std::pair<float, float> >::iterator range_it = dimension_range.begin (); range_it != dimension_range.end (); ++range_it)
  {
    float aux = range_it->first - range_it->second;
    D += aux * aux;
  }
  D = sqrt (D);
  levels = ceil (Log2 (D));
  PCL_INFO ("PyramidFeatureMatching: pyramid will have %u levels with a hyper-parallelepiped diagonal size of %f\n", levels, D);
}


template <typename PointFeature> void
pcl::PyramidFeatureMatching<PointFeature>::computePyramidHistogram (const FeatureCloudConstPtr &feature_cloud,
                                                                    pcl::PyramidHistogram::Ptr &pyramid)
{
  pyramid = pcl::PyramidHistogram::Ptr (new PyramidHistogram (dimension_range,
                                                              levels,
                                                              feature_cloud->points.size ()));
  for (size_t feature_i = 0; feature_i < feature_cloud->points.size (); ++feature_i)
  {
    std::vector<float> feature_vector;
    convertFeatureToVector (feature_cloud->points[feature_i], feature_vector);
//    PCL_INFO ("adding feature_i %u with values: %f %f %f %f\n", feature_i, feature_vector[0], feature_vector[1], feature_vector[2], feature_vector[3]);
    pyramid->addFeature (feature_vector);
  }
}


template <typename PointFeature> float
pcl::PyramidFeatureMatching<PointFeature>::comparePyramidHistograms (const pcl::PyramidHistogram::Ptr &pyramid_a,
                                                                     const pcl::PyramidHistogram::Ptr &pyramid_b)
{
  // do a few consistency checks before and during the computation
  if (pyramid_a->dimensions != pyramid_b->dimensions)
  {
    PCL_ERROR ("PyramidFeatureMatching: the two given pyramids have different numbers of dimensions: %u vs %u\n", pyramid_a->dimensions, pyramid_b->dimensions);
    return -1;
  }
  if (pyramid_a->nr_levels != pyramid_b->nr_levels)
  {
    PCL_ERROR ("PyramidFeatureMatching: the two given pyramids have different numbers of levels: %u vs %u\n", pyramid_a->nr_levels, pyramid_b->nr_levels);
    return -1;
  }


  // calculate for level 0 first
  if (pyramid_a->hist_levels[0].hist.size () != pyramid_b->hist_levels[0].hist.size ())
  {
    PCL_ERROR ("PyramidFeatureMatching: the two given pyramids have different numbers of bins on level 0: %u vs %u\n", pyramid_a->hist_levels[0].hist.size (), pyramid_b->hist_levels[0].hist.size ());
    return -1;
  }
  float match_count_level = 0.0f, match_count_prev_level = 0.0f;
  for (size_t bin_i = 0; bin_i < pyramid_a->hist_levels[0].hist.size (); ++bin_i)
  {
    if (pyramid_a->hist_levels[0].hist[bin_i] < pyramid_b->hist_levels[0].hist[bin_i])
      match_count_level += pyramid_a->hist_levels[0].hist[bin_i];
    else
      match_count_level += pyramid_b->hist_levels[0].hist[bin_i];
  }


  float match_count = match_count_level;
  for (size_t level_i = 1; level_i < pyramid_a->nr_levels; ++level_i)
  {
    if (pyramid_a->hist_levels[level_i].hist.size () != pyramid_b->hist_levels[level_i].hist.size ())
    {
      PCL_ERROR ("PyramidFeatureMatching: the two given pyramids have different numbers of bins on level %u: %u vs %u\n", level_i, pyramid_a->hist_levels[level_i].hist.size (), pyramid_b->hist_levels[level_i].hist.size ());
      return -1;
    }

    match_count_prev_level = match_count_level;
    match_count_level = 0.0f;
    for (size_t bin_i = 0; bin_i < pyramid_a->hist_levels[level_i].hist.size (); ++bin_i)
    {
      if (pyramid_a->hist_levels[level_i].hist[bin_i] < pyramid_b->hist_levels[level_i].hist[bin_i])
        match_count_level += pyramid_a->hist_levels[level_i].hist[bin_i];
      else
        match_count_level += pyramid_b->hist_levels[level_i].hist[bin_i];
    }

    float level_normalization_factor = pow(2.0f, (int) level_i);
    match_count += (match_count_level - match_count_prev_level) / level_normalization_factor;
  }


  // include self-similarity factors
  float self_similarity_a = pyramid_a->nr_features,
      self_similarity_b = pyramid_b->nr_features;
  PCL_INFO ("Self similarity measures: %f, %f\n", self_similarity_a, self_similarity_b);
  match_count /= sqrt (self_similarity_a * self_similarity_b);

  return match_count;
}


#define PCL_INSTANTIATE_PyramidFeatureMatching(PointFeature) template class PCL_EXPORTS pcl::PyramidFeatureMatching<PointFeature>;

#endif /* PCL_REGISTRATION_IMPL_PYRAMID_FEATURE_MATCHING_H_ */
