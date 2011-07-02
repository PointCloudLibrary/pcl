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
    dimension_length.push_back (aux);
  }
  D = sqrt (D);
  levels = ceil (Log2 (D));
  PCL_INFO ("PyramidFeatureMatching: pyramid will have %u levels with a hyper-parallelepiped diagonal size of %f\n", levels, D);

  for (size_t level_i = 0; level_i < levels; ++level_i)
  {
    std::vector<size_t> bins_per_dimension;
    for (size_t dim_i = 0; dim_i < dimensions; ++dim_i) {
      bins_per_dimension.push_back (ceil (dimension_length[dim_i] / (pow (2.0f, (int) level_i) * sqrt ((float) dimensions))));
      bin_step.push_back (pow (2.0f, (int) level_i) * sqrt ((float) dimensions));
    }
    hist_levels.push_back (PyramidHistogram (bins_per_dimension));
  }
}

template <typename PointFeature> unsigned int&
pcl::PyramidFeatureMatching<PointFeature>::at (std::vector<float>& feature,
                                               size_t& level)
{
  if (feature.size () != dimensions)
  {
    PCL_ERROR ("PyramidFeatureMatching: the given feature vector does not match the feature dimensions of the pyramid histogram\n");
    return hist_levels.front ().hist.front ();
  }
  if (level >= hist_levels.size ())
  {
    PCL_ERROR ("PyramidFeatureMatching: trying to access a too large level\n");
    return hist_levels.front ().hist.front ();
  }

  std::vector<size_t> access;
  for (size_t dim_i = 0; dim_i < dimensions; ++dim_i)
    access.push_back ( floor ((feature[dim_i] - dimension_range[dim_i].first) / bin_step[dim_i]));

  return hist_levels[level].at (access);
}


template <typename PointFeature> void
pcl::PyramidFeatureMatching<PointFeature>::setInputCloud (const FeatureCloudConstPtr &a_feature_cloud)
{

}


#define PCL_INSTANTIATE_PyramidFeatureMatching(PointFeature) template class PCL_EXPORTS pcl::PyramidFeatureMatching<PointFeature>;

#endif /* PCL_REGISTRATION_IMPL_PYRAMID_FEATURE_MATCHING_H_ */
