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

#include <pcl/point_types.h>
#include <pcl/impl/instantiate.hpp>
#include "pcl/registration/pyramid_feature_matching.h"
#include "pcl/registration/impl/pyramid_feature_matching.hpp"


void
pcl::PyramidHistogram::PyramidHistogramLevel::initializeHistogramLevel ()
{
  size_t total_vector_size = 1;
  for (std::vector<size_t>::iterator dim_it = bins_per_dimension.begin (); dim_it != bins_per_dimension.end (); ++dim_it)
    total_vector_size *= *dim_it;

  hist.resize (total_vector_size, 0);
}

void
pcl::PyramidHistogram::initializeHistogram ()
{
  for (size_t level_i = 0; level_i < nr_levels; ++level_i)
  {
    std::vector<size_t> bins_per_dimension;
    std::vector<float> bin_step;
    for (size_t dim_i = 0; dim_i < dimensions; ++dim_i) {
      bins_per_dimension.push_back (ceil ( (dimension_range[dim_i].second - dimension_range[dim_i].first) / (pow (2.0f, (int) level_i) * sqrt ((float) dimensions))));
      bin_step.push_back (pow (2.0f, (int) level_i) * sqrt ((float) dimensions));
    }
    hist_levels.push_back (PyramidHistogramLevel (bins_per_dimension, bin_step));

    PCL_INFO ("PyramidHistogram: created vector of size %u at level %u\nwith #bins per dimension:", hist_levels.back ().hist.size (), level_i);
    for (size_t dim_i = 0; dim_i < dimensions; ++dim_i)
      PCL_INFO ("%u ", bins_per_dimension[dim_i]);
    PCL_INFO ("\n");
  }
}

unsigned int&
pcl::PyramidHistogram::at (std::vector<size_t> &access,
                           size_t &level)
{
  if (access.size () != dimensions)
  {
    PCL_ERROR ("PyramidHistogram: cannot access histogram position because the access point does not have the right number of dimensions\n");
    return hist_levels.front ().hist.front ();
  }
  if (level >= hist_levels.size ())
  {
    PCL_ERROR ("PyramidFeatureMatching: trying to access a too large level\n");
    return hist_levels.front ().hist.front ();
  }

  size_t vector_position = 0;
  size_t dim_accumulator = 1;

  for (int i = access.size ()-1; i >= 0; --i)
  {
    vector_position += access[i] * dim_accumulator;
    dim_accumulator *= hist_levels[level].bins_per_dimension[i];
  }

  return hist_levels[level].hist[vector_position];
}

unsigned int&
pcl::PyramidHistogram::at (std::vector<float> &feature,
                           size_t &level)
{
  if (feature.size () != dimensions)
  {
    PCL_ERROR ("PyramidFeatureMatching: the given feature vector does not match the feature dimensions of the pyramid histogram: %u vs %u\n", feature.size (), dimensions);
    return hist_levels.front ().hist.front ();
  }
  if (level >= hist_levels.size ())
  {
    PCL_ERROR ("PyramidFeatureMatching: trying to access a too large level\n");
    return hist_levels.front ().hist.front ();
  }

  std::vector<size_t> access;
  for (size_t dim_i = 0; dim_i < dimensions; ++dim_i)
    access.push_back ( floor ((feature[dim_i] - dimension_range[dim_i].first) / hist_levels[level].bin_step[dim_i]));

  return at (access, level);
}

void
pcl::PyramidHistogram::addFeature (std::vector<float> &feature)
{
  for (size_t level_i = 0; level_i < nr_levels; ++level_i)
    at (feature, level_i) ++;
}

PCL_INSTANTIATE_PRODUCT(PyramidFeatureMatching, (PCL_POINT_TYPES));
