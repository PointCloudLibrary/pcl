/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009-2012, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
 *  Copyright (c) 2014, RadiantBlue Technologies, Inc.
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
 */

#ifndef PCL_SEGMENTATION_PROGRESSIVE_MORPHOLOGICAL_FILTER_HPP_
#define PCL_SEGMENTATION_PROGRESSIVE_MORPHOLOGICAL_FILTER_HPP_

#include <pcl/common/common.h>
#include <pcl/common/io.h>
#include <pcl/filters/morphological_filter.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
pcl::ProgressiveMorphologicalFilter<PointT>::ProgressiveMorphologicalFilter () :
  max_window_size_ (33),
  slope_ (0.7f),
  max_distance_ (10.0f),
  initial_distance_ (0.15f),
  cell_size_ (1.0f),
  base_ (2.0f),
  exponential_ (true)
{
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
pcl::ProgressiveMorphologicalFilter<PointT>::~ProgressiveMorphologicalFilter () = default;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::ProgressiveMorphologicalFilter<PointT>::extract (Indices& ground)
{
  bool segmentation_is_possible = initCompute ();
  if (!segmentation_is_possible)
  {
    deinitCompute ();
    return;
  }

  // Compute the series of window sizes and height thresholds
  std::vector<float> height_thresholds;
  std::vector<float> window_sizes;
  int iteration = 0;
  float window_size = 0.0f;
  float height_threshold = 0.0f;

  while (window_size < max_window_size_)
  {
    // Determine the initial window size.
    if (exponential_)
      window_size = cell_size_ * (2.0f * std::pow (base_, iteration) + 1.0f);
    else
      window_size = cell_size_ * (2.0f * (iteration+1) * base_ + 1.0f);

    // Calculate the height threshold to be used in the next iteration.
    if (iteration == 0)
      height_threshold = initial_distance_;
    else
      height_threshold = slope_ * (window_size - window_sizes[iteration-1]) * cell_size_ + initial_distance_;

    // Enforce max distance on height threshold
    if (height_threshold > max_distance_)
      height_threshold = max_distance_;

    window_sizes.push_back (window_size);
    height_thresholds.push_back (height_threshold);

    iteration++;
  }

  // Ground indices are initially limited to those points in the input cloud we
  // wish to process
  ground = *indices_;

  // Progressively filter ground returns using morphological open
  for (std::size_t i = 0; i < window_sizes.size (); ++i)
  {
    PCL_DEBUG ("      Iteration %d (height threshold = %f, window size = %f)...",
               i, height_thresholds[i], window_sizes[i]);

    // Limit filtering to those points currently considered ground returns
    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
    pcl::copyPointCloud<PointT> (*input_, ground, *cloud);

    // Create new cloud to hold the filtered results. Apply the morphological
    // opening operation at the current window size.
    typename pcl::PointCloud<PointT>::Ptr cloud_f (new pcl::PointCloud<PointT>);
    pcl::applyMorphologicalOperator<PointT> (cloud, window_sizes[i], MORPH_OPEN, *cloud_f);

    // Find indices of the points whose difference between the source and
    // filtered point clouds is less than the current height threshold.
    Indices pt_indices;
    for (std::size_t p_idx = 0; p_idx < ground.size (); ++p_idx)
    {
      float diff = (*cloud)[p_idx].z - (*cloud_f)[p_idx].z;
      if (diff < height_thresholds[i])
        pt_indices.push_back (ground[p_idx]);
    }

    // Ground is now limited to pt_indices
    ground.swap (pt_indices);

    PCL_DEBUG ("ground now has %d points\n", ground.size ());
  }

  deinitCompute ();
}

#define PCL_INSTANTIATE_ProgressiveMorphologicalFilter(T) template class pcl::ProgressiveMorphologicalFilter<T>;

#endif    // PCL_SEGMENTATION_PROGRESSIVE_MORPHOLOGICAL_FILTER_HPP_

