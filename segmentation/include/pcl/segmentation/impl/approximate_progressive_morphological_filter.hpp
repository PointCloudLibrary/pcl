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

#ifndef PCL_SEGMENTATION_APPROXIMATE_PROGRESSIVE_MORPHOLOGICAL_FILTER_HPP_
#define PCL_SEGMENTATION_APPROXIMATE_PROGRESSIVE_MORPHOLOGICAL_FILTER_HPP_

#include <pcl/common/common.h>
#include <pcl/common/io.h>
#include <pcl/filters/morphological_filter.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/approximate_progressive_morphological_filter.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
pcl::ApproximateProgressiveMorphologicalFilter<PointT>::ApproximateProgressiveMorphologicalFilter () :
  max_window_size_ (33),
  slope_ (0.7f),
  max_distance_ (10.0f),
  initial_distance_ (0.15f),
  cell_size_ (1.0f),
  base_ (2.0f),
  exponential_ (true),
  threads_ (0)
{
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
pcl::ApproximateProgressiveMorphologicalFilter<PointT>::~ApproximateProgressiveMorphologicalFilter ()
{
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::ApproximateProgressiveMorphologicalFilter<PointT>::extract (std::vector<int>& ground)
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
  std::vector<int> half_sizes;
  int iteration = 0;
  int half_size = 0.0f;
  float window_size = 0.0f;
  float height_threshold = 0.0f;

  while (window_size < max_window_size_)
  {
    // Determine the initial window size.
    if (exponential_)
      half_size = static_cast<int> (std::pow (static_cast<float> (base_), iteration));
    else
      half_size = (iteration+1) * base_;

    window_size = 2 * half_size + 1;

    // Calculate the height threshold to be used in the next iteration.
    if (iteration == 0)
      height_threshold = initial_distance_;
    else
      height_threshold = slope_ * (window_size - window_sizes[iteration-1]) * cell_size_ + initial_distance_;

    // Enforce max distance on height threshold
    if (height_threshold > max_distance_)
      height_threshold = max_distance_;

    half_sizes.push_back (half_size);
    window_sizes.push_back (window_size);
    height_thresholds.push_back (height_threshold);

    iteration++;
  }

  // setup grid based on scale and extents
  Eigen::Vector4f global_max, global_min;
  pcl::getMinMax3D<PointT> (*input_, global_min, global_max);

  float xextent = global_max.x () - global_min.x ();
  float yextent = global_max.y () - global_min.y ();

  int rows = static_cast<int> (std::floor (yextent / cell_size_) + 1);
  int cols = static_cast<int> (std::floor (xextent / cell_size_) + 1);

  Eigen::MatrixXf A (rows, cols);
  A.setConstant (std::numeric_limits<float>::quiet_NaN ());

  Eigen::MatrixXf Z (rows, cols);
  Z.setConstant (std::numeric_limits<float>::quiet_NaN ());

  Eigen::MatrixXf Zf (rows, cols);
  Zf.setConstant (std::numeric_limits<float>::quiet_NaN ());

#ifdef _OPENMP
#pragma omp parallel for num_threads(threads_)
#endif
  for (int i = 0; i < (int)input_->points.size (); ++i)
  {
    // ...then test for lower points within the cell
    PointT p = input_->points[i];
    int row = std::floor(p.y - global_min.y ());
    int col = std::floor(p.x - global_min.x ());

    if (p.z < A (row, col) || pcl_isnan (A (row, col)))
    {
      A (row, col) = p.z;
    }
  }

  // Ground indices are initially limited to those points in the input cloud we
  // wish to process
  ground = *indices_;

  // Progressively filter ground returns using morphological open
  for (size_t i = 0; i < window_sizes.size (); ++i)
  {
    PCL_DEBUG ("      Iteration %d (height threshold = %f, window size = %f, half size = %d)...",
               i, height_thresholds[i], window_sizes[i], half_sizes[i]);

    // Limit filtering to those points currently considered ground returns
    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
    pcl::copyPointCloud<PointT> (*input_, ground, *cloud);

    // Apply the morphological opening operation at the current window size.
#ifdef _OPENMP
#pragma omp parallel for num_threads(threads_)
#endif
    for (int row = 0; row < rows; ++row)
    {
      int rs, re;
      rs = ((row - half_sizes[i]) < 0) ? 0 : row - half_sizes[i];
      re = ((row + half_sizes[i]) > (rows-1)) ? (rows-1) : row + half_sizes[i];

      for (int col = 0; col < cols; ++col)
      {
        int cs, ce;
        cs = ((col - half_sizes[i]) < 0) ? 0 : col - half_sizes[i];
        ce = ((col + half_sizes[i]) > (cols-1)) ? (cols-1) : col + half_sizes[i];

        float min_coeff = std::numeric_limits<float>::max ();

        for (int j = rs; j < (re + 1); ++j)
        {
          for (int k = cs; k < (ce + 1); ++k)
          {
            if (A (j, k) != std::numeric_limits<float>::quiet_NaN ())
            {
              if (A (j, k) < min_coeff)
                min_coeff = A (j, k);
            }
          }
        }

        if (min_coeff != std::numeric_limits<float>::max ())
          Z(row, col) = min_coeff;
      }
    }

#ifdef _OPENMP
#pragma omp parallel for num_threads(threads_)
#endif
    for (int row = 0; row < rows; ++row)
    {
      int rs, re;
      rs = ((row - half_sizes[i]) < 0) ? 0 : row - half_sizes[i];
      re = ((row + half_sizes[i]) > (rows-1)) ? (rows-1) : row + half_sizes[i];

      for (int col = 0; col < cols; ++col)
      {
        int cs, ce;
        cs = ((col - half_sizes[i]) < 0) ? 0 : col - half_sizes[i];
        ce = ((col + half_sizes[i]) > (cols-1)) ? (cols-1) : col + half_sizes[i];

        float max_coeff = -std::numeric_limits<float>::max ();

        for (int j = rs; j < (re + 1); ++j)
        {
          for (int k = cs; k < (ce + 1); ++k)
          {
            if (Z (j, k) != std::numeric_limits<float>::quiet_NaN ())
            {
              if (Z (j, k) > max_coeff)
                max_coeff = Z (j, k);
            }
          }
        }

        if (max_coeff != -std::numeric_limits<float>::max ())
          Zf (row, col) = max_coeff;
      }
    }

    // Find indices of the points whose difference between the source and
    // filtered point clouds is less than the current height threshold.
    std::vector<int> pt_indices;
    for (size_t p_idx = 0; p_idx < ground.size (); ++p_idx)
    {
      PointT p = cloud->points[p_idx];
      int erow = static_cast<int> (std::floor ((p.y - global_min.y ()) / cell_size_));
      int ecol = static_cast<int> (std::floor ((p.x - global_min.x ()) / cell_size_));

      float diff = p.z - Zf (erow, ecol);
      if (diff < height_thresholds[i])
        pt_indices.push_back (ground[p_idx]);
    }

    A.swap (Zf);

    // Ground is now limited to pt_indices
    ground.swap (pt_indices);

    PCL_DEBUG ("ground now has %d points\n", ground.size ());
  }

  deinitCompute ();
}


#define PCL_INSTANTIATE_ApproximateProgressiveMorphologicalFilter(T) template class pcl::ApproximateProgressiveMorphologicalFilter<T>;

#endif    // PCL_SEGMENTATION_APPROXIMATE_PROGRESSIVE_MORPHOLOGICAL_FILTER_HPP_

