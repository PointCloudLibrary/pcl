/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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
 *
 */

#ifndef PCL_FILTERS_IMPL_STATISTICAL_OUTLIER_REMOVAL_H_
#define PCL_FILTERS_IMPL_STATISTICAL_OUTLIER_REMOVAL_H_

#include "pcl/filters/statistical_outlier_removal.h"


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::StatisticalOutlierRemoval<PointT>::applyFilter (PointCloud &output)
{
  if (std_mul_ == 0.0)
  {
    PCL_ERROR ("[pcl::%s::applyFilter] Standard deviation multiplier not set!\n", getClassName ().c_str ());
    output.width = output.height = 0;
    output.points.clear ();
    return;
  }

  if (input_->points.empty ())
  {
    output.width = output.height = 0;
    output.points.clear ();
    return;
  }

  // Initialize the spatial locator
  if (!tree_)
  {
    if (input_->isOrganized ())
      tree_.reset (new pcl::search::OrganizedNeighbor<PointT> ());
    else
      tree_.reset (new pcl::search::KdTree<PointT> (false));
  }

  // Send the input dataset to the spatial locator
  tree_->setInputCloud (input_);

  // Allocate enough space to hold the results
  std::vector<int> nn_indices (mean_k_);
  std::vector<float> nn_dists (mean_k_);

  std::vector<float> distances (indices_->size ());
  // Go over all the points and calculate the mean or smallest distance
  for (size_t cp = 0; cp < indices_->size (); ++cp)
  {
    if (!pcl_isfinite (input_->points[(*indices_)[cp]].x) ||
        !pcl_isfinite (input_->points[(*indices_)[cp]].y) ||
        !pcl_isfinite (input_->points[(*indices_)[cp]].z))
    {
      distances[cp] = 0;
      continue;
    }

    if (tree_->nearestKSearch ((*indices_)[cp], mean_k_, nn_indices, nn_dists) == 0)
    {
      distances[cp] = 0;
      PCL_WARN ("[pcl::%s::applyFilter] Searching for the closest %d neighbors failed.\n", getClassName ().c_str (), mean_k_);
      continue;
    }

    // Minimum distance (if mean_k_ == 2) or mean distance
    double dist_sum = 0;
    for (int j = 1; j < mean_k_; ++j)
      dist_sum += sqrt (nn_dists[j]);
    distances[cp] = dist_sum / (mean_k_-1);
  }

  // Estimate the mean and the standard deviation of the distance vector
  double mean, stddev;
  getMeanStd (distances, mean, stddev);
  double distance_threshold = mean + std_mul_ * stddev; // a distance that is bigger than this signals an outlier

  output.points.resize (input_->points.size ());      // reserve enough space
  removed_indices_->resize (input_->points.size ());
  
  // Build a new cloud by neglecting outliers
  int nr_p = 0;
  int nr_removed_p = 0;
  
  for (int cp = 0; cp < static_cast<int> (indices_->size ()); ++cp)
  {
    if (negative_)
    {
      if (distances[cp] <= distance_threshold)
      {
        if (extract_removed_indices_)
        {
          (*removed_indices_)[nr_removed_p] = cp;
          nr_removed_p++;
        }
        continue;
      }
    }
    else
    {
      if (distances[cp] > distance_threshold)
      {
        if (extract_removed_indices_)
        {
          (*removed_indices_)[nr_removed_p] = cp;
          nr_removed_p++;
        }
        continue;
      }
    }

    output.points[nr_p++] = input_->points[(*indices_)[cp]];
  }

  output.points.resize (nr_p);
  output.width  = nr_p;
  output.height = 1;
  output.is_dense = true; // nearestKSearch filters invalid points

  removed_indices_->resize (nr_removed_p);
}

#define PCL_INSTANTIATE_StatisticalOutlierRemoval(T) template class PCL_EXPORTS pcl::StatisticalOutlierRemoval<T>;

#endif    // PCL_FILTERS_IMPL_STATISTICAL_OUTLIER_REMOVAL_H_

