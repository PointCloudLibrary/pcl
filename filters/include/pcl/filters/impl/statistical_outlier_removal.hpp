/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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

#ifndef PCL_FILTERS_IMPL_STATISTICAL_OUTLIER_REMOVAL_H_
#define PCL_FILTERS_IMPL_STATISTICAL_OUTLIER_REMOVAL_H_

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/search/organized.h> // for OrganizedNeighbor
#include <pcl/search/kdtree.h> // for KdTree

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::StatisticalOutlierRemoval<PointT>::applyFilterIndices (Indices &indices)
{
  // Initialize the search class
  if (!searcher_)
  {
    if (input_->isOrganized ())
      searcher_.reset (new pcl::search::OrganizedNeighbor<PointT> ());
    else
      searcher_.reset (new pcl::search::KdTree<PointT> (false));
  }
  searcher_->setInputCloud (input_);

  // The arrays to be used
  const int searcher_k = mean_k_ + 1;  // Find one more, since results include the query point.
  Indices nn_indices (searcher_k);
  std::vector<float> nn_dists (searcher_k);
  std::vector<float> distances (indices_->size ());
  indices.resize (indices_->size ());
  removed_indices_->resize (indices_->size ());
  int oii = 0, rii = 0;  // oii = output indices iterator, rii = removed indices iterator

  // First pass: Compute the mean distances for all points with respect to their k nearest neighbors
  int valid_distances = 0;
  for (int iii = 0; iii < static_cast<int> (indices_->size ()); ++iii)  // iii = input indices iterator
  {
    if (!std::isfinite ((*input_)[(*indices_)[iii]].x) ||
        !std::isfinite ((*input_)[(*indices_)[iii]].y) ||
        !std::isfinite ((*input_)[(*indices_)[iii]].z))
    {
      distances[iii] = 0.0;
      continue;
    }

    // Perform the nearest k search
    if (searcher_->nearestKSearch ((*indices_)[iii], searcher_k, nn_indices, nn_dists) == 0)
    {
      distances[iii] = 0.0;
      PCL_WARN ("[pcl::%s::applyFilter] Searching for the closest %d neighbors failed.\n", getClassName ().c_str (), mean_k_);
      continue;
    }

    // Calculate the mean distance to its neighbors
    double dist_sum = 0.0;
    for (int k = 1; k < searcher_k; ++k)  // k = 0 is the query point
      dist_sum += sqrt (nn_dists[k]);
    distances[iii] = static_cast<float> (dist_sum / mean_k_);
    valid_distances++;
  }

  // Estimate the mean and the standard deviation of the distance vector
  double sum = 0, sq_sum = 0;
  for (const float &distance : distances)
  {
    sum += distance;
    sq_sum += distance * distance;
  }
  double mean = sum / static_cast<double>(valid_distances);
  double variance = (sq_sum - sum * sum / static_cast<double>(valid_distances)) / (static_cast<double>(valid_distances) - 1);
  double stddev = sqrt (variance);
  //getMeanStd (distances, mean, stddev);

  double distance_threshold = mean + std_mul_ * stddev;

  // Second pass: Classify the points on the computed distance threshold
  for (int iii = 0; iii < static_cast<int> (indices_->size ()); ++iii)  // iii = input indices iterator
  {
    // Points having a too high average distance are outliers and are passed to removed indices
    // Unless negative was set, then it's the opposite condition
    if ((!negative_ && distances[iii] > distance_threshold) || (negative_ && distances[iii] <= distance_threshold))
    {
      if (extract_removed_indices_)
        (*removed_indices_)[rii++] = (*indices_)[iii];
      continue;
    }

    // Otherwise it was a normal point for output (inlier)
    indices[oii++] = (*indices_)[iii];
  }

  // Resize the output arrays
  indices.resize (oii);
  removed_indices_->resize (rii);
}

#define PCL_INSTANTIATE_StatisticalOutlierRemoval(T) template class PCL_EXPORTS pcl::StatisticalOutlierRemoval<T>;

#endif  // PCL_FILTERS_IMPL_STATISTICAL_OUTLIER_REMOVAL_H_

