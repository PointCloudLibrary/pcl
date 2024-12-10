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

#ifndef PCL_FILTERS_IMPL_RADIUS_OUTLIER_REMOVAL_H_
#define PCL_FILTERS_IMPL_RADIUS_OUTLIER_REMOVAL_H_

#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/search/organized.h> // for OrganizedNeighbor
#include <pcl/search/kdtree.h> // for KdTree

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::RadiusOutlierRemoval<PointT>::applyFilterIndices (Indices &indices)
{
  if (search_radius_ == 0.0)
  {
    PCL_ERROR ("[pcl::%s::applyFilter] No radius defined!\n", getClassName ().c_str ());
    indices.clear ();
    removed_indices_->clear ();
    return;
  }

  // Initialize the search class
  if (!searcher_)
  {
    if (input_->isOrganized ())
      searcher_.reset (new pcl::search::OrganizedNeighbor<PointT> ());
    else
      searcher_.reset (new pcl::search::KdTree<PointT> (false));
  }
  if (!searcher_->setInputCloud (input_))
  {
    PCL_ERROR ("[pcl::%s::applyFilter] Error when initializing search method!\n", getClassName ().c_str ());
    indices.clear ();
    removed_indices_->clear ();
    return;
  }

  // Note: k includes the query point, so is always at least 1
  const int mean_k = min_pts_radius_ + 1;
  const double nn_dists_max = search_radius_ * search_radius_;

  // The arrays to be used
  Indices nn_indices (mean_k);
  std::vector<float> nn_dists(mean_k);
  // Set to keep all points and in the filtering set those we don't want to keep, assuming
  // we want to keep the majority of the points.
  // 0 = remove, 1 = keep
  std::vector<std::uint8_t> to_keep(indices_->size(), 1);
  indices.resize (indices_->size ());
  removed_indices_->resize (indices_->size ());
  int oii = 0, rii = 0;  // oii = output indices iterator, rii = removed indices iterator

  // If the data is dense => use nearest-k search
  if (input_->is_dense)
  {
    #pragma omp parallel for \
    schedule(dynamic,64) \
    firstprivate(nn_indices, nn_dists)                                                 \
    shared(to_keep) \
    num_threads(num_threads_)
    for (ptrdiff_t i = 0; i < static_cast<ptrdiff_t>(indices_->size()); i++)
    {
      const auto& index = (*indices_)[i];
      // Perform the nearest-k search
      const int k = searcher_->nearestKSearch (index, mean_k, nn_indices, nn_dists);

      // Check the number of neighbors
      // Note: nn_dists is sorted, so check the last item
      if (k == mean_k)
      {
          // if negative_ is false and a neighbor is further away than max distance, remove the point
          // or
          // if negative is true and a neighbor is closer than max distance, remove the point
          if ((!negative_ && nn_dists_max < nn_dists[k-1]) || (negative_ && nn_dists_max >= nn_dists[k - 1]))
          {
            to_keep[i] = 0;
            continue;
          }
      }
      else
      {
        if (!negative_)
        {
          // if too few neighbors, remove the point
          to_keep[i] = 0;
          continue;
        }
      }
    }
  }
  // NaN or Inf values could exist => use radius search
  else
  {
    #pragma omp parallel for \
    schedule(dynamic, 64) \
    firstprivate(nn_indices, nn_dists) \
    shared(to_keep) \
    num_threads(num_threads_)
    for (ptrdiff_t i = 0; i < static_cast<ptrdiff_t>(indices_->size()); i++)
    {
      const auto& index = (*indices_)[i];
      if (!pcl::isXYZFinite((*input_)[index]))
      {
        to_keep[i] = 0;
        continue;
      }
        
      // Perform the radius search
      // Note: k includes the query point, so is always at least 1
      // last parameter (max_nn) is the maximum number of neighbors returned. If enough neighbors are found so that point can not be an outlier, we stop searching.
      const int k = searcher_->radiusSearch (index, search_radius_, nn_indices, nn_dists, min_pts_radius_ + 1);

      // Points having too few neighbors are removed
      // or if negative_ is true, then if it has too many neighbors
      if ((!negative_ && k <= min_pts_radius_) || (negative_ && k > min_pts_radius_))
      {
        to_keep[i] = 0;
        continue;
      }
    }
  }

    for (index_t i=0; i < static_cast<index_t>(to_keep.size()); i++)
    {
      if (to_keep[i] == 0)
      {
        if (extract_removed_indices_)
          (*removed_indices_)[rii++] = (*indices_)[i];
        continue;
      }

      // Otherwise it was a normal point for output (inlier)
      indices[oii++] = (*indices_)[i];
    }

  // Resize the output arrays
  indices.resize (oii);
  removed_indices_->resize (rii);
}

#define PCL_INSTANTIATE_RadiusOutlierRemoval(T) template class PCL_EXPORTS pcl::RadiusOutlierRemoval<T>;

#endif  // PCL_FILTERS_IMPL_RADIUS_OUTLIER_REMOVAL_H_

