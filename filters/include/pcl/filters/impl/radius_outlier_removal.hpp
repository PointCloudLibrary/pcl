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
#include <pcl/common/io.h>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::RadiusOutlierRemoval<PointT>::applyFilter (PointCloud &output)
{
  std::vector<int> indices;
  if (keep_organized_)
  {
    bool temp = extract_removed_indices_;
    extract_removed_indices_ = true;
    applyFilterIndices (indices);
    extract_removed_indices_ = temp;

    output = *input_;
    for (int rii = 0; rii < static_cast<int> (removed_indices_->size ()); ++rii)  // rii = removed indices iterator
      output.points[(*removed_indices_)[rii]].x = output.points[(*removed_indices_)[rii]].y = output.points[(*removed_indices_)[rii]].z = user_filter_value_;
    if (!pcl_isfinite (user_filter_value_))
      output.is_dense = false;
  }
  else
  {
    applyFilterIndices (indices);
    copyPointCloud (*input_, indices, output);
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::RadiusOutlierRemoval<PointT>::applyFilterIndices (std::vector<int> &indices)
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
  searcher_->setInputCloud (input_);

  // The arrays to be used
  std::vector<int> nn_indices (indices_->size ());
  std::vector<float> nn_dists (indices_->size ());
  indices.resize (indices_->size ());
  removed_indices_->resize (indices_->size ());
  int oii = 0, rii = 0;  // oii = output indices iterator, rii = removed indices iterator

  for (std::vector<int>::const_iterator it = indices_->begin (); it != indices_->end (); ++it)
  {
    // Perform the radius search
    // Note: k includes the query point, so is always at least 1
    int k = searcher_->radiusSearch (*it, search_radius_, nn_indices, nn_dists);

    // Points having too few neighbors are outliers and are passed to removed indices
    // Unless negative was set, then it's the opposite condition
    if ((!negative_ && k <= min_pts_radius_) || (negative_ && k > min_pts_radius_))
    {
      if (extract_removed_indices_)
        (*removed_indices_)[rii++] = *it;
      continue;
    }

    // Otherwise it was a normal point for output (inlier)
    indices[oii++] = *it;
  }

  // Resize the output arrays
  indices.resize (oii);
  removed_indices_->resize (rii);
}

#define PCL_INSTANTIATE_RadiusOutlierRemoval(T) template class PCL_EXPORTS pcl::RadiusOutlierRemoval<T>;

#endif  // PCL_FILTERS_IMPL_RADIUS_OUTLIER_REMOVAL_H_

