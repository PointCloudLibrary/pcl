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

#ifndef PCL_FILTERS_IMPL_RADIUS_OUTLIER_REMOVAL_H_
#define PCL_FILTERS_IMPL_RADIUS_OUTLIER_REMOVAL_H_

#include <pcl/filters/radius_outlier_removal.h>

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::RadiusOutlierRemoval<PointT>::applyFilter (PointCloud &output)
{
  if (search_radius_ == 0.0)
  {
    PCL_ERROR ("[pcl::%s::applyFilter] No radius defined!\n", getClassName ().c_str ());
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
  std::vector<int> nn_indices (indices_->size ());
  std::vector<float> nn_dists (indices_->size ());


  output.points.resize (input_->points.size ());      // reserve enough space
  removed_indices_->resize (input_->points.size ());
  
  int nr_p = 0;
  int nr_removed_p = 0;
  
  // Go over all the points and check which doesn't have enough neighbors
  for (int cp = 0; cp < static_cast<int>(indices_->size ()); ++cp)
  {
    int k = tree_->radiusSearch ((*indices_)[cp], search_radius_, nn_indices, nn_dists);
    // Check if the number of neighbors is larger than the user imposed limit
    if (k < min_pts_radius_)
    {
      if (extract_removed_indices_)
      {
        (*removed_indices_)[nr_removed_p] = cp;
        nr_removed_p++;
      }
      continue;
    }

    output.points[nr_p++] = input_->points[(*indices_)[cp]];
  }
  removed_indices_->resize (nr_removed_p);
  output.points.resize (nr_p);
  output.width  = nr_p;
  output.height = 1;
  output.is_dense = true; // radiusSearch filters invalid points
}

#define PCL_INSTANTIATE_RadiusOutlierRemoval(T) template class PCL_EXPORTS pcl::RadiusOutlierRemoval<T>;

#endif    // PCL_FILTERS_IMPL_RADIUS_OUTLIER_REMOVAL_H_

