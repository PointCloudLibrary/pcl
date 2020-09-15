/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
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

#include <pcl/filters/impl/radius_outlier_removal.hpp>
#include <pcl/conversions.h>
#include <pcl/memory.h>

///////////////////////////////////////////////////////////////////////////////////////////
void
pcl::RadiusOutlierRemoval<pcl::PCLPointCloud2>::applyFilter (PCLPointCloud2 &output)
{
  // If fields x/y/z are not present, we cannot filter
  if (x_idx_ == UNAVAILABLE|| y_idx_ == UNAVAILABLE || z_idx_ == UNAVAILABLE)
  {
    PCL_ERROR ("[pcl::%s::applyFilter] Input dataset doesn't have x-y-z coordinates!\n", getClassName ().c_str ());
    output.width = output.height = 0;
    output.data.clear ();
    return;
  }

  if (search_radius_ == 0.0)
  {
    PCL_ERROR ("[pcl::%s::applyFilter] No radius defined!\n", getClassName ().c_str ());
    output.width = output.height = 0;
    output.data.clear ();
    return;
  }
  // Send the input dataset to the spatial locator
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2 (*input_, *cloud);

  // Initialize the spatial locator
  if (!searcher_)
  {
    if (cloud->isOrganized ())
    {
      PCL_DEBUG ("[pcl::%s::applyFilter] Cloud is organized, so using OrganizedNeighbor.\n", getClassName ().c_str ());
      searcher_.reset (new pcl::search::OrganizedNeighbor<pcl::PointXYZ> ());
    }
    else
    {
      PCL_DEBUG ("[pcl::%s::applyFilter] Cloud is not organized, so using KdTree.\n", getClassName ().c_str ());
      searcher_.reset (new pcl::search::KdTree<pcl::PointXYZ> (false));
    }
  }
  searcher_->setInputCloud (cloud);

  // Allocate enough space to hold the results
  Indices nn_indices (indices_->size ());
  std::vector<float> nn_dists (indices_->size ());

  // Copy the common fields
  output.is_dense = input_->is_dense;
  output.is_bigendian = input_->is_bigendian;
  output.point_step = input_->point_step;
  if (keep_organized_)
  {
    output.height = input_->height;
    output.data.resize (input_->data.size()); // this is the final size
  }
  else
  {
    output.height = 1;
    output.data.resize (input_->data.size()); // reserve enough space
  }

  removed_indices_->resize (input_->data.size ());

  int nr_p = 0;
  int nr_removed_p = 0;
  //size_t log_step = indices_->size () / 10;
  // Go over all the points and check which doesn't have enough neighbors
  for (int cp = 0; cp < static_cast<int> (indices_->size ()); ++cp)
  {
    //if(cp%log_step == 0)
    //  PCL_DEBUG ("[pcl::%s::applyFilter] Iteration %i of %lu\n", getClassName ().c_str (), cp, indices_->size());
    int k = searcher_->radiusSearch ((*indices_)[cp], search_radius_, nn_indices, nn_dists);
    // Check if the number of neighbors is larger than the user imposed limit
    if (k < min_pts_radius_)
    {
      if (extract_removed_indices_)
      {
        (*removed_indices_)[nr_removed_p] = cp;
        nr_removed_p++;
      }
      if (keep_organized_)
      {
          /* Set the current point to the user filter value. */
          *(reinterpret_cast<float*>(&output.data[nr_p * output.point_step])+0) = user_filter_value_;
          *(reinterpret_cast<float*>(&output.data[nr_p * output.point_step])+1) = user_filter_value_;
          *(reinterpret_cast<float*>(&output.data[nr_p * output.point_step])+2) = user_filter_value_;
          nr_p++;
          output.is_dense = false;
      }
    }
    else
    {
      memcpy (&output.data[nr_p * output.point_step], &input_->data[(*indices_)[cp] * output.point_step],
              output.point_step);
      nr_p++;
    }
  }

  if (keep_organized_)
  {
    output.width = input_->width;
    // no need to resize output.data since the final size is known and set from the beginning
  }
  else
  {
    output.width = nr_p;
    output.data.resize (output.width * output.point_step);
  }
  output.row_step = output.point_step * output.width;

  removed_indices_->resize (nr_removed_p);
}

void
pcl::RadiusOutlierRemoval<pcl::PCLPointCloud2>::applyFilter (Indices &indices)
{
  if (search_radius_ == 0.0)
  {
    PCL_ERROR ("[pcl::%s::applyFilter] No radius defined!\n", getClassName ().c_str ());
    indices.clear ();
    removed_indices_->clear ();
    return;
  }

  auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>> ();
  pcl::fromPCLPointCloud2 (*input_, *cloud);

  // Initialize the search class
  if (!searcher_)
  {
    if (cloud->isOrganized ())
      searcher_.reset (new pcl::search::OrganizedNeighbor<pcl::PointXYZ> ());
    else
      searcher_.reset (new pcl::search::KdTree<pcl::PointXYZ> (false));
  }
  searcher_->setInputCloud (cloud);

  // The arrays to be used
  Indices nn_indices (indices_->size ());
  std::vector<float> nn_dists (indices_->size ());
  indices.resize (indices_->size ());
  removed_indices_->resize (indices_->size ());
  int oii = 0, rii = 0;  // oii = output indices iterator, rii = removed indices iterator

  // If the data is dense => use nearest-k search
  if (cloud->is_dense)
  {
    // Note: k includes the query point, so is always at least 1
    int mean_k = min_pts_radius_ + 1;
    double nn_dists_max = search_radius_ * search_radius_;

    for (const auto idx : *indices_)
    {
      // Perform the nearest-k search
      int k = searcher_->nearestKSearch (idx, mean_k, nn_indices, nn_dists);

      // Check the number of neighbors
      // Note: nn_dists is sorted, so check the last item
      bool chk_neighbors = (k == mean_k)? (negative_ == (nn_dists_max < nn_dists[k-1])) : negative_;

      // Points having too few neighbors are outliers and are passed to removed indices
      // Unless negative was set, then it's the opposite condition
      if (!chk_neighbors)
      {
        if (extract_removed_indices_)
          (*removed_indices_)[rii++] = idx;
        continue;
      }

      // Otherwise it was a normal point for output (inlier)
      indices[oii++] = idx;
    }
  }
  // NaN or Inf values could exist => use radius search
  else
  {
    for (const auto idx : *indices_)
    {
      // Perform the radius search
      // Note: k includes the query point, so is always at least 1
      int k = searcher_->radiusSearch (idx, search_radius_, nn_indices, nn_dists);

      // Points having too few neighbors are outliers and are passed to removed indices
      // Unless negative was set, then it's the opposite condition
      if ((!negative_ && k <= min_pts_radius_) || (negative_ && k > min_pts_radius_))
      {
        if (extract_removed_indices_)
          (*removed_indices_)[rii++] = idx;
        continue;
      }

      // Otherwise it was a normal point for output (inlier)
      indices[oii++] = idx;
    }
  }

  // Resize the output arrays
  indices.resize (oii);
  removed_indices_->resize (rii);
}

#ifndef PCL_NO_PRECOMPILE
#include <pcl/impl/instantiate.hpp>
#include <pcl/point_types.h>

// Instantiations of specific point types
PCL_INSTANTIATE(RadiusOutlierRemoval, PCL_XYZ_POINT_TYPES)

#endif    // PCL_NO_PRECOMPILE

