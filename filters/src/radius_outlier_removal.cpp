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

///////////////////////////////////////////////////////////////////////////////////////////
void
pcl::RadiusOutlierRemoval<pcl::PCLPointCloud2>::applyFilter (PCLPointCloud2 &output)
{
  output.is_dense = true;
  // If fields x/y/z are not present, we cannot filter
  if (x_idx_ == -1 || y_idx_ == -1 || z_idx_ == -1)
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
  if (!tree_)
  {
    if (cloud->isOrganized ())
      tree_.reset (new pcl::search::OrganizedNeighbor<pcl::PointXYZ> ());
    else
      tree_.reset (new pcl::search::KdTree<pcl::PointXYZ> (false));
  }
  tree_->setInputCloud (cloud);

  // Allocate enough space to hold the results
  std::vector<int> nn_indices (indices_->size ());
  std::vector<float> nn_dists (indices_->size ());

  // Copy the common fields
  output.is_bigendian = input_->is_bigendian;
  output.point_step = input_->point_step;
  output.height = 1;

  output.data.resize (input_->width * input_->point_step); // reserve enough space
  removed_indices_->resize (input_->data.size ());

  int nr_p = 0;
  int nr_removed_p = 0;
  // Go over all the points and check which doesn't have enough neighbors
  for (int cp = 0; cp < static_cast<int> (indices_->size ()); ++cp)
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

    memcpy (&output.data[nr_p * output.point_step], &input_->data[(*indices_)[cp] * output.point_step],
            output.point_step);
    nr_p++;
  }

  output.width = nr_p;
  output.height = 1;
  output.data.resize (output.width * output.point_step);
  output.row_step = output.point_step * output.width;

  removed_indices_->resize (nr_removed_p);
}

#ifndef PCL_NO_PRECOMPILE
#include <pcl/impl/instantiate.hpp>
#include <pcl/point_types.h>

// Instantiations of specific point types
PCL_INSTANTIATE(RadiusOutlierRemoval, PCL_XYZ_POINT_TYPES)

#endif    // PCL_NO_PRECOMPILE

