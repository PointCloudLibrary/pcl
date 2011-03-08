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
 * $Id: statistical_outlier_removal.cpp 35873 2011-02-09 00:58:01Z rusu $
 *
 */

#include "pcl/impl/instantiate.hpp"
#include "pcl/point_types.h"
#include "pcl/filters/statistical_outlier_removal.h"
#include "pcl/filters/impl/statistical_outlier_removal.hpp"
#include "pcl/ros/conversions.h"

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::StatisticalOutlierRemoval<sensor_msgs::PointCloud2>::applyFilter (PointCloud2 &output)
{
  output.is_dense = true;
  // If fields x/y/z are not present, we cannot filter
  if (x_idx_ == -1 || y_idx_ == -1 || z_idx_ == -1)
  {
    ROS_ERROR ("[pcl::%s::applyFilter] Input dataset doesn't have x-y-z coordinates!", getClassName ().c_str ());
    output.width = output.height = 0;
    output.data.clear ();
    return;
  }

  if (std_mul_ == 0.0)
  {
    ROS_ERROR ("[pcl::%s::applyFilter] Standard deviation multipler not set!", getClassName ().c_str ());
    output.width = output.height = 0;
    output.data.clear ();
    return;
  }

  // Initialize the spatial locator
  //initTree (spatial_locator_type_, tree_, k_);

  // TODO: fix this
  tree_.reset (new pcl::KdTreeFLANN<pcl::PointXYZ>);

  // Send the input dataset to the spatial locator
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg (*input_, *cloud);
  tree_->setInputCloud (cloud);

  // Allocate enough space to hold the results
  std::vector<int> nn_indices (mean_k_);
  std::vector<float> nn_dists (mean_k_);

  std::vector<float> distances (indices_->size ());
  // Go over all the points and calculate the mean or smallest distance
  for (size_t cp = 0; cp < indices_->size (); ++cp)
  {
    if (!pcl_isfinite (cloud->points[(*indices_)[cp]].x) ||
        !pcl_isfinite (cloud->points[(*indices_)[cp]].y) ||
        !pcl_isfinite (cloud->points[(*indices_)[cp]].z))
    {
      distances[cp] = 0;
      continue;
    }

    if (tree_->nearestKSearch ((*indices_)[cp], mean_k_, nn_indices, nn_dists) == 0)
    {
      distances[cp] = 0;
      ROS_WARN ("[pcl::%s::applyFilter] Searching for the closest %d neighbors failed.", getClassName ().c_str (), mean_k_);
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

  // Copy the common fields
  output.is_bigendian = input_->is_bigendian;
  output.point_step   = input_->point_step;
  output.height       = 1;

  output.data.resize (indices_->size () * input_->point_step);      // reserve enough space

  // Build a new cloud by neglecting outliers
  int nr_p = 0;
  for (size_t cp = 0; cp < indices_->size (); ++cp)
  {
    if (negative_)
    {
      if (distances[cp] <= distance_threshold)
        continue;
    }
    else
    {
      if (distances[cp] > distance_threshold)
        continue;
    }

    memcpy (&output.data[nr_p * output.point_step], &input_->data[(*indices_)[cp] * output.point_step], output.point_step);
    nr_p++;
  }
  output.width = nr_p;
  output.data.resize (output.width * output.point_step);
  output.row_step = output.point_step * output.width;
}

// Instantiations of specific point types
PCL_INSTANTIATE(StatisticalOutlierRemoval, PCL_XYZ_POINT_TYPES);

