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

#include <pcl/filters/impl/statistical_outlier_removal.hpp>
#include <pcl/conversions.h>

///////////////////////////////////////////////////////////////////////////////////////////
void
pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2>::applyFilter (PCLPointCloud2 &output)
{
  // If fields x/y/z are not present, we cannot filter
  if (x_idx_ == UNAVAILABLE || y_idx_ == UNAVAILABLE || z_idx_ == UNAVAILABLE)
  {
    PCL_ERROR ("[pcl::%s::applyFilter] Input dataset doesn't have x-y-z coordinates!\n", getClassName ().c_str ());
    output.width = output.height = 0;
    output.data.clear ();
    return;
  }

  if (std_mul_ == 0.0)
  {
    PCL_ERROR ("[pcl::%s::applyFilter] Standard deviation multipler not set!\n", getClassName ().c_str ());
    output.width = output.height = 0;
    output.data.clear ();
    return;
  }

  double mean;
  double variance;
  double stddev;
  std::vector<float> distances;
  generateStatistics (mean, variance, stddev, distances);
  double const distance_threshold = mean + std_mul_ * stddev; // a distance that is bigger than this signals an outlier

  // Copy the common fields
  output.is_dense = input_->is_dense;
  output.is_bigendian = input_->is_bigendian;
  output.point_step = input_->point_step;
  if (keep_organized_)
  {
    output.width = input_->width;
    output.height = input_->height;
    output.data.resize (input_->data.size ());
  }
  else
  {
    output.height = 1;
    output.data.resize (indices_->size () * input_->point_step); // reserve enough space
  }

  removed_indices_->resize (input_->data.size ());

  // Build a new cloud by neglecting outliers
  int nr_p = 0;
  int nr_removed_p = 0;
  bool remove_point = false;
  for (int cp = 0; cp < static_cast<int> (indices_->size ()); ++cp)
  {
    if (negative_)
      remove_point = (distances[cp] <= distance_threshold);
    else
      remove_point = (distances[cp] > distance_threshold);

    if (remove_point)
    {
      if (extract_removed_indices_)
        (*removed_indices_)[nr_removed_p++] = cp;

      if (keep_organized_)
      {
          /* Set the current point to NaN. */
          *(reinterpret_cast<float*>(&output.data[nr_p * output.point_step])+0) = std::numeric_limits<float>::quiet_NaN();
          *(reinterpret_cast<float*>(&output.data[nr_p * output.point_step])+1) = std::numeric_limits<float>::quiet_NaN();
          *(reinterpret_cast<float*>(&output.data[nr_p * output.point_step])+2) = std::numeric_limits<float>::quiet_NaN();
          nr_p++;
          output.is_dense = false;
      }
      else
        continue;
    }
    else
    {
      memcpy (&output.data[nr_p * output.point_step], &input_->data[(*indices_)[cp] * output.point_step],
              output.point_step);
      nr_p++;
    }
  }

  if (!keep_organized_)
  {
    output.width = nr_p;
    output.data.resize (output.width * output.point_step);
  }
  output.row_step = output.point_step * output.width;

  removed_indices_->resize (nr_removed_p);
}

///////////////////////////////////////////////////////////////////////////////////////////
void
pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2>::applyFilter (Indices& indices)
{
  // If fields x/y/z are not present, we cannot filter
  if (x_idx_ == UNAVAILABLE || y_idx_ == UNAVAILABLE || z_idx_ == UNAVAILABLE)
  {
    PCL_ERROR ("[pcl::%s::applyFilter] Input dataset doesn't have x-y-z coordinates!\n", getClassName ().c_str ());
    indices.clear();
    return;
  }

  if (std_mul_ == 0.0)
  {
    PCL_ERROR ("[pcl::%s::applyFilter] Standard deviation multipler not set!\n", getClassName ().c_str ());
    indices.clear();
    return;
  }

  double mean;
  double variance;
  double stddev;
  std::vector<float> distances;
  generateStatistics(mean, variance, stddev, distances);
  double const distance_threshold = mean + std_mul_ * stddev; // a distance that is bigger than this signals an outlier

  // Second pass: Classify the points on the computed distance threshold
  std::size_t nr_p = 0, nr_removed_p = 0;
  for (std::size_t cp = 0; cp < indices_->size (); ++cp)
  {
    // Points having a too high average distance are outliers and are passed to removed indices
    // Unless negative was set, then it's the opposite condition
    if ((!negative_ && distances[cp] > distance_threshold) || (negative_ && distances[cp] <= distance_threshold))
    {
      if (extract_removed_indices_)
        (*removed_indices_)[nr_removed_p++] = (*indices_)[cp];
      continue;
    }

    // Otherwise it was a normal point for output (inlier)
    indices[nr_p++] = (*indices_)[cp];
  }

  // Resize the output arrays
  indices.resize (nr_p);
  removed_indices_->resize (nr_p);
}

///////////////////////////////////////////////////////////////////////////////////////////
void
pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2>::generateStatistics (double& mean,
                                                                         double& variance,
                                                                         double& stddev,
                                                                         std::vector<float>& distances)
{
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
  Indices nn_indices (mean_k_);
  std::vector<float> nn_dists (mean_k_);

  distances.resize (indices_->size ());
  int valid_distances = 0;
  // Go over all the points and calculate the mean or smallest distance
  for (std::size_t cp = 0; cp < indices_->size (); ++cp)
  {
    if (!std::isfinite ((*cloud)[(*indices_)[cp]].x) || 
        !std::isfinite ((*cloud)[(*indices_)[cp]].y) ||
        !std::isfinite ((*cloud)[(*indices_)[cp]].z))
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
    distances[cp] = static_cast<float> (dist_sum / (mean_k_ - 1));
    valid_distances++;
  }

  // Estimate the mean and the standard deviation of the distance vector
  double sum = 0, sq_sum = 0;
  for (const float &distance : distances)
  {
    sum += distance;
    sq_sum += distance * distance;
  }

  mean = sum / static_cast<double>(valid_distances);
  variance = (sq_sum - sum * sum / static_cast<double>(valid_distances)) / (static_cast<double>(valid_distances) - 1);
  stddev = sqrt (variance);
}


#ifndef PCL_NO_PRECOMPILE
#include <pcl/impl/instantiate.hpp>
#include <pcl/point_types.h>

// Instantiations of specific point types
PCL_INSTANTIATE(StatisticalOutlierRemoval, PCL_XYZ_POINT_TYPES)

#endif    // PCL_NO_PRECOMPILE

