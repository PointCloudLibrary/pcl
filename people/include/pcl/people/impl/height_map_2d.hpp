/*
 * Software License Agreement (BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2013-, Open Perception, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 * with the distribution.
 * * Neither the name of the copyright holder(s) nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * height_map_2d.hpp
 * Created on: Nov 30, 2012
 * Author: Matteo Munaro
 */

#include <pcl/people/height_map_2d.h>

#ifndef PCL_PEOPLE_HEIGHT_MAP_2D_HPP_
#define PCL_PEOPLE_HEIGHT_MAP_2D_HPP_

template <typename PointT>
pcl::people::HeightMap2D<PointT>::HeightMap2D ()
{
  // set default values for optional parameters:
  vertical_ = false;
  min_dist_between_maxima_ = 0.3;
  bin_size_ = 0.06;

  // set flag values for mandatory parameters:
  sqrt_ground_coeffs_ = std::numeric_limits<float>::quiet_NaN();
}

template <typename PointT> void
pcl::people::HeightMap2D<PointT>::compute (pcl::people::PersonCluster<PointT>& cluster)
{
  // Check if all mandatory variables have been set:
  if (sqrt_ground_coeffs_ != sqrt_ground_coeffs_)
  {
    PCL_ERROR ("[pcl::people::HeightMap2D::compute] Floor parameters have not been set or they are not valid!\n");
    return;
  }
  if (cloud_ == NULL)
  {
    PCL_ERROR ("[pcl::people::HeightMap2D::compute] Input cloud has not been set!\n");
    return;
  }

  // Reset variables:
  buckets_.clear();
  buckets_cloud_indices_.clear();
  maxima_indices_.clear();
  maxima_cloud_indices_.clear();
  maxima_indices_filtered_.clear();
  maxima_cloud_indices_filtered_.clear();

  // Create a height map with the projection of cluster points onto the ground plane:
  if (!vertical_)    // camera horizontal
    buckets_.resize(size_t((cluster.getMax()(0) - cluster.getMin()(0)) / bin_size_) + 1, 0);
  else        // camera vertical
    buckets_.resize(size_t((cluster.getMax()(1) - cluster.getMin()(1)) / bin_size_) + 1, 0);
  buckets_cloud_indices_.resize(buckets_.size(), 0);

  for(std::vector<int>::const_iterator pit = cluster.getIndices().indices.begin(); pit != cluster.getIndices().indices.end(); pit++)
  {
    PointT* p = &cloud_->points[*pit];
    int index;
    if (!vertical_)    // camera horizontal
      index = int((p->x - cluster.getMin()(0)) / bin_size_);
    else        // camera vertical
      index = int((p->y - cluster.getMin()(1)) / bin_size_);
    if (index > (static_cast<int> (buckets_.size ()) - 1))
      std::cout << "Error: out of array - " << index << " of " << buckets_.size() << std::endl;
    else
    {
      Eigen::Vector4f new_point(p->x, p->y, p->z, 1.0f);      // select point from cluster
      float heightp = std::fabs(new_point.dot(ground_coeffs_)); // compute point height from the groundplane
      heightp /= sqrt_ground_coeffs_;
      if ((heightp * 60) > buckets_[index])   // compare the height of the new point with the existing one
      {
        buckets_[index] = heightp * 60;   // maximum height
        buckets_cloud_indices_[index] = *pit;     // point cloud index of the point with maximum height
      }
    }
  }

  // Compute local maxima of the height map:
  searchLocalMaxima();

  // Filter maxima by imposing a minimum distance between them (minimum distance between people heads):
  filterMaxima();
}

template <typename PointT> void
pcl::people::HeightMap2D<PointT>::searchLocalMaxima ()
{
  // Search for local maxima:
  maxima_number_ = 0;
  int left = buckets_[0];         // current left element
  int right = 0;              // current right element
  float offset = 0;           // used to center the maximum to the right place
  maxima_indices_.resize(size_t(buckets_.size()), 0);
  maxima_cloud_indices_.resize(size_t(buckets_.size()), 0);

  // Handle first element:
  if (buckets_[0] > buckets_[1])
  {
    maxima_indices_[maxima_number_] = 0;
    maxima_cloud_indices_[maxima_number_] = buckets_cloud_indices_[maxima_indices_[maxima_number_]];
    maxima_number_++;
  }

  // Main loop:
  int i = 1;
  while (i < (static_cast<int> (buckets_.size()) - 1))
  {
    right = buckets_[i+1];
    if ((buckets_[i] > left) && (buckets_[i] > right))
    {
      // Search where to insert the new element (in an ordered array):
      int t = 0;    // position of the new element
      while ((t < maxima_number_) && (buckets_[i] < buckets_[maxima_indices_[t]]))
      {
        t++;
      }
      // Move forward the smaller elements:
      for (int m = maxima_number_; m > t; m--)
      {
        maxima_indices_[m] = maxima_indices_[m-1];
        maxima_cloud_indices_[m] = maxima_cloud_indices_[m-1];
      }
      // Insert the new element:
      maxima_indices_[t] = i - int(offset/2 + 0.5);
      maxima_cloud_indices_[t] = buckets_cloud_indices_[maxima_indices_[t]];
      left = buckets_[i+1];
      i = i+2;
      offset = 0;
      maxima_number_++;
    }
    else
    {
      if (buckets_[i] == right)
      {
        offset++;
      }
      else
      {
        left = buckets_[i];
        offset = 0;
      }
      i++;
    }
  }

  // Handle last element:
  if (buckets_[buckets_.size()-1] > left)
  {
    // Search where to insert the new element (in an ordered array):
    int t = 0;    // position of the new element
    while ((t < maxima_number_) && (buckets_[buckets_.size()-1] < buckets_[maxima_indices_[t]]))
    {
      t++;
    }
    // Move forward the smaller elements:
    for (int m = maxima_number_; m > t; m--)
    {
      maxima_indices_[m] = maxima_indices_[m-1];
      maxima_cloud_indices_[m] = maxima_cloud_indices_[m-1];
    }
    // Insert the new element:
    maxima_indices_[t] = i - int(offset/2 + 0.5);
    maxima_cloud_indices_[t] = buckets_cloud_indices_[maxima_indices_[t]];

    maxima_number_++;
  }
}

template <typename PointT> void
pcl::people::HeightMap2D<PointT>::filterMaxima ()
{
  // Filter maxima according to their distance when projected on the ground plane:
  maxima_number_after_filtering_ = 0;
  maxima_indices_filtered_.resize(maxima_number_, 0);
  maxima_cloud_indices_filtered_.resize(maxima_number_, 0);
  if (maxima_number_ > 0)
  {
    for (int i = 0; i < maxima_number_; i++)
    {
      bool good_maximum = true;
      float distance = 0;

      PointT* p_current = &cloud_->points[maxima_cloud_indices_[i]];  // pointcloud point referring to the current maximum
      Eigen::Vector3f p_current_eigen(p_current->x, p_current->y, p_current->z);  // conversion to eigen
      float t = p_current_eigen.dot(ground_coeffs_.head(3)) / std::pow(sqrt_ground_coeffs_, 2); // height from the ground
      p_current_eigen = p_current_eigen - ground_coeffs_.head(3) * t;       // projection of the point on the groundplane

      int j = i-1;
      while ((j >= 0) && (good_maximum))
      {
        PointT* p_previous = &cloud_->points[maxima_cloud_indices_[j]];         // pointcloud point referring to an already validated maximum
        Eigen::Vector3f p_previous_eigen(p_previous->x, p_previous->y, p_previous->z);  // conversion to eigen
        float t = p_previous_eigen.dot(ground_coeffs_.head(3)) / std::pow(sqrt_ground_coeffs_, 2); // height from the ground
        p_previous_eigen = p_previous_eigen - ground_coeffs_.head(3) * t;         // projection of the point on the groundplane

        // distance of the projection of the points on the groundplane:
        distance = (p_current_eigen-p_previous_eigen).norm();
        if (distance < min_dist_between_maxima_)
        {
          good_maximum = false;
        }
        j--;
      }
      if (good_maximum)
      {
        maxima_indices_filtered_[maxima_number_after_filtering_] = maxima_indices_[i];
        maxima_cloud_indices_filtered_[maxima_number_after_filtering_] = maxima_cloud_indices_[i];
        maxima_number_after_filtering_++;
      }
    }
  }
}

template <typename PointT> void
pcl::people::HeightMap2D<PointT>::setInputCloud (PointCloudPtr& cloud)
{
  cloud_ = cloud;
}

template <typename PointT>
void pcl::people::HeightMap2D<PointT>::setGround(Eigen::VectorXf& ground_coeffs)
{
  ground_coeffs_ = ground_coeffs;
  sqrt_ground_coeffs_ = (ground_coeffs - Eigen::Vector4f(0.0f, 0.0f, 0.0f, ground_coeffs(3))).norm();
}

template <typename PointT> void
pcl::people::HeightMap2D<PointT>::setBinSize (float bin_size)
{
  bin_size_ = bin_size;
}

template <typename PointT> void
pcl::people::HeightMap2D<PointT>::setMinimumDistanceBetweenMaxima (float minimum_distance_between_maxima)
{
  min_dist_between_maxima_ = minimum_distance_between_maxima;
}

template <typename PointT> void
pcl::people::HeightMap2D<PointT>::setSensorPortraitOrientation (bool vertical)
{
  vertical_ = vertical;
}

template <typename PointT> std::vector<int>&
pcl::people::HeightMap2D<PointT>::getHeightMap ()
{
  return (buckets_);
}

template <typename PointT> float
pcl::people::HeightMap2D<PointT>::getBinSize ()
{
  return (bin_size_);
}

template <typename PointT> float
pcl::people::HeightMap2D<PointT>::getMinimumDistanceBetweenMaxima ()
{
  return (min_dist_between_maxima_);
}

template <typename PointT> int&
pcl::people::HeightMap2D<PointT>::getMaximaNumberAfterFiltering ()
{
  return (maxima_number_after_filtering_);
}

template <typename PointT> std::vector<int>&
pcl::people::HeightMap2D<PointT>::getMaximaCloudIndicesFiltered ()
{
  return (maxima_cloud_indices_filtered_);
}

template <typename PointT>
pcl::people::HeightMap2D<PointT>::~HeightMap2D ()
{
  // TODO Auto-generated destructor stub
}
#endif /* PCL_PEOPLE_HEIGHT_MAP_2D_HPP_ */
