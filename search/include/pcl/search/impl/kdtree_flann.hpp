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
 */

#ifndef PCL_SEARCH_IMPL_FLANN_H_
#define PCL_SEARCH_IMPL_FLANN_H_

#include "pcl/search/kdtree_flann.h"
#include <pcl/console/print.h>
#include <iostream>
#include <flann/flann.hpp>

////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void 
pcl::KdTreeFLANN<PointT>::cleanup ()
{
  delete flann_index_;

  m_lock_.lock ();
  // Data array cleanup
  free (cloud_);
  cloud_ = NULL;
  index_mapping_.clear();

  if (indices_) indices_.reset ();

  m_lock_.unlock ();
}

////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void 
pcl::KdTreeFLANN<PointT>::setInputCloud (const PointCloudConstPtr& cloud, const IndicesConstPtr& indices)
{
    cleanup ();   // Perform an automatic cleanup of structures

    if (!initParameters()) return;

    input_   = cloud;
    indices_ = indices;

    if (input_ == NULL) return;

    m_lock_.lock ();
    // Allocate enough data
    if (!input_) {
        PCL_ERROR ("[pcl::KdTreeANN::setInputCloud] Invalid input!\n");
        return;
    }
    if (indices != NULL) convertCloudToArray (*input_, *indices_);
    else convertCloudToArray (*input_);

    initData ();
    m_lock_.unlock ();
}

////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void 
pcl::KdTreeFLANN<PointT>::setInputCloud (const PointCloudConstPtr& cloud)
{
  const IndicesConstPtr& indices = IndicesConstPtr();
  setInputCloud(cloud,indices);
}

////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> int
pcl::KdTreeFLANN<PointT>::nearestKSearch (const PointT& point, int k,
                                          std::vector<int>& k_indices,
                                          std::vector<float>& k_distances)
{
  if (!point_representation_->isValid (point)) 
  {
      //PCL_ERROR_STREAM ("[pcl::KdTreeFLANN::nearestKSearch] Invalid query point given!" << point);
      return 0;
  }

  //	std::cout << "reached nearestK search" << std::endl;
  std::vector<float> tmp (dim_);
  point_representation_->vectorize ((PointT)point, tmp);

  flann::Matrix<int> k_indices_mat (&k_indices[0], 1, k);
  flann::Matrix<float> k_distances_mat (&k_distances[0], 1, k);
  flann_index_->knnSearch (flann::Matrix<float>(&tmp[0], 1, dim_), k_indices_mat, k_distances_mat, k, flann::SearchParams (-1,epsilon_));

  // Do mapping to original point cloud
  if (!identity_mapping_) {
      for (size_t i = 0; i < k_indices.size (); ++i) {
          int& neighbor_index = k_indices[i];
          neighbor_index = index_mapping_[neighbor_index];
      }
  }

  return k;
}


////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> int 
pcl::KdTreeFLANN<PointT>::radiusSearch (const PointT& point, double radius, std::vector<int>& k_indices,
                                        std::vector<float>& k_squared_distances, int max_nn) const
{
  static flann::Matrix<int> indices_empty;
  static flann::Matrix<float> dists_empty;

  if (!point_representation_->isValid (point)) 
  {
      //PCL_ERROR_STREAM ("[pcl::KdTreeFLANN::radiusSearch] Invalid query point given!" << point);
      return 0;
  }

  std::vector<float> tmp(dim_);
  point_representation_->vectorize ((PointT)point, tmp);
  radius *= radius; // flann uses squared radius

  size_t size;
  if (indices_ == NULL) // no indices set, use full size of point cloud:
      size = input_->points.size ();
  else size = indices_->size ();

  int neighbors_in_radius = 0;
  if ((k_indices.size () == size)&&(k_squared_distances.size () == size)) 
  { // preallocated vectors
    // if using preallocated vectors we ignore max_nn as we are sure to have enought space
    // to store all neighbors found in radius
    flann::Matrix<int> k_indices_mat (&k_indices[0], 1, k_indices.size());
    flann::Matrix<float> k_distances_mat (&k_squared_distances[0], 1, k_squared_distances.size());
    neighbors_in_radius = flann_index_->radiusSearch (flann::Matrix<float>(&tmp[0], 1, dim_),
                                                      k_indices_mat, k_distances_mat, radius, flann::SearchParams (-1, epsilon_, sorted_));
  }
  else 
  { // need to do search twice, first to find how many neighbors and allocate the vectors
    neighbors_in_radius = flann_index_->radiusSearch (flann::Matrix<float>(&tmp[0], 1, dim_),
                                                      indices_empty, dists_empty, radius, flann::SearchParams (-1, epsilon_, sorted_));
    if (max_nn > 0) {
        neighbors_in_radius = std::min(neighbors_in_radius, max_nn);
    }
    k_indices.resize (neighbors_in_radius);
    k_squared_distances.resize (neighbors_in_radius);
    if (neighbors_in_radius == 0) {
        return 0;
    }

    flann::Matrix<int> k_indices_mat (&k_indices[0], 1, k_indices.size());
    flann::Matrix<float> k_distances_mat (&k_squared_distances[0], 1, k_squared_distances.size());
    flann_index_->radiusSearch (flann::Matrix<float>(&tmp[0], 1, dim_),
                                k_indices_mat, k_distances_mat, radius, flann::SearchParams (-1, epsilon_, sorted_));

  }

  // Do mapping to original point cloud
  if (!identity_mapping_) 
  {
    for (int i = 0; i < neighbors_in_radius; ++i) 
    {
      int& neighbor_index = k_indices[i];
      neighbor_index = index_mapping_[neighbor_index];
    }
  }

  return (neighbors_in_radius);
}



////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool 
pcl::KdTreeFLANN<PointT>::initParameters ()
{
  epsilon_ = 0.0;   // default error bound value
  dim_ = point_representation_->getNumberOfDimensions (); // Number of dimensions - default is 3 = xyz
  // Create the kd_tree representation
  return (true);
}

////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void 
pcl::KdTreeFLANN<PointT>::initData ()
{
  flann_index_ = new FLANNIndex (flann::Matrix<float>(cloud_, index_mapping_.size(), dim_),
                                 flann::KDTreeSingleIndexParams(15)); // max 15 points/leaf
  flann_index_->buildIndex();
}

////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void 
pcl::KdTreeFLANN<PointT>::convertCloudToArray (const PointCloud& ros_cloud)
{
  // No point in doing anything if the array is empty
  if (ros_cloud.points.empty ()) 
  {
    cloud_ = NULL;
    return;
  }

  int original_no_of_points = ros_cloud.points.size();

  cloud_ = (float*)malloc (original_no_of_points * dim_ * sizeof(float));
  float* cloud_ptr = cloud_;
  index_mapping_.reserve(original_no_of_points);
  identity_mapping_ = true;

  for (int cloud_index = 0; cloud_index < original_no_of_points; ++cloud_index) 
  {
    const PointT point = ros_cloud.points[cloud_index];
    // Check if the point is invalid
    if (!point_representation_->isValid(point)) 
    {
      identity_mapping_ = false;
      continue;
    }

    index_mapping_.push_back(cloud_index);

    point_representation_->vectorize(point, cloud_ptr);
    cloud_ptr += dim_;
  }
}

////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void 
pcl::KdTreeFLANN<PointT>::convertCloudToArray (const PointCloud& ros_cloud, const std::vector<int>& indices)
{
  // No point in doing anything if the array is empty
  if (ros_cloud.points.empty ()) 
  {
    cloud_ = NULL;
    return;
  }

  int original_no_of_points = indices.size();

  cloud_ = (float*)malloc (original_no_of_points * dim_ * sizeof (float));
  float* cloud_ptr = cloud_;
  index_mapping_.reserve(original_no_of_points);
  identity_mapping_ = true;

  for (int indices_index = 0; indices_index < original_no_of_points; ++indices_index) 
  {
    int cloud_index = indices[indices_index];
    const PointT point = ros_cloud.points[cloud_index];
    // Check if the point is invalid
    if (!point_representation_->isValid(point)) 
    {
      identity_mapping_ = false;
      continue;
    }

    index_mapping_.push_back(indices_index);  // If the returned index should be for the indices vector
    //index_mapping_.push_back(cloud_index);  // If the returned index should be for the ros cloud

    point_representation_->vectorize(point, cloud_ptr);
    cloud_ptr += dim_;
  }
}

#define PCL_INSTANTIATE_KdTreeFLANN(T) template class PCL_EXPORTS pcl::KdTreeFLANN<T>;

#endif  //#ifndef _PCL_KDTREE_KDTREE_IMPL_FLANN_H_

