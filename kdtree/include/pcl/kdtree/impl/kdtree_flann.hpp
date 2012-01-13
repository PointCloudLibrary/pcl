/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009-2011, Willow Garage, Inc.
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

#ifndef PCL_KDTREE_KDTREE_IMPL_FLANN_H_
#define PCL_KDTREE_KDTREE_IMPL_FLANN_H_

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/console/print.h>

////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename Dist> void 
pcl::KdTreeFLANN<PointT, Dist>::setInputCloud (const PointCloudConstPtr &cloud, const IndicesConstPtr &indices)
{
  cleanup ();   // Perform an automatic cleanup of structures

  epsilon_ = 0.0;   // default error bound value
  dim_ = point_representation_->getNumberOfDimensions (); // Number of dimensions - default is 3 = xyz

  input_   = cloud;
  indices_ = indices;
  
  // Allocate enough data
  if (!input_)
  {
    PCL_ERROR ("[pcl::KdTreeANN::setInputCloud] Invalid input!\n");
    return;
  }
  if (indices != NULL)
    convertCloudToArray (*input_, *indices_);
  else
    convertCloudToArray (*input_);

  flann_index_ = new FLANNIndex (flann::Matrix<float> (cloud_, index_mapping_.size (), dim_),
                                 flann::KDTreeSingleIndexParams (15)); // max 15 points/leaf
  flann_index_->buildIndex ();
}

////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename Dist> int 
pcl::KdTreeFLANN<PointT, Dist>::nearestKSearch (const PointT &point, int k, 
                                                std::vector<int> &k_indices, 
                                                std::vector<float> &k_distances)
{
  assert (point_representation_->isValid (point) && "Invalid (NaN, Inf) point coordinates given to nearestKSearch!");

  // Pay the price of resizing the arrays the first time nearestKSearch gets called
  if (k_indices.size () < (size_t)k)
    k_indices.resize (k);
  if (k_distances.size () < (size_t)k) 
    k_distances.resize (k);

  // Wrap the k_indices and k_distances vectors (no data copy)
  flann::Matrix<int> k_indices_mat (&k_indices[0], 1, k);
  flann::Matrix<float> k_distances_mat (&k_distances[0], 1, k);

  std::vector<float> tmp (dim_);
  point_representation_->vectorize ((PointT)point, tmp);

  flann_index_->knnSearch (flann::Matrix<float>(&tmp[0], 1, dim_), k_indices_mat, k_distances_mat, k, param_k_);

  // Do mapping to original point cloud
  if (!identity_mapping_) 
  {
    for (size_t i = 0; i < (size_t)k; ++i)
    {
      int& neighbor_index = k_indices[i];
      neighbor_index = index_mapping_[neighbor_index];
    }
  }

  return (k);
}

////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename Dist> int 
pcl::KdTreeFLANN<PointT, Dist>::radiusSearch (const PointT &point, double radius, std::vector<int> &k_indices,
                                              std::vector<float> &k_squared_distances, int max_nn) const
{
  assert (point_representation_->isValid (point) && "Invalid (NaN, Inf) point coordinates given to radiusSearch!");

  static flann::Matrix<int> indices_empty;
  static flann::Matrix<float> dists_empty;

  std::vector<float> tmp (dim_);
  point_representation_->vectorize ((PointT)point, tmp);
  radius *= radius; // flann uses squared radius

  size_t size;
  if (indices_ == NULL) // no indices set, use full size of point cloud:
    size = input_->points.size ();
  else
    size = indices_->size ();

  int neighbors_in_radius = 0;
  if (k_indices.size () == size && k_squared_distances.size () == size)  // preallocated vectors
  {
    // if using preallocated vectors we ignore max_nn as we are sure to have enought space
    // to store all neighbors found in radius
    flann::Matrix<int> k_indices_mat (&k_indices[0], 1, k_indices.size ());
    flann::Matrix<float> k_distances_mat (&k_squared_distances[0], 1, k_squared_distances.size ());
    neighbors_in_radius = flann_index_->radiusSearch (flann::Matrix<float>(&tmp[0], 1, dim_),
                                                      k_indices_mat, k_distances_mat, radius, 
                                                      param_radius_);
  }
  else // need to do search twice, first to find how many neighbors and allocate the vectors
  {
    neighbors_in_radius = flann_index_->radiusSearch (flann::Matrix<float>(&tmp[0], 1, dim_),
                                                      indices_empty, dists_empty, radius, 
                                                      param_radius_);

    // If the number of maximum nearest neighbors is given, use it to cap the return
    if (max_nn > 0) 
      neighbors_in_radius = std::min (neighbors_in_radius, max_nn);

    k_indices.resize (neighbors_in_radius);
    k_squared_distances.resize (neighbors_in_radius);

    if (neighbors_in_radius == 0)
      return (0);

    flann::Matrix<int> k_indices_mat (&k_indices[0], 1, k_indices.size ());
    flann::Matrix<float> k_distances_mat (&k_squared_distances[0], 1, k_squared_distances.size ());
    flann_index_->radiusSearch (flann::Matrix<float>(&tmp[0], 1, dim_),
                                k_indices_mat, k_distances_mat, radius, 
                                param_radius_);
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
template <typename PointT, typename Dist> void 
pcl::KdTreeFLANN<PointT, Dist>::cleanup ()
{
  delete flann_index_;

  // Data array cleanup
  free (cloud_);
  cloud_ = NULL;
  index_mapping_.clear();

  if (indices_)
    indices_.reset ();
}

////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename Dist> void 
pcl::KdTreeFLANN<PointT, Dist>::convertCloudToArray (const PointCloud &cloud)
{
  // No point in doing anything if the array is empty
  if (cloud.points.empty ())
  {
    cloud_ = NULL;
    return;
  }

  int original_no_of_points = cloud.points.size ();

  cloud_ = (float*)malloc (original_no_of_points * dim_ * sizeof(float));
  float* cloud_ptr = cloud_;
  index_mapping_.reserve (original_no_of_points);
  identity_mapping_ = true;

  for (int cloud_index = 0; cloud_index < original_no_of_points; ++cloud_index)
  {
    // Check if the point is invalid
    if (!point_representation_->isValid (cloud.points[cloud_index]))
    {
      identity_mapping_ = false;
      continue;
    }

    index_mapping_.push_back (cloud_index);

    point_representation_->vectorize (cloud.points[cloud_index], cloud_ptr);
    cloud_ptr += dim_;
  }
}

////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename Dist> void 
pcl::KdTreeFLANN<PointT, Dist>::convertCloudToArray (const PointCloud &cloud, const std::vector<int> &indices)
{
  // No point in doing anything if the array is empty
  if (cloud.points.empty ())
  {
    cloud_ = NULL;
    return;
  }

  int original_no_of_points = indices.size ();

  cloud_ = (float*)malloc (original_no_of_points * dim_ * sizeof (float));
  float* cloud_ptr = cloud_;
  index_mapping_.reserve (original_no_of_points);
  identity_mapping_ = true;

  for (int indices_index = 0; indices_index < original_no_of_points; ++indices_index)
  {
    // Check if the point is invalid
    if (!point_representation_->isValid (cloud.points[indices[indices_index]]))
    {
      identity_mapping_ = false;
      continue;
    }

    index_mapping_.push_back (indices_index);  // If the returned index should be for the indices vector
    
    point_representation_->vectorize (cloud.points[indices[indices_index]], cloud_ptr);
    cloud_ptr += dim_;
  }
}

#define PCL_INSTANTIATE_KdTreeFLANN(T) template class PCL_EXPORTS pcl::KdTreeFLANN<T>;

#endif  //#ifndef _PCL_KDTREE_KDTREE_IMPL_FLANN_H_

