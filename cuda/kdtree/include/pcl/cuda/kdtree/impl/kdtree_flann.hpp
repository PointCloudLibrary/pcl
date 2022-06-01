/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009-2011, Willow Garage, Inc.
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
 * kdtree_flann.hpp
 * Adapted from: kdtree/include/pcl/kdtree/impl/kdtree_flann.hpp
 * Created on: Jun 01, 2022
 * Author: Ramzi Sabra
 */

#ifndef PCL_CUDA_KDTREE_KDTREE_IMPL_FLANN_H_
#define PCL_CUDA_KDTREE_KDTREE_IMPL_FLANN_H_

#include <stdexcept>

#include <flann/flann.hpp>
#include <flann/algorithms/kdtree_cuda_3d_index.h>

#include <pcl/cuda/kdtree/kdtree_flann.h>
#include <pcl/console/print.h>

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename Dist>
constexpr int pcl::cuda::KdTreeFLANN<PointT, Dist>::dim_;

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename Dist>
pcl::cuda::KdTreeFLANN<PointT, Dist>::KdTreeFLANN (bool sorted, int max_leaf_size)
  : pcl::cuda::KdTree<PointT> (sorted, max_leaf_size)
  , flann_index_ ()
  , identity_mapping_ (false)
  , total_nr_points_ (0)
  , param_k_ (::flann::SearchParams (-1 , epsilon_))
  , param_radius_ (::flann::SearchParams (-1, epsilon_, sorted))
{
  if (point_representation_->getNumberOfDimensions() != 3)
    throw std::domain_error("Invalid number of dimensions per point (should be 3).");
  
  if (!std::is_same<std::size_t, pcl::index_t>::value) {
    const auto message = "FLANN is not optimized for current index type. Will incur "
                         "extra allocations and copy\n";
    if (std::is_same<int, pcl::index_t>::value) {
      PCL_DEBUG(message); // since this has been the default behavior till PCL 1.12
    }
    else {
      PCL_WARN(message);
    }
  }
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename Dist>
pcl::cuda::KdTreeFLANN<PointT, Dist>::KdTreeFLANN (const KdTreeFLANN<PointT, Dist> &k)
  : pcl::cuda::KdTree<PointT> (false)
  , flann_index_ ()
  , identity_mapping_ (false)
  , total_nr_points_ (0)
  , param_k_ (::flann::SearchParams (-1 , epsilon_))
  , param_radius_ (::flann::SearchParams (-1, epsilon_, false))
{
  *this = k;
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename Dist> void
pcl::cuda::KdTreeFLANN<PointT, Dist>::setMaxLeafSize (int max_leaf_size)
{
  max_leaf_size_ = max_leaf_size;
  
  if (!input_) return;
  ::flann::Matrix<float> mat (cloud_.get (), index_mapping_.size (), dim_);
  ::flann::KDTreeCuda3dIndexParams params (max_leaf_size_);
  flann_index_.reset (new FLANNIndex (mat, params));
  flann_index_->buildIndex ();
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename Dist> void
pcl::cuda::KdTreeFLANN<PointT, Dist>::setEpsilon (float eps)
{
  epsilon_ = eps;
  param_k_ =  ::flann::SearchParams (-1 , epsilon_);
  param_radius_ = ::flann::SearchParams (-1 , epsilon_, sorted_);
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename Dist> void
pcl::cuda::KdTreeFLANN<PointT, Dist>::setSortedResults (bool sorted)
{
  sorted_ = sorted;
  param_k_ = ::flann::SearchParams (-1, epsilon_);
  param_radius_ = ::flann::SearchParams (-1, epsilon_, sorted_);
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename Dist> void
pcl::cuda::KdTreeFLANN<PointT, Dist>::setInputCloud (const PointCloudConstPtr &cloud, const IndicesConstPtr &indices)
{
  cleanup ();   // Perform an automatic cleanup of structures

  epsilon_ = 0.0f;   // default error bound value

  input_   = cloud;
  indices_ = indices;

  // Allocate enough data
  if (!input_)
  {
    PCL_ERROR ("[pcl::cuda::KdTreeFLANN::setInputCloud] Invalid input!\n");
    return;
  }
  if (indices != nullptr)
  {
    convertCloudToArray (*input_, *indices_);
  }
  else
  {
    convertCloudToArray (*input_);
  }
  total_nr_points_ = static_cast<uindex_t> (index_mapping_.size ());
  if (total_nr_points_ == 0)
  {
    PCL_ERROR ("[pcl::cuda::KdTreeFLANN::setInputCloud] Cannot create a KDTree with an empty input cloud!\n");
    return;
  }

  ::flann::Matrix<float> mat (cloud_.get (), index_mapping_.size (), dim_);
  ::flann::KDTreeCuda3dIndexParams params (max_leaf_size_);
  flann_index_.reset (new FLANNIndex (mat, params));
  flann_index_->buildIndex ();
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename Dist> void
pcl::cuda::KdTreeFLANN<PointT, Dist>::setPointRepresentation(const PointRepresentationConstPtr &point_representation)
{
  if (point_representation->getNumberOfDimensions() != 3)
    throw std::domain_error("Invalid number of dimensions per point (should be 3).");
  Base::setPointRepresentation(point_representation);
}

///////////////////////////////////////////////////////////////////////////////////////////
namespace pcl {
namespace cuda {
namespace flann {
namespace detail {
using IndexMatrix = Eigen::Matrix<index_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
// Replace using constexpr in C++17
template <class IndexT, class A, class B, class F, CompatWithFlann<IndexT> = true>
int
knn_search(A& index,
           B& queries,
           IndexMatrix& k_indices,
           ::flann::Matrix<float>& dists,
           unsigned int k,
           F& params)
{
  ::flann::Matrix<index_t> k_indices_wrapper (k_indices.data(), k_indices.rows(), k);
  return index.knnSearch(queries, k_indices_wrapper, dists, k, params);
}

template <class IndexT, class A, class B, class F, NotCompatWithFlann<IndexT> = true>
int
knn_search(A& index,
           B& queries,
           IndexMatrix& k_indices,
           ::flann::Matrix<float>& dists,
           unsigned int k,
           F& params)
{
  Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> indices (queries.rows, k);
  ::flann::Matrix<int> indices_wrapper (indices.data(), queries.rows, k);
  // flann will resize accordingly
  auto ret = index.knnSearch(queries, indices_wrapper, dists, k, params);
  k_indices = indices.template cast<index_t>();
  return ret;
}
} // namespace detail
template <class FlannIndex,
          class Queries,
          class Indices,
          class Distances,
          class SearchParams>
int
knn_search(const FlannIndex& index,
           const Queries& queries,
           Indices& indices,
           Distances& dists,
           unsigned int k,
           const SearchParams& params)
{
  return detail::knn_search<pcl::index_t>(index, queries, indices, dists, k, params);
}
} // namespace flann
} // namespace cuda
} // namespace pcl

template <typename PointT, typename Dist> int
pcl::cuda::KdTreeFLANN<PointT, Dist>::nearestKSearch(const PointVector& points, unsigned int k,
                                                     IndexMatrix& k_indices_mat,
                                                     DistanceMatrix& k_sqr_distances_mat) const
{
  using QueryMatrix = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
  
#if DEBUG
  for (const PointT& point : points)
    assert (point_representation_->isValid (point) && "Invalid (NaN, Inf) point coordinates given to nearestKSearch!");
#endif

  if (k > total_nr_points_)
    k = total_nr_points_;
  
  k_indices_mat.resize (points.size(), k);
  k_sqr_distances_mat.resize (points.size(), k);

  if (k==0)
    return 0;

  QueryMatrix queries_mat (points.size(), dim_);

  auto point_itr = points.cbegin();
  auto query_itr = queries_mat.rowwise().begin();

  for (; point_itr != points.cend(); ++point_itr, ++query_itr)
  {
    const PointT& point = *point_itr;
    auto query = *query_itr;
    point_representation_->vectorize (point, query);
  }

  ::flann::Matrix<float> queries_mat_wrapper (queries_mat.data(), points.size(), dim_);
  ::flann::Matrix<float> k_distances_mat_wrapper (k_sqr_distances_mat.data(), points.size(), k);

  pcl::cuda::flann::knn_search(*flann_index_,
             queries_mat_wrapper,
             k_indices_mat,
             k_distances_mat_wrapper,
             k,
             param_k_);

  // Do mapping to original point cloud
  if (!identity_mapping_)
  {
    std::transform(
      k_indices_mat.reshaped().cbegin(),
      k_indices_mat.reshaped().cend(),
      k_indices_mat.reshaped().begin(),
      [this](const index_t& neighbor_index)
      { return index_mapping_[neighbor_index]; }
    );
  }

  return (k);
}

template <typename PointT, typename Dist> int 
pcl::cuda::KdTreeFLANN<PointT, Dist>::nearestKSearch (const PointVector& points, unsigned int k,
                                                      IndicesVector& k_indices_vec,
                                                      DistancesVector& k_sqr_distances_vec) const
{
  if (k > total_nr_points_)
    k = total_nr_points_;
  
  IndexMatrix k_indices_mat;
  DistanceMatrix k_sqr_distances_mat;
  
  nearestKSearch(points, k, k_indices_mat, k_sqr_distances_mat);

  k_indices_vec.resize(points.size());
  k_sqr_distances_vec.resize(points.size());
  
  for (Indices& k_indices : k_indices_vec)
    k_indices.resize (k);
  for (std::vector<float>& k_sqr_distances : k_sqr_distances_vec)
    k_sqr_distances.resize (k);

  // copy indices from matrix to vector
  auto k_indices_row_itr = k_indices_mat.rowwise().cbegin();
  auto k_indices_itr = k_indices_vec.begin();

  for (; k_indices_itr != k_indices_vec.end(); ++k_indices_row_itr, ++k_indices_itr)
  {
    auto k_indices_row = *k_indices_row_itr;
    auto& k_indices = *k_indices_itr;
    std::copy(k_indices_row.cbegin(), k_indices_row.cend(), k_indices.begin());
  }

  // copy square distances from matrix to vector
  auto k_distances_row_itr = k_sqr_distances_mat.rowwise().cbegin();
  auto k_distances_itr = k_sqr_distances_vec.begin();

  for (; k_distances_itr != k_sqr_distances_vec.end(); ++k_distances_row_itr, ++k_distances_itr)
  {
    auto k_distances_row = *k_distances_row_itr;
    auto& k_distances = *k_distances_itr;
    std::copy(k_distances_row.cbegin(), k_distances_row.cend(), k_distances.begin());
  }
  
  return (k);
}

///////////////////////////////////////////////////////////////////////////////////////////
namespace pcl {
namespace cuda {
namespace flann {
namespace detail {
// Replace using constexpr in C++17
template <class IndexT, class A, class B, class F, CompatWithFlann<IndexT> = true>
int
radius_search(A& index,
              B& queries,
              IndexMatrix& indices,
              ::flann::Matrix<float>& sqr_distances,
              float radius,
              F& params)
{
  ::flann::Matrix<index_t> indices_wrapper (indices.data(), indices.rows(), indices.cols());
  return index.radiusSearch(queries, indices_wrapper, sqr_distances, radius, params);
}

template <class IndexT, class A, class B, class F, NotCompatWithFlann<IndexT> = true>
int
radius_search(A& index,
              B& queries,
              IndexMatrix& indices,
              ::flann::Matrix<float>& sqr_distances,
              float radius,
              F& params)
{
  Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> indices_int (queries.rows, params.max_neighbors);
  ::flann::Matrix<int> indices_wrapper (indices_int.data(), queries.rows, params.max_neighbors);
  // flann will resize accordingly
  int result = index.radiusSearch(queries, indices_wrapper, sqr_distances, radius, params);
  indices = indices_int.template cast<index_t>();
  return result;
}
} // namespace detail
template <class FlannIndex,
          class Queries,
          class Indices,
          class Distances,
          class SearchParams>
int
radius_search(const FlannIndex& index,
              const Queries& queries,
              Indices& indices,
              Distances& sqr_distances,
              float radius,
              const SearchParams& params)
{
  return detail::radius_search<pcl::index_t>(
      index, queries, indices, sqr_distances, radius, params);
}
} // namespace flann
} // namespace cuda
} // namespace pcl

template <typename PointT, typename Dist> int
pcl::cuda::KdTreeFLANN<PointT, Dist>::radiusSearch (const PointVector& points,
                                                    double radius,
                                                    IndexMatrix &indices_mat,
                                                    DistanceMatrix& sqr_distances_mat,
                                                    unsigned int max_nn) const
{
  using QueryMatrix = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
  
# if DEBUG
  for (const PointT& point : points)
    assert (point_representation_->isValid (point) && "Invalid (NaN, Inf) point coordinates given to radiusSearch!");
#endif

  // Has max_nn been set properly?
  if (max_nn == 0 || max_nn > total_nr_points_)
    max_nn = total_nr_points_;

  ::flann::SearchParams params (param_radius_);
  if (max_nn == total_nr_points_)
    params.max_neighbors = -1;  // return all neighbors in radius
  else
    params.max_neighbors = max_nn;

  QueryMatrix queries_mat (points.size(), dim_);

  auto point_itr = points.cbegin();
  auto query_itr = queries_mat.rowwise().begin();

  for (; point_itr != points.cend(); ++point_itr, ++query_itr)
  {
    const PointT& point = *point_itr;
    auto query = *query_itr;
    point_representation_->vectorize (point, query);
  }
  
  indices_mat.resize (points.size(), max_nn);
  sqr_distances_mat.resize (points.size(), max_nn);
  
  ::flann::Matrix<float> queries_mat_wrapper (queries_mat.data(), points.size(), dim_);
  ::flann::Matrix<float> sqr_distances_mat_wrapper (sqr_distances_mat.data(), points.size(), max_nn);
  
  int neighbors_in_radius = pcl::cuda::flann::radius_search(*flann_index_,
                                          queries_mat_wrapper,
                                          indices_mat,
                                          sqr_distances_mat_wrapper,
                                          static_cast<float>(radius * radius),
                                          params);
  
  // Do mapping to original point cloud
  if (!identity_mapping_)
  {
    std::transform(
      indices_mat.reshaped().cbegin(),
      indices_mat.reshaped().cend(),
      indices_mat.reshaped().begin(),
      [this](const index_t& neighbor_index)
      { return index_mapping_[neighbor_index]; }
    );
  }

  return (neighbors_in_radius);
}

template <typename PointT, typename Dist> int 
pcl::cuda::KdTreeFLANN<PointT, Dist>::radiusSearch (const PointVector& points,
                                                    double radius,
                                                    IndicesVector& indices_vec,
                                                    DistancesVector& sqr_distances_vec,
                                                    unsigned int max_nn) const
{
  IndexMatrix indices_mat;
  DistanceMatrix distances_mat;
  
  int neighbors_in_radius = radiusSearch(points, radius, indices_mat, distances_mat, max_nn);

  if (max_nn == 0 || max_nn > total_nr_points_)
    max_nn = total_nr_points_;

  indices_vec.resize(points.size());
  sqr_distances_vec.resize(points.size());

  // copy indices from matrix to vector
  auto indices_row_itr = indices_mat.rowwise().cbegin();
  auto indices_itr = indices_vec.begin();

  for (; indices_itr != indices_vec.end(); ++indices_row_itr, ++indices_itr)
  {
    auto indices_row = *indices_row_itr;
    Indices& indices = *indices_itr;

    indices.clear();
    indices.reserve(max_nn);

    std::copy_if(
      indices_row.cbegin(),
      indices_row.cend(),
      std::back_inserter(indices),
      [](int index)
      { return index != -1; }
    );
  }

  // copy square distances from matrix to vector
  auto distances_row_itr = distances_mat.rowwise().cbegin();
  auto distances_itr = sqr_distances_vec.begin();

  for (; distances_itr != sqr_distances_vec.end(); ++distances_row_itr, ++distances_itr)
  {
    auto distances_row = *distances_row_itr;
    std::vector<float>& distances = *distances_itr;
    
    distances.clear();
    distances.reserve(max_nn);
    
    std::copy_if(
      distances_row.cbegin(),
      distances_row.cend(),
      std::back_inserter(distances),
      [](float distance)
      { return std::isfinite(distance); }
    );
  }
  
  return (neighbors_in_radius);
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename Dist> void
pcl::cuda::KdTreeFLANN<PointT, Dist>::cleanup ()
{
  // Data array cleanup
  index_mapping_.clear ();

  if (indices_)
    indices_.reset ();
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename Dist> void
pcl::cuda::KdTreeFLANN<PointT, Dist>::convertCloudToArray (const PointCloud &cloud)
{
  // No point in doing anything if the array is empty
  if (cloud.empty ())
  {
    cloud_.reset ();
    return;
  }

  const auto original_no_of_points = cloud.size ();

  cloud_.reset (new float[original_no_of_points * dim_], std::default_delete<float[]> ());
  float* cloud_ptr = cloud_.get ();
  index_mapping_.reserve (original_no_of_points);
  identity_mapping_ = true;

  for (std::size_t cloud_index = 0; cloud_index < original_no_of_points; ++cloud_index)
  {
    // Check if the point is invalid
    if (!point_representation_->isValid (cloud[cloud_index]))
    {
      identity_mapping_ = false;
      continue;
    }

    index_mapping_.push_back (cloud_index);

    point_representation_->vectorize (cloud[cloud_index], cloud_ptr);
    cloud_ptr += dim_;
  }
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename Dist> void
pcl::cuda::KdTreeFLANN<PointT, Dist>::convertCloudToArray (const PointCloud &cloud, const Indices &indices)
{
  // No point in doing anything if the array is empty
  if (cloud.empty ())
  {
    cloud_.reset ();
    return;
  }

  int original_no_of_points = static_cast<int> (indices.size ());

  cloud_.reset (new float[original_no_of_points * dim_], std::default_delete<float[]> ());
  float* cloud_ptr = cloud_.get ();
  index_mapping_.reserve (original_no_of_points);
  // its a subcloud -> false
  // true only identity:
  //     - indices size equals cloud size
  //     - indices only contain values between 0 and cloud.size - 1
  //     - no index is multiple times in the list
  //     => index is complete
  // But we can not guarantee that => identity_mapping_ = false
  identity_mapping_ = false;

  for (const auto &index : indices)
  {
    // Check if the point is invalid
    if (!point_representation_->isValid (cloud[index]))
      continue;

    // map from 0 - N -> indices [0] - indices [N]
    index_mapping_.push_back (index);  // If the returned index should be for the indices vector

    point_representation_->vectorize (cloud[index], cloud_ptr);
    cloud_ptr += dim_;
  }
}

#define PCL_INSTANTIATE_KdTreeFLANN(T) template class PCL_EXPORTS pcl::cuda::KdTreeFLANN<T>;

#endif  //#ifndef PCL_CUDA_KDTREE_KDTREE_IMPL_FLANN_H_

