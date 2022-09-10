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
 */

#ifndef PCL_KDTREE_KDTREE_IMPL_FLANN_H_
#define PCL_KDTREE_KDTREE_IMPL_FLANN_H_

#include <flann/flann.hpp>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/console/print.h>

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename Dist>
pcl::KdTreeFLANN<PointT, Dist>::KdTreeFLANN (bool sorted)
  : pcl::KdTree<PointT> (sorted)
  , flann_index_ ()
  , identity_mapping_ (false)
  , dim_ (0), total_nr_points_ (0)
  , param_k_ (::flann::SearchParams (-1 , epsilon_))
  , param_radius_ (::flann::SearchParams (-1, epsilon_, sorted))
{
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
pcl::KdTreeFLANN<PointT, Dist>::KdTreeFLANN (const KdTreeFLANN<PointT, Dist> &k)
  : pcl::KdTree<PointT> (false)
  , flann_index_ ()
  , identity_mapping_ (false)
  , dim_ (0), total_nr_points_ (0)
  , param_k_ (::flann::SearchParams (-1 , epsilon_))
  , param_radius_ (::flann::SearchParams (-1, epsilon_, false))
{
  *this = k;
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename Dist> void
pcl::KdTreeFLANN<PointT, Dist>::setEpsilon (float eps)
{
  epsilon_ = eps;
  param_k_ =  ::flann::SearchParams (-1 , epsilon_);
  param_radius_ = ::flann::SearchParams (-1 , epsilon_, sorted_);
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename Dist> void
pcl::KdTreeFLANN<PointT, Dist>::setSortedResults (bool sorted)
{
  sorted_ = sorted;
  param_k_ = ::flann::SearchParams (-1, epsilon_);
  param_radius_ = ::flann::SearchParams (-1, epsilon_, sorted_);
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename Dist> void
pcl::KdTreeFLANN<PointT, Dist>::setInputCloud (const PointCloudConstPtr &cloud, const IndicesConstPtr &indices)
{
  cleanup ();   // Perform an automatic cleanup of structures

  epsilon_ = 0.0f;   // default error bound value
  dim_ = point_representation_->getNumberOfDimensions (); // Number of dimensions - default is 3 = xyz

  input_   = cloud;
  indices_ = indices;

  // Allocate enough data
  if (!input_)
  {
    PCL_ERROR ("[pcl::KdTreeFLANN::setInputCloud] Invalid input!\n");
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
    PCL_ERROR ("[pcl::KdTreeFLANN::setInputCloud] Cannot create a KDTree with an empty input cloud!\n");
    return;
  }

  flann_index_.reset (new FLANNIndex (::flann::Matrix<float> (cloud_.get (),
                                                              index_mapping_.size (),
                                                              dim_),
                                      ::flann::KDTreeSingleIndexParams (15))); // max 15 points/leaf
  flann_index_->buildIndex ();
}

///////////////////////////////////////////////////////////////////////////////////////////
namespace pcl {
namespace detail {
// Replace using constexpr in C++17
template <class IndexT,
          class A,
          class B,
          class C,
          class D,
          class F,
          CompatWithFlann<IndexT> = true>
int
knn_search(A& index, B& query, C& k_indices, D& dists, unsigned int k, F& params)
{
  // Wrap k_indices vector (no data allocation)
  ::flann::Matrix<index_t> k_indices_mat(&k_indices[0], 1, k);
  return index.knnSearch(query, k_indices_mat, dists, k, params);
}

template <class IndexT,
          class A,
          class B,
          class C,
          class D,
          class F,
          NotCompatWithFlann<IndexT> = true>
int
knn_search(A& index, B& query, C& k_indices, D& dists, unsigned int k, F& params)
{
  std::vector<std::size_t> indices(k);
  k_indices.resize(k);
  // Wrap indices vector (no data allocation)
  ::flann::Matrix<std::size_t> indices_mat(&indices[0], 1, k);
  auto ret = index.knnSearch(query, indices_mat, dists, k, params);
  // cast appropriately
  std::transform(indices.cbegin(),
                 indices.cend(),
                 k_indices.begin(),
                 [](const auto& x) { return static_cast<pcl::index_t>(x); });
  return ret;
}

template <class IndexT, class A, class B, class F, CompatWithFlann<IndexT> = true>
int
knn_search(A& index,
           B& query,
           std::vector<Indices>& k_indices,
           std::vector<std::vector<float>>& dists,
           unsigned int k,
           F& params)
{
  return index.knnSearch(query, k_indices, dists, k, params);
}

template <class IndexT, class A, class B, class F, NotCompatWithFlann<IndexT> = true>
int
knn_search(A& index,
           B& query,
           std::vector<Indices>& k_indices,
           std::vector<std::vector<float>>& dists,
           unsigned int k,
           F& params)
{
  std::vector<std::vector<std::size_t>> indices;
  // flann will resize accordingly
  auto ret = index.knnSearch(query, indices, dists, k, params);

  k_indices.resize(indices.size());
  {
    auto it = indices.cbegin();
    auto jt = k_indices.begin();
    for (; it != indices.cend(); ++it, ++jt) {
      jt->resize(it->size());
      std::copy(it->cbegin(), it->cend(), jt->begin());
    }
  }
  return ret;
}
} // namespace detail
template <class FlannIndex,
          class Query,
          class Indices,
          class Distances,
          class SearchParams>
int
knn_search(const FlannIndex& index,
           const Query& query,
           Indices& indices,
           Distances& dists,
           unsigned int k,
           const SearchParams& params)
{
  return detail::knn_search<pcl::index_t>(index, query, indices, dists, k, params);
}
} // namespace pcl

template <typename PointT, typename Dist> int 
pcl::KdTreeFLANN<PointT, Dist>::nearestKSearch (const PointT &point, unsigned int k,
                                                Indices &k_indices,
                                                std::vector<float> &k_distances) const
{
  assert (point_representation_->isValid (point) && "Invalid (NaN, Inf) point coordinates given to nearestKSearch!");

  if (k > total_nr_points_)
    k = total_nr_points_;

  k_indices.resize (k);
  k_distances.resize (k);

  if (k==0)
    return 0;

  std::vector<float> query (dim_);
  point_representation_->vectorize (static_cast<PointT> (point), query);

  // Wrap the k_distances vector (no data copy)
  ::flann::Matrix<float> k_distances_mat (&k_distances[0], 1, k);

  knn_search(*flann_index_,
             ::flann::Matrix<float>(&query[0], 1, dim_),
             k_indices,
             k_distances_mat,
             k,
             param_k_);

  // Do mapping to original point cloud
  if (!identity_mapping_)
  {
    for (std::size_t i = 0; i < static_cast<std::size_t> (k); ++i)
    {
      auto& neighbor_index = k_indices[i];
      neighbor_index = index_mapping_[neighbor_index];
    }
  }

  return (k);
}

///////////////////////////////////////////////////////////////////////////////////////////
namespace pcl {
namespace detail {
// Replace using constexpr in C++17
template <class IndexT,
          class A,
          class B,
          class C,
          class D,
          class F,
          CompatWithFlann<IndexT> = true>
int
radius_search(A& index, B& query, C& k_indices, D& dists, float radius, F& params)
{
  std::vector<pcl::Indices> indices(1);
  int neighbors_in_radius = index.radiusSearch(query, indices, dists, radius, params);
  k_indices = std::move(indices[0]);
  return neighbors_in_radius;
}

template <class IndexT,
          class A,
          class B,
          class C,
          class D,
          class F,
          NotCompatWithFlann<IndexT> = true>
int
radius_search(A& index, B& query, C& k_indices, D& dists, float radius, F& params)
{
  std::vector<std::vector<std::size_t>> indices(1);
  int neighbors_in_radius = index.radiusSearch(query, indices, dists, radius, params);
  k_indices.resize(indices[0].size());
  // cast appropriately
  std::transform(indices[0].cbegin(),
                 indices[0].cend(),
                 k_indices.begin(),
                 [](const auto& x) { return static_cast<pcl::index_t>(x); });
  return neighbors_in_radius;
}

template <class IndexT, class A, class B, class F, CompatWithFlann<IndexT> = true>
int
radius_search(A& index,
              B& query,
              std::vector<Indices>& k_indices,
              std::vector<std::vector<float>>& dists,
              float radius,
              F& params)
{
  return index.radiusSearch(query, k_indices, dists, radius, params);
}

template <class IndexT, class A, class B, class F, NotCompatWithFlann<IndexT> = true>
int
radius_search(A& index,
              B& query,
              std::vector<Indices>& k_indices,
              std::vector<std::vector<float>>& dists,
              float radius,
              F& params)
{
  std::vector<std::vector<std::size_t>> indices;
  // flann will resize accordingly
  auto ret = index.radiusSearch(query, indices, dists, radius, params);

  k_indices.resize(indices.size());
  {
    auto it = indices.cbegin();
    auto jt = k_indices.begin();
    for (; it != indices.cend(); ++it, ++jt) {
      jt->resize(it->size());
      std::copy(it->cbegin(), it->cend(), jt->begin());
    }
  }
  return ret;
}
} // namespace detail
template <class FlannIndex,
          class Query,
          class Indices,
          class Distances,
          class SearchParams>
int
radius_search(const FlannIndex& index,
              const Query& query,
              Indices& indices,
              Distances& dists,
              float radius,
              const SearchParams& params)
{
  return detail::radius_search<pcl::index_t>(
      index, query, indices, dists, radius, params);
}
} // namespace pcl

template <typename PointT, typename Dist> int
pcl::KdTreeFLANN<PointT, Dist>::radiusSearch (const PointT &point, double radius, Indices &k_indices,
                                              std::vector<float> &k_sqr_dists, unsigned int max_nn) const
{
  assert (point_representation_->isValid (point) && "Invalid (NaN, Inf) point coordinates given to radiusSearch!");

  std::vector<float> query (dim_);
  point_representation_->vectorize (static_cast<PointT> (point), query);

  // Has max_nn been set properly?
  if (max_nn == 0 || max_nn > total_nr_points_)
    max_nn = total_nr_points_;

  std::vector<std::vector<float> > dists(1);

  ::flann::SearchParams params (param_radius_);
  if (max_nn == total_nr_points_)
    params.max_neighbors = -1;  // return all neighbors in radius
  else
    params.max_neighbors = max_nn;

  auto query_mat = ::flann::Matrix<float>(&query[0], 1, dim_);
  int neighbors_in_radius = radius_search(*flann_index_,
                                          query_mat,
                                          k_indices,
                                          dists,
                                          static_cast<float>(radius * radius),
                                          params);

  k_sqr_dists = dists[0];

  // Do mapping to original point cloud
  if (!identity_mapping_)
  {
    for (int i = 0; i < neighbors_in_radius; ++i)
    {
      auto& neighbor_index = k_indices[i];
      neighbor_index = index_mapping_[neighbor_index];
    }
  }

  return (neighbors_in_radius);
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename Dist> void
pcl::KdTreeFLANN<PointT, Dist>::cleanup ()
{
  // Data array cleanup
  index_mapping_.clear ();

  if (indices_)
    indices_.reset ();
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename Dist> void
pcl::KdTreeFLANN<PointT, Dist>::convertCloudToArray (const PointCloud &cloud)
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
pcl::KdTreeFLANN<PointT, Dist>::convertCloudToArray (const PointCloud &cloud, const Indices &indices)
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

#define PCL_INSTANTIATE_KdTreeFLANN(T) template class PCL_EXPORTS pcl::KdTreeFLANN<T>;

#endif  //#ifndef _PCL_KDTREE_KDTREE_IMPL_FLANN_H_

