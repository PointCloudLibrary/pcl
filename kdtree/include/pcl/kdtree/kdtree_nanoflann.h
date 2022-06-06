/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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
 * kdtree_nanoflann.h
 * Adapted from: kdtree_flann.h
 * Created on: Jun 01, 2022
 * Author: Ramzi Sabra
 */

#pragma once

#include <pcl/kdtree/kdtree.h>
#include <nanoflann.hpp>

#include <memory>

namespace pcl {
namespace nanoflann {
/**
 * @brief Compatibility template function to allow use of various types of indices with
 * nanoflann
 * @details Template is used for all params to not constrain any nanoflann side capability
 * @param[in,out] index A index searcher, of type
 * ::nanoflann::KDTreeSingleIndexAdaptor<Matrix, dims, Dist> or similar, where Dist is a
 * template for computing distance between 2 points
 * @param[in] query An Eigen::MatrixXf or compatible matrix representation of the
 * query point
 * @param[out] indices Indices found in radius
 * @param[out] dists Computed distance matrix
 * @param[in] radius Threshold for consideration
 * @param[in] params Any parameters to pass to the radius_search call
 */
template <class NanoflannIndex,
          class Query,
          class Indices,
          class Distances,
          class SearchParams>
int
radius_search(const NanoflannIndex& index,
              const Query& query,
              Indices& indices,
              Distances& dists,
              float radius,
              const SearchParams& params);

/**
 * @brief Compatibility template function to allow use of various types of indices with
 * nanoflann
 * @details Template is used for all params to not constrain any nanoflann side capability
 * @param[in,out] index A index searcher, of type
 * ::nanoflann::KDTreeSingleIndexAdaptor<Matrix, dims, Dist> or similar, where Dist is a
 * template for computing distance between 2 points
 * @param[in] query An Eigen::MatrixXf or compatible matrix representation of the
 * query point
 * @param[out] indices Neighboring k indices found
 * @param[out] dists Computed distance matrix
 * @param[in] k Number of neighbors to search for
 * @param[in] params Any parameters to pass to the knn_search call
 */
template <class NanoflannIndex,
          class Query,
          class Indices,
          class Distances,
          class SearchParams>
int
knn_search(const NanoflannIndex& index,
           const Query& query,
           Indices& indices,
           Distances& dists,
           unsigned int k,
           const SearchParams& params);
} // namespace nanoflann

/** \brief KdTreeNanoflann is a generic type of 3D spatial locator using kd-tree structures.
 * The class is making use of the nanoflann project by Jose Luis Blanco-Claraco.
 *
 * \author Ramzi Sabra
 * \ingroup kdtree
 */
template <typename PointT, typename Dist = ::nanoflann::metric_L2_Simple, int dims = -1>
class KdTreeNanoflann : public pcl::KdTree<PointT> {
public:
  using KdTree<PointT>::input_;
  using KdTree<PointT>::indices_;
  using KdTree<PointT>::max_leaf_size_;
  using KdTree<PointT>::epsilon_;
  using KdTree<PointT>::sorted_;
  using KdTree<PointT>::point_representation_;
  using KdTree<PointT>::nearestKSearch;
  using KdTree<PointT>::radiusSearch;

  using PointCloud = typename KdTree<PointT>::PointCloud;
  using PointCloudConstPtr = typename KdTree<PointT>::PointCloudConstPtr;

  using IndicesPtr = shared_ptr<Indices>;
  using IndicesConstPtr = shared_ptr<const Indices>;

  using Matrix = Eigen::Matrix<float, Eigen::Dynamic, dims, Eigen::RowMajor>;
  using NanoflannIndex = ::nanoflann::KDTreeEigenMatrixAdaptor<Matrix, dims, Dist>;

  using Tree = KdTreeNanoflann<PointT, Dist, dims>;
  using Ptr = shared_ptr<Tree>;
  using ConstPtr = shared_ptr<const Tree>;

  /** \brief Default Constructor for KdTreeNanoflann.
   * \param[in] sorted set to true if the application that the tree will be used for
   * requires sorted nearest neighbor indices (default). False otherwise.
   * \param[in] max_leaf_size maximum leaf node size. Set to 15 by default.
   *
   * By setting sorted to false, \ref radiusSearch operations will be faster.
   */
  KdTreeNanoflann(bool sorted = true, int max_leaf_size = 15);

  /** \brief Copy constructor
   * \param[in] k the tree to copy into this
   */
  KdTreeNanoflann(const Tree& k);

  /** \brief Copy operator
   * \param[in] k the tree to copy into this
   */
  inline Tree&
  operator=(const Tree& k)
  {
    KdTree<PointT>::operator=(k);
    index_mapping_ = k.index_mapping_;
    identity_mapping_ = k.identity_mapping_;
    dim_ = k.dim_;
    total_nr_points_ = k.total_nr_points_;
    param_radius_ = k.param_radius_;
    
    if (k.nanoflann_index_ != nullptr)
    {
      cloud_mat_ = k.cloud_mat_;
      nanoflann_index_.reset(new NanoflannIndex(dim_, cloud_mat_, max_leaf_size_));
    }
    
    return (*this);
  }

  /** \brief Set the maximum leaf node size.
   * \param[in] max_leaf_size maximum leaf node size
   */
  void
  setMaxLeafSize(int max_leaf_size) override;

  /** \brief Set the search epsilon precision (error bound) for nearest neighbors
   * searches. \param[in] eps precision (error bound) for nearest neighbors searches
   */
  void
  setEpsilon(float eps) override;

  void
  setSortedResults(bool sorted);

  inline Ptr
  makeShared()
  {
    return Ptr(new Tree(*this));
  }

  /** \brief Destructor for KdTreeNanoflann.
   * Deletes all allocated data arrays and destroys the kd-tree structures.
   */
  ~KdTreeNanoflann() { cleanup(); }

  /** \brief Provide a pointer to the input dataset.
   * \param[in] cloud the const shared pointer to a PointCloud message
   * \param[in] indices the point indices subset that is to be used from \a cloud - if
   * NULL the whole cloud is used
   */
  void
  setInputCloud(const PointCloudConstPtr& cloud,
                const IndicesConstPtr& indices = IndicesConstPtr()) override;
  
  /** \brief Search for k-nearest neighbors for the given query point.
   *
   * \attention This method does not do any bounds checking for the input index
   * (i.e., index >= cloud.size () || index < 0), and assumes valid (i.e., finite) data.
   *
   * \param[in] point a given \a valid (i.e., finite) query point
   * \param[in] k the number of neighbors to search for
   * \param[out] k_indices the resultant indices of the neighboring points
   * \param[out] k_sqr_distances the resultant squared distances to the neighboring
   * points \return number of neighbors found
   *
   * \exception asserts in debug mode if the index is not between 0 and the maximum
   * number of points
   */
  int
  nearestKSearch(const PointT& point,
                 unsigned int k,
                 Indices& k_indices,
                 std::vector<float>& k_sqr_distances) const override;
  
  /** \brief Search for all the nearest neighbors of the query point in a given radius.
   *
   * \attention This method does not do any bounds checking for the input index
   * (i.e., index >= cloud.size () || index < 0), and assumes valid (i.e., finite) data.
   *
   * \param[in] point a given \a valid (i.e., finite) query point
   * \param[in] radius the radius of the sphere bounding all of p_q's neighbors
   * \param[out] k_indices the resultant indices of the neighboring points
   * \param[out] k_sqr_distances the resultant squared distances to the neighboring
   * points \param[in] max_nn if given, bounds the maximum returned neighbors to this
   * value. If \a max_nn is set to 0 or to a number higher than the number of points in
   * the input cloud, all neighbors in \a radius will be returned. Note that nanoflann,
   * internally, fetches all neighbors in radius regardless of the value of \a max_nn.
   * \return number of neighbors found in radius
   *
   * \exception asserts in debug mode if the index is not between 0 and the maximum
   * number of points
   */
  int
  radiusSearch(const PointT& point,
               double radius,
               Indices& k_indices,
               std::vector<float>& k_sqr_distances,
               unsigned int max_nn = 0) const override;

private:
  /** \brief Internal cleanup method. */
  void
  cleanup();

  /** \brief Converts a PointCloud to an Eigen row-major matrix of floats.
   * \param cloud the PointCloud
   */
  void
  convertCloudToArray(const PointCloud& cloud);

  /** \brief Converts a PointCloud with a given set of indices to an Eigen row-major
   * matrix of floats. \param[in] cloud the PointCloud data \param[in] indices the point
   * cloud indices
   */
  void
  convertCloudToArray(const PointCloud& cloud, const Indices& indices);

  /** \brief Class getName method. */
  std::string
  getName() const override
  {
    return ("KdTreeNanoflann");
  }

  /** \brief A FLANN index object. */
  std::shared_ptr<NanoflannIndex> nanoflann_index_;
  
  /** \brief cloud dataset matrix. */
  Matrix cloud_mat_;
  
  /** \brief Mapping between internal and external indices. */
  std::vector<int> index_mapping_;
  
  /** \brief Whether the mapping between internal and external indices is identity */
  bool identity_mapping_;
  
  /** \brief Tree dimensionality (i.e. the number of dimensions per point). */
  int dim_;
  
  /** \brief The total size of the data (either equal to the number of points in the
   * input cloud or to the number of indices - if passed). */
  uindex_t total_nr_points_;
  
  /** \brief The kd-tree search parameters for radius search. */
  ::nanoflann::SearchParams param_radius_;
};
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/kdtree/impl/kdtree_nanoflann.hpp>
#endif
