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
 * kdtree_flann.h
 * Adapted from: kdtree/include/pcl/kdtree/kdtree_flann.h
 * Created on: Jun 01, 2022
 * Author: Ramzi Sabra
 */

#pragma once

#include <pcl/cuda/kdtree/kdtree.h>
#include <flann/util/params.h>

#include <memory>

// Forward declarations
namespace flann
{
  template <typename T> struct L2_Simple;
  template <typename T> class KDTreeCuda3dIndex;
}

namespace pcl {
namespace cuda {
namespace flann {
namespace detail {
// Helper struct to create a compatible Matrix and copy data back when needed
// Replace using if constexpr in C++17
template <typename IndexT>
struct compat_with_flann : std::false_type {};

template <>
struct compat_with_flann<int> : std::true_type {};

template <typename IndexT>
using CompatWithFlann = std::enable_if_t<compat_with_flann<IndexT>::value, bool>;
template <typename IndexT>
using NotCompatWithFlann = std::enable_if_t<!compat_with_flann<IndexT>::value, bool>;
} //namespace detail

/**
 * @brief Compatibility template function to allow use of various types of indices with
 * FLANN
 * @details Template is used for all params to not constrain any FLANN side capability
 * @param[in,out] index A index searcher, of type ::flann::KDTreeCuda3dIndex<Dist> or
 * similar, where Dist is a template for computing distance between 2 points
 * @param[in] queries A ::flann::Matrix<float> or compatible matrix representation of the
 * query point
 * @param[out] indices Indices found in radius for each query point
 * @param[out] dists Computed distance matrix
 * @param[in] radius Threshold for consideration
 * @param[in] params Any parameters to pass to the radius_search call
 */
template <class FlannIndex,
          class Queries,
          class Indices,
          class Distances,
          class SearchParams>
int
radius_search(const FlannIndex& index,
              const Queries& queries,
              Indices& indices,
              Distances& dists,
              float radius,
              const SearchParams& params);

/**
 * @brief Compatibility template function to allow use of various types of indices with
 * FLANN
 * @details Template is used for all params to not constrain any FLANN side capability
 * @param[in,out] index A index searcher, of type ::flann::Index<Dist> or similar, where
 * Dist is a template for computing distance between 2 points
 * @param[in] query A ::flann::Matrix<float> or compatible matrix representation of the
 * query point
 * @param[out] indices Neighboring k indices found for each query point
 * @param[out] dists Computed distance matrix
 * @param[in] k Number of neighbors to search for
 * @param[in] params Any parameters to pass to the knn_search call
 */
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
           const SearchParams& params);
} //namespace flann

/** \brief KdTreeFLANN is a generic type of 3D spatial locator using kD-tree structures.
 * The class is making use of the FLANN (Fast Library for Approximate Nearest Neighbor)
 * project by Marius Muja and David Lowe.
 *
 * \author Ramzi Sabra
 * \ingroup cuda/kdtree
 */
template <typename PointT, typename Dist = ::flann::L2_Simple<float>>
class KdTreeFLANN : public pcl::cuda::KdTree<PointT> {
  using Base = cuda::KdTree<PointT>;

protected:
  using Base::input_;
  using Base::indices_;
  using Base::max_leaf_size_;
  using Base::epsilon_;
  using Base::sorted_;
  using Base::point_representation_;

public:
  using PointCloud = typename Base::PointCloud;
  using PointCloudConstPtr = typename Base::PointCloudConstPtr;

  using PointRepresentation = pcl::PointRepresentation<PointT>;
  using PointRepresentationConstPtr = typename PointRepresentation::ConstPtr;

  using IndicesPtr = shared_ptr<Indices>;
  using IndicesConstPtr = shared_ptr<const Indices>;
  
  using PointVector = typename Base::PointVector;
  using IndexMatrix = typename Base::IndexMatrix;
  using DistanceMatrix = typename Base::DistanceMatrix;

  using IndicesVector = typename Base::IndicesVector;
  using DistancesVector = typename Base::DistancesVector;

  using FLANNIndex = ::flann::KDTreeCuda3dIndex<Dist>;

  using Tree = KdTreeFLANN<PointT, Dist>;

  // Shared pointers
  using Ptr = shared_ptr<Tree>;
  using ConstPtr = shared_ptr<const Tree>;

  /** \brief Default Constructor for KdTreeFLANN.
   * \param[in] sorted set to true if the application that the tree will be used for
   * requires sorted nearest neighbor indices (default). False otherwise.
   * \param[in] max_leaf_size maximum leaf node size. Set to 64 by default.
   *
   * By setting sorted to false, the \ref radiusSearch operations will be faster.
   */
  KdTreeFLANN(bool sorted = true, int max_leaf_size = 64);

  /** \brief Copy constructor
   * \param[in] k the tree to copy into this
   */
  KdTreeFLANN(const Tree& k);

  /** \brief Copy operator
   * \param[in] k the tree to copy into this
   */
  inline Tree&
  operator=(const Tree& k)
  {
    cuda::KdTree<PointT>::operator=(k);
    flann_index_ = k.flann_index_;
    cloud_ = k.cloud_;
    index_mapping_ = k.index_mapping_;
    identity_mapping_ = k.identity_mapping_;
    total_nr_points_ = k.total_nr_points_;
    param_k_ = k.param_k_;
    param_radius_ = k.param_radius_;
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

  /** \brief Destructor for KdTreeFLANN.
   * Deletes all allocated data arrays and destroys the kd-tree structures.
   */
  ~KdTreeFLANN() { cleanup(); }

  /** \brief Provide a pointer to the input dataset.
   * \param[in] cloud the const shared pointer to a PointCloud message
   * \param[in] indices the point indices subset that is to be used from \a cloud - if
   * NULL the whole cloud is used
   */
  void
  setInputCloud(const PointCloudConstPtr& cloud,
                const IndicesConstPtr& indices = IndicesConstPtr()) override;

  /** \brief Search for k-nearest neighbors for each of the given query points.
   *
   * \attention This method does not do any bounds checking for the input index
   * (i.e., index >= cloud.size () || index < 0), and assumes valid (i.e., finite) data.
   *
   * \param[in] points a given \a valid (i.e., finite) query points vector
   * \param[in] k the number of neighbors to search for
   * \param[out] k_indices_mat the resultant indices matrix of the neighboring points of each query
   * point \param[out] k_sqr_distances_mat the resultant matrix of squared distances to the
   * neighboring points of each query point \return number of neighbors found
   *
   * \exception asserts in debug mode if the index is not between 0 and the maximum
   * number of points
   */

  /** \brief Provide a pointer to the point representation to use to convert points into k-D vectors.
    * \param[in] point_representation the const shared pointer to a PointRepresentation
    */
  inline void
  setPointRepresentation (const PointRepresentationConstPtr &point_representation) override;

  int
  nearestKSearch(const PointVector& points,
                 unsigned int k,
                 IndexMatrix& k_indices_mat,
                 DistanceMatrix& k_sqr_distances_mat) const override;
  
  /** \brief Search for k-nearest neighbors for each of the given query points.
   *
   * \attention This method does not do any bounds checking for the input index
   * (i.e., index >= cloud.size () || index < 0), and assumes valid (i.e., finite) data.
   *
   * \param[in] points a given \a valid (i.e., finite) query points vector
   * \param[in] k the number of neighbors to search for
   * \param[out] k_indices_vec the resultant indices vector of the neighboring points of each query
   * point \param[out] k_sqr_distances_vec the resultant vector of squared distances to the
   * neighboring points of each query point \return number of neighbors found
   *
   * \exception asserts in debug mode if the index is not between 0 and the maximum
   * number of points
   */
  int
  nearestKSearch(const PointVector& points,
                 unsigned int k,
                 IndicesVector& k_indices_vec,
                 DistancesVector& k_sqr_distances_vec) const override;

  /** \brief Search for all the nearest neighbors of each of the query points in a given
   * radius.
   * \attention This method does not do any bounds checking for the input index
   * (i.e., index >= cloud.size () || index < 0), and assumes valid (i.e., finite) data.
   * 
   * \param[in] points a given \a valid (i.e., finite) query points vector
   * \param[in] radius the radius of the spheres bounding each query point's neighbors
   * \param[out] indices_mat the resultant matrix of indices of the neighboring points of
   * each query point \param[out] sqr_distances_mat the resultant matrix of squared
   * distances to the neighboring points \param[in] max_nn if given, bounds the maximum
   * returned neighbors for each query point to this value. If \a max_nn is set to 0 or
   * to a number higher than the number of points in the input cloud, all neighbors in
   * \a radius will be returned. \return number of neighbors found in radius
   *
   * \exception asserts in debug mode if the index is not between 0 and the maximum
   * number of points
   */
  int
  radiusSearch(const PointVector& points,
               double radius,
               IndexMatrix& indices_mat,
               DistanceMatrix& sqr_distances_mat,
               unsigned int max_nn = 0) const override;
  
  /** \brief Search for all the nearest neighbors of each of the query points in a given
   * radius.
   * \attention This method does not do any bounds checking for the input index
   * (i.e., index >= cloud.size () || index < 0), and assumes valid (i.e., finite) data.
   * 
   * \param[in] points a given \a valid (i.e., finite) query points vector
   * \param[in] radius the radius of the spheres bounding each query point's neighbors
   * \param[out] indices_vec the resultant vector of indices of the neighboring points of
   * each query point \param[out] sqr_distances_vec the resultant vector of squared
   * distances to the neighboring points \param[in] max_nn if given, bounds the maximum
   * returned neighbors for each query point to this value. If \a max_nn is set to 0 or
   * to a number higher than the number of points in the input cloud, all neighbors in
   * \a radius will be returned. \return number of neighbors found in radius
   *
   * \exception asserts in debug mode if the index is not between 0 and the maximum
   * number of points
   */
  int
  radiusSearch(const PointVector& points,
               double radius,
               IndicesVector& indices_vec,
               DistancesVector& sqr_distances_vec,
               unsigned int max_nn = 0) const override;

private:
  /** \brief Internal cleanup method. */
  void
  cleanup();

  /** \brief Converts a PointCloud to the internal FLANN point array representation.
   * Returns the number of points. \param cloud the PointCloud
   */
  void
  convertCloudToArray(const PointCloud& cloud);

  /** \brief Converts a PointCloud with a given set of indices to the internal FLANN
   * point array representation. Returns the number of points. \param[in] cloud the
   * PointCloud data \param[in] indices the point cloud indices
   */
  void
  convertCloudToArray(const PointCloud& cloud, const Indices& indices);

private:
  /** \brief Class getName method. */
  std::string
  getName() const override
  {
    return ("cuda::KdTreeFLANN");
  }

  /** \brief A FLANN index object. */
  std::shared_ptr<FLANNIndex> flann_index_;

  /** \brief Internal pointer to data. TODO: replace with std::shared_ptr<float[]> with
   * C++17*/
  std::shared_ptr<float> cloud_;

  /** \brief mapping between internal and external indices. */
  std::vector<int> index_mapping_;

  /** \brief whether the mapping between internal and external indices is identity */
  bool identity_mapping_;

  /** \brief Tree dimensionality (i.e. the number of dimensions per point) - fixed to 3. */
  constexpr static int dim_ = 3;

  /** \brief The total size of the data (either equal to the number of points in the
   * input cloud or to the number of indices - if passed). */
  uindex_t total_nr_points_;

  /** \brief The KdTree search parameters for K-nearest neighbors. */
  ::flann::SearchParams param_k_;

  /** \brief The KdTree search parameters for radius search. */
  ::flann::SearchParams param_radius_;
  };
} //namespace cuda
} //namespace pcl

#ifdef PCL_NO_PRECOMPILE
#include <pcl/cuda/kdtree/impl/kdtree_flann.hpp>
#endif
