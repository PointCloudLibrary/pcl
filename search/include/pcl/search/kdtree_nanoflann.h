/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2025-, Open Perception Inc.
 *
 *  All rights reserved
 */

#pragma once

#include <pcl/pcl_config.h>
#if PCL_HAS_NANOFLANN || defined(DOXYGEN_ONLY)

#include <pcl/search/kdtree.h>
#include <pcl/point_representation.h>

#include <nanoflann.hpp>

namespace pcl {
namespace search {
namespace internal {
/** Helper for KdTreeNanoflann, serves as a dataset adaptor.
 */
template <typename T = float>
struct PointCloudAdaptor {
  PointCloudAdaptor(const T* data,
                    bool delete_data,
                    std::size_t dim,
                    std::size_t point_count)
  : data_(data), delete_data_(delete_data), dim_(dim), point_count_(point_count)
  {}

  ~PointCloudAdaptor()
  {
    if (delete_data_)
      delete[] data_;
  }

  inline void
  reset_adaptor(const T* data,
                bool delete_data,
                std::size_t dim,
                std::size_t point_count)
  {
    if (delete_data_)
      delete[] data_;
    data_ = data;
    delete_data_ = delete_data;
    dim_ = dim;
    point_count_ = point_count;
  }

  inline std::size_t
  kdtree_get_point_count() const
  {
    return point_count_;
  }

  inline T
  kdtree_get_pt(const std::size_t idx, const std::size_t dim) const
  {
    return data_[dim_ * idx + dim];
  }

  template <class BBOX>
  bool
  kdtree_get_bbox(BBOX& /* bb */) const
  {
    return false;
  }

private:
  const T* data_;
  bool delete_data_;
  std::size_t dim_;
  std::size_t point_count_;
};

/** Helper function for radiusSearch: must have a template specialization if the
 * distance norm is L2
 */
template <typename Dist>
float
square_if_l2(float radius)
{
  return radius;
};
template <>
float
square_if_l2<nanoflann::L2_Adaptor<float,
                                   pcl::search::internal::PointCloudAdaptor<float>,
                                   float,
                                   pcl::index_t>>(float radius)
{
  return radius * radius;
};
template <>
float
square_if_l2<
    nanoflann::L2_Simple_Adaptor<float,
                                 pcl::search::internal::PointCloudAdaptor<float>,
                                 float,
                                 pcl::index_t>>(float radius)
{
  return radius * radius;
};
} // namespace internal

// for convenience/brevity
using L1_Adaptor =
    nanoflann::L1_Adaptor<float,
                          pcl::search::internal::PointCloudAdaptor<float>,
                          float,
                          pcl::index_t>;
using L2_Adaptor =
    nanoflann::L2_Adaptor<float,
                          pcl::search::internal::PointCloudAdaptor<float>,
                          float,
                          pcl::index_t>;
using L2_Simple_Adaptor =
    nanoflann::L2_Simple_Adaptor<float,
                                 pcl::search::internal::PointCloudAdaptor<float>,
                                 float,
                                 pcl::index_t>;
using SO2_Adaptor =
    nanoflann::SO2_Adaptor<float,
                           pcl::search::internal::PointCloudAdaptor<float>,
                           float,
                           pcl::index_t>;
using SO3_Adaptor =
    nanoflann::SO3_Adaptor<float,
                           pcl::search::internal::PointCloudAdaptor<float>,
                           float,
                           pcl::index_t>;
// TODO maybe additional adaptor with simd?

/** @brief @b pcl::search::KdTreeNanoflann is a faster and flexible alternative to
 * pcl::search::KdTree. It is based on the nanoflann library by Jose Luis Blanco-Claraco
 * ( https://github.com/jlblancoc/nanoflann ). Since nanoflann is treated as an optional
 * dependency to PCL, you must test the preprocessor symbol `PCL_HAS_NANOFLANN` after
 * including `pcl/search/kdtree_nanoflann.h`.
 * @code
 * // Example 1: using KdTreeNanoflann in NormalEstimation:
 * pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
 * auto tree = pcl::make_shared<pcl::search::KdTreeNanoflann<pcl::PointXYZ>>();
 * ne.setSearchMethod (tree);
 * // rest as usual
 * // Example 2: using KdTreeNanoflannn in ICP:
 * pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
 * icp.setSearchMethodSource(pcl::make_shared<pcl::search::KdTreeNanoflann<pcl::PointXYZ>>());
 * icp.setSearchMethodTarget(pcl::make_shared<pcl::search::KdTreeNanoflann<pcl::PointXYZ>>());
 * // rest as usual
 * // Example 3: using a L1 distance norm:
 * pcl::search::KdTreeNanoflann<PointXYZ, 3, pcl::search::L1_Adaptor> kdtree_nanoflann;
 * kdtree_nanoflann.setInputCloud (my_cloud);
 * // now ready to use kdtree_nanoflann.nearestKSearch(...) and
 * // kdtree_nanoflann.radiusSearch(...);
 * // Example 4: using KdTreeNanoflann with features instead of 3D points:
 * pcl::search::KdTreeNanoflann<pcl::FPFHSignature33, 33> kdtree_nanoflann_feat;
 * kdtree_nanoflann_feat.setInputCloud(my_features_cloud);
 * // now ready for searching
 * @endcode
 *
 * @tparam PointT Can be a point like pcl::PointXYZ or a feature like pcl::SHOT352
 * @tparam Dim Dimension of the KdTree to build. If set to -1, it will infer the
 * dimension at runtime from the given point representation, but with a performance
 * penalty. Default is 3, appropriate for all xyz point types.
 * @tparam Distance The distance to use. Possible values include pcl::L1_Adaptor
 * (taxicab/manhattan distance), pcl::L2_Simple_Adaptor (euclidean distance, optimized
 * for few dimensions), pcl::L2_Adaptor (euclidean distance, optimized for many
 * dimensions), pcl::SO2_Adaptor, pcl::SO3_Adaptor. Default is pcl::L2_Simple_Adaptor
 *
 * @author Markus Vieth
 * @ingroup search
 */
template <typename PointT,
          std::int32_t Dim = 3,
          typename Distance =
              std::conditional_t<Dim >= 4, L2_Adaptor, L2_Simple_Adaptor>,
          typename Tree = nanoflann::KDTreeSingleIndexAdaptor<
              Distance,
              pcl::search::internal::PointCloudAdaptor<float>,
              Dim,
              pcl::index_t>>
class KdTreeNanoflann : public pcl::search::KdTree<PointT> {
private:
  /** The special thing here is that indices and distances are stored in _two_ vectors,
   * not as a pair in _one_ vector.
   */
  template <typename _DistanceType = float, typename _IndexType = pcl::index_t>
  class PCLRadiusResultSet {
  public:
    using DistanceType = _DistanceType;
    using IndexType = _IndexType;

  public:
    const DistanceType radius;

    std::vector<IndexType>& m_indices;
    std::vector<DistanceType>& m_dists;
    std::size_t max_results_;

    explicit PCLRadiusResultSet(
        DistanceType radius_,
        std::vector<IndexType>& indices,
        std::vector<DistanceType>& dists,
        std::size_t max_results = std::numeric_limits<std::size_t>::max())
    : radius(radius_), m_indices(indices), m_dists(dists), max_results_(max_results)
    {
      init();
    }

    void
    init()
    {
      clear();
      if (max_results_ != std::numeric_limits<std::size_t>::max()) {
        m_indices.reserve(max_results_);
        m_dists.reserve(max_results_);
      }
    }
    void
    clear()
    {
      m_indices.clear();
      m_dists.clear();
    }

    size_t
    size() const
    {
      return m_indices.size();
    }
    size_t
    empty() const
    {
      return m_indices.empty();
    }

    bool
    full() const
    {
      return true;
    }

    /**
     * Called during search to add an element matching the criteria.
     * @return true if the search should be continued, false if the results are
     * sufficient
     */
    bool
    addPoint(DistanceType dist, IndexType index)
    {
      if (dist < radius) {
        m_indices.emplace_back(index);
        m_dists.emplace_back(dist);
      }
      return (m_indices.size() < max_results_);
    }

    DistanceType
    worstDist() const
    {
      return radius;
    }

    void
    sort()
    {
      // this sort function does not do anything (yet). For now, PCLRadiusResultSet
      // should only be used if the results do not have to be sorted. In the future, we
      // might add sorting here if we find a really efficient way to sort both vectors
      // simultaneously.
    }
  };

public:
  using PointCloud = typename Search<PointT>::PointCloud;
  using PointCloudConstPtr = typename pcl::PointCloud<PointT>::ConstPtr;

  using pcl::search::Search<PointT>::indices_;
  using pcl::search::Search<PointT>::input_;
  using pcl::search::Search<PointT>::getIndices;
  using pcl::search::Search<PointT>::getInputCloud;
  using pcl::search::Search<PointT>::nearestKSearch;
  using pcl::search::Search<PointT>::radiusSearch;
  using pcl::search::Search<PointT>::sorted_results_;
  using pcl::search::Search<PointT>::name_;

  using Ptr = shared_ptr<KdTreeNanoflann<PointT, Dim, Distance, Tree>>;
  using ConstPtr = shared_ptr<const KdTreeNanoflann<PointT, Dim, Distance, Tree>>;

  using KdTreePtr = shared_ptr<Tree>;
  using KdTreeConstPtr = shared_ptr<const Tree>;
  using PointRepresentationConstPtr = shared_ptr<const PointRepresentation<PointT>>;

  /** @brief Constructor for KdTreeNanoflann.
   *
   * @param[in] sorted Set to true if the nearest neighbor search results
   * need to be sorted in ascending order based on their distance to the
   * query point (default is false, which is faster)
   * @param[in] leaf_max_size Max number of points/samples in a leaf, can be tuned to
   * achieve best performance. When using this for 1nn search (e.g. correspondence
   * estimation in icp/registration), then leaf_max_size=10 is usually fastest.
   * Otherwise, leaf_max_size=20 is a good choice.
   * @param[in] n_thread_build Number of threads for concurrent tree build (default is
   * 1, meaning no concurrent build)
   */
  KdTreeNanoflann(bool sorted,
                  std::size_t leaf_max_size,
                  unsigned int n_thread_build = 1)
  : pcl::search::KdTree<PointT>("KdTreeNanoflann", sorted)
  , leaf_max_size_(leaf_max_size)
  , n_thread_build_(n_thread_build)
  {}

  /** @brief Constructor for KdTreeNanoflann.
   *
   * @param[in] sorted set to true if the nearest neighbor search results
   * need to be sorted in ascending order based on their distance to the
   * query point
   *
   */
  KdTreeNanoflann(bool sorted = false) : KdTreeNanoflann(sorted, 15, 1) {}

  /** @brief Destructor for KdTreeNanoflann. */
  ~KdTreeNanoflann() override = default;

  /** @brief Provide a pointer to the point representation to use to convert points into
   * k-D vectors. If you want to use this function, it is recommended to do so _before_
   * calling setInputCloud, to avoid setting up the kd-tree twice.
   * @param[in] point_representation the const shared pointer to a PointRepresentation
   */
  void
  setPointRepresentation(const PointRepresentationConstPtr& point_representation)
  {
    PCL_DEBUG("[KdTreeNanoflann::setPointRepresentation] "
              "KdTreeNanoflann::setPointRepresentation called, "
              "point_representation->getNumberOfDimensions()=%i, "
              "point_representation->isTrivial()=%i\n",
              point_representation->getNumberOfDimensions(),
              point_representation->isTrivial());
    if (Dim != -1 && Dim != point_representation->getNumberOfDimensions()) {
      PCL_ERROR("[KdTreeNanoflann::setPointRepresentation] The given point "
                "representation uses %i dimensions, but the nr of dimensions given as "
                "template parameter to KdTreeNanoflann is %i.\n",
                point_representation->getNumberOfDimensions(),
                Dim);
      return;
    }
    point_representation_ = point_representation;
    setUpTree();
  }

  /** @brief Get a pointer to the point representation used when converting points into
   * k-D vectors. */
  inline PointRepresentationConstPtr
  getPointRepresentation() const
  {
    return point_representation_;
  }

  /** @brief Sets whether the results have to be sorted or not.
   * @param[in] sorted_results set to true if the radius search results should be sorted
   */
  void
  setSortedResults(bool sorted_results) override
  {
    sorted_results_ = sorted_results;
  }

  /** @brief Set the search epsilon precision (error bound) for nearest neighbors
   * searches.
   * @param[in] eps precision (error bound) for nearest neighbors searches
   */
  void
  setEpsilon(float eps)
  {
    eps_ = eps;
  }

  /** @brief Get the search epsilon precision (error bound) for nearest neighbors
   * searches. */
  inline float
  getEpsilon() const
  {
    return eps_;
  }

  /** @brief Influences the results of radiusSearch when max_nn is not set to zero.
   * If the parameter max_nn of radiusSearch is set to a value greater than zero, it
   * will return _at most_ that many points. If you set `use_rknn=true`, it will always
   * return the points _closest_ to the query point. If you set `use_rknn=false`, it may
   * return _any_ points within the radius, which can be faster.
   */
  void
  setUseRKNN(bool use_rknn)
  {
    use_rknn_ = use_rknn;
  }

  /** @brief Provide a pointer to the input dataset, this will build the kd-tree.
   * @param[in] cloud the const shared pointer to a PointCloud
   * @param[in] indices the point indices subset that is to be used from \a cloud
   */
  bool
  setInputCloud(const PointCloudConstPtr& cloud,
                const IndicesConstPtr& indices = IndicesConstPtr()) override
  {
    input_ = cloud;
    indices_ = indices;

    return setUpTree();
  }

  /** @brief Get pointer to internal nanoflann tree. Use with caution.
   */
  KdTreePtr
  getNanoflannTree()
  {
    return nanoflann_tree_;
  }

  /** @brief Search for the k-nearest neighbors for the given query point.
   * @param[in] point the given query point
   * @param[in] k the number of neighbors to search for
   * @param[out] k_indices the resultant indices of the neighboring points (must be
   * resized to \a k a priori!)
   * @param[out] k_sqr_distances the resultant squared distances to the neighboring
   * points (must be resized to \a k a priori!)
   * @return number of neighbors found
   */
  int
  nearestKSearch(const PointT& point,
                 int k,
                 Indices& k_indices,
                 std::vector<float>& k_sqr_distances) const override
  {
    assert(point_representation_->isValid(point) &&
           "Invalid (NaN, Inf) point coordinates given to nearestKSearch!");
    if (static_cast<std::size_t>(k) > adaptor_->kdtree_get_point_count())
      k = adaptor_->kdtree_get_point_count();
    k_indices.resize(k);
    k_sqr_distances.resize(k);
    float* query_point = nullptr;
    if (!point_representation_->isTrivial()) {
      query_point = new float[point_representation_->getNumberOfDimensions()];
      point_representation_->vectorize(point, query_point);
    }
    // like nanoflann_tree_->knnSearch
    nanoflann::KNNResultSet<float, pcl::index_t> resultSet(k);
    resultSet.init(k_indices.data(), k_sqr_distances.data());
    nanoflann_tree_->findNeighbors(
        resultSet,
        (query_point ? query_point : reinterpret_cast<const float*>(&point))
#if NANOFLANN_VERSION < 0x150
            ,
        nanoflann::SearchParams()
#endif // NANOFLANN_VERSION < 0x150
    );
    const auto search_result = resultSet.size();
    delete[] query_point;
    assert(search_result == k);

    if (!identity_mapping_) {
      for (auto& index : k_indices) {
        index = index_mapping_[index];
      }
    }
    return search_result;
  }

  /** @brief Search for all the nearest neighbors of the query point in a given radius.
   * @param[in] point the given query point
   * @param[in] radius the radius of the sphere bounding all of p_q's neighbors
   * @param[out] k_indices the resultant indices of the neighboring points
   * @param[out] k_sqr_distances the resultant squared distances to the neighboring
   * points
   * @param[in] max_nn if given, bounds the maximum returned neighbors to this value. If
   * \a max_nn is set to 0 or to a number higher than the number of points in the input
   * cloud, all neighbors in \a radius will be returned.
   * @return number of neighbors found in radius
   */
  int
  radiusSearch(const PointT& point,
               double radius,
               Indices& k_indices,
               std::vector<float>& k_sqr_distances,
               unsigned int max_nn = 0) const override
  {
    assert(point_representation_->isValid(point) &&
           "Invalid (NaN, Inf) point coordinates given to radiusSearch!");
    float* query_point = nullptr;
    if (!point_representation_->isTrivial()) {
      query_point = new float[point_representation_->getNumberOfDimensions()];
      point_representation_->vectorize(point, query_point);
    }
    if (max_nn == 0) {
      // return _all_ points within radius
      // Calling this->sortResults is currently slower than sorting the vector of
      // nanoflann::ResultItem However, if sorted results are not requested, using
      // PCLRadiusResultSet with radiusSearchCustomCallback avoids copies and is faster
      if (sorted_results_) {
#if NANOFLANN_VERSION < 0x150
        std::vector<std::pair<pcl::index_t, float>>
            IndicesDists; // nanoflann::ResultItem was introduced in version 1.5.0
#else
        std::vector<nanoflann::ResultItem<pcl::index_t, float>> IndicesDists;
#endif // NANOFLANN_VERSION < 0x150
       // like nanoflann_tree_->radiusSearch
        nanoflann::RadiusResultSet<float, pcl::index_t> resultSet(
            pcl::search::internal::square_if_l2<Distance>(radius), IndicesDists);
        nanoflann_tree_->findNeighbors(
            resultSet,
            (query_point ? query_point : reinterpret_cast<const float*>(&point)),
#if NANOFLANN_VERSION < 0x150
            {32, eps_, sorted_results_} // first parameter is ignored in older versions,
                                        // and removed in newer versions
#else
            {eps_, sorted_results_}
#endif // NANOFLANN_VERSION < 0x150
        );
#if NANOFLANN_VERSION < 0x160
        // before, nanoflann did not sort in findNeighbours
        if (sorted_results_)
          std::sort(
              IndicesDists.begin(), IndicesDists.end(), nanoflann::IndexDist_Sorter());
#endif // NANOFLANN_VERSION < 0x160
        const auto search_result = resultSet.size();
        k_indices.resize(IndicesDists.size());
        k_sqr_distances.resize(IndicesDists.size());
        for (std::size_t i = 0; i < IndicesDists.size(); ++i) {
          k_indices[i] = identity_mapping_ ? IndicesDists[i].first
                                           : index_mapping_[IndicesDists[i].first];
          k_sqr_distances[i] = IndicesDists[i].second;
        }
        delete[] query_point;
        return search_result;
      }
      else {
        PCLRadiusResultSet<float, pcl::index_t> resultSet(
            pcl::search::internal::square_if_l2<Distance>(radius),
            k_indices,
            k_sqr_distances);
        // like nanoflann_tree_->radiusSearchCustomCallback
        nanoflann_tree_->findNeighbors(
            resultSet,
            (query_point ? query_point : reinterpret_cast<const float*>(&point)),
#if NANOFLANN_VERSION < 0x150
            {32, eps_, sorted_results_} // first parameter is ignored in older versions,
                                        // and removed in newer versions
#else
            {eps_, sorted_results_}
#endif // NANOFLANN_VERSION < 0x150
        );
        const auto search_result = resultSet.size();
        if (!identity_mapping_) {
          for (auto& index : k_indices) {
            index = index_mapping_[index];
          }
        }
        if (sorted_results_)
          this->sortResults(k_indices, k_sqr_distances);
        delete[] query_point;
        return search_result;
      }
    }
    else {
      // return no more than max_nn points
      if (use_rknn_) {
        // return points closest to query point
        if (static_cast<std::size_t>(max_nn) > adaptor_->kdtree_get_point_count())
          max_nn = adaptor_->kdtree_get_point_count();
        k_indices.resize(max_nn);
        k_sqr_distances.resize(max_nn);
#if NANOFLANN_VERSION < 0x151
        // rknnSearch and RKNNResultSet were added in nanoflann 1.5.1, so do knn search
        // and discard those neighbors that are outside of search radius
        nanoflann::KNNResultSet<float, pcl::index_t> resultSet(max_nn);
        resultSet.init(k_indices.data(), k_sqr_distances.data());
        nanoflann_tree_->findNeighbors(
            resultSet,
            (query_point ? query_point : reinterpret_cast<const float*>(&point))
#if NANOFLANN_VERSION < 0x150
                ,
            nanoflann::SearchParams()
#endif // NANOFLANN_VERSION < 0x150
        );
        auto search_result = resultSet.size();
        for (auto iter = k_sqr_distances.rbegin();
             iter != k_sqr_distances.rend() &&
             *iter > pcl::search::internal::square_if_l2<Distance>(radius);
             ++iter) {
          --search_result;
        }
        k_indices.resize(search_result);
        k_sqr_distances.resize(search_result);
#else
        // like nanoflann_tree_->rknnSearch
        nanoflann::RKNNResultSet<float, pcl::index_t> resultSet(
            max_nn, pcl::search::internal::square_if_l2<Distance>(radius));
        resultSet.init(k_indices.data(), k_sqr_distances.data());
        nanoflann_tree_->findNeighbors(
            resultSet,
            (query_point ? query_point : reinterpret_cast<const float*>(&point)));
        const auto search_result = resultSet.size();
        k_indices.resize(search_result);
        k_sqr_distances.resize(search_result);
#endif // NANOFLANN_VERSION < 0x151
        if (!identity_mapping_) {
          for (auto& index : k_indices) {
            index = index_mapping_[index];
          }
        }
        delete[] query_point;
        return search_result;
      }
      else {
        // may return any points within the radius, can be faster
        PCLRadiusResultSet<float, pcl::index_t> resultSet(
            pcl::search::internal::square_if_l2<Distance>(radius),
            k_indices,
            k_sqr_distances,
            max_nn);
        // like nanoflann_tree_->radiusSearchCustomCallback
        nanoflann_tree_->findNeighbors(
            resultSet,
            (query_point ? query_point : reinterpret_cast<const float*>(&point)),
#if NANOFLANN_VERSION < 0x150
            {32, eps_, sorted_results_} // first parameter is ignored in older versions,
                                        // and removed in newer versions
#else
            {eps_, sorted_results_}
#endif // NANOFLANN_VERSION < 0x150
        );
        const auto search_result = resultSet.size();
        if (!identity_mapping_) {
          for (auto& index : k_indices) {
            index = index_mapping_[index];
          }
        }
        if (sorted_results_)
          this->sortResults(k_indices, k_sqr_distances);
        delete[] query_point;
        return search_result;
      }
    }
  }

private:
  /** Set up index_mapping_, adaptor_, and nanoflann_tree_, based on
   * point_representation_, input_, and indices_.
   */
  bool
  setUpTree()
  {
    if (!point_representation_ || !input_ || input_->empty()) {
      PCL_ERROR("[KdTreeNanoflann::setUpTree] point representation is null or input is "
                "null or input is empty\n");
      return false;
    }

    const std::size_t dim = point_representation_->getNumberOfDimensions();
    if (Dim != -1 && Dim != static_cast<int>(dim)) {
      PCL_ERROR("[KdTreeNanoflann::setUpTree] The given point "
                "representation uses %i dimensions, but the nr of dimensions given as "
                "template parameter to KdTreeNanoflann is %i.\n",
                dim,
                Dim);
      return false;
    }
    index_mapping_.clear();
    if (indices_ && !indices_->empty()) {
      index_mapping_.reserve(indices_->size());
      for (const auto& index : *indices_) {
        if (point_representation_->isValid((*input_)[index])) {
          index_mapping_.emplace_back(index);
        }
      }
    }
    else {
      index_mapping_.reserve(input_->size());
      for (std::size_t index = 0; index < input_->size(); ++index) {
        if (point_representation_->isValid((*input_)[index])) {
          index_mapping_.emplace_back(index);
        }
      }
    }
    identity_mapping_ = true;
    for (pcl::index_t index = 0;
         identity_mapping_ && static_cast<std::size_t>(index) < index_mapping_.size();
         ++index) {
      identity_mapping_ = identity_mapping_ && index_mapping_[index] == index;
    }

    PCL_DEBUG("[KdTreeNanoflann::setUpTree] identity_mapping_=%i, "
              "point_representation_->getNumberOfDimensions()=%lu, "
              "point_representation_->isTrivial ()=%i\n",
              identity_mapping_ ? 1 : 0,
              dim,
              point_representation_->isTrivial() ? 1 : 0);
    if (identity_mapping_ && point_representation_->isTrivial() &&
        sizeof(PointT) % sizeof(float) == 0) {
      // no need to allocate memory and copy data
      PCL_DEBUG("[KdTreeNanoflann::setUpTree] Mapping cloud directly, without "
                "allocating memory.\n");
      adaptor_.reset(new internal::PointCloudAdaptor<float>(
          reinterpret_cast<const float*>(&(*input_)[0]),
          false,
          sizeof(PointT) / sizeof(float),
          index_mapping_.size()));
    }
    else {
      float* data = new float[dim * index_mapping_.size()];
      float* data_itr = data;
      for (auto& index : index_mapping_) {
        point_representation_->vectorize((*input_)[index], data_itr);
        data_itr += dim;
      }
      adaptor_.reset(new internal::PointCloudAdaptor<float>(
          data, true, dim, index_mapping_.size()));
    }
    nanoflann_tree_.reset(
        new Tree(dim,
                 *adaptor_,
#if NANOFLANN_VERSION >= 0x150
                 // concurrent tree building is possible since nanoflann 1.5.0
                 {leaf_max_size_,
                  nanoflann::KDTreeSingleIndexAdaptorFlags::None,
                  n_thread_build_}));
#else
                 {leaf_max_size_}));
    if (n_thread_build_ != 1)
      PCL_WARN("[KdTreeNanoflann::setUpTree] concurrent tree building is only possible "
               "with nanoflann version 1.5.0 and newer\n");
#endif // NANOFLANN_VERSION >= 0x150
    return true;
  }

  KdTreePtr nanoflann_tree_;

  shared_ptr<internal::PointCloudAdaptor<float>> adaptor_;

  PointRepresentationConstPtr point_representation_{
      new DefaultPointRepresentation<PointT>};

  Indices index_mapping_;

  bool identity_mapping_{false};

  std::size_t leaf_max_size_{15};

  unsigned int n_thread_build_{1};

  float eps_{0.0f};

  bool use_rknn_{true};
};
} // namespace search
} // namespace pcl
#ifdef PCL_NO_PRECOMPILE
#include <pcl/search/impl/kdtree.hpp>
#endif

#else
//#warning "KdTreeNanoflann is not available"
#endif // PCL_HAS_NANOFLANN
