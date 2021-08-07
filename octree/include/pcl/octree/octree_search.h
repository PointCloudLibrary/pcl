/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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
 * $Id$
 */

#pragma once

#include <pcl/octree/octree_pointcloud.h>
#include <pcl/point_cloud.h>

namespace pcl {
namespace octree {

/** \brief @b Octree pointcloud search class
 * \note This class provides several methods for spatial neighbor search based on octree
 * structure
 * \tparam PointT type of point used in pointcloud
 * \ingroup octree
 * \author Julius Kammerl (julius@kammerl.de)
 */
template <typename PointT,
          typename LeafContainerT = OctreeContainerPointIndices,
          typename BranchContainerT = OctreeContainerEmpty>
class OctreePointCloudSearch
: public OctreePointCloud<PointT, LeafContainerT, BranchContainerT> {
public:
  // public typedefs
  using IndicesPtr = shared_ptr<Indices>;
  using IndicesConstPtr = shared_ptr<const Indices>;

  using PointCloud = pcl::PointCloud<PointT>;
  using PointCloudPtr = typename PointCloud::Ptr;
  using PointCloudConstPtr = typename PointCloud::ConstPtr;

  // Boost shared pointers
  using Ptr =
      shared_ptr<OctreePointCloudSearch<PointT, LeafContainerT, BranchContainerT>>;
  using ConstPtr = shared_ptr<
      const OctreePointCloudSearch<PointT, LeafContainerT, BranchContainerT>>;

  // Eigen aligned allocator
  using AlignedPointTVector = std::vector<PointT, Eigen::aligned_allocator<PointT>>;

  using OctreeT = OctreePointCloud<PointT, LeafContainerT, BranchContainerT>;
  using LeafNode = typename OctreeT::LeafNode;
  using BranchNode = typename OctreeT::BranchNode;

  /** \brief Constructor.
   * \param[in] resolution octree resolution at lowest octree level
   */
  OctreePointCloudSearch(const double resolution)
  : OctreePointCloud<PointT, LeafContainerT, BranchContainerT>(resolution)
  {}

  /** \brief Search for neighbors within a voxel at given point
   * \param[in] point point addressing a leaf node voxel
   * \param[out] point_idx_data the resultant indices of the neighboring voxel points
   * \return "true" if leaf node exist; "false" otherwise
   */
  bool
  voxelSearch(const PointT& point, Indices& point_idx_data);

  /** \brief Search for neighbors within a voxel at given point referenced by a point
   * index
   * \param[in] index the index in input cloud defining the query point
   * \param[out] point_idx_data the resultant indices of the neighboring voxel points
   * \return "true" if leaf node exist; "false" otherwise
   */
  bool
  voxelSearch(uindex_t index, Indices& point_idx_data);

  /** \brief Search for k-nearest neighbors at the query point.
   * \param[in] cloud the point cloud data
   * \param[in] index the index in \a cloud representing the query point
   * \param[in] k the number of neighbors to search for
   * \param[out] k_indices the resultant indices of the neighboring points (must be
   * resized to \a k a priori!)
   * \param[out] k_sqr_distances the resultant squared distances to the neighboring
   * points (must be resized to \a k a priori!)
   * \return number of neighbors found
   */
  inline uindex_t
  nearestKSearch(const PointCloud& cloud,
                 uindex_t index,
                 uindex_t k,
                 Indices& k_indices,
                 std::vector<float>& k_sqr_distances)
  {
    return (nearestKSearch(cloud[index], k, k_indices, k_sqr_distances));
  }

  /** \brief Search for k-nearest neighbors at given query point.
   * \param[in] p_q the given query point
   * \param[in] k the number of neighbors to search for
   * \param[out] k_indices the resultant indices of the neighboring points (must be
   * resized to k a priori!)
   * \param[out] k_sqr_distances  the resultant squared distances to the neighboring
   * points (must be resized to k a priori!)
   * \return number of neighbors found
   */
  uindex_t
  nearestKSearch(const PointT& p_q,
                 uindex_t k,
                 Indices& k_indices,
                 std::vector<float>& k_sqr_distances);

  /** \brief Search for k-nearest neighbors at query point
   * \param[in] index index representing the query point in the dataset given by \a
   * setInputCloud. If indices were given in setInputCloud, index will be the position
   * in the indices vector.
   * \param[in] k the number of neighbors to search for
   * \param[out] k_indices the resultant indices of the neighboring points (must be
   * resized to \a k a priori!)
   * \param[out] k_sqr_distances the resultant squared distances to the neighboring
   * points (must be resized to \a k a priori!)
   * \return number of neighbors found
   */
  uindex_t
  nearestKSearch(uindex_t index,
                 uindex_t k,
                 Indices& k_indices,
                 std::vector<float>& k_sqr_distances);

  /** \brief Search for approx. nearest neighbor at the query point.
   * \param[in] cloud the point cloud data
   * \param[in] query_index the index in \a cloud representing the query point
   * \param[out] result_index the resultant index of the neighbor point
   * \param[out] sqr_distance the resultant squared distance to the neighboring point
   * \return number of neighbors found
   */
  inline void
  approxNearestSearch(const PointCloud& cloud,
                      uindex_t query_index,
                      index_t& result_index,
                      float& sqr_distance)
  {
    return (approxNearestSearch(cloud[query_index], result_index, sqr_distance));
  }

  /** \brief Search for approx. nearest neighbor at the query point.
   * \param[in] p_q the given query point
   * \param[out] result_index the resultant index of the neighbor point
   * \param[out] sqr_distance the resultant squared distance to the neighboring point
   */
  void
  approxNearestSearch(const PointT& p_q, index_t& result_index, float& sqr_distance);

  /** \brief Search for approx. nearest neighbor at the query point.
   * \param[in] query_index index representing the query point in the dataset given by
   * \a setInputCloud. If indices were given in setInputCloud, index will be the
   * position in the indices vector.
   * \param[out] result_index the resultant index of the neighbor point
   * \param[out] sqr_distance the resultant squared distance to the neighboring point
   * \return number of neighbors found
   */
  void
  approxNearestSearch(uindex_t query_index, index_t& result_index, float& sqr_distance);

  /** \brief Search for all neighbors of query point that are within a given radius.
   * \param[in] cloud the point cloud data
   * \param[in] index the index in \a cloud representing the query point
   * \param[in] radius the radius of the sphere bounding all of p_q's neighbors
   * \param[out] k_indices the resultant indices of the neighboring points
   * \param[out] k_sqr_distances the resultant squared distances to the neighboring
   * points
   * \param[in] max_nn if given, bounds the maximum returned neighbors to this value
   * \return number of neighbors found in radius
   */
  uindex_t
  radiusSearch(const PointCloud& cloud,
               uindex_t index,
               double radius,
               Indices& k_indices,
               std::vector<float>& k_sqr_distances,
               index_t max_nn = 0)
  {
    return (radiusSearch(cloud[index], radius, k_indices, k_sqr_distances, max_nn));
  }

  /** \brief Search for all neighbors of query point that are within a given radius.
   * \param[in] p_q the given query point
   * \param[in] radius the radius of the sphere bounding all of p_q's neighbors
   * \param[out] k_indices the resultant indices of the neighboring points
   * \param[out] k_sqr_distances the resultant squared distances to the neighboring
   * points
   * \param[in] max_nn if given, bounds the maximum returned neighbors to this value
   * \return number of neighbors found in radius
   */
  uindex_t
  radiusSearch(const PointT& p_q,
               const double radius,
               Indices& k_indices,
               std::vector<float>& k_sqr_distances,
               uindex_t max_nn = 0) const;

  /** \brief Search for all neighbors of query point that are within a given radius.
   * \param[in] index index representing the query point in the dataset given by \a
   * setInputCloud. If indices were given in setInputCloud, index will be the position
   * in the indices vector
   * \param[in] radius radius of the sphere bounding all of p_q's neighbors
   * \param[out] k_indices the resultant indices of the neighboring points
   * \param[out] k_sqr_distances the resultant squared distances to the neighboring
   * points
   * \param[in] max_nn if given, bounds the maximum returned neighbors to this value
   * \return number of neighbors found in radius
   */
  uindex_t
  radiusSearch(uindex_t index,
               const double radius,
               Indices& k_indices,
               std::vector<float>& k_sqr_distances,
               uindex_t max_nn = 0) const;

  /** \brief Get a PointT vector of centers of all voxels that intersected by a ray
   * (origin, direction).
   * \param[in] origin ray origin
   * \param[in] direction ray direction vector
   * \param[out] voxel_center_list results are written to this vector of PointT elements
   * \param[in] max_voxel_count stop raycasting when this many voxels intersected (0:
   * disable)
   * \return number of intersected voxels
   */
  uindex_t
  getIntersectedVoxelCenters(Eigen::Vector3f origin,
                             Eigen::Vector3f direction,
                             AlignedPointTVector& voxel_center_list,
                             uindex_t max_voxel_count = 0) const;

  /** \brief Get indices of all voxels that are intersected by a ray (origin,
   * direction).
   * \param[in] origin ray origin \param[in] direction ray direction vector
   * \param[out] k_indices resulting point indices from intersected voxels
   * \param[in] max_voxel_count stop raycasting when this many voxels intersected (0:
   * disable)
   * \return number of intersected voxels
   */
  uindex_t
  getIntersectedVoxelIndices(Eigen::Vector3f origin,
                             Eigen::Vector3f direction,
                             Indices& k_indices,
                             uindex_t max_voxel_count = 0) const;

  /** \brief Search for points within rectangular search area
   * Points exactly on the edges of the search rectangle are included.
   * \param[in] min_pt lower corner of search area
   * \param[in] max_pt upper corner of search area
   * \param[out] k_indices the resultant point indices
   * \return number of points found within search area
   */
  uindex_t
  boxSearch(const Eigen::Vector3f& min_pt,
            const Eigen::Vector3f& max_pt,
            Indices& k_indices) const;

protected:
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Octree-based search routines & helpers
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief @b Priority queue entry for branch nodes
   *  \note This class defines priority queue entries for the nearest neighbor search.
   *  \author Julius Kammerl (julius@kammerl.de)
   */
  class prioBranchQueueEntry {
  public:
    /** \brief Empty constructor  */
    prioBranchQueueEntry() : node(), point_distance(0) {}

    /** \brief Constructor for initializing priority queue entry.
     * \param _node pointer to octree node
     * \param _key octree key addressing voxel in octree structure
     * \param[in] _point_distance distance of query point to voxel center
     */
    prioBranchQueueEntry(OctreeNode* _node, OctreeKey& _key, float _point_distance)
    : node(_node), point_distance(_point_distance), key(_key)
    {}

    /** \brief Operator< for comparing priority queue entries with each other.
     * \param[in] rhs the priority queue to compare this against
     */
    bool
    operator<(const prioBranchQueueEntry rhs) const
    {
      return (this->point_distance > rhs.point_distance);
    }

    /** \brief Pointer to octree node. */
    const OctreeNode* node;

    /** \brief Distance to query point. */
    float point_distance;

    /** \brief Octree key. */
    OctreeKey key;
  };

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief @b Priority queue entry for point candidates
   * \note This class defines priority queue entries for the nearest neighbor point
   * candidates.
   * \author Julius Kammerl (julius@kammerl.de)
   */
  class prioPointQueueEntry {
  public:
    /** \brief Empty constructor  */
    prioPointQueueEntry() : point_idx_(0), point_distance_(0) {}

    /** \brief Constructor for initializing priority queue entry.
     * \param[in] point_idx index for a dataset point given by \a setInputCloud
     * \param[in] point_distance distance of query point to voxel center
     */
    prioPointQueueEntry(uindex_t point_idx, float point_distance)
    : point_idx_(point_idx), point_distance_(point_distance)
    {}

    /** \brief Operator< for comparing priority queue entries with each other.
     * \param[in] rhs priority queue to compare this against
     */
    bool
    operator<(const prioPointQueueEntry& rhs) const
    {
      return (this->point_distance_ < rhs.point_distance_);
    }

    /** \brief Index representing a point in the dataset given by \a setInputCloud. */
    uindex_t point_idx_;

    /** \brief Distance to query point. */
    float point_distance_;
  };

  /** \brief Helper function to calculate the squared distance between two points
   * \param[in] point_a point A
   * \param[in] point_b point B
   * \return squared distance between point A and point B
   */
  float
  pointSquaredDist(const PointT& point_a, const PointT& point_b) const;

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Recursive search routine methods
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  /** \brief Recursive search method that explores the octree and finds neighbors within
   * a given radius
   * \param[in] point query point \param[in] radiusSquared squared search radius
   * \param[in] node current octree node to be explored
   * \param[in] key octree key addressing a leaf node.
   * \param[in] tree_depth current depth/level in the octree
   * \param[out] k_indices vector of indices found to be neighbors of query point
   * \param[out] k_sqr_distances squared distances of neighbors to query point
   * \param[in] max_nn maximum of neighbors to be found
   */
  void
  getNeighborsWithinRadiusRecursive(const PointT& point,
                                    const double radiusSquared,
                                    const BranchNode* node,
                                    const OctreeKey& key,
                                    uindex_t tree_depth,
                                    Indices& k_indices,
                                    std::vector<float>& k_sqr_distances,
                                    uindex_t max_nn) const;

  /** \brief Recursive search method that explores the octree and finds the K nearest
   * neighbors
   * \param[in] point query point
   * \param[in] K amount of nearest neighbors to be found
   * \param[in] node current octree node to be explored
   * \param[in] key octree key addressing a leaf node.
   * \param[in] tree_depth current depth/level in the octree
   * \param[in] squared_search_radius squared search radius distance
   * \param[out] point_candidates priority queue of nearest neigbor point candidates
   * \return squared search radius based on current point candidate set found
   */
  double
  getKNearestNeighborRecursive(
      const PointT& point,
      uindex_t K,
      const BranchNode* node,
      const OctreeKey& key,
      uindex_t tree_depth,
      const double squared_search_radius,
      std::vector<prioPointQueueEntry>& point_candidates) const;

  /** \brief Recursive search method that explores the octree and finds the approximate
   * nearest neighbor
   * \param[in] point query point
   * \param[in] node current octree node to be explored
   * \param[in] key octree key addressing a leaf node.
   * \param[in] tree_depth current depth/level in the octree
   * \param[out] result_index result index is written to this reference
   * \param[out] sqr_distance squared distance to search
   */
  void
  approxNearestSearchRecursive(const PointT& point,
                               const BranchNode* node,
                               const OctreeKey& key,
                               uindex_t tree_depth,
                               index_t& result_index,
                               float& sqr_distance);

  /** \brief Recursively search the tree for all intersected leaf nodes and return a
   * vector of voxel centers. This algorithm is based off the paper An Efficient
   * Parametric Algorithm for Octree Traversal:
   * http://wscg.zcu.cz/wscg2000/Papers_2000/X31.pdf
   * \param[in] min_x octree nodes X coordinate of lower bounding box corner
   * \param[in] min_y octree nodes Y coordinate of lower bounding box corner
   * \param[in] min_z octree nodes Z coordinate of lower bounding box corner
   * \param[in] max_x octree nodes X coordinate of upper bounding box corner
   * \param[in] max_y octree nodes Y coordinate of upper bounding box corner
   * \param[in] max_z octree nodes Z coordinate of upper bounding box corner
   * \param[in] a
   * \param[in] node current octree node to be explored
   * \param[in] key octree key addressing a leaf node.
   * \param[out] voxel_center_list results are written to this vector of PointT elements
   * \param[in] max_voxel_count stop raycasting when this many voxels intersected (0:
   * disable)
   * \return number of voxels found
   */
  uindex_t
  getIntersectedVoxelCentersRecursive(double min_x,
                                      double min_y,
                                      double min_z,
                                      double max_x,
                                      double max_y,
                                      double max_z,
                                      unsigned char a,
                                      const OctreeNode* node,
                                      const OctreeKey& key,
                                      AlignedPointTVector& voxel_center_list,
                                      uindex_t max_voxel_count) const;

  /** \brief Recursive search method that explores the octree and finds points within a
   * rectangular search area
   * \param[in] min_pt lower corner of search area
   * \param[in] max_pt upper corner of search area
   * \param[in] node current octree node to be explored
   * \param[in] key octree key addressing a leaf node.
   * \param[in] tree_depth current depth/level in the octree
   * \param[out] k_indices the resultant point indices
   */
  void
  boxSearchRecursive(const Eigen::Vector3f& min_pt,
                     const Eigen::Vector3f& max_pt,
                     const BranchNode* node,
                     const OctreeKey& key,
                     uindex_t tree_depth,
                     Indices& k_indices) const;

  /** \brief Recursively search the tree for all intersected leaf nodes and return a
   * vector of indices. This algorithm is based off the paper An Efficient Parametric
   * Algorithm for Octree Traversal: http://wscg.zcu.cz/wscg2000/Papers_2000/X31.pdf
   * \param[in] min_x octree nodes X coordinate of lower bounding box corner
   * \param[in] min_y octree nodes Y coordinate of lower bounding box corner
   * \param[in] min_z octree nodes Z coordinate of lower bounding box corner
   * \param[in] max_x octree nodes X coordinate of upper bounding box corner
   * \param[in] max_y octree nodes Y coordinate of upper bounding box corner
   * \param[in] max_z octree nodes Z coordinate of upper bounding box corner
   * \param[in] a
   * \param[in] node current octree node to be explored
   * \param[in] key octree key addressing a leaf node.
   * \param[out] k_indices resulting indices
   * \param[in] max_voxel_count stop raycasting when this many voxels intersected (0:
   * disable)
   * \return number of voxels found
   */
  uindex_t
  getIntersectedVoxelIndicesRecursive(double min_x,
                                      double min_y,
                                      double min_z,
                                      double max_x,
                                      double max_y,
                                      double max_z,
                                      unsigned char a,
                                      const OctreeNode* node,
                                      const OctreeKey& key,
                                      Indices& k_indices,
                                      uindex_t max_voxel_count) const;

  /** \brief Initialize raytracing algorithm
   * \param origin
   * \param direction
   * \param[in] min_x octree nodes X coordinate of lower bounding box corner
   * \param[in] min_y octree nodes Y coordinate of lower bounding box corner
   * \param[in] min_z octree nodes Z coordinate of lower bounding box corner
   * \param[in] max_x octree nodes X coordinate of upper bounding box corner
   * \param[in] max_y octree nodes Y coordinate of upper bounding box corner
   * \param[in] max_z octree nodes Z coordinate of upper bounding box corner
   * \param a
   */
  inline void
  initIntersectedVoxel(Eigen::Vector3f& origin,
                       Eigen::Vector3f& direction,
                       double& min_x,
                       double& min_y,
                       double& min_z,
                       double& max_x,
                       double& max_y,
                       double& max_z,
                       unsigned char& a) const
  {
    // Account for division by zero when direction vector is 0.0
    const float epsilon = 1e-10f;
    if (direction.x() == 0.0)
      direction.x() = epsilon;
    if (direction.y() == 0.0)
      direction.y() = epsilon;
    if (direction.z() == 0.0)
      direction.z() = epsilon;

    // Voxel childIdx remapping
    a = 0;

    // Handle negative axis direction vector
    if (direction.x() < 0.0) {
      origin.x() = static_cast<float>(this->min_x_) + static_cast<float>(this->max_x_) -
                   origin.x();
      direction.x() = -direction.x();
      a |= 4;
    }
    if (direction.y() < 0.0) {
      origin.y() = static_cast<float>(this->min_y_) + static_cast<float>(this->max_y_) -
                   origin.y();
      direction.y() = -direction.y();
      a |= 2;
    }
    if (direction.z() < 0.0) {
      origin.z() = static_cast<float>(this->min_z_) + static_cast<float>(this->max_z_) -
                   origin.z();
      direction.z() = -direction.z();
      a |= 1;
    }
    min_x = (this->min_x_ - origin.x()) / direction.x();
    max_x = (this->max_x_ - origin.x()) / direction.x();
    min_y = (this->min_y_ - origin.y()) / direction.y();
    max_y = (this->max_y_ - origin.y()) / direction.y();
    min_z = (this->min_z_ - origin.z()) / direction.z();
    max_z = (this->max_z_ - origin.z()) / direction.z();
  }

  /** \brief Find first child node ray will enter
   * \param[in] min_x octree nodes X coordinate of lower bounding box corner
   * \param[in] min_y octree nodes Y coordinate of lower bounding box corner
   * \param[in] min_z octree nodes Z coordinate of lower bounding box corner
   * \param[in] mid_x octree nodes X coordinate of bounding box mid line
   * \param[in] mid_y octree nodes Y coordinate of bounding box mid line
   * \param[in] mid_z octree nodes Z coordinate of bounding box mid line
   * \return the first child node ray will enter
   */
  inline int
  getFirstIntersectedNode(double min_x,
                          double min_y,
                          double min_z,
                          double mid_x,
                          double mid_y,
                          double mid_z) const
  {
    int currNode = 0;

    if (min_x > min_y) {
      if (min_x > min_z) {
        // max(min_x, min_y, min_z) is min_x. Entry plane is YZ.
        if (mid_y < min_x)
          currNode |= 2;
        if (mid_z < min_x)
          currNode |= 1;
      }
      else {
        // max(min_x, min_y, min_z) is min_z. Entry plane is XY.
        if (mid_x < min_z)
          currNode |= 4;
        if (mid_y < min_z)
          currNode |= 2;
      }
    }
    else {
      if (min_y > min_z) {
        // max(min_x, min_y, min_z) is min_y. Entry plane is XZ.
        if (mid_x < min_y)
          currNode |= 4;
        if (mid_z < min_y)
          currNode |= 1;
      }
      else {
        // max(min_x, min_y, min_z) is min_z. Entry plane is XY.
        if (mid_x < min_z)
          currNode |= 4;
        if (mid_y < min_z)
          currNode |= 2;
      }
    }

    return currNode;
  }

  /** \brief Get the next visited node given the current node upper
   *   bounding box corner. This function accepts three float values, and
   *   three int values. The function returns the ith integer where the
   *   ith float value is the minimum of the three float values.
   * \param[in] x current nodes X coordinate of upper bounding box corner
   * \param[in] y current nodes Y coordinate of upper bounding box corner
   * \param[in] z current nodes Z coordinate of upper bounding box corner
   * \param[in] a next node if exit Plane YZ
   * \param[in] b next node if exit Plane XZ
   * \param[in] c next node if exit Plane XY
   * \return the next child node ray will enter or 8 if exiting
   */
  inline int
  getNextIntersectedNode(double x, double y, double z, int a, int b, int c) const
  {
    if (x < y) {
      if (x < z)
        return a;
      return c;
    }
    if (y < z)
      return b;
    return c;
  }
};
} // namespace octree
} // namespace pcl

#ifdef PCL_NO_PRECOMPILE
#include <pcl/octree/impl/octree_search.hpp>
#endif
