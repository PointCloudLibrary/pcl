/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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

#ifndef PCL_OCTREE_SEARCH_IMPL_H_
#define PCL_OCTREE_SEARCH_IMPL_H_

#include <cassert>

namespace pcl {

namespace octree {

template <typename PointT, typename LeafContainerT, typename BranchContainerT>
bool
OctreePointCloudSearch<PointT, LeafContainerT, BranchContainerT>::voxelSearch(
    const PointT& point, Indices& point_idx_data)
{
  assert(isFinite(point) &&
         "Invalid (NaN, Inf) point coordinates given to nearestKSearch!");
  OctreeKey key;
  bool b_success = false;

  // generate key
  this->genOctreeKeyforPoint(point, key);

  LeafContainerT* leaf = this->findLeaf(key);

  if (leaf) {
    (*leaf).getPointIndices(point_idx_data);
    b_success = true;
  }

  return (b_success);
}

template <typename PointT, typename LeafContainerT, typename BranchContainerT>
bool
OctreePointCloudSearch<PointT, LeafContainerT, BranchContainerT>::voxelSearch(
    const uindex_t index, Indices& point_idx_data)
{
  const PointT search_point = this->getPointByIndex(index);
  return (this->voxelSearch(search_point, point_idx_data));
}

template <typename PointT, typename LeafContainerT, typename BranchContainerT>
uindex_t
OctreePointCloudSearch<PointT, LeafContainerT, BranchContainerT>::nearestKSearch(
    const PointT& p_q,
    uindex_t k,
    Indices& k_indices,
    std::vector<float>& k_sqr_distances)
{
  assert(this->leaf_count_ > 0);
  assert(isFinite(p_q) &&
         "Invalid (NaN, Inf) point coordinates given to nearestKSearch!");

  k_indices.clear();
  k_sqr_distances.clear();

  if (k < 1)
    return 0;

  prioPointQueueEntry point_entry;
  std::vector<prioPointQueueEntry> point_candidates;

  OctreeKey key;
  key.x = key.y = key.z = 0;

  // initialize smallest point distance in search with high value
  double smallest_dist = std::numeric_limits<double>::max();

  getKNearestNeighborRecursive(
      p_q, k, this->root_node_, key, 1, smallest_dist, point_candidates);

  const auto result_count = static_cast<uindex_t>(point_candidates.size());

  k_indices.resize(result_count);
  k_sqr_distances.resize(result_count);

  for (uindex_t i = 0; i < result_count; ++i) {
    k_indices[i] = point_candidates[i].point_idx_;
    k_sqr_distances[i] = point_candidates[i].point_distance_;
  }

  return k_indices.size();
}

template <typename PointT, typename LeafContainerT, typename BranchContainerT>
uindex_t
OctreePointCloudSearch<PointT, LeafContainerT, BranchContainerT>::nearestKSearch(
    uindex_t index, uindex_t k, Indices& k_indices, std::vector<float>& k_sqr_distances)
{
  const PointT search_point = this->getPointByIndex(index);
  return (nearestKSearch(search_point, k, k_indices, k_sqr_distances));
}

template <typename PointT, typename LeafContainerT, typename BranchContainerT>
void
OctreePointCloudSearch<PointT, LeafContainerT, BranchContainerT>::approxNearestSearch(
    const PointT& p_q, index_t& result_index, float& sqr_distance)
{
  assert(this->leaf_count_ > 0);
  assert(isFinite(p_q) &&
         "Invalid (NaN, Inf) point coordinates given to nearestKSearch!");

  OctreeKey key;
  key.x = key.y = key.z = 0;

  approxNearestSearchRecursive(
      p_q, this->root_node_, key, 1, result_index, sqr_distance);

  return;
}

template <typename PointT, typename LeafContainerT, typename BranchContainerT>
void
OctreePointCloudSearch<PointT, LeafContainerT, BranchContainerT>::approxNearestSearch(
    uindex_t query_index, index_t& result_index, float& sqr_distance)
{
  const PointT search_point = this->getPointByIndex(query_index);

  return (approxNearestSearch(search_point, result_index, sqr_distance));
}

template <typename PointT, typename LeafContainerT, typename BranchContainerT>
uindex_t
OctreePointCloudSearch<PointT, LeafContainerT, BranchContainerT>::radiusSearch(
    const PointT& p_q,
    const double radius,
    Indices& k_indices,
    std::vector<float>& k_sqr_distances,
    uindex_t max_nn) const
{
  assert(isFinite(p_q) &&
         "Invalid (NaN, Inf) point coordinates given to nearestKSearch!");
  OctreeKey key;
  key.x = key.y = key.z = 0;

  k_indices.clear();
  k_sqr_distances.clear();

  getNeighborsWithinRadiusRecursive(p_q,
                                    radius * radius,
                                    this->root_node_,
                                    key,
                                    1,
                                    k_indices,
                                    k_sqr_distances,
                                    max_nn);

  return k_indices.size();
}

template <typename PointT, typename LeafContainerT, typename BranchContainerT>
uindex_t
OctreePointCloudSearch<PointT, LeafContainerT, BranchContainerT>::radiusSearch(
    uindex_t index,
    const double radius,
    Indices& k_indices,
    std::vector<float>& k_sqr_distances,
    uindex_t max_nn) const
{
  const PointT search_point = this->getPointByIndex(index);

  return (radiusSearch(search_point, radius, k_indices, k_sqr_distances, max_nn));
}

template <typename PointT, typename LeafContainerT, typename BranchContainerT>
uindex_t
OctreePointCloudSearch<PointT, LeafContainerT, BranchContainerT>::boxSearch(
    const Eigen::Vector3f& min_pt,
    const Eigen::Vector3f& max_pt,
    Indices& k_indices) const
{

  OctreeKey key;
  key.x = key.y = key.z = 0;

  k_indices.clear();

  boxSearchRecursive(min_pt, max_pt, this->root_node_, key, 1, k_indices);

  return k_indices.size();
}

template <typename PointT, typename LeafContainerT, typename BranchContainerT>
double
OctreePointCloudSearch<PointT, LeafContainerT, BranchContainerT>::
    getKNearestNeighborRecursive(
        const PointT& point,
        uindex_t K,
        const BranchNode* node,
        const OctreeKey& key,
        uindex_t tree_depth,
        const double squared_search_radius,
        std::vector<prioPointQueueEntry>& point_candidates) const
{
  std::vector<prioBranchQueueEntry> search_heap;
  search_heap.resize(8);

  OctreeKey new_key;

  double smallest_squared_dist = squared_search_radius;

  // get spatial voxel information
  double voxelSquaredDiameter = this->getVoxelSquaredDiameter(tree_depth);

  // iterate over all children
  for (unsigned char child_idx = 0; child_idx < 8; child_idx++) {
    if (this->branchHasChild(*node, child_idx)) {
      PointT voxel_center;

      search_heap[child_idx].key.x = (key.x << 1) + (!!(child_idx & (1 << 2)));
      search_heap[child_idx].key.y = (key.y << 1) + (!!(child_idx & (1 << 1)));
      search_heap[child_idx].key.z = (key.z << 1) + (!!(child_idx & (1 << 0)));

      // generate voxel center point for voxel at key
      this->genVoxelCenterFromOctreeKey(
          search_heap[child_idx].key, tree_depth, voxel_center);

      // generate new priority queue element
      search_heap[child_idx].node = this->getBranchChildPtr(*node, child_idx);
      search_heap[child_idx].point_distance = pointSquaredDist(voxel_center, point);
    }
    else {
      search_heap[child_idx].point_distance = std::numeric_limits<float>::infinity();
    }
  }

  std::sort(search_heap.begin(), search_heap.end());

  // iterate over all children in priority queue
  // check if the distance to search candidate is smaller than the best point distance
  // (smallest_squared_dist)
  while ((!search_heap.empty()) &&
         (search_heap.back().point_distance <
          smallest_squared_dist + voxelSquaredDiameter / 4.0 +
              sqrt(smallest_squared_dist * voxelSquaredDiameter) - this->epsilon_)) {
    const OctreeNode* child_node;

    // read from priority queue element
    child_node = search_heap.back().node;
    new_key = search_heap.back().key;

    if (child_node->getNodeType() == BRANCH_NODE) {
      // we have not reached maximum tree depth
      smallest_squared_dist =
          getKNearestNeighborRecursive(point,
                                       K,
                                       static_cast<const BranchNode*>(child_node),
                                       new_key,
                                       tree_depth + 1,
                                       smallest_squared_dist,
                                       point_candidates);
    }
    else {
      // we reached leaf node level
      Indices decoded_point_vector;

      const LeafNode* child_leaf = static_cast<const LeafNode*>(child_node);

      // decode leaf node into decoded_point_vector
      (*child_leaf)->getPointIndices(decoded_point_vector);

      // Linearly iterate over all decoded (unsorted) points
      for (const auto& point_index : decoded_point_vector) {

        const PointT& candidate_point = this->getPointByIndex(point_index);

        // calculate point distance to search point
        float squared_dist = pointSquaredDist(candidate_point, point);

        // check if a closer match is found
        if (squared_dist < smallest_squared_dist) {
          prioPointQueueEntry point_entry;

          point_entry.point_distance_ = squared_dist;
          point_entry.point_idx_ = point_index;
          point_candidates.push_back(point_entry);
        }
      }

      std::sort(point_candidates.begin(), point_candidates.end());

      if (point_candidates.size() > K)
        point_candidates.resize(K);

      if (point_candidates.size() == K)
        smallest_squared_dist = point_candidates.back().point_distance_;
    }
    // pop element from priority queue
    search_heap.pop_back();
  }

  return (smallest_squared_dist);
}

template <typename PointT, typename LeafContainerT, typename BranchContainerT>
void
OctreePointCloudSearch<PointT, LeafContainerT, BranchContainerT>::
    getNeighborsWithinRadiusRecursive(const PointT& point,
                                      const double radiusSquared,
                                      const BranchNode* node,
                                      const OctreeKey& key,
                                      uindex_t tree_depth,
                                      Indices& k_indices,
                                      std::vector<float>& k_sqr_distances,
                                      uindex_t max_nn) const
{
  // get spatial voxel information
  double voxel_squared_diameter = this->getVoxelSquaredDiameter(tree_depth);

  // iterate over all children
  for (unsigned char child_idx = 0; child_idx < 8; child_idx++) {
    if (!this->branchHasChild(*node, child_idx))
      continue;

    const OctreeNode* child_node;
    child_node = this->getBranchChildPtr(*node, child_idx);

    OctreeKey new_key;
    PointT voxel_center;
    float squared_dist;

    // generate new key for current branch voxel
    new_key.x = (key.x << 1) + (!!(child_idx & (1 << 2)));
    new_key.y = (key.y << 1) + (!!(child_idx & (1 << 1)));
    new_key.z = (key.z << 1) + (!!(child_idx & (1 << 0)));

    // generate voxel center point for voxel at key
    this->genVoxelCenterFromOctreeKey(new_key, tree_depth, voxel_center);

    // calculate distance to search point
    squared_dist = pointSquaredDist(static_cast<const PointT&>(voxel_center), point);

    // if distance is smaller than search radius
    if (squared_dist + this->epsilon_ <=
        voxel_squared_diameter / 4.0 + radiusSquared +
            sqrt(voxel_squared_diameter * radiusSquared)) {

      if (child_node->getNodeType() == BRANCH_NODE) {
        // we have not reached maximum tree depth
        getNeighborsWithinRadiusRecursive(point,
                                          radiusSquared,
                                          static_cast<const BranchNode*>(child_node),
                                          new_key,
                                          tree_depth + 1,
                                          k_indices,
                                          k_sqr_distances,
                                          max_nn);
        if (max_nn != 0 && k_indices.size() == max_nn)
          return;
      }
      else {
        // we reached leaf node level
        const LeafNode* child_leaf = static_cast<const LeafNode*>(child_node);
        Indices decoded_point_vector;

        // decode leaf node into decoded_point_vector
        (*child_leaf)->getPointIndices(decoded_point_vector);

        // Linearly iterate over all decoded (unsorted) points
        for (const auto& index : decoded_point_vector) {
          const PointT& candidate_point = this->getPointByIndex(index);

          // calculate point distance to search point
          squared_dist = pointSquaredDist(candidate_point, point);

          // check if a match is found
          if (squared_dist > radiusSquared)
            continue;

          // add point to result vector
          k_indices.push_back(index);
          k_sqr_distances.push_back(squared_dist);

          if (max_nn != 0 && k_indices.size() == max_nn)
            return;
        }
      }
    }
  }
}

template <typename PointT, typename LeafContainerT, typename BranchContainerT>
void
OctreePointCloudSearch<PointT, LeafContainerT, BranchContainerT>::
    approxNearestSearchRecursive(const PointT& point,
                                 const BranchNode* node,
                                 const OctreeKey& key,
                                 uindex_t tree_depth,
                                 index_t& result_index,
                                 float& sqr_distance)
{
  OctreeKey minChildKey;
  OctreeKey new_key;

  const OctreeNode* child_node;

  // set minimum voxel distance to maximum value
  double min_voxel_center_distance = std::numeric_limits<double>::max();

  unsigned char min_child_idx = 0xFF;

  // iterate over all children
  for (unsigned char child_idx = 0; child_idx < 8; child_idx++) {
    if (!this->branchHasChild(*node, child_idx))
      continue;

    PointT voxel_center;
    double voxelPointDist;

    new_key.x = (key.x << 1) + (!!(child_idx & (1 << 2)));
    new_key.y = (key.y << 1) + (!!(child_idx & (1 << 1)));
    new_key.z = (key.z << 1) + (!!(child_idx & (1 << 0)));

    // generate voxel center point for voxel at key
    this->genVoxelCenterFromOctreeKey(new_key, tree_depth, voxel_center);

    voxelPointDist = pointSquaredDist(voxel_center, point);

    // search for child voxel with shortest distance to search point
    if (voxelPointDist >= min_voxel_center_distance)
      continue;

    min_voxel_center_distance = voxelPointDist;
    min_child_idx = child_idx;
    minChildKey = new_key;
  }

  // make sure we found at least one branch child
  assert(min_child_idx < 8);

  child_node = this->getBranchChildPtr(*node, min_child_idx);

  if (child_node->getNodeType() == BRANCH_NODE) {
    // we have not reached maximum tree depth
    approxNearestSearchRecursive(point,
                                 static_cast<const BranchNode*>(child_node),
                                 minChildKey,
                                 tree_depth + 1,
                                 result_index,
                                 sqr_distance);
  }
  else {
    // we reached leaf node level
    Indices decoded_point_vector;

    const LeafNode* child_leaf = static_cast<const LeafNode*>(child_node);

    float smallest_squared_dist = std::numeric_limits<float>::max();

    // decode leaf node into decoded_point_vector
    (**child_leaf).getPointIndices(decoded_point_vector);

    // Linearly iterate over all decoded (unsorted) points
    for (const auto& index : decoded_point_vector) {
      const PointT& candidate_point = this->getPointByIndex(index);

      // calculate point distance to search point
      float squared_dist = pointSquaredDist(candidate_point, point);

      // check if a closer match is found
      if (squared_dist >= smallest_squared_dist)
        continue;

      result_index = index;
      smallest_squared_dist = squared_dist;
      sqr_distance = squared_dist;
    }
  }
}

template <typename PointT, typename LeafContainerT, typename BranchContainerT>
float
OctreePointCloudSearch<PointT, LeafContainerT, BranchContainerT>::pointSquaredDist(
    const PointT& point_a, const PointT& point_b) const
{
  return (point_a.getVector3fMap() - point_b.getVector3fMap()).squaredNorm();
}

template <typename PointT, typename LeafContainerT, typename BranchContainerT>
void
OctreePointCloudSearch<PointT, LeafContainerT, BranchContainerT>::boxSearchRecursive(
    const Eigen::Vector3f& min_pt,
    const Eigen::Vector3f& max_pt,
    const BranchNode* node,
    const OctreeKey& key,
    uindex_t tree_depth,
    Indices& k_indices) const
{
  // iterate over all children
  for (unsigned char child_idx = 0; child_idx < 8; child_idx++) {

    const OctreeNode* child_node;
    child_node = this->getBranchChildPtr(*node, child_idx);

    if (!child_node)
      continue;

    OctreeKey new_key;
    // generate new key for current branch voxel
    new_key.x = (key.x << 1) + (!!(child_idx & (1 << 2)));
    new_key.y = (key.y << 1) + (!!(child_idx & (1 << 1)));
    new_key.z = (key.z << 1) + (!!(child_idx & (1 << 0)));

    // voxel corners
    Eigen::Vector3f lower_voxel_corner;
    Eigen::Vector3f upper_voxel_corner;
    // get voxel coordinates
    this->genVoxelBoundsFromOctreeKey(
        new_key, tree_depth, lower_voxel_corner, upper_voxel_corner);

    // test if search region overlap with voxel space

    if (!((lower_voxel_corner(0) > max_pt(0)) || (min_pt(0) > upper_voxel_corner(0)) ||
          (lower_voxel_corner(1) > max_pt(1)) || (min_pt(1) > upper_voxel_corner(1)) ||
          (lower_voxel_corner(2) > max_pt(2)) || (min_pt(2) > upper_voxel_corner(2)))) {

      if (child_node->getNodeType() == BRANCH_NODE) {
        // we have not reached maximum tree depth
        boxSearchRecursive(min_pt,
                           max_pt,
                           static_cast<const BranchNode*>(child_node),
                           new_key,
                           tree_depth + 1,
                           k_indices);
      }
      else {
        // we reached leaf node level
        Indices decoded_point_vector;

        const LeafNode* child_leaf = static_cast<const LeafNode*>(child_node);

        // decode leaf node into decoded_point_vector
        (**child_leaf).getPointIndices(decoded_point_vector);

        // Linearly iterate over all decoded (unsorted) points
        for (const auto& index : decoded_point_vector) {
          const PointT& candidate_point = this->getPointByIndex(index);

          // check if point falls within search box
          bool bInBox =
              ((candidate_point.x >= min_pt(0)) && (candidate_point.x <= max_pt(0)) &&
               (candidate_point.y >= min_pt(1)) && (candidate_point.y <= max_pt(1)) &&
               (candidate_point.z >= min_pt(2)) && (candidate_point.z <= max_pt(2)));

          if (bInBox)
            // add to result vector
            k_indices.push_back(index);
        }
      }
    }
  }
}

template <typename PointT, typename LeafContainerT, typename BranchContainerT>
uindex_t
OctreePointCloudSearch<PointT, LeafContainerT, BranchContainerT>::
    getIntersectedVoxelCenters(Eigen::Vector3f origin,
                               Eigen::Vector3f direction,
                               AlignedPointTVector& voxel_center_list,
                               uindex_t max_voxel_count) const
{
  OctreeKey key;
  key.x = key.y = key.z = 0;

  voxel_center_list.clear();

  // Voxel child_idx remapping
  unsigned char a = 0;

  double min_x, min_y, min_z, max_x, max_y, max_z;

  initIntersectedVoxel(origin, direction, min_x, min_y, min_z, max_x, max_y, max_z, a);

  if (std::max(std::max(min_x, min_y), min_z) < std::min(std::min(max_x, max_y), max_z))
    return getIntersectedVoxelCentersRecursive(min_x,
                                               min_y,
                                               min_z,
                                               max_x,
                                               max_y,
                                               max_z,
                                               a,
                                               this->root_node_,
                                               key,
                                               voxel_center_list,
                                               max_voxel_count);

  return (0);
}

template <typename PointT, typename LeafContainerT, typename BranchContainerT>
uindex_t
OctreePointCloudSearch<PointT, LeafContainerT, BranchContainerT>::
    getIntersectedVoxelIndices(Eigen::Vector3f origin,
                               Eigen::Vector3f direction,
                               Indices& k_indices,
                               uindex_t max_voxel_count) const
{
  OctreeKey key;
  key.x = key.y = key.z = 0;

  k_indices.clear();

  // Voxel child_idx remapping
  unsigned char a = 0;
  double min_x, min_y, min_z, max_x, max_y, max_z;

  initIntersectedVoxel(origin, direction, min_x, min_y, min_z, max_x, max_y, max_z, a);

  if (std::max(std::max(min_x, min_y), min_z) < std::min(std::min(max_x, max_y), max_z))
    return getIntersectedVoxelIndicesRecursive(min_x,
                                               min_y,
                                               min_z,
                                               max_x,
                                               max_y,
                                               max_z,
                                               a,
                                               this->root_node_,
                                               key,
                                               k_indices,
                                               max_voxel_count);
  return (0);
}

template <typename PointT, typename LeafContainerT, typename BranchContainerT>
uindex_t
OctreePointCloudSearch<PointT, LeafContainerT, BranchContainerT>::
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
                                        uindex_t max_voxel_count) const
{
  if (max_x < 0.0 || max_y < 0.0 || max_z < 0.0)
    return (0);

  // If leaf node, get voxel center and increment intersection count
  if (node->getNodeType() == LEAF_NODE) {
    PointT newPoint;

    this->genLeafNodeCenterFromOctreeKey(key, newPoint);

    voxel_center_list.push_back(newPoint);

    return (1);
  }

  // Voxel intersection count for branches children
  uindex_t voxel_count = 0;

  // Voxel mid lines
  double mid_x = 0.5 * (min_x + max_x);
  double mid_y = 0.5 * (min_y + max_y);
  double mid_z = 0.5 * (min_z + max_z);

  // First voxel node ray will intersect
  auto curr_node = getFirstIntersectedNode(min_x, min_y, min_z, mid_x, mid_y, mid_z);

  // Child index, node and key
  unsigned char child_idx;
  OctreeKey child_key;

  do {
    if (curr_node != 0)
      child_idx = static_cast<unsigned char>(curr_node ^ a);
    else
      child_idx = a;

    // child_node == 0 if child_node doesn't exist
    const OctreeNode* child_node =
        this->getBranchChildPtr(static_cast<const BranchNode&>(*node), child_idx);

    // Generate new key for current branch voxel
    child_key.x = (key.x << 1) | (!!(child_idx & (1 << 2)));
    child_key.y = (key.y << 1) | (!!(child_idx & (1 << 1)));
    child_key.z = (key.z << 1) | (!!(child_idx & (1 << 0)));

    // Recursively call each intersected child node, selecting the next
    //   node intersected by the ray.  Children that do not intersect will
    //   not be traversed.

    switch (curr_node) {
    case 0:
      if (child_node)
        voxel_count += getIntersectedVoxelCentersRecursive(min_x,
                                                           min_y,
                                                           min_z,
                                                           mid_x,
                                                           mid_y,
                                                           mid_z,
                                                           a,
                                                           child_node,
                                                           child_key,
                                                           voxel_center_list,
                                                           max_voxel_count);
      curr_node = getNextIntersectedNode(mid_x, mid_y, mid_z, 4, 2, 1);
      break;

    case 1:
      if (child_node)
        voxel_count += getIntersectedVoxelCentersRecursive(min_x,
                                                           min_y,
                                                           mid_z,
                                                           mid_x,
                                                           mid_y,
                                                           max_z,
                                                           a,
                                                           child_node,
                                                           child_key,
                                                           voxel_center_list,
                                                           max_voxel_count);
      curr_node = getNextIntersectedNode(mid_x, mid_y, max_z, 5, 3, 8);
      break;

    case 2:
      if (child_node)
        voxel_count += getIntersectedVoxelCentersRecursive(min_x,
                                                           mid_y,
                                                           min_z,
                                                           mid_x,
                                                           max_y,
                                                           mid_z,
                                                           a,
                                                           child_node,
                                                           child_key,
                                                           voxel_center_list,
                                                           max_voxel_count);
      curr_node = getNextIntersectedNode(mid_x, max_y, mid_z, 6, 8, 3);
      break;

    case 3:
      if (child_node)
        voxel_count += getIntersectedVoxelCentersRecursive(min_x,
                                                           mid_y,
                                                           mid_z,
                                                           mid_x,
                                                           max_y,
                                                           max_z,
                                                           a,
                                                           child_node,
                                                           child_key,
                                                           voxel_center_list,
                                                           max_voxel_count);
      curr_node = getNextIntersectedNode(mid_x, max_y, max_z, 7, 8, 8);
      break;

    case 4:
      if (child_node)
        voxel_count += getIntersectedVoxelCentersRecursive(mid_x,
                                                           min_y,
                                                           min_z,
                                                           max_x,
                                                           mid_y,
                                                           mid_z,
                                                           a,
                                                           child_node,
                                                           child_key,
                                                           voxel_center_list,
                                                           max_voxel_count);
      curr_node = getNextIntersectedNode(max_x, mid_y, mid_z, 8, 6, 5);
      break;

    case 5:
      if (child_node)
        voxel_count += getIntersectedVoxelCentersRecursive(mid_x,
                                                           min_y,
                                                           mid_z,
                                                           max_x,
                                                           mid_y,
                                                           max_z,
                                                           a,
                                                           child_node,
                                                           child_key,
                                                           voxel_center_list,
                                                           max_voxel_count);
      curr_node = getNextIntersectedNode(max_x, mid_y, max_z, 8, 7, 8);
      break;

    case 6:
      if (child_node)
        voxel_count += getIntersectedVoxelCentersRecursive(mid_x,
                                                           mid_y,
                                                           min_z,
                                                           max_x,
                                                           max_y,
                                                           mid_z,
                                                           a,
                                                           child_node,
                                                           child_key,
                                                           voxel_center_list,
                                                           max_voxel_count);
      curr_node = getNextIntersectedNode(max_x, max_y, mid_z, 8, 8, 7);
      break;

    case 7:
      if (child_node)
        voxel_count += getIntersectedVoxelCentersRecursive(mid_x,
                                                           mid_y,
                                                           mid_z,
                                                           max_x,
                                                           max_y,
                                                           max_z,
                                                           a,
                                                           child_node,
                                                           child_key,
                                                           voxel_center_list,
                                                           max_voxel_count);
      curr_node = 8;
      break;
    }
  } while ((curr_node < 8) && (max_voxel_count <= 0 || voxel_count < max_voxel_count));
  return (voxel_count);
}

template <typename PointT, typename LeafContainerT, typename BranchContainerT>
uindex_t
OctreePointCloudSearch<PointT, LeafContainerT, BranchContainerT>::
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
                                        uindex_t max_voxel_count) const
{
  if (max_x < 0.0 || max_y < 0.0 || max_z < 0.0)
    return (0);

  // If leaf node, get voxel center and increment intersection count
  if (node->getNodeType() == LEAF_NODE) {
    const LeafNode* leaf = static_cast<const LeafNode*>(node);

    // decode leaf node into k_indices
    (*leaf)->getPointIndices(k_indices);

    return (1);
  }

  // Voxel intersection count for branches children
  uindex_t voxel_count = 0;

  // Voxel mid lines
  double mid_x = 0.5 * (min_x + max_x);
  double mid_y = 0.5 * (min_y + max_y);
  double mid_z = 0.5 * (min_z + max_z);

  // First voxel node ray will intersect
  auto curr_node = getFirstIntersectedNode(min_x, min_y, min_z, mid_x, mid_y, mid_z);

  // Child index, node and key
  unsigned char child_idx;
  OctreeKey child_key;
  do {
    if (curr_node != 0)
      child_idx = static_cast<unsigned char>(curr_node ^ a);
    else
      child_idx = a;

    // child_node == 0 if child_node doesn't exist
    const OctreeNode* child_node =
        this->getBranchChildPtr(static_cast<const BranchNode&>(*node), child_idx);
    // Generate new key for current branch voxel
    child_key.x = (key.x << 1) | (!!(child_idx & (1 << 2)));
    child_key.y = (key.y << 1) | (!!(child_idx & (1 << 1)));
    child_key.z = (key.z << 1) | (!!(child_idx & (1 << 0)));

    // Recursively call each intersected child node, selecting the next
    //   node intersected by the ray.  Children that do not intersect will
    //   not be traversed.
    switch (curr_node) {
    case 0:
      if (child_node)
        voxel_count += getIntersectedVoxelIndicesRecursive(min_x,
                                                           min_y,
                                                           min_z,
                                                           mid_x,
                                                           mid_y,
                                                           mid_z,
                                                           a,
                                                           child_node,
                                                           child_key,
                                                           k_indices,
                                                           max_voxel_count);
      curr_node = getNextIntersectedNode(mid_x, mid_y, mid_z, 4, 2, 1);
      break;

    case 1:
      if (child_node)
        voxel_count += getIntersectedVoxelIndicesRecursive(min_x,
                                                           min_y,
                                                           mid_z,
                                                           mid_x,
                                                           mid_y,
                                                           max_z,
                                                           a,
                                                           child_node,
                                                           child_key,
                                                           k_indices,
                                                           max_voxel_count);
      curr_node = getNextIntersectedNode(mid_x, mid_y, max_z, 5, 3, 8);
      break;

    case 2:
      if (child_node)
        voxel_count += getIntersectedVoxelIndicesRecursive(min_x,
                                                           mid_y,
                                                           min_z,
                                                           mid_x,
                                                           max_y,
                                                           mid_z,
                                                           a,
                                                           child_node,
                                                           child_key,
                                                           k_indices,
                                                           max_voxel_count);
      curr_node = getNextIntersectedNode(mid_x, max_y, mid_z, 6, 8, 3);
      break;

    case 3:
      if (child_node)
        voxel_count += getIntersectedVoxelIndicesRecursive(min_x,
                                                           mid_y,
                                                           mid_z,
                                                           mid_x,
                                                           max_y,
                                                           max_z,
                                                           a,
                                                           child_node,
                                                           child_key,
                                                           k_indices,
                                                           max_voxel_count);
      curr_node = getNextIntersectedNode(mid_x, max_y, max_z, 7, 8, 8);
      break;

    case 4:
      if (child_node)
        voxel_count += getIntersectedVoxelIndicesRecursive(mid_x,
                                                           min_y,
                                                           min_z,
                                                           max_x,
                                                           mid_y,
                                                           mid_z,
                                                           a,
                                                           child_node,
                                                           child_key,
                                                           k_indices,
                                                           max_voxel_count);
      curr_node = getNextIntersectedNode(max_x, mid_y, mid_z, 8, 6, 5);
      break;

    case 5:
      if (child_node)
        voxel_count += getIntersectedVoxelIndicesRecursive(mid_x,
                                                           min_y,
                                                           mid_z,
                                                           max_x,
                                                           mid_y,
                                                           max_z,
                                                           a,
                                                           child_node,
                                                           child_key,
                                                           k_indices,
                                                           max_voxel_count);
      curr_node = getNextIntersectedNode(max_x, mid_y, max_z, 8, 7, 8);
      break;

    case 6:
      if (child_node)
        voxel_count += getIntersectedVoxelIndicesRecursive(mid_x,
                                                           mid_y,
                                                           min_z,
                                                           max_x,
                                                           max_y,
                                                           mid_z,
                                                           a,
                                                           child_node,
                                                           child_key,
                                                           k_indices,
                                                           max_voxel_count);
      curr_node = getNextIntersectedNode(max_x, max_y, mid_z, 8, 8, 7);
      break;

    case 7:
      if (child_node)
        voxel_count += getIntersectedVoxelIndicesRecursive(mid_x,
                                                           mid_y,
                                                           mid_z,
                                                           max_x,
                                                           max_y,
                                                           max_z,
                                                           a,
                                                           child_node,
                                                           child_key,
                                                           k_indices,
                                                           max_voxel_count);
      curr_node = 8;
      break;
    }
  } while ((curr_node < 8) && (max_voxel_count <= 0 || voxel_count < max_voxel_count));

  return (voxel_count);
}

} // namespace octree
} // namespace pcl

#define PCL_INSTANTIATE_OctreePointCloudSearch(T)                                      \
  template class PCL_EXPORTS pcl::octree::OctreePointCloudSearch<T>;

#endif // PCL_OCTREE_SEARCH_IMPL_H_
