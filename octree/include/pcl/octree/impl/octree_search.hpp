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

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/common/common.h>
#include <assert.h>


//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafContainerT, typename BranchContainerT> bool
pcl::octree::OctreePointCloudSearch<PointT, LeafContainerT, BranchContainerT>::voxelSearch (const PointT& point,
                                                                          std::vector<int>& point_idx_data)
{
  assert (isFinite (point) && "Invalid (NaN, Inf) point coordinates given to nearestKSearch!");
  OctreeKey key;
  bool b_success = false;

  // generate key
  this->genOctreeKeyforPoint (point, key);

  LeafContainerT* leaf = this->findLeaf (key);

  if (leaf)
  {
    (*leaf).getPointIndices (point_idx_data);
    b_success = true;
  }

  return (b_success);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafContainerT, typename BranchContainerT> bool
pcl::octree::OctreePointCloudSearch<PointT, LeafContainerT, BranchContainerT>::voxelSearch (const int index,
                                                                          std::vector<int>& point_idx_data)
{
  const PointT search_point = this->getPointByIndex (index);
  return (this->voxelSearch (search_point, point_idx_data));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafContainerT, typename BranchContainerT> int
pcl::octree::OctreePointCloudSearch<PointT, LeafContainerT, BranchContainerT>::nearestKSearch (const PointT &p_q, int k,
                                                                             std::vector<int> &k_indices,
                                                                             std::vector<float> &k_sqr_distances)
{
  assert(this->leaf_count_>0);
  assert (isFinite (p_q) && "Invalid (NaN, Inf) point coordinates given to nearestKSearch!");

  k_indices.clear ();
  k_sqr_distances.clear ();

  if (k < 1)
    return 0;
  
  unsigned int i;
  unsigned int result_count;

  prioPointQueueEntry point_entry;
  std::vector<prioPointQueueEntry> point_candidates;

  OctreeKey key;
  key.x = key.y = key.z = 0;

  // initalize smallest point distance in search with high value
  double smallest_dist = std::numeric_limits<double>::max ();

  getKNearestNeighborRecursive (p_q, k, this->root_node_, key, 1, smallest_dist, point_candidates);

  result_count = static_cast<unsigned int> (point_candidates.size ());

  k_indices.resize (result_count);
  k_sqr_distances.resize (result_count);
  
  for (i = 0; i < result_count; ++i)
  {
    k_indices [i] = point_candidates [i].point_idx_;
    k_sqr_distances [i] = point_candidates [i].point_distance_;
  }

  return static_cast<int> (k_indices.size ());
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafContainerT, typename BranchContainerT> int
pcl::octree::OctreePointCloudSearch<PointT, LeafContainerT, BranchContainerT>::nearestKSearch (int index, int k,
                                                                             std::vector<int> &k_indices,
                                                                             std::vector<float> &k_sqr_distances)
{
  const PointT search_point = this->getPointByIndex (index);
  return (nearestKSearch (search_point, k, k_indices, k_sqr_distances));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafContainerT, typename BranchContainerT> void
pcl::octree::OctreePointCloudSearch<PointT, LeafContainerT, BranchContainerT>::approxNearestSearch (const PointT &p_q,
                                                                                  int &result_index,
                                                                                  float &sqr_distance)
{
  assert(this->leaf_count_>0);
  assert (isFinite (p_q) && "Invalid (NaN, Inf) point coordinates given to nearestKSearch!");
  
  OctreeKey key;
  key.x = key.y = key.z = 0;

  approxNearestSearchRecursive (p_q, this->root_node_, key, 1, result_index, sqr_distance);

  return;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafContainerT, typename BranchContainerT> void
pcl::octree::OctreePointCloudSearch<PointT, LeafContainerT, BranchContainerT>::approxNearestSearch (int query_index, int &result_index,
                                                                                  float &sqr_distance)
{
  const PointT search_point = this->getPointByIndex (query_index);

  return (approxNearestSearch (search_point, result_index, sqr_distance));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafContainerT, typename BranchContainerT> int
pcl::octree::OctreePointCloudSearch<PointT, LeafContainerT, BranchContainerT>::radiusSearch (const PointT &p_q, const double radius,
                                                                           std::vector<int> &k_indices,
                                                                           std::vector<float> &k_sqr_distances,
                                                                           unsigned int max_nn) const
{
  assert (isFinite (p_q) && "Invalid (NaN, Inf) point coordinates given to nearestKSearch!");
  OctreeKey key;
  key.x = key.y = key.z = 0;

  k_indices.clear ();
  k_sqr_distances.clear ();

  getNeighborsWithinRadiusRecursive (p_q, radius * radius, this->root_node_, key, 1, k_indices, k_sqr_distances,
                                     max_nn);

  return (static_cast<int> (k_indices.size ()));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafContainerT, typename BranchContainerT> int
pcl::octree::OctreePointCloudSearch<PointT, LeafContainerT, BranchContainerT>::radiusSearch (int index, const double radius,
                                                                           std::vector<int> &k_indices,
                                                                           std::vector<float> &k_sqr_distances,
                                                                           unsigned int max_nn) const
{
  const PointT search_point = this->getPointByIndex (index);

  return (radiusSearch (search_point, radius, k_indices, k_sqr_distances, max_nn));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafContainerT, typename BranchContainerT> int
pcl::octree::OctreePointCloudSearch<PointT, LeafContainerT, BranchContainerT>::boxSearch (const Eigen::Vector3f &min_pt,
                                                                        const Eigen::Vector3f &max_pt,
                                                                        std::vector<int> &k_indices) const
{

  OctreeKey key;
  key.x = key.y = key.z = 0;

  k_indices.clear ();

  boxSearchRecursive (min_pt, max_pt, this->root_node_, key, 1, k_indices);

  return (static_cast<int> (k_indices.size ()));

}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafContainerT, typename BranchContainerT> double
pcl::octree::OctreePointCloudSearch<PointT, LeafContainerT, BranchContainerT>::getKNearestNeighborRecursive (
    const PointT & point, unsigned int K, const BranchNode* node, const OctreeKey& key, unsigned int tree_depth,
    const double squared_search_radius, std::vector<prioPointQueueEntry>& point_candidates) const
{
  std::vector<prioBranchQueueEntry> search_heap;
  search_heap.resize (8);

  unsigned char child_idx;

  OctreeKey new_key;

  double smallest_squared_dist = squared_search_radius;

  // get spatial voxel information
  double voxelSquaredDiameter = this->getVoxelSquaredDiameter (tree_depth);

  // iterate over all children
  for (child_idx = 0; child_idx < 8; child_idx++)
  {
    if (this->branchHasChild (*node, child_idx))
    {
      PointT voxel_center;

      search_heap[child_idx].key.x = (key.x << 1) + (!!(child_idx & (1 << 2)));
      search_heap[child_idx].key.y = (key.y << 1) + (!!(child_idx & (1 << 1)));
      search_heap[child_idx].key.z = (key.z << 1) + (!!(child_idx & (1 << 0)));

      // generate voxel center point for voxel at key
      this->genVoxelCenterFromOctreeKey (search_heap[child_idx].key, tree_depth, voxel_center);

      // generate new priority queue element
      search_heap[child_idx].node = this->getBranchChildPtr (*node, child_idx);
      search_heap[child_idx].point_distance = pointSquaredDist (voxel_center, point);
    }
    else
    {
      search_heap[child_idx].point_distance = std::numeric_limits<float>::infinity ();
    }
  }

  std::sort (search_heap.begin (), search_heap.end ());

  // iterate over all children in priority queue
  // check if the distance to search candidate is smaller than the best point distance (smallest_squared_dist)
  while ((!search_heap.empty ()) && (search_heap.back ().point_distance <
         smallest_squared_dist + voxelSquaredDiameter / 4.0 + sqrt (smallest_squared_dist * voxelSquaredDiameter) - this->epsilon_))
  {
    const OctreeNode* child_node;

    // read from priority queue element
    child_node = search_heap.back ().node;
    new_key = search_heap.back ().key;

    if (tree_depth < this->octree_depth_)
    {
      // we have not reached maximum tree depth
      smallest_squared_dist = getKNearestNeighborRecursive (point, K, static_cast<const BranchNode*> (child_node), new_key, tree_depth + 1,
                                                            smallest_squared_dist, point_candidates);
    }
    else
    {
      // we reached leaf node level

      float squared_dist;
      size_t i;
      std::vector<int> decoded_point_vector;

      const LeafNode* child_leaf = static_cast<const LeafNode*> (child_node);

      // decode leaf node into decoded_point_vector
      (*child_leaf)->getPointIndices (decoded_point_vector);

      // Linearly iterate over all decoded (unsorted) points
      for (i = 0; i < decoded_point_vector.size (); i++)
      {

        const PointT& candidate_point = this->getPointByIndex (decoded_point_vector[i]);

        // calculate point distance to search point
        squared_dist = pointSquaredDist (candidate_point, point);

        // check if a closer match is found
        if (squared_dist < smallest_squared_dist)
        {
          prioPointQueueEntry point_entry;

          point_entry.point_distance_ = squared_dist;
          point_entry.point_idx_ = decoded_point_vector[i];
          point_candidates.push_back (point_entry);
        }
      }

      std::sort (point_candidates.begin (), point_candidates.end ());

      if (point_candidates.size () > K)
        point_candidates.resize (K);

      if (point_candidates.size () == K)
        smallest_squared_dist = point_candidates.back ().point_distance_;
    }
    // pop element from priority queue
    search_heap.pop_back ();
  }

  return (smallest_squared_dist);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafContainerT, typename BranchContainerT> void
pcl::octree::OctreePointCloudSearch<PointT, LeafContainerT, BranchContainerT>::getNeighborsWithinRadiusRecursive (
    const PointT & point, const double radiusSquared, const BranchNode* node, const OctreeKey& key,
    unsigned int tree_depth, std::vector<int>& k_indices, std::vector<float>& k_sqr_distances,
    unsigned int max_nn) const
{
  // child iterator
  unsigned char child_idx;

  // get spatial voxel information
  double voxel_squared_diameter = this->getVoxelSquaredDiameter (tree_depth);

  // iterate over all children
  for (child_idx = 0; child_idx < 8; child_idx++)
  {
    if (!this->branchHasChild (*node, child_idx))
      continue;

    const OctreeNode* child_node;
    child_node = this->getBranchChildPtr (*node, child_idx);

    OctreeKey new_key;
    PointT voxel_center;
    float squared_dist;

    // generate new key for current branch voxel
    new_key.x = (key.x << 1) + (!!(child_idx & (1 << 2)));
    new_key.y = (key.y << 1) + (!!(child_idx & (1 << 1)));
    new_key.z = (key.z << 1) + (!!(child_idx & (1 << 0)));

    // generate voxel center point for voxel at key
    this->genVoxelCenterFromOctreeKey (new_key, tree_depth, voxel_center);

    // calculate distance to search point
    squared_dist = pointSquaredDist (static_cast<const PointT&> (voxel_center), point);

    // if distance is smaller than search radius
    if (squared_dist + this->epsilon_
        <= voxel_squared_diameter / 4.0 + radiusSquared + sqrt (voxel_squared_diameter * radiusSquared))
    {

      if (tree_depth < this->octree_depth_)
      {
        // we have not reached maximum tree depth
        getNeighborsWithinRadiusRecursive (point, radiusSquared, static_cast<const BranchNode*> (child_node), new_key, tree_depth + 1,
                                           k_indices, k_sqr_distances, max_nn);
        if (max_nn != 0 && k_indices.size () == static_cast<unsigned int> (max_nn))
          return;
      }
      else
      {
        // we reached leaf node level

        size_t i;
        const LeafNode* child_leaf = static_cast<const LeafNode*> (child_node);
        std::vector<int> decoded_point_vector;

        // decode leaf node into decoded_point_vector
        (*child_leaf)->getPointIndices (decoded_point_vector);

        // Linearly iterate over all decoded (unsorted) points
        for (i = 0; i < decoded_point_vector.size (); i++)
        {
          const PointT& candidate_point = this->getPointByIndex (decoded_point_vector[i]);

          // calculate point distance to search point
          squared_dist = pointSquaredDist (candidate_point, point);

          // check if a match is found
          if (squared_dist > radiusSquared)
            continue;

          // add point to result vector
          k_indices.push_back (decoded_point_vector[i]);
          k_sqr_distances.push_back (squared_dist);

          if (max_nn != 0 && k_indices.size () == static_cast<unsigned int> (max_nn))
            return;
        }
      }
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafContainerT, typename BranchContainerT> void
pcl::octree::OctreePointCloudSearch<PointT, LeafContainerT, BranchContainerT>::approxNearestSearchRecursive (const PointT & point,
                                                                                           const BranchNode* node,
                                                                                           const OctreeKey& key,
                                                                                           unsigned int tree_depth,
                                                                                           int& result_index,
                                                                                           float& sqr_distance)
{
  unsigned char child_idx;
  unsigned char min_child_idx;
  double min_voxel_center_distance;

  OctreeKey minChildKey;
  OctreeKey new_key;

  const OctreeNode* child_node;

  // set minimum voxel distance to maximum value
  min_voxel_center_distance = std::numeric_limits<double>::max ();

  min_child_idx = 0xFF;

  // iterate over all children
  for (child_idx = 0; child_idx < 8; child_idx++)
  {
    if (!this->branchHasChild (*node, child_idx))
      continue;

    PointT voxel_center;
    double voxelPointDist;

    new_key.x = (key.x << 1) + (!!(child_idx & (1 << 2)));
    new_key.y = (key.y << 1) + (!!(child_idx & (1 << 1)));
    new_key.z = (key.z << 1) + (!!(child_idx & (1 << 0)));

    // generate voxel center point for voxel at key
    this->genVoxelCenterFromOctreeKey (new_key, tree_depth, voxel_center);

    voxelPointDist = pointSquaredDist (voxel_center, point);

    // search for child voxel with shortest distance to search point
    if (voxelPointDist >= min_voxel_center_distance)
      continue;

    min_voxel_center_distance = voxelPointDist;
    min_child_idx = child_idx;
    minChildKey = new_key;
  }

  // make sure we found at least one branch child
  assert(min_child_idx<8);

  child_node = this->getBranchChildPtr (*node, min_child_idx);

  if (tree_depth < this->octree_depth_)
  {
    // we have not reached maximum tree depth
    approxNearestSearchRecursive (point, static_cast<const BranchNode*> (child_node), minChildKey, tree_depth + 1, result_index,
                                  sqr_distance);
  }
  else
  {
    // we reached leaf node level

    double squared_dist;
    double smallest_squared_dist;
    size_t i;
    std::vector<int> decoded_point_vector;

    const LeafNode* child_leaf = static_cast<const LeafNode*> (child_node);

    smallest_squared_dist = std::numeric_limits<double>::max ();

    // decode leaf node into decoded_point_vector
    (**child_leaf).getPointIndices (decoded_point_vector);

    // Linearly iterate over all decoded (unsorted) points
    for (i = 0; i < decoded_point_vector.size (); i++)
    {
      const PointT& candidate_point = this->getPointByIndex (decoded_point_vector[i]);

      // calculate point distance to search point
      squared_dist = pointSquaredDist (candidate_point, point);

      // check if a closer match is found
      if (squared_dist >= smallest_squared_dist)
        continue;

      result_index = decoded_point_vector[i];
      smallest_squared_dist = squared_dist;
      sqr_distance = static_cast<float> (squared_dist);
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafContainerT, typename BranchContainerT> float
pcl::octree::OctreePointCloudSearch<PointT, LeafContainerT, BranchContainerT>::pointSquaredDist (const PointT & point_a,
                                                                               const PointT & point_b) const
{
  return (point_a.getVector3fMap () - point_b.getVector3fMap ()).squaredNorm ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafContainerT, typename BranchContainerT> void
pcl::octree::OctreePointCloudSearch<PointT, LeafContainerT, BranchContainerT>::boxSearchRecursive (const Eigen::Vector3f &min_pt,
                                                                                 const Eigen::Vector3f &max_pt,
                                                                                 const BranchNode* node,
                                                                                 const OctreeKey& key,
                                                                                 unsigned int tree_depth,
                                                                                 std::vector<int>& k_indices) const
{
  // child iterator
  unsigned char child_idx;

  // iterate over all children
  for (child_idx = 0; child_idx < 8; child_idx++)
  {

    const OctreeNode* child_node;
    child_node = this->getBranchChildPtr (*node, child_idx);

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
    this->genVoxelBoundsFromOctreeKey (new_key, tree_depth, lower_voxel_corner, upper_voxel_corner);

    // test if search region overlap with voxel space

    if ( !( (lower_voxel_corner (0) > max_pt (0)) || (min_pt (0) > upper_voxel_corner(0)) ||
            (lower_voxel_corner (1) > max_pt (1)) || (min_pt (1) > upper_voxel_corner(1)) ||
            (lower_voxel_corner (2) > max_pt (2)) || (min_pt (2) > upper_voxel_corner(2)) ) )
    {

      if (tree_depth < this->octree_depth_)
      {
        // we have not reached maximum tree depth
        boxSearchRecursive (min_pt, max_pt, static_cast<const BranchNode*> (child_node), new_key, tree_depth + 1, k_indices);
      }
      else
      {
        // we reached leaf node level
        size_t i;
        std::vector<int> decoded_point_vector;
        bool bInBox;

        const LeafNode* child_leaf = static_cast<const LeafNode*> (child_node);

        // decode leaf node into decoded_point_vector
        (**child_leaf).getPointIndices (decoded_point_vector);

        // Linearly iterate over all decoded (unsorted) points
        for (i = 0; i < decoded_point_vector.size (); i++)
        {
          const PointT& candidate_point = this->getPointByIndex (decoded_point_vector[i]);

          // check if point falls within search box
          bInBox = ( (candidate_point.x >= min_pt (0)) && (candidate_point.x <= max_pt (0)) &&
                     (candidate_point.y >= min_pt (1)) && (candidate_point.y <= max_pt (1)) &&
                     (candidate_point.z >= min_pt (2)) && (candidate_point.z <= max_pt (2)) );

          if (bInBox)
            // add to result vector
            k_indices.push_back (decoded_point_vector[i]);
        }
      }
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafContainerT, typename BranchContainerT> int
pcl::octree::OctreePointCloudSearch<PointT, LeafContainerT, BranchContainerT>::getIntersectedVoxelCenters (
    Eigen::Vector3f origin, Eigen::Vector3f direction, AlignedPointTVector &voxel_center_list,
    int max_voxel_count) const
{
  OctreeKey key;
  key.x = key.y = key.z = 0;

  voxel_center_list.clear ();

  // Voxel child_idx remapping
  unsigned char a = 0;

  double min_x, min_y, min_z, max_x, max_y, max_z;

  initIntersectedVoxel (origin, direction, min_x, min_y, min_z, max_x, max_y, max_z, a);

  if (std::max (std::max (min_x, min_y), min_z) < std::min (std::min (max_x, max_y), max_z))
    return getIntersectedVoxelCentersRecursive (min_x, min_y, min_z, max_x, max_y, max_z, a, this->root_node_, key,
                                                voxel_center_list, max_voxel_count);

  return (0);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafContainerT, typename BranchContainerT> int
pcl::octree::OctreePointCloudSearch<PointT, LeafContainerT, BranchContainerT>::getIntersectedVoxelIndices (
    Eigen::Vector3f origin, Eigen::Vector3f direction, std::vector<int> &k_indices,
    int max_voxel_count) const
{
  OctreeKey key;
  key.x = key.y = key.z = 0;

  k_indices.clear ();

  // Voxel child_idx remapping
  unsigned char a = 0;
  double min_x, min_y, min_z, max_x, max_y, max_z;

  initIntersectedVoxel (origin, direction, min_x, min_y, min_z, max_x, max_y, max_z, a);

  if (std::max (std::max (min_x, min_y), min_z) < std::min (std::min (max_x, max_y), max_z))
    return getIntersectedVoxelIndicesRecursive (min_x, min_y, min_z, max_x, max_y, max_z, a, this->root_node_, key,
                                                k_indices, max_voxel_count);
  return (0);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafContainerT, typename BranchContainerT> int
pcl::octree::OctreePointCloudSearch<PointT, LeafContainerT, BranchContainerT>::getIntersectedVoxelCentersRecursive (
    double min_x, double min_y, double min_z, double max_x, double max_y, double max_z, unsigned char a,
    const OctreeNode* node, const OctreeKey& key, AlignedPointTVector &voxel_center_list, int max_voxel_count) const
{
  if (max_x < 0.0 || max_y < 0.0 || max_z < 0.0)
    return (0);

  // If leaf node, get voxel center and increment intersection count
  if (node->getNodeType () == LEAF_NODE)
  {
    PointT newPoint;

    this->genLeafNodeCenterFromOctreeKey (key, newPoint);

    voxel_center_list.push_back (newPoint);

    return (1);
  }

  // Voxel intersection count for branches children
  int voxel_count = 0;

  // Voxel mid lines
  double mid_x = 0.5 * (min_x + max_x);
  double mid_y = 0.5 * (min_y + max_y);
  double mid_z = 0.5 * (min_z + max_z);

  // First voxel node ray will intersect
  int curr_node = getFirstIntersectedNode (min_x, min_y, min_z, mid_x, mid_y, mid_z);

  // Child index, node and key
  unsigned char child_idx;
  const OctreeNode *child_node;
  OctreeKey child_key;

  do
  {
    if (curr_node != 0)
      child_idx = static_cast<unsigned char> (curr_node ^ a);
    else
      child_idx = a;

    // child_node == 0 if child_node doesn't exist
    child_node = this->getBranchChildPtr (static_cast<const BranchNode&> (*node), child_idx);

    // Generate new key for current branch voxel
    child_key.x = (key.x << 1) | (!!(child_idx & (1 << 2)));
    child_key.y = (key.y << 1) | (!!(child_idx & (1 << 1)));
    child_key.z = (key.z << 1) | (!!(child_idx & (1 << 0)));

    // Recursively call each intersected child node, selecting the next
    //   node intersected by the ray.  Children that do not intersect will
    //   not be traversed.

    switch (curr_node)
    {
      case 0:
        if (child_node)
          voxel_count += getIntersectedVoxelCentersRecursive (min_x, min_y, min_z, mid_x, mid_y, mid_z, a, child_node,
                                                             child_key, voxel_center_list, max_voxel_count);
        curr_node = getNextIntersectedNode (mid_x, mid_y, mid_z, 4, 2, 1);
        break;

      case 1:
        if (child_node)
          voxel_count += getIntersectedVoxelCentersRecursive (min_x, min_y, mid_z, mid_x, mid_y, max_z, a, child_node,
                                                             child_key, voxel_center_list, max_voxel_count);
        curr_node = getNextIntersectedNode (mid_x, mid_y, max_z, 5, 3, 8);
        break;

      case 2:
        if (child_node)
          voxel_count += getIntersectedVoxelCentersRecursive (min_x, mid_y, min_z, mid_x, max_y, mid_z, a, child_node,
                                                             child_key, voxel_center_list, max_voxel_count);
        curr_node = getNextIntersectedNode (mid_x, max_y, mid_z, 6, 8, 3);
        break;

      case 3:
        if (child_node)
          voxel_count += getIntersectedVoxelCentersRecursive (min_x, mid_y, mid_z, mid_x, max_y, max_z, a, child_node,
                                                             child_key, voxel_center_list, max_voxel_count);
        curr_node = getNextIntersectedNode (mid_x, max_y, max_z, 7, 8, 8);
        break;

      case 4:
        if (child_node)
          voxel_count += getIntersectedVoxelCentersRecursive (mid_x, min_y, min_z, max_x, mid_y, mid_z, a, child_node,
                                                             child_key, voxel_center_list, max_voxel_count);
        curr_node = getNextIntersectedNode (max_x, mid_y, mid_z, 8, 6, 5);
        break;

      case 5:
        if (child_node)
          voxel_count += getIntersectedVoxelCentersRecursive (mid_x, min_y, mid_z, max_x, mid_y, max_z, a, child_node,
                                                             child_key, voxel_center_list, max_voxel_count);
        curr_node = getNextIntersectedNode (max_x, mid_y, max_z, 8, 7, 8);
        break;

      case 6:
        if (child_node)
          voxel_count += getIntersectedVoxelCentersRecursive (mid_x, mid_y, min_z, max_x, max_y, mid_z, a, child_node,
                                                             child_key, voxel_center_list, max_voxel_count);
        curr_node = getNextIntersectedNode (max_x, max_y, mid_z, 8, 8, 7);
        break;

      case 7:
        if (child_node)
          voxel_count += getIntersectedVoxelCentersRecursive (mid_x, mid_y, mid_z, max_x, max_y, max_z, a, child_node,
                                                             child_key, voxel_center_list, max_voxel_count);
        curr_node = 8;
        break;
    }
  } while ((curr_node < 8) && (max_voxel_count <= 0 || voxel_count < max_voxel_count));
  return (voxel_count);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafContainerT, typename BranchContainerT> int
pcl::octree::OctreePointCloudSearch<PointT, LeafContainerT, BranchContainerT>::getIntersectedVoxelIndicesRecursive (
    double min_x, double min_y, double min_z, double max_x, double max_y, double max_z, unsigned char a,
    const OctreeNode* node, const OctreeKey& key, std::vector<int> &k_indices, int max_voxel_count) const
{
  if (max_x < 0.0 || max_y < 0.0 || max_z < 0.0)
    return (0);

  // If leaf node, get voxel center and increment intersection count
  if (node->getNodeType () == LEAF_NODE)
  {
    const LeafNode* leaf = static_cast<const LeafNode*> (node);

    // decode leaf node into k_indices
    (*leaf)->getPointIndices (k_indices);

    return (1);
  }

  // Voxel intersection count for branches children
  int voxel_count = 0;

  // Voxel mid lines
  double mid_x = 0.5 * (min_x + max_x);
  double mid_y = 0.5 * (min_y + max_y);
  double mid_z = 0.5 * (min_z + max_z);

  // First voxel node ray will intersect
  int curr_node = getFirstIntersectedNode (min_x, min_y, min_z, mid_x, mid_y, mid_z);

  // Child index, node and key
  unsigned char child_idx;
  const OctreeNode *child_node;
  OctreeKey child_key;
  do
  {
    if (curr_node != 0)
      child_idx = static_cast<unsigned char> (curr_node ^ a);
    else
      child_idx = a;

    // child_node == 0 if child_node doesn't exist
    child_node = this->getBranchChildPtr (static_cast<const BranchNode&> (*node), child_idx);
    // Generate new key for current branch voxel
    child_key.x = (key.x << 1) | (!!(child_idx & (1 << 2)));
    child_key.y = (key.y << 1) | (!!(child_idx & (1 << 1)));
    child_key.z = (key.z << 1) | (!!(child_idx & (1 << 0)));

    // Recursively call each intersected child node, selecting the next
    //   node intersected by the ray.  Children that do not intersect will
    //   not be traversed.
    switch (curr_node)
    {
      case 0:
        if (child_node)
          voxel_count += getIntersectedVoxelIndicesRecursive (min_x, min_y, min_z, mid_x, mid_y, mid_z, a, child_node,
                                                             child_key, k_indices, max_voxel_count);
        curr_node = getNextIntersectedNode (mid_x, mid_y, mid_z, 4, 2, 1);
        break;

      case 1:
        if (child_node)
          voxel_count += getIntersectedVoxelIndicesRecursive (min_x, min_y, mid_z, mid_x, mid_y, max_z, a, child_node,
                                                             child_key, k_indices, max_voxel_count);
        curr_node = getNextIntersectedNode (mid_x, mid_y, max_z, 5, 3, 8);
        break;

      case 2:
        if (child_node)
          voxel_count += getIntersectedVoxelIndicesRecursive (min_x, mid_y, min_z, mid_x, max_y, mid_z, a, child_node,
                                                             child_key, k_indices, max_voxel_count);
        curr_node = getNextIntersectedNode (mid_x, max_y, mid_z, 6, 8, 3);
        break;

      case 3:
        if (child_node)
          voxel_count += getIntersectedVoxelIndicesRecursive (min_x, mid_y, mid_z, mid_x, max_y, max_z, a, child_node,
                                                             child_key, k_indices, max_voxel_count);
        curr_node = getNextIntersectedNode (mid_x, max_y, max_z, 7, 8, 8);
        break;

      case 4:
        if (child_node)
          voxel_count += getIntersectedVoxelIndicesRecursive (mid_x, min_y, min_z, max_x, mid_y, mid_z, a, child_node,
                                                             child_key, k_indices, max_voxel_count);
        curr_node = getNextIntersectedNode (max_x, mid_y, mid_z, 8, 6, 5);
        break;

      case 5:
        if (child_node)
          voxel_count += getIntersectedVoxelIndicesRecursive (mid_x, min_y, mid_z, max_x, mid_y, max_z, a, child_node,
                                                             child_key, k_indices, max_voxel_count);
        curr_node = getNextIntersectedNode (max_x, mid_y, max_z, 8, 7, 8);
        break;

      case 6:
        if (child_node)
          voxel_count += getIntersectedVoxelIndicesRecursive (mid_x, mid_y, min_z, max_x, max_y, mid_z, a, child_node,
                                                             child_key, k_indices, max_voxel_count);
        curr_node = getNextIntersectedNode (max_x, max_y, mid_z, 8, 8, 7);
        break;

      case 7:
        if (child_node)
          voxel_count += getIntersectedVoxelIndicesRecursive (mid_x, mid_y, mid_z, max_x, max_y, max_z, a, child_node,
                                                             child_key, k_indices, max_voxel_count);
        curr_node = 8;
        break;
    }
  } while ((curr_node < 8) && (max_voxel_count <= 0 || voxel_count < max_voxel_count));

  return (voxel_count);
}

#endif    // PCL_OCTREE_SEARCH_IMPL_H_
