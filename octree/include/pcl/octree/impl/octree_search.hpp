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
template<typename PointT, typename LeafT, typename BranchT> bool
pcl::octree::OctreePointCloudSearch<PointT, LeafT, BranchT>::voxelSearch (const PointT& point,
                                                                          std::vector<int>& pointIdx_data)
{
  assert (isFinite (point) && "Invalid (NaN, Inf) point coordinates given to nearestKSearch!");
  OctreeKey key;
  bool b_success = false;

  // generate key
  this->genOctreeKeyforPoint (point, key);

  LeafT* leaf = this->findLeaf (key);

  if (leaf)
  {
    leaf->getData (pointIdx_data);
    b_success = true;
  }

  return (b_success);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafT, typename BranchT> bool
pcl::octree::OctreePointCloudSearch<PointT, LeafT, BranchT>::voxelSearch (const int index,
                                                                          std::vector<int>& pointIdx_data)
{
  const PointT search_point = this->getPointByIndex (index);
  return (this->voxelSearch (search_point, pointIdx_data));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafT, typename BranchT> int
pcl::octree::OctreePointCloudSearch<PointT, LeafT, BranchT>::nearestKSearch (const PointT &p_q, int k,
                                                                             std::vector<int> &k_indices,
                                                                             std::vector<float> &k_sqr_distances)
{
  assert(this->leafCount_>0);
  assert (isFinite (p_q) && "Invalid (NaN, Inf) point coordinates given to nearestKSearch!");

  k_indices.clear ();
  k_sqr_distances.clear ();

  if (k < 1)
    return 0;
  
  unsigned int i;
  unsigned int resultCount;

  prioPointQueueEntry pointEntry;
  std::vector<prioPointQueueEntry> pointCandidates;

  OctreeKey key;
  key.x = key.y = key.z = 0;

  // initalize smallest point distance in search with high value
  double smallestDist = numeric_limits<double>::max ();

  getKNearestNeighborRecursive (p_q, k, this->rootNode_, key, 1, smallestDist, pointCandidates);

  resultCount = static_cast<unsigned int> (pointCandidates.size ());

  k_indices.resize (resultCount);
  k_sqr_distances.resize (resultCount);
  
  for (i = 0; i < resultCount; ++i)
  {
    k_indices [i] = pointCandidates [i].pointIdx_;
    k_sqr_distances [i] = pointCandidates [i].pointDistance_;
  }

  return static_cast<int> (k_indices.size ());
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafT, typename BranchT> int
pcl::octree::OctreePointCloudSearch<PointT, LeafT, BranchT>::nearestKSearch (int index, int k,
                                                                             std::vector<int> &k_indices,
                                                                             std::vector<float> &k_sqr_distances)
{
  const PointT search_point = this->getPointByIndex (index);
  return (nearestKSearch (search_point, k, k_indices, k_sqr_distances));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafT, typename BranchT> void
pcl::octree::OctreePointCloudSearch<PointT, LeafT, BranchT>::approxNearestSearch (const PointT &p_q,
                                                                                  int &result_index,
                                                                                  float &sqr_distance)
{
  assert(this->leafCount_>0);
  assert (isFinite (p_q) && "Invalid (NaN, Inf) point coordinates given to nearestKSearch!");
  
  OctreeKey key;
  key.x = key.y = key.z = 0;

  approxNearestSearchRecursive (p_q, this->rootNode_, key, 1, result_index, sqr_distance);

  return;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafT, typename BranchT> void
pcl::octree::OctreePointCloudSearch<PointT, LeafT, BranchT>::approxNearestSearch (int query_index, int &result_index,
                                                                                  float &sqr_distance)
{
  const PointT searchPoint = this->getPointByIndex (query_index);

  return (approxNearestSearch (searchPoint, result_index, sqr_distance));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafT, typename BranchT> int
pcl::octree::OctreePointCloudSearch<PointT, LeafT, BranchT>::radiusSearch (const PointT &p_q, const double radius,
                                                                           std::vector<int> &k_indices,
                                                                           std::vector<float> &k_sqr_distances,
                                                                           unsigned int max_nn) const
{
  assert (isFinite (p_q) && "Invalid (NaN, Inf) point coordinates given to nearestKSearch!");
  OctreeKey key;
  key.x = key.y = key.z = 0;

  k_indices.clear ();
  k_sqr_distances.clear ();

  getNeighborsWithinRadiusRecursive (p_q, radius * radius, this->rootNode_, key, 1, k_indices, k_sqr_distances,
                                     max_nn);

  return (static_cast<int> (k_indices.size ()));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafT, typename BranchT> int
pcl::octree::OctreePointCloudSearch<PointT, LeafT, BranchT>::radiusSearch (int index, const double radius,
                                                                           std::vector<int> &k_indices,
                                                                           std::vector<float> &k_sqr_distances,
                                                                           unsigned int max_nn) const
{
  const PointT search_point = this->getPointByIndex (index);

  return (radiusSearch (search_point, radius, k_indices, k_sqr_distances, max_nn));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafT, typename BranchT> int
pcl::octree::OctreePointCloudSearch<PointT, LeafT, BranchT>::boxSearch (const Eigen::Vector3f &min_pt,
                                                                        const Eigen::Vector3f &max_pt,
                                                                        std::vector<int> &k_indices) const
{

  OctreeKey key;
  key.x = key.y = key.z = 0;

  k_indices.clear ();

  boxSearchRecursive (min_pt, max_pt, this->rootNode_, key, 1, k_indices);

  return (static_cast<int> (k_indices.size ()));

}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafT, typename BranchT> double
pcl::octree::OctreePointCloudSearch<PointT, LeafT, BranchT>::getKNearestNeighborRecursive (
    const PointT & point, unsigned int K, const BranchNode* node, const OctreeKey& key, unsigned int treeDepth,
    const double squaredSearchRadius, std::vector<prioPointQueueEntry>& pointCandidates) const
{
  std::vector<prioBranchQueueEntry> searchEntryHeap;
  searchEntryHeap.resize (8);

  unsigned char childIdx;

  OctreeKey newKey;

  double smallestSquaredDist = squaredSearchRadius;

  // get spatial voxel information
  double voxelSquaredDiameter = this->getVoxelSquaredDiameter (treeDepth);

  // iterate over all children
  for (childIdx = 0; childIdx < 8; childIdx++)
  {
    if (this->branchHasChild (*node, childIdx))
    {
      PointT voxelCenter;

      searchEntryHeap[childIdx].key.x = (key.x << 1) + (!!(childIdx & (1 << 2)));
      searchEntryHeap[childIdx].key.y = (key.y << 1) + (!!(childIdx & (1 << 1)));
      searchEntryHeap[childIdx].key.z = (key.z << 1) + (!!(childIdx & (1 << 0)));

      // generate voxel center point for voxel at key
      this->genVoxelCenterFromOctreeKey (searchEntryHeap[childIdx].key, treeDepth, voxelCenter);

      // generate new priority queue element
      searchEntryHeap[childIdx].node = this->getBranchChildPtr (*node, childIdx);
      searchEntryHeap[childIdx].pointDistance = pointSquaredDist (voxelCenter, point);
    }
    else
    {
      searchEntryHeap[childIdx].pointDistance = numeric_limits<float>::infinity ();
    }
  }

  std::sort (searchEntryHeap.begin (), searchEntryHeap.end ());

  // iterate over all children in priority queue
  // check if the distance to search candidate is smaller than the best point distance (smallestSquaredDist)
  while ((!searchEntryHeap.empty ())
      && (searchEntryHeap.back ().pointDistance
          < smallestSquaredDist + voxelSquaredDiameter / 4.0 + sqrt (smallestSquaredDist * voxelSquaredDiameter)
              - this->epsilon_))
  {
    const OctreeNode* childNode;

    // read from priority queue element
    childNode = searchEntryHeap.back ().node;
    newKey = searchEntryHeap.back ().key;

    if (treeDepth < this->octreeDepth_)
    {
      // we have not reached maximum tree depth
      smallestSquaredDist = getKNearestNeighborRecursive (point, K, static_cast<const BranchNode*> (childNode), newKey, treeDepth + 1,
                                                          smallestSquaredDist, pointCandidates);
    }
    else
    {
      // we reached leaf node level

      float squaredDist;
      size_t i;
      vector<int> decodedPointVector;

      const LeafNode* childLeaf = static_cast<const LeafNode*> (childNode);

      // decode leaf node into decodedPointVector
      childLeaf->getData (decodedPointVector);

      // Linearly iterate over all decoded (unsorted) points
      for (i = 0; i < decodedPointVector.size (); i++)
      {

        const PointT& candidatePoint = this->getPointByIndex (decodedPointVector[i]);

        // calculate point distance to search point
        squaredDist = pointSquaredDist (candidatePoint, point);

        // check if a closer match is found
        if (squaredDist < smallestSquaredDist)
        {
          prioPointQueueEntry pointEntry;

          pointEntry.pointDistance_ = squaredDist;
          pointEntry.pointIdx_ = decodedPointVector[i];
          pointCandidates.push_back (pointEntry);
        }
      }

      std::sort (pointCandidates.begin (), pointCandidates.end ());

      if (pointCandidates.size () > K)
        pointCandidates.resize (K);

      if (pointCandidates.size () == K)
        smallestSquaredDist = pointCandidates.back ().pointDistance_;
    }
    // pop element from priority queue
    searchEntryHeap.pop_back ();
  }

  return (smallestSquaredDist);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafT, typename BranchT> void
pcl::octree::OctreePointCloudSearch<PointT, LeafT, BranchT>::getNeighborsWithinRadiusRecursive (
    const PointT & point, const double radiusSquared, const BranchNode* node, const OctreeKey& key,
    unsigned int treeDepth, std::vector<int>& k_indices, std::vector<float>& k_sqr_distances,
    unsigned int max_nn) const
{
  // child iterator
  unsigned char childIdx;

  // get spatial voxel information
  double voxelSquaredDiameter = this->getVoxelSquaredDiameter (treeDepth);

  // iterate over all children
  for (childIdx = 0; childIdx < 8; childIdx++)
  {
    if (!this->branchHasChild (*node, childIdx))
      continue;

    const OctreeNode* childNode;
    childNode = this->getBranchChildPtr (*node, childIdx);

    OctreeKey newKey;
    PointT voxelCenter;
    float squaredDist;

    // generate new key for current branch voxel
    newKey.x = (key.x << 1) + (!!(childIdx & (1 << 2)));
    newKey.y = (key.y << 1) + (!!(childIdx & (1 << 1)));
    newKey.z = (key.z << 1) + (!!(childIdx & (1 << 0)));

    // generate voxel center point for voxel at key
    this->genVoxelCenterFromOctreeKey (newKey, treeDepth, voxelCenter);

    // calculate distance to search point
    squaredDist = pointSquaredDist (static_cast<const PointT&> (voxelCenter), point);

    // if distance is smaller than search radius
    if (squaredDist + this->epsilon_
        <= voxelSquaredDiameter / 4.0 + radiusSquared + sqrt (voxelSquaredDiameter * radiusSquared))
    {

      if (treeDepth < this->octreeDepth_)
      {
        // we have not reached maximum tree depth
        getNeighborsWithinRadiusRecursive (point, radiusSquared, static_cast<const BranchNode*> (childNode), newKey, treeDepth + 1,
                                           k_indices, k_sqr_distances, max_nn);
        if (max_nn != 0 && k_indices.size () == static_cast<unsigned int> (max_nn))
          return;
      }
      else
      {
        // we reached leaf node level

        size_t i;
        const LeafNode* childLeaf = static_cast<const LeafNode*> (childNode);
        vector<int> decodedPointVector;

        // decode leaf node into decodedPointVector
        childLeaf->getData (decodedPointVector);

        // Linearly iterate over all decoded (unsorted) points
        for (i = 0; i < decodedPointVector.size (); i++)
        {
          const PointT& candidatePoint = this->getPointByIndex (decodedPointVector[i]);

          // calculate point distance to search point
          squaredDist = pointSquaredDist (candidatePoint, point);

          // check if a match is found
          if (squaredDist > radiusSquared)
            continue;

          // add point to result vector
          k_indices.push_back (decodedPointVector[i]);
          k_sqr_distances.push_back (squaredDist);

          if (max_nn != 0 && k_indices.size () == static_cast<unsigned int> (max_nn))
            return;
        }
      }
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafT, typename BranchT> void
pcl::octree::OctreePointCloudSearch<PointT, LeafT, BranchT>::approxNearestSearchRecursive (const PointT & point,
                                                                                           const BranchNode* node,
                                                                                           const OctreeKey& key,
                                                                                           unsigned int treeDepth,
                                                                                           int& result_index,
                                                                                           float& sqr_distance)
{
  unsigned char childIdx;
  unsigned char minChildIdx;
  double minVoxelCenterDistance;

  OctreeKey minChildKey;
  OctreeKey newKey;

  const OctreeNode* childNode;

  // set minimum voxel distance to maximum value
  minVoxelCenterDistance = numeric_limits<double>::max ();

  minChildIdx = 0xFF;

  // iterate over all children
  for (childIdx = 0; childIdx < 8; childIdx++)
  {
    if (!this->branchHasChild (*node, childIdx))
      continue;

    PointT voxelCenter;
    double voxelPointDist;

    newKey.x = (key.x << 1) + (!!(childIdx & (1 << 2)));
    newKey.y = (key.y << 1) + (!!(childIdx & (1 << 1)));
    newKey.z = (key.z << 1) + (!!(childIdx & (1 << 0)));

    // generate voxel center point for voxel at key
    this->genVoxelCenterFromOctreeKey (newKey, treeDepth, voxelCenter);

    voxelPointDist = pointSquaredDist (voxelCenter, point);

    // search for child voxel with shortest distance to search point
    if (voxelPointDist >= minVoxelCenterDistance)
      continue;

    minVoxelCenterDistance = voxelPointDist;
    minChildIdx = childIdx;
    minChildKey = newKey;
  }

  // make sure we found at least one branch child
  assert(minChildIdx<8);

  childNode = this->getBranchChildPtr (*node, minChildIdx);

  if (treeDepth < this->octreeDepth_)
  {
    // we have not reached maximum tree depth
    approxNearestSearchRecursive (point, static_cast<const BranchNode*> (childNode), minChildKey, treeDepth + 1, result_index,
                                  sqr_distance);
  }
  else
  {
    // we reached leaf node level

    double squaredDist;
    double smallestSquaredDist;
    size_t i;
    vector<int> decodedPointVector;

    const LeafNode* childLeaf = static_cast<const LeafNode*> (childNode);

    smallestSquaredDist = numeric_limits<double>::max ();

    // decode leaf node into decodedPointVector
    childLeaf->getData (decodedPointVector);

    // Linearly iterate over all decoded (unsorted) points
    for (i = 0; i < decodedPointVector.size (); i++)
    {
      const PointT& candidatePoint = this->getPointByIndex (decodedPointVector[i]);

      // calculate point distance to search point
      squaredDist = pointSquaredDist (candidatePoint, point);

      // check if a closer match is found
      if (squaredDist >= smallestSquaredDist)
        continue;

      result_index = decodedPointVector[i];
      smallestSquaredDist = squaredDist;
      sqr_distance = static_cast<float> (squaredDist);
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafT, typename BranchT> float
pcl::octree::OctreePointCloudSearch<PointT, LeafT, BranchT>::pointSquaredDist (const PointT & pointA,
                                                                               const PointT & pointB) const
{
  return (pointA.getVector3fMap () - pointB.getVector3fMap ()).squaredNorm ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafT, typename BranchT> void
pcl::octree::OctreePointCloudSearch<PointT, LeafT, BranchT>::boxSearchRecursive (const Eigen::Vector3f &min_pt,
                                                                                 const Eigen::Vector3f &max_pt,
                                                                                 const BranchNode* node,
                                                                                 const OctreeKey& key,
                                                                                 unsigned int treeDepth,
                                                                                 std::vector<int>& k_indices) const
{
  // child iterator
  unsigned char childIdx;

  // iterate over all children
  for (childIdx = 0; childIdx < 8; childIdx++)
  {

    const OctreeNode* childNode;
    childNode = this->getBranchChildPtr (*node, childIdx);

    if (!childNode)
      continue;

    OctreeKey newKey;
    // generate new key for current branch voxel
    newKey.x = (key.x << 1) + (!!(childIdx & (1 << 2)));
    newKey.y = (key.y << 1) + (!!(childIdx & (1 << 1)));
    newKey.z = (key.z << 1) + (!!(childIdx & (1 << 0)));

    // voxel corners
    Eigen::Vector3f lowerVoxelCorner;
    Eigen::Vector3f upperVoxelCorner;
    // get voxel coordinates
    this->genVoxelBoundsFromOctreeKey (newKey, treeDepth, lowerVoxelCorner, upperVoxelCorner);

    // test if search region overlap with voxel space

    if ( !( (lowerVoxelCorner (0) > max_pt (0)) || (min_pt (0) > upperVoxelCorner(0)) ||
            (lowerVoxelCorner (1) > max_pt (1)) || (min_pt (1) > upperVoxelCorner(1)) ||
            (lowerVoxelCorner (2) > max_pt (2)) || (min_pt (2) > upperVoxelCorner(2)) ) )
    {

      if (treeDepth < this->octreeDepth_)
      {
        // we have not reached maximum tree depth
        boxSearchRecursive (min_pt, max_pt, static_cast<const BranchNode*> (childNode), newKey, treeDepth + 1, k_indices);
      }
      else
      {
        // we reached leaf node level
        size_t i;
        vector<int> decodedPointVector;
        bool bInBox;

        const LeafNode* childLeaf = static_cast<const LeafNode*> (childNode);

        // decode leaf node into decodedPointVector
        childLeaf->getData (decodedPointVector);

        // Linearly iterate over all decoded (unsorted) points
        for (i = 0; i < decodedPointVector.size (); i++)
        {
          const PointT& candidatePoint = this->getPointByIndex (decodedPointVector[i]);

          // check if point falls within search box
          bInBox = ( (candidatePoint.x > min_pt (0)) && (candidatePoint.x < max_pt (0)) &&
                     (candidatePoint.y > min_pt (1)) && (candidatePoint.y < max_pt (1)) &&
                     (candidatePoint.z > min_pt (2)) && (candidatePoint.z < max_pt (2)) );

          if (bInBox)
            // add to result vector
            k_indices.push_back (decodedPointVector[i]);
        }
      }
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafT, typename BranchT> int
pcl::octree::OctreePointCloudSearch<PointT, LeafT, BranchT>::getIntersectedVoxelCenters (
    Eigen::Vector3f origin, Eigen::Vector3f direction, AlignedPointTVector &voxelCenterList,
    int maxVoxelCount) const
{
  OctreeKey key;
  key.x = key.y = key.z = 0;

  voxelCenterList.clear ();

  // Voxel childIdx remapping
  unsigned char a = 0;

  double minX, minY, minZ, maxX, maxY, maxZ;

  initIntersectedVoxel (origin, direction, minX, minY, minZ, maxX, maxY, maxZ, a);

  if (max (max (minX, minY), minZ) < min (min (maxX, maxY), maxZ))
    return getIntersectedVoxelCentersRecursive (minX, minY, minZ, maxX, maxY, maxZ, a, this->rootNode_, key,
                                                voxelCenterList, maxVoxelCount);

  return (0);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafT, typename BranchT> int
pcl::octree::OctreePointCloudSearch<PointT, LeafT, BranchT>::getIntersectedVoxelIndices (
    Eigen::Vector3f origin, Eigen::Vector3f direction, std::vector<int> &k_indices,
    int maxVoxelCount) const
{
  OctreeKey key;
  key.x = key.y = key.z = 0;

  k_indices.clear ();

  // Voxel childIdx remapping
  unsigned char a = 0;
  double minX, minY, minZ, maxX, maxY, maxZ;

  initIntersectedVoxel (origin, direction, minX, minY, minZ, maxX, maxY, maxZ, a);

  if (max (max (minX, minY), minZ) < min (min (maxX, maxY), maxZ))
    return getIntersectedVoxelIndicesRecursive (minX, minY, minZ, maxX, maxY, maxZ, a, this->rootNode_, key,
                                                k_indices, maxVoxelCount);
  return (0);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafT, typename BranchT> int
pcl::octree::OctreePointCloudSearch<PointT, LeafT, BranchT>::getIntersectedVoxelCentersRecursive (
    double minX, double minY, double minZ, double maxX, double maxY, double maxZ, unsigned char a,
    const OctreeNode* node, const OctreeKey& key, AlignedPointTVector &voxelCenterList, int maxVoxelCount) const
{
  if (maxX < 0.0 || maxY < 0.0 || maxZ < 0.0)
    return (0);

  // If leaf node, get voxel center and increment intersection count
  if (node->getNodeType () == LEAF_NODE)
  {
    PointT newPoint;

    this->genLeafNodeCenterFromOctreeKey (key, newPoint);

    voxelCenterList.push_back (newPoint);

    return (1);
  }

  // Voxel intersection count for branches children
  int voxelCount = 0;

  // Voxel mid lines
  double midX = 0.5 * (minX + maxX);
  double midY = 0.5 * (minY + maxY);
  double midZ = 0.5 * (minZ + maxZ);

  // First voxel node ray will intersect
  int currNode = getFirstIntersectedNode (minX, minY, minZ, midX, midY, midZ);

  // Child index, node and key
  unsigned char childIdx;
  const OctreeNode *childNode;
  OctreeKey childKey;

  do
  {
    if (currNode != 0)
      childIdx = static_cast<unsigned char> (currNode ^ a);
    else
      childIdx = a;

    // childNode == 0 if childNode doesn't exist
    childNode = this->getBranchChildPtr (static_cast<const BranchNode&> (*node), childIdx);

    // Generate new key for current branch voxel
    childKey.x = (key.x << 1) | (!!(childIdx & (1 << 2)));
    childKey.y = (key.y << 1) | (!!(childIdx & (1 << 1)));
    childKey.z = (key.z << 1) | (!!(childIdx & (1 << 0)));

    // Recursively call each intersected child node, selecting the next
    //   node intersected by the ray.  Children that do not intersect will
    //   not be traversed.

    switch (currNode)
    {
      case 0:
        if (childNode)
          voxelCount += getIntersectedVoxelCentersRecursive (minX, minY, minZ, midX, midY, midZ, a, childNode,
                                                             childKey, voxelCenterList, maxVoxelCount);
        currNode = getNextIntersectedNode (midX, midY, midZ, 4, 2, 1);
        break;

      case 1:
        if (childNode)
          voxelCount += getIntersectedVoxelCentersRecursive (minX, minY, midZ, midX, midY, maxZ, a, childNode,
                                                             childKey, voxelCenterList, maxVoxelCount);
        currNode = getNextIntersectedNode (midX, midY, maxZ, 5, 3, 8);
        break;

      case 2:
        if (childNode)
          voxelCount += getIntersectedVoxelCentersRecursive (minX, midY, minZ, midX, maxY, midZ, a, childNode,
                                                             childKey, voxelCenterList, maxVoxelCount);
        currNode = getNextIntersectedNode (midX, maxY, midZ, 6, 8, 3);
        break;

      case 3:
        if (childNode)
          voxelCount += getIntersectedVoxelCentersRecursive (minX, midY, midZ, midX, maxY, maxZ, a, childNode,
                                                             childKey, voxelCenterList, maxVoxelCount);
        currNode = getNextIntersectedNode (midX, maxY, maxZ, 7, 8, 8);
        break;

      case 4:
        if (childNode)
          voxelCount += getIntersectedVoxelCentersRecursive (midX, minY, minZ, maxX, midY, midZ, a, childNode,
                                                             childKey, voxelCenterList, maxVoxelCount);
        currNode = getNextIntersectedNode (maxX, midY, midZ, 8, 6, 5);
        break;

      case 5:
        if (childNode)
          voxelCount += getIntersectedVoxelCentersRecursive (midX, minY, midZ, maxX, midY, maxZ, a, childNode,
                                                             childKey, voxelCenterList, maxVoxelCount);
        currNode = getNextIntersectedNode (maxX, midY, maxZ, 8, 7, 8);
        break;

      case 6:
        if (childNode)
          voxelCount += getIntersectedVoxelCentersRecursive (midX, midY, minZ, maxX, maxY, midZ, a, childNode,
                                                             childKey, voxelCenterList, maxVoxelCount);
        currNode = getNextIntersectedNode (maxX, maxY, midZ, 8, 8, 7);
        break;

      case 7:
        if (childNode)
          voxelCount += getIntersectedVoxelCentersRecursive (midX, midY, midZ, maxX, maxY, maxZ, a, childNode,
                                                             childKey, voxelCenterList, maxVoxelCount);
        currNode = 8;
        break;
    }
  } while ((currNode < 8) && (maxVoxelCount <= 0 || voxelCount < maxVoxelCount));
  return (voxelCount);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafT, typename BranchT> int
pcl::octree::OctreePointCloudSearch<PointT, LeafT, BranchT>::getIntersectedVoxelIndicesRecursive (
    double minX, double minY, double minZ, double maxX, double maxY, double maxZ, unsigned char a,
    const OctreeNode* node, const OctreeKey& key, std::vector<int> &k_indices, int maxVoxelCount) const
{
  if (maxX < 0.0 || maxY < 0.0 || maxZ < 0.0)
    return (0);

  // If leaf node, get voxel center and increment intersection count
  if (node->getNodeType () == LEAF_NODE)
  {
    const LeafNode* leaf = static_cast<const LeafNode*> (node);

    // decode leaf node into k_indices
    leaf->getData (k_indices);

    return (1);
  }

  // Voxel intersection count for branches children
  int voxelCount = 0;

  // Voxel mid lines
  double midX = 0.5 * (minX + maxX);
  double midY = 0.5 * (minY + maxY);
  double midZ = 0.5 * (minZ + maxZ);

  // First voxel node ray will intersect
  int currNode = getFirstIntersectedNode (minX, minY, minZ, midX, midY, midZ);

  // Child index, node and key
  unsigned char childIdx;
  const OctreeNode *childNode;
  OctreeKey childKey;
  do
  {
    if (currNode != 0)
      childIdx = static_cast<unsigned char> (currNode ^ a);
    else
      childIdx = a;

    // childNode == 0 if childNode doesn't exist
    childNode = this->getBranchChildPtr (static_cast<const BranchNode&> (*node), childIdx);
    // Generate new key for current branch voxel
    childKey.x = (key.x << 1) | (!!(childIdx & (1 << 2)));
    childKey.y = (key.y << 1) | (!!(childIdx & (1 << 1)));
    childKey.z = (key.z << 1) | (!!(childIdx & (1 << 0)));

    // Recursively call each intersected child node, selecting the next
    //   node intersected by the ray.  Children that do not intersect will
    //   not be traversed.
    switch (currNode)
    {
      case 0:
        if (childNode)
          voxelCount += getIntersectedVoxelIndicesRecursive (minX, minY, minZ, midX, midY, midZ, a, childNode,
                                                             childKey, k_indices, maxVoxelCount);
        currNode = getNextIntersectedNode (midX, midY, midZ, 4, 2, 1);
        break;

      case 1:
        if (childNode)
          voxelCount += getIntersectedVoxelIndicesRecursive (minX, minY, midZ, midX, midY, maxZ, a, childNode,
                                                             childKey, k_indices, maxVoxelCount);
        currNode = getNextIntersectedNode (midX, midY, maxZ, 5, 3, 8);
        break;

      case 2:
        if (childNode)
          voxelCount += getIntersectedVoxelIndicesRecursive (minX, midY, minZ, midX, maxY, midZ, a, childNode,
                                                             childKey, k_indices, maxVoxelCount);
        currNode = getNextIntersectedNode (midX, maxY, midZ, 6, 8, 3);
        break;

      case 3:
        if (childNode)
          voxelCount += getIntersectedVoxelIndicesRecursive (minX, midY, midZ, midX, maxY, maxZ, a, childNode,
                                                             childKey, k_indices, maxVoxelCount);
        currNode = getNextIntersectedNode (midX, maxY, maxZ, 7, 8, 8);
        break;

      case 4:
        if (childNode)
          voxelCount += getIntersectedVoxelIndicesRecursive (midX, minY, minZ, maxX, midY, midZ, a, childNode,
                                                             childKey, k_indices, maxVoxelCount);
        currNode = getNextIntersectedNode (maxX, midY, midZ, 8, 6, 5);
        break;

      case 5:
        if (childNode)
          voxelCount += getIntersectedVoxelIndicesRecursive (midX, minY, midZ, maxX, midY, maxZ, a, childNode,
                                                             childKey, k_indices, maxVoxelCount);
        currNode = getNextIntersectedNode (maxX, midY, maxZ, 8, 7, 8);
        break;

      case 6:
        if (childNode)
          voxelCount += getIntersectedVoxelIndicesRecursive (midX, midY, minZ, maxX, maxY, midZ, a, childNode,
                                                             childKey, k_indices, maxVoxelCount);
        currNode = getNextIntersectedNode (maxX, maxY, midZ, 8, 8, 7);
        break;

      case 7:
        if (childNode)
          voxelCount += getIntersectedVoxelIndicesRecursive (midX, midY, midZ, maxX, maxY, maxZ, a, childNode,
                                                             childKey, k_indices, maxVoxelCount);
        currNode = 8;
        break;
    }
  } while ((currNode < 8) && (maxVoxelCount <= 0 || voxelCount < maxVoxelCount));

  return (voxelCount);
}

#endif    // PCL_OCTREE_SEARCH_IMPL_H_
