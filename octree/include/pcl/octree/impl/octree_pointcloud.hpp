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

#ifndef OCTREE_POINTCLOUD_HPP_
#define OCTREE_POINTCLOUD_HPP_

#include <vector>
#include <assert.h>

#include <pcl/common/common.h>

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafT, typename BranchT, typename OctreeT>
pcl::octree::OctreePointCloud<PointT, LeafT, BranchT, OctreeT>::OctreePointCloud (const double resolution) :
    OctreeT (), input_ (PointCloudConstPtr ()), indices_ (IndicesConstPtr ()),
    epsilon_ (0), resolution_ (resolution), minX_ (0.0f), maxX_ (resolution), minY_ (0.0f),
    maxY_ (resolution), minZ_ (0.0f), maxZ_ (resolution), boundingBoxDefined_ (false)
{
  assert (resolution > 0.0f);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafT, typename BranchT, typename OctreeT>
pcl::octree::OctreePointCloud<PointT, LeafT, BranchT, OctreeT>::~OctreePointCloud ()
{
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafT, typename BranchT, typename OctreeT> void
pcl::octree::OctreePointCloud<PointT, LeafT, BranchT, OctreeT>::addPointsFromInputCloud ()
{
  size_t i;

  assert (this->leafCount_==0);
  if (indices_)
  {
    for (std::vector<int>::const_iterator current = indices_->begin (); current != indices_->end (); ++current)
    {
      if (isFinite (input_->points[*current]))
      {
        assert( (*current>=0) && (*current < static_cast<int> (input_->points.size ())));

        // add points to octree
        this->addPointIdx (*current);
      }
    }
  }
  else
  {
    for (i = 0; i < input_->points.size (); i++)
    {
      if (isFinite (input_->points[i]))
      {
        // add points to octree
        this->addPointIdx (static_cast<unsigned int> (i));
      }
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafT, typename BranchT, typename OctreeT> void
pcl::octree::OctreePointCloud<PointT, LeafT, BranchT, OctreeT>::addPointFromCloud (const int pointIdx_arg, IndicesPtr indices_arg)
{
  this->addPointIdx (pointIdx_arg);
  if (indices_arg)
    indices_arg->push_back (pointIdx_arg);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafT, typename BranchT, typename OctreeT> void
pcl::octree::OctreePointCloud<PointT, LeafT, BranchT, OctreeT>::addPointToCloud (const PointT& point_arg, PointCloudPtr cloud_arg)
{
  assert (cloud_arg==input_);

  cloud_arg->push_back (point_arg);

  this->addPointIdx (static_cast<const int> (cloud_arg->points.size ()) - 1);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafT, typename BranchT, typename OctreeT> void
pcl::octree::OctreePointCloud<PointT, LeafT, BranchT, OctreeT>::addPointToCloud (const PointT& point_arg, PointCloudPtr cloud_arg,
                                                           IndicesPtr indices_arg)
{
  assert (cloud_arg==input_);
  assert (indices_arg==indices_);

  cloud_arg->push_back (point_arg);

  this->addPointFromCloud (static_cast<const int> (cloud_arg->points.size ()) - 1, indices_arg);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafT, typename BranchT, typename OctreeT> bool
pcl::octree::OctreePointCloud<PointT, LeafT, BranchT, OctreeT>::isVoxelOccupiedAtPoint (const PointT& point_arg) const
{
  OctreeKey key;

  // generate key for point
  this->genOctreeKeyforPoint (point_arg, key);

  // search for key in octree
  return (isPointWithinBoundingBox (point_arg) && this->existLeaf (key));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafT, typename BranchT, typename OctreeT> bool
pcl::octree::OctreePointCloud<PointT, LeafT, BranchT, OctreeT>::isVoxelOccupiedAtPoint (const int& pointIdx_arg) const
{
  // retrieve point from input cloud
  const PointT& point = this->input_->points[pointIdx_arg];

  // search for voxel at point in octree
  return (this->isVoxelOccupiedAtPoint (point));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafT, typename BranchT, typename OctreeT> bool
pcl::octree::OctreePointCloud<PointT, LeafT, BranchT, OctreeT>::isVoxelOccupiedAtPoint (
    const double pointX_arg, const double pointY_arg, const double pointZ_arg) const
{
  OctreeKey key;

  // generate key for point
  this->genOctreeKeyforPoint (pointX_arg, pointY_arg, pointZ_arg, key);

  return (this->existLeaf (key));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafT, typename BranchT, typename OctreeT> void
pcl::octree::OctreePointCloud<PointT, LeafT, BranchT, OctreeT>::deleteVoxelAtPoint (const PointT& point_arg)
{
  OctreeKey key;

  // generate key for point
  this->genOctreeKeyforPoint (point_arg, key);

  this->removeLeaf (key);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafT, typename BranchT, typename OctreeT> void
pcl::octree::OctreePointCloud<PointT, LeafT, BranchT, OctreeT>::deleteVoxelAtPoint (const int& pointIdx_arg)
{
  // retrieve point from input cloud
  const PointT& point = this->input_->points[pointIdx_arg];

  // delete leaf at point
  this->deleteVoxelAtPoint (point);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafT, typename BranchT, typename OctreeT> int
pcl::octree::OctreePointCloud<PointT, LeafT, BranchT, OctreeT>::getOccupiedVoxelCenters (
    AlignedPointTVector &voxelCenterList_arg) const
{
  OctreeKey key;
  key.x = key.y = key.z = 0;

  voxelCenterList_arg.clear ();

  return getOccupiedVoxelCentersRecursive (this->rootNode_, key, voxelCenterList_arg);

}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafT, typename BranchT, typename OctreeT> int
pcl::octree::OctreePointCloud<PointT, LeafT, BranchT, OctreeT>::getApproxIntersectedVoxelCentersBySegment (
    const Eigen::Vector3f& origin,
    const Eigen::Vector3f& end,
    AlignedPointTVector &voxel_center_list,
    float precision)
{
  Eigen::Vector3f direction = end - origin;
  float norm = direction.norm ();
  direction.normalize ();

  const float step_size = static_cast<const float> (resolution_) * precision;
  // Ensure we get at least one step for the first voxel.
  const int nsteps = std::max (1, static_cast<int> (norm / step_size));

  OctreeKey prev_key;

  bool bkeyDefined = false;

  // Walk along the line segment with small steps.
  for (int i = 0; i < nsteps; ++i)
  {
    Eigen::Vector3f p = origin + (direction * step_size * static_cast<const float> (i));

    PointT octree_p;
    octree_p.x = p.x ();
    octree_p.y = p.y ();
    octree_p.z = p.z ();

    OctreeKey key;
    this->genOctreeKeyforPoint (octree_p, key);

    // Not a new key, still the same voxel.
    if ((key == prev_key) && (bkeyDefined) )
      continue;

    prev_key = key;
    bkeyDefined = true;

    PointT center;
    genLeafNodeCenterFromOctreeKey (key, center);
    voxel_center_list.push_back (center);
  }

  OctreeKey end_key;
  PointT end_p;
  end_p.x = end.x ();
  end_p.y = end.y ();
  end_p.z = end.z ();
  this->genOctreeKeyforPoint (end_p, end_key);
  if (!(end_key == prev_key))
  {
    PointT center;
    genLeafNodeCenterFromOctreeKey (end_key, center);
    voxel_center_list.push_back (center);
  }

  return (static_cast<int> (voxel_center_list.size ()));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafT, typename BranchT, typename OctreeT> void
pcl::octree::OctreePointCloud<PointT, LeafT, BranchT, OctreeT>::defineBoundingBox ()
{

  double minX, minY, minZ, maxX, maxY, maxZ;

  PointT min_pt;
  PointT max_pt;

  // bounding box cannot be changed once the octree contains elements
  assert (this->leafCount_ == 0);

  pcl::getMinMax3D (*input_, min_pt, max_pt);

  float minValue = std::numeric_limits<float>::epsilon () * 512.0f;

  minX = min_pt.x;
  minY = min_pt.y;
  minZ = min_pt.z;

  maxX = max_pt.x + minValue;
  maxY = max_pt.y + minValue;
  maxZ = max_pt.z + minValue;

  // generate bit masks for octree
  defineBoundingBox (minX, minY, minZ, maxX, maxY, maxZ);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafT, typename BranchT, typename OctreeT> void
pcl::octree::OctreePointCloud<PointT, LeafT, BranchT, OctreeT>::defineBoundingBox (const double minX_arg,
                                                                          const double minY_arg,
                                                                          const double minZ_arg,
                                                                          const double maxX_arg,
                                                                          const double maxY_arg, 
                                                                          const double maxZ_arg)
{
  // bounding box cannot be changed once the octree contains elements
  assert (this->leafCount_ == 0);

  assert (maxX_arg >= minX_arg);
  assert (maxY_arg >= minY_arg);
  assert (maxZ_arg >= minZ_arg);

  minX_ = minX_arg;
  maxX_ = maxX_arg;

  minY_ = minY_arg;
  maxY_ = maxY_arg;

  minZ_ = minZ_arg;
  maxZ_ = maxZ_arg;

  minX_ = min (minX_, maxX_);
  minY_ = min (minY_, maxY_);
  minZ_ = min (minZ_, maxZ_);

  maxX_ = max (minX_, maxX_);
  maxY_ = max (minY_, maxY_);
  maxZ_ = max (minZ_, maxZ_);

  // generate bit masks for octree
  getKeyBitSize ();

  boundingBoxDefined_ = true;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafT, typename BranchT, typename OctreeT> void
pcl::octree::OctreePointCloud<PointT, LeafT, BranchT, OctreeT>::defineBoundingBox (
    const double maxX_arg, const double maxY_arg, const double maxZ_arg)
{
  // bounding box cannot be changed once the octree contains elements
  assert (this->leafCount_ == 0);

  assert (maxX_arg >= 0.0f);
  assert (maxY_arg >= 0.0f);
  assert (maxZ_arg >= 0.0f);

  minX_ = 0.0f;
  maxX_ = maxX_arg;

  minY_ = 0.0f;
  maxY_ = maxY_arg;

  minZ_ = 0.0f;
  maxZ_ = maxZ_arg;

  minX_ = min (minX_, maxX_);
  minY_ = min (minY_, maxY_);
  minZ_ = min (minZ_, maxZ_);

  maxX_ = max (minX_, maxX_);
  maxY_ = max (minY_, maxY_);
  maxZ_ = max (minZ_, maxZ_);

  // generate bit masks for octree
  getKeyBitSize ();

  boundingBoxDefined_ = true;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafT, typename BranchT, typename OctreeT> void
pcl::octree::OctreePointCloud<PointT, LeafT, BranchT, OctreeT>::defineBoundingBox (const double cubeLen_arg)
{
  // bounding box cannot be changed once the octree contains elements
  assert (this->leafCount_ == 0);

  assert (cubeLen_arg >= 0.0f);

  minX_ = 0.0f;
  maxX_ = cubeLen_arg;

  minY_ = 0.0f;
  maxY_ = cubeLen_arg;

  minZ_ = 0.0f;
  maxZ_ = cubeLen_arg;

  minX_ = min (minX_, maxX_);
  minY_ = min (minY_, maxY_);
  minZ_ = min (minZ_, maxZ_);

  maxX_ = max (minX_, maxX_);
  maxY_ = max (minY_, maxY_);
  maxZ_ = max (minZ_, maxZ_);

  // generate bit masks for octree
  getKeyBitSize ();

  boundingBoxDefined_ = true;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafT, typename BranchT, typename OctreeT> void
pcl::octree::OctreePointCloud<PointT, LeafT, BranchT, OctreeT>::getBoundingBox (
    double& minX_arg, double& minY_arg, double& minZ_arg,
    double& maxX_arg, double& maxY_arg, double& maxZ_arg) const
{
  minX_arg = minX_;
  minY_arg = minY_;
  minZ_arg = minZ_;

  maxX_arg = maxX_;
  maxY_arg = maxY_;
  maxZ_arg = maxZ_;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafT, typename BranchT, typename OctreeT>
void
pcl::octree::OctreePointCloud<PointT, LeafT, BranchT, OctreeT>::adoptBoundingBoxToPoint (const PointT& pointIdx_arg)
{

  const float minValue = std::numeric_limits<float>::epsilon ();

  // increase octree size until point fits into bounding box
  while (true)
  {
    bool bLowerBoundViolationX = (pointIdx_arg.x < minX_);
    bool bLowerBoundViolationY = (pointIdx_arg.y < minY_);
    bool bLowerBoundViolationZ = (pointIdx_arg.z < minZ_);

    bool bUpperBoundViolationX = (pointIdx_arg.x >= maxX_);
    bool bUpperBoundViolationY = (pointIdx_arg.y >= maxY_);
    bool bUpperBoundViolationZ = (pointIdx_arg.z >= maxZ_);

    // do we violate any bounds?
    if (bLowerBoundViolationX || bLowerBoundViolationY || bLowerBoundViolationZ || bUpperBoundViolationX
        || bUpperBoundViolationY || bUpperBoundViolationZ )
    {

      if (boundingBoxDefined_)
      {

        double octreeSideLen;
        unsigned char childIdx;

        // octree not empty - we add another tree level and thus increase its size by a factor of 2*2*2
        childIdx = static_cast<unsigned char> (((!bUpperBoundViolationX) << 2) | ((!bUpperBoundViolationY) << 1)
            | ((!bUpperBoundViolationZ)));

        BranchNode* newRootBranch;

        newRootBranch = this->branchNodePool_.popNode();
        this->branchCount_++;

        this->setBranchChildPtr (*newRootBranch, childIdx, this->rootNode_);

        this->rootNode_ = newRootBranch;

        octreeSideLen = static_cast<double> (1 << this->octreeDepth_) * resolution_;

        if (!bUpperBoundViolationX)
          minX_ -= octreeSideLen;

        if (!bUpperBoundViolationY)
          minY_ -= octreeSideLen;

        if (!bUpperBoundViolationZ)
          minZ_ -= octreeSideLen;

        // configure tree depth of octree
        this->octreeDepth_++;
        this->setTreeDepth (this->octreeDepth_);

        // recalculate bounding box width
        octreeSideLen = static_cast<double> (1 << this->octreeDepth_) * resolution_ - minValue;

        // increase octree bounding box
        maxX_ = minX_ + octreeSideLen;
        maxY_ = minY_ + octreeSideLen;
        maxZ_ = minZ_ + octreeSideLen;

      }
      // bounding box is not defined - set it to point position
      else
      {
        // octree is empty - we set the center of the bounding box to our first pixel
        this->minX_ = pointIdx_arg.x - this->resolution_ / 2;
        this->minY_ = pointIdx_arg.y - this->resolution_ / 2;
        this->minZ_ = pointIdx_arg.z - this->resolution_ / 2;

        this->maxX_ = pointIdx_arg.x + this->resolution_ / 2;
        this->maxY_ = pointIdx_arg.y + this->resolution_ / 2;
        this->maxZ_ = pointIdx_arg.z + this->resolution_ / 2;

        getKeyBitSize ();

        boundingBoxDefined_ = true;
      }

    }
    else
      // no bound violations anymore - leave while loop
      break;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafT, typename BranchT, typename OctreeT> void
pcl::octree::OctreePointCloud<PointT, LeafT, BranchT, OctreeT>::addPointIdx (const int pointIdx_arg)
{
  OctreeKey key;

  assert (pointIdx_arg < static_cast<int> (input_->points.size ()));

  const PointT& point = input_->points[pointIdx_arg];

  // make sure bounding box is big enough
  adoptBoundingBoxToPoint (point);

  // generate key
  genOctreeKeyforPoint (point, key);

  // add point to octree at key
  this->addData (key, pointIdx_arg);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafT, typename BranchT, typename OctreeT> const PointT&
pcl::octree::OctreePointCloud<PointT, LeafT, BranchT, OctreeT>::getPointByIndex (const unsigned int index_arg) const
{
  // retrieve point from input cloud
  assert (index_arg < static_cast<unsigned int> (input_->points.size ()));
  return (this->input_->points[index_arg]);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafT, typename BranchT, typename OctreeT> LeafT*
pcl::octree::OctreePointCloud<PointT, LeafT, BranchT, OctreeT>::findLeafAtPoint (const PointT& point) const
{
  OctreeKey key;

  // generate key for point
  this->genOctreeKeyforPoint (point, key);

  return (this->findLeaf (key));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafT, typename BranchT, typename OctreeT> void
pcl::octree::OctreePointCloud<PointT, LeafT, BranchT, OctreeT>::getKeyBitSize ()
{
  unsigned int maxVoxels;

  unsigned int maxKeyX;
  unsigned int maxKeyY;
  unsigned int maxKeyZ;

  double octreeSideLen;

  const float minValue = std::numeric_limits<float>::epsilon();

  // find maximum key values for x, y, z
  maxKeyX = static_cast<unsigned int> ((maxX_ - minX_) / resolution_);
  maxKeyY = static_cast<unsigned int> ((maxY_ - minY_) / resolution_);
  maxKeyZ = static_cast<unsigned int> ((maxZ_ - minZ_) / resolution_);

  // find maximum amount of keys
  maxVoxels = max (max (max (maxKeyX, maxKeyY), maxKeyZ), static_cast<unsigned int> (2));


  // tree depth == amount of bits of maxVoxels
  this->octreeDepth_ = max ((min (static_cast<unsigned int> (OCT_MAXTREEDEPTH), static_cast<unsigned int> (ceil (this->Log2 (maxVoxels)-minValue)))),
                                  static_cast<unsigned int> (0));

  octreeSideLen = static_cast<double> (1 << this->octreeDepth_) * resolution_-minValue;

  if (this->leafCount_ == 0)
  {
    double octreeOversizeX;
    double octreeOversizeY;
    double octreeOversizeZ;

    octreeOversizeX = (octreeSideLen - (maxX_ - minX_)) / 2.0;
    octreeOversizeY = (octreeSideLen - (maxY_ - minY_)) / 2.0;
    octreeOversizeZ = (octreeSideLen - (maxZ_ - minZ_)) / 2.0;

    minX_ -= octreeOversizeX;
    minY_ -= octreeOversizeY;
    minZ_ -= octreeOversizeZ;

    maxX_ += octreeOversizeX;
    maxY_ += octreeOversizeY;
    maxZ_ += octreeOversizeZ;
  }
  else
  {
    maxX_ = minX_ + octreeSideLen;
    maxY_ = minY_ + octreeSideLen;
    maxZ_ = minZ_ + octreeSideLen;
  }

 // configure tree depth of octree
  this->setTreeDepth (this->octreeDepth_);

}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafT, typename BranchT, typename OctreeT> void
pcl::octree::OctreePointCloud<PointT, LeafT, BranchT, OctreeT>::genOctreeKeyforPoint (const PointT& point_arg,
                                                                             OctreeKey & key_arg) const
  {
    // calculate integer key for point coordinates
    key_arg.x = static_cast<unsigned int> ((point_arg.x - this->minX_) / this->resolution_);
    key_arg.y = static_cast<unsigned int> ((point_arg.y - this->minY_) / this->resolution_);
    key_arg.z = static_cast<unsigned int> ((point_arg.z - this->minZ_) / this->resolution_);
  }

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafT, typename BranchT, typename OctreeT> void
pcl::octree::OctreePointCloud<PointT, LeafT, BranchT, OctreeT>::genOctreeKeyforPoint (
    const double pointX_arg, const double pointY_arg,
    const double pointZ_arg, OctreeKey & key_arg) const
{
  PointT tempPoint;

  tempPoint.x = static_cast<float> (pointX_arg);
  tempPoint.y = static_cast<float> (pointY_arg);
  tempPoint.z = static_cast<float> (pointZ_arg);

  // generate key for point
  genOctreeKeyforPoint (tempPoint, key_arg);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafT, typename BranchT, typename OctreeT> bool
pcl::octree::OctreePointCloud<PointT, LeafT, BranchT, OctreeT>::genOctreeKeyForDataT (const int& data_arg, OctreeKey & key_arg) const
{
  const PointT tempPoint = getPointByIndex (data_arg);

  // generate key for point
  genOctreeKeyforPoint (tempPoint, key_arg);

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafT, typename BranchT, typename OctreeT> void
pcl::octree::OctreePointCloud<PointT, LeafT, BranchT, OctreeT>::genLeafNodeCenterFromOctreeKey (const OctreeKey & key, PointT & point) const
{
  // define point to leaf node voxel center
  point.x = static_cast<float> ((static_cast<double> (key.x) + 0.5f) * this->resolution_ + this->minX_);
  point.y = static_cast<float> ((static_cast<double> (key.y) + 0.5f) * this->resolution_ + this->minY_);
  point.z = static_cast<float> ((static_cast<double> (key.z) + 0.5f) * this->resolution_ + this->minZ_);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafT, typename BranchT, typename OctreeT> void
pcl::octree::OctreePointCloud<PointT, LeafT, BranchT, OctreeT>::genVoxelCenterFromOctreeKey (
    const OctreeKey & key_arg,
    unsigned int treeDepth_arg,
    PointT& point_arg) const
{
  // generate point for voxel center defined by treedepth (bitLen) and key
  point_arg.x = static_cast<float> ((static_cast <double> (key_arg.x) + 0.5f) * (this->resolution_ * static_cast<double> (1 << (this->octreeDepth_ - treeDepth_arg))) + this->minX_);
  point_arg.y = static_cast<float> ((static_cast <double> (key_arg.y) + 0.5f) * (this->resolution_ * static_cast<double> (1 << (this->octreeDepth_ - treeDepth_arg))) + this->minY_);
  point_arg.z = static_cast<float> ((static_cast <double> (key_arg.z) + 0.5f) * (this->resolution_ * static_cast<double> (1 << (this->octreeDepth_ - treeDepth_arg))) + this->minZ_);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafT, typename BranchT, typename OctreeT> void
pcl::octree::OctreePointCloud<PointT, LeafT, BranchT, OctreeT>::genVoxelBoundsFromOctreeKey (
    const OctreeKey & key_arg,
    unsigned int treeDepth_arg,
    Eigen::Vector3f &min_pt,
    Eigen::Vector3f &max_pt) const
{
  // calculate voxel size of current tree depth
  double voxel_side_len = this->resolution_ * static_cast<double> (1 << (this->octreeDepth_ - treeDepth_arg));

  // calculate voxel bounds
  min_pt (0) = static_cast<float> (static_cast<double> (key_arg.x) * voxel_side_len + this->minX_);
  min_pt (1) = static_cast<float> (static_cast<double> (key_arg.y) * voxel_side_len + this->minY_);
  min_pt (2) = static_cast<float> (static_cast<double> (key_arg.z) * voxel_side_len + this->minZ_);

  max_pt (0) = static_cast<float> (static_cast<double> (key_arg.x + 1) * voxel_side_len + this->minX_);
  max_pt (1) = static_cast<float> (static_cast<double> (key_arg.y + 1) * voxel_side_len + this->minY_);
  max_pt (2) = static_cast<float> (static_cast<double> (key_arg.z + 1) * voxel_side_len + this->minZ_);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafT, typename BranchT, typename OctreeT> double
pcl::octree::OctreePointCloud<PointT, LeafT, BranchT, OctreeT>::getVoxelSquaredSideLen (unsigned int treeDepth_arg) const
{
  double sideLen;

  // side length of the voxel cube increases exponentially with the octree depth
  sideLen = this->resolution_ * static_cast<double>(1 << (this->octreeDepth_ - treeDepth_arg));

  // squared voxel side length
  sideLen *= sideLen;

  return (sideLen);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafT, typename BranchT, typename OctreeT> double
pcl::octree::OctreePointCloud<PointT, LeafT, BranchT, OctreeT>::getVoxelSquaredDiameter (unsigned int treeDepth_arg) const
{
  // return the squared side length of the voxel cube as a function of the octree depth
  return (getVoxelSquaredSideLen (treeDepth_arg) * 3);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafT, typename BranchT, typename OctreeT> int
pcl::octree::OctreePointCloud<PointT, LeafT, BranchT, OctreeT>::getOccupiedVoxelCentersRecursive (
    const BranchNode* node_arg,
    const OctreeKey& key_arg,
    AlignedPointTVector &voxelCenterList_arg) const
{
  // child iterator
  unsigned char childIdx;

  int voxelCount = 0;

  // iterate over all children
  for (childIdx = 0; childIdx < 8; childIdx++)
  {
    if (!this->branchHasChild (*node_arg, childIdx))
      continue;

    const OctreeNode * childNode;
    childNode = this->getBranchChildPtr (*node_arg, childIdx);

    // generate new key for current branch voxel
    OctreeKey newKey;
    newKey.x = (key_arg.x << 1) | (!!(childIdx & (1 << 2)));
    newKey.y = (key_arg.y << 1) | (!!(childIdx & (1 << 1)));
    newKey.z = (key_arg.z << 1) | (!!(childIdx & (1 << 0)));

    switch (childNode->getNodeType ())
    {
      case BRANCH_NODE:
      {
        // recursively proceed with indexed child branch
        voxelCount += getOccupiedVoxelCentersRecursive (static_cast<const BranchNode*> (childNode), newKey, voxelCenterList_arg);
        break;
      }
      case LEAF_NODE:
      {
        PointT newPoint;

        genLeafNodeCenterFromOctreeKey (newKey, newPoint);
        voxelCenterList_arg.push_back (newPoint);

        voxelCount++;
        break;
      }
      default:
        break;
    }
  }
  return (voxelCount);
}

#define PCL_INSTANTIATE_OctreePointCloudSingleBufferWithLeafDataTVector(T) template class PCL_EXPORTS pcl::octree::OctreePointCloud<T, pcl::octree::OctreeContainerDataTVector<int> , pcl::octree::OctreeContainerEmpty<int>, pcl::octree::OctreeBase<int, pcl::octree::OctreeContainerDataTVector<int>, pcl::octree::OctreeContainerEmpty<int> > >;
#define PCL_INSTANTIATE_OctreePointCloudDoubleBufferWithLeafDataTVector(T) template class PCL_EXPORTS pcl::octree::OctreePointCloud<T, pcl::octree::OctreeContainerDataTVector<int> , pcl::octree::OctreeContainerEmpty<int>, pcl::octree::Octree2BufBase<int, pcl::octree::OctreeContainerDataTVector<int>, pcl::octree::OctreeContainerEmpty<int> > >;

#define PCL_INSTANTIATE_OctreePointCloudSingleBufferWithLeafDataT(T) template class PCL_EXPORTS pcl::octree::OctreePointCloud<T, pcl::octree::OctreeLeafDataT<int>, pcl::octree::OctreeContainerEmpty<int> , pcl::octree::OctreeBase<int, pcl::octree::OctreeContainerDataT<int>, pcl::octree::OctreeContainerEmpty<int> > >;
#define PCL_INSTANTIATE_OctreePointCloudDoubleBufferWithLeafDataT(T) template class PCL_EXPORTS pcl::octree::OctreePointCloud<T, pcl::octree::OctreeLeafDataT<int>, pcl::octree::OctreeContainerEmpty<int> , pcl::octree::Octree2BufBase<int, pcl::octree::OctreeContainerDataT<int>, pcl::octree::OctreeContainerEmpty<int> > >;

#define PCL_INSTANTIATE_OctreePointCloudSingleBufferWithEmptyLeaf(T) template class PCL_EXPORTS pcl::octree::OctreePointCloud<T, pcl::octree::OctreeLeafEmpty<int>, pcl::octree::OctreeContainerEmpty<int> , pcl::octree::OctreeBase<int, pcl::octree::OctreeContainerDataT<int>, pcl::octree::OctreeContainerEmpty<int> > >;
#define PCL_INSTANTIATE_OctreePointCloudDoubleBufferWithEmptyLeaf(T) template class PCL_EXPORTS pcl::octree::OctreePointCloud<T, pcl::octree::OctreeLeafEmpty<int>, pcl::octree::OctreeContainerEmpty<int> , pcl::octree::Octree2BufBase<int, pcl::octree::OctreeContainerDataT<int>, pcl::octree::OctreeContainerEmpty<int> > >;

#endif /* OCTREE_POINTCLOUD_HPP_ */
