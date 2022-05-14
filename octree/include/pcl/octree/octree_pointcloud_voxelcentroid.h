/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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
 * $Id$
 */

#pragma once

#include <pcl/octree/octree_pointcloud.h>

namespace pcl {
namespace octree {
/** \brief @b Octree pointcloud voxel centroid leaf node class
 * \note This class implements a leaf node that calculates the mean centroid of all
 * points added this octree container.
 * \author Julius Kammerl (julius@kammerl.de)
 */
template <typename PointT>
class OctreePointCloudVoxelCentroidContainer : public OctreeContainerBase {
public:
  /** \brief Class initialization. */
  OctreePointCloudVoxelCentroidContainer() { this->reset(); }

  /** \brief Empty class deconstructor. */
  ~OctreePointCloudVoxelCentroidContainer() {}

  /** \brief deep copy function */
  virtual OctreePointCloudVoxelCentroidContainer*
  deepCopy() const
  {
    return (new OctreePointCloudVoxelCentroidContainer(*this));
  }

  /** \brief Equal comparison operator - set to false
   */
  // param[in] OctreePointCloudVoxelCentroidContainer to compare with
  bool
  operator==(const OctreeContainerBase&) const override
  {
    return (false);
  }

  /** \brief Add new point to voxel.
   * \param[in] new_point the new point to add
   */
  void
  addPoint(const PointT& new_point)
  {
    using namespace pcl::common;

    ++point_counter_;

    point_sum_ += new_point;
  }

  /** \brief Calculate centroid of voxel.
   * \param[out] centroid_arg the resultant centroid of the voxel
   */
  void
  getCentroid(PointT& centroid_arg) const
  {
    using namespace pcl::common;

    if (point_counter_) {
      centroid_arg = point_sum_;
      centroid_arg /= static_cast<float>(point_counter_);
    }
    else {
      centroid_arg *= 0.0f;
    }
  }

  /** \brief Reset leaf container. */
  void
  reset() override
  {
    using namespace pcl::common;

    point_counter_ = 0;
    point_sum_ *= 0.0f;
  }

private:
  uindex_t point_counter_;
  PointT point_sum_;
};

/** \brief @b Octree pointcloud voxel centroid class
 * \note This class generate an octrees from a point cloud (zero-copy). It provides a
 * vector of centroids for all occupied voxels.
 * \note The octree pointcloud is initialized with its voxel resolution. Its bounding
 * box is automatically adjusted or can be predefined.
 * \tparam PointT type of point used in pointcloud
 * \ingroup octree
 * \author Julius Kammerl (julius@kammerl.de)
 */
template <typename PointT,
          typename LeafContainerT = OctreePointCloudVoxelCentroidContainer<PointT>,
          typename BranchContainerT = OctreeContainerEmpty>
class OctreePointCloudVoxelCentroid
: public OctreePointCloud<PointT, LeafContainerT, BranchContainerT> {
public:
  using Ptr = shared_ptr<OctreePointCloudVoxelCentroid<PointT, LeafContainerT>>;
  using ConstPtr =
      shared_ptr<const OctreePointCloudVoxelCentroid<PointT, LeafContainerT>>;

  using OctreeT = OctreePointCloud<PointT, LeafContainerT, BranchContainerT>;
  using LeafNode = typename OctreeT::LeafNode;
  using BranchNode = typename OctreeT::BranchNode;

  /** \brief OctreePointCloudVoxelCentroids class constructor.
   * \param[in] resolution_arg octree resolution at lowest octree level
   */
  OctreePointCloudVoxelCentroid(const double resolution_arg)
  : OctreePointCloud<PointT, LeafContainerT, BranchContainerT>(resolution_arg)
  {}

  /** \brief Empty class deconstructor. */

  ~OctreePointCloudVoxelCentroid() {}

  /** \brief Add DataT object to leaf node at octree key.
   * \param pointIdx_arg
   */
  void
  addPointIdx(const uindex_t pointIdx_arg) override
  {
    OctreeKey key;

    assert(pointIdx_arg < this->input_->size());

    const PointT& point = (*this->input_)[pointIdx_arg];

    // make sure bounding box is big enough
    this->adoptBoundingBoxToPoint(point);

    // generate key
    this->genOctreeKeyforPoint(point, key);

    // add point to octree at key
    LeafContainerT* container = this->createLeaf(key);
    container->addPoint(point);
  }

  /** \brief Get centroid for a single voxel addressed by a PointT point.
   * \param[in] point_arg point addressing a voxel in octree
   * \param[out] voxel_centroid_arg centroid is written to this PointT reference
   * \return "true" if voxel is found; "false" otherwise
   */
  bool
  getVoxelCentroidAtPoint(const PointT& point_arg, PointT& voxel_centroid_arg) const;

  /** \brief Get centroid for a single voxel addressed by a PointT point from input
   * cloud.
   * \param[in] point_idx_arg point index from input cloud addressing a voxel in octree
   * \param[out] voxel_centroid_arg centroid is written to this PointT reference
   * \return "true" if voxel is found; "false" otherwise
   */
  inline bool
  getVoxelCentroidAtPoint(const index_t& point_idx_arg,
                          PointT& voxel_centroid_arg) const
  {
    // get centroid at point
    return (this->getVoxelCentroidAtPoint((*this->input_)[point_idx_arg],
                                          voxel_centroid_arg));
  }

  /** \brief Get PointT vector of centroids for all occupied voxels.
   * \param[out] voxel_centroid_list_arg results are written to this vector of PointT
   * elements
   * \return number of occupied voxels
   */
  uindex_t
  getVoxelCentroids(
      typename OctreePointCloud<PointT, LeafContainerT, BranchContainerT>::
          AlignedPointTVector& voxel_centroid_list_arg) const;

  /** \brief Recursively explore the octree and output a PointT vector of centroids for
   * all occupied voxels.
   * \param[in] branch_arg: current branch node
   * \param[in] key_arg: current key
   * \param[out] voxel_centroid_list_arg results are written to this vector of PointT
   * elements
   */
  void
  getVoxelCentroidsRecursive(
      const BranchNode* branch_arg,
      OctreeKey& key_arg,
      typename OctreePointCloud<PointT, LeafContainerT, BranchContainerT>::
          AlignedPointTVector& voxel_centroid_list_arg) const;
};
} // namespace octree
} // namespace pcl

// Note: Don't precompile this octree type to speed up compilation. It's probably rarely
// used.
#include <pcl/octree/impl/octree_pointcloud_voxelcentroid.hpp>
