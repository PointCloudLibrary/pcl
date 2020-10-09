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

#pragma once

#include <pcl/octree/octree2buf_base.h>
#include <pcl/octree/octree_pointcloud.h>
#include <pcl/memory.h>

namespace pcl {
namespace octree {

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief @b Octree pointcloud change detector class
 *  \note This pointcloud octree class generate an octrees from a point cloud
 * (zero-copy). It allows to detect new leaf nodes and serialize their point indices
 *  \note The octree pointcloud is initialized with its voxel resolution. Its bounding
 * box is automatically adjusted or can be predefined.
 * \tparam PointT type of point used in pointcloud
 * \ingroup octree
 * \author Julius Kammerl (julius@kammerl.de)
 */
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT,
          typename LeafContainerT = OctreeContainerPointIndices,
          typename BranchContainerT = OctreeContainerEmpty>

class OctreePointCloudChangeDetector
: public OctreePointCloud<PointT,
                          LeafContainerT,
                          BranchContainerT,
                          Octree2BufBase<LeafContainerT, BranchContainerT>>

{

public:
  using Ptr = shared_ptr<
      OctreePointCloudChangeDetector<PointT, LeafContainerT, BranchContainerT>>;
  using ConstPtr = shared_ptr<
      const OctreePointCloudChangeDetector<PointT, LeafContainerT, BranchContainerT>>;

  /** \brief Constructor.
   *  \param resolution_arg:  octree resolution at lowest octree level
   * */
  OctreePointCloudChangeDetector(const double resolution_arg)
  : OctreePointCloud<PointT,
                     LeafContainerT,
                     BranchContainerT,
                     Octree2BufBase<LeafContainerT, BranchContainerT>>(resolution_arg)
  {}

  /** \brief Get a indices from all leaf nodes that did not exist in previous buffer.
   * \param indicesVector_arg: results are written to this vector of int indices
   * \param minPointsPerLeaf_arg: minimum amount of points required within leaf node to
   * become serialized.
   * \return number of point indices
   */
  std::size_t
  getPointIndicesFromNewVoxels(Indices& indicesVector_arg,
                               const uindex_t minPointsPerLeaf_arg = 0)
  {

    std::vector<OctreeContainerPointIndices*> leaf_containers;
    this->serializeNewLeafs(leaf_containers);

    for (const auto& leaf_container : leaf_containers) {
      if (static_cast<uindex_t>(leaf_container->getSize()) >= minPointsPerLeaf_arg)
        leaf_container->getPointIndices(indicesVector_arg);
    }

    return (indicesVector_arg.size());
  }
};
} // namespace octree
} // namespace pcl

#define PCL_INSTANTIATE_OctreePointCloudChangeDetector(T)                              \
  template class PCL_EXPORTS pcl::octree::OctreePointCloudChangeDetector<T>;
