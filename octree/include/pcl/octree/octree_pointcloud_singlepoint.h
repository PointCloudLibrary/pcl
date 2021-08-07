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

#include <pcl/octree/octree_pointcloud.h>

namespace pcl {
namespace octree {
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief @b Octree pointcloud single point class
 *  \note This pointcloud octree class generate an octrees from a point cloud
 * (zero-copy). Every leaf node contains a single point index from the dataset given by
 * \a setInputCloud.
 * \note The octree pointcloud is initialized with its voxel resolution. Its bounding
 * box is automatically adjusted or can be predefined.
 *  \tparam PointT type of point used in pointcloud
 *  \ingroup octree
 *  \author Julius Kammerl (julius@kammerl.de)
 */
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT,
          typename LeafContainerT = OctreeContainerPointIndex,
          typename BranchContainerT = OctreeContainerEmpty,
          typename OctreeT = OctreeBase<LeafContainerT, BranchContainerT>>

class OctreePointCloudSinglePoint
: public OctreePointCloud<PointT, LeafContainerT, BranchContainerT, OctreeT> {

public:
  // public typedefs for single/double buffering
  using SingleBuffer =
      OctreePointCloudSinglePoint<PointT,
                                  LeafContainerT,
                                  BranchContainerT,
                                  OctreeBase<LeafContainerT, BranchContainerT>>;
  //      typedef OctreePointCloudSinglePoint<PointT, LeafContainerT, BranchContainerT,
  //         Octree2BufBase<int, LeafContainerT, BranchContainerT> > DoubleBuffer;

  /** \brief Constructor.
   *  \param resolution_arg: octree resolution at lowest octree level
   * */
  OctreePointCloudSinglePoint(const double resolution_arg)
  : OctreePointCloud<PointT, LeafContainerT, BranchContainerT, OctreeT>(resolution_arg)
  {}

  /** \brief Empty class constructor. */
  ~OctreePointCloudSinglePoint() {}
};

} // namespace octree
} // namespace pcl

// needed since OctreePointCloud is not instantiated with template parameters used above
#include <pcl/octree/impl/octree_pointcloud.hpp>

#define PCL_INSTANTIATE_OctreePointCloudSinglePoint(T)                                 \
  template class PCL_EXPORTS pcl::octree::OctreePointCloudSinglePoint<T>;
