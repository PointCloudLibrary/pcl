/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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
 * Author: Julius Kammerl (julius@kammerl.de)
 */

#include <pcl/octree/octree.h>
#include <pcl/octree/octree_impl.h>

// Instantiations of specific point types

template class PCL_EXPORTS pcl::octree::OctreeBase<int>;
template class PCL_EXPORTS pcl::octree::Octree2BufBase<int>;

template class PCL_EXPORTS
    pcl::octree::OctreeBase<pcl::octree::OctreeContainerPointIndices,
                            pcl::octree::OctreeContainerEmpty>;

template class PCL_EXPORTS
    pcl::octree::Octree2BufBase<pcl::octree::OctreeContainerPointIndices,
                                pcl::octree::OctreeContainerEmpty>;

template class PCL_EXPORTS pcl::octree::OctreeBase<pcl::octree::OctreeContainerEmpty,
                                                   pcl::octree::OctreeContainerEmpty>;

#ifndef PCL_NO_PRECOMPILE
#include <pcl/impl/instantiate.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

PCL_INSTANTIATE(OctreePointCloudSingleBufferWithLeafDataTVector, PCL_XYZ_POINT_TYPES)
PCL_INSTANTIATE(OctreePointCloudDoubleBufferWithLeafDataTVector, PCL_XYZ_POINT_TYPES)

PCL_INSTANTIATE(OctreePointCloudSearch, PCL_XYZ_POINT_TYPES)

// PCL_INSTANTIATE(OctreePointCloudSingleBufferWithLeafDataT, PCL_XYZ_POINT_TYPES)
PCL_INSTANTIATE(OctreePointCloudSingleBufferWithEmptyLeaf, PCL_XYZ_POINT_TYPES)

/*
 * Note: Disable apriori instantiation of these octree types to speed up compilation.
 * They are probably rarely used.
 */
// PCL_INSTANTIATE(OctreePointCloudDensity, PCL_XYZ_POINT_TYPES)
// PCL_INSTANTIATE(OctreePointCloudSingleBufferWithDensityLeaf, PCL_XYZ_POINT_TYPES)
// PCL_INSTANTIATE(OctreePointCloudDoubleBufferWithDensityLeaf, PCL_XYZ_POINT_TYPES)

// PCL_INSTANTIATE(OctreePointCloudOccupancy, PCL_XYZ_POINT_TYPES)
// PCL_INSTANTIATE(OctreePointCloudSinglePoint, PCL_XYZ_POINT_TYPES)
// PCL_INSTANTIATE(OctreePointCloudPointVector, PCL_XYZ_POINT_TYPES)
// PCL_INSTANTIATE(OctreePointCloudChangeDetector, PCL_XYZ_POINT_TYPES)
// PCL_INSTANTIATE(OctreePointCloudVoxelCentroid, PCL_XYZ_POINT_TYPES)
// PCL_INSTANTIATE(OctreePointCloudAdjacency, PCL_XYZ_POINT_TYPES)

#endif // PCL_NO_PRECOMPILE
