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

#ifndef PCL_OCTREE_VOXELCENTROID_H
#define PCL_OCTREE_VOXELCENTROID_H

#include "octree_pointcloud.h"
#include "octree_base.h"
#include "octree2buf_base.h"

namespace pcl
{
  namespace octree
  {
    /** \brief @b Octree pointcloud voxel centroid class
      * \note This class generate an octrees from a point cloud (zero-copy). It provides a vector of centroids for all occupied voxels.
      * \note The octree pointcloud is initialized with its voxel resolution. Its bounding box is automatically adjusted or can be predefined.
      * \note
      * \note typename: PointT: type of point used in pointcloud
      *
      * \ingroup octree
      * \author Julius Kammerl (julius@kammerl.de)
      */
    template<typename PointT, typename LeafT = OctreeContainerDataTVector<int> , typename BranchT = OctreeContainerEmpty<int> >
    class OctreePointCloudVoxelCentroid : public OctreePointCloud<PointT, LeafT, BranchT>
    {
      public:
        /** \brief OctreePointCloudVoxelCentroids class constructor.
          * \param[in] resolution_arg octree resolution at lowest octree level
          */
        OctreePointCloudVoxelCentroid (const double resolution_arg) :
          OctreePointCloud<PointT, LeafT, BranchT> (resolution_arg)
        {
        }

        /** \brief Empty class deconstructor. */
        virtual
        ~OctreePointCloudVoxelCentroid ()
        {
        }

        /** \brief Get PointT vector of centroids for all occupied voxels.
          * \param[out] voxel_centroid_list_arg results are written to this vector of PointT elements
          * \return number of occupied voxels
          */
        unsigned int
        getVoxelCentroids (typename pcl::octree::OctreePointCloud<PointT, LeafT, BranchT>::AlignedPointTVector &voxel_centroid_list_arg);

        /** \brief Get centroid for a single voxel addressed by a PointT point.
          * \param[in] point_arg point addressing a voxel in octree
          * \param[out] voxel_centroid_arg centroid is written to this PointT reference
          * \return "true" if voxel is found; "false" otherwise
          */
        bool
        getVoxelCentroidAtPoint (const PointT& point_arg, PointT& voxel_centroid_arg);

        /** \brief Get centroid for a single voxel addressed by a PointT point from input cloud.
          * \param[in] point_idx_arg point index from input cloud addressing a voxel in octree
          * \param[out] voxel_centroid_arg centroid is written to this PointT reference
          * \return "true" if voxel is found; "false" otherwise
          */
        inline bool
        getVoxelCentroidAtPoint (const int& point_idx_arg, PointT& voxel_centroid_arg)
        {
          // get centroid at point
          return (this->getVoxelCentroidAtPoint (this->input_->points[point_idx_arg], voxel_centroid_arg));
        }
    };
  }
}

#include <pcl/octree/impl/octree_pointcloud_voxelcentroid.hpp>

#endif

