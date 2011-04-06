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

#ifndef OCTREE_OCCUPANCY_H
#define OCTREE_OCCUPANCY_H

#include "octree_pointcloud.h"

#include "octree_base.h"
#include "octree2buf_base.h"

namespace pcl
{
  namespace octree
  {

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief @b Octree pointcloud occupancy class
     *  \note This pointcloud octree class generate an octrees from a point cloud (zero-copy). No information is stored at the lead nodes. It can be used of occupancy checks.
     *  \note The octree pointcloud is initialized with its voxel resolution. Its bounding box is automatically adjusted or can be predefined.
     *  \note
     *  \note typename: PointT: type of point used in pointcloud
     *  \author Julius Kammerl (julius@kammerl.de)
     */
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template<typename PointT, typename LeafT = OctreeLeafEmpty<int> , typename OctreeT = OctreeBase<int, LeafT> >

      class OctreePointCloudOccupancy : public OctreePointCloud<PointT, LeafT, OctreeT>

      {

      public:
        // public typedefs for single/double buffering
        typedef OctreePointCloudOccupancy<PointT, LeafT, OctreeBase<int, LeafT> > SingleBuffer;
        typedef OctreePointCloudOccupancy<PointT, LeafT, Octree2BufBase<int, LeafT> > DoubleBuffer;

        /** \brief Constructor.
         *  \param resolution_arg:  octree resolution at lowest octree level
         * */
        OctreePointCloudOccupancy (const double resolution) :
          OctreePointCloud<PointT, LeafT, OctreeT> (resolution)
        {
        }

        /** \brief Empty class constructor. */
        virtual
        ~OctreePointCloudOccupancy ()
        {
        }

      };
  }

}

#define PCL_INSTANTIATE_OctreePointCloudOccupancy(T) template class pcl::octree::OctreePointCloudOccupancy<T>;

#endif

