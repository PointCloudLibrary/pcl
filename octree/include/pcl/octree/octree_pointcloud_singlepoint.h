/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *  Copyright (c) 2014-, Open Perception, Inc.
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

#ifndef PCL_OCTREE_SINGLE_POINT_H
#define PCL_OCTREE_SINGLE_POINT_H

#include "octree_pointcloud.h"
#include "octree_container.h"

namespace pcl
{

  namespace octree
  {

    /** An octree leaf container that stores the index of the very last point
      * that was added to it. */
    template <typename UserDataT = boost::blank>
    class OctreeIndexContainer : public OctreeLeafContainer<UserDataT>
    {

      public:

        OctreeIndexContainer ()
        : last_index_ (-1)
        {
        }

        /** Add a new point (index). */
        inline void
        insertPointIndex (int index_arg)
        {
          last_index_ = index_arg;
        }

        /** Retrieve the index of the last point. */
        inline int
        getPointIndex () const
        {
          return (last_index_);
        }

        virtual void
        reset ()
        {
          last_index_ = -1;
        }

      protected:

        int last_index_;

    };

    /** Typedef to preserve existing interface. */
    typedef OctreeIndexContainer<boost::blank> OctreeContainerPointIndex;

    /** LeafContainerTraits specialization for OctreeIndexContainer. */
    template <typename LeafContainerT>
    struct LeafContainerTraits<LeafContainerT,
                               typename boost::enable_if<
                                 boost::is_base_of<
                                   OctreeIndexContainer<typename LeafContainerT::data_type>
                                 , LeafContainerT
                                 >
                               >::type>
    {
      template <typename PointT>
      static void insert (LeafContainerT& container, int idx, const PointT&)
      {
        container.insertPointIndex (idx);
      }
    };

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief @b Octree pointcloud single point class.
      *
      * This pointcloud octree class generates an octree from a point cloud (zero-copy). Every leaf node contains a single
      * point index from the dataset given by \a setInputCloud.
      *
      * The octree pointcloud is initialized with its voxel resolution. Its bounding box is automatically adjusted or can
      * be predefined.
      *
      * typename: PointT: type of point used in pointcloud
      * \ingroup octree
      * \author Julius Kammerl (julius@kammerl.de)
      */
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template<typename PointT,
             typename LeafContainerT = OctreeIndexContainer<>,
             typename BranchContainerT = OctreeEmptyContainer,
             typename OctreeT = OctreeBase<LeafContainerT, BranchContainerT> >

    class OctreePointCloudSinglePoint : public OctreePointCloud<PointT, LeafContainerT, BranchContainerT, OctreeT>
    {

      public:

        // public typedefs for single/double buffering
        typedef OctreePointCloudSinglePoint<PointT, LeafContainerT, BranchContainerT,
                                            OctreeBase<LeafContainerT, BranchContainerT> > SingleBuffer;

        /** Constructor.
          * \param resolution_arg octree resolution at lowest octree level
          */
        OctreePointCloudSinglePoint (const double resolution_arg)
        : OctreePointCloud<PointT, LeafContainerT, BranchContainerT, OctreeT> (resolution_arg)
        {
        }

        /** Empty class deconstructor. */
        virtual ~OctreePointCloudSinglePoint ()
        {
        }

    };

  }

}

#define PCL_INSTANTIATE_OctreePointCloudSinglePoint(T) template class PCL_EXPORTS pcl::octree::OctreePointCloudSinglePoint<T>;

#endif
