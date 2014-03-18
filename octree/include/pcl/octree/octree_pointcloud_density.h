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

#ifndef PCL_OCTREE_DENSITY_H
#define PCL_OCTREE_DENSITY_H

#include "octree_pointcloud.h"
#include "octree_container.h"

namespace pcl
{
  namespace octree
  {

    /** \brief An octree leaf container that stores the number of points
      * that were added to it. */
    template <typename UserDataT = boost::blank>
    class OctreeDensityContainer : public OctreeLeafContainer<UserDataT>
    {

        typedef OctreeLeafContainer<UserDataT> OctreeLeafContainerT;

      public:

        OctreeDensityContainer ()
        : OctreeLeafContainerT ()
        , num_points_ (0)
        {
        }

        /** Add a new point. */
        void
        insertPoint ()
        {
          ++num_points_;
        }

        /** \brief Get the number of points that have been inserted. */
        size_t
        getSize () const
        {
          return (num_points_);
        }

        virtual void
        reset ()
        {
          num_points_ = 0;
        }

      protected:

        size_t num_points_;

    };

    /** Typedef to preserve existing interface. */
    typedef OctreeDensityContainer<boost::blank> OctreePointCloudDensityContainer;

    template <typename LeafContainerT>
    struct LeafContainerTraits<LeafContainerT,
                               typename boost::enable_if<
                                 boost::is_base_of<
                                   OctreeDensityContainer<typename LeafContainerT::data_type>
                                 , LeafContainerT
                                 >
                               >::type>
    {
      template <typename PointT>
      static void insert (LeafContainerT& container, int, const PointT&)
      {
        container.insertPoint ();
      }
    };

    /** \brief @b Octree pointcloud density class
      * \note This class generate an octrees from a point cloud (zero-copy). Only the amount of points that fall into the leaf node voxel are stored.
      * \note The octree pointcloud is initialized with its voxel resolution. Its bounding box is automatically adjusted or can be predefined.
      * \note
      * \note typename: PointT: type of point used in pointcloud
      * \ingroup octree
      * \author Julius Kammerl (julius@kammerl.de)
      */
    template<typename PointT,
             typename LeafContainerT = OctreeDensityContainer<>,
             typename BranchContainerT = OctreeEmptyContainer>
    class OctreePointCloudDensity : public OctreePointCloud<PointT, LeafContainerT, BranchContainerT>
    {

      public:

        typedef OctreePointCloud<PointT, LeafContainerT, BranchContainerT> OctreeT;

        /** \brief OctreePointCloudDensity class constructor.
         *  \param resolution_arg:  octree resolution at lowest octree level
         * */
        OctreePointCloudDensity (const double resolution_arg)
        : OctreeT (resolution_arg)
        {
        }

        /** \brief Empty class deconstructor. */
        virtual
        ~OctreePointCloudDensity ()
        {
        }

        /** \brief Get the amount of points within a leaf node voxel which is addressed by a point
          * \param[in] point_arg: a point addressing a voxel
          * \return amount of points that fall within leaf node voxel
          */
        unsigned int
        getVoxelDensityAtPoint (const PointT& point_arg) const
        {
          LeafContainerT* leaf = this->findLeafAtPoint (point_arg);
          return (leaf ? leaf->getSize () : 0);
        }

    };
  }
}

#define PCL_INSTANTIATE_OctreePointCloudDensity(T) template class PCL_EXPORTS pcl::octree::OctreePointCloudDensity<T>;

#endif

