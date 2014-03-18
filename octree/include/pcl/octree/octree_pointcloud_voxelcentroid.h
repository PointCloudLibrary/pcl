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

#include <pcl/octree/octree_pointcloud.h>
#include <pcl/octree/octree_container.h>
#include <pcl/octree/centroid_point.h>
#include <pcl/common/point_operators.h>
#include <pcl/point_types.h>
#include <pcl/register_point_struct.h>

namespace pcl
{
  namespace octree
  {

    /** \brief An octree leaf container that computes the cenrtoid of the points
      * that were inserted into it. */
    template <typename PointT = pcl::PointXYZ,
              typename UserDataT = boost::blank>
    class OctreeCentroidContainer : public OctreeLeafContainer<UserDataT>
    {

        typedef OctreeLeafContainer<UserDataT> OctreeLeafContainerT;

      public:

        typedef PointT point_type;
        typedef UserDataT data_type;

        OctreeCentroidContainer ()
        : OctreeLeafContainerT ()
        {
        }

        /** Add a new point. */
        void
        insertPoint (const PointT& point_arg)
        {
          centroid_.add (point_arg);
        }

        /** \brief Retrieve the computed average (centroid) point.
          *
          * This container maintains the sum of all fields of the
          * inserted points. When this function is called it will divide the
          * sum by the actual number of inserted points. */
        template <typename T> void
        getCentroid (T& point_arg) const
        {
          centroid_.get (point_arg);
        }

        /** \brief Get the number of points that have been inserted. */
        size_t
        getSize () const
        {
          return (centroid_.getSize ());
        }

        virtual void
        reset ()
        {
          centroid_ = CentroidPoint<PointT> ();
        }

      protected:

        CentroidPoint<PointT> centroid_;

    };

    /** Wrapper class to preserve existing interface. */
    template <typename PointT>
    class OctreePointCloudVoxelCentroidContainer : public OctreeCentroidContainer<PointT, boost::blank>
    {

      public:

        OctreePointCloudVoxelCentroidContainer ()
        : OctreeCentroidContainer<PointT, boost::blank> ()
        {
        }

    };

    template <typename LeafContainerT>
    struct LeafContainerTraits<LeafContainerT,
                               typename boost::enable_if<
                                 boost::is_base_of<
                                   OctreeCentroidContainer<
                                     typename LeafContainerT::point_type
                                   , typename LeafContainerT::data_type
                                   >
                                 , LeafContainerT
                                 >
                               >::type>
    {
      static void insert (LeafContainerT& container, int, const typename LeafContainerT::point_type& point)
      {
        container.insertPoint (point);
      }
    };

    /** \brief @b Octree pointcloud voxel centroid class
    * \note This class generate an octrees from a point cloud (zero-copy). It provides a vector of centroids for all occupied voxels.
    * \note The octree pointcloud is initialized with its voxel resolution. Its bounding box is automatically adjusted or can be predefined.
    * \note
    * \note typename: PointT: type of point used in pointcloud
    *
    * \ingroup octree
    * \author Julius Kammerl (julius@kammerl.de)
    */
    template<typename PointT,
             typename LeafContainerT = OctreeCentroidContainer<PointT>,
             typename BranchContainerT = OctreeEmptyContainer>
    class OctreePointCloudVoxelCentroid : public OctreePointCloud<PointT, LeafContainerT, BranchContainerT>
    {

      public:

        typedef OctreePointCloudVoxelCentroid<PointT, LeafContainerT, BranchContainerT> OctreeCentroidT;
        typedef boost::shared_ptr<OctreeCentroidT> Ptr;
        typedef boost::shared_ptr<const OctreeCentroidT> ConstPtr;

        typedef OctreePointCloud<PointT, LeafContainerT, BranchContainerT> OctreeT;
        typedef typename OctreeT::LeafNode LeafNode;
        typedef typename OctreeT::BranchNode BranchNode;

        /** \brief OctreePointCloudVoxelCentroids class constructor.
          * \param[in] resolution_arg octree resolution at lowest octree level
          */
        OctreePointCloudVoxelCentroid (const double resolution_arg)
        : OctreeT (resolution_arg)
        {
        }

        /** \brief Empty class deconstructor. */
        virtual
        ~OctreePointCloudVoxelCentroid ()
        {
        }

        /** \brief Get centroid for a single voxel addressed by a PointT point.
          * \param[in] point_arg point addressing a voxel in octree
          * \param[out] voxel_centroid_arg centroid is written to this PointT reference
          * \return "true" if voxel is found; "false" otherwise
          */
        bool
        getVoxelCentroidAtPoint (const PointT& point_arg, PointT& voxel_centroid_arg) const;

        /** \brief Get centroid for a single voxel addressed by a PointT point from input cloud.
          * \param[in] point_idx_arg point index from input cloud addressing a voxel in octree
          * \param[out] voxel_centroid_arg centroid is written to this PointT reference
          * \return "true" if voxel is found; "false" otherwise
          */
        inline bool
        getVoxelCentroidAtPoint (const int& point_idx_arg, PointT& voxel_centroid_arg) const
        {
          // get centroid at point
          return (this->getVoxelCentroidAtPoint (this->input_->points[point_idx_arg], voxel_centroid_arg));
        }

        /** \brief Get PointT vector of centroids for all occupied voxels.
          * \param[out] voxel_centroid_list_arg results are written to this vector of PointT elements
          * \return number of occupied voxels
          */
        size_t
        getVoxelCentroids (typename OctreeT::AlignedPointTVector &voxel_centroid_list_arg) const;

        /** \brief Recursively explore the octree and output a PointT vector of centroids for all occupied voxels.
          * \param[in] branch_arg: current branch node
          * \param[in] key_arg: current key
          * \param[out] voxel_centroid_list_arg results are written to this vector of PointT elements
          */
        void
        getVoxelCentroidsRecursive (const BranchNode* branch_arg,
                                    OctreeKey& key_arg,
                                    typename OctreeT::AlignedPointTVector &voxel_centroid_list_arg) const;

    };
  }
}

#include <pcl/octree/impl/octree_pointcloud_voxelcentroid.hpp>

#endif

