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
#include "octree_container.h"

namespace pcl
{
  namespace octree
  {
    /** \brief @b Octree pointcloud voxel centroid leaf node class
     * \note This class implements a leaf node that calculates the mean centroid of all points added this octree container.
     * \author Julius Kammerl (julius@kammerl.de)
     */
    template<typename DataT>
    class OctreePointCloudVoxelCentroidContainer : public OctreeContainerBase<DataT>
    {
      public:
        /** \brief Class initialization. */
        OctreePointCloudVoxelCentroidContainer ()
        {
          this->reset();
        }

        /** \brief Empty class deconstructor. */
        virtual ~OctreePointCloudVoxelCentroidContainer ()
        {
        }

        /** \brief deep copy function */
        virtual OctreePointCloudVoxelCentroidContainer *
        deepCopy () const
        {
          return (new OctreePointCloudVoxelCentroidContainer (*this));
        }

        /** \brief Add new point to voxel.
         */
        void addPoint (const PointXYZ& newPoint_arg)
        {
          ++pointCounter_;

          pointSum.x += newPoint_arg.x;
          pointSum.y += newPoint_arg.y;
          pointSum.z += newPoint_arg.z;
        }

        /** \brief Calculate centroid of voxel.
         */
        void getCentroid (PointXYZ& centroid_arg) const
        {
          if (pointCounter_)
          {
            centroid_arg.x = pointSum.x/static_cast<float>(pointCounter_);
            centroid_arg.y = pointSum.y/static_cast<float>(pointCounter_);
            centroid_arg.z = pointSum.z/static_cast<float>(pointCounter_);
          } else
          {
            centroid_arg.x = centroid_arg.y = centroid_arg.z = 0.0f;
          }
        }

        /** \brief Reset leaf container. */
        virtual void reset ()
        {
          pointCounter_ = 0;
          pointSum.x = pointSum.y = pointSum.z = 0.0f;
        }

      private:
        unsigned int pointCounter_;
        PointXYZ pointSum;
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
    template<typename PointT, typename LeafT = OctreePointCloudVoxelCentroidContainer<int> , typename BranchT = OctreeContainerEmpty<int> >
    class OctreePointCloudVoxelCentroid : public OctreePointCloud<PointT, LeafT, BranchT>
    {
      public:
        typedef OctreePointCloud<PointT, LeafT, BranchT> OctreeT;
        typedef typename OctreeT::LeafNode LeafNode;
        typedef typename OctreeT::BranchNode BranchNode;

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

        /** \brief Add DataT object to leaf node at octree key.
         *  \param key_arg: octree key addressing a leaf node.
         *  \param data_arg: DataT object to be added.
         * */
        virtual void addData (const OctreeKey& key_arg, const int& data_arg)
        {
          LeafNode* newLeaf = 0;
          createLeafRecursive (key_arg, this->depthMask_, data_arg, this->rootNode_, newLeaf);

          if (newLeaf)
          {
            PointXYZ point;
            const PointT& cloudPoint = this->getPointByIndex(data_arg);
            point.x = cloudPoint.x;
            point.y = cloudPoint.y;
            point.z = cloudPoint.z;

            // add data to leaf
            LeafT* container = newLeaf;
            container->addPoint (point);
            this->objectCount_++;
          }
        }

        /** \brief Get centroid for a single voxel addressed by a PointT point.
          * \param[in] point_arg point addressing a voxel in octree
          * \param[out] voxel_centroid_arg centroid is written to this PointT reference
          * \return "true" if voxel is found; "false" otherwise
          */
        bool
        getVoxelCentroidAtPoint (const PointT& point_arg, PointXYZ& voxel_centroid_arg) const;

        /** \brief Get centroid for a single voxel addressed by a PointT point from input cloud.
          * \param[in] point_idx_arg point index from input cloud addressing a voxel in octree
          * \param[out] voxel_centroid_arg centroid is written to this PointT reference
          * \return "true" if voxel is found; "false" otherwise
          */
        inline bool
        getVoxelCentroidAtPoint (const int& point_idx_arg, PointXYZ& voxel_centroid_arg) const
        {
          // get centroid at point
          return (this->getVoxelCentroidAtPoint (this->input_->points[point_idx_arg], voxel_centroid_arg));
        }

        /** \brief Get PointT vector of centroids for all occupied voxels.
          * \param[out] voxel_centroid_list_arg results are written to this vector of PointT elements
          * \return number of occupied voxels
          */
        size_t
        getVoxelCentroids (typename OctreePointCloud<PointT, LeafT, BranchT>::AlignedPointXYZVector &voxel_centroid_list_arg) const;

        /** \brief Recursively explore the octree and output a PointT vector of centroids for all occupied voxels.
         ** \param[in] binaryTreeOut_arg: binary output vector
          * \param[in] branch_arg: current branch node
          * \param[out] voxel_centroid_list_arg results are written to this vector of PointT elements
          * \return number of occupied voxels
          */
        void
        getVoxelCentroidsRecursive (const BranchNode* branch_arg, OctreeKey& key_arg, typename OctreePointCloud<PointT, LeafT, BranchT>::AlignedPointXYZVector &voxel_centroid_list_arg) const;

    };
  }
}

#include <pcl/octree/impl/octree_pointcloud_voxelcentroid.hpp>

#endif

