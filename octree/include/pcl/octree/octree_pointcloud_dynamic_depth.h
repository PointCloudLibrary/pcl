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

#ifndef PCL_OCTREE_POINTCLOUD_DYNAMIC_DEPTH_H
#define PCL_OCTREE_POINTCLOUD_DYNAMIC_DEPTH_H

#include <pcl/octree/octree_pointcloud_pointvector.h>
#include <pcl/octree/octree_container.h>

namespace pcl
{
  namespace octree
  {
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief @b Octree pointcloud dynamic depth class
     *  \note This pointcloud octree class generate an octrees from a point cloud (zero-copy). Every leaf node contains a list of point indices of the dataset given by \a setInputCloud.
     *  \note The depth of the octree is determined by max_objects_per_leaf, and is set dynamically depending on the input pointcloud data
     *  \note typename: PointT: type of point used in pointcloud
     *  \ingroup octree
     *  \author Julius Kammerl (julius@kammerl.de)
     */
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template<typename PointT,
             typename BranchContainerT = OctreeEmptyContainer,
             typename OctreeT = OctreeBase<OctreeIndicesContainer<>, BranchContainerT> >
             class OctreePointCloudDynamicDepth : public OctreePointCloud<PointT, OctreeIndicesContainer<>, BranchContainerT, OctreeT>
    {
      public:
        typedef typename OctreeT::LeafNode LeafNode;
        typedef typename OctreeT::BranchNode BranchNode;
        typedef OctreeIndicesContainer<> LeafContainerT;
        /** \brief Constructor.
         *  \param resolution_arg: octree resolution at lowest octree level
         *  \param max_objects_per_leaf: maximum numbers of points allowed in a leaf before depth is increased
         * */
        OctreePointCloudDynamicDepth (const double resolution_arg, std::size_t max_objects_per_leaf) :
            max_objs_per_leaf_ (max_objects_per_leaf),
            OctreePointCloud<PointT, LeafContainerT, BranchContainerT, OctreeT> (resolution_arg)
        {
        }
        
        /** \brief Empty class destructor. */
        virtual ~OctreePointCloudDynamicDepth ()
        {
        }
        
        /** \brief Sets the maximum number of points allowed in a leaf
         *  \note Leaf nodes are kept as close to the root as possible and are only expanded if the number of points within a leaf node exceeds a fixed limit.
         *  \param maxObjsPerLeaf: maximum number of points per leaf
         * */
        void
        setMaxObjectsPerLeaf ( std::size_t max_objects_per_leaf )
        {
          assert(this->leaf_count_== 0);
          max_objs_per_leaf_ = max_objects_per_leaf;
        }
        
        /** \brief Gets the maxinum number of objects allowed in a leaf
         *  \note Leaf nodes are kept as close to the root as possible and are only expanded if the number of points within a leaf node exceeds a fixed limit.
         *  \return maximum number of points per leaf
         * */
        std::size_t
        getMaxObjectsPerLeaf ( ) const
        {
          return max_objs_per_leaf_;
        }
        
      protected:
        /** \brief Add point at index from input pointcloud dataset to octree
         * \param[in] point_idx_arg the index representing the point in the dataset given by \a setInputCloud to be added
         */
        virtual void
        addPointIdx (const int point_idx_arg);
        
        /** \brief Add point at index from input pointcloud dataset to octree
         * \param[in] leaf_node to be expanded
         * \param[in] parent_branch parent of leaf node to be expanded
         * \param[in] child_idx child index of leaf node (in parent branch)
         * \param[in] depth_mask of leaf node to be expanded
         */
        void
        expandLeafNode (LeafNode* leaf_node, BranchNode* parent_branch, unsigned char child_idx, unsigned int depth_mask);
        
        
        /** \brief Amount of DataT objects per leafNode before expanding branch
        *  \note zero indicates a fixed/maximum depth octree structure
        * **/
        std::size_t max_objs_per_leaf_;

    };
  }
}

#include <pcl/octree/impl/octree_pointcloud_dynamic_depth.hpp>

#endif
