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

#ifndef OCTREE_POINTCLOUD_H
#define OCTREE_POINTCLOUD_H

#include "octree_base.h"
#include "octree2buf_base.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "octree_nodes.h"
#include "octree_iterator.h"

#include <queue>
#include <vector>
#include <algorithm>
#include <iostream>

namespace pcl
{
  namespace octree
  {

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief @b Octree pointcloud class
     *  \note Octree implementation for pointclouds. Only indices are stored by the octree leaf nodes (zero-copy).
     *  \note The octree pointcloud class needs to be initialized with its voxel resolution. Its bounding box is automatically adjusted
     *  \note according to the pointcloud dimension or it can be predefined.
     *  \note Note: The tree depth equates to the resolution and the bounding box dimensions of the octree.
     *  \note
     *  \note typename: PointT: type of point used in pointcloud
     *  \note typename: LeafT:  leaf node container (
     *  \note typename: BranchT:  branch node container
     *  \note typename: OctreeT: octree implementation ()
     *  \ingroup octree
     *  \author Julius Kammerl (julius@kammerl.de)
     */
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template<typename PointT, typename LeafT = OctreeContainerDataTVector<int>,
        typename BranchT = OctreeContainerEmpty<int>,
        typename OctreeT = OctreeBase<int, LeafT, BranchT> >

    class OctreePointCloud : public OctreeT
    {
        // iterators are friends
        friend class OctreeIteratorBase<int, OctreeT> ;
        friend class OctreeDepthFirstIterator<int, OctreeT> ;
        friend class OctreeBreadthFirstIterator<int, OctreeT> ;
        friend class OctreeLeafNodeIterator<int, OctreeT> ;

      public:
        typedef OctreeT Base;

        typedef typename OctreeT::LeafNode LeafNode;
        typedef typename OctreeT::BranchNode BranchNode;

        // Octree iterators
        typedef OctreeDepthFirstIterator<int, OctreeT> Iterator;
        typedef const OctreeDepthFirstIterator<int, OctreeT> ConstIterator;

        typedef OctreeLeafNodeIterator<int, OctreeT> LeafNodeIterator;
        typedef const OctreeLeafNodeIterator<int, OctreeT> ConstLeafNodeIterator;

        typedef OctreeDepthFirstIterator<int, OctreeT> DepthFirstIterator;
        typedef const OctreeDepthFirstIterator<int, OctreeT> ConstDepthFirstIterator;
        typedef OctreeBreadthFirstIterator<int, OctreeT> BreadthFirstIterator;
        typedef const OctreeBreadthFirstIterator<int, OctreeT> ConstBreadthFirstIterator;

        /** \brief Octree pointcloud constructor.
         * \param[in] resolution_arg octree resolution at lowest octree level
         */
        OctreePointCloud (const double resolution_arg);

        /** \brief Empty deconstructor. */
        virtual
        ~OctreePointCloud ();

        // public typedefs
        typedef boost::shared_ptr<std::vector<int> > IndicesPtr;
        typedef boost::shared_ptr<const std::vector<int> > IndicesConstPtr;

        typedef pcl::PointCloud<PointT> PointCloud;
        typedef boost::shared_ptr<PointCloud> PointCloudPtr;
        typedef boost::shared_ptr<const PointCloud> PointCloudConstPtr;

        // public typedefs for single/double buffering
        typedef OctreePointCloud<PointT, LeafT, OctreeBase<int, LeafT> > SingleBuffer;
        typedef OctreePointCloud<PointT, LeafT, Octree2BufBase<int, LeafT> > DoubleBuffer;

        // Boost shared pointers
        typedef boost::shared_ptr<OctreePointCloud<PointT, LeafT, OctreeT> > Ptr;
        typedef boost::shared_ptr<const OctreePointCloud<PointT, LeafT, OctreeT> > ConstPtr;

        // Eigen aligned allocator
        typedef std::vector<PointT, Eigen::aligned_allocator<PointT> > AlignedPointTVector;

        /** \brief Provide a pointer to the input data set.
         * \param[in] cloud_arg the const boost shared pointer to a PointCloud message
         * \param[in] indices_arg the point indices subset that is to be used from \a cloud - if 0 the whole point cloud is used
         */
        inline void setInputCloud (const PointCloudConstPtr &cloud_arg,
            const IndicesConstPtr &indices_arg = IndicesConstPtr ())
        {
          assert(this->leafCount_==0);

          input_ = cloud_arg;
          indices_ = indices_arg;
        }

        /** \brief Get a pointer to the vector of indices used.
         * \return pointer to vector of indices used.
         */
        inline IndicesConstPtr const getIndices () const
        {
          return (indices_);
        }

        /** \brief Get a pointer to the input point cloud dataset.
         * \return pointer to pointcloud input class.
         */
        inline PointCloudConstPtr getInputCloud () const
        {
          return (input_);
        }

        /** \brief Set the search epsilon precision (error bound) for nearest neighbors searches.
         * \param[in] eps precision (error bound) for nearest neighbors searches
         */
        inline void setEpsilon (double eps)
        {
          epsilon_ = eps;
        }

        /** \brief Get the search epsilon precision (error bound) for nearest neighbors searches. */
        inline double getEpsilon () const
        {
          return (epsilon_);
        }

        /** \brief Set/change the octree voxel resolution
         * \param[in] resolution_arg side length of voxels at lowest tree level
         */
        inline void setResolution (double resolution_arg)
        {
          // octree needs to be empty to change its resolution
          assert( this->leafCount_ == 0);

          resolution_ = resolution_arg;

          getKeyBitSize ();
        }

        /** \brief Get octree voxel resolution
         * \return voxel resolution at lowest tree level
         */
        inline double getResolution () const
        {
          return (resolution_);
        }

        /** \brief Get the maximum depth of the octree.
         *  \return depth_arg: maximum depth of octree
         * */
        inline unsigned int getTreeDepth () const
        {
          return this->octreeDepth_;
        }

        /** \brief Add points from input point cloud to octree. */
        void
        addPointsFromInputCloud ();

        /** \brief Add point at given index from input point cloud to octree. Index will be also added to indices vector.
         * \param[in] pointIdx_arg index of point to be added
         * \param[in] indices_arg pointer to indices vector of the dataset (given by \a setInputCloud)
         */
        void
        addPointFromCloud (const int pointIdx_arg, IndicesPtr indices_arg);

        /** \brief Add point simultaneously to octree and input point cloud.
         *  \param[in] point_arg point to be added
         *  \param[in] cloud_arg pointer to input point cloud dataset (given by \a setInputCloud)
         */
        void
        addPointToCloud (const PointT& point_arg, PointCloudPtr cloud_arg);

        /** \brief Add point simultaneously to octree and input point cloud. A corresponding index will be added to the indices vector.
         * \param[in] point_arg point to be added
         * \param[in] cloud_arg pointer to input point cloud dataset (given by \a setInputCloud)
         * \param[in] indices_arg pointer to indices vector of the dataset (given by \a setInputCloud)
         */
        void
        addPointToCloud (const PointT& point_arg, PointCloudPtr cloud_arg,
            IndicesPtr indices_arg);

        /** \brief Check if voxel at given point exist.
         * \param[in] point_arg point to be checked
         * \return "true" if voxel exist; "false" otherwise
         */
        bool
        isVoxelOccupiedAtPoint (const PointT& point_arg) const;

        /** \brief Delete the octree structure and its leaf nodes.
         *  \param freeMemory_arg: if "true", allocated octree nodes are deleted, otherwise they are pushed to the octree node pool
         * */
        void deleteTree (bool freeMemory_arg = false)
        {
          // reset bounding box
          minX_ = minY_ = maxY_ = minZ_ = maxZ_ = 0;
          this->boundingBoxDefined_ = false;

          OctreeT::deleteTree (freeMemory_arg);
        }

        /** \brief Check if voxel at given point coordinates exist.
         * \param[in] pointX_arg X coordinate of point to be checked
         * \param[in] pointY_arg Y coordinate of point to be checked
         * \param[in] pointZ_arg Z coordinate of point to be checked
         * \return "true" if voxel exist; "false" otherwise
         */
        bool
        isVoxelOccupiedAtPoint (const double pointX_arg,
            const double pointY_arg, const double pointZ_arg) const;

        /** \brief Check if voxel at given point from input cloud exist.
         * \param[in] pointIdx_arg point to be checked
         * \return "true" if voxel exist; "false" otherwise
         */
        bool
        isVoxelOccupiedAtPoint (const int& pointIdx_arg) const;

        /** \brief Get a PointT vector of centers of all occupied voxels.
         * \param[out] voxelCenterList_arg results are written to this vector of PointT elements
         * \return number of occupied voxels
         */
        int
        getOccupiedVoxelCenters (
            AlignedPointTVector &voxelCenterList_arg) const;

        /** \brief Get a PointT vector of centers of voxels intersected by a line segment.
         * This returns a approximation of the actual intersected voxels by walking
         * along the line with small steps. Voxels are ordered, from closest to
         * furthest w.r.t. the origin.
         * \param[in] origin origin of the line segment
         * \param[in] end end of the line segment
         * \param[out] voxel_center_list results are written to this vector of PointT elements
         * \param[in] precision determines the size of the steps: step_size = octree_resolution x precision
         * \return number of intersected voxels
         */
        int
        getApproxIntersectedVoxelCentersBySegment (
            const Eigen::Vector3f& origin, const Eigen::Vector3f& end,
            AlignedPointTVector &voxel_center_list, float precision = 0.2);

        /** \brief Delete leaf node / voxel at given point
         * \param[in] point_arg point addressing the voxel to be deleted.
         */
        void
        deleteVoxelAtPoint (const PointT& point_arg);

        /** \brief Delete leaf node / voxel at given point from input cloud
         *  \param[in] pointIdx_arg index of point addressing the voxel to be deleted.
         */
        void
        deleteVoxelAtPoint (const int& pointIdx_arg);

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Bounding box methods
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        /** \brief Investigate dimensions of pointcloud data set and define corresponding bounding box for octree. */
        void
        defineBoundingBox ();

        /** \brief Define bounding box for octree
         * \note Bounding box cannot be changed once the octree contains elements.
         * \param[in] minX_arg X coordinate of lower bounding box corner
         * \param[in] minY_arg Y coordinate of lower bounding box corner
         * \param[in] minZ_arg Z coordinate of lower bounding box corner
         * \param[in] maxX_arg X coordinate of upper bounding box corner
         * \param[in] maxY_arg Y coordinate of upper bounding box corner
         * \param[in] maxZ_arg Z coordinate of upper bounding box corner
         */
        void
        defineBoundingBox (const double minX_arg, const double minY_arg,
            const double minZ_arg, const double maxX_arg, const double maxY_arg,
            const double maxZ_arg);

        /** \brief Define bounding box for octree
         * \note Lower bounding box point is set to (0, 0, 0)
         * \note Bounding box cannot be changed once the octree contains elements.
         * \param[in] maxX_arg X coordinate of upper bounding box corner
         * \param[in] maxY_arg Y coordinate of upper bounding box corner
         * \param[in] maxZ_arg Z coordinate of upper bounding box corner
         */
        void
        defineBoundingBox (const double maxX_arg, const double maxY_arg,
            const double maxZ_arg);

        /** \brief Define bounding box cube for octree
         * \note Lower bounding box corner is set to (0, 0, 0)
         * \note Bounding box cannot be changed once the octree contains elements.
         * \param[in] cubeLen_arg side length of bounding box cube.
         */
        void
        defineBoundingBox (const double cubeLen_arg);

        /** \brief Get bounding box for octree
         * \note Bounding box cannot be changed once the octree contains elements.
         * \param[in] minX_arg X coordinate of lower bounding box corner
         * \param[in] minY_arg Y coordinate of lower bounding box corner
         * \param[in] minZ_arg Z coordinate of lower bounding box corner
         * \param[in] maxX_arg X coordinate of upper bounding box corner
         * \param[in] maxY_arg Y coordinate of upper bounding box corner
         * \param[in] maxZ_arg Z coordinate of upper bounding box corner
         */
        void
        getBoundingBox (double& minX_arg, double& minY_arg, double& minZ_arg,
            double& maxX_arg, double& maxY_arg, double& maxZ_arg) const;

        /** \brief Calculates the squared diameter of a voxel at given tree depth
         * \param[in] treeDepth_arg depth/level in octree
         * \return squared diameter
         */
        double
        getVoxelSquaredDiameter (unsigned int treeDepth_arg) const;

        /** \brief Calculates the squared diameter of a voxel at leaf depth
         * \return squared diameter
         */
        inline double getVoxelSquaredDiameter () const
        {
          return getVoxelSquaredDiameter (this->octreeDepth_);
        }

        /** \brief Calculates the squared voxel cube side length at given tree depth
         * \param[in] treeDepth_arg depth/level in octree
         * \return squared voxel cube side length
         */
        double
        getVoxelSquaredSideLen (unsigned int treeDepth_arg) const;

        /** \brief Calculates the squared voxel cube side length at leaf level
         * \return squared voxel cube side length
         */
        inline double getVoxelSquaredSideLen () const
        {
          return getVoxelSquaredSideLen (this->octreeDepth_);
        }

        /** \brief Generate bounds of the current voxel of an octree iterator
         * \param[in] iterator: octree iterator
         * \param[out] min_pt lower bound of voxel
         * \param[out] max_pt upper bound of voxel
         */
        inline void getVoxelBounds (OctreeIteratorBase<int, OctreeT>& iterator,
            Eigen::Vector3f &min_pt, Eigen::Vector3f &max_pt)
        {
          this->genVoxelBoundsFromOctreeKey (iterator.getCurrentOctreeKey (),
              iterator.getCurrentOctreeDepth (), min_pt, max_pt);
        }

      protected:

        /** \brief Add point at index from input pointcloud dataset to octree
         * \param[in] pointIdx_arg the index representing the point in the dataset given by \a setInputCloud to be added
         */
        void
        addPointIdx (const int pointIdx_arg);

        /** \brief Get point at index from input pointcloud dataset
         * \param[in] index_arg index representing the point in the dataset given by \a setInputCloud
         * \return PointT from input pointcloud dataset
         */
        const PointT&
        getPointByIndex (const unsigned int index_arg) const;

        /** \brief Find octree leaf node at a given point
         * \param[in] point_arg query point
         * \return pointer to leaf node. If leaf node does not exist, pointer is 0.
         */
        LeafT*
        findLeafAtPoint (const PointT& point_arg) const;

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Protected octree methods based on octree keys
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        /** \brief Define octree key setting and octree depth based on defined bounding box. */
        void
        getKeyBitSize ();

        /** \brief Grow the bounding box/octree until point fits
         * \param[in] pointIdx_arg point that should be within bounding box;
         */
        void
        adoptBoundingBoxToPoint (const PointT& pointIdx_arg);

        /** \brief Checks if given point is within the bounding box of the octree
         * \param[in] pointIdx_arg point to be checked for bounding box violations
         * \return "true" - no bound violation
         */
        inline bool isPointWithinBoundingBox (const PointT& pointIdx_arg) const
        {
          return (! ( (pointIdx_arg.x < minX_) || (pointIdx_arg.y < minY_)
              || (pointIdx_arg.z < minZ_) || (pointIdx_arg.x >= maxX_)
              || (pointIdx_arg.y >= maxY_) || (pointIdx_arg.z >= maxZ_)));
        }

        /** \brief Generate octree key for voxel at a given point
         * \param[in] point_arg the point addressing a voxel
         * \param[out] key_arg write octree key to this reference
         */
        void
        genOctreeKeyforPoint (const PointT & point_arg,
            OctreeKey &key_arg) const;

        /** \brief Generate octree key for voxel at a given point
         * \param[in] pointX_arg X coordinate of point addressing a voxel
         * \param[in] pointY_arg Y coordinate of point addressing a voxel
         * \param[in] pointZ_arg Z coordinate of point addressing a voxel
         * \param[out] key_arg write octree key to this reference
         */
        void
        genOctreeKeyforPoint (const double pointX_arg, const double pointY_arg,
            const double pointZ_arg, OctreeKey & key_arg) const;

        /** \brief Virtual method for generating octree key for a given point index.
         * \note This method enables to assign indices to leaf nodes during octree deserialization.
         * \param[in] data_arg index value representing a point in the dataset given by \a setInputCloud
         * \param[out] key_arg write octree key to this reference
         * \return "true" - octree keys are assignable
         */
        virtual bool
        genOctreeKeyForDataT (const int& data_arg, OctreeKey & key_arg) const;

        /** \brief Generate a point at center of leaf node voxel
         * \param[in] key_arg octree key addressing a leaf node.
         * \param[out] point_arg write leaf node voxel center to this point reference
         */
        void
        genLeafNodeCenterFromOctreeKey (const OctreeKey & key_arg,
            PointT& point_arg) const;

        /** \brief Generate a point at center of octree voxel at given tree level
         * \param[in] key_arg octree key addressing an octree node.
         * \param[in] treeDepth_arg octree depth of query voxel
         * \param[out] point_arg write leaf node center point to this reference
         */
        void
        genVoxelCenterFromOctreeKey (const OctreeKey & key_arg,
            unsigned int treeDepth_arg, PointT& point_arg) const;

        /** \brief Generate bounds of an octree voxel using octree key and tree depth arguments
         * \param[in] key_arg octree key addressing an octree node.
         * \param[in] treeDepth_arg octree depth of query voxel
         * \param[out] min_pt lower bound of voxel
         * \param[out] max_pt upper bound of voxel
         */
        void
        genVoxelBoundsFromOctreeKey (const OctreeKey & key_arg,
            unsigned int treeDepth_arg, Eigen::Vector3f &min_pt,
            Eigen::Vector3f &max_pt) const;

        /** \brief Recursively search the tree for all leaf nodes and return a vector of voxel centers.
         * \param[in] node_arg current octree node to be explored
         * \param[in] key_arg octree key addressing a leaf node.
         * \param[out] voxelCenterList_arg results are written to this vector of PointT elements
         * \return number of voxels found
         */
        int
        getOccupiedVoxelCentersRecursive (const BranchNode* node_arg,
            const OctreeKey& key_arg,
            AlignedPointTVector &voxelCenterList_arg) const;

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Globals
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /** \brief Pointer to input point cloud dataset. */
        PointCloudConstPtr input_;

        /** \brief A pointer to the vector of point indices to use. */
        IndicesConstPtr indices_;

        /** \brief Epsilon precision (error bound) for nearest neighbors searches. */
        double epsilon_;

        /** \brief Octree resolution. */
        double resolution_;

        // Octree bounding box coordinates
        double minX_;
        double maxX_;

        double minY_;
        double maxY_;

        double minZ_;
        double maxZ_;

        /** \brief Flag indicating if octree has defined bounding box. */
        bool boundingBoxDefined_;
    };
  }
}

#endif

