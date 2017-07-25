/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012, Jeremie Papon
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
 *  Author : jpapon@gmail.com
 *  Email  : jpapon@gmail.com
 */

#ifndef PCL_OCTREE_POINTCLOUD_ADJACENCY_H_
#define PCL_OCTREE_POINTCLOUD_ADJACENCY_H_

#include <pcl/octree/boost.h>
#include <pcl/octree/octree_pointcloud.h>
#include <pcl/octree/octree_pointcloud_adjacency_container.h>

#include <set>
#include <list>

namespace pcl
{

  namespace octree
  {
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief @b Octree pointcloud voxel class which maintains adjacency information for its voxels.
      *
      * This pointcloud octree class generates an octree from a point cloud (zero-copy). The octree pointcloud is
      * initialized with its voxel resolution. Its bounding box is automatically adjusted or can be predefined.
      *
      * The OctreePointCloudAdjacencyContainer class can be used to store data in leaf nodes.
      *
      * An optional transform function can be provided which changes how the voxel grid is computed - this can be used to,
      * for example, make voxel bins larger as they increase in distance from the origin (camera).
      * \note See SupervoxelClustering for an example of how to provide a transform function.
      *
      * If used in academic work, please cite:
      *
      * - J. Papon, A. Abramov, M. Schoeler, F. Woergoetter
      *   Voxel Cloud Connectivity Segmentation - Supervoxels from PointClouds
      *   In Proceedings of the IEEE Conference on Computer Vision and Pattern Recognition (CVPR) 2013
      *
      * \ingroup octree
      * \author Jeremie Papon (jpapon@gmail.com) */
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template <typename PointT,
              typename LeafContainerT = OctreePointCloudAdjacencyContainer<PointT>,
              typename BranchContainerT = OctreeContainerEmpty>
    class OctreePointCloudAdjacency : public OctreePointCloud<PointT, LeafContainerT, BranchContainerT>
    {

      public:

        typedef OctreeBase<LeafContainerT, BranchContainerT> OctreeBaseT;

        typedef OctreePointCloudAdjacency<PointT, LeafContainerT, BranchContainerT> OctreeAdjacencyT;
        typedef boost::shared_ptr<OctreeAdjacencyT> Ptr;
        typedef boost::shared_ptr<const OctreeAdjacencyT> ConstPtr;

        typedef OctreePointCloud<PointT, LeafContainerT, BranchContainerT, OctreeBaseT> OctreePointCloudT;
        typedef typename OctreePointCloudT::LeafNode LeafNode;
        typedef typename OctreePointCloudT::BranchNode BranchNode;

        typedef pcl::PointCloud<PointT> PointCloud;
        typedef boost::shared_ptr<PointCloud> PointCloudPtr;
        typedef boost::shared_ptr<const PointCloud> PointCloudConstPtr;

        // Iterators are friends
        friend class OctreeIteratorBase<OctreeAdjacencyT>;
        friend class OctreeDepthFirstIterator<OctreeAdjacencyT>;
        friend class OctreeBreadthFirstIterator<OctreeAdjacencyT>;
        friend class OctreeLeafNodeIterator<OctreeAdjacencyT>;

        // Octree default iterators
        typedef OctreeDepthFirstIterator<OctreeAdjacencyT> Iterator;
        typedef const OctreeDepthFirstIterator<OctreeAdjacencyT> ConstIterator;

        Iterator depth_begin (unsigned int max_depth_arg = 0) { return Iterator (this, max_depth_arg); }
        const Iterator depth_end () { return Iterator (); }

        // Octree leaf node iterators
        typedef OctreeLeafNodeIterator<OctreeAdjacencyT> LeafNodeIterator;
        typedef const OctreeLeafNodeIterator<OctreeAdjacencyT> ConstLeafNodeIterator;

        LeafNodeIterator leaf_begin (unsigned int max_depth_arg = 0) { return LeafNodeIterator (this, max_depth_arg); }
        const LeafNodeIterator leaf_end () { return LeafNodeIterator (); }

        // BGL graph
        typedef boost::adjacency_list<boost::setS, boost::setS, boost::undirectedS, PointT, float> VoxelAdjacencyList;
        typedef typename VoxelAdjacencyList::vertex_descriptor VoxelID;
        typedef typename VoxelAdjacencyList::edge_descriptor EdgeID;

        // Leaf vector - pointers to all leaves
        typedef std::vector<LeafContainerT*> LeafVectorT;

        // Fast leaf iterators that don't require traversing tree
        typedef typename LeafVectorT::iterator iterator;
        typedef typename LeafVectorT::const_iterator const_iterator;

        inline iterator begin () { return (leaf_vector_.begin ()); }
        inline iterator end ()   { return (leaf_vector_.end ()); }
        inline LeafContainerT* at (size_t idx)   { return leaf_vector_.at (idx); }
        
        // Size of neighbors
        inline size_t size () const { return leaf_vector_.size (); }

        /** \brief Constructor.
          *
          * \param[in] resolution_arg Octree resolution at lowest octree level (voxel size) */
        OctreePointCloudAdjacency (const double resolution_arg);

        /** \brief Empty class destructor. */
        virtual ~OctreePointCloudAdjacency ()
        {
        }

        /** \brief Adds points from cloud to the octree.
          *
          * \note This overrides addPointsFromInputCloud() from the OctreePointCloud class. */
        void
        addPointsFromInputCloud ();

        /** \brief Gets the leaf container for a given point.
          *
          * \param[in] point_arg Point to search for
          *
          * \returns Pointer to the leaf container - null if no leaf container found. */
        LeafContainerT*
        getLeafContainerAtPoint (const PointT& point_arg) const;

        /** \brief Computes an adjacency graph of voxel relations.
          *
          * \warning This slows down rapidly as cloud size increases due to the number of edges.
          *
          * \param[out] voxel_adjacency_graph Boost Graph Library Adjacency graph of the voxel touching relationships.
          * Vertices are PointT, edges represent touching, and edge lengths are the distance between the points. */
        void
        computeVoxelAdjacencyGraph (VoxelAdjacencyList &voxel_adjacency_graph);

        /** \brief Sets a point transform (and inverse) used to transform the space of the input cloud.
          *
          * This is useful for changing how adjacency is calculated - such as relaxing the adjacency criterion for
          * points further from the camera.
          *
          * \param[in] transform_func A boost:function pointer to the transform to be used. The transform must have one
          * parameter (a point) which it modifies in place. */
        void
        setTransformFunction (boost::function<void (PointT &p)> transform_func)
        {
          transform_func_ = transform_func;
        }

        /** \brief Tests whether input point is occluded from specified camera point by other voxels.
          *
          * \param[in] point_arg Point to test for
          * \param[in] camera_pos Position of camera, defaults to origin
          *
          * \returns True if path to camera is blocked by a voxel, false otherwise. */
        bool
        testForOcclusion (const PointT& point_arg, const PointXYZ &camera_pos = PointXYZ (0, 0, 0));

      protected:

        /** \brief Add point at index from input pointcloud dataset to octree.
          *
          * \param[in] point_idx_arg The index representing the point in the dataset given by setInputCloud() to be added
          *
          * \note This virtual implementation allows the use of a transform function to compute keys. */
         virtual void
         addPointIdx (const int point_idx_arg);

        /** \brief Fills in the neighbors fields for new voxels.
          *
          * \param[in] key_arg Key of the voxel to check neighbors for
          * \param[in] leaf_container Pointer to container of the leaf to check neighbors for */
        void
        computeNeighbors (OctreeKey &key_arg, LeafContainerT* leaf_container);

        /** \brief Generates octree key for specified point (uses transform if provided).
          *
          * \param[in] point_arg Point to generate key for
          * \param[out] key_arg Resulting octree key */
        void
        genOctreeKeyforPoint (const PointT& point_arg, OctreeKey& key_arg) const;

      private:

        /** \brief Add point at given index from input point cloud to octree.
          *
          * Index will be also added to indices vector. This functionality is not enabled for adjacency octree. */
        using OctreePointCloudT::addPointFromCloud;

        /** \brief Add point simultaneously to octree and input point cloud.
          *
          * This functionality is not enabled for adjacency octree. */
        using OctreePointCloudT::addPointToCloud;

        using OctreePointCloudT::input_;
        using OctreePointCloudT::resolution_;
        using OctreePointCloudT::min_x_;
        using OctreePointCloudT::min_y_;
        using OctreePointCloudT::min_z_;
        using OctreePointCloudT::max_x_;
        using OctreePointCloudT::max_y_;
        using OctreePointCloudT::max_z_;

        /// Local leaf pointer vector used to make iterating through leaves fast.
        LeafVectorT leaf_vector_;

        boost::function<void (PointT &p)> transform_func_;

    };

  }

}

// Note: Do not precompile this octree type because it is typically used with custom leaf containers.
#include <pcl/octree/impl/octree_pointcloud_adjacency.hpp>

#endif // PCL_OCTREE_POINTCLOUD_ADJACENCY_H_

