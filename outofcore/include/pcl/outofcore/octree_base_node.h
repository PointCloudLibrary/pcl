/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
 *  Copyright (c) 2012, Urban Robotics, Inc.
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
 *  $Id$
 */

#pragma once

#include <memory>
#include <mutex>
#include <random>

#include <pcl/common/io.h>
#include <pcl/PCLPointCloud2.h>

#include <pcl/outofcore/boost.h>
#include <pcl/outofcore/octree_base.h>
#include <pcl/outofcore/octree_disk_container.h>
#include <pcl/outofcore/outofcore_node_data.h>

#include <pcl/octree/octree_nodes.h>

namespace pcl
{
  namespace outofcore
  {
    // Forward Declarations
    template<typename ContainerT, typename PointT>
    class OutofcoreOctreeBaseNode;

    template<typename ContainerT, typename PointT>
    class OutofcoreOctreeBase;

    /** \brief Non-class function which creates a single child leaf; used with \ref queryBBIntersects_noload to avoid loading the data from disk */
    template<typename ContainerT, typename PointT> OutofcoreOctreeBaseNode<ContainerT, PointT>*
    makenode_norec (const boost::filesystem::path &path, OutofcoreOctreeBaseNode<ContainerT, PointT>* super);

    /** \brief Non-class method which performs a bounding box query without loading any of the point cloud data from disk */
    template<typename ContainerT, typename PointT> void
    queryBBIntersects_noload (const boost::filesystem::path &root_node, const Eigen::Vector3d &min, const Eigen::Vector3d &max, const std::uint32_t query_depth, std::list<std::string> &bin_name);

    /** \brief Non-class method overload */
    template<typename ContainerT, typename PointT> void
    queryBBIntersects_noload (OutofcoreOctreeBaseNode<ContainerT, PointT>* current, const Eigen::Vector3d&, const Eigen::Vector3d &max, const std::uint32_t query_depth, std::list<std::string> &bin_name);

    /** \class OutofcoreOctreeBaseNode 
     *
     *  \note Code was adapted from the Urban Robotics out of core octree implementation. 
     *  Contact Jacob Schloss <jacob.schloss@urbanrobotics.net> with any questions. 
     *  http://www.urbanrobotics.net/
     *
     *  \brief OutofcoreOctreeBaseNode Class internally representing nodes of an
     *  outofcore octree, with accessors to its data via the \ref
     *  pcl::outofcore::OutofcoreOctreeDiskContainer class or \ref pcl::outofcore::OutofcoreOctreeRamContainer class,
     *  whichever it is templated against.  
     * 
     *  \ingroup outofcore
     *  \author Jacob Schloss (jacob.schloss@urbanrobotics.net)
     *
     */
    template<typename ContainerT = OutofcoreOctreeDiskContainer<pcl::PointXYZ>, typename PointT = pcl::PointXYZ>
    class OutofcoreOctreeBaseNode : public pcl::octree::OctreeNode
    {
      friend class OutofcoreOctreeBase<ContainerT, PointT> ;

      //these methods can be rewritten with the iterators. 
      friend OutofcoreOctreeBaseNode<ContainerT, PointT>*
      makenode_norec<ContainerT, PointT> (const boost::filesystem::path &path, OutofcoreOctreeBaseNode<ContainerT, PointT>* super);
  
      friend void
      queryBBIntersects_noload<ContainerT, PointT> (const boost::filesystem::path &rootnode, const Eigen::Vector3d &min, const Eigen::Vector3d &max, const std::uint32_t query_depth, std::list<std::string> &bin_name);

      friend void
      queryBBIntersects_noload<ContainerT, PointT> (OutofcoreOctreeBaseNode<ContainerT, PointT>* current, const Eigen::Vector3d &min, const Eigen::Vector3d &max, const std::uint32_t query_depth, std::list<std::string> &bin_name);
  
      public:
        using octree_disk = OutofcoreOctreeBase<OutofcoreOctreeDiskContainer < PointT > , PointT >;
        using octree_disk_node = OutofcoreOctreeBaseNode<OutofcoreOctreeDiskContainer < PointT > , PointT >;

        using AlignedPointTVector = std::vector<PointT, Eigen::aligned_allocator<PointT> >;

        using node_type_t = pcl::octree::node_type_t;

        const static std::string node_index_basename;
        const static std::string node_container_basename;
        const static std::string node_index_extension;
        const static std::string node_container_extension;
        const static double sample_percent_;

        /** \brief Empty constructor; sets pointers for children and for bounding boxes to 0
         */
        OutofcoreOctreeBaseNode ();

        /** \brief Create root node and directory */
        OutofcoreOctreeBaseNode (const Eigen::Vector3d &bb_min, const Eigen::Vector3d &bb_max, OutofcoreOctreeBase<ContainerT, PointT> * const tree, const boost::filesystem::path &root_name);

        /** \brief Will recursively delete all children calling recFreeChildrein */
        
        ~OutofcoreOctreeBaseNode ();

        //query
        /** \brief gets the minimum and maximum corner of the bounding box represented by this node
         * \param[out] min_bb returns the minimum corner of the bounding box indexed by 0-->X, 1-->Y, 2-->Z 
         * \param[out] max_bb returns the maximum corner of the bounding box indexed by 0-->X, 1-->Y, 2-->Z 
         */
        virtual inline void
        getBoundingBox (Eigen::Vector3d &min_bb, Eigen::Vector3d &max_bb) const
        {
          node_metadata_->getBoundingBox (min_bb, max_bb);
        }


        const boost::filesystem::path&
        getPCDFilename () const
        {
          return node_metadata_->getPCDFilename ();
        }

        const boost::filesystem::path&
        getMetadataFilename () const
        {
          return node_metadata_->getMetadataFilename ();
        }

        void
        queryFrustum (const double planes[24], std::list<std::string>& file_names);

        void
        queryFrustum (const double planes[24], std::list<std::string>& file_names, const std::uint32_t query_depth, const bool skip_vfc_check = false);

        void
        queryFrustum (const double planes[24], const Eigen::Vector3d &eye, const Eigen::Matrix4d &view_projection_matrix, std::list<std::string>& file_names, const std::uint32_t query_depth, const bool skip_vfc_check = false);

        //point extraction
        /** \brief Recursively add points that fall into the queried bounding box up to the \b query_depth 
         *
         *  \param[in] min_bb the minimum corner of the bounding box, indexed by X,Y,Z coordinates
         *  \param[in] max_bb the maximum corner of the bounding box, indexed by X,Y,Z coordinates
         *  \param[in] query_depth the maximum depth to query in the octree for points within the bounding box
         *  \param[out] dst destion of points returned by the queries
         */
        virtual void
        queryBBIncludes (const Eigen::Vector3d &min_bb, const Eigen::Vector3d &max_bb, std::size_t query_depth, AlignedPointTVector &dst);

        /** \brief Recursively add points that fall into the queried bounding box up to the \b query_depth
         *
         *  \param[in] min_bb the minimum corner of the bounding box, indexed by X,Y,Z coordinates
         *  \param[in] max_bb the maximum corner of the bounding box, indexed by X,Y,Z coordinates
         *  \param[in] query_depth the maximum depth to query in the octree for points within the bounding box
         *  \param[out] dst_blob destion of points returned by the queries
         */
        virtual void
        queryBBIncludes (const Eigen::Vector3d &min_bb, const Eigen::Vector3d &max_bb, std::size_t query_depth, const pcl::PCLPointCloud2::Ptr &dst_blob);

        /** \brief Recursively add points that fall into the queried bounding box up to the \b query_depth 
         *
         *  \param[in] min_bb the minimum corner of the bounding box, indexed by X,Y,Z coordinates
         *  \param[in] max_bb the maximum corner of the bounding box, indexed by X,Y,Z coordinates
         *  \param[in] query_depth
         *  \param percent
         *  \param[out] v std::list of points returned by the query
         */
        virtual void
        queryBBIncludes_subsample (const Eigen::Vector3d &min_bb, const Eigen::Vector3d &max_bb, std::uint64_t query_depth, const double percent, AlignedPointTVector &v);

        virtual void
        queryBBIncludes_subsample (const Eigen::Vector3d &min_bb, const Eigen::Vector3d &max_bb, std::uint64_t query_depth, const pcl::PCLPointCloud2::Ptr& dst_blob, double percent = 1.0);

        /** \brief Recursive acquires PCD paths to any node with which the queried bounding box intersects (at query_depth only).
         */
        virtual void
        queryBBIntersects (const Eigen::Vector3d &min_bb, const Eigen::Vector3d &max_bb, const std::uint32_t query_depth, std::list<std::string> &file_names);

        /** \brief Write the voxel size to stdout at \c query_depth 
         * \param[in] query_depth The depth at which to print the size of the voxel/bounding boxes
         */
        virtual void
        printBoundingBox (const std::size_t query_depth) const;

        /** \brief add point to this node if we are a leaf, or find the leaf below us that is supposed to take the point 
         *  \param[in] p vector of points to add to the leaf
         *  \param[in] skip_bb_check whether to check if the point's coordinates fall within the bounding box
         */
        virtual std::uint64_t
        addDataToLeaf (const AlignedPointTVector &p, const bool skip_bb_check = true);

        virtual std::uint64_t
        addDataToLeaf (const std::vector<const PointT*> &p, const bool skip_bb_check = true);

        /** \brief Add a single PCLPointCloud2 object into the octree.
         *
         * \param[in] input_cloud
         * \param[in] skip_bb_check (default = false)
         */
        virtual std::uint64_t
        addPointCloud (const pcl::PCLPointCloud2::Ptr &input_cloud, const bool skip_bb_check = false);

        /** \brief Add a single PCLPointCloud2 into the octree and build the subsampled LOD during construction; this method of LOD construction is <b>not</b> multiresolution. Rather, there are no redundant data. */
        virtual std::uint64_t
        addPointCloud_and_genLOD (const pcl::PCLPointCloud2::Ptr input_cloud); //, const bool skip_bb_check);
        
        /** \brief Recursively add points to the leaf and children subsampling LODs
         * on the way down.
         *
         * \note rng_mutex_ lock occurs
         */
        virtual std::uint64_t
        addDataToLeaf_and_genLOD (const AlignedPointTVector &p, const bool skip_bb_check);

        /** \brief Write a python visual script to @b file
         * \param[in] file output file stream to write the python visual script
         */
        void
        writeVPythonVisual (std::ofstream &file);

        virtual int
        read (pcl::PCLPointCloud2::Ptr &output_cloud);

        inline node_type_t
        getNodeType () const override
        {
          if(this->getNumChildren () > 0)
          {
            return (pcl::octree::BRANCH_NODE);
          }
          return (pcl::octree::LEAF_NODE);
        }
        
        
        OutofcoreOctreeBaseNode* 
        deepCopy () const override
        {
          OutofcoreOctreeBaseNode* res = nullptr;
          PCL_THROW_EXCEPTION (PCLException, "Not implemented\n");
          return (res);
        }

        virtual inline std::size_t
        getDepth () const
        {
          return (this->depth_);
        }

        /** \brief Returns the total number of children on disk */
        virtual std::size_t
        getNumChildren () const 
        {
          std::size_t res = this->countNumChildren ();
          return (res);
        }

        /** \brief Count loaded chilren */
        virtual std::size_t
        getNumLoadedChildren ()  const
        {
          std::size_t res = this->countNumLoadedChildren ();
          return (res);
        }        
        
        /** \brief Returns a pointer to the child in octant index_arg */
        virtual OutofcoreOctreeBaseNode*
        getChildPtr (std::size_t index_arg) const;

        /** \brief Gets the number of points available in the PCD file */
        virtual std::uint64_t
        getDataSize () const;

        inline virtual void
        clearData ()
        {
          //clears write cache and removes PCD file from disk
          this->payload_->clear ();
        }

      ///////////////////////////////////////////////////////////////////////////////
      // PROTECTED METHODS
      ////////////////////////////////////////////////////////////////////////////////
      protected:
        /** \brief Load from disk 
         * If creating root, path is full name. If creating any other
         * node, path is dir; throws exception if directory or metadata not found
         *
         * \param[in] directory_path pathname
         * \param[in] super
         * \param[in] load_all
         * \throws PCLException if directory is missing
         * \throws PCLException if node index is missing
         */
        OutofcoreOctreeBaseNode (const boost::filesystem::path &directory_path, OutofcoreOctreeBaseNode<ContainerT, PointT>* super, bool load_all);

        /** \brief Create root node and directory
         *
         * Initializes the root node and performs initial filesystem checks for the octree; 
         * throws OctreeException::OCT_BAD_PATH if root directory is an existing file
         *
         * \param bb_min triple of x,y,z minima for bounding box
         * \param bb_max triple of x,y,z maxima for bounding box
         * \param tree address of the tree data structure that will hold this initial root node
         * \param rootname Root directory for location of on-disk octree storage; if directory 
         * doesn't exist, it is created; if "rootname" is an existing file, 
         * 
         * \throws PCLException if the specified path already exists
         */
        void init_root_node (const Eigen::Vector3d &bb_min, const Eigen::Vector3d &bb_max, OutofcoreOctreeBase<ContainerT, PointT> * const tree, const boost::filesystem::path &rootname);

        /** \brief no copy construction right now */
        OutofcoreOctreeBaseNode (const OutofcoreOctreeBaseNode &rval);

        /** \brief Operator= is not implemented */
        OutofcoreOctreeBaseNode&
        operator= (const OutofcoreOctreeBaseNode &rval);

        /** \brief Counts the number of child directories on disk; used to update num_children_ */
        virtual std::size_t
        countNumChildren () const;

        /** \brief Counts the number of loaded chilren by testing the \c children_ array; 
         *  used to update num_loaded_chilren_ internally 
         */
        virtual std::size_t
        countNumLoadedChildren () const;
        
        /** \brief Save node's metadata to file
         * \param[in] recursive if false, save only this node's metadata to file; if true, recursively
         * save all children's metadata to files as well
         */
        void
        saveIdx (bool recursive);

        /** \brief Randomly sample point data 
         */
        void
        randomSample (const AlignedPointTVector &p, AlignedPointTVector &insertBuff, const bool skip_bb_check);

        /** \brief Subdivide points to pass to child nodes */
        void
        subdividePoints (const AlignedPointTVector &p, std::vector< AlignedPointTVector > &c, const bool skip_bb_check);
        /** \brief Subdivide a single point into a specific child node */
        void
        subdividePoint (const PointT &point, std::vector< AlignedPointTVector > &c);

        /** \brief Add data to the leaf when at max depth of tree. If
         *   skip_bb_check is true, adds to the node regardless of the
         *   bounding box it represents; otherwise only adds points that
         *   fall within the bounding box 
         *
         *  \param[in] p vector of points to attempt to add to the tree
         *  \param[in] skip_bb_check if @b true, doesn't check that points
         *  are in the proper bounding box; if @b false, only adds the
         *  points that fall into the bounding box to this node 
         *  \return number of points successfully added
         */
        std::uint64_t
        addDataAtMaxDepth (const AlignedPointTVector &p, const bool skip_bb_check = true);

        /** \brief Add data to the leaf when at max depth of tree. If
         *   \c skip_bb_check is true, adds to the node regardless of the
         *   bounding box it represents; otherwise only adds points that
         *   fall within the bounding box 
         *
         *  \param[in] input_cloud PCLPointCloud2 points to attempt to add to the tree;
         *  \warning PCLPointCloud2 inserted into the tree must have x,y,z fields, and must be of same type of any other points inserted in the tree
         *  \param[in] skip_bb_check (default true) if @b true, doesn't check that points
         *  are in the proper bounding box; if @b false, only adds the
         *  points that fall into the bounding box to this node 
         *  \return number of points successfully added
         */
        std::uint64_t
        addDataAtMaxDepth (const pcl::PCLPointCloud2::Ptr input_cloud, const bool skip_bb_check = true);
        
        /** \brief Tests whether the input bounding box intersects with the current node's bounding box 
         *  \param[in] min_bb The minimum corner of the input bounding box
         *  \param[in] max_bb The maximum corner of the input bounding box
         *  \return bool True if any portion of the bounding box intersects with this node's bounding box; false otherwise
         */
        inline bool
        intersectsWithBoundingBox (const Eigen::Vector3d &min_bb, const Eigen::Vector3d &max_bb) const;

        /** \brief Tests whether the input bounding box falls inclusively within this node's bounding box
         *  \param[in] min_bb The minimum corner of the input bounding box
         *  \param[in] max_bb The maximum corner of the input bounding box
         *  \return bool True if the input bounding box falls inclusively within the boundaries of this node's bounding box
         **/
        inline bool
        inBoundingBox (const Eigen::Vector3d &min_bb, const Eigen::Vector3d &max_bb) const;

        /** \brief Tests whether \c point falls within the input bounding box
         *  \param[in] min_bb The minimum corner of the input bounding box
         *  \param[in] max_bb The maximum corner of the input bounding box
         *  \param[in] point The test point
         */
        bool
        pointInBoundingBox (const Eigen::Vector3d &min_bb, const Eigen::Vector3d &max_bb, const Eigen::Vector3d &point);

        /** \brief Tests whether \c p falls within the input bounding box
         *  \param[in] min_bb The minimum corner of the input bounding box
         *  \param[in] max_bb The maximum corner of the input bounding box
         *  \param[in] p The point to be tested
         **/
        static bool
        pointInBoundingBox (const Eigen::Vector3d &min_bb, const Eigen::Vector3d &max_bb, const PointT &p);

        /** \brief Tests whether \c x, \c y, and \c z fall within the input bounding box
         *  \param[in] min_bb The minimum corner of the input bounding box
         *  \param[in] max_bb The maximum corner of the input bounding box
         *  \param x
         *  \param y
         *  \param z
          **/
        static bool
        pointInBoundingBox (const Eigen::Vector3d &min_bb, const Eigen::Vector3d &max_bb, const double x, const double y, const double z);

        /** \brief Tests if specified point is within bounds of current node's bounding box */
        inline bool
        pointInBoundingBox (const PointT &p) const;

        /** \brief Creates child node \c idx
         *  \param[in] idx Index (0-7) of the child node
         */
        void
        createChild (const std::size_t idx);

        /** \brief Write JSON metadata for this node to file */
        void
        saveMetadataToFile (const boost::filesystem::path &path);

        /** \brief Method which recursively free children of this node 
         */
        void
        recFreeChildren ();

        /** \brief Number of points in the payload */
        inline std::uint64_t
        size () const
        {
          return (payload_->size ());
        }

        void
        flushToDiskRecursive ();

        /** \brief Loads the nodes metadata from the JSON file 
         */
        void
        loadFromFile (const boost::filesystem::path &path, OutofcoreOctreeBaseNode* super);

        /** \brief Recursively converts data files to ascii XZY files */
        void
        convertToXYZRecursive ();

        /** \brief Private constructor used for children 
         */
        OutofcoreOctreeBaseNode (const Eigen::Vector3d &bb_min, const Eigen::Vector3d &bb_max, const char* dir, OutofcoreOctreeBaseNode<ContainerT, PointT>* super);

        /** \brief Copies points from this and all children into a single point container (std::list)
         */
        void
        copyAllCurrentAndChildPointsRec (std::list<PointT> &v);

        void
        copyAllCurrentAndChildPointsRec_sub (std::list<PointT> &v, const double percent);

        /** \brief Returns whether or not a node has unloaded children data */
        bool
        hasUnloadedChildren () const;

        /** \brief Load nodes child data creating new nodes for each */
        virtual void
        loadChildren (bool recursive);

        /** \brief Gets a vector of occupied voxel centers
         * \param[out] voxel_centers
         * \param[in] query_depth
         */
        void
        getOccupiedVoxelCentersRecursive (AlignedPointTVector &voxel_centers, const std::size_t query_depth);

        /** \brief Gets a vector of occupied voxel centers
         * \param[out] voxel_centers
         * \param[in] query_depth
         */
        void
        getOccupiedVoxelCentersRecursive (std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > &voxel_centers, const std::size_t query_depth);

        /** \brief Sorts the indices based on x,y,z fields and pushes the index into the proper octant's vector;
         *  This could be overloaded with a parallelized implementation
         */
        void
        sortOctantIndices (const pcl::PCLPointCloud2::Ptr &input_cloud, std::vector< pcl::Indices > &indices, const Eigen::Vector3d &mid_xyz);

        /** \brief Enlarges the shortest two sidelengths of the
         *  bounding box to a cubic shape; operation is done in
         *  place.
        */
        void
        enlargeToCube (Eigen::Vector3d &bb_min, Eigen::Vector3d &bb_max);

        /** \brief The tree we belong to */
        OutofcoreOctreeBase<ContainerT, PointT>* m_tree_;//
        /** \brief The root node of the tree we belong to */
        OutofcoreOctreeBaseNode* root_node_;//
        /** \brief super-node */
        OutofcoreOctreeBaseNode* parent_;
        /** \brief Depth in the tree, root is 0, root's children are 1, ... */
        std::size_t depth_;
        /** \brief The children of this node */
        std::vector<OutofcoreOctreeBaseNode*> children_;

        /** \brief Number of children on disk. This is only changed when a new node is created */
        std::uint64_t num_children_;

        /** \brief Number of loaded children this node has
         *
         *  "Loaded" means child OctreeBaseNodes have been allocated,
         *  and their metadata files have been loaded into
         *  memory. num_loaded_children_ <= num_children_
         */
        std::uint64_t num_loaded_children_;

        /** \brief what holds the points. currently a custom class, but in theory
         * you could use an stl container if you rewrote some of this class. I used
         * to use deques for this... */
        std::shared_ptr<ContainerT> payload_;

        /** \brief Random number generator mutex */
        static std::mutex rng_mutex_;

        /** \brief Mersenne Twister: A 623-dimensionally equidistributed uniform
         * pseudo-random number generator */
        static std::mt19937 rng_;

        /** \brief Extension for this class to find the pcd files on disk */
        const static std::string pcd_extension;

        OutofcoreOctreeNodeMetadata::Ptr node_metadata_;
    };
  }//namespace outofcore
}//namespace pcl
