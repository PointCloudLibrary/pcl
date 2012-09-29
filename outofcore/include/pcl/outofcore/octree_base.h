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
 *  $Id: octree_base.h 6927M 2012-08-25 00:09:53Z (local) $
 */

#ifndef PCL_OUTOFCORE_OUTOFCOREOCTREEBASE_H_
#define PCL_OUTOFCORE_OUTOFCOREOCTREEBASE_H_

#include <pcl/outofcore/boost.h>
#include <pcl/common/io.h>

//outofcore classes
#include <pcl/outofcore/octree_base_node.h>
#include <pcl/outofcore/octree_disk_container.h>
#include <pcl/outofcore/octree_ram_container.h>

//outofcore iterators
#include <pcl/outofcore/OutofcoreIteratorBase.h>
#include <pcl/outofcore/OutofcoreDepthFirstIterator.h>
#include <pcl/outofcore/impl/OutofcoreDepthFirstIterator.hpp>

//outofcore metadata
#include <pcl/outofcore/metadata.h>
#include <pcl/outofcore/outofcore_base_data.h>

#include <pcl/filters/filter.h>

#include <sensor_msgs/PointCloud2.h>

namespace pcl
{
  namespace outofcore
  {
    /** \class OutofcoreOctreeBase 
     *  \brief This code defines the octree used for point storage at Urban Robotics. 
     * 
     *  \note Code was adapted from the Urban Robotics out of core octree implementation. 
     *  Contact Jacob Schloss <jacob.schloss@urbanrobotics.net> with any questions. 
     *  http://www.urbanrobotics.net/
     *
     *  The primary purpose of this class is an interface to the
     *  recursive traversal (recursion handled by \ref OutofcoreOctreeBaseNode) of the
     *  in-memory/top-level octree structure. The metadata in each node
     *  can be loaded entirely into main memory, and the tree traversed
     *  recursively in this state. This class provides an the interface
     *  for: 
     *               -# Point/Region insertion methods 
     *               -# Frustrum/box/region queries
     *               -# Parameterization of compression, resolution, container type, etc...
     *
     *  \todo downsampling of queries
     *  \todo downsampling of octree during construction (or leave that to the user's own preprocessing)
     *  \todo parameterize compression 
     *  \todo parameterize container type (template?)
     *  \todo adjust for varying densities at different LODs
     *  \todo Re-implement buildLOD for PointCloud2 / pcd files
     *  \todo Add support for an array of input clouds or locations of pcd files on disk
     *  \todo clean up access specifiers to streamline public interface
     *  \todo add PCL instantiation macro into templated hpp files
     *
     *  \ingroup outofcore
     *  \author Jacob Schloss (jacob.schloss@urbanrobotics.net)
     *
     */
    template<typename ContainerT, typename PointT>
    class OutofcoreOctreeBase
    {
      friend class OutofcoreOctreeBaseNode<ContainerT, PointT>;
      friend class pcl::outofcore::OutofcoreIteratorBase<PointT, ContainerT>;

      public:
        // public typedefs
        // UR Typedefs
        typedef OutofcoreOctreeBase<OutofcoreOctreeDiskContainer < PointT > , PointT > octree_disk;
        typedef OutofcoreOctreeBaseNode<OutofcoreOctreeDiskContainer < PointT > , PointT > octree_disk_node;

        typedef OutofcoreOctreeBase<OutofcoreOctreeRamContainer< PointT> , PointT> octree_ram;
        typedef OutofcoreOctreeBaseNode<OutofcoreOctreeRamContainer<PointT> , PointT> octree_ram_node;

        typedef OutofcoreOctreeBaseNode<ContainerT, PointT> OutofcoreNodeType;

        typedef OutofcoreOctreeBaseNode<ContainerT, PointT> BranchNode;
        typedef OutofcoreOctreeBaseNode<ContainerT, PointT> LeafNode;

        typedef OutofcoreDepthFirstIterator<PointT, ContainerT> Iterator;
        typedef const OutofcoreDepthFirstIterator<PointT, ContainerT> ConstIterator;

        typedef boost::shared_ptr<OutofcoreOctreeBase<ContainerT, PointT> > Ptr;
        typedef boost::shared_ptr<const OutofcoreOctreeBase<ContainerT, PointT> > ConstPtr;

        typedef pcl::PointCloud<PointT> PointCloud;
        typedef boost::shared_ptr<std::vector<int> > IndicesPtr;
        typedef boost::shared_ptr<const std::vector<int> > IndicesConstPtr;

        typedef boost::shared_ptr<PointCloud> PointCloudPtr;
        typedef boost::shared_ptr<const PointCloud> PointCloudConstPtr;

        typedef std::vector<PointT, Eigen::aligned_allocator<PointT> > AlignedPointTVector;

        // Constructors
        // -----------------------------------------------------------------------

        /** \brief Load an existing tree
         *
         * If load_all is set, the BB and point count for every node is loaded,
         * otherwise only the root node is actually created, and the rest will be
         * generated on insertion or query.
         *
         * \param Path to the top-level tree/tree.oct_idx metadata file
         * \param load_all Load entire tree metadata (does not load any points from disk)
         * \throws PCLException for bad extension (root node metadata must be .oct_idx extension)
         */
        OutofcoreOctreeBase (const boost::filesystem::path &root_node_name, const bool load_all);

        /** \brief Create a new tree
         *
         * Create a new tree rootname with specified bounding box; will remove and overwrite existing tree with the same name
         *
         * Makes a tree with enough LODs for the lowest bin to be have a diagonal
         * smaller than node_dim_meters, or a volume less than (node_dim_meters)^3.
         * Meters is a misnomer: the coord system is assumed to be Cartesian, but
         * not any particular unit
         *
         * \param min Bounding box min
         * \param max Bounding box max
         * \param node_dim_meters Node dimension in meters (assuming your point data is in meters)
         * \param root_node_name must end in ".oct_idx" 
         * \param coord_sys 
         * \throws PCLException if root file extension does not match \ref OutofcoreOctreeBaseNode::node_index_extension
         */
        OutofcoreOctreeBase (const Eigen::Vector3d& min, const Eigen::Vector3d& max, const double node_dim_meters, const boost::filesystem::path &root_node_name, const std::string &coord_sys);

        /** \brief Create a new tree; will not overwrite existing tree of same name
         *
         * Create a new tree rootname with specified bounding box; will not overwrite an existing tree
         *
         * \param max_depth Specifies a fixed number of LODs to generate, which is the depth of the tree
         * \param min Bounding box min
         * \param max Bounding box max
         * \param rootname must end in ".oct_idx" 
         * \param coord_sys
         * \throws PCLException if the parent directory has existing children (detects an existing tree)
         * \throws PCLException if file extension is not ".oct_idx"
         */
        OutofcoreOctreeBase (const boost::uint64_t max_depth, const Eigen::Vector3d &min, const Eigen::Vector3d &max, const boost::filesystem::path &rootname, const std::string &coord_sys);

        virtual
        ~OutofcoreOctreeBase ();

        // Point/Region INSERTION methods
        // --------------------------------------------------------------------------------
        /** \brief Recursively add points to the tree 
         *  \note shared read_write_mutex lock occurs
         */
        boost::uint64_t
        addDataToLeaf (const AlignedPointTVector &p);

        /** \brief Copies the points from the point_cloud falling within the bounding box of the octree to the
         *   out-of-core octree; this is an interface to addDataToLeaf and can be used multiple times.
         *  \param point_cloud Pointer to the point cloud data to copy to the outofcore octree; Assumes templated
         *   PointT matches for each.
         *  \return Number of points successfully copied from the point cloud to the octree.
         */
        boost::uint64_t
        addPointCloud (PointCloudConstPtr point_cloud);

        /** \brief Recursively copies points from input_cloud into the leaf nodes of the out-of-core octree, and stores them to disk.
         *
         * \param[in] input_cloud The cloud of points to be inserted into the out-of-core octree. Note if multiple PointCloud2 objects are added to the tree, this assumes that they all have exactly the same fields.
         * \param[in] skip_bb_check (default=false) whether to skip the bounding box check on insertion. Note the bounding box check is never skipped in the current implementation.
         * \return Number of poitns successfully copied from the point cloud to the octree
         */
        boost::uint64_t
        addPointCloud (sensor_msgs::PointCloud2::Ptr &input_cloud, const bool skip_bb_check);

        /** \brief Recursively add points to the tree. 
         *
         * Recursively add points to the tree. 1/8 of the remaining
         * points at each LOD are stored at each internal node of the
         * octree until either (a) runs out of points, in which case
         * the leaf is not at the \ref max_depth_ of the tree, or (b)
         * a larger set of points falls in the leaf at \ref
         * max_depth_. Note unlike the old implementation, multiple
         * copies of the same point will \b not be added at multiple
         * LODs as it walks the tree. Once the point is added to the
         * octree, it is no longer propagated further down the tree.
         *
         *\param[in] input_cloud The input cloud of points which will
         * be copied into the sorted nodes of the out-of-core octree
         * \return The total number of points added to the out-of-core
         * octree.
         */
        boost::uint64_t
        addPointCloud_and_genLOD (sensor_msgs::PointCloud2::Ptr &input_cloud);

        boost::uint64_t
        addPointCloud (sensor_msgs::PointCloud2::Ptr &input_cloud);
        
        boost::uint64_t
        addPointCloud (sensor_msgs::PointCloud2::Ptr &input_cloud, pcl::Filter<sensor_msgs::PointCloud2> &lod_filter);
        
        boost::uint64_t
        addPointCloud_and_genLOD (PointCloudConstPtr point_cloud);

        /** \brief Recursively add points to the tree subsampling LODs on the way.
         *
         * shared read_write_mutex lock occurs
         */
        boost::uint64_t
        addDataToLeaf_and_genLOD (AlignedPointTVector &p);

        // Frustrum/Box/Region REQUESTS/QUERIES: DB Accessors
        // -----------------------------------------------------------------------

        //--------------------------------------------------------------------------------
        //templated PointT methods
        //--------------------------------------------------------------------------------

        /** \brief Get a list of file paths at query_depth that intersect with your bounding box specified by \ref min and \ref max. When querying with this method, you may be stuck with extra data (some outside of your query bounds) that reside in the files.
         *
         * \param[in] min The minimum corner of the bounding box
         * \param[in] max The maximum corner of the bounding box
         * \param[in] query_depth 0 is root, (this->depth) is full
         * \param[out] bin_name List of paths to point data files (PCD currently) which satisfy the query
         */
        void
        queryBBIntersects (const Eigen::Vector3d &min, const Eigen::Vector3d &max, const boost::uint32_t query_depth, std::list<std::string> &bin_name) const;

        /** \brief Get Points in BB, only points inside BB. The query
         * processes the data at each node, filtering points that fall
         * out of the query bounds, and returns a single, concatenated
         * point cloud.
         *
         * \param[in] min The minimum corner of the bounding box for querying
         * \param[in] max The maximum corner of the bounding box for querying
         * \param[in] query_depth The depth from which point data will be taken
         *   \note If the LODs of the tree have not been built, you must specify the \ref max_depth_ in order to retrieve any data
         * \param[out] dst The destination vector of points
         */
        void
        queryBBIncludes (const Eigen::Vector3d &min, const Eigen::Vector3d &max, const size_t query_depth, AlignedPointTVector &dst) const;

        /** \brief Query all points falling within the input bounding box at \ref query_depth and return a PointCloud2 object in \ref dst_blob.
         *
         * \param[in] min The minimum corner of the input bounding box.
         * \param[in] max The maximum corner of the input bounding box.
         * \param[in] query_depth The query depth at which to search for points; only points at this depth are returned
         * \param[out] dst_blob Storage location for the points satisfying the query.
         **/
        void
        queryBBIncludes (const Eigen::Vector3d &min, const Eigen::Vector3d &max, const int query_depth, const sensor_msgs::PointCloud2::Ptr &dst_blob) const;
        
        /** \brief Returns a random subsample of points within the given bounding box at \ref query_depth.
         *
         * \param[in] min The minimum corner of the boudning box to query.
         * \param[out] max The maximum corner of the bounding box to query.
         * \param[in] query_depth The depth in the tree at which to look for the points. Only returns points within the given bounding box at the specified \ref query_depth.
         * \param[out] dst The destination in which to return the points.
         * 
         */
        void
        queryBBIncludes_subsample (const Eigen::Vector3d &min, const Eigen::Vector3d &max, size_t query_depth, const double percent, AlignedPointTVector &dst) const;

        //--------------------------------------------------------------------------------
        //PointCloud2 methods
        //--------------------------------------------------------------------------------

        /** \brief Query all points falling within the input bounding box at \ref query_depth and return a PointCloud2 object in \ref dst_blob. If the optional argument for filter is given, points are processed by that filter before returning.
         *  \param[in] min The minimum corner of the input bounding box.
         *  \param[in] max The maximum corner of the input bounding box.
         *  \param[in] query_depth The depth of tree at which to query; only points at this depth are returned
         *  \param[out] dst_blob The destination in which points within the bounding box are stored.
         *  \param[in] filter Optional filter for the points before returning. The most common uses of this would be to subsample, downsample, or upsample data.
         */
        virtual void
        queryBoundingBox (const Eigen::Vector3d &min, const Eigen::Vector3d &max, const int query_depth, const sensor_msgs::PointCloud2::Ptr &dst_blob, pcl::Filter<sensor_msgs::PointCloud2> &filter = 0)
        {
          PCL_THROW_EXCEPTION (PCLException, "Not implemented\n");
        }
        
        /** \brief Returns list of pcd files from nodes whose bounding boxes intersect with the input bounding box.
         * \param[in] min The minimum corner of the input bounding box.
         * \param[in] max The maximum corner of the input bounding box.
         * \param[out] filenames The list of paths to the PCD files which can be loaded and processed.
         */
        virtual void
        queryBoundingBox (const Eigen::Vector3d &min, const Eigen::Vector3d &max, const int query_depth, std::list<std::string> &filenames) 
        {
          PCL_THROW_EXCEPTION (PCLException, "Not implemented\n");
        }

        // Parameterization: getters and setters
        // --------------------------------------------------------------------------------

        /** \brief Get the overall bounding box of the outofcore
         *  octree; this is the same as the bounding box of the \ref root_node_ node */
        bool
        getBoundingBox (Eigen::Vector3d &min, Eigen::Vector3d &max) const;

        /** \brief Get number of points at specified LOD 
         * \param[in] depth the level of detail at which we want the number of points (0 is root, 1, 2,...)
         * \return number of points in the tree at \b depth
         */
        inline boost::uint64_t
        getNumPointsAtDepth (const boost::uint64_t& depth_index) const
        {
          return (metadata_->getLODPoints (depth_index));
        }

        /** \brief Queries the number of points in a bounding box 
         * 
         *  \param[in] min The minimum corner of the input bounding box
         *  \param[out] max The maximum corner of the input bounding box
         *  \param[in] query_depth The depth of the nodes to restrict the search to (only this depth is searched)
         *  \param[in] load_from_disk (default true) Whether to load PCD files to count exactly the number of points within the bounding box; setting this to false will return an upper bound by just reading the number of points from the PCD header, even if there may be some points in that node do not fall within the query bounding box.
         *  \return Number of points in the bounding box at depth \b query_depth
         **/
        boost::uint64_t
        queryBoundingBoxNumPoints (const Eigen::Vector3d& min, const Eigen::Vector3d& max, const int query_depth, bool load_from_disk = true);
        

        /** \brief Get number of points at each LOD 
         * \return std::vector of number of points in each LOD indexed by each level of depth, 0 to \ref max_depth_
         */
        inline const std::vector<boost::uint64_t>&
        getNumPointsVector () const
        {
          return (metadata_->getLODPoints ());
        }

        /** \brief Get number of LODs, which is the height of the tree
         */
        inline boost::uint64_t
        getDepth () const
        {
          return (metadata_->getDepth ());
        }

        inline boost::uint64_t
        getTreeDepth () const
        {
          return (this->getDepth ());
        }

        /** \brief Computes the expected voxel dimensions at the leaves (at \ref max_depth_)
         */
        bool
        getBinDimension (double &x, double &y) const;

        /** \brief gets the side length of an (assumed) perfect cubic voxel.
         *  \note If the initial bounding box specified in constructing the octree is not square, then this method does not return a sensible value 
         *  \return the side length of the cubic voxel size at the specified depth
         */
        double
        getVoxelSideLength (const boost::uint64_t depth) const;

        /** \brief Gets the smallest (assumed) cubic voxel side lengths. The smallest voxels are located at the max depth of the tree.
         * \return The side length of a the cubic voxel located at \ref max_depth_
         */
        double
        getVoxelSideLength () const
        {
          return (this->getVoxelSideLength (metadata_->getDepth ()));
        }

        /** \brief Get coord system tag in the metadata 
         *  
         *  \note This was part of the original Urban Robotics
         *  implementation, and can be stored in the metadata of the
         *  tree. If you have additional payload, we recommend 
         *  using a custom field of the PointCloud2 input upon
         *  construction of the tree.
         */
        const std::string&
        getCoordSystem ()
        {
          return (metadata_->getCoordinateSystem ());
        }

        // Mutators
        // -----------------------------------------------------------------------

        /** \brief Generate LODs for the tree 
         *  \note This is not implemented for PointCloud2 yet
         *  \throws PCLException, not implemented
         */
        void
        buildLOD ();

        /** \brief Prints size of BBox to stdout
         */ 
        void
        printBoundingBox (const size_t query_depth) const;

        /** \brief Prints the coordinates of the bounding box of the node to stdout */
        void
        printBoundingBox (OutofcoreNodeType& node) const;

        /** \brief Prints size of BBoxes to stdout
         */
        inline void
        printBoundingBox() const
        {
          this->printBoundingBox (metadata_->getDepth ());
        }

        /** \brief Returns the voxel centers of all existing voxels at \ref query_depth
            \param[in] query_depth: the depth of the tree at which to retrieve occupied/existing voxels
            \param[out] vector of PointXYZ voxel centers for nodes that exist at that depth
        */
        void
        getOccupiedVoxelCenters(AlignedPointTVector &voxel_centers, size_t query_depth) const;

        /** \brief Returns the voxel centers of all existing voxels at \ref query_depth
            \param[in] query_depth: the depth of the tree at which to retrieve occupied/existing voxels
            \param[out] vector of PointXYZ voxel centers for nodes that exist at that depth
        */
        void
        getOccupiedVoxelCenters(std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > &voxel_centers, size_t query_depth) const;

        /** \brief Gets the voxel centers of all occupied/existing leaves of the tree */
        void
        getOccupiedVoxelCenters(AlignedPointTVector &voxel_centers) const
        {
          getOccupiedVoxelCenters(voxel_centers, metadata_->getDepth ());
        }

        /** \brief Returns the voxel centers of all occupied/existing leaves of the tree 
         *  \param[out] voxel_centers std::vector of the centers of all occupied leaves of the octree
         */
        void
        getOccupiedVoxelCenters(std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > &voxel_centers) const
        {
          getOccupiedVoxelCenters(voxel_centers, metadata_->getDepth ());
        }

        // Serializers
        // -----------------------------------------------------------------------

        /** \brief Save the index files for each node.  You do not need to call this
         * explicitly
         * \todo does saveIdx () need to be  public?
         */
        void
        saveIdx ();

        /** \brief Save each .bin file as an XYZ file */
        void
        convertToXYZ ();

        /** \brief Write a python script using the vpython module containing all
         * the bounding boxes */
        void
        writeVPythonVisual (const boost::filesystem::path filename);

        // (note from UR) The following are DEPRECATED since I found that writeback caches did not
        // scale well, and they are currently disabled in the backend

        /** \brief DEPRECATED - Flush all nodes' cache 
         *  \deprecated this was moved to the octree_node class
         */
        void
        flushToDisk ();

        /** \brief DEPRECATED - Flush all non leaf nodes' cache 
         *  \deprecated
         */
        void
        flushToDiskLazy ();

        /** \brief DEPRECATED - Flush empty nodes only 
         *  \deprecated
         */
        void
        DeAllocEmptyNodeCache ();

        OutofcoreOctreeBaseNode<ContainerT, PointT>*
        getBranchChildPtr (const BranchNode& branch_arg, unsigned char childIdx_arg) const;
          
      protected:
        void
        init (const boost::uint64_t& depth, const Eigen::Vector3d& min, const Eigen::Vector3d& max, const boost::filesystem::path& root_name, const std::string& coord_sys);

        OutofcoreOctreeBase (OutofcoreOctreeBase &rval);
        OutofcoreOctreeBase (const OutofcoreOctreeBase &rval);

        OutofcoreOctreeBase&
        operator= (OutofcoreOctreeBase &rval);

        OutofcoreOctreeBase&
        operator= (const OutofcoreOctreeBase &rval);

        inline OutofcoreOctreeBaseNode<ContainerT, PointT>*
        getRootNode ()
        {
          return (this->root_node_);
        }

        /** \brief flush empty nodes only */
        void
        DeAllocEmptyNodeCache (OutofcoreOctreeBaseNode<ContainerT, PointT>* current);

        /** \brief Write octree definition ".octree" (defined by octree_extension_) to disk */
        void
        saveToFile ();

        /** \brief recursive portion of lod builder
         * \todo Rewrite and parameterize*/
        void
        buildLODRecursive (OutofcoreOctreeBaseNode<ContainerT, PointT>** current_branch, const int current_dims);

        /** \brief Increment current depths (LOD for branch nodes) point count; called by addDataAtMaxDepth in OutofcoreOctreeBaseNode
         */
        inline void
        incrementPointsInLOD (boost::uint64_t depth, boost::uint64_t inc);

        /** \brief Auxiliary function to validate path_name extension is .octree
         *  
         *  \return 0 if bad; 1 if extension is .oct_idx
         */
        bool
        checkExtension (const boost::filesystem::path& path_name);

        /** \brief Pointer to the root node of the octree data structure */
        OutofcoreOctreeBaseNode<ContainerT, PointT>* root_node_;

        /** \brief shared mutex for controlling read/write access to disk */
        mutable boost::shared_mutex read_write_mutex_;

        boost::shared_ptr<OutofcoreOctreeBaseMetadata> metadata_;
        
        /** \brief the pre-set maximum depth of the tree */
        boost::uint64_t max_depth_;

        /** \brief defined as ".octree" to append to treepath files
         *  \note this might change
         */
        const static std::string TREE_EXTENSION_;
        const static int OUTOFCORE_VERSION_;

        /** \todo @b loadcount mystery constant 2e9 points at a time to subsample; should parameterize */
        const static uint64_t LOAD_COUNT_ = static_cast<uint64_t>(2e9);

      private:    

        /** \brief Auxiliary function to enlarge a bounding box to a cube. */
        void
        enlargeToCube (Eigen::Vector3d &bb_min, Eigen::Vector3d &bb_max);

        /** \brief Auxiliary function to compute the depth of the tree given the bounding box and the desired size of the leaf voxels */
        boost::uint64_t
        calculateDepth (const Eigen::Vector3d& min_bb, const Eigen::Vector3d& max_bb, const double leaf_resolution);
          
    };
  }
}

  
#endif // PCL_OUTOFCORE_OUTOFCOREOCTREEBASE_H_
