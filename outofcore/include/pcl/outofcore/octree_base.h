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

#include <pcl/common/io.h>

//outofcore classes
#include <pcl/outofcore/octree_base_node.h>
#include <pcl/outofcore/octree_disk_container.h>
#include <pcl/outofcore/octree_ram_container.h>

//outofcore iterators
#include <pcl/outofcore/outofcore_iterator_base.h>
#include <pcl/outofcore/outofcore_breadth_first_iterator.h>
#include <pcl/outofcore/outofcore_depth_first_iterator.h>
#include <pcl/outofcore/impl/outofcore_breadth_first_iterator.hpp>
#include <pcl/outofcore/impl/outofcore_depth_first_iterator.hpp>

//outofcore metadata
#include <pcl/outofcore/metadata.h>
#include <pcl/outofcore/outofcore_base_data.h>

#include <pcl/filters/filter.h>
#include <pcl/filters/random_sample.h>

#include <pcl/PCLPointCloud2.h>

#include <shared_mutex>

namespace pcl
{
  namespace outofcore
  {
    struct OutofcoreParams
    {
      std::string node_index_basename_;
      std::string node_container_basename_;
      std::string node_index_extension_;
      std::string node_container_extension_;
      double sample_percent;
    };
    
    /** \class OutofcoreOctreeBase 
     *  \brief This code defines the octree used for point storage at Urban Robotics. 
     * 
     *  \note Code was adapted from the Urban Robotics out of core octree implementation. 
     *  Contact Jacob Schloss <jacob.schloss@urbanrobotics.net> with any questions. 
     *  http://www.urbanrobotics.net/. This code was integrated for the Urban Robotics 
     *  Code Sprint (URCS) by Stephen Fox (foxstephend@gmail.com). Additional development notes can be found at
     *  http://www.pointclouds.org/blog/urcs/.
     *
     *  The primary purpose of this class is an interface to the
     *  recursive traversal (recursion handled by \ref pcl::outofcore::OutofcoreOctreeBaseNode) of the
     *  in-memory/top-level octree structure. The metadata in each node
     *  can be loaded entirely into main memory, from which the tree can be traversed
     *  recursively in this state. This class provides an the interface
     *  for: 
     *               -# Point/Region insertion methods 
     *               -# Frustrum/box/region queries
     *               -# Parameterization of resolution, container type, etc...
     *
     *  For lower-level node access, there is a Depth-First iterator
     *  for traversing the trees with direct access to the nodes. This
     *  can be used for implementing other algorithms, and other
     *  iterators can be written in a similar fashion.
     *
     *  The format of the octree is stored on disk in a hierarchical
     *  octree structure, where .oct_idx are the JSON-based node
     *  metadata files managed by \ref pcl::outofcore::OutofcoreOctreeNodeMetadata,
     *  and .octree is the JSON-based octree metadata file managed by
     *  \ref pcl::outofcore::OutofcoreOctreeBaseMetadata. Children of each node live
     *  in up to eight subdirectories named from 0 to 7, where a
     *  metadata and optionally a pcd file will exist. The PCD files
     *  are stored in compressed binary PCD format, containing all of
     *  the fields existing in the PCLPointCloud2 objects originally
     *  inserted into the out of core object.
     *  
     *  A brief outline of the out of core octree can be seen
     *  below. The files in [brackets] exist only when the LOD are
     *  built.
     *
     *  At this point in time, there is not support for multiple trees
     *  existing in a single directory hierarchy.
     *
     *  \verbatim
     tree_name/
          tree_name.oct_idx
          tree_name.octree
          [tree_name-uuid.pcd]
          0/
               tree_name.oct_idx
               [tree_name-uuid.pcd]
               0/
                  ...
               1/
                   ...
                     ...
                         0/
                             tree_name.oct_idx
                             tree_name.pcd
          1/
          ...
          7/
     \endverbatim
     *
     *  \ingroup outofcore
     *  \author Jacob Schloss (jacob.schloss@urbanrobotics.net)
     *  \author Stephen Fox, Urban Robotics Code Sprint (foxstephend@gmail.com)
     *
     */
    template<typename ContainerT = OutofcoreOctreeDiskContainer<pcl::PointXYZ>, typename PointT = pcl::PointXYZ>
    class OutofcoreOctreeBase
    {
      friend class OutofcoreOctreeBaseNode<ContainerT, PointT>;
      friend class pcl::outofcore::OutofcoreIteratorBase<PointT, ContainerT>;

      public:

        // public typedefs
        using octree_disk = OutofcoreOctreeBase<OutofcoreOctreeDiskContainer<PointT>, PointT >;
        using octree_disk_node = OutofcoreOctreeBaseNode<OutofcoreOctreeDiskContainer<PointT>, PointT >;

        using octree_ram = OutofcoreOctreeBase<OutofcoreOctreeRamContainer<PointT>, PointT>;
        using octree_ram_node = OutofcoreOctreeBaseNode<OutofcoreOctreeRamContainer<PointT>, PointT>;

        using OutofcoreNodeType = OutofcoreOctreeBaseNode<ContainerT, PointT>;

        using BranchNode = OutofcoreOctreeBaseNode<ContainerT, PointT>;
        using LeafNode = OutofcoreOctreeBaseNode<ContainerT, PointT>;

        using Iterator = OutofcoreDepthFirstIterator<PointT, ContainerT>;
        using ConstIterator = const OutofcoreDepthFirstIterator<PointT, ContainerT>;

        using BreadthFirstIterator = OutofcoreBreadthFirstIterator<PointT, ContainerT>;
        using BreadthFirstConstIterator = const OutofcoreBreadthFirstIterator<PointT, ContainerT>;

        using DepthFirstIterator = OutofcoreDepthFirstIterator<PointT, ContainerT>;
        using DepthFirstConstIterator = const OutofcoreDepthFirstIterator<PointT, ContainerT>;

        using Ptr = shared_ptr<OutofcoreOctreeBase<ContainerT, PointT> >;
        using ConstPtr = shared_ptr<const OutofcoreOctreeBase<ContainerT, PointT> >;

        using PointCloud = pcl::PointCloud<PointT>;

        using IndicesPtr = shared_ptr<pcl::Indices>;
        using IndicesConstPtr = shared_ptr<const pcl::Indices>;

        using PointCloudPtr = typename PointCloud::Ptr;
        using PointCloudConstPtr = typename PointCloud::ConstPtr;

        using AlignedPointTVector = std::vector<PointT, Eigen::aligned_allocator<PointT> >;

        // Constructors
        // -----------------------------------------------------------------------

        /** \brief Load an existing tree
         *
         * If load_all is set, the BB and point count for every node is loaded,
         * otherwise only the root node is actually created, and the rest will be
         * generated on insertion or query.
         *
         * \param root_node_name Path to the top-level tree/tree.oct_idx metadata file
         * \param load_all Load entire tree metadata (does not load any points from disk)
         * \throws PCLException for bad extension (root node metadata must be .oct_idx extension)
         */
        OutofcoreOctreeBase (const boost::filesystem::path &root_node_name, const bool load_all);

        /** \brief Create a new tree
         *
         * Create a new tree rootname with specified bounding box; will remove and overwrite existing tree with the same name
         *
         * Computes the depth of the tree based on desired leaf , then calls the other constructor.
         *
         * \param min Bounding box min
         * \param max Bounding box max
         * \param resolution_arg Node dimension in meters (assuming your point data is in meters)
         * \param root_node_name must end in ".oct_idx" 
         * \param coord_sys Coordinate system which is stored in the JSON metadata
         * \throws PCLException if root file extension does not match \ref pcl::outofcore::OutofcoreOctreeBaseNode::node_index_extension
         */
        OutofcoreOctreeBase (const Eigen::Vector3d& min, const Eigen::Vector3d& max, const double resolution_arg, const boost::filesystem::path &root_node_name, const std::string &coord_sys);

        /** \brief Create a new tree; will not overwrite existing tree of same name
         *
         * Create a new tree rootname with specified bounding box; will not overwrite an existing tree
         *
         * \param max_depth Specifies a fixed number of LODs to generate, which is the depth of the tree
         * \param min Bounding box min
         * \param max Bounding box max
         * \note Bounding box of the tree must be set before inserting any points. The tree \b cannot be resized at this time.
         * \param root_node_name must end in ".oct_idx" 
         * \param coord_sys Coordinate system which is stored in the JSON metadata
         * \throws PCLException if the parent directory has existing children (detects an existing tree)
         * \throws PCLException if file extension is not ".oct_idx"
         */
        OutofcoreOctreeBase (const std::uint64_t max_depth, const Eigen::Vector3d &min, const Eigen::Vector3d &max, const boost::filesystem::path &root_node_name, const std::string &coord_sys);

        virtual
        ~OutofcoreOctreeBase ();

        // Point/Region INSERTION methods
        // --------------------------------------------------------------------------------
        /** \brief Recursively add points to the tree 
         *  \note shared read_write_mutex lock occurs
         */
        std::uint64_t
        addDataToLeaf (const AlignedPointTVector &p);

        /** \brief Copies the points from the point_cloud falling within the bounding box of the octree to the
         *   out-of-core octree; this is an interface to addDataToLeaf and can be used multiple times.
         *  \param point_cloud Pointer to the point cloud data to copy to the outofcore octree; Assumes templated
         *   PointT matches for each.
         *  \return Number of points successfully copied from the point cloud to the octree.
         */
        std::uint64_t
        addPointCloud (PointCloudConstPtr point_cloud);

        /** \brief Recursively copies points from input_cloud into the leaf nodes of the out-of-core octree, and stores them to disk.
         *
         * \param[in] input_cloud The cloud of points to be inserted into the out-of-core octree. Note if multiple PCLPointCloud2 objects are added to the tree, this assumes that they all have exactly the same fields.
         * \param[in] skip_bb_check (default=false) whether to skip the bounding box check on insertion. Note the bounding box check is never skipped in the current implementation.
         * \return Number of points successfully copied from the point cloud to the octree
         */
        std::uint64_t
        addPointCloud (pcl::PCLPointCloud2::Ptr &input_cloud, const bool skip_bb_check = false);

        /** \brief Recursively add points to the tree. 
         *
         * Recursively add points to the tree. 1/8 of the remaining
         * points at each LOD are stored at each internal node of the
         * octree until either (a) runs out of points, in which case
         * the leaf is not at the maximum depth of the tree, or (b)
         * a larger set of points falls in the leaf at the maximum depth.
         * Note unlike the old implementation, multiple
         * copies of the same point will \b not be added at multiple
         * LODs as it walks the tree. Once the point is added to the
         * octree, it is no longer propagated further down the tree.
         *
         *\param[in] input_cloud The input cloud of points which will
         * be copied into the sorted nodes of the out-of-core octree
         * \return The total number of points added to the out-of-core
         * octree.
         */
        std::uint64_t
        addPointCloud_and_genLOD (pcl::PCLPointCloud2::Ptr &input_cloud);

        std::uint64_t
        addPointCloud (pcl::PCLPointCloud2::Ptr &input_cloud);
        
        std::uint64_t
        addPointCloud_and_genLOD (PointCloudConstPtr point_cloud);

        /** \brief Recursively add points to the tree subsampling LODs on the way.
         *
         * shared read_write_mutex lock occurs
         */
        std::uint64_t
        addDataToLeaf_and_genLOD (AlignedPointTVector &p);

        // Frustrum/Box/Region REQUESTS/QUERIES: DB Accessors
        // -----------------------------------------------------------------------
        void
        queryFrustum (const double *planes, std::list<std::string>& file_names) const;

	      void
        queryFrustum (const double *planes, std::list<std::string>& file_names, const std::uint32_t query_depth) const;

	      void
        queryFrustum (const double *planes, const Eigen::Vector3d &eye, const Eigen::Matrix4d &view_projection_matrix,
                      std::list<std::string>& file_names, const std::uint32_t query_depth) const;
        
        //--------------------------------------------------------------------------------
        //templated PointT methods
        //--------------------------------------------------------------------------------

        /** \brief Get a list of file paths at query_depth that intersect with your bounding box specified by \c min and \c max.
         *  When querying with this method, you may be stuck with extra data (some outside of your query bounds) that reside in the files.
         *
         * \param[in] min The minimum corner of the bounding box
         * \param[in] max The maximum corner of the bounding box
         * \param[in] query_depth 0 is root, (this->depth) is full
         * \param[out] bin_name List of paths to point data files (PCD currently) which satisfy the query
         */
        void
        queryBBIntersects (const Eigen::Vector3d &min, const Eigen::Vector3d &max, const std::uint32_t query_depth, std::list<std::string> &bin_name) const;

        /** \brief Get Points in BB, only points inside BB. The query
         * processes the data at each node, filtering points that fall
         * out of the query bounds, and returns a single, concatenated
         * point cloud.
         *
         * \param[in] min The minimum corner of the bounding box for querying
         * \param[in] max The maximum corner of the bounding box for querying
         * \param[in] query_depth The depth from which point data will be taken
         *   \note If the LODs of the tree have not been built, you must specify the maximum depth in order to retrieve any data
         * \param[out] dst The destination vector of points
         */
        void
        queryBBIncludes (const Eigen::Vector3d &min, const Eigen::Vector3d &max, const std::uint64_t query_depth, AlignedPointTVector &dst) const;

        /** \brief Query all points falling within the input bounding box at \c query_depth and return a PCLPointCloud2 object in \c dst_blob.
         *
         * \param[in] min The minimum corner of the input bounding box.
         * \param[in] max The maximum corner of the input bounding box.
         * \param[in] query_depth The query depth at which to search for points; only points at this depth are returned
         * \param[out] dst_blob Storage location for the points satisfying the query.
         **/
        void
        queryBBIncludes (const Eigen::Vector3d &min, const Eigen::Vector3d &max, const std::uint64_t query_depth, const pcl::PCLPointCloud2::Ptr &dst_blob) const;
        
        /** \brief Returns a random subsample of points within the given bounding box at \c query_depth.
         *
         * \param[in] min The minimum corner of the boudning box to query.
         * \param[out] max The maximum corner of the bounding box to query.
         * \param[in] query_depth The depth in the tree at which to look for the points. Only returns points within the given bounding box at the specified \c query_depth.
         * \param percent
         * \param[out] dst The destination in which to return the points.
         * 
         */
        void
        queryBBIncludes_subsample (const Eigen::Vector3d &min, const Eigen::Vector3d &max, std::uint64_t query_depth, const double percent, AlignedPointTVector &dst) const;

        //--------------------------------------------------------------------------------
        //PCLPointCloud2 methods
        //--------------------------------------------------------------------------------

        /** \brief Query all points falling within the input bounding box at \c query_depth and return a PCLPointCloud2 object in \c dst_blob.
         *   If the optional argument for filter is given, points are processed by that filter before returning.
         *  \param[in] min The minimum corner of the input bounding box.
         *  \param[in] max The maximum corner of the input bounding box.
         *  \param[in] query_depth The depth of tree at which to query; only points at this depth are returned
         *  \param[out] dst_blob The destination in which points within the bounding box are stored.
         *  \param[in] percent optional sampling percentage which is applied after each time data are read from disk
         */
        virtual void
        queryBoundingBox (const Eigen::Vector3d &min, const Eigen::Vector3d &max, const int query_depth, const pcl::PCLPointCloud2::Ptr &dst_blob, double percent = 1.0);
        
        /** \brief Returns list of pcd files from nodes whose bounding boxes intersect with the input bounding box.
         * \param[in] min The minimum corner of the input bounding box.
         * \param[in] max The maximum corner of the input bounding box.
         * \param query_depth
         * \param[out] filenames The list of paths to the PCD files which can be loaded and processed.
         */
        inline virtual void
        queryBoundingBox (const Eigen::Vector3d &min, const Eigen::Vector3d &max, const int query_depth, std::list<std::string> &filenames) const
        {
          std::shared_lock < std::shared_timed_mutex > lock (read_write_mutex_);
          filenames.clear ();
          this->root_node_->queryBBIntersects (min, max, query_depth, filenames);
        }

        // Parameterization: getters and setters
        // --------------------------------------------------------------------------------

        /** \brief Get the overall bounding box of the outofcore
         *  octree; this is the same as the bounding box of the \c root_node_ node
         *  \param min
         *  \param max
         */
        bool
        getBoundingBox (Eigen::Vector3d &min, Eigen::Vector3d &max) const;

        /** \brief Get number of points at specified LOD 
         * \param[in] depth_index the level of detail at which we want the number of points (0 is root, 1, 2,...)
         * \return number of points in the tree at \b depth
         */
        inline std::uint64_t
        getNumPointsAtDepth (const std::uint64_t& depth_index) const
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
        std::uint64_t
        queryBoundingBoxNumPoints (const Eigen::Vector3d& min, const Eigen::Vector3d& max, const int query_depth, bool load_from_disk = true);
        

        /** \brief Get number of points at each LOD 
         * \return vector of number of points in each LOD indexed by each level of depth, 0 to the depth of the tree.
         */
        inline const std::vector<std::uint64_t>&
        getNumPointsVector () const
        {
          return (metadata_->getLODPoints ());
        }

        /** \brief Get number of LODs, which is the height of the tree
         */
        inline std::uint64_t
        getDepth () const
        {
          return (metadata_->getDepth ());
        }

        inline std::uint64_t
        getTreeDepth () const
        {
          return (this->getDepth ());
        }

        /** \brief Computes the expected voxel dimensions at the leaves 
         */
        bool
        getBinDimension (double &x, double &y) const;

        /** \brief gets the side length of an (assumed) perfect cubic voxel.
         *  \note If the initial bounding box specified in constructing the octree is not square, then this method does not return a sensible value 
         *  \return the side length of the cubic voxel size at the specified depth
         */
        double
        getVoxelSideLength (const std::uint64_t& depth) const;

        /** \brief Gets the smallest (assumed) cubic voxel side lengths. The smallest voxels are located at the max depth of the tree.
         * \return The side length of a the cubic voxel located at the leaves
         */
        double
        getVoxelSideLength () const
        {
          return (this->getVoxelSideLength (metadata_->getDepth ()));
        }

        /** \brief Get coordinate system tag from the JSON metadata file
         */
        const std::string&
        getCoordSystem () const
        {
          return (metadata_->getCoordinateSystem ());
        }

        // Mutators
        // -----------------------------------------------------------------------

        /** \brief Generate multi-resolution LODs for the tree, which are a uniform random sampling all child leafs below the node.
         */
        void
        buildLOD ();

        /** \brief Prints size of BBox to stdout
         */ 
        void
        printBoundingBox (const std::size_t query_depth) const;

        /** \brief Prints the coordinates of the bounding box of the node to stdout */
        void
        printBoundingBox (OutofcoreNodeType& node) const;

        /** \brief Prints size of the bounding boxes to stdou
         */
        inline void
        printBoundingBox() const
        {
          this->printBoundingBox (metadata_->getDepth ());
        }

        /** \brief Returns the voxel centers of all existing voxels at \c query_depth
            \param[out] voxel_centers Vector of PointXYZ voxel centers for nodes that exist at that depth
            \param[in] query_depth the depth of the tree at which to retrieve occupied/existing voxels
        */
        void
        getOccupiedVoxelCenters(AlignedPointTVector &voxel_centers, std::size_t query_depth) const;

        /** \brief Returns the voxel centers of all existing voxels at \c query_depth
            \param[out] voxel_centers Vector of PointXYZ voxel centers for nodes that exist at that depth
            \param[in] query_depth the depth of the tree at which to retrieve occupied/existing voxels
        */
        void
        getOccupiedVoxelCenters(std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > &voxel_centers, std::size_t query_depth) const;

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

        /** \brief Save each .bin file as an XYZ file */
        void
        convertToXYZ ();

        /** \brief Write a python script using the vpython module containing all
         * the bounding boxes */
        void
        writeVPythonVisual (const boost::filesystem::path& filename);

        OutofcoreNodeType*
        getBranchChildPtr (const BranchNode& branch_arg, unsigned char childIdx_arg) const;

        pcl::Filter<pcl::PCLPointCloud2>::Ptr
        getLODFilter ();

        const pcl::Filter<pcl::PCLPointCloud2>::ConstPtr
        getLODFilter () const;

        /** \brief Sets the filter to use when building the levels of depth. Recommended filters are pcl::RandomSample<pcl::PCLPointCloud2> or pcl::VoxelGrid */
        void
        setLODFilter (const pcl::Filter<pcl::PCLPointCloud2>::Ptr& filter_arg);

        /** \brief Returns the sample_percent_ used when constructing the LOD. */
        double 
        getSamplePercent () const
        {
          return (sample_percent_);
        }
        
        /** \brief Sets the sampling percent for constructing LODs. Each LOD gets sample_percent^d points. 
         * \param[in] sample_percent_arg Percentage between 0 and 1. */
        inline void 
        setSamplePercent (const double sample_percent_arg)
        {
          this->sample_percent_ = std::fabs (sample_percent_arg) > 1.0 ? 1.0 : std::fabs (sample_percent_arg);
        }
	
      protected:
        void
        init (const std::uint64_t& depth, const Eigen::Vector3d& min, const Eigen::Vector3d& max, const boost::filesystem::path& root_name, const std::string& coord_sys);

        OutofcoreOctreeBase (OutofcoreOctreeBase &rval);

        OutofcoreOctreeBase (const OutofcoreOctreeBase &rval);

        OutofcoreOctreeBase&
        operator= (OutofcoreOctreeBase &rval);

        OutofcoreOctreeBase&
        operator= (const OutofcoreOctreeBase &rval);

        inline OutofcoreNodeType*
        getRootNode ()
        {
          return (this->root_node_);
        }

        /** \brief flush empty nodes only */
        void
        DeAllocEmptyNodeCache (OutofcoreNodeType* current);

        /** \brief Write octree definition ".octree" (defined by octree_extension_) to disk */
        void
        saveToFile ();

        /** \brief recursive portion of lod builder */
        void
        buildLODRecursive (const std::vector<BranchNode*>& current_branch);

        /** \brief Increment current depths (LOD for branch nodes) point count; called by addDataAtMaxDepth in OutofcoreOctreeBaseNode
         */
        inline void
        incrementPointsInLOD (std::uint64_t depth, std::uint64_t inc);

        /** \brief Auxiliary function to validate path_name extension is .octree
         *  
         *  \return 0 if bad; 1 if extension is .oct_idx
         */
        bool
        checkExtension (const boost::filesystem::path& path_name);

        /** \brief Flush all nodes' cache */
        void
        flushToDisk ();

        /** \brief Flush all non leaf nodes' cache */
        void
        flushToDiskLazy ();

        /** \brief Flush empty nodes only */
        void
        DeAllocEmptyNodeCache ();

        /** \brief Pointer to the root node of the octree data structure */
        OutofcoreNodeType* root_node_;

        /** \brief shared mutex for controlling read/write access to disk */
        mutable std::shared_timed_mutex read_write_mutex_;

        OutofcoreOctreeBaseMetadata::Ptr metadata_;
        
        /** \brief defined as ".octree" to append to treepath files
         *  \note this might change
         */
        const static std::string TREE_EXTENSION_;
        const static int OUTOFCORE_VERSION_;

        const static std::uint64_t LOAD_COUNT_ = static_cast<std::uint64_t>(2e9);

      private:    

        /** \brief Auxiliary function to enlarge a bounding box to a cube. */
        void
        enlargeToCube (Eigen::Vector3d &bb_min, Eigen::Vector3d &bb_max);

        /** \brief Auxiliary function to compute the depth of the tree given the bounding box and the desired size of the leaf voxels */
        std::uint64_t
        calculateDepth (const Eigen::Vector3d& min_bb, const Eigen::Vector3d& max_bb, const double leaf_resolution);

        double sample_percent_;

        pcl::RandomSample<pcl::PCLPointCloud2>::Ptr lod_filter_ptr_;
        
    };
  }
}
