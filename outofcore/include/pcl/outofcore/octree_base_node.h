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

#ifndef PCL_OUTOFCORE_OCTREE_BASE_NODE_H_
#define PCL_OUTOFCORE_OCTREE_BASE_NODE_H_

// Boost
#include <boost/filesystem.hpp>
#include <boost/thread.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int.hpp>
#include <boost/random/bernoulli_distribution.hpp>

// PCL (Urban Robotics)
#include <pcl/common/io.h>

#include <pcl/outofcore/octree_base.h>
#include <pcl/outofcore/octree_disk_container.h>

#include <sensor_msgs/PointCloud2.h>

namespace pcl
{
  namespace outofcore
  {
// Forward Declarations
    template<typename ContainerT, typename PointT>
    class octree_base_node;

    template<typename ContainerT, typename PointT>
    class octree_base;

    /** \todo Move non-class functions to an octree accessor class */

    /** \brief Non-class function which creates a single child leaf; used with \ref queryBBIntersects_noload to avoid loading the data from disk */
    template<typename ContainerT, typename PointT> octree_base_node<ContainerT, PointT>*
    makenode_norec (const boost::filesystem::path &path, octree_base_node<ContainerT, PointT>* super);

    /** \brief Non-class method which performs a bounding box query without loading any of the point cloud data from disk */
    template<typename ContainerT, typename PointT> void
    queryBBIntersects_noload (const boost::filesystem::path &root_node, const Eigen::Vector3d &min, const Eigen::Vector3d &max, const boost::uint32_t query_depth, std::list<std::string> &bin_name);

    /** \brief Non-class method overload */
    template<typename ContainerT, typename PointT> void
    queryBBIntersects_noload (octree_base_node<ContainerT, PointT>* current, const Eigen::Vector3d&, const Eigen::Vector3d &max, const boost::uint32_t query_depth, std::list<std::string> &bin_name);

    /** \class octree_base_node 
     *
     *  \note Code was adapted from the Urban Robotics out of core octree implementation. 
     *  Contact Jacob Schloss <jacob.schloss@urbanrobotics.net> with any questions. 
     *  http://www.urbanrobotics.net/
     *
     *  \brief octree_base_node Class internally representing nodes of an
     *  outofcore octree, with accessors to its data via the \ref
     *  octree_disk_container class or \ref octree_ram_container class,
     *  whichever it is templated against.  
     * 
     *  \ingroup outofcore
     *  \author Jacob Schloss (jacob.schloss@urbanrobotics.net)
     *
     */
    template<typename ContainerT, typename PointT>
    class octree_base_node
    {
      friend class octree_base<ContainerT, PointT> ;
  
      friend octree_base_node<ContainerT, PointT>*
      makenode_norec<ContainerT, PointT> (const boost::filesystem::path &path, octree_base_node<ContainerT, PointT>* super);
  
      friend void
      queryBBIntersects_noload<ContainerT, PointT> (const boost::filesystem::path &rootnode, const Eigen::Vector3d &min, const Eigen::Vector3d &max, const boost::uint32_t query_depth, std::list<std::string> &bin_name);

      friend void
      queryBBIntersects_noload<ContainerT, PointT> (octree_base_node<ContainerT, PointT>* current, const Eigen::Vector3d &min, const Eigen::Vector3d &max, const boost::uint32_t query_depth, std::list<std::string> &bin_name);
  
      public:
        typedef octree_base<octree_disk_container < PointT > , PointT > octree_disk;
        typedef octree_base_node<octree_disk_container < PointT > , PointT > octree_disk_node;

        typedef std::vector<PointT, Eigen::aligned_allocator<PointT> > AlignedPointTVector;

        const static std::string node_index_basename;
        const static std::string node_container_basename;
        const static std::string node_index_extension;
        const static std::string node_container_extension;
        const static double sample_precent;

        /** \brief Empty constructor; sets pointers for children and for bounding boxes to 0
         */
        octree_base_node ();

        /** \brief Create root node and directory setting voxel size*/
        octree_base_node (const Eigen::Vector3d &bb_min, const Eigen::Vector3d &bb_max, const double node_dim_meters, octree_base<ContainerT, PointT> * const tree, const boost::filesystem::path &root_name);

        /** \brief Create root node and directory setting setting max depth*/
        octree_base_node (const int max_depth, const Eigen::Vector3d &bb_min, const Eigen::Vector3d &bb_max, octree_base<ContainerT, PointT> * const tree, const boost::filesystem::path &root_name);

        /** \brief Will recursively delete all children calling recFreeChildrein */
        ~octree_base_node ();

        //query
        /** \brief gets the minimum and maximum corner of the bounding box represented by this node
         * \param[out] minCoord returns the minimum corner of the bounding box indexed by 0-->X, 1-->Y, 2-->Z 
         * \param[out] maxCoord returns the maximum corner of the bounding box indexed by 0-->X, 1-->Y, 2-->Z 
         */
        inline void
        getBB (Eigen::Vector3d &minCoord, Eigen::Vector3d &maxCoord) const
        {
          minCoord = min_;
          maxCoord = max_;
        }
    

        //point extraction
        /** \brief Recursively add points that fall into the queried bounding box up to the \b query_depth 
         *
         *  \param[in] min_bb the minimum corner of the bounding box, indexed by X,Y,Z coordinates
         *  \param[in] max_bb the maximum corner of the bounding box, indexed by X,Y,Z coordinates
         *  \param[in] query_depth the maximum depth to query in the octree for points within the bounding box
         *  \param[out] dst destion of points returned by the queries
         */
        void
        queryBBIncludes (const Eigen::Vector3d &min_bb, const Eigen::Vector3d &max_bb, size_t query_depth, AlignedPointTVector &dst);

        /** \brief Recursively add points that fall into the queried bounding box up to the \b query_depth
         *
         *  \param[in] min_bb the minimum corner of the bounding box, indexed by X,Y,Z coordinates
         *  \param[in] max_bb the maximum corner of the bounding box, indexed by X,Y,Z coordinates
         *  \param[in] query_depth the maximum depth to query in the octree for points within the bounding box
         *  \param[out] dst_blob destion of points returned by the queries
         */
        void
        queryBBIncludes (const Eigen::Vector3d &min_bb, const Eigen::Vector3d &max_bb, size_t query_depth, const sensor_msgs::PointCloud2::Ptr &dst_blob);

        /** \brief Recursively add points that fall into the queried bounding box up to the \b query_depth 
         *
         *  \param[in] min_bb the minimum corner of the bounding box, indexed by X,Y,Z coordinates
         *  \param[in] max_bb the maximum corner of the bounding box, indexed by X,Y,Z coordinates
         *  \param[in] query_depth
         *  \param[out] v std::list of points returned by the query
         */
        void
        queryBBIncludes_subsample (const Eigen::Vector3d &min_bb, const Eigen::Vector3d &max_bb, int query_depth, const double percent, AlignedPointTVector &v);

        /** \brief Tests if the coordinate falls within the boundaries of the bounding box, inclusively
         */
        void
        queryBBIntersects (const Eigen::Vector3d &min_bb, const Eigen::Vector3d &max_bb, const boost::uint32_t query_depth, std::list<std::string> &file_names);

        /** \brief Write the voxel size to stdout at \ref query_depth 
         * \param[in] query_depth The depth at which to print the size of the voxel/bounding boxes
         */
        void
        printBBox (const size_t query_depth) const;

        /** \brief Tests whether the input bounding box intersects with the current node's bounding box 
         *  \param[in] min_bb The minimum corner of the input bounding box
         *  \param[in] min_bb The maximum corner of the input bounding box
         *  \return bool True if any portion of the bounding box intersects with this node's bounding box; false otherwise
         */
        inline bool
        intersectsWithBB (const Eigen::Vector3d &min_bb, const Eigen::Vector3d &max_bb) const;

        /** \brief Tests whether the input bounding box falls inclusively within this node's bounding box
         *  \param[in] min_bb The minimum corner of the input bounding box
         *  \param[in] max_bb The maximum corner of the input bounding box
         *  \return bool True if the input bounding box falls inclusively within the boundaries of this node's bounding box
         **/
        inline bool
        withinBB (const Eigen::Vector3d &min_bb, const Eigen::Vector3d &max_bb) const;

        /** \brief Tests whether \ref point falls within the input bounding box
         *  \param[in] min_bb The minimum corner of the input bounding box
         *  \param[in] max_bb The maximum corner of the input bounding box
         */
        bool
        pointWithinBB (const Eigen::Vector3d &min_bb, const Eigen::Vector3d &max_bb, const Eigen::Vector3d &point);

        /** \brief Tests whether \ref p falls within the input bounding box
         *  \param[in] min_bb The minimum corner of the input bounding box
         *  \param[in] max_bb The maximum corner of the input bounding box
         **/
        static inline bool
        pointWithinBB (const Eigen::Vector3d &min_bb, const Eigen::Vector3d &max_bb, const PointT &p);

        /** \brief Tests whether \ref x, \ref y, and \ref z fall within the input bounding box
         *  \param[in] min_bb The minimum corner of the input bounding box
         *  \param[in] max_bb The maximum corner of the input bounding box
         **/
        static inline bool
        pointWithinBB (const Eigen::Vector3d &min_bb, const Eigen::Vector3d &max_bb, const double x, const double y, const double z);

        /** \brief Tests if specified point is within bounds of current node's bounding box */
        inline bool
        pointWithinBB (const PointT &p) const;

        /** \brief add point to this node if we are a leaf, or find the leaf below us that is supposed to take the point 
         *  \param[in] p vector of points to add to the leaf
         *  \param[in] skipBBCheck whether to check if the point's coordinates fall within the bounding box
         */
        boost::uint64_t
        addDataToLeaf (const AlignedPointTVector &p, const bool skip_bb_check);

        boost::uint64_t
        addDataToLeaf (const std::vector<const PointT*> &p, const bool skip_bb_check);

        /** \brief Add a single PointCloud2 object into the octree.
         *
         * \param[in] input_cloud
         * \param[in] skip_bb_check (default = false)
         */
        boost::uint64_t
        addPointCloud (const sensor_msgs::PointCloud2::Ptr &input_cloud, const bool skip_bb_check);

        /** \brief Add a single PointCloud2 into the octree and build the subsampled LOD during construction */
        boost::uint64_t
        addPointCloud_and_genLOD (const sensor_msgs::PointCloud2::Ptr input_cloud); //, const bool skip_bb_check);
        
        /** \brief Recursively add points to the leaf and children subsampling LODs
         * on the way down.
         *
         * \note rng_mutex_ lock occurs
         */
        boost::uint64_t
        addDataToLeaf_and_genLOD (const AlignedPointTVector &p, const bool skip_bb_check);

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
        boost::uint64_t
        addDataAtMaxDepth (const AlignedPointTVector &p, const bool skip_bb_check);

        /** \brief Add data to the leaf when at max depth of tree. If
         *   \ref skip_bb_check is true, adds to the node regardless of the
         *   bounding box it represents; otherwise only adds points that
         *   fall within the bounding box 
         *
         *  \param[in] input_cloud PointCloud2 points to attempt to add to the tree; 
         *  \warning PointCloud2 inserted into the tree must have x,y,z fields, and must be of same type of any other points inserted in the tree
         *  \param[in] skip_bb_check (default true) if @b true, doesn't check that points
         *  are in the proper bounding box; if @b false, only adds the
         *  points that fall into the bounding box to this node 
         *  \return number of points successfully added
         */
        boost::uint64_t
        addDataAtMaxDepth (const sensor_msgs::PointCloud2::Ptr input_cloud, const bool skip_bb_check);
        
        /** \brief Randomly sample point data 
         *  \todo This needs to be deprecated; random sampling has its own class
         *  \todo Parameterize random sampling, uniform downsampling, etc...
         */
        void
        randomSample (const AlignedPointTVector &p, AlignedPointTVector &insertBuff, const bool skip_bb_check);

        /** \brief Subdivide points to pass to child nodes */
        void
        subdividePoints (const AlignedPointTVector &p, std::vector< AlignedPointTVector > &c, const bool skip_bb_check);

        /** \brief Subdivide a single point into a specific child node */
        void
        subdividePoint (const PointT &pt, std::vector< AlignedPointTVector > &c);

        /** \brief Write a python visual script to @b file
         * \param[in] file output file stream to write the python visual script
         */
        void
        writeVPythonVisual (std::ofstream &file);

      protected:
        /** \brief Load from disk 
         * If creating root, path is full name. If creating any other
         * node, path is dir; throws exception if directory or metadata not found
         *
         * \param[in] path
         * \param[in] super
         * \param[in] loadAll
         * \throws PCLException if directory is missing
         * \throws PCLException if node index is missing
         */
        octree_base_node (const boost::filesystem::path &path, octree_base_node<ContainerT, PointT>* super, bool load_all);

        /** \brief Create root node and directory
         *
         * Initializes the root node and performs initial filesystem checks for the octree; 
         * throws OctreeException::OCT_BAD_PATH if root directory is an existing file
         *
         * \param bb_min triple of x,y,z minima for bounding box
         * \param bb_max triple of x,y,z maxima for bounding box
         * \param tree adress of the tree data structure that will hold this initial root node
         * \param rootname Root directory for location of on-disk octree storage; if directory 
         * doesn't exist, it is created; if "rootname" is an existing file, 
         * 
         * \throws PCLException if the specified path already exists
         */
        void init_root_node (const Eigen::Vector3d &bb_min, const Eigen::Vector3d &bb_max, octree_base<ContainerT, PointT> * const tree, const boost::filesystem::path &rootname);

        /** \brief Creates child node \ref idx
         *  \param[in] idx Index (0-7) of the child node
         */
        void
        createChild (const int idx);

        /** \brief Write JSON metadata for this node to file */
        void
        saveMetadataToFile (const boost::filesystem::path &path);

        /** \brief Auxiliary method which computes the depth of the tree based on the dimension of the smallest voxel.
         *  \param[in] min_bb The minimum corner of t
         */
        int
        calcDepthForDim (const Eigen::Vector3d &min_bb, const Eigen::Vector3d &max_bb, const double dim);

        /** \brief Method which recursively free children of this node 
         */
        void
        recFreeChildren ();

        /** \brief Number of points in the payload */
        inline boost::uint64_t
        size () const
        {
          return payload_->size ();
        }

        /** \brief Number of children (0 or 8). */
        inline size_t
        num_child_ren () const
        {
          return num_child_;
        }

        void
        flushToDiskRecursive ();

        /** \brief Loads the nodes metadata from the JSON file 
         */
        void
        loadFromFile (const boost::filesystem::path &path, octree_base_node* super);

        /** \brief Save node's metadata to file
         * \param[in] recursive: if false, save only this node's metadata to file; if true, recursively
         * save all children's metadata to files as well
         */
        void
        saveIdx (bool recursive);

        /** \brief Recursively converts data files to ascii XZY files
         *  \note This will be deprecated soon
         */
        void
        convertToXYZRecursive ();

        /** \brief no copy construction right now */
        octree_base_node (const octree_base_node &rval);

        /** \brief Operator= is not implemented */
        octree_base_node&
        operator= (const octree_base_node &rval);

        /** \brief Private constructor used for children 
         */
        octree_base_node (const Eigen::Vector3d &bb_min, const Eigen::Vector3d &bb_max, const char* dir, octree_base_node<ContainerT, PointT>* super);

        /** \brief Copies points from this and all children into a single point container (std::list)
         */
        void
        copyAllCurrentAndChildPointsRec (std::list<PointT> &v);

        void
        copyAllCurrentAndChildPointsRec_sub (std::list<PointT> &v, const double percent);

        /** \brief Returns whether or not a node has unloaded children data */
        inline bool
        hasUnloadedChildren () const;

        /** \brief Load nodes child data creating new nodes for each */
        void
        loadChildren (bool recursive);

        /** \brief Gets a vector of occupied voxel centers
         * \param[out] voxel_centers
         * \param[in] query_depth
         */
        void
        getVoxelCentersRecursive (AlignedPointTVector &voxel_centers, const size_t query_depth);

        /** \brief Gets a vector of occupied voxel centers
         * \param[out] voxel_centers
         * \param[in] query_depth
         */
        void
        getVoxelCentersRecursive (std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > &voxel_centers, const size_t query_depth);

        /** \brief the dir containing the node's data and its children */
        boost::filesystem::path thisdir_;
        /** \brief the node's index file, node.idx */
        boost::filesystem::path thisnodeindex_;
        /** \brief the node's storage file, node.pcd */
        boost::filesystem::path thisnodestorage_;

        /** \brief The tree we belong to */
        octree_base<ContainerT, PointT>* m_tree_;//
        /** \brief The root node of the tree we belong to */
        octree_base_node* root_;//
        /** \brief super-node */
        octree_base_node* parent_;
        /** \brief Depth in the tree, root is 0, root's children are 1, ... */
        size_t depth_;
        /** \brief The children of this node */
        octree_base_node* children_[8];
        /** \brief number of children this node has. Between 0 and 8 inclusive */
        size_t num_child_;

        /** \brief what holds the points. currently a custom class, but in theory
         * you could use an stl container if you rewrote some of this class. I used
         * to use deques for this... */
        boost::shared_ptr<ContainerT> payload_;

        /** \brief The X,Y,Z axes-aligned minima for the bounding box*/
        Eigen::Vector3d min_;
        /** \brief The X,Y,Z axes-aligned maxima for the bounding box*/
        Eigen::Vector3d max_;
        /** \brief The midpoints of the X, Y and Z sides of the bounding boxes */
        Eigen::Vector3d mid_xyz_;

        /** \brief Random number generator mutex */
        static boost::mutex rng_mutex_;

        /** \brief Mersenne Twister: A 623-dimensionally equidistributed uniform
         * pseudo-random number generator */
        static boost::mt19937 rand_gen_;

        /** \brief Random number generator seed */
        const static boost::uint32_t rngseed = 0xAABBCCDD;
        /** \brief Extension for this class to find the pcd files on disk */
        const static std::string pcd_extension;

    };
  }//namespace outofcore
}//namespace pcl

#endif //PCL_OUTOFCORE_OCTREE_BASE_NODE_H_
