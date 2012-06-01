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
 */

/*
  This code defines the octree used for point storage at Urban Robotics. Please
  contact Jacob Schloss <jacob.schloss@urbanrobotics.net> with any questions.
  http://www.urbanrobotics.net/
*/

#ifndef PCL_OUTOFCORE_OCTREE_BASE_H_
#define PCL_OUTOFCORE_OCTREE_BASE_H_

// Boost
#include <boost/filesystem.hpp>
#include <boost/thread.hpp>

// PCL (Urban Robotics)
#include <pcl/outofcore/octree_base_node.h>
#include <pcl/outofcore/octree_disk_container.h>
#include <pcl/outofcore/octree_ram_container.h>

/** 
 *  \todo Add support for an array of input clouds or locations of pcd files on disk
 *  \todo clean up access specifiers to streamline public interface
 *  \todo add PCL macro into templated hpp files
 */
namespace pcl
{
  namespace outofcore
  {
  /** \class octree_base This code defines the octree used for point storage at Urban Robotics. 
   *   \note Code was adapted from the Urban Robotics out of core octree implementation. 
   *   Contact Jacob Schloss <jacob.schloss@urbanrobotics.net> with any questions. 
   *   http://www.urbanrobotics.net/
   *
   *  The primary purpose of this class is recursive traversal of the
   *  in-memory/top-level octree structure. The metadata in each node
   *  can be loaded into memory, and the tree traversed recursively in
   *  this state. This class provides an the interface for:
   *      I. Point/Region INSERTION methods
   *      II. Frustrum/box/region REQUESTS/QUERIES
   *      III. Parameterization of compression, resolution, container type, etc...
   */
    template<typename Container, typename PointT>
    class octree_base
    {
      friend class octree_base_node<Container, PointT> ;

      public:
        // public typedefs
        // UR Typedefs
        typedef octree_base<octree_disk_container < PointT > , PointT > octree_disk;
        typedef octree_base_node<octree_disk_container < PointT > , PointT > octree_disk_node;

        typedef octree_base<octree_ram_container< PointT> , PointT> octree_ram;
        typedef octree_base_node< octree_ram_container<PointT> , PointT> octree_ram_node;

        //typedef octree_disk octree;
        //typedef octree_disk_node octree_node;
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
         * \param rootname boost::filesystem::path to existing tree
         * \param load_all Load entire tree
         */
        octree_base (const boost::filesystem::path& rootname, const bool load_all);

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
         * \param node_dim_meters
         * \param rootname must end in ".oct_idx" (THIS SHOULD CHANGE)
         * \param coord_sys \todo put coordinate system into the templated PointT payload
         * \throws OctreeException(OCT_BAD_PATH) if file extension is not ".oct_idx"
         */
        octree_base (const double min[3], const double max[3], const double node_dim_meters, const boost::filesystem::path& rootname, const std::string& coord_sys);

        /** \brief Create a new tree; will not overwrite existing tree of same name
         *
         * Create a new tree rootname with specified bounding box; will not overwrite an existing tree
         *
         * \param max_depth Specifies a fixed number of LODs to generate
         * \param min Bounding box min
         * \param max Bounding box max
         * \param rootname must end in ".oct_idx" (THIS SHOULD CHANGE)
         * \param coord_sys
         * \throws OctreeException(OCT_CHILD_EXISTS) if the parent directory has existing children (detects an existing tree)
         * \throws OctreeException(OCT_BAD_PATH) if file extension is not ".oct_idx"
         */
        octree_base (const int max_depth, const double min[3], const double max[3], const boost::filesystem::path& rootname, const std::string& coord_sys);

        ~octree_base ();

        // Point/Region INSERTION methods
        // --------------------------------------------------------------------------------
        boost::uint64_t
        addDataToLeaf (const AlignedPointTVector& p);

        /** \brief Copies the points from the point_cloud falling within the bounding box of the octree to the
         *   out-of-core octree; this is an interface to addDataToLeaf and can be used multiple times.
         *  \param point_cloud Pointer to the point cloud data to copy to the outofcore octree; Assumes templated
         *   PointT matches for each.
         *  \return Number of points successfully copied from the point cloud to the octree.
         */
        boost::uint64_t
        addPointCloud (PointCloudConstPtr point_cloud);

        boost::uint64_t
        addPointCloud_and_genLOD (PointCloudConstPtr point_cloud);

        /** \brief Recursively add points to the tree subsampling LODs on the way.
         *
         * shared read_write_mutex lock occurs
         * \todo overload this to use shared point cloud pointer
         */
        boost::uint64_t
        addDataToLeaf_and_genLOD (AlignedPointTVector& p);
        
        

        // Frustrum/Box/Region REQUESTS/QUERIES: DB Accessors
        // -----------------------------------------------------------------------

        /** \brief Get bins at query_depth that intersect with your bin
         *
         * query_depth == 0 is root
         * query_depth == (this->depth) is full
         */
        void
        queryBBIntersects (const double min[3], const double max[3], const boost::uint32_t query_depth, std::list<std::string>& bin_name) const;

        //get Points in BB, returning all possible matches, including just BB intersect
        //bool queryBBInterects(const double min[3], const double max[3]);

        /** \brief get Points in BB, only points inside BB */
        void
        queryBBIncludes (const double min[3], const double max[3], size_t query_depth, std::list<PointT>& v) const;

        /** \brief random sample of points in BB includes
         *  \todo adjust for varying densities at different LODs */
        void
        queryBBIncludes_subsample (const double min[3], const double max[3], size_t query_depth, const double percent, std::list<PointT>& v) const;


        // Parameterization: getters and setters
        // --------------------------------------------------------------------------------
        /** 
            \todo downsampling of queries
            \todo downsampling of octree during construction (or leave that to the user's own preprocessing)
            \todo parameterize compression 
            \todo parameterize container type (template?)
        */


        /** \brief Copy the overall BB to min max */
        inline bool
        getBB (double min[3], double max[3]) const
        {
          if (root_ != NULL)
          {
            root_->getBB (min, max);
            return true;
          }
          return false;
        }

        /** \brief Get number of points at specified LOD 
         * \param[in] depth the level of detail at which we want the number of points (0 is root, 1, 2,...)
         * \return number of points in the lodPoints_ cache 
         */
        inline boost::uint64_t
        getNumPointsAtDepth (const boost::uint64_t depth) const
        {
          assert ( depth < lodPoints_.size () );
          
          return lodPoints_[depth];
        }

        /** \brief Get number of points at each LOD */
        inline const std::vector<boost::uint64_t>&
        getNumPointsVector () const
        {
          return lodPoints_;
        }

        /** \brief Get number of LODs
         *
         * Assume fully balanced tree -- all nodes have 8 children, and all branches
         * are same depth
         */
        inline boost::uint64_t
        getDepth () const
        {
          return max_depth_;
        }

        /** \brief Assume fully balanced tree -- all nodes have same dim */
        bool
        getBinDimension (double& x, double& y) const
        {
          if (root_ == NULL)
          {
            x = 0;
            y = 0;
            return false;
          }

          double y_len = root_->max_[1] - root_->min_[1];
          double x_len = root_->max_[0] - root_->min_[0];

          y = y_len * pow (.5, double (max_depth_));
          x = x_len * pow (.5, double (max_depth_));

          return true;
        }

        double
        getVoxelSideLength (const boost::uint64_t depth) const
        {
          return (root_->max_[0] - root_->min_[0]) * pow (.5, double (max_depth_)) * double (1 << (max_depth_ - depth));
        }

        double
        getVoxelSideLength () const
        {
          return getVoxelSideLength (max_depth_);
        }

        /** \brief Get coord system tag in the metadata */
        const std::string&
        getCoordSystem ()
        {
          return coord_system_;
        }

        // Mutators
        // -----------------------------------------------------------------------

        /** \brief Generate LODs for the tree */
        void
        buildLOD ();

        /** \brief Recursively add points to the tree 
         *  \note shared read_write_mutex lock occurs
         *  \todo overload this to use shared point cloud pointer
         */

        void
        printBBox(const size_t query_depth) const;

        void
        printBBox() const
        {
          printBBox(max_depth_);
        }

        void
        getVoxelCenters(AlignedPointTVector &voxel_centers, size_t query_depth) const;

        void
        getVoxelCenters(AlignedPointTVector &voxel_centers) const
        {
          getVoxelCenters(voxel_centers, max_depth_);
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
        writeVPythonVisual (const char* file);

        // (note from UR) The following are DEPRECATED since I found that writeback caches did not
        // scale well, and they are currently disabled in the backend

        /** \brief DEPRECATED - Flush all nodes' cache 
         *  \note this was moved to the octree_node class
         */
        void
        flushToDisk ();

        /** \brief DEPRECATED - Flush all non leaf nodes' cache */
        void
        flushToDiskLazy ();

        /** \brief DEPRECATED - Flush empty nodes only */
        void
        DeAllocEmptyNodeCache ();

      protected:

        octree_base (octree_base& rval);
        octree_base (const octree_base& rval);

        octree_base&
        operator= (octree_base& rval);

        octree_base&
        operator= (const octree_base& rval);

        /** \brief flush empty nodes only */
        void
        DeAllocEmptyNodeCache (octree_base_node<Container, PointT>* current);

        /** \brief Write octree definition ".octree" (defined by octree_extension_) to disk */
        void
        saveToFile ();

        void
        loadFromFile ();

        /** \brief recursive portion of lod builder
         * \todo does this need to be public? */
        void
        buildLOD (octree_base_node<Container, PointT>** current_branch, const int current_dims);

        /** \brief Increment current depths (LOD for branch nodes) point count; called by addDataAtMaxDepth in octree_base_node
         * \todo rename count_point to something more informative
         */
        inline void
        incrementPointsInLOD (boost::uint64_t depth, boost::uint64_t inc)
        {
          //if we overflow here, we've got one massive octree
          assert ( std::numeric_limits<uint64_t>::max () - inc > inc );

          lodPoints_[depth] += inc;
        }
    
        /** \brief Pointer to the root node of the octree data structure */
        octree_base_node<Container, PointT>* root_;
        /** \brief shared mutex for controlling read/write access to disk */
        mutable boost::shared_mutex read_write_mutex_;
        /** \brief vector indexed by depth containing number of points at each level of detail */
        std::vector<boost::uint64_t> lodPoints_;
        /** \brief the pre-set maximum depth of the tree */
        boost::uint64_t max_depth_;
        /** \brief boost::filesystem::path to the location of the root of
         *  the tree on disk relative to execution directory*/
        boost::filesystem::path treepath_;
        /** \brief string representing the coordinate system
         *
         *  \note Goal is to support: WGS84 (World Geodetic System), UTM
         *  (Universal Transverse Mercator), ECEF (Earth Centered Earth
         *  Fixed) and more. Currently nothing special is done for each
         *  coordinate system.
         */
        std::string coord_system_;
        /** \brief defined as ".octree" to append to treepath files
         * 
         *  \note this might change
         */
        const static std::string TREE_EXTENSION_;
        const static int OUTOFCORE_VERSION_;

        /** \todo @b loadcount mystery constant 2e9 points at a time to subsample; should parameterize; use mmap */
        const static uint64_t LOAD_COUNT_ = static_cast<uint64_t>(2e9);

        double resolution_;

    };
  }
}

  
#endif // PCL_OUTOFCORE_OCTREE_BASE_H_
