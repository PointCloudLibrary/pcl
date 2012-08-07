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
    template<typename Container, typename PointT>
    class octree_base_node;

    template<typename Container, typename PointT>
    class octree_base;

    /** \brief document */
    template<typename Container, typename PointT> octree_base_node<Container, PointT>*
    makenode_norec (const boost::filesystem::path& path, octree_base_node<Container, PointT>* super);

    /** \brief document */
    template<typename Container, typename PointT> void
    queryBBIntersects_noload (const boost::filesystem::path& root_node, const Eigen::Vector3d& min, const Eigen::Vector3d& max, const boost::uint32_t query_depth, std::list<std::string>& bin_name);

    /** \brief document */
    template<typename Container, typename PointT> void
    queryBBIntersects_noload (octree_base_node<Container, PointT>* current, const Eigen::Vector3d&, const Eigen::Vector3d& max, const boost::uint32_t query_depth, std::list<std::string>& bin_name);

/** \class octree_base_node
    Document this class
 */
    template<typename Container, typename PointT>
    class octree_base_node
    {
      friend class octree_base<Container, PointT> ;
  
      friend octree_base_node<Container, PointT>*
      makenode_norec<Container, PointT> (const boost::filesystem::path& path, octree_base_node<Container, PointT>* super);
  
      friend void
      queryBBIntersects_noload<Container, PointT> (const boost::filesystem::path& rootnode, const Eigen::Vector3d& min, const Eigen::Vector3d& max, const boost::uint32_t query_depth, std::list<std::string>& bin_name);

      friend void
      queryBBIntersects_noload<Container, PointT> (octree_base_node<Container, PointT>* current, const Eigen::Vector3d& min, const Eigen::Vector3d& max, const boost::uint32_t query_depth, std::list<std::string>& bin_name);
  
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
        octree_base_node () 
          : parent_ (NULL),
            root_ (NULL),
            depth_ (0),
            num_child_ (0),
            mid_xyz_ (Eigen::Vector3d (0, 0, 0)),
            min_ (Eigen::Vector3d (0, 0, 0)),
            max_ (Eigen::Vector3d (0, 0, 0))
        {
          memset (children_, 0, 8 * sizeof(octree_base_node<Container, PointT>*));
        }

        /** \brief Create root node and directory setting voxel size*/
        octree_base_node (const Eigen::Vector3d& bb_min, const Eigen::Vector3d& bb_max, const double node_dim_meters, octree_base<Container, PointT> * const tree, const boost::filesystem::path& root_name);

        /** \brief Create root node and directory setting setting max depth*/
        octree_base_node (const int max_depth, const Eigen::Vector3d& bb_min, const Eigen::Vector3d& bb_max, octree_base<Container, PointT> * const tree, const boost::filesystem::path& root_name);

        /** \brief Will recursively delete all children calling recFreeChildrein */
        ~octree_base_node ();

        //query
        /** \brief gets the minimum and maximum corner of the bounding box represented by this node
         * \param[out] minCoord returns the minimum corner of the bounding box indexed by 0-->X, 1-->Y, 2-->Z 
         * \param[out] maxCoord returns the maximum corner of the bounding box indexed by 0-->X, 1-->Y, 2-->Z 
         */
        inline void
        getBB (Eigen::Vector3d& minCoord, Eigen::Vector3d& maxCoord) const
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
         *  \todo benchmark queryBBIncludes
         */
        void
        queryBBIncludes (const Eigen::Vector3d& min_bb, const Eigen::Vector3d& max_bb, size_t query_depth, AlignedPointTVector& dst);

        /** \brief Recursively add points that fall into the queried bounding box up to the \b query_depth
         *
         *  \param[in] min_bb the minimum corner of the bounding box, indexed by X,Y,Z coordinates
         *  \param[in] max_bb the maximum corner of the bounding box, indexed by X,Y,Z coordinates
         *  \param[in] query_depth the maximum depth to query in the octree for points within the bounding box
         *  \param[out] dst_blob destion of points returned by the queries
         *  \todo benchmark queryBBIncludes
         *  \todo use this as wrapper for queryBBIncludes into AlignedPointTVector
         */
        void
        queryBBIncludes (const Eigen::Vector3d& min_bb, const Eigen::Vector3d& max_bb, size_t query_depth, const sensor_msgs::PointCloud2::Ptr& dst_blob);

        /** \brief Recursively add points that fall into the queried bounding box up to the \b query_depth 
         *
         *  \param[in] min_bb the minimum corner of the bounding box, indexed by X,Y,Z coordinates
         *  \param[in] max_bb the maximum corner of the bounding box, indexed by X,Y,Z coordinates
         *  \param[in] query_depth
         *  \param[out] v std::list of points returned by the query
         *
         *  \todo clean up the interface and standardize the parameters to these functions
         */
        void
        queryBBIncludes_subsample (const Eigen::Vector3d& min_bb, const Eigen::Vector3d& max_bb, int query_depth, const double percent, AlignedPointTVector& v);

        //bin extraction
        //query_depth == 0 is root
        //query_depth == tree->depth is leaf
        /** \brief Tests if the coordinate falls within the
         * boundaries of the bounding box, inclusively
         */
        void
        queryBBIntersects (const Eigen::Vector3d& min_bb, const Eigen::Vector3d& max_bb, const boost::uint32_t query_depth, std::list<std::string>& file_names);

        /** \brief Write the voxel size to stdout at \ref query_depth 
         * \param[in] query_depth The depth at which to print the size of the voxel/bounding boxes
         */
        void
        printBBox(const size_t query_depth) const;

        //bb check
        //checks if 
        /** \brief document */
        inline bool
        intersectsWithBB (const Eigen::Vector3d& min_bb, const Eigen::Vector3d& max_bb) const;

        /** \brief document */
        inline bool
        withinBB (const Eigen::Vector3d& min, const Eigen::Vector3d& max) const;
        
        bool
        pointWithinBB (const Eigen::Vector3d& min_bb, const Eigen::Vector3d& max_bb, const Eigen::Vector3d& point);
        

        /** \brief document */
        static inline bool
        pointWithinBB (const Eigen::Vector3d& min_bb, const Eigen::Vector3d& max_bb, const PointT& p);

        /** \brief document */
        static inline bool
        pointWithinBB ( const Eigen::Vector3d& min_bb, const Eigen::Vector3d& max_bb, const double x, const double y, const double z );

        /** \brief Check whether specified point is within bounds of current node */
        inline bool
        pointWithinBB (const PointT& p) const;

        /** \todo: All of the add data methods check whether the point being added is
         *       within the bounds of the octree created.  This requires a traversal
         *       of the cloud to find the min/max before proceeding to adding the
         *       points.
         */

        /** \brief add point to this node if we are a leaf, or find the leaf below us that is supposed to take the point 
         *  \param[in] p vector of points to add to the leaf
         *  \param[in] skipBBCheck whether to check if the point's coordinates fall within the bounding box
         */
        boost::uint64_t
        addDataToLeaf (const AlignedPointTVector& p, const bool skip_bb_check);

        boost::uint64_t
        addDataToLeaf (const std::vector<const PointT*>& p, const bool skip_bb_check);

        /** \brief Add a single point to the octree
         * 
         * \param[in] p The point (templated) to add to the tree
         */
        void
        addPointToLeaf (const PointT& p);

        /** \brief Add a single PointCloud2 object into the octree.
         *
         * \param[in] input_cloud
         * \param[in] skip_bb_check (default = false)
         */
        boost::uint64_t
        addPointCloud ( const sensor_msgs::PointCloud2::Ptr& input_cloud, const bool skip_bb_check = false )
        {
      
          if ( input_cloud->height*input_cloud->width == 0)
            return (0);
      

          if( this->depth_ == root_->m_tree_->max_depth_)
            return (addDataAtMaxDepth (input_cloud, true));
      
          if( num_child_ < 8 )
            if(hasUnloadedChildren ())
              loadChildren (false);

          if( skip_bb_check == false )
          {

            //indices to store the points for each bin
            //these lists will be used to copy data to new point clouds and pass down recursively
            std::vector < std::vector<int> > indices;
            indices.resize (8);

            int x_idx = pcl::getFieldIndex (*input_cloud , std::string ("x") );
            int y_idx = pcl::getFieldIndex (*input_cloud, std::string ("y") );
            int z_idx = pcl::getFieldIndex (*input_cloud, std::string ("z") );

            int x_offset = input_cloud->fields[x_idx].offset;
            int y_offset = input_cloud->fields[y_idx].offset;
            int z_offset = input_cloud->fields[z_idx].offset;
      
            for ( size_t point_idx =0; point_idx < input_cloud->data.size (); point_idx +=input_cloud->point_step )
            {
              PointXYZ local_pt;

              local_pt.x = * (reinterpret_cast<float*>(&input_cloud->data[point_idx + x_offset]));
              local_pt.y = * (reinterpret_cast<float*>(&input_cloud->data[point_idx + y_offset]));
              local_pt.z = * (reinterpret_cast<float*>(&input_cloud->data[point_idx + z_offset]));

              if( !this->pointWithinBB (local_pt) )
              {
                PCL_ERROR ( "[pcl::outofcore::octree_base_node::%s] Failed to place point ( %.2f,%.2f,%.2f) within bounding box\n", __FUNCTION__, local_pt.x, local_pt.y, local_pt.z );
                continue;
              }

              //compute the box we are in
              size_t box = 0;
              box = ((local_pt.z >= mid_xyz_[2]) << 2) | ((local_pt.y >= mid_xyz_[1]) << 1) | ((local_pt.x >= mid_xyz_[0]) << 0);
              assert ( box < 8 );
              
              //insert to the vector of indices
              indices[box].push_back ( static_cast<int> ( point_idx / input_cloud->point_step ) );
            }

            boost::uint64_t points_added = 0;

            for(int i=0; i<8; i++)
            {
              if ( indices[i].empty () )
                continue;

              if ( children_[i] == false )
              {
                createChild (i);
              }

              sensor_msgs::PointCloud2::Ptr dst_cloud (new sensor_msgs::PointCloud2 () );

//              PCL_INFO ( "[pcl::outofcore::octree_base_node::%s] Extracting indices to bins\n", __FUNCTION__);
              
              //copy the points from extracted indices from input cloud to destination cloud
              pcl::copyPointCloud ( *input_cloud, indices[i], *dst_cloud ) ;
          
              //recursively add the new cloud to the data
              points_added += children_[i]->addPointCloud ( dst_cloud );
              indices[i].clear ();
            }
        
            return (points_added);
          }
      
          PCL_ERROR ("[pcl::outofcore::octree_base_node] Skipped bb check. Points not inserted\n");
      
          return 0;
        }

        /** \brief Add a single PointCloud2 into the octree and build the subsampled LOD during construction */
        boost::uint64_t
        addPointCloud_and_genLOD (const sensor_msgs::PointCloud2::Ptr input_cloud); //, const bool skip_bb_check);
        
        /** \brief Recursively add points to the leaf and children subsampling LODs
         * on the way down.
         *
         * \note rng_mutex_ lock occurs
         */
        boost::uint64_t
        addDataToLeaf_and_genLOD (const AlignedPointTVector& p, const bool skip_bb_check);

        /** \todo Do we need to support std::vector<PointT*>?
         *
         * boost::uint64_t
         * addDataToLeaf_and_genLOD (const std::vector<PointT*>& p, const bool skip_bb_check);
         */

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
        addDataAtMaxDepth (const AlignedPointTVector& p, const bool skip_bb_check);

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
        addDataAtMaxDepth ( const sensor_msgs::PointCloud2::Ptr input_cloud, const bool skip_bb_check = true )
        {
          //this assumes data is already in the correct bin
          if(skip_bb_check == true)
          {
//            PCL_INFO ("[pcl::outofcore::octree_base_node::%s] Adding %u points at max depth, %u\n",__FUNCTION__, input_cloud->width*input_cloud->height, this->depth_);
            
            root_->m_tree_->incrementPointsInLOD (this->depth_, input_cloud->width * input_cloud->height );
            payload_->insertRange (input_cloud);            
            return (input_cloud->width * input_cloud->height);
          }
          else
          {
            PCL_ERROR ("[pcl::outofcore::octree_base_node] Not implemented\n");
            return (0);
          }
        }
        
        /** \brief document */
        void 
        randomSample ( const typename PointCloud<PointT>::Ptr input_cloud, 
                       typename PointCloud<PointT>::Ptr output_cloud, 
                                         const bool skip_bb_check);

        /** \brief Randomly sample point data */
        void
        randomSample(const AlignedPointTVector& p, AlignedPointTVector& insertBuff, const bool skip_bb_check);

        /** \brief Subdivide points to pass to child nodes */
        void
        subdividePoints (const AlignedPointTVector& p, std::vector< AlignedPointTVector >& c, const bool skip_bb_check);

        /** \brief Subdivide a single point into a specific child node */
        void
        subdividePoint (const PointT& pt, std::vector< AlignedPointTVector >& c);

        /** \brief Write a python visual script to @b file
         * \param[in] file output file stream to write the python visual script
         */
        void
        writeVPythonVisual (std::ofstream& file);

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
        octree_base_node (const boost::filesystem::path& path, octree_base_node<Container, PointT>* super, bool load_all);

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
        void init_root_node (const Eigen::Vector3d& bb_min, const Eigen::Vector3d& bb_max, octree_base<Container, PointT> * const tree, const boost::filesystem::path& rootname);

        /** \brief document */
        void
        createChild (const int idx);

        /** \brief Write JSON metadata for this node to file */
        void
        saveMetadataToFile (const boost::filesystem::path& path);

        /** \brief document */
        int
        calcDepthForDim (const Eigen::Vector3d& min_bb, const Eigen::Vector3d& max_bb, const double dim);

        /** \brief document */
        void
        freeChildren ();

        /** \brief document */
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

        /** \brief Flush payload's cache to disk */
        void
        flushToDisk ();

        /** \brief Document or deprecate */
        void
        flushToDiskLazy ();

        /** \brief Document */
        void
        flush_DeAlloc_this_only ();

        /** \brief document */
        void
        loadFromFile (const boost::filesystem::path& path, octree_base_node* super);

        /** \brief Save node's metadata to file
         * \param[in] recursive: if false, save only this node's metadata to file; if true, recursively
         * save all children's metadata to files as well
         */
        void
        saveIdx (bool recursive);

        /** \brief document */
        void
        convertToXYZ ();

        /** \brief no copy construction right now */
        octree_base_node (const octree_base_node& rval);

        /** \brief document */
        octree_base_node&
        operator= (const octree_base_node& rval);

        /** \brief document */
        octree_base_node (const Eigen::Vector3d& bb_min, const Eigen::Vector3d& bb_max, const char* dir, octree_base_node<Container, PointT>* super);

        /** \brief document */
        void
        copyAllCurrentAndChildPointsRec (std::list<PointT>& v);

        void
        copyAllCurrentAndChildPointsRec_sub (std::list<PointT>& v, const double percent);

        /** \brief Returns whether or not a node has unloaded children data */
        inline bool
        hasUnloadedChildren () const;

        /** \brief Load nodes child data creating new nodes for each */
        void
        loadChildren (bool recursive);

        /** \brief document 
         * \param[out] voxel_centers
         * \param[in] query_depth
         */
        void
        getVoxelCenters(AlignedPointTVector &voxel_centers, const size_t query_depth);

        /** \brief document 
         * \param[out] voxel_centers
         * \param[in] query_depth
         */
        void
        getVoxelCenters(std::vector<Eigen::Vector3d> &voxel_centers, const size_t query_depth);

        /** \brief the dir containing the node's data and its children */
        boost::filesystem::path thisdir_;
        /** \brief the node's index file, node.idx */
        boost::filesystem::path thisnodeindex_;
        /** \brief the node's storage file, node.pcd */
        boost::filesystem::path thisnodestorage_;

        /** \brief The tree we belong to */
        octree_base<Container, PointT>* m_tree_;//
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
        Container* payload_;

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
        /** \brief document */
        const static std::string pcd_extension;

    };
  }//namespace outofcore
}//namespace pcl

#endif //PCL_OUTOFCORE_OCTREE_BASE_NODE_H_
