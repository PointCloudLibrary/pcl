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

#ifndef PCL_OUTOFCORE_OCTREE_BASE_IMPL_H_
#define PCL_OUTOFCORE_OCTREE_BASE_IMPL_H_


#include <pcl/outofcore/octree_base.h>

// JSON
#include <pcl/outofcore/cJSON.h>

#include <pcl/filters/random_sample.h>
#include <pcl/filters/extract_indices.h>

// C++
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <exception>

namespace pcl
{
  namespace outofcore
  {

    ////////////////////////////////////////////////////////////////////////////////
    //Static constants
    ////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT>    
    const std::string OutofcoreOctreeBase<ContainerT, PointT>::TREE_EXTENSION_ = ".octree";

    template<typename ContainerT, typename PointT>
    const int OutofcoreOctreeBase<ContainerT, PointT>::OUTOFCORE_VERSION_ = static_cast<int>(3);

    ////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT>
    OutofcoreOctreeBase<ContainerT, PointT>::OutofcoreOctreeBase (const boost::filesystem::path& root_name, const bool load_all)
      : root_node_ ()
      , read_write_mutex_ ()
      , metadata_ (new OutofcoreOctreeBaseMetadata ())
      , sample_percent_ (0.125)
      , lod_filter_ptr_ (new pcl::RandomSample<pcl::PCLPointCloud2> ())
    {
      //validate the root filename
      if (!this->checkExtension (root_name))
      {
        PCL_THROW_EXCEPTION (PCLException, "[pcl::outofcore::OutofcoreOctreeBase] Bad extension. Outofcore Octrees must have a root node ending in .oct_idx\n");
      }
      
      // Create root_node_node
      root_node_ = new OutofcoreOctreeBaseNode<ContainerT, PointT> (root_name, nullptr, load_all);
      // Set root_node_nodes tree to the newly created tree
      root_node_->m_tree_ = this;

      // Set the path to the outofcore octree metadata (unique to the root folder) ending in .octree
      boost::filesystem::path treepath = root_name.parent_path () / (boost::filesystem::basename (root_name) + TREE_EXTENSION_);

      //Load the JSON metadata
      metadata_->loadMetadataFromDisk (treepath);
    }

    ////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT>
    OutofcoreOctreeBase<ContainerT, PointT>::OutofcoreOctreeBase (const Eigen::Vector3d& min, const Eigen::Vector3d& max, const double resolution_arg, const boost::filesystem::path& root_node_name, const std::string& coord_sys)
      : root_node_()
      , read_write_mutex_ ()
      , metadata_ (new OutofcoreOctreeBaseMetadata ())
      , sample_percent_ (0.125)
      , lod_filter_ptr_ (new pcl::RandomSample<pcl::PCLPointCloud2> ())
    {
      //Enlarge the bounding box to a cube so our voxels will be cubes
      Eigen::Vector3d tmp_min = min;
      Eigen::Vector3d tmp_max = max;
      this->enlargeToCube (tmp_min, tmp_max);

      //Compute the depth of the tree given the resolution
      std::uint64_t depth = this->calculateDepth (tmp_min, tmp_max, resolution_arg);

      //Create a new outofcore tree
      this->init (depth, tmp_min, tmp_max, root_node_name, coord_sys);
    }

    ////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT>
    OutofcoreOctreeBase<ContainerT, PointT>::OutofcoreOctreeBase (const std::uint64_t max_depth, const Eigen::Vector3d& min, const Eigen::Vector3d& max, const boost::filesystem::path& root_node_name, const std::string& coord_sys)
      : root_node_()
      , read_write_mutex_ ()
      , metadata_ (new OutofcoreOctreeBaseMetadata ())
      , sample_percent_ (0.125)
      , lod_filter_ptr_ (new pcl::RandomSample<pcl::PCLPointCloud2> ())
    {
      //Create a new outofcore tree
      this->init (max_depth, min, max, root_node_name, coord_sys);
    }

    ////////////////////////////////////////////////////////////////////////////////
    template<typename ContainerT, typename PointT> void
    OutofcoreOctreeBase<ContainerT, PointT>::init (const std::uint64_t& depth, const Eigen::Vector3d& min, const Eigen::Vector3d& max, const boost::filesystem::path& root_name, const std::string& coord_sys)
    {
      //Validate the extension of the pathname
      if (!this->checkExtension (root_name))
      {
        PCL_THROW_EXCEPTION (PCLException, "[pcl::outofcore::OutofcoreOctreeBase] Bad extension. Outofcore Octrees must have a root node ending in .oct_idx\n");
      }

      //Check to make sure that we are not overwriting existing data
      if (boost::filesystem::exists (root_name.parent_path ()))
      {
        PCL_ERROR ("[pcl::outofcore::OutofcoreOctreeBase] A dir named %s already exists. Overwriting an existing tree is not supported.\n", root_name.parent_path ().c_str () );
        PCL_THROW_EXCEPTION ( PCLException, "[pcl::outofcore::OutofcoreOctreeBase] Directory exists; Overwriting an existing tree is not supported\n");
      }

      // Get fullpath and recreate directories
      boost::filesystem::path dir = root_name.parent_path ();

      if (!boost::filesystem::exists (dir))
      {
        boost::filesystem::create_directory (dir);
      }

      Eigen::Vector3d tmp_min = min;
      Eigen::Vector3d tmp_max = max;
      this->enlargeToCube (tmp_min, tmp_max);

      // Create root node
      root_node_= new OutofcoreOctreeBaseNode<ContainerT, PointT> (tmp_min, tmp_max, this, root_name);
      root_node_->m_tree_ = this;
      
      // Set root nodes file path
      boost::filesystem::path treepath = dir / (boost::filesystem::basename (root_name) + TREE_EXTENSION_);

      //fill the fields of the metadata
      metadata_->setCoordinateSystem (coord_sys);
      metadata_->setDepth (depth);
      metadata_->setLODPoints (depth+1);
      metadata_->setMetadataFilename (treepath);
      metadata_->setOutofcoreVersion (OUTOFCORE_VERSION_);
      //metadata_->setPointType ( <point type string here> );

      //save to disk
      metadata_->serializeMetadataToDisk ();
    }
    
    
    ////////////////////////////////////////////////////////////////////////////////
    template<typename ContainerT, typename PointT>
    OutofcoreOctreeBase<ContainerT, PointT>::~OutofcoreOctreeBase ()
    {
      root_node_->flushToDiskRecursive ();

      saveToFile ();
      delete (root_node_);
    }

    ////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT> void
    OutofcoreOctreeBase<ContainerT, PointT>::saveToFile ()
    {
      this->metadata_->serializeMetadataToDisk ();
    }

    ////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT> std::uint64_t
    OutofcoreOctreeBase<ContainerT, PointT>::addDataToLeaf (const AlignedPointTVector& p)
    {
      std::unique_lock < std::shared_timed_mutex > lock (read_write_mutex_);

      const bool _FORCE_BB_CHECK = true;
      
      std::uint64_t pt_added = root_node_->addDataToLeaf (p, _FORCE_BB_CHECK);

      assert (p.size () == pt_added);

      return (pt_added);
    }

    ////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT> std::uint64_t
    OutofcoreOctreeBase<ContainerT, PointT>::addPointCloud (PointCloudConstPtr point_cloud)
    {
      return (addDataToLeaf (point_cloud->points));
    }
    
    ////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT> std::uint64_t
    OutofcoreOctreeBase<ContainerT, PointT>::addPointCloud (pcl::PCLPointCloud2::Ptr &input_cloud, const bool skip_bb_check)
    {
      std::uint64_t pt_added = this->root_node_->addPointCloud (input_cloud, skip_bb_check) ;
//      assert (input_cloud->width*input_cloud->height == pt_added);
      return (pt_added);
    }

    
    ////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT> std::uint64_t
    OutofcoreOctreeBase<ContainerT, PointT>::addPointCloud_and_genLOD (PointCloudConstPtr point_cloud)
    {
      // Lock the tree while writing
      std::unique_lock < std::shared_timed_mutex > lock (read_write_mutex_);
      std::uint64_t pt_added = root_node_->addDataToLeaf_and_genLOD (point_cloud->points, false);
      return (pt_added);
    }

    ////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT> std::uint64_t
    OutofcoreOctreeBase<ContainerT, PointT>::addPointCloud_and_genLOD (pcl::PCLPointCloud2::Ptr &input_cloud)
    {
      // Lock the tree while writing
      std::unique_lock < std::shared_timed_mutex > lock (read_write_mutex_);
      std::uint64_t pt_added = root_node_->addPointCloud_and_genLOD (input_cloud);
      
      PCL_DEBUG ("[pcl::outofcore::OutofcoreOctreeBase::%s] Points added %lu, points in input cloud, %lu\n",__FUNCTION__, pt_added, input_cloud->width*input_cloud->height );
 
      assert ( input_cloud->width*input_cloud->height == pt_added );

      return (pt_added);
    }

    ////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT> std::uint64_t
    OutofcoreOctreeBase<ContainerT, PointT>::addDataToLeaf_and_genLOD (AlignedPointTVector& src)
    {
      // Lock the tree while writing
      std::unique_lock < std::shared_timed_mutex > lock (read_write_mutex_);
      std::uint64_t pt_added = root_node_->addDataToLeaf_and_genLOD (src, false);
      return (pt_added);
    }

    ////////////////////////////////////////////////////////////////////////////////

    template<typename Container, typename PointT> void
    OutofcoreOctreeBase<Container, PointT>::queryFrustum (const double planes[24], std::list<std::string>& file_names) const
    {
      std::shared_lock < std::shared_timed_mutex > lock (read_write_mutex_);
      root_node_->queryFrustum (planes, file_names, this->getTreeDepth());
    }

    ////////////////////////////////////////////////////////////////////////////////

    template<typename Container, typename PointT> void
    OutofcoreOctreeBase<Container, PointT>::queryFrustum(const double *planes, std::list<std::string>& file_names, const std::uint32_t query_depth) const
    {
      std::shared_lock < std::shared_timed_mutex > lock (read_write_mutex_);
      root_node_->queryFrustum (planes, file_names, query_depth);
    }

    ////////////////////////////////////////////////////////////////////////////////

    template<typename Container, typename PointT> void
    OutofcoreOctreeBase<Container, PointT>::queryFrustum (
        const double *planes, 
        const Eigen::Vector3d &eye, 
        const Eigen::Matrix4d &view_projection_matrix, 
        std::list<std::string>& file_names, 
        const std::uint32_t query_depth) const
    {
      std::shared_lock < std::shared_timed_mutex > lock (read_write_mutex_);
      root_node_->queryFrustum (planes, eye, view_projection_matrix, file_names, query_depth);
    }

    ////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT> void
    OutofcoreOctreeBase<ContainerT, PointT>::queryBBIncludes (const Eigen::Vector3d& min, const Eigen::Vector3d& max, const std::uint64_t query_depth, AlignedPointTVector& dst) const
    {
      std::shared_lock < std::shared_timed_mutex > lock (read_write_mutex_);
      dst.clear ();
      PCL_DEBUG ("[pcl::outofcore::OutofcoreOctreeBaseNode] Querying Bounding Box %.2lf %.2lf %.2lf, %.2lf %.2lf %.2lf", min[0], min[1], min[2], max[0], max[1], max[2]);
      root_node_->queryBBIncludes (min, max, query_depth, dst);
    }

    ////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT> void
    OutofcoreOctreeBase<ContainerT, PointT>::queryBBIncludes (const Eigen::Vector3d& min, const Eigen::Vector3d& max, const std::uint64_t query_depth, const pcl::PCLPointCloud2::Ptr& dst_blob) const
    {
      std::shared_lock < std::shared_timed_mutex > lock (read_write_mutex_);

      dst_blob->data.clear ();
      dst_blob->width = 0;
      dst_blob->height =1;

      root_node_->queryBBIncludes ( min, max, query_depth, dst_blob );
    }

    ////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT> void
    OutofcoreOctreeBase<ContainerT, PointT>::queryBBIncludes_subsample (const Eigen::Vector3d& min, const Eigen::Vector3d& max, const std::uint64_t query_depth, const double percent, AlignedPointTVector& dst) const
    {
      std::shared_lock < std::shared_timed_mutex > lock (read_write_mutex_);
      dst.clear ();
      root_node_->queryBBIncludes_subsample (min, max, query_depth, percent, dst);
    }

    ////////////////////////////////////////////////////////////////////////////////
    template<typename ContainerT, typename PointT> void
    OutofcoreOctreeBase<ContainerT, PointT>::queryBoundingBox (const Eigen::Vector3d &min, const Eigen::Vector3d &max, const int query_depth, const pcl::PCLPointCloud2::Ptr &dst_blob, double percent)
    {
      if (percent==1.0)
      {
        root_node_->queryBBIncludes (min, max, query_depth, dst_blob);
      }
      else
      {
        root_node_->queryBBIncludes_subsample (min, max, query_depth, dst_blob, percent);
      }
    }

    ////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT> bool
    OutofcoreOctreeBase<ContainerT, PointT>::getBoundingBox (Eigen::Vector3d &min, Eigen::Vector3d &max) const
    {
      if (root_node_!= nullptr)
      {
        root_node_->getBoundingBox (min, max);
        return true;
      }
      return false;
    }

    ////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT> void
    OutofcoreOctreeBase<ContainerT, PointT>::printBoundingBox(const std::size_t query_depth) const
    {
      std::shared_lock < std::shared_timed_mutex > lock (read_write_mutex_);
      root_node_->printBoundingBox (query_depth);
    }

    ////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT> void
    OutofcoreOctreeBase<ContainerT, PointT>::getOccupiedVoxelCenters(AlignedPointTVector &voxel_centers, const std::size_t query_depth) const
    {
      std::shared_lock < std::shared_timed_mutex > lock (read_write_mutex_);
      if (query_depth > metadata_->getDepth ()) 
      {
        root_node_->getOccupiedVoxelCentersRecursive (voxel_centers, metadata_->getDepth ());
      }
      else
      {
        root_node_->getOccupiedVoxelCentersRecursive (voxel_centers, query_depth);
      }
    }

    ////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT> void
    OutofcoreOctreeBase<ContainerT, PointT>::getOccupiedVoxelCenters(std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > &voxel_centers, const std::size_t query_depth) const
    {
      std::shared_lock < std::shared_timed_mutex > lock (read_write_mutex_);
      if (query_depth > metadata_->getDepth ())
      {
        root_node_->getOccupiedVoxelCentersRecursive (voxel_centers, metadata_->getDepth ());
      }
      else
      {
        root_node_->getOccupiedVoxelCentersRecursive (voxel_centers, query_depth);
      }
    }

    ////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT> void
    OutofcoreOctreeBase<ContainerT, PointT>::queryBBIntersects (const Eigen::Vector3d& min, const Eigen::Vector3d& max, const std::uint32_t query_depth, std::list<std::string>& bin_name) const
    {
      std::shared_lock < std::shared_timed_mutex > lock (read_write_mutex_);
      bin_name.clear ();
#if defined _MSC_VER
  #pragma warning(push)
  #pragma warning(disable : 4267)
#endif
      root_node_->queryBBIntersects (min, max, query_depth, bin_name);
#if defined _MSC_VER
  #pragma warning(pop)
#endif
    }

    ////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT> void
    OutofcoreOctreeBase<ContainerT, PointT>::writeVPythonVisual (const boost::filesystem::path& filename)
    {
      std::ofstream f (filename.c_str ());

      f << "from visual import *\n\n";

      root_node_->writeVPythonVisual (f);
    }

    ////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT> void
    OutofcoreOctreeBase<ContainerT, PointT>::flushToDisk ()
    {
      root_node_->flushToDisk ();
    }

    ////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT> void
    OutofcoreOctreeBase<ContainerT, PointT>::flushToDiskLazy ()
    {
      root_node_->flushToDiskLazy ();
    }

    ////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT> void
    OutofcoreOctreeBase<ContainerT, PointT>::convertToXYZ ()
    {
      saveToFile ();
      root_node_->convertToXYZ ();
    }

    ////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT> void
    OutofcoreOctreeBase<ContainerT, PointT>::DeAllocEmptyNodeCache ()
    {
      DeAllocEmptyNodeCache (root_node_);
    }

    ////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT> void
    OutofcoreOctreeBase<ContainerT, PointT>::DeAllocEmptyNodeCache (OutofcoreOctreeBaseNode<ContainerT, PointT>* current)
    {
      if (current == nullptr)
        current = root_node_;

      if (current->size () == 0)
      {
        current->flush_DeAlloc_this_only ();
      }

      for (int i = 0; i < current->numchildren (); i++)
      {
        DeAllocEmptyNodeCache (current->children[i]);
      }
    }

    ////////////////////////////////////////////////////////////////////////////////
    template<typename ContainerT, typename PointT> OutofcoreOctreeBaseNode<ContainerT, PointT>*
    OutofcoreOctreeBase<ContainerT, PointT>::getBranchChildPtr (const BranchNode& branch_arg, unsigned char childIdx_arg) const
    {
      return (branch_arg.getChildPtr (childIdx_arg));
    }      

    ////////////////////////////////////////////////////////////////////////////////
    template<typename ContainerT, typename PointT> pcl::Filter<pcl::PCLPointCloud2>::Ptr
    OutofcoreOctreeBase<ContainerT, PointT>::getLODFilter ()
    {
      return (lod_filter_ptr_);
    }

    ////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT> const pcl::Filter<pcl::PCLPointCloud2>::ConstPtr
    OutofcoreOctreeBase<ContainerT, PointT>::getLODFilter () const
    {
      return (lod_filter_ptr_);
    }

    ////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT> void
    OutofcoreOctreeBase<ContainerT, PointT>::setLODFilter (const pcl::Filter<pcl::PCLPointCloud2>::Ptr& filter_arg)
    {
      lod_filter_ptr_ = std::static_pointer_cast<decltype(lod_filter_ptr_)>(filter_arg);
    }

    ////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT> bool
    OutofcoreOctreeBase<ContainerT, PointT>::getBinDimension (double& x, double& y) const
    {
      if (root_node_== nullptr)
      {
        x = 0;
        y = 0;
        return (false);
      }

      Eigen::Vector3d min, max;
      this->getBoundingBox (min, max);
      
      double depth = static_cast<double> (metadata_->getDepth ());
      Eigen::Vector3d diff = max-min;

      y = diff[1] * pow (.5, depth);
      x = diff[0] * pow (.5, depth);

      return (true);
    }

    ////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT> double
    OutofcoreOctreeBase<ContainerT, PointT>::getVoxelSideLength (const std::uint64_t& depth) const
    {
      Eigen::Vector3d min, max;
      this->getBoundingBox (min, max);
      double result = (max[0] - min[0]) * pow (.5, static_cast<double> (metadata_->getDepth ())) * static_cast<double> (1 << (metadata_->getDepth () - depth));
      return (result);
    }

    ////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT> void
    OutofcoreOctreeBase<ContainerT, PointT>::buildLOD ()
    {
      if (root_node_== nullptr)
      {
        PCL_ERROR ("Root node is null; aborting buildLOD.\n");
        return;
      }

      std::unique_lock < std::shared_timed_mutex > lock (read_write_mutex_);

      const int number_of_nodes = 1;

      std::vector<BranchNode*> current_branch (number_of_nodes, static_cast<BranchNode*>(nullptr));
      current_branch[0] = root_node_;
      assert (current_branch.back () != nullptr);
      this->buildLODRecursive (current_branch);
    }

    ////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT> void
    OutofcoreOctreeBase<ContainerT, PointT>::printBoundingBox (OutofcoreOctreeBaseNode<ContainerT, PointT>& node) const
    {
      Eigen::Vector3d min, max;
      node.getBoundingBox (min,max);
      PCL_INFO ("[pcl::outofcore::OutofcoreOctreeBase::%s] min(%lf,%lf,%lf), max(%lf,%lf,%lf)\n", __FUNCTION__, min[0], min[1], min[2], max[0], max[1], max[2]);      
    }
    

    ////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT> void
    OutofcoreOctreeBase<ContainerT, PointT>::buildLODRecursive (const std::vector<BranchNode*>& current_branch)
    {
      PCL_DEBUG ("%s Building LOD at depth %d",__PRETTY_FUNCTION__, current_branch.size ());
      
      if (!current_branch.back ())
      {
        return;
      }
      
      if (current_branch.back ()->getNodeType () == pcl::octree::LEAF_NODE)
      {
        assert (current_branch.back ()->getDepth () == this->getDepth ());
        
        BranchNode* leaf = current_branch.back ();

        pcl::PCLPointCloud2::Ptr leaf_input_cloud (new pcl::PCLPointCloud2 ());
        //read the data from the PCD file associated with the leaf; it is full resolution
        leaf->read (leaf_input_cloud);
        assert (leaf_input_cloud->width*leaf_input_cloud->height > 0);
        
        //go up the tree, re-downsampling the full resolution leaf cloud at lower and lower resolution
        for (auto level = static_cast<std::int64_t>(current_branch.size ()-1); level >= 1; level--)
        {
          BranchNode* target_parent = current_branch[level-1];
          assert (target_parent != nullptr);
          double exponent = static_cast<double>(current_branch.size () - target_parent->getDepth ());
          double current_depth_sample_percent = pow (sample_percent_, exponent);

          assert (current_depth_sample_percent > 0.0);
          //------------------------------------------------------------
          //subsample data:
          //   1. Get indices from a random sample
          //   2. Extract those indices with the extract indices class (in order to also get the complement)
          //------------------------------------------------------------

          lod_filter_ptr_->setInputCloud (leaf_input_cloud);

          //set sample size to 1/8 of total points (12.5%)
          auto sample_size = static_cast<std::uint64_t> (static_cast<double> (leaf_input_cloud->width*leaf_input_cloud->height) * current_depth_sample_percent);

          if (sample_size == 0)
            sample_size = 1;
          
          lod_filter_ptr_->setSample (static_cast<unsigned int>(sample_size));
      
          //create our destination
          pcl::PCLPointCloud2::Ptr downsampled_cloud (new pcl::PCLPointCloud2 ());

          //create destination for indices
          pcl::IndicesPtr downsampled_cloud_indices (new pcl::Indices ());
          lod_filter_ptr_->filter (*downsampled_cloud_indices);

          //extract the "random subset", size by setSampleSize
          pcl::ExtractIndices<pcl::PCLPointCloud2> extractor;
          extractor.setInputCloud (leaf_input_cloud);
          extractor.setIndices (downsampled_cloud_indices);
          extractor.filter (*downsampled_cloud);

          //write to the target
          if (downsampled_cloud->width*downsampled_cloud->height > 0)
          {
            target_parent->payload_->insertRange (downsampled_cloud);
            this->incrementPointsInLOD (target_parent->getDepth (), downsampled_cloud->width*downsampled_cloud->height);
          }
        }
      }
      else//not at leaf, keep going down
      {
        //clear this node while walking down the tree in case we are updating the LOD
        current_branch.back ()->clearData ();
        
        std::vector<BranchNode*> next_branch (current_branch);

        if (current_branch.back ()->hasUnloadedChildren ())
        {
          current_branch.back ()->loadChildren (false);
        }

        for (std::size_t i = 0; i < 8; i++)
        {
          next_branch.push_back (current_branch.back ()->getChildPtr (i));
          //skip that child if it doesn't exist
          if (next_branch.back () != nullptr)
            buildLODRecursive (next_branch);
          
          next_branch.pop_back ();
        }
      }
    }
    ////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT> void
    OutofcoreOctreeBase<ContainerT, PointT>::incrementPointsInLOD (std::uint64_t depth, std::uint64_t new_point_count)
    {
      if (std::numeric_limits<std::uint64_t>::max () - metadata_->getLODPoints (depth) < new_point_count)
      {
        PCL_ERROR ("[pcl::outofcore::OutofcoreOctreeBase::incrementPointsInLOD] Overflow error. Too many points in depth %d of outofcore octree with root at %s\n", depth, metadata_->getMetadataFilename().c_str());
        PCL_THROW_EXCEPTION (PCLException, "Overflow error");
      }
        
      metadata_->setLODPoints (depth, new_point_count, true /*true->increment*/);
    }

    ////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT> bool
    OutofcoreOctreeBase<ContainerT, PointT>::checkExtension (const boost::filesystem::path& path_name)
    {
      if (boost::filesystem::extension (path_name) != OutofcoreOctreeBaseNode<ContainerT, PointT>::node_index_extension)
      {
        PCL_ERROR ( "[pcl::outofcore::OutofcoreOctreeBase] Wrong root node file extension: %s. The tree must have a root node ending in %s\n", boost::filesystem::extension (path_name).c_str (), OutofcoreOctreeBaseNode<ContainerT, PointT>::node_index_extension.c_str () );
        return (false);
      }

      return (true);
    }

    ////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT> void
    OutofcoreOctreeBase<ContainerT, PointT>::enlargeToCube (Eigen::Vector3d& bb_min, Eigen::Vector3d& bb_max)
    {
      Eigen::Vector3d diff = bb_max - bb_min;
      assert (diff[0] > 0);
      assert (diff[1] > 0);
      assert (diff[2] > 0);
      Eigen::Vector3d center = (bb_max + bb_min)/2.0;

      double max_sidelength = std::max (std::max (std::abs (diff[0]), std::abs (diff[1])), std::abs (diff[2]));
      assert (max_sidelength > 0);
      bb_min = center - Eigen::Vector3d (1.0, 1.0, 1.0)*(max_sidelength/2.0);
      bb_max = center + Eigen::Vector3d (1.0, 1.0, 1.0)*(max_sidelength/2.0);
    }

    ////////////////////////////////////////////////////////////////////////////////    

    template<typename ContainerT, typename PointT> std::uint64_t
    OutofcoreOctreeBase<ContainerT, PointT>::calculateDepth (const Eigen::Vector3d& min_bb, const Eigen::Vector3d& max_bb, const double leaf_resolution)
    {
      //Assume cube
      double side_length = max_bb[0] - min_bb[0];

      if (side_length < leaf_resolution)
          return (0);
          
      auto res = static_cast<std::uint64_t> (std::ceil (std::log2 (side_length / leaf_resolution)));
      
      PCL_DEBUG ("[pcl::outofcore::OutofcoreOctreeBase::calculateDepth] Setting depth to %d\n",res);
      return (res);
    }
  }//namespace outofcore
}//namespace pcl

#endif //PCL_OUTOFCORE_OCTREE_BASE_IMPL_H_
