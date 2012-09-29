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
 *  $Id: octree_base.hpp 6927M 2012-08-24 20:22:36Z (local) $
 */

#ifndef PCL_OUTOFCORE_OCTREE_BASE_IMPL_H_
#define PCL_OUTOFCORE_OCTREE_BASE_IMPL_H_


#include <pcl/outofcore/octree_base.h>

// JSON
#include <pcl/outofcore/cJSON.h>

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

    //Static constants
    template<typename ContainerT, typename PointT>    
    const std::string OutofcoreOctreeBase<ContainerT, PointT>::TREE_EXTENSION_ = ".octree";

    template<typename ContainerT, typename PointT>
    const int OutofcoreOctreeBase<ContainerT, PointT>::OUTOFCORE_VERSION_ = static_cast<int>(3);


// Constructors
// ---------------------------------------------------------------------------
    template<typename ContainerT, typename PointT>
    OutofcoreOctreeBase<ContainerT, PointT>::OutofcoreOctreeBase (const boost::filesystem::path& root_name, const bool load_all)
      : root_node_ ()
      , read_write_mutex_ ()
      , metadata_ (new OutofcoreOctreeBaseMetadata ())
    {

      if (!this->checkExtension (root_name))
      {
        PCL_THROW_EXCEPTION (PCLException, "[pcl::outofcore::OutofcoreOctreeBase] Bad extension. Outofcore Octrees must have a root node ending in .oct_idx\n");
      }
      
      // Create root_node_node
      root_node_ = new OutofcoreOctreeBaseNode<ContainerT, PointT> (root_name, NULL, load_all);
      // Set root_node_nodes tree to the newly created tree
      root_node_->m_tree_ = this;

      // Set root_node_nodes file path
      boost::filesystem::path treepath = root_name.parent_path () / (boost::filesystem::basename (root_name) + TREE_EXTENSION_);

      //Load the JSON metadata
      metadata_->loadMetadataFromDisk (treepath);
    }
////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT>
    OutofcoreOctreeBase<ContainerT, PointT>::OutofcoreOctreeBase (const Eigen::Vector3d& min, const Eigen::Vector3d& max, const double resolution_arg, const boost::filesystem::path& root_name, const std::string& coord_sys)
      : root_node_()
      , read_write_mutex_ ()
      , metadata_ (new OutofcoreOctreeBaseMetadata ())
    {
      Eigen::Vector3d tmp_min = min;
      Eigen::Vector3d tmp_max = max;
      this->enlargeToCube (tmp_min, tmp_max);

      //Compute the depth of the tree given the resolution
      boost::uint64_t depth = this->calculateDepth (tmp_min, tmp_max, resolution_arg);

      this->init (depth, tmp_min, tmp_max, root_name, coord_sys);
    }
////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT>
    OutofcoreOctreeBase<ContainerT, PointT>::OutofcoreOctreeBase (const boost::uint64_t max_depth, const Eigen::Vector3d& min, const Eigen::Vector3d& max, const boost::filesystem::path& root_name, const std::string& coord_sys)
      : root_node_()
      , read_write_mutex_ ()
      , metadata_ (new OutofcoreOctreeBaseMetadata ())
    {
      this->init (max_depth, min, max, root_name, coord_sys);
    }

////////////////////////////////////////////////////////////////////////////////
    template<typename ContainerT, typename PointT> void
    OutofcoreOctreeBase<ContainerT, PointT>::init (const uint64_t& depth, const Eigen::Vector3d& min, const Eigen::Vector3d& max, const boost::filesystem::path& root_name, const std::string& coord_sys)
    {
      if (!this->checkExtension (root_name))
      {
        PCL_THROW_EXCEPTION (PCLException, "[pcl::outofcore::OutofcoreOctreeBase] Bad extension. Outofcore Octrees must have a root node ending in .oct_idx\n");
      }

      //Check to make sure that we are not overwriting existing data
      ///\todo allow multiple outofcore octrees in same directory structure: 
      //     - requires changing the book keeping of children (i.e. can't just check which child directories exist) 
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
      root_node_= new OutofcoreOctreeBaseNode<ContainerT, PointT> (depth, tmp_min, tmp_max, this, root_name);
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

    template<typename ContainerT, typename PointT> boost::uint64_t
    OutofcoreOctreeBase<ContainerT, PointT>::addDataToLeaf (const AlignedPointTVector& p)
    {
      boost::unique_lock < boost::shared_mutex > lock (read_write_mutex_);

      const bool _FORCE_BB_CHECK = true;
      
      uint64_t pt_added = root_node_->addDataToLeaf (p, _FORCE_BB_CHECK);

      assert (p.size () == pt_added);

      return (pt_added);
    }
////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT> boost::uint64_t
    OutofcoreOctreeBase<ContainerT, PointT>::addPointCloud (PointCloudConstPtr point_cloud)
    {
      return (addDataToLeaf (point_cloud->points));
    }
    
////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT> boost::uint64_t
    OutofcoreOctreeBase<ContainerT, PointT>::addPointCloud (sensor_msgs::PointCloud2::Ptr &input_cloud, const bool skip_bb_check = false)
    {
      uint64_t pt_added = this->root_node_->addPointCloud (input_cloud, skip_bb_check) ;
      assert (input_cloud->width*input_cloud->height == pt_added);
      return (pt_added);
    }

    
////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT> boost::uint64_t
    OutofcoreOctreeBase<ContainerT, PointT>::addPointCloud_and_genLOD (PointCloudConstPtr point_cloud)
    {
      // Lock the tree while writing
      boost::unique_lock < boost::shared_mutex > lock (read_write_mutex_);
      boost::uint64_t pt_added = root_node_->addDataToLeaf_and_genLOD (point_cloud->points, false);
      return (pt_added);
    }

////////////////////////////////////////////////////////////////////////////////
    template<typename ContainerT, typename PointT> boost::uint64_t
    OutofcoreOctreeBase<ContainerT, PointT>::addPointCloud_and_genLOD (sensor_msgs::PointCloud2::Ptr &input_cloud)
    {
      // Lock the tree while writing
      boost::unique_lock < boost::shared_mutex > lock (read_write_mutex_);
      boost::uint64_t pt_added = root_node_->addPointCloud_and_genLOD (input_cloud);
      
//      PCL_INFO ("[pcl::outofcore::OutofcoreOctreeBase::%s] Points added %lu, points in input cloud, %lu\n",__FUNCTION__, pt_added, input_cloud->width*input_cloud->height );
 
      assert ( input_cloud->width*input_cloud->height == pt_added );

      return (pt_added);
    }



////////////////////////////////////////////////////////////////////////////////
#if 0
    template<typename ContainerT, typename PointT> void
    OutofcoreOctreeBase<ContainerT, PointT>::addPointToLeaf (const PointT& src)
    {
      boost::unique_lock < boost::shared_mutex > lock (read_write_mutex_);
      root_node_->addPointToLeaf (src, false);
    }
#endif    

////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT> boost::uint64_t
    OutofcoreOctreeBase<ContainerT, PointT>::addDataToLeaf_and_genLOD (AlignedPointTVector& src)
    {
      // Lock the tree while writing
      boost::unique_lock < boost::shared_mutex > lock (read_write_mutex_);
      boost::uint64_t pt_added = root_node_->addDataToLeaf_and_genLOD (src, false);
      return (pt_added);
    }
////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT> void
    OutofcoreOctreeBase<ContainerT, PointT>::queryBBIncludes (const Eigen::Vector3d& min, const Eigen::Vector3d& max, const size_t query_depth, AlignedPointTVector& dst) const
    {
      boost::shared_lock < boost::shared_mutex > lock (read_write_mutex_);
      dst.clear ();
      PCL_DEBUG ("[pcl::outofcore::OutofcoreOctreeBaseNode] Querying Bounding Box %.2lf %.2lf %.2lf, %.2lf %.2lf %.2lf", min[0], min[1], min[2], max[0], max[1], max[2]);
      root_node_->queryBBIncludes (min, max, query_depth, dst);
    }

////////////////////////////////////////////////////////////////////////////////
    template<typename ContainerT, typename PointT> void
    OutofcoreOctreeBase<ContainerT, PointT>::queryBBIncludes (const Eigen::Vector3d& min, const Eigen::Vector3d& max, const int query_depth, const sensor_msgs::PointCloud2::Ptr& dst_blob) const
    {
      boost::shared_lock < boost::shared_mutex > lock (read_write_mutex_);

      dst_blob->data.clear ();
      dst_blob->width = 0;
      dst_blob->height =1;

      root_node_->queryBBIncludes ( min, max, query_depth, dst_blob );
    }

////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT> void
    OutofcoreOctreeBase<ContainerT, PointT>::queryBBIncludes_subsample (const Eigen::Vector3d& min, const Eigen::Vector3d& max, const size_t query_depth, const double percent, AlignedPointTVector& dst) const
    {
      boost::shared_lock < boost::shared_mutex > lock (read_write_mutex_);
      dst.clear ();
      root_node_->queryBBIncludes_subsample (min, max, query_depth, percent, dst);
    }

////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT> bool
    OutofcoreOctreeBase<ContainerT, PointT>::getBoundingBox (Eigen::Vector3d &min, Eigen::Vector3d &max) const
    {
      if (root_node_!= NULL)
      {
        root_node_->getBoundingBox (min, max);
        return true;
      }
      return false;
    }

////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT> void
    OutofcoreOctreeBase<ContainerT, PointT>::printBoundingBox(const size_t query_depth) const
    {
      boost::shared_lock < boost::shared_mutex > lock (read_write_mutex_);
      root_node_->printBoundingBox (query_depth);
    }

////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT> void
    OutofcoreOctreeBase<ContainerT, PointT>::getOccupiedVoxelCenters(AlignedPointTVector &voxel_centers, const size_t query_depth) const
    {
      boost::shared_lock < boost::shared_mutex > lock (read_write_mutex_);
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
    OutofcoreOctreeBase<ContainerT, PointT>::getOccupiedVoxelCenters(std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > &voxel_centers, const size_t query_depth) const
    {
      boost::shared_lock < boost::shared_mutex > lock (read_write_mutex_);
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
    OutofcoreOctreeBase<ContainerT, PointT>::queryBBIntersects (const Eigen::Vector3d& min, const Eigen::Vector3d& max, const boost::uint32_t query_depth, std::list<std::string>& bin_name) const
    {
      boost::shared_lock < boost::shared_mutex > lock (read_write_mutex_);
      bin_name.clear ();
#pragma warning(push)
#pragma warning(disable : 4267)
      root_node_->queryBBIntersects (min, max, query_depth, bin_name);
#pragma warning(pop)
    }
////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT> void
    OutofcoreOctreeBase<ContainerT, PointT>::writeVPythonVisual (const boost::filesystem::path filename)
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
    OutofcoreOctreeBase<ContainerT, PointT>::saveIdx ()
    {
      root_node_->saveIdx (true);
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

    template<typename ContainerT, typename PointT> bool
    OutofcoreOctreeBase<ContainerT, PointT>::getBinDimension (double& x, double& y) const
    {
      if (root_node_== NULL)
      {
        x = 0;
        y = 0;
        return (false);
      }

      Eigen::Vector3d min, max;
      this->getBoundingBox (min, max);
      
      double depth = static_cast<double> (metadata_->getDepth ());
      Eigen::Vector3d diff = max-min;

      y = diff[1] * pow (.5, double (depth));
      x = diff[0] * pow (.5, double (depth));

      return (true);
    }

////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT> double
    OutofcoreOctreeBase<ContainerT, PointT>::getVoxelSideLength (const boost::uint64_t depth) const
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
      if (root_node_== NULL)
      {
        PCL_ERROR ("Root node is null; aborting buildLOD.\n");
        return;
      }
      boost::unique_lock < boost::shared_mutex > lock (read_write_mutex_);

      const int current_dims = 1;
      OutofcoreOctreeBaseNode<ContainerT, PointT>* current_branch[current_dims] = {root_node_};
      this->buildLODRecursive (current_branch, current_dims);
    }
////////////////////////////////////////////////////////////////////////////////
    template<typename ContainerT, typename PointT> void
    OutofcoreOctreeBase<ContainerT, PointT>::printBoundingBox (OutofcoreNodeType& node) const
    {
      Eigen::Vector3d min, max;
      node.getBoundingBox (min,max);
      PCL_INFO ("[pcl::outofcore::OutofcoreOctreeBase::%s] min(%lf,%lf,%lf), max(%lf,%lf,%lf)\n", __FUNCTION__, min[0], min[1], min[2], max[0], max[1], max[2]);      
    }
    

////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT> void
    OutofcoreOctreeBase<ContainerT, PointT>::buildLODRecursive (OutofcoreOctreeBaseNode<ContainerT, PointT>** current_branch, const int current_dims)
    {
      //stop if this brach does not exist
      if (!current_branch[current_dims - 1])
      {
        return;
      }

      if ((current_branch[current_dims - 1]->getNumChildren () == 0)
          && (!current_branch[current_dims - 1]->hasUnloadedChildren ()))//at leaf: subsample, remove, and copy to higher nodes
      {
        //this node's idx is (current_dims-1)
        OutofcoreOctreeBaseNode<ContainerT, PointT>* leaf = current_branch[current_dims - 1];

        boost::uint64_t leaf_start_size = leaf->payload_->size ();
        if (leaf_start_size > 0)//skip empty
        {
          for (boost::uint64_t startp = 0; startp < leaf_start_size; startp += LOAD_COUNT_)
          {
            //there are (current_dims-1) nodes above this one, indexed 0 thru (current_dims-2)
            for (size_t level = (current_dims - 1); level >= 1; level--)
            {
              //the target to copy points into
              OutofcoreOctreeBaseNode<ContainerT, PointT>* target_parent = current_branch[level - 1];

              //the percent to copy
              //each level up the chain gets sample_precent^l of the leaf's data
              double percent = pow (double (OutofcoreOctreeBaseNode<ContainerT, PointT>::sample_precent), double (current_dims - level));

              //read in percent of node
              sensor_msgs::PointCloud2::Ptr tmp_blob (new sensor_msgs::PointCloud2 ());
              sensor_msgs::PointCloud2::Ptr subsampled_blob (new sensor_msgs::PointCloud2 ());
              leaf->read (tmp_blob);
              //subsample the blob
              
              //write to the target
              
              PCL_THROW_EXCEPTION (PCLException, "Not implemented\n");

#if 0              
              if (!v.empty ())
              {
                target_parent->payload_->insertRange ( v );
//                target_parent->payload->insertRange (&(v.front ()), v.size ());
                this->incrementPointsInLOD (target_parent->depth_, v.size ());
              }
#endif
            }
          }
        }
      }
      else//not at leaf, keep going down
      {
        //clear this node, in case we are updating the LOD
        current_branch[current_dims - 1]->payload_->clear ();

        const int next_dims = current_dims + 1;
        OutofcoreOctreeBaseNode<ContainerT, PointT>** next_branch = new OutofcoreOctreeBaseNode<ContainerT, PointT>*[next_dims];
        memcpy (next_branch, current_branch, current_dims * sizeof(OutofcoreOctreeBaseNode<ContainerT, PointT>**));

        size_t numchild = current_branch[current_dims - 1]->getNumChildren ();
        if ((numchild != 8) && (current_branch[current_dims - 1]->hasUnloadedChildren ()))
        {
          current_branch[current_dims - 1]->loadChildren (false);
          numchild = current_branch[current_dims - 1]->getNumChildren ();
        }

        for (size_t i = 0; i < numchild; i++)
        {
          next_branch[next_dims - 1] = next_branch[current_dims - 1]->children_[i];
          buildLODRecursive (next_branch, next_dims);
        }

        delete[] next_branch;
      }
    }

    ////////////////////////////////////////////////////////////////////////////////
    template<typename ContainerT, typename PointT> void
    OutofcoreOctreeBase<ContainerT, PointT>::incrementPointsInLOD (boost::uint64_t depth, boost::uint64_t new_point_count)
    {
      if (std::numeric_limits<uint64_t>::max () - metadata_->getLODPoints (depth) < new_point_count)
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
        return (0);
      }

      return (1);
    }

    ////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT> void
    OutofcoreOctreeBase<ContainerT, PointT>::enlargeToCube (Eigen::Vector3d& bb_min, Eigen::Vector3d& bb_max)
    {
      Eigen::Vector3d diff = bb_max - bb_min;
          
      // X is largest, increase y/z in both +/- directions
      if (diff[0] > diff[1] && diff[0] > diff[2])
      {
        bb_min[1] -= (diff[0] - diff[1])/2.0;
        bb_max[1] += (diff[0] - diff[1])/2.0;
        bb_min[2] -= (diff[0] - diff[2])/2.0;
        bb_max[2] += (diff[0] - diff[2])/2.0;
        // Y is largest, increase y/z in both +/- directions
      }
      else if (diff[1] > diff[0] && diff[1] > diff[2])
      {
        bb_min[0] -= (diff[1] - diff[0])/2.0;
        bb_max[0] += (diff[1] - diff[0])/2.0;
        bb_min[2] -= (diff[1] - diff[2])/2.0;
        bb_max[2] += (diff[1] - diff[2])/2.0;
        // Z is largest, increase y/z in both +/- directions
      }
      else if (diff[2] > diff[0] && diff[2] > diff[1])
      {
        bb_min[0] -= (diff[2] - diff[0])/2.0;
        bb_max[0] += (diff[2] - diff[0])/2.0;
        bb_min[1] -= (diff[2] - diff[1])/2.0;
        bb_max[1] += (diff[2] - diff[1])/2.0;
      }
    }
    ////////////////////////////////////////////////////////////////////////////////    

    template<typename ContainerT, typename PointT> boost::uint64_t
    OutofcoreOctreeBase<ContainerT, PointT>::calculateDepth (const Eigen::Vector3d& min_bb, const Eigen::Vector3d& max_bb, const double leaf_resolution)
    {
      //Assume cube
      double side_length = max_bb[0] - min_bb[0];

      if (side_length < leaf_resolution)
          return (0);
          
      boost::uint64_t res = static_cast<boost::uint64_t>(std::ceil (log2 (side_length/leaf_resolution)));
      
      PCL_DEBUG ("[pcl::outofcore::OutofcoreOctreeBase::calculateDepth] Setting depth to %d\n",res);
      return (res);
    }
    
  }//namespace outofcore
}//namespace pcl

#endif //PCL_OUTOFCORE_OCTREE_BASE_IMPL_H_
