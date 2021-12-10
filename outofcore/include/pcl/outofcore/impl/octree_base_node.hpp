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
 *
 */

#ifndef PCL_OUTOFCORE_OCTREE_BASE_NODE_IMPL_H_
#define PCL_OUTOFCORE_OCTREE_BASE_NODE_IMPL_H_

// C++
#include <iostream>
#include <fstream>
#include <random>
#include <sstream>
#include <string>
#include <exception>

#include <pcl/common/common.h>
#include <pcl/common/utils.h> // pcl::utils::ignore
#include <pcl/visualization/common/common.h>
#include <pcl/outofcore/octree_base_node.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/extract_indices.h>

// JSON
#include <pcl/outofcore/cJSON.h>

namespace pcl
{
  namespace outofcore
  {
    
    template<typename ContainerT, typename PointT>
    const std::string OutofcoreOctreeBaseNode<ContainerT, PointT>::node_index_basename = "node";

    template<typename ContainerT, typename PointT>
    const std::string OutofcoreOctreeBaseNode<ContainerT, PointT>::node_container_basename = "node";

    template<typename ContainerT, typename PointT>
    const std::string OutofcoreOctreeBaseNode<ContainerT, PointT>::node_index_extension = ".oct_idx";

    template<typename ContainerT, typename PointT>
    const std::string OutofcoreOctreeBaseNode<ContainerT, PointT>::node_container_extension = ".oct_dat";

    template<typename ContainerT, typename PointT>
    std::mutex OutofcoreOctreeBaseNode<ContainerT, PointT>::rng_mutex_;

    template<typename ContainerT, typename PointT>
    std::mt19937 OutofcoreOctreeBaseNode<ContainerT, PointT>::rng_;

    template<typename ContainerT, typename PointT>
    const double OutofcoreOctreeBaseNode<ContainerT, PointT>::sample_percent_ = .125;

    template<typename ContainerT, typename PointT>
    const std::string OutofcoreOctreeBaseNode<ContainerT, PointT>::pcd_extension = ".pcd";

    template<typename ContainerT, typename PointT>
    OutofcoreOctreeBaseNode<ContainerT, PointT>::OutofcoreOctreeBaseNode ()
      : m_tree_ ()
      , root_node_ (NULL)
      , parent_ (NULL)
      , depth_ (0)
      , children_ (8, nullptr)
      , num_children_ (0)
      , num_loaded_children_ (0)
      , payload_ ()
      , node_metadata_ (new OutofcoreOctreeNodeMetadata)
    {
      node_metadata_->setOutofcoreVersion (3);
    }

    ////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT>
    OutofcoreOctreeBaseNode<ContainerT, PointT>::OutofcoreOctreeBaseNode (const boost::filesystem::path& directory_path, OutofcoreOctreeBaseNode<ContainerT, PointT>* super, bool load_all)
      : m_tree_ ()
      , root_node_ ()
      , parent_ (super)
      , depth_ ()
      , children_ (8, nullptr)
      , num_children_ (0)
      , num_loaded_children_ (0)
      , payload_ ()
      , node_metadata_ (new OutofcoreOctreeNodeMetadata)
    {
      node_metadata_->setOutofcoreVersion (3);

      //Check if this is the first node created/loaded (this is true if super, i.e. node's parent is NULL)
      if (super == nullptr)
      {
        node_metadata_->setDirectoryPathname (directory_path.parent_path ());
        node_metadata_->setMetadataFilename (directory_path);
        depth_ = 0;
        root_node_ = this;

        //Check if the specified directory to load currently exists; if not, don't continue
        if (!boost::filesystem::exists (node_metadata_->getDirectoryPathname ()))
        {
          PCL_ERROR ("[pcl::outofcore::OutofcoreOctreeBaseNode] Could not find dir %s\n", node_metadata_->getDirectoryPathname ().c_str ());
          PCL_THROW_EXCEPTION (PCLException, "[pcl::outofcore::OutofcoreOctreeBaseNode] Outofcore Exception: missing directory");
        }
      }
      else
      {
        node_metadata_->setDirectoryPathname (directory_path);
        depth_ = super->getDepth () + 1;
        root_node_ = super->root_node_;

        boost::filesystem::directory_iterator directory_it_end; //empty constructor creates end of iterator

        //flag to test if the desired metadata file was found
        bool b_loaded = false;

        for (boost::filesystem::directory_iterator directory_it (node_metadata_->getDirectoryPathname ()); directory_it != directory_it_end; ++directory_it)
        {
          const boost::filesystem::path& file = *directory_it;

          if (!boost::filesystem::is_directory (file))
          {
            if (boost::filesystem::extension (file) == node_index_extension)
            {
              b_loaded = node_metadata_->loadMetadataFromDisk (file);
              break;
            }
          }
        }

        if (!b_loaded)
        {
          PCL_ERROR ("[pcl::outofcore::OutofcoreOctreeBaseNode] Could not find index\n");
          PCL_THROW_EXCEPTION (PCLException, "[pcl::outofcore::OutofcoreOctreeBaseNode] Outofcore: Could not find node index");
        }
      }
      
      //load the metadata
      loadFromFile (node_metadata_->getMetadataFilename (), super);

      //set the number of children in this node
      num_children_ = this->countNumChildren ();

      if (load_all)
      {
        loadChildren (true);
      }
    }
//////////////////////////////////////////////////////////////////////////////// 

    template<typename ContainerT, typename PointT>
    OutofcoreOctreeBaseNode<ContainerT, PointT>::OutofcoreOctreeBaseNode (const Eigen::Vector3d& bb_min, const Eigen::Vector3d& bb_max, OutofcoreOctreeBase<ContainerT, PointT> * const tree, const boost::filesystem::path& root_name)
      : m_tree_ (tree)
      , root_node_ ()
      , parent_ ()
      , depth_ ()
      , children_ (8, nullptr)
      , num_children_ (0)
      , num_loaded_children_ (0)
      , payload_ ()
      , node_metadata_ (new OutofcoreOctreeNodeMetadata ())
    {
      assert (tree != NULL);
      node_metadata_->setOutofcoreVersion (3);
      init_root_node (bb_min, bb_max, tree, root_name);
    }

    ////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT> void
    OutofcoreOctreeBaseNode<ContainerT, PointT>::init_root_node (const Eigen::Vector3d& bb_min, const Eigen::Vector3d& bb_max, OutofcoreOctreeBase<ContainerT, PointT> * const tree, const boost::filesystem::path& root_name)
    {
      assert (tree != NULL);

      parent_ = nullptr;
      root_node_ = this;
      m_tree_ = tree;
      depth_ = 0;

      //Mark the children as unallocated
      num_children_ = 0;

      Eigen::Vector3d tmp_max = bb_max;
      Eigen::Vector3d tmp_min = bb_min;

      // Need to make the bounding box slightly bigger so points that fall on the max side aren't excluded
      double epsilon = 1e-8;
      tmp_max += epsilon*Eigen::Vector3d (1.0, 1.0, 1.0);

      node_metadata_->setBoundingBox (tmp_min, tmp_max);
      node_metadata_->setDirectoryPathname (root_name.parent_path ());
      node_metadata_->setOutofcoreVersion (3);

      // If the root directory doesn't exist create it
      if (!boost::filesystem::exists (node_metadata_->getDirectoryPathname ()))
      {
        boost::filesystem::create_directory (node_metadata_->getDirectoryPathname ());
      }
      // If the root directory is a file, do not continue
      else if (!boost::filesystem::is_directory (node_metadata_->getDirectoryPathname ()))
      {
        PCL_ERROR ("[pcl::outofcore::OutofcoreOctreeBaseNode] Need empty directory structure. Dir %s exists and is a file.\n",node_metadata_->getDirectoryPathname ().c_str ());
        PCL_THROW_EXCEPTION (PCLException, "[pcl::outofcore::OutofcoreOctreeBaseNode] Bad Path: Directory Already Exists");
      }

      // Create a unique id for node file name
      std::string uuid;
      
      OutofcoreOctreeDiskContainer<PointT>::getRandomUUIDString (uuid);

      std::string node_container_name;

      node_container_name = uuid + std::string ("_") + node_container_basename + pcd_extension;

      node_metadata_->setMetadataFilename (node_metadata_->getDirectoryPathname () / root_name.filename ());
      node_metadata_->setPCDFilename (node_metadata_->getDirectoryPathname () / boost::filesystem::path (node_container_name));

      boost::filesystem::create_directory (node_metadata_->getDirectoryPathname ());
      node_metadata_->serializeMetadataToDisk ();

      // Create data container, ie octree_disk_container, octree_ram_container
      payload_.reset (new ContainerT (node_metadata_->getPCDFilename ()));
    }

    ////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT>
    OutofcoreOctreeBaseNode<ContainerT, PointT>::~OutofcoreOctreeBaseNode ()
    {
      // Recursively delete all children and this nodes data
      recFreeChildren ();
    }

    ////////////////////////////////////////////////////////////////////////////////
    
    template<typename ContainerT, typename PointT> std::size_t
    OutofcoreOctreeBaseNode<ContainerT, PointT>::countNumChildren () const
    {
      std::size_t child_count = 0;
      
      for(std::size_t i=0; i<8; i++)
      {
        boost::filesystem::path child_path = this->node_metadata_->getDirectoryPathname () / boost::filesystem::path (std::to_string(i));
        if (boost::filesystem::exists (child_path))
          child_count++;
      }
      return (child_count);
    }

    ////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT> void
    OutofcoreOctreeBaseNode<ContainerT, PointT>::saveIdx (bool recursive)
    {
      node_metadata_->serializeMetadataToDisk ();

      if (recursive)
      {
        for (std::size_t i = 0; i < 8; i++)
        {
          if (children_[i])
            children_[i]->saveIdx (true);
        }
      }
    }

    ////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT> bool
    OutofcoreOctreeBaseNode<ContainerT, PointT>::hasUnloadedChildren () const
    {
      return (this->getNumLoadedChildren () < this->getNumChildren ());
    }
    ////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT> void
    OutofcoreOctreeBaseNode<ContainerT, PointT>::loadChildren (bool recursive)
    {
      //if we have fewer children loaded than exist on disk, load them
      if (num_loaded_children_ < this->getNumChildren ())
      {
        //check all 8 possible child directories
        for (int i = 0; i < 8; i++)
        {
          boost::filesystem::path child_dir = node_metadata_->getDirectoryPathname () / boost::filesystem::path (std::to_string(i));
          //if the directory exists and the child hasn't been created (set to 0 by this node's constructor)
          if (boost::filesystem::exists (child_dir) && this->children_[i] == nullptr)
          {
            //load the child node
            this->children_[i] = new OutofcoreOctreeBaseNode<ContainerT, PointT> (child_dir, this, recursive);
            //keep track of the children loaded
            num_loaded_children_++;
          }
        }
      }
      assert (num_loaded_children_ == this->getNumChildren ());
    }
    ////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT> void
    OutofcoreOctreeBaseNode<ContainerT, PointT>::recFreeChildren ()
    {
      if (num_children_ == 0)
      {
        return;
      }

      for (std::size_t i = 0; i < 8; i++)
      {
        delete static_cast<OutofcoreOctreeBaseNode<ContainerT, PointT>*>(children_[i]);
      }
      children_.resize (8, static_cast<OutofcoreOctreeBaseNode<ContainerT, PointT>* > (nullptr));
      num_children_ = 0;
    }
    ////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT> std::uint64_t
    OutofcoreOctreeBaseNode<ContainerT, PointT>::addDataToLeaf (const AlignedPointTVector& p, const bool skip_bb_check)
    {
      //quit if there are no points to add
      if (p.empty ())
      {
        return (0);
      }

      //if this depth is the max depth of the tree, then add the points
      if (this->depth_ == this->root_node_->m_tree_->getDepth ())
        return (addDataAtMaxDepth( p, skip_bb_check));

      if (hasUnloadedChildren ())
        loadChildren (false);

      std::vector < std::vector<const PointT*> > c;
      c.resize (8);
      for (std::size_t i = 0; i < 8; i++)
      {
        c[i].reserve (p.size () / 8);
      }

      const std::size_t len = p.size ();
      for (std::size_t i = 0; i < len; i++)
      {
        const PointT& pt = p[i];

        if (!skip_bb_check)
        {
          if (!this->pointInBoundingBox (pt))
          {
            PCL_ERROR ( "[pcl::outofcore::OutofcoreOctreeBaseNode::%s] Failed to place point within bounding box\n", __FUNCTION__ );
            continue;
          }
        }

        std::uint8_t box = 0;
        Eigen::Vector3d mid_xyz = node_metadata_->getVoxelCenter ();
        
        box = static_cast<std::uint8_t>(((pt.z >= mid_xyz[2]) << 2) | ((pt.y >= mid_xyz[1]) << 1) | ((pt.x >= mid_xyz[0]) << 0));
        c[static_cast<std::size_t>(box)].push_back (&pt);
      }
      
      std::uint64_t points_added = 0;
      for (std::size_t i = 0; i < 8; i++)
      {
        if (c[i].empty ())
          continue;
        if (!children_[i])
          createChild (i);
        points_added += children_[i]->addDataToLeaf (c[i], true);
        c[i].clear ();
      }
      return (points_added);
    }
    ////////////////////////////////////////////////////////////////////////////////


    template<typename ContainerT, typename PointT> std::uint64_t
    OutofcoreOctreeBaseNode<ContainerT, PointT>::addDataToLeaf (const std::vector<const PointT*>& p, const bool skip_bb_check)
    {
      if (p.empty ())
      {
        return (0);
      }

      if (this->depth_ == this->root_node_->m_tree_->getDepth ())
      {
        //trust me, just add the points
        if (skip_bb_check)
        {
          root_node_->m_tree_->incrementPointsInLOD (this->depth_, p.size ());
          
          payload_->insertRange (p.data (), p.size ());
          
          return (p.size ());
        }
        //check which points belong to this node, throw away the rest
        std::vector<const PointT*> buff;
        for (const PointT* pt : p)
        {
          if(pointInBoundingBox(*pt))
          {
            buff.push_back(pt);
          }
        }

        if (!buff.empty ())
        {
          root_node_->m_tree_->incrementPointsInLOD (this->depth_, buff.size ());
          payload_->insertRange (buff.data (), buff.size ());
//          payload_->insertRange ( buff );

        }
        return (buff.size ());
      }

      if (this->hasUnloadedChildren ())
      {
        loadChildren (false);
      }

      std::vector < std::vector<const PointT*> > c;
      c.resize (8);
      for (std::size_t i = 0; i < 8; i++)
      {
        c[i].reserve (p.size () / 8);
      }

      const std::size_t len = p.size ();
      for (std::size_t i = 0; i < len; i++)
      {
        //const PointT& pt = p[i];
        if (!skip_bb_check)
        {
          if (!this->pointInBoundingBox (*p[i]))
          {
            // std::cerr << "failed to place point!!!" << std::endl;
            continue;
          }
        }

        std::uint8_t box = 00;
        Eigen::Vector3d mid_xyz = node_metadata_->getVoxelCenter ();
        //hash each coordinate to the appropriate octant
        box = static_cast<std::uint8_t> (((p[i]->z >= mid_xyz[2]) << 2) | ((p[i]->y >= mid_xyz[1]) << 1) | ((p[i]->x >= mid_xyz[0] )));
        //3 bit, 8 octants
        c[box].push_back (p[i]);
      }

      std::uint64_t points_added = 0;
      for (std::size_t i = 0; i < 8; i++)
      {
        if (c[i].empty ())
          continue;
        if (!children_[i])
          createChild (i);
        points_added += children_[i]->addDataToLeaf (c[i], true);
        c[i].clear ();
      }
      return (points_added);
    }
    ////////////////////////////////////////////////////////////////////////////////


    template<typename ContainerT, typename PointT> std::uint64_t
    OutofcoreOctreeBaseNode<ContainerT, PointT>::addPointCloud (const typename pcl::PCLPointCloud2::Ptr& input_cloud, const bool skip_bb_check)
    {
      assert (this->root_node_->m_tree_ != NULL);
      
      if (input_cloud->height*input_cloud->width == 0)
        return (0);
      
      if (this->depth_ == this->root_node_->m_tree_->getDepth ())
        return (addDataAtMaxDepth (input_cloud, true));
      
      if( num_children_ < 8 )
        if(hasUnloadedChildren ())
          loadChildren (false);

      if( !skip_bb_check )
      {

        //indices to store the points for each bin
        //these lists will be used to copy data to new point clouds and pass down recursively
        std::vector < pcl::Indices > indices;
        indices.resize (8);
        
        this->sortOctantIndices (input_cloud, indices, node_metadata_->getVoxelCenter ());

        for(std::size_t k=0; k<indices.size (); k++)
        {
          PCL_DEBUG ("[pcl::outofcore::OutofcoreOctreeBaseNode::%s] Computed %d indices in octact %d\n", __FUNCTION__, indices[k].size (), k);
        }

        std::uint64_t points_added = 0;

        for(std::size_t i=0; i<8; i++)
        {
          if ( indices[i].empty () )
            continue;

          if (children_[i] == nullptr)
          {
            createChild (i);
          }

          pcl::PCLPointCloud2::Ptr dst_cloud (new pcl::PCLPointCloud2 () );

              PCL_DEBUG ( "[pcl::outofcore::OutofcoreOctreeBaseNode::%s] Extracting indices to bins\n", __FUNCTION__);
              
          //copy the points from extracted indices from input cloud to destination cloud
          pcl::copyPointCloud ( *input_cloud, indices[i], *dst_cloud ) ;
          
          //recursively add the new cloud to the data
          points_added += children_[i]->addPointCloud (dst_cloud, false);
          indices[i].clear ();
        }
        
        return (points_added);
      }
      
      PCL_ERROR ("[pcl::outofcore::OutofcoreOctreeBaseNode] Skipped bounding box check. Points not inserted\n");
      
      return 0;
    }


    ////////////////////////////////////////////////////////////////////////////////
    template<typename ContainerT, typename PointT> void
    OutofcoreOctreeBaseNode<ContainerT, PointT>::randomSample(const AlignedPointTVector& p, AlignedPointTVector& insertBuff, const bool skip_bb_check)
    {
      assert (this->root_node_->m_tree_ != NULL);
      
      AlignedPointTVector sampleBuff;
      if (!skip_bb_check)
      {
        for (const PointT& pt: p)
        {
          if (pointInBoundingBox(pt))
          {
            sampleBuff.push_back(pt);
          }
        }
      }
      else
      {
        sampleBuff = p;
      }

      // Derive percentage from specified sample_percent and tree depth
      const double percent = pow(sample_percent_, double((this->root_node_->m_tree_->getDepth () - depth_)));
      const std::uint64_t samplesize = static_cast<std::uint64_t>(percent * static_cast<double>(sampleBuff.size()));
      const std::uint64_t inputsize = sampleBuff.size();

      if(samplesize > 0)
      {
        // Resize buffer to sample size
        insertBuff.resize(samplesize);

        // Create random number generator
        std::lock_guard<std::mutex> lock(rng_mutex_);
        std::uniform_int_distribution<std::uint64_t> buffdist(0, inputsize-1);

        // Randomly pick sampled points
        for(std::uint64_t i = 0; i < samplesize; ++i)
        {
          std::uint64_t buffstart = buffdist(rng_);
          insertBuff[i] = ( sampleBuff[buffstart] );
        }
      }
      // Have to do it the slow way
      else
      {
        std::lock_guard<std::mutex> lock(rng_mutex_);
        std::bernoulli_distribution buffdist(percent);

        for(std::uint64_t i = 0; i < inputsize; ++i)
          if(buffdist(rng_))
            insertBuff.push_back( p[i] );
      }
    }
    ////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT> std::uint64_t
    OutofcoreOctreeBaseNode<ContainerT, PointT>::addDataAtMaxDepth (const AlignedPointTVector& p, const bool skip_bb_check)
    {
      assert (this->root_node_->m_tree_ != NULL);

      // Trust me, just add the points
      if (skip_bb_check)
      {
        // Increment point count for node
        root_node_->m_tree_->incrementPointsInLOD (this->depth_, p.size ());

        // Insert point data
        payload_->insertRange ( p );
        
        return (p.size ());
      }

      // Add points found within the current node's bounding box
      AlignedPointTVector buff;
      const std::size_t len = p.size ();

      for (std::size_t i = 0; i < len; i++)
      {
        if (pointInBoundingBox (p[i]))
        {
          buff.push_back (p[i]);
        }
      }

      if (!buff.empty ())
      {
        root_node_->m_tree_->incrementPointsInLOD (this->depth_, buff.size ());
        payload_->insertRange ( buff );
      }
      return (buff.size ());
    }
    ////////////////////////////////////////////////////////////////////////////////
    template<typename ContainerT, typename PointT> std::uint64_t
    OutofcoreOctreeBaseNode<ContainerT, PointT>::addDataAtMaxDepth (const pcl::PCLPointCloud2::Ptr input_cloud, const bool skip_bb_check)
    {
      //this assumes data is already in the correct bin
      if(skip_bb_check)
      {
        PCL_DEBUG ("[pcl::outofcore::OutofcoreOctreeBaseNode::%s] Adding %u points at max depth, %u\n",__FUNCTION__, input_cloud->width*input_cloud->height, this->depth_);
        
        this->root_node_->m_tree_->incrementPointsInLOD (this->depth_, input_cloud->width*input_cloud->height );
        payload_->insertRange (input_cloud);            

        return (input_cloud->width*input_cloud->height);
      }
      PCL_ERROR ("[pcl::outofcore::OutofcoreOctreeBaseNode] Not implemented\n");
      return (0);
    }


    ////////////////////////////////////////////////////////////////////////////////
    template<typename ContainerT, typename PointT> void
    OutofcoreOctreeBaseNode<ContainerT, PointT>::subdividePoints (const AlignedPointTVector &p, std::vector< AlignedPointTVector > &c, const bool skip_bb_check)
    {
      // Reserve space for children nodes
      c.resize(8);
      for(std::size_t i = 0; i < 8; i++)
        c[i].reserve(p.size() / 8);

      const std::size_t len = p.size();
      for(std::size_t i = 0; i < len; i++)
      {
        const PointT& pt = p[i];

        if(!skip_bb_check)
          if(!this->pointInBoundingBox(pt))
            continue;

        subdividePoint (pt, c);
      }
    }
    ////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT> void
    OutofcoreOctreeBaseNode<ContainerT, PointT>::subdividePoint (const PointT& point, std::vector< AlignedPointTVector >& c)
    {
      Eigen::Vector3d mid_xyz = node_metadata_->getVoxelCenter ();
      std::size_t octant = 0;
      octant = ((point.z >= mid_xyz[2]) << 2) | ((point.y >= mid_xyz[1]) << 1) | ((point.x >= mid_xyz[0]) << 0);
      c[octant].push_back (point);
    }

    ////////////////////////////////////////////////////////////////////////////////
    template<typename ContainerT, typename PointT> std::uint64_t
    OutofcoreOctreeBaseNode<ContainerT, PointT>::addPointCloud_and_genLOD (const pcl::PCLPointCloud2::Ptr input_cloud) //, const bool skip_bb_check = false )
    {
      std::uint64_t points_added = 0;
      
      if ( input_cloud->width * input_cloud->height == 0 )
      {
        return (0);
      }
      
      if ( this->depth_ == this->root_node_->m_tree_->getDepth () || input_cloud->width*input_cloud->height < 8 )
      {
        std::uint64_t points_added = addDataAtMaxDepth (input_cloud, true);
        assert (points_added > 0);
        return (points_added);        
      }
      
      if (num_children_ < 8 )
      {
        if ( hasUnloadedChildren () )
        {
          loadChildren (false);
        }
      }

      //------------------------------------------------------------
      //subsample data:
      //   1. Get indices from a random sample
      //   2. Extract those indices with the extract indices class (in order to also get the complement)
      //------------------------------------------------------------
      pcl::RandomSample<pcl::PCLPointCloud2> random_sampler;
      random_sampler.setInputCloud (input_cloud);

      //set sample size to 1/8 of total points (12.5%)
      std::uint64_t sample_size = input_cloud->width*input_cloud->height / 8;
      random_sampler.setSample (static_cast<unsigned int> (sample_size));
      
      //create our destination
      pcl::PCLPointCloud2::Ptr downsampled_cloud ( new pcl::PCLPointCloud2 () );

      //create destination for indices
      pcl::IndicesPtr downsampled_cloud_indices ( new pcl::Indices () );
      random_sampler.filter (*downsampled_cloud_indices);

      //extract the "random subset", size by setSampleSize
      pcl::ExtractIndices<pcl::PCLPointCloud2> extractor;
      extractor.setInputCloud (input_cloud);
      extractor.setIndices (downsampled_cloud_indices);
      extractor.filter (*downsampled_cloud);

      //extract the complement of those points (i.e. everything remaining)
      pcl::PCLPointCloud2::Ptr remaining_points ( new pcl::PCLPointCloud2 () );
      extractor.setNegative (true);
      extractor.filter (*remaining_points);

      PCL_DEBUG ( "[pcl::outofcore::OutofcoreOctreeBaseNode::%s] Random sampled: %lu of %lu\n", __FUNCTION__, downsampled_cloud->width * downsampled_cloud->height, input_cloud->width * input_cloud->height );
      
      //insert subsampled data to the node's disk container payload
      if ( downsampled_cloud->width * downsampled_cloud->height != 0 )
      {
        root_node_->m_tree_->incrementPointsInLOD ( this->depth_, downsampled_cloud->width * downsampled_cloud->height );
        payload_->insertRange (downsampled_cloud);
        points_added += downsampled_cloud->width*downsampled_cloud->height ;
      }

      PCL_DEBUG ("[pcl::outofcore::OutofcoreOctreeBaseNode::%s] Remaining points are %u\n",__FUNCTION__, remaining_points->width*remaining_points->height);

      //subdivide remaining data by destination octant
      std::vector<pcl::Indices> indices;
      indices.resize (8);

      this->sortOctantIndices (remaining_points, indices, node_metadata_->getVoxelCenter ());

      //pass each set of points to the appropriate child octant
      for(std::size_t i=0; i<8; i++)
      {

        if(indices[i].empty ())
          continue;

        if (children_[i] == nullptr)
        {
          assert (i < 8);
          createChild (i);
        }
        
        //copy correct indices into a temporary cloud
        pcl::PCLPointCloud2::Ptr tmp_local_point_cloud (new pcl::PCLPointCloud2 ());
        pcl::copyPointCloud (*remaining_points, indices[i], *tmp_local_point_cloud);

        //recursively add points and keep track of how many were successfully added to the tree
        points_added += children_[i]->addPointCloud_and_genLOD (tmp_local_point_cloud);
        PCL_DEBUG ("[pcl::outofcore::OutofcoreOctreeBaseNode::%s] points_added: %lu, indices[i].size: %lu, tmp_local_point_cloud size: %lu\n", __FUNCTION__, points_added, indices[i].size (), tmp_local_point_cloud->width*tmp_local_point_cloud->height);

      }
      assert (points_added == input_cloud->width*input_cloud->height);
      return (points_added);
    }
    ////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT> std::uint64_t
    OutofcoreOctreeBaseNode<ContainerT, PointT>::addDataToLeaf_and_genLOD (const AlignedPointTVector& p, const bool skip_bb_check)
    {
      // If there are no points return
      if (p.empty ())
        return (0);

      //  when adding data and generating sampled LOD 
      // If the max depth has been reached
      assert (this->root_node_->m_tree_ != NULL );
      
      if (this->depth_ == this->root_node_->m_tree_->getDepth ())
      {
        PCL_DEBUG ("[pcl::outofcore::OutofcoreOctreeBaseNode::addDataToLeaf_and_genLOD] Adding data to the leaves\n");
        return (addDataAtMaxDepth(p, false));
      }
      
      // Create child nodes of the current node but not grand children+
      if (this->hasUnloadedChildren ())
        loadChildren (false /*no recursive loading*/);

      // Randomly sample data
      AlignedPointTVector insertBuff;
      randomSample(p, insertBuff, skip_bb_check);

      if(!insertBuff.empty())
      {
        // Increment point count for node
        root_node_->m_tree_->incrementPointsInLOD (this->depth_, insertBuff.size());
        // Insert sampled point data
        payload_->insertRange (insertBuff);
        
      }

      //subdivide vec to pass data down lower
      std::vector<AlignedPointTVector> c;
      subdividePoints(p, c, skip_bb_check);

      std::uint64_t points_added = 0;
      for(std::size_t i = 0; i < 8; i++)
      {
        // If child doesn't have points
        if(c[i].empty())
          continue;

        // If child doesn't exist
        if(!children_[i])
          createChild(i);

        // Recursively build children
        points_added += children_[i]->addDataToLeaf_and_genLOD(c[i], true);
        c[i].clear();
      }

      return (points_added);
    }
    ////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT> void
    OutofcoreOctreeBaseNode<ContainerT, PointT>::createChild (const std::size_t idx)
    {
      assert (idx < 8);
      
      //if already has 8 children, return
      if (children_[idx] || (num_children_ == 8))
      {
        PCL_ERROR ("[pcl::outofcore::OutofcoreOctreeBaseNode::createChild] Not allowed to create a 9th child of %s\n",this->node_metadata_->getMetadataFilename ().c_str ());
        return;
      }

      Eigen::Vector3d start = node_metadata_->getBoundingBoxMin ();
      Eigen::Vector3d step = (node_metadata_->getBoundingBoxMax () - start)/static_cast<double>(2.0);

      Eigen::Vector3d childbb_min;
      Eigen::Vector3d childbb_max;

      int x, y, z;
      if (idx > 3)
      {
        x = ((idx == 5) || (idx == 7)) ? 1 : 0;
        y = ((idx == 6) || (idx == 7)) ? 1 : 0;
        z = 1;
      }
      else
      {
        x = ((idx == 1) || (idx == 3)) ? 1 : 0;
        y = ((idx == 2) || (idx == 3)) ? 1 : 0;
        z = 0;
      }

      childbb_min[2] = start[2] + static_cast<double> (z) * step[2];
      childbb_max[2] = start[2] + static_cast<double> (z + 1) * step[2];

      childbb_min[1] = start[1] + static_cast<double> (y) * step[1];
      childbb_max[1] = start[1] + static_cast<double> (y + 1) * step[1];

      childbb_min[0] = start[0] + static_cast<double> (x) * step[0];
      childbb_max[0] = start[0] + static_cast<double> (x + 1) * step[0];

      boost::filesystem::path childdir = node_metadata_->getDirectoryPathname () / boost::filesystem::path (std::to_string(idx));
      children_[idx] = new OutofcoreOctreeBaseNode<ContainerT, PointT> (childbb_min, childbb_max, childdir.string ().c_str (), this);

      num_children_++;
    }
    ////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT> bool
    pointInBoundingBox (const Eigen::Vector3d& min_bb, const Eigen::Vector3d& max_bb, const Eigen::Vector3d& point)
    {
      if (((min_bb[0] <= point[0]) && (point[0] < max_bb[0])) &&
          ((min_bb[1] <= point[1]) && (point[1] < max_bb[1])) &&
          ((min_bb[2] <= point[2]) && (point[2] < max_bb[2])))
      {
        return (true);
    
      }
      return (false);
    }


    ////////////////////////////////////////////////////////////////////////////////
    template<typename ContainerT, typename PointT> bool
    OutofcoreOctreeBaseNode<ContainerT, PointT>::pointInBoundingBox (const PointT& p) const
    {
      const Eigen::Vector3d& min = node_metadata_->getBoundingBoxMin ();
      const Eigen::Vector3d& max = node_metadata_->getBoundingBoxMax ();

      if (((min[0] <= p.x) && (p.x < max[0])) &&
          ((min[1] <= p.y) && (p.y < max[1])) &&
          ((min[2] <= p.z) && (p.z < max[2])))
      {
        return (true);
    
      }
      return (false);
    }

    ////////////////////////////////////////////////////////////////////////////////
    template<typename ContainerT, typename PointT> void
    OutofcoreOctreeBaseNode<ContainerT, PointT>::printBoundingBox (const std::size_t query_depth) const
    {
      Eigen::Vector3d min;
      Eigen::Vector3d max;
      node_metadata_->getBoundingBox (min, max);

      if (this->depth_ < query_depth){
        for (std::size_t i = 0; i < this->depth_; i++)
          std::cout << "  ";

        std::cout << "[" << min[0] << ", " << min[1] << ", " << min[2] << "] - ";
        std::cout << "[" << max[0] << ", " << max[1] << ", " << max[2] << "] - ";
        std::cout <<  "[" << max[0] - min[0] << ", " << max[1] - min[1];
        std::cout << ", " << max[2] - min[2] << "]" << std::endl;

        if (num_children_ > 0)
        {
          for (std::size_t i = 0; i < 8; i++)
          {
            if (children_[i])
              children_[i]->printBoundingBox (query_depth);
          }
        }
      }
    }

    ////////////////////////////////////////////////////////////////////////////////
    template<typename ContainerT, typename PointT> void
    OutofcoreOctreeBaseNode<ContainerT, PointT>::getOccupiedVoxelCentersRecursive (AlignedPointTVector &voxel_centers, const std::size_t query_depth)
    {
      if (this->depth_ < query_depth){
        if (num_children_ > 0)
        {
          for (std::size_t i = 0; i < 8; i++)
          {
            if (children_[i])
              children_[i]->getOccupiedVoxelCentersRecursive (voxel_centers, query_depth);
          }
        }
      }
      else
      {
        PointT voxel_center;
        Eigen::Vector3d mid_xyz = node_metadata_->getVoxelCenter ();
        voxel_center.x = static_cast<float>(mid_xyz[0]);
        voxel_center.y = static_cast<float>(mid_xyz[1]);
        voxel_center.z = static_cast<float>(mid_xyz[2]);

        voxel_centers.push_back(voxel_center);
      }
    }

    ////////////////////////////////////////////////////////////////////////////////
//    Eigen::Vector3d cornerOffsets[] =
//    {
//      Eigen::Vector3d(-1.0, -1.0, -1.0),     // - - -
//      Eigen::Vector3d( 1.0, -1.0, -1.0),     // - - +
//      Eigen::Vector3d(-1.0,  1.0, -1.0),     // - + -
//      Eigen::Vector3d( 1.0,  1.0, -1.0),     // - + +
//      Eigen::Vector3d(-1.0, -1.0,  1.0),     // + - -
//      Eigen::Vector3d( 1.0, -1.0,  1.0),     // + - +
//      Eigen::Vector3d(-1.0,  1.0,  1.0),     // + + -
//      Eigen::Vector3d( 1.0,  1.0,  1.0)      // + + +
//    };
//
//    // Note that the input vector must already be negated when using this code for halfplane tests
//    int
//    vectorToIndex(Eigen::Vector3d normal)
//    {
//      int index = 0;
//
//      if (normal.z () >= 0) index |= 1;
//      if (normal.y () >= 0) index |= 2;
//      if (normal.x () >= 0) index |= 4;
//
//      return index;
//    }
//
//    void
//    get_np_vertices(Eigen::Vector3d normal, Eigen::Vector3d &p_vertex, Eigen::Vector3d &n_vertex, Eigen::Vector3d min_bb, Eigen::Vector3d max_bb)
//    {
//
//      p_vertex = min_bb;
//      n_vertex = max_bb;
//
//      if (normal.x () >= 0)
//      {
//        n_vertex.x () = min_bb.x ();
//        p_vertex.x () = max_bb.x ();
//      }
//
//      if (normal.y () >= 0)
//      {
//        n_vertex.y () = min_bb.y ();
//        p_vertex.y () = max_bb.y ();
//      }
//
//      if (normal.z () >= 0)
//      {
//        p_vertex.z () = max_bb.z ();
//        n_vertex.z () = min_bb.z ();
//      }
//    }

    template<typename Container, typename PointT> void
    OutofcoreOctreeBaseNode<Container, PointT>::queryFrustum (const double planes[24], std::list<std::string>& file_names)
    {
      queryFrustum(planes, file_names, this->m_tree_->getTreeDepth());
    }

    template<typename Container, typename PointT> void
    OutofcoreOctreeBaseNode<Container, PointT>::queryFrustum (const double planes[24], std::list<std::string>& file_names, const std::uint32_t query_depth, const bool skip_vfc_check)
    {

      enum {INSIDE, INTERSECT, OUTSIDE};

      int result = INSIDE;

      if (this->depth_ > query_depth)
      {
        return;
      }

//      if (this->depth_ > query_depth)
//        return;

      if (!skip_vfc_check)
      {
        for(int i =0; i < 6; i++){
          double a = planes[(i*4)];
          double b = planes[(i*4)+1];
          double c = planes[(i*4)+2];
          double d = planes[(i*4)+3];

          //std::cout << i << ": " << a << "x + " << b << "y + " << c << "z + " << d << std::endl;

          Eigen::Vector3d normal(a, b, c);

          Eigen::Vector3d min_bb;
          Eigen::Vector3d max_bb;
          node_metadata_->getBoundingBox(min_bb, max_bb);

          //  Basic VFC algorithm
          Eigen::Vector3d center = node_metadata_->getVoxelCenter();
          Eigen::Vector3d radius (std::abs (static_cast<double> (max_bb.x () - center.x ())),
                                  std::abs (static_cast<double> (max_bb.y () - center.y ())),
                                  std::abs (static_cast<double> (max_bb.z () - center.z ())));

          double m = (center.x () * a) + (center.y () * b) + (center.z () * c) + d;
          double n = (radius.x () * std::abs(a)) + (radius.y () * std::abs(b)) + (radius.z () * std::abs(c));

          if (m + n < 0){
            result = OUTSIDE;
            break;
          }

          if (m - n < 0) result = INTERSECT;

  //        // n-p implementation
  //        Eigen::Vector3d p_vertex; //pos vertex
  //        Eigen::Vector3d n_vertex; //neg vertex
  //        get_np_vertices(normal, p_vertex, n_vertex, min_bb, max_bb);
  //
  //        std::cout << "n_vertex: " << n_vertex.x () << ", " << n_vertex.y () << ", " << n_vertex.z () << std::endl;
  //        std::cout << "p_vertex: " << p_vertex.x () << ", " << p_vertex.y () << ", " << p_vertex.z () << std::endl;

          // is the positive vertex outside?
  //        if (pl[i].distance(b.getVertexP(pl[i].normal)) < 0)
  //        {
  //          result = OUTSIDE;
  //        }
  //        // is the negative vertex outside?
  //        else if (pl[i].distance(b.getVertexN(pl[i].normal)) < 0)
  //        {
  //          result = INTERSECT;
  //        }

  //
  //
  //        // This should be the same as below
  //        if (normal.dot(n_vertex) + d > 0)
  //        {
  //          result = OUTSIDE;
  //        }
  //
  //        if (normal.dot(p_vertex) + d >= 0)
  //        {
  //          result = INTERSECT;
  //        }

          // This should be the same as above
  //        double m = (a * n_vertex.x ()) + (b * n_vertex.y ()) + (c * n_vertex.z ());
  //        std::cout << "m = " << m << std::endl;
  //        if (m > -d)
  //        {
  //          result = OUTSIDE;
  //        }
  //
  //        double n = (a * p_vertex.x ()) + (b * p_vertex.y ()) + (c * p_vertex.z ());
  //        std::cout << "n = " << n << std::endl;
  //        if (n > -d)
  //        {
  //          result = INTERSECT;
  //        }
        }
      }

      if (result == OUTSIDE)
      {
        return;
      }

//      switch(result){
//        case OUTSIDE:
//          //std::cout << this->depth_ << " [OUTSIDE]: " << node_metadata_->getPCDFilename() << std::endl;
//          return;
//        case INTERSECT:
//          //std::cout << this->depth_ << " [INTERSECT]: " << node_metadata_->getPCDFilename() << std::endl;
//          break;
//        case INSIDE:
//          //std::cout << this->depth_ << " [INSIDE]: " << node_metadata_->getPCDFilename() << std::endl;
//          break;
//      }

      // Add files breadth first
      if (this->depth_ == query_depth && payload_->getDataSize () > 0)
      //if (payload_->getDataSize () > 0)
      {
        file_names.push_back (this->node_metadata_->getMetadataFilename ().string ());
      }

      if (hasUnloadedChildren ())
      {
        loadChildren (false);
      }

      if (this->getNumChildren () > 0)
      {
        for (std::size_t i = 0; i < 8; i++)
        {
          if (children_[i])
            children_[i]->queryFrustum (planes, file_names, query_depth, (result == INSIDE) /*skip_vfc_check*/);
        }
      }
//      else if (hasUnloadedChildren ())
//      {
//        loadChildren (false);
//
//        for (std::size_t i = 0; i < 8; i++)
//        {
//          if (children_[i])
//            children_[i]->queryFrustum (planes, file_names, query_depth);
//        }
//      }
      //}
    }

////////////////////////////////////////////////////////////////////////////////

    template<typename Container, typename PointT> void
    OutofcoreOctreeBaseNode<Container, PointT>::queryFrustum (const double planes[24], const Eigen::Vector3d &eye, const Eigen::Matrix4d &view_projection_matrix, std::list<std::string>& file_names, const std::uint32_t query_depth, const bool skip_vfc_check)
    {

      // If we're above our query depth
      if (this->depth_ > query_depth)
      {
        return;
      }

      // Bounding Box
      Eigen::Vector3d min_bb;
      Eigen::Vector3d max_bb;
      node_metadata_->getBoundingBox(min_bb, max_bb);

      // Frustum Culling
      enum {INSIDE, INTERSECT, OUTSIDE};

      int result = INSIDE;

      if (!skip_vfc_check)
      {
        for(int i =0; i < 6; i++){
          double a = planes[(i*4)];
          double b = planes[(i*4)+1];
          double c = planes[(i*4)+2];
          double d = planes[(i*4)+3];

          //std::cout << i << ": " << a << "x + " << b << "y + " << c << "z + " << d << std::endl;

          Eigen::Vector3d normal(a, b, c);

          //  Basic VFC algorithm
          Eigen::Vector3d center = node_metadata_->getVoxelCenter();
          Eigen::Vector3d radius (std::abs (static_cast<double> (max_bb.x () - center.x ())),
                                  std::abs (static_cast<double> (max_bb.y () - center.y ())),
                                  std::abs (static_cast<double> (max_bb.z () - center.z ())));

          double m = (center.x () * a) + (center.y () * b) + (center.z () * c) + d;
          double n = (radius.x () * std::abs(a)) + (radius.y () * std::abs(b)) + (radius.z () * std::abs(c));

          if (m + n < 0){
            result = OUTSIDE;
            break;
          }

          if (m - n < 0) result = INTERSECT;

        }
      }

      if (result == OUTSIDE)
      {
        return;
      }

      // Bounding box projection
      //      3--------2
      //     /|       /|       Y      0 = xmin, ymin, zmin
      //    / |      / |       |      6 = xmax, ymax. zmax
      //   7--------6  |       |
      //   |  |     |  |       |
      //   |  0-----|--1       +------X
      //   | /      | /       /
      //   |/       |/       /
      //   4--------5       Z

//      bounding_box[0] = Eigen::Vector4d(min_bb.x (), min_bb.y (), min_bb.z (), 1.0);
//      bounding_box[1] = Eigen::Vector4d(max_bb.x (), min_bb.y (), min_bb.z (), 1.0);
//      bounding_box[2] = Eigen::Vector4d(max_bb.x (), max_bb.y (), min_bb.z (), 1.0);
//      bounding_box[3] = Eigen::Vector4d(min_bb.x (), max_bb.y (), min_bb.z (), 1.0);
//      bounding_box[4] = Eigen::Vector4d(min_bb.x (), min_bb.y (), max_bb.z (), 1.0);
//      bounding_box[5] = Eigen::Vector4d(max_bb.x (), min_bb.y (), max_bb.z (), 1.0);
//      bounding_box[6] = Eigen::Vector4d(max_bb.x (), max_bb.y (), max_bb.z (), 1.0);
//      bounding_box[7] = Eigen::Vector4d(min_bb.x (), max_bb.y (), max_bb.z (), 1.0);

      int width = 500;
      int height = 500;

      float coverage = pcl::visualization::viewScreenArea(eye, min_bb, max_bb, view_projection_matrix, width, height);
      //float coverage = pcl::visualization::viewScreenArea(eye, bounding_box, view_projection_matrix);

//      for (int i=0; i < this->depth_; i++) std::cout << " ";
//      std::cout << this->depth_ << ": " << coverage << std::endl;

      // Add files breadth first
      if (this->depth_ <= query_depth && payload_->getDataSize () > 0)
      //if (payload_->getDataSize () > 0)
      {
        file_names.push_back (this->node_metadata_->getMetadataFilename ().string ());
      }

      //if (coverage <= 0.075)
      if (coverage <= 10000)
        return;

      if (hasUnloadedChildren ())
      {
        loadChildren (false);
      }

      if (this->getNumChildren () > 0)
      {
        for (std::size_t i = 0; i < 8; i++)
        {
          if (children_[i])
            children_[i]->queryFrustum (planes, eye, view_projection_matrix, file_names, query_depth, (result == INSIDE) /*skip_vfc_check*/);
        }
      }
    }

////////////////////////////////////////////////////////////////////////////////
    template<typename ContainerT, typename PointT> void
    OutofcoreOctreeBaseNode<ContainerT, PointT>::getOccupiedVoxelCentersRecursive (std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > &voxel_centers, const std::size_t query_depth)
    {
      if (this->depth_ < query_depth){
        if (num_children_ > 0)
        {
          for (std::size_t i = 0; i < 8; i++)
          {
            if (children_[i])
              children_[i]->getOccupiedVoxelCentersRecursive (voxel_centers, query_depth);
          }
        }
      }
      else
      {
        Eigen::Vector3d voxel_center = node_metadata_->getVoxelCenter ();
        voxel_centers.push_back(voxel_center);
      }
    }


    ////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT> void
    OutofcoreOctreeBaseNode<ContainerT, PointT>::queryBBIntersects (const Eigen::Vector3d& min_bb, const Eigen::Vector3d& max_bb, const std::uint32_t query_depth, std::list<std::string>& file_names)
    {
      
      Eigen::Vector3d my_min = min_bb;
      Eigen::Vector3d my_max = max_bb;
      
      if (intersectsWithBoundingBox (my_min, my_max))
      {
        if (this->depth_ < query_depth)
        {
          if (this->getNumChildren () > 0)
          {
            for (std::size_t i = 0; i < 8; i++)
            {
              if (children_[i])
                children_[i]->queryBBIntersects (my_min, my_max, query_depth, file_names);
            }
          }
          else if (hasUnloadedChildren ())
          {
            loadChildren (false);

            for (std::size_t i = 0; i < 8; i++)
            {
              if (children_[i])
                children_[i]->queryBBIntersects (my_min, my_max, query_depth, file_names);
            }
          }
          return;
        }

        if (payload_->getDataSize () > 0)
        {
          file_names.push_back (this->node_metadata_->getMetadataFilename ().string ());
        }
      }
    }
    ////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT> void
    OutofcoreOctreeBaseNode<ContainerT, PointT>::queryBBIncludes (const Eigen::Vector3d& min_bb, const Eigen::Vector3d& max_bb, std::size_t query_depth, const pcl::PCLPointCloud2::Ptr& dst_blob)
    {
      std::uint64_t startingSize = dst_blob->width*dst_blob->height;
      PCL_DEBUG ("[pcl::outofcore::OutofcoreOctreeBaseNode::%s] Starting points in destination blob: %ul\n", __FUNCTION__, startingSize );

      // If the queried bounding box has any intersection with this node's bounding box
      if (intersectsWithBoundingBox (min_bb, max_bb))
      {
        // If we aren't at the max desired depth
        if (this->depth_ < query_depth)
        {
          //if this node doesn't have any children, we are at the max depth for this query
          if ((num_children_ == 0) && (hasUnloadedChildren ()))
            loadChildren (false);

          //if this node has children
          if (num_children_ > 0)
          {
            //recursively store any points that fall into the queried bounding box into v and return
            for (std::size_t i = 0; i < 8; i++)
            {
              if (children_[i])
                children_[i]->queryBBIncludes (min_bb, max_bb, query_depth, dst_blob);
            }
            PCL_DEBUG ( "[pcl::outofcore::OutofcoreOctreeBaseNode::%s] Points in dst_blob: %ul\n", __FUNCTION__, dst_blob->width*dst_blob->height );
            return;
          }
        }
        else //otherwise if we are at the max depth
        {
          //get all the points from the payload and return (easy with PCLPointCloud2)
          pcl::PCLPointCloud2::Ptr tmp_blob (new pcl::PCLPointCloud2 ());
          pcl::PCLPointCloud2::Ptr tmp_dst_blob (new pcl::PCLPointCloud2 ());
          //load all the data in this node from disk
          payload_->readRange (0, payload_->size (), tmp_blob);

          if( tmp_blob->width*tmp_blob->height == 0 )
            return;

          //if this node's bounding box falls completely within the queried bounding box, keep all the points
          if (inBoundingBox (min_bb, max_bb))
          {
            //concatenate all of what was just read into the main dst_blob
            //(is it safe to do in place?)
            
            //if there is already something in the destination blob (remember this method is recursive)
            if( dst_blob->width*dst_blob->height != 0 )
            {
              PCL_DEBUG ("[pcl::outofocre::OutofcoreOctreeBaseNode::%s] Size of cloud before: %lu\n", __FUNCTION__, dst_blob->width*dst_blob->height );
              PCL_DEBUG ("[pcl::outofcore::OutofcoreOctreeBaseNode::%s] Concatenating point cloud\n", __FUNCTION__);
              int res = pcl::concatenate (*dst_blob, *tmp_blob, *dst_blob);
              pcl::utils::ignore(res);
              assert (res == 1);

              PCL_DEBUG ("[pcl::outofocre::OutofcoreOctreeBaseNode::%s] Size of cloud after: %lu\n", __FUNCTION__, dst_blob->width*dst_blob->height );
            }
            //otherwise, just copy the tmp_blob into the dst_blob
            else 
            {
              PCL_DEBUG ( "[pcl::outofcore::OutofcoreOctreeBaseNode] Copying point cloud into the destination blob\n");
              pcl::copyPointCloud (*tmp_blob, *dst_blob);
              assert (tmp_blob->width*tmp_blob->height == dst_blob->width*dst_blob->height);
            }
            PCL_DEBUG ( "[pcl::outofcore::OutofcoreOctreeBaseNode::%s] Points in dst_blob: %ul\n", __FUNCTION__, dst_blob->width*dst_blob->height );
            return;
          }
          //otherwise queried bounding box only partially intersects this
          //node's bounding box, so we have to check all the points in
          //this box for intersection with queried bounding box

//          PCL_DEBUG ("[pcl::outofcore::OutofcoreOctreeBaseNode::%s] Partial extraction of points in bounding box. Desired: %.2lf %.2lf %2lf, %.2lf %.2lf %.2lf; This node BB: %.2lf %.2lf %.2lf, %.2lf %.2lf %.2lf\n", __FUNCTION__, min_bb[0], min_bb[1], min_bb[2], max_bb[0], max_bb[1], max_bb[2], min_[0], min_[1], min_[2], max_[0], max_[1], max_[2] );
          //put the ros message into a pointxyz point cloud (just to get the indices by using getPointsInBox)
          typename pcl::PointCloud<PointT>::Ptr tmp_cloud ( new pcl::PointCloud<PointT> () );
          pcl::fromPCLPointCloud2 ( *tmp_blob, *tmp_cloud );
          assert (tmp_blob->width*tmp_blob->height == tmp_cloud->width*tmp_cloud->height );

          Eigen::Vector4f min_pt ( static_cast<float> ( min_bb[0] ), static_cast<float> ( min_bb[1] ), static_cast<float> ( min_bb[2] ), 1.0f);
          Eigen::Vector4f max_pt ( static_cast<float> ( max_bb[0] ), static_cast<float> ( max_bb[1] ) , static_cast<float>( max_bb[2] ), 1.0f );

          pcl::Indices indices;

          pcl::getPointsInBox ( *tmp_cloud, min_pt, max_pt, indices );
          PCL_DEBUG ( "[pcl::outofcore::OutofcoreOctreeBaseNode::%s] Points in box: %d\n", __FUNCTION__, indices.size () );
          PCL_DEBUG ( "[pcl::outofcore::OutofcoreOctreeBaseNode::%s] Points remaining: %d\n", __FUNCTION__, tmp_cloud->width*tmp_cloud->height - indices.size () );

            if ( !indices.empty () )
          {
            if( dst_blob->width*dst_blob->height > 0 )
            {
              //need a new tmp destination with extracted points within BB
              pcl::PCLPointCloud2::Ptr tmp_blob_within_bb (new pcl::PCLPointCloud2 ());

              //copy just the points marked in indices
              pcl::copyPointCloud ( *tmp_blob, indices, *tmp_blob_within_bb );
              assert ( tmp_blob_within_bb->width*tmp_blob_within_bb->height == indices.size () );
              assert ( tmp_blob->fields.size () == tmp_blob_within_bb->fields.size () );
              //concatenate those points into the returned dst_blob
              PCL_DEBUG ("[pcl::outofcore::OutofcoreOctreeBaseNode::%s] Concatenating point cloud in place\n", __FUNCTION__);
              std::uint64_t orig_points_in_destination = dst_blob->width*dst_blob->height;
              int res = pcl::concatenate (*dst_blob, *tmp_blob_within_bb, *dst_blob);
              pcl::utils::ignore(orig_points_in_destination, res);
              assert (res == 1);
              assert (dst_blob->width*dst_blob->height == indices.size () + orig_points_in_destination);

            }
            else
            {
              pcl::copyPointCloud ( *tmp_blob, indices, *dst_blob );
              assert ( dst_blob->width*dst_blob->height == indices.size () );
            }
          }
        }
      }

      PCL_DEBUG ("[pcl::outofcore::OutofcoreOctreeBaseNode::%s] Points added by function call: %ul\n", __FUNCTION__, dst_blob->width*dst_blob->height - startingSize );
    }

    template<typename ContainerT, typename PointT> void
    OutofcoreOctreeBaseNode<ContainerT, PointT>::queryBBIncludes (const Eigen::Vector3d& min_bb, const Eigen::Vector3d& max_bb, std::size_t query_depth, AlignedPointTVector& v)
    {

      //if the queried bounding box has any intersection with this node's bounding box
      if (intersectsWithBoundingBox (min_bb, max_bb))
      {
        //if we aren't at the max desired depth
        if (this->depth_ < query_depth)
        {
          //if this node doesn't have any children, we are at the max depth for this query
          if (this->hasUnloadedChildren ())
          {
            this->loadChildren (false);
          }

          //if this node has children
          if (getNumChildren () > 0)
          {
            if(hasUnloadedChildren ())
              loadChildren (false);

            //recursively store any points that fall into the queried bounding box into v and return
            for (std::size_t i = 0; i < 8; i++)
            {
              if (children_[i])
                children_[i]->queryBBIncludes (min_bb, max_bb, query_depth, v);
            }
            return;
          }
        }
        //otherwise if we are at the max depth
        else
        {
          //if this node's bounding box falls completely within the queried bounding box
          if (inBoundingBox (min_bb, max_bb))
          {
            //get all the points from the payload and return
            AlignedPointTVector payload_cache;
            payload_->readRange (0, payload_->size (), payload_cache);
            v.insert (v.end (), payload_cache.begin (), payload_cache.end ());
            return;
          }
          //otherwise queried bounding box only partially intersects this
          //node's bounding box, so we have to check all the points in
          //this box for intersection with queried bounding box
          //read _all_ the points in from the disk container
          AlignedPointTVector payload_cache;
          payload_->readRange (0, payload_->size (), payload_cache);

          std::uint64_t len = payload_->size ();
          //iterate through each of them
          for (std::uint64_t i = 0; i < len; i++)
          {
            const PointT& p = payload_cache[i];
            //if it falls within this bounding box
            if (pointInBoundingBox (min_bb, max_bb, p))
            {
              //store it in the list
              v.push_back (p);
            }
            else
            {
              PCL_DEBUG ("[pcl::outofcore::queryBBIncludes] Point %.2lf %.2lf %.2lf not in bounding box %.2lf %.2lf %.2lf", p.x, p.y, p.z, min_bb[0], min_bb[1], min_bb[2], max_bb[0], max_bb[1], max_bb[2]);
            }
          }
        }
      }
    }
    
    ////////////////////////////////////////////////////////////////////////////////
    template<typename ContainerT, typename PointT> void
    OutofcoreOctreeBaseNode<ContainerT, PointT>::queryBBIncludes_subsample (const Eigen::Vector3d& min_bb, const Eigen::Vector3d& max_bb, std::uint64_t query_depth, const pcl::PCLPointCloud2::Ptr& dst_blob, double percent)
    {
      if (intersectsWithBoundingBox (min_bb, max_bb))
        {
          if (this->depth_ < query_depth)
          {
            if (this->hasUnloadedChildren ())
              this->loadChildren (false);

            if (this->getNumChildren () > 0)
            {
              for (std::size_t i=0; i<8; i++)
              {
                //recursively traverse (depth first)
                if (children_[i]!=nullptr)
                  children_[i]->queryBBIncludes_subsample (min_bb, max_bb, query_depth, dst_blob, percent);
              }
              return;
            }
          }
          //otherwise, at max depth --> read from disk, subsample, concatenate
          else
          {
            
            if (inBoundingBox (min_bb, max_bb))
            {
              pcl::PCLPointCloud2::Ptr tmp_blob;
              this->payload_->read (tmp_blob);
              std::uint64_t num_pts = tmp_blob->width*tmp_blob->height;
                
              double sample_points = static_cast<double>(num_pts) * percent;
              if (num_pts > 0)
              {
                //always sample at least one point
                sample_points = sample_points > 1 ? sample_points : 1;
              }
              else
              {
                return;
              }
              
              
              pcl::RandomSample<pcl::PCLPointCloud2> random_sampler;
              random_sampler.setInputCloud (tmp_blob);
              
              pcl::PCLPointCloud2::Ptr downsampled_points (new pcl::PCLPointCloud2 ());
              
              //set sample size as percent * number of points read
              random_sampler.setSample (static_cast<unsigned int> (sample_points));

              pcl::ExtractIndices<pcl::PCLPointCloud2> extractor;
              extractor.setInputCloud (tmp_blob);
              
              pcl::IndicesPtr downsampled_cloud_indices (new pcl::Indices ());
              random_sampler.filter (*downsampled_cloud_indices);
              extractor.setIndices (downsampled_cloud_indices);
              extractor.filter (*downsampled_points);
              
              //concatenate the result into the destination cloud
              pcl::concatenate (*dst_blob, *downsampled_points, *dst_blob);
            }
          }
        }
    }
    
    
    ////////////////////////////////////////////////////////////////////////////////
    template<typename ContainerT, typename PointT> void
    OutofcoreOctreeBaseNode<ContainerT, PointT>::queryBBIncludes_subsample (const Eigen::Vector3d& min_bb, const Eigen::Vector3d& max_bb, std::uint64_t query_depth, const double percent, AlignedPointTVector& dst)
    {
      //check if the queried bounding box has any intersection with this node's bounding box
      if (intersectsWithBoundingBox (min_bb, max_bb))
      {
        //if we are not at the max depth for queried nodes
        if (this->depth_ < query_depth)
        {
          //check if we don't have children
          if ((num_children_ == 0) && (hasUnloadedChildren ()))
          {
            loadChildren (false);
          }
          //if we do have children
          if (num_children_ > 0)
          {
            //recursively add their valid points within the queried bounding box to the list v
            for (std::size_t i = 0; i < 8; i++)
            {
              if (children_[i])
                children_[i]->queryBBIncludes_subsample (min_bb, max_bb, query_depth, percent, dst);
            }
            return;
          }
        }
        //otherwise we are at the max depth, so we add all our points or some of our points
        else
        {
          //if this node's bounding box falls completely within the queried bounding box
          if (inBoundingBox (min_bb, max_bb))
          {
            //add a random sample of all the points
            AlignedPointTVector payload_cache;
            payload_->readRangeSubSample (0, payload_->size (), percent, payload_cache);
            dst.insert (dst.end (), payload_cache.begin (), payload_cache.end ());
            return;
          }
          //otherwise the queried bounding box only partially intersects with this node's bounding box
          //brute force selection of all valid points
          AlignedPointTVector payload_cache_within_region;
          {
            AlignedPointTVector payload_cache;
            payload_->readRange (0, payload_->size (), payload_cache);
            for (std::size_t i = 0; i < payload_->size (); i++)
            {
              const PointT& p = payload_cache[i];
              if (pointInBoundingBox (min_bb, max_bb, p))
              {
                payload_cache_within_region.push_back (p);
              }
            }
          }//force the payload cache to deconstruct here

          //use STL random_shuffle and push back a random selection of the points onto our list
          std::shuffle (payload_cache_within_region.begin (), payload_cache_within_region.end (), std::mt19937(std::random_device()()));
          std::size_t numpick = static_cast<std::size_t> (percent * static_cast<double> (payload_cache_within_region.size ()));;

          for (std::size_t i = 0; i < numpick; i++)
          {
            dst.push_back (payload_cache_within_region[i]);
          }
        }
      }
    }
    ////////////////////////////////////////////////////////////////////////////////

//dir is current level. we put this nodes files into it
    template<typename ContainerT, typename PointT>
    OutofcoreOctreeBaseNode<ContainerT, PointT>::OutofcoreOctreeBaseNode (const Eigen::Vector3d& bb_min, const Eigen::Vector3d& bb_max, const char* dir, OutofcoreOctreeBaseNode<ContainerT,PointT>* super)
      : m_tree_ ()
      , root_node_ ()
      , parent_ ()
      , depth_ ()
      , children_ (8, nullptr)
      , num_children_ ()
      , num_loaded_children_ (0)
      , payload_ ()
      , node_metadata_ (new OutofcoreOctreeNodeMetadata)
    {
      node_metadata_->setOutofcoreVersion (3);
      
      if (super == nullptr)
      {
        PCL_ERROR ( "[pc::outofcore::OutofcoreOctreeBaseNode] Super is null - don't make a root node this way!\n" );
        PCL_THROW_EXCEPTION (PCLException, "[pcl::outofcore::OutofcoreOctreeBaseNode] Outofcore Exception: Bad parent");
      }

      this->parent_ = super;
      root_node_ = super->root_node_;
      m_tree_ = super->root_node_->m_tree_;
      assert (m_tree_ != NULL);

      depth_ = super->depth_ + 1;
      num_children_ = 0;

      node_metadata_->setBoundingBox (bb_min, bb_max);

      std::string uuid_idx;
      std::string uuid_cont;
      OutofcoreOctreeDiskContainer<PointT>::getRandomUUIDString (uuid_idx);
      OutofcoreOctreeDiskContainer<PointT>::getRandomUUIDString (uuid_cont);

      std::string node_index_name = uuid_idx + std::string ("_") + node_index_basename + node_index_extension;

      std::string node_container_name;
      node_container_name = uuid_cont + std::string ("_") + node_container_basename + pcd_extension;

      node_metadata_->setDirectoryPathname (boost::filesystem::path (dir));
      node_metadata_->setPCDFilename (node_metadata_->getDirectoryPathname () / boost::filesystem::path (node_container_name));
      node_metadata_->setMetadataFilename ( node_metadata_->getDirectoryPathname ()/boost::filesystem::path (node_index_name));

      boost::filesystem::create_directory (node_metadata_->getDirectoryPathname ());

      payload_.reset (new ContainerT (node_metadata_->getPCDFilename ()));
      this->saveIdx (false);
    }

    ////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT> void
    OutofcoreOctreeBaseNode<ContainerT, PointT>::copyAllCurrentAndChildPointsRec (std::list<PointT>& v)
    {
      if ((num_children_ == 0) && (hasUnloadedChildren ()))
      {
        loadChildren (false);
      }

      for (std::size_t i = 0; i < num_children_; i++)
      {
        children_[i]->copyAllCurrentAndChildPointsRec (v);
      }

      AlignedPointTVector payload_cache;
      payload_->readRange (0, payload_->size (), payload_cache);

      {
        v.insert (v.end (), payload_cache.begin (), payload_cache.end ());
      }
    }

    ////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT> void
    OutofcoreOctreeBaseNode<ContainerT, PointT>::copyAllCurrentAndChildPointsRec_sub (std::list<PointT>& v, const double percent)
    {
      if ((num_children_ == 0) && (hasUnloadedChildren ()))
      {
        loadChildren (false);
      }

      for (std::size_t i = 0; i < 8; i++)
      {
        if (children_[i])
          children_[i]->copyAllCurrentAndChildPointsRec_sub (v, percent);
      }

      std::vector<PointT> payload_cache;
      payload_->readRangeSubSample (0, payload_->size (), percent, payload_cache);

      for (std::size_t i = 0; i < payload_cache.size (); i++)
      {
        v.push_back (payload_cache[i]);
      }
    }

    ////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT> inline bool
    OutofcoreOctreeBaseNode<ContainerT, PointT>::intersectsWithBoundingBox (const Eigen::Vector3d& min_bb, const Eigen::Vector3d& max_bb) const
    {
      Eigen::Vector3d min, max;
      node_metadata_->getBoundingBox (min, max);
      
      //Check whether any portion of min_bb, max_bb falls within min,max
      if (((min[0] <= min_bb[0]) && (min_bb[0] <= max[0])) || ((min_bb[0] <= min[0]) && (min[0] <= max_bb[0])))
      {
        if (((min[1] <= min_bb[1]) && (min_bb[1] <= max[1])) || ((min_bb[1] <= min[1]) && (min[1] <= max_bb[1])))
        {
          if (((min[2] <= min_bb[2]) && (min_bb[2] <= max[2])) || ((min_bb[2] <= min[2]) && (min[2] <= max_bb[2])))
          {
            return (true);
          }
        }
      }

      return (false);
    }
    ////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT> inline bool
    OutofcoreOctreeBaseNode<ContainerT, PointT>::inBoundingBox (const Eigen::Vector3d& min_bb, const Eigen::Vector3d& max_bb) const
    {
      Eigen::Vector3d min, max;

      node_metadata_->getBoundingBox (min, max);

      if ((min_bb[0] <= min[0]) && (max[0] <= max_bb[0]))
      {
        if ((min_bb[1] <= min[1]) && (max[1] <= max_bb[1]))
        {
          if ((min_bb[2] <= min[2]) && (max[2] <= max_bb[2]))
          {
            return (true);
          }
        }
      }

      return (false);
    }
    ////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT> inline bool
    OutofcoreOctreeBaseNode<ContainerT, PointT>::pointInBoundingBox (const Eigen::Vector3d& min_bb, const Eigen::Vector3d& max_bb,
                                                         const PointT& p)
    {
      //by convention, minimum boundary is included; maximum boundary is not
      if ((min_bb[0] <= p.x) && (p.x < max_bb[0]))
      {
        if ((min_bb[1] <= p.y) && (p.y < max_bb[1]))
        {
          if ((min_bb[2] <= p.z) && (p.z < max_bb[2]))
          {
            return (true);
          }
        }
      }
      return (false);
    }

    ////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT> void
    OutofcoreOctreeBaseNode<ContainerT, PointT>::writeVPythonVisual (std::ofstream& file)
    {
      Eigen::Vector3d min;
      Eigen::Vector3d max;
      node_metadata_->getBoundingBox (min, max);

      double l = max[0] - min[0];
      double h = max[1] - min[1];
      double w = max[2] - min[2];
      file << "box( pos=(" << min[0] << ", " << min[1] << ", " << min[2] << "), length=" << l << ", height=" << h
           << ", width=" << w << " )\n";

      for (std::size_t i = 0; i < num_children_; i++)
      {
        children_[i]->writeVPythonVisual (file);
      }
    }

    ////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT> int
    OutofcoreOctreeBaseNode<ContainerT, PointT>::read (pcl::PCLPointCloud2::Ptr &output_cloud)
    {
      return (this->payload_->read (output_cloud));
    }

    ////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT> OutofcoreOctreeBaseNode<ContainerT, PointT>*
    OutofcoreOctreeBaseNode<ContainerT, PointT>::getChildPtr (std::size_t index_arg) const
    {
      PCL_DEBUG ("[pcl::outofcore::OutofcoreOctreeBaseNode::%s] %d", __FUNCTION__, index_arg);
      return (children_[index_arg]);
    }

    ////////////////////////////////////////////////////////////////////////////////
    template<typename ContainerT, typename PointT> std::uint64_t
    OutofcoreOctreeBaseNode<ContainerT, PointT>::getDataSize () const
    {
      return (this->payload_->getDataSize ());
    }

    ////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT> std::size_t
    OutofcoreOctreeBaseNode<ContainerT, PointT>::countNumLoadedChildren () const
    {
      std::size_t loaded_children_count = 0;
      
      for (std::size_t i=0; i<8; i++)
      {
        if (children_[i] != nullptr)
          loaded_children_count++;
      }
      
      return (loaded_children_count);
    }
    
    ////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT> void
    OutofcoreOctreeBaseNode<ContainerT, PointT>::loadFromFile (const boost::filesystem::path& path, OutofcoreOctreeBaseNode<ContainerT, PointT>* super)
    {
      PCL_DEBUG ("[pcl:outofcore::OutofcoreOctreeBaseNode] Loading metadata from %s\n", path.filename ().c_str ());
      node_metadata_->loadMetadataFromDisk (path);

      //this shouldn't be part of 'loadFromFile'
      this->parent_ = super;

      if (num_children_ > 0)
        recFreeChildren ();      

      this->num_children_ = 0;
      this->payload_.reset (new ContainerT (node_metadata_->getPCDFilename ()));
    }

    ////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT> void
    OutofcoreOctreeBaseNode<ContainerT, PointT>::convertToXYZRecursive ()
    {
      std::string fname = boost::filesystem::basename (node_metadata_->getPCDFilename ()) + std::string (".dat.xyz");
      boost::filesystem::path xyzfile = node_metadata_->getDirectoryPathname () / fname;
      payload_->convertToXYZ (xyzfile);

      if (hasUnloadedChildren ())
      {
        loadChildren (false);
      }

      for (std::size_t i = 0; i < 8; i++)
      {
        if (children_[i])
          children_[i]->convertToXYZ ();
      }
    }

    ////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT> void
    OutofcoreOctreeBaseNode<ContainerT, PointT>::flushToDiskRecursive ()
    {
      for (std::size_t i = 0; i < 8; i++)
      {
        if (children_[i])
          children_[i]->flushToDiskRecursive ();
      }
    }

    ////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT> void
    OutofcoreOctreeBaseNode<ContainerT, PointT>::sortOctantIndices (const pcl::PCLPointCloud2::Ptr &input_cloud, std::vector< pcl::Indices > &indices, const Eigen::Vector3d &mid_xyz)
    {
      if (indices.size () < 8)
        indices.resize (8);

      int x_idx = pcl::getFieldIndex (*input_cloud , std::string ("x") );
      int y_idx = pcl::getFieldIndex (*input_cloud, std::string ("y") );
      int z_idx = pcl::getFieldIndex (*input_cloud, std::string ("z") );

      int x_offset = input_cloud->fields[x_idx].offset;
      int y_offset = input_cloud->fields[y_idx].offset;
      int z_offset = input_cloud->fields[z_idx].offset;
      
      for ( std::size_t point_idx =0; point_idx < input_cloud->data.size (); point_idx +=input_cloud->point_step )
      {
        PointT local_pt;

        local_pt.x = * (reinterpret_cast<float*>(&input_cloud->data[point_idx + x_offset]));
        local_pt.y = * (reinterpret_cast<float*>(&input_cloud->data[point_idx + y_offset]));
        local_pt.z = * (reinterpret_cast<float*>(&input_cloud->data[point_idx + z_offset]));

        if (!std::isfinite (local_pt.x) || !std::isfinite (local_pt.y) || !std::isfinite (local_pt.z))
          continue;

        if(!this->pointInBoundingBox (local_pt))
        {
          PCL_ERROR ("pcl::outofcore::OutofcoreOctreeBaseNode::%s] Point %2.lf %.2lf %.2lf not in bounding box\n", __FUNCTION__, local_pt.x, local_pt.y, local_pt.z);
        }
        
        assert (this->pointInBoundingBox (local_pt) == true);

        //compute the box we are in
        std::size_t box = 0;
        box = ((local_pt.z >= mid_xyz[2]) << 2) | ((local_pt.y >= mid_xyz[1]) << 1) | ((local_pt.x >= mid_xyz[0]) << 0);
        assert (box < 8);
              
        //insert to the vector of indices
        indices[box].push_back (static_cast<int> (point_idx/input_cloud->point_step));
      }
    }
    ////////////////////////////////////////////////////////////////////////////////

#if 0  //A bunch of non-class methods left from the Urban Robotics code that has been deactivated
    template<typename ContainerT, typename PointT> OutofcoreOctreeBaseNode<ContainerT, PointT>*
    makenode_norec (const boost::filesystem::path& path, OutofcoreOctreeBaseNode<ContainerT, PointT>* super)
    {
      OutofcoreOctreeBaseNode<ContainerT, PointT>* thisnode = new OutofcoreOctreeBaseNode<OutofcoreOctreeDiskContainer < PointT > , PointT > ();
//octree_disk_node ();

      if (super == NULL)
      {
        thisnode->thisdir_ = path.parent_path ();

        if (!boost::filesystem::exists (thisnode->thisdir_))
        {
          PCL_ERROR ( "[pcl::outofcore::OutofcoreOctreeBaseNode] could not find dir %s\n",thisnode->thisdir_.c_str () );
          PCL_THROW_EXCEPTION (PCLException, "[pcl::outofcore::OutofcoreOctreeBaseNode] Outofcore Octree Exception: Could not find directory");
        }

        thisnode->thisnodeindex_ = path;

        thisnode->depth_ = 0;
        thisnode->root_node_ = thisnode;
      }
      else
      {
        thisnode->thisdir_ = path;
        thisnode->depth_ = super->depth_ + 1;
        thisnode->root_node_ = super->root_node_;

        if (thisnode->depth_ > thisnode->root->max_depth_)
        {
          thisnode->root->max_depth_ = thisnode->depth_;
        }

        boost::filesystem::directory_iterator diterend;
        bool loaded = false;
        for (boost::filesystem::directory_iterator diter (thisnode->thisdir_); diter != diterend; ++diter)
        {
          const boost::filesystem::path& file = *diter;
          if (!boost::filesystem::is_directory (file))
          {
            if (boost::filesystem::extension (file) == OutofcoreOctreeBaseNode<ContainerT, PointT>::node_index_extension)
            {
              thisnode->thisnodeindex_ = file;
              loaded = true;
              break;
            }
          }
        }

        if (!loaded)
        {
          PCL_ERROR ("[pcl::outofcore::OutofcoreOctreeBaseNode] Could not find index!\n");
          PCL_THROW_EXCEPTION (PCLException, "[pcl::outofcore::OutofcoreOctreeBaseNode] Could not find node metadata index file");
        }

      }
      thisnode->max_depth_ = 0;

      {
        std::ifstream f (thisnode->thisnodeindex_.string ().c_str (), std::ios::in);

        f >> thisnode->min_[0];
        f >> thisnode->min_[1];
        f >> thisnode->min_[2];
        f >> thisnode->max_[0];
        f >> thisnode->max_[1];
        f >> thisnode->max_[2];

        std::string filename;
        f >> filename;
        thisnode->thisnodestorage_ = thisnode->thisdir_ / filename;

        f.close ();

        thisnode->payload_.reset (new ContainerT (thisnode->thisnodestorage_));
      }

      thisnode->parent_ = super;
      children_.clear ();
      children_.resize (8, static_cast<OutofcoreOctreeBaseNode<ContainerT, PointT>* > (0));
      thisnode->num_children_ = 0;

      return (thisnode);
    }

    ////////////////////////////////////////////////////////////////////////////////

//accelerate search
    template<typename ContainerT, typename PointT> void
    queryBBIntersects_noload (const boost::filesystem::path& root_node, const Eigen::Vector3d& min, const Eigen::Vector3d& max, const std::uint32_t query_depth, std::list<std::string>& bin_name)
    {
      OutofcoreOctreeBaseNode<ContainerT, PointT>* root = makenode_norec<ContainerT, PointT> (root_node, NULL);
      if (root == NULL)
      {
        std::cout << "test";
      }
      if (root->intersectsWithBoundingBox (min, max))
      {
        if (query_depth == root->max_depth_)
        {
          if (!root->payload_->empty ())
          {
            bin_name.push_back (root->thisnodestorage_.string ());
          }
          return;
        }

        for (int i = 0; i < 8; i++)
        {
          boost::filesystem::path child_dir = root->thisdir_
          / boost::filesystem::path (boost::lexical_cast<std::string> (i));
          if (boost::filesystem::exists (child_dir))
          {
            root->children_[i] = makenode_norec (child_dir, root);
            root->num_children_++;
            queryBBIntersects_noload (root->children_[i], min, max, root->max_depth_ - query_depth, bin_name);
          }
        }
      }
      delete root;
    }

    ////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT> void
    queryBBIntersects_noload (OutofcoreOctreeBaseNode<ContainerT, PointT>* current, const Eigen::Vector3d& min, const Eigen::Vector3d& max, const std::uint32_t query_depth, std::list<std::string>& bin_name)
    {
      if (current->intersectsWithBoundingBox (min, max))
      {
        if (current->depth_ == query_depth)
        {
          if (!current->payload_->empty ())
          {
            bin_name.push_back (current->thisnodestorage_.string ());
          }
        }
        else
        {
          for (int i = 0; i < 8; i++)
          {
            boost::filesystem::path child_dir = current->thisdir_ / boost::filesystem::path (boost::lexical_cast<std::string> (i));
            if (boost::filesystem::exists (child_dir))
            {
              current->children_[i] = makenode_norec<ContainerT, PointT> (child_dir, current);
              current->num_children_++;
              queryBBIntersects_noload (current->children_[i], min, max, query_depth, bin_name);
            }
          }
        }
      }
    }
#endif
    ////////////////////////////////////////////////////////////////////////////////

  }//namespace outofcore
}//namespace pcl

//#define PCL_INSTANTIATE....

#endif //PCL_OUTOFCORE_OCTREE_BASE_NODE_IMPL_H_
