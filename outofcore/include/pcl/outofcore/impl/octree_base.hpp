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

// PCL (Urban Robotics)
#include <pcl/outofcore/octree_base.h>

// JSON
#include <pcl/outofcore/cJSON.h>

// C++
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <exception>

// Boost
#include <boost/thread.hpp>
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>

namespace pcl
{
  namespace outofcore
  {

    //Static constants
    // --------------------------------------------------------------------------------
    template<typename ContainerT, typename PointT>
    const std::string octree_base<ContainerT, PointT>::TREE_EXTENSION_ = ".octree";

    template<typename ContainerT, typename PointT>
    const int octree_base<ContainerT, PointT>::OUTOFCORE_VERSION_ = static_cast<int>(3);

// Constructors
// ---------------------------------------------------------------------------
    template<typename ContainerT, typename PointT>
    octree_base<ContainerT, PointT>::octree_base (const boost::filesystem::path& root_name, const bool load_all)
      : root_ ()
      , read_write_mutex_ ()
      , lodPoints_ ()
      , max_depth_ ()
      , treepath_ ()
      , coord_system_ ()
      , resolution_ ()
    {
      // Check file extension
      if (boost::filesystem::extension (root_name) != octree_base_node<ContainerT, PointT>::node_index_extension)
      {
        PCL_ERROR ( "[pcl::outofcore::octree_base] Wrong root node file extension: %s. The tree must have a root node ending in %s\n", boost::filesystem::extension (root_name).c_str (), octree_base_node<ContainerT, PointT>::node_index_extension.c_str () );
        PCL_THROW_EXCEPTION (PCLException, "[pcl::outofcore::octree_base] Bad extension. Tree must have a root node ending in .oct_idx\n");
      }

      // Create root_ node
      root_ = new octree_base_node<ContainerT, PointT> (root_name, NULL, load_all);

      // Set root_ nodes tree to the newly created tree
      root_->m_tree_ = this;

      // Set root_ nodes file path
      treepath_ = root_name.parent_path () / (boost::filesystem::basename (root_name) + TREE_EXTENSION_);

      loadFromFile ();
    }
////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT>
    octree_base<ContainerT, PointT>::octree_base (const Eigen::Vector3d& min, const Eigen::Vector3d& max, const double node_dim_meters, const boost::filesystem::path& root_name, const std::string& coord_sys)
      : root_ ()
      , read_write_mutex_ ()
      , lodPoints_ ()
      , max_depth_ ()
      , treepath_ ()
      , coord_system_ ()
      , resolution_ ()
    {
      if (boost::filesystem::exists (root_name.parent_path ()))
      {
        PCL_ERROR ("[pcl::outofcore::octree_base] A dir named %s already exists. Overwriting an existing tree is not supported.\n", root_name.parent_path ().c_str () );
        PCL_THROW_EXCEPTION ( PCLException, "[pcl::outofcore::octree_base] Directory exists; Overwriting an existing tree is not supported\n");
      }

     // Check file extension
      if (boost::filesystem::extension (root_name) != octree_base_node<ContainerT, PointT>::node_index_extension)
      {
        PCL_ERROR ( "[pcl::outofcore::octree_base] The tree must be created with a root node ending in .oct_idx\n" );
        PCL_THROW_EXCEPTION (PCLException, "[pcl::outofcore::octree_base] Root file extension does not match .oct_idx\n");
      }

      ///\todo Incorporate arbitrary optional metadata for the tree (new PCD file type might be useful here)
      coord_system_ = coord_sys;

      // Get fullpath and recreate directories
      boost::filesystem::path dir = boost::filesystem::system_complete (root_name.parent_path ());
      boost::filesystem::remove_all (dir);
      boost::filesystem::create_directory (dir);

      // Create root_ node
      root_ = new octree_base_node<ContainerT, PointT> (min, max, node_dim_meters, this, root_name);
      root_->m_tree_ = this;
      root_->saveIdx (false);

      // max_depth_ is set when creating the root_ node
      lodPoints_.resize (max_depth_ + 1);

      // Set root_ nodes file path
      treepath_ = dir / (boost::filesystem::basename (root_name) + TREE_EXTENSION_);
      this->saveToFile ();
    }
////////////////////////////////////////////////////////////////////////////////

// todo: Both constructs share the same code except for a single line
    template<typename ContainerT, typename PointT>
    octree_base<ContainerT, PointT>::octree_base (const int max_depth, const Eigen::Vector3d& min, const Eigen::Vector3d& max, const boost::filesystem::path& root_name, const std::string& coord_sys)
      : root_ ()
      , read_write_mutex_ ()
      , lodPoints_ ()
      , max_depth_ ()
      , treepath_ ()
      , coord_system_ ()
      , resolution_ ()
    {
      // Check file extension
      if (boost::filesystem::extension (root_name) != octree_base_node<ContainerT, PointT>::node_index_extension)
      {
        PCL_ERROR ( "[pcl::outofcore::octree_base] the tree must be created with a root_ node ending in .oct_idx\n" );
        PCL_THROW_EXCEPTION (PCLException, "[pcl::outofcore::octree_base] Bad extension. Tree must be created with node ending in .oct_idx\n");
      }

      coord_system_ = coord_sys;

      // Get fullpath and recreate directories
      boost::filesystem::path dir = root_name.parent_path ();

      if (!boost::filesystem::exists (dir))
      {
        boost::filesystem::create_directory (dir);
      }

      // todo: Why is overwriting an existing tree not permitted when setting the
      // max depth, but supported when setting node dims?
      for (int i = 0; i < 8; i++)
      {
        boost::filesystem::path childdir = dir / boost::lexical_cast<std::string> (i);
        if (boost::filesystem::exists (childdir))
        {
          PCL_ERROR ("[pcl::outofcore::octree_base] A dir named %d exists under the root node. Overwriting an existing tree is not supported.\n", i);
          PCL_THROW_EXCEPTION ( PCLException, "[pcl::outofcore::octree_base] Directory exists; Overwriting an existing tree is not supported\n");
        }
      }

      // Create root node
      root_ = new octree_base_node<ContainerT, PointT> (max_depth, min, max, this, root_name);
      root_->saveIdx (false);

      // max_depth_ is set when creating the root_ node
      lodPoints_.resize (max_depth_ + 1);

      // Set root nodes file path
      treepath_ = dir / (boost::filesystem::basename (root_name) + TREE_EXTENSION_);
      saveToFile ();
    }
////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT>
    octree_base<ContainerT, PointT>::~octree_base ()
    {
      root_->flushToDisk ();
      root_->saveIdx (false);
      saveToFile ();
      delete root_;
    }
////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT> void
    octree_base<ContainerT, PointT>::saveToFile ()
    {
      // Create JSON object
      boost::shared_ptr<cJSON> idx (cJSON_CreateObject (), cJSON_Delete);
  
      cJSON* name = cJSON_CreateString ("test");
      cJSON* version = cJSON_CreateNumber ( OUTOFCORE_VERSION_ );
      cJSON* pointtype = cJSON_CreateString ("urp");
      cJSON* lod = cJSON_CreateNumber (static_cast<double>(root_->m_tree_->max_depth_));

      // cJSON does not allow 64 bit ints.  Have to put the points in a double to
      // use this api, will allow counts up to 2^52 points to be stored correctly
      std::vector<double> lodPoints_db;
      lodPoints_db.insert (lodPoints_db.begin (), lodPoints_.begin (), lodPoints_.end ());

      cJSON* numpts = cJSON_CreateDoubleArray ( &(lodPoints_db.front ()), static_cast<int>(lodPoints_db.size ()) );

      cJSON_AddItemToObject (idx.get (), "name", name);
      cJSON_AddItemToObject (idx.get (), "version", version);
      cJSON_AddItemToObject (idx.get (), "pointtype", pointtype);
      cJSON_AddItemToObject (idx.get (), "lod", lod);
      cJSON_AddItemToObject (idx.get (), "numpts", numpts);
      cJSON_AddStringToObject(idx.get(), "coord_system", coord_system_.c_str());

      char* idx_txt = cJSON_Print (idx.get ());

      std::ofstream f (treepath_.string ().c_str (), std::ios::out | std::ios::trunc);
      f << idx_txt;
      f.close ();

      free (idx_txt);
    }
////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT> void
    octree_base<ContainerT, PointT>::loadFromFile ()
    {
      // Open JSON
      std::vector<char> idx_input;
      boost::uintmax_t len = boost::filesystem::file_size (treepath_);
      idx_input.resize (len + 1);

      std::ifstream f (treepath_.string ().c_str (), std::ios::in);
      f.read (&(idx_input.front ()), len);
      idx_input.back () = '\0';

      // Parse JSON
      boost::shared_ptr<cJSON> idx (cJSON_Parse (&(idx_input.front ())), cJSON_Delete);
      cJSON* name = cJSON_GetObjectItem (idx.get (), "name");
      cJSON* version = cJSON_GetObjectItem (idx.get (), "version");
      cJSON* pointtype = cJSON_GetObjectItem (idx.get (), "pointtype");
      cJSON* lod = cJSON_GetObjectItem (idx.get (), "lod");
      cJSON* numpts = cJSON_GetObjectItem (idx.get (), "numpts");
      cJSON* coord = cJSON_GetObjectItem (idx.get (), "coord_system");

      // Validate JSON
      if (!((name) && (version) && (pointtype) && (lod) && (numpts) && (coord)))
      {
        PCL_ERROR ( "index %s failed to parse!\n", treepath_.c_str ());
        PCL_THROW_EXCEPTION (PCLException, "Outofcore Octree Parse Failure\n");
      }
      if ((name->type != cJSON_String) || (version->type != cJSON_Number) || (pointtype->type != cJSON_String)
          || (lod->type != cJSON_Number) || (numpts->type != cJSON_Array) || (coord->type != cJSON_String))
      {
        PCL_ERROR ( "index failed to parse!\n",treepath_.c_str ());
        PCL_THROW_EXCEPTION (PCLException, "Outofcore Octree Exception: Index failed to parse\n");
      }
      if (version->valuedouble != 2.0 && version->valuedouble != 3.0)// || version->valuedouble != 3.0)
      {
        PCL_ERROR ( "index failed to parse!\n",treepath_.c_str ());
        PCL_THROW_EXCEPTION (PCLException, "Outofcore Octree Parse Failure: Incompatible Version of Outofcore Octree\n");
        
      }
      if ((lod->valueint + 1) != cJSON_GetArraySize (numpts))
      {
        PCL_ERROR ( "[pcl::outofcore::octree_base] Index failed to parse: %s\n",treepath_.c_str ());
        PCL_THROW_EXCEPTION (PCLException, "Outofcore Octree Prase Failure: LOD and array size of points is not valid\n");
      }

      // Get Data
      lodPoints_.resize (lod->valueint + 1);
      for (int i = 0; i < (lod->valueint + 1); i++)
      {
        //cJSON doesn't have explicit 64bit int, have to use double, get up to 2^52
        lodPoints_[i] = static_cast<boost::uint64_t> (cJSON_GetArrayItem (numpts, i)->valuedouble );
      }
      max_depth_ = lod->valueint;
      coord_system_ = coord->valuestring;
    }
////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT> boost::uint64_t
    octree_base<ContainerT, PointT>::addDataToLeaf (const AlignedPointTVector& p)
    {
      boost::unique_lock < boost::shared_mutex > lock (read_write_mutex_);

      const bool _FORCE_BB_CHECK = true;
      
      uint64_t pt_added = root_->addDataToLeaf (p, _FORCE_BB_CHECK);

      assert (p.size () == pt_added);

      return (pt_added);
    }
////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT> boost::uint64_t
    octree_base<ContainerT, PointT>::addPointCloud (PointCloudConstPtr point_cloud)
    {
      return addDataToLeaf (point_cloud->points);
    }
    
////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT> boost::uint64_t
    octree_base<ContainerT, PointT>::addPointCloud (sensor_msgs::PointCloud2::Ptr input_cloud, const bool skip_bb_check = false)
    {
      uint64_t pt_added = this->root_->addPointCloud ( input_cloud, skip_bb_check ) ;
      assert ( input_cloud->width*input_cloud->height == pt_added );
      return (pt_added);
    }

    
////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT> boost::uint64_t
    octree_base<ContainerT, PointT>::addPointCloud_and_genLOD (PointCloudConstPtr point_cloud)
    {
      // Lock the tree while writing
      boost::unique_lock < boost::shared_mutex > lock (read_write_mutex_);
      boost::uint64_t pt_added = root_->addDataToLeaf_and_genLOD (point_cloud->points, false);
      return (pt_added);
    }

////////////////////////////////////////////////////////////////////////////////
    template<typename ContainerT, typename PointT> boost::uint64_t
    octree_base<ContainerT, PointT>::addPointCloud_and_genLOD (sensor_msgs::PointCloud2::Ptr input_cloud)
    {
      // Lock the tree while writing
      boost::unique_lock < boost::shared_mutex > lock (read_write_mutex_);
      boost::uint64_t pt_added = root_->addPointCloud_and_genLOD ( input_cloud );
      
//      PCL_INFO ("[pcl::outofcore::octree_base::%s] Points added %lu, points in input cloud, %lu\n",__FUNCTION__, pt_added, input_cloud->width*input_cloud->height );
 
      assert ( input_cloud->width*input_cloud->height == pt_added );

      return (pt_added);
    }



////////////////////////////////////////////////////////////////////////////////
#if 0
    template<typename ContainerT, typename PointT> void
    octree_base<ContainerT, PointT>::addPointToLeaf (const PointT& src)
    {
      boost::unique_lock < boost::shared_mutex > lock (read_write_mutex_);
      root_->addPointToLeaf (src, false);
    }
#endif    

////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT> boost::uint64_t
    octree_base<ContainerT, PointT>::addDataToLeaf_and_genLOD (AlignedPointTVector& src)
    {
      // Lock the tree while writing
      boost::unique_lock < boost::shared_mutex > lock (read_write_mutex_);
      boost::uint64_t pt_added = root_->addDataToLeaf_and_genLOD (src, false);
      return (pt_added);
    }
////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT> void
    octree_base<ContainerT, PointT>::queryBBIncludes (const Eigen::Vector3d& min, const Eigen::Vector3d& max, const size_t query_depth, AlignedPointTVector& dst) const
    {
      boost::shared_lock < boost::shared_mutex > lock (read_write_mutex_);
      dst.clear ();
      root_->queryBBIncludes (min, max, query_depth, dst);
    }
////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT> void
    octree_base<ContainerT, PointT>::queryBBIncludes_subsample (const Eigen::Vector3d& min, const Eigen::Vector3d& max, const size_t query_depth, const double percent, AlignedPointTVector& dst) const
    {
      boost::shared_lock < boost::shared_mutex > lock (read_write_mutex_);
      dst.clear ();
      root_->queryBBIncludes_subsample (min, max, query_depth, percent, dst);
    }

////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT> void
    octree_base<ContainerT, PointT>::printBBox(const size_t query_depth) const
    {
      boost::shared_lock < boost::shared_mutex > lock (read_write_mutex_);
      root_->printBBox (query_depth);
    }

////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT> void
    octree_base<ContainerT, PointT>::getVoxelCenters(AlignedPointTVector &voxel_centers, const size_t query_depth) const
    {
      boost::shared_lock < boost::shared_mutex > lock (read_write_mutex_);
      if (query_depth > max_depth_) 
      {
        root_->getVoxelCenters (voxel_centers, max_depth_);
      }
      else
      {
        root_->getVoxelCenters (voxel_centers, query_depth);
      }
    }

    template<typename ContainerT, typename PointT> void
    octree_base<ContainerT, PointT>::getVoxelCenters(std::vector<Eigen::Vector3d> &voxel_centers, const size_t query_depth) const
    {
      boost::shared_lock < boost::shared_mutex > lock (read_write_mutex_);
      if (query_depth > max_depth_)
      {
        root_->getVoxelCenters (voxel_centers, max_depth_);
      }
      else
      {
        root_->getVoxelCenters (voxel_centers, query_depth);
      }
    }

////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT> void
    octree_base<ContainerT, PointT>::queryBBIntersects (const Eigen::Vector3d& min, const Eigen::Vector3d& max, const boost::uint32_t query_depth, std::list<std::string>& bin_name) const
    {
      boost::shared_lock < boost::shared_mutex > lock (read_write_mutex_);
      bin_name.clear ();
#pragma warning(push)
#pragma warning(disable : 4267)
      root_->queryBBIntersects (min, max, query_depth, bin_name);
#pragma warning(pop)
    }
////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT> void
    octree_base<ContainerT, PointT>::writeVPythonVisual (const boost::filesystem::path filename)
    {
      std::ofstream f (filename.c_str ());

      f << "from visual import *\n\n";

      root_->writeVPythonVisual (f);
    }
////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT> void
    octree_base<ContainerT, PointT>::flushToDisk ()
    {
      root_->flushToDisk ();
    }
////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT> void
    octree_base<ContainerT, PointT>::flushToDiskLazy ()
    {
      root_->flushToDiskLazy ();
    }
////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT> void
    octree_base<ContainerT, PointT>::saveIdx ()
    {
      root_->saveIdx (true);
    }
////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT> void
    octree_base<ContainerT, PointT>::convertToXYZ ()
    {
      saveToFile ();
      root_->convertToXYZ ();
    }
////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT> void
    octree_base<ContainerT, PointT>::DeAllocEmptyNodeCache ()
    {
      DeAllocEmptyNodeCache (root_ );
    }
////////////////////////////////////////////////////////////////////////////////

    template<typename ContainerT, typename PointT> void
    octree_base<ContainerT, PointT>::DeAllocEmptyNodeCache (octree_base_node<ContainerT, PointT>* current)
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

    template<typename ContainerT, typename PointT> void
    octree_base<ContainerT, PointT>::buildLOD ()
    {
      if (root_ == NULL)
      {
        PCL_ERROR ( "root is null, aborting buildLOD\n" );
        return;
      }
      boost::unique_lock < boost::shared_mutex > lock (read_write_mutex_);

      const int current_dims = 1;
      octree_base_node<ContainerT, PointT>* current_branch[current_dims] = {root_};
      this->buildLOD (current_branch, current_dims);
    }
////////////////////////////////////////////////////////////////////////////////

//loads chunks of up to 2e9 pts at a time; this is a completely arbitrary number
//TODO rewrite for new point container (PointCloud2) support
    template<typename ContainerT, typename PointT> void
    octree_base<ContainerT, PointT>::buildLOD (octree_base_node<ContainerT, PointT>** current_branch, const int current_dims)
    {
      //stop if this brach DNE
      if (!current_branch[current_dims - 1])
      {
        return;
      }

      if ((current_branch[current_dims - 1]->numchildren () == 0)
          && (!current_branch[current_dims - 1]->hasUnloadedChildren ()))//at leaf: subsample, remove, and copy to higher nodes
      {
        //this node's idx is (current_dims-1)
        octree_base_node<ContainerT, PointT>* leaf = current_branch[current_dims - 1];

        boost::uint64_t leaf_start_size = leaf->payload->size ();
        if (leaf_start_size > 0)//skip empty
        {
          for (boost::uint64_t startp = 0; startp < leaf_start_size; startp += LOAD_COUNT_)
          {
            //there are (current_dims-1) nodes above this one, indexed 0 thru (current_dims-2)
            for (size_t level = (current_dims - 1); level >= 1; level--)
            {
              //the target
              octree_base_node<ContainerT, PointT>* target_parent = current_branch[level - 1];

              //the percent to copy
              //each level up the chain gets sample_precent^l of the leaf's data
              double percent = pow (double (octree_base_node<ContainerT, PointT>::sample_precent), double (current_dims - level));

              //read in percent of node
              AlignedPointTVector v;
              if ((startp + LOAD_COUNT_) < leaf_start_size)
              {
                leaf->payload->readRangeSubSample (startp, LOAD_COUNT_, percent, v);
              }
              else
              {
                leaf->payload->readRangeSubSample (startp, leaf_start_size - startp, percent, v);
              }

              //write to the target
              if (!v.empty ())
              {
                target_parent->payload->insertRange ( v );
//                target_parent->payload->insertRange (&(v.front ()), v.size ());
                this->incrementPointsInLOD (target_parent->depth, v.size ());
              }

            }
          }
        }
      }
      else//not at leaf, keep going down
      {
        //clear this node, in case we are updating the LOD
        current_branch[current_dims - 1]->payload->clear ();

        const int next_dims = current_dims + 1;
        octree_base_node<ContainerT, PointT>** next_branch = new octree_base_node<ContainerT, PointT>*[next_dims];
        memcpy (next_branch, current_branch, current_dims * sizeof(octree_base_node<ContainerT, PointT>**));

        size_t numchild = current_branch[current_dims - 1]->numchildren ();
        if ((numchild != 8) && (current_branch[current_dims - 1]->hasUnloadedChildren ()))
        {
          current_branch[current_dims - 1]->loadChildren (false);
          numchild = current_branch[current_dims - 1]->numchildren ();
        }

        for (size_t i = 0; i < numchild; i++)
        {
          next_branch[next_dims - 1] = next_branch[current_dims - 1]->children[i];
          buildLOD (next_branch, next_dims);
        }

        delete[] next_branch;
      }
    }
  }//namespace outofcore
}//namespace pcl

#endif //PCL_OUTOFCORE_OCTREE_BASE_IMPL_H_
