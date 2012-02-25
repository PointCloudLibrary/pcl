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
#ifndef PCL_OUTOFCORE_OCTREE_BASE_IMPL_H_
#define PCL_OUTOFCORE_OCTREE_BASE_IMPL_H_

// PCL (Urban Robotics)
#include "pcl/outofcore/octree_base.h"
#include "pcl/outofcore/octree_exceptions.h"

// JSON
#include "pcl/outofcore/cJSON.h"

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

    template<typename Container, typename PointT>
    const std::string octree_base<Container, PointT>::tree_extension_ = ".octree";

// Constructors
// ---------------------------------------------------------------------------
    template<typename Container, typename PointT>
    octree_base<Container, PointT>::octree_base (const boost::filesystem::path& rootname, const bool loadAll)
    {
      // Check file extension
      if (boost::filesystem::extension (rootname) != octree_base_node<Container, PointT>::node_index_extension)
      {
        std::cerr << "the tree must be have a root_ node ending in .oct_idx" << std::endl;
        throw(OctreeException (OctreeException::OCT_BAD_EXTENTION));
      }

      // Create root_ node
      root_ = new octree_base_node<Container, PointT> (rootname, NULL, loadAll);

      // Set root_ nodes tree to the newly created tree
      root_->m_tree_ = this;

      // Set root_ nodes file path
      treepath_ = rootname.parent_path () / (boost::filesystem::basename (rootname) + tree_extension_);

      loadFromFile ();
    }
////////////////////////////////////////////////////////////////////////////////

    template<typename Container, typename PointT>
    octree_base<Container, PointT>::octree_base (const double min[3], const double max[3], const double node_dim_meters, const boost::filesystem::path& rootname, const std::string& coord_sys)
    {
      // Check file extension
      if (boost::filesystem::extension (rootname) != octree_base_node<Container, PointT>::node_index_extension)
      {
        std::cerr << "the tree must be created with a root_ node ending in .oct_idx" << std::endl;
        throw(OctreeException (OctreeException::OCT_BAD_EXTENTION));
      }

      coord_system_ = coord_sys;

      // Get fullpath and recreate directories
      boost::filesystem::path dir = boost::filesystem::system_complete (rootname.parent_path ());
      boost::filesystem::remove_all (dir);
      boost::filesystem::create_directory (dir);

      // Create root_ node
      root_ = new octree_base_node<Container, PointT> (min, max, node_dim_meters, this, rootname);
      root_->m_tree_ = this;
      root_->saveIdx (false);

      // maxDepth_ is set when creating the root_ node
      lodPoints_.resize (maxDepth_ + 1);

      // Set root_ nodes file path
      treepath_ = dir / (boost::filesystem::basename (rootname) + tree_extension_);
      saveToFile ();
    }
////////////////////////////////////////////////////////////////////////////////

// todo: Both constructs share the same code except for a single line
    template<typename Container, typename PointT>
    octree_base<Container, PointT>::octree_base (const int maxdepth, const double min[3], const double max[3], const boost::filesystem::path& rootname, const std::string& coord_sys)
    {
      // Check file extension
      if (boost::filesystem::extension (rootname) != octree_base_node<Container, PointT>::node_index_extension)
      {
        std::cerr << "the tree must be created with a root_ node ending in .oct_idx" << std::endl;
        throw(OctreeException::OCT_BAD_EXTENTION);
      }

      coord_system_ = coord_sys;

      // Get fullpath and recreate directories
      boost::filesystem::path dir = rootname.parent_path ();

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
          std::cerr << "A dir named " << i
                    << " exists under the root_ node. Overwriting an existant tree is not supported!";
          throw(OctreeException (OctreeException::OCT_CHILD_EXISTS));
        }
      }

      // Create root node
      root_ = new octree_base_node<Container, PointT> (maxdepth, min, max, this, rootname);
      root_->saveIdx (false);

      // maxDepth_ is set when creating the root_ node
      lodPoints_.resize (maxDepth_ + 1);

      // Set root nodes file path
      treepath_ = dir / (boost::filesystem::basename (rootname) + tree_extension_);
      saveToFile ();
    }
////////////////////////////////////////////////////////////////////////////////

    template<typename Container, typename PointT>
    octree_base<Container, PointT>::~octree_base ()
    {
      root_->flushToDisk ();
      root_->saveIdx (false);
      saveToFile ();
      delete root_;
    }
////////////////////////////////////////////////////////////////////////////////

    template<typename Container, typename PointT> void
    octree_base<Container, PointT>::saveToFile ()
    {
      // Create JSON object
      boost::shared_ptr<cJSON> idx (cJSON_CreateObject (), cJSON_Delete);
  
      cJSON* name = cJSON_CreateString ("test");
      cJSON* version = cJSON_CreateNumber (2.0);
      cJSON* pointtype = cJSON_CreateString ("urp");
      cJSON* lod = cJSON_CreateNumber (root_->m_tree_->maxDepth_);

      // cJSON does not allow 64 bit ints.  Have to put the points in a double to
      // use this api, will allow counts up to 2^52 points to be stored correctly
      std::vector<double> lodPoints_db;
      lodPoints_db.insert (lodPoints_db.begin (), lodPoints_.begin (), lodPoints_.end ());
      cJSON* numpts = cJSON_CreateDoubleArray (&(lodPoints_db.front ()), lodPoints_db.size ());

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

    template<typename Container, typename PointT> void
    octree_base<Container, PointT>::loadFromFile ()
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
        std::cerr << "index " << treepath_ << " failed to parse!" << std::endl;
        throw OctreeException (OctreeException::OCT_PARSE_FAILURE);
      }
      if ((name->type != cJSON_String) || (version->type != cJSON_Number) || (pointtype->type != cJSON_String)
          || (lod->type != cJSON_Number) || (numpts->type != cJSON_Array) || (coord->type != cJSON_String))
      {
        std::cerr << "index " << treepath_ << " failed to parse!" << std::endl;
        throw OctreeException (OctreeException::OCT_PARSE_FAILURE);
      }
      if (version->valuedouble != 2.0)
      {
        std::cerr << "index " << treepath_ << " failed to parse!" << std::endl;
        throw OctreeException (OctreeException::OCT_PARSE_FAILURE);
      }
      if ((lod->valueint + 1) != cJSON_GetArraySize (numpts))
      {
        std::cerr << "index " << treepath_ << " failed to parse!" << std::endl;
        throw OctreeException (OctreeException::OCT_PARSE_FAILURE);
      }

      // Get Data
      lodPoints_.resize (lod->valueint + 1);
      for (int i = 0; i < (lod->valueint + 1); i++)
      {
        //cJSON doesn't have explicit 64bit int, have to use double, get up to 2^52
        lodPoints_[i] = boost::uint64_t (cJSON_GetArrayItem (numpts, i)->valuedouble);
      }
      maxDepth_ = lod->valueint;
      coord_system_ = coord->valuestring;
    }
////////////////////////////////////////////////////////////////////////////////

    template<typename Container, typename PointT> boost::uint64_t
    octree_base<Container, PointT>::addDataToLeaf (const std::vector<PointT>& p)
    {
      boost::unique_lock < boost::shared_mutex > lock (read_write_mutex_);
      boost::uint64_t pt_added = root_->addDataToLeaf (p, false);
      return (pt_added);
    }
////////////////////////////////////////////////////////////////////////////////

    template<typename Container, typename PointT> boost::uint64_t
    octree_base<Container, PointT>::addDataToLeaf_and_genLOD (const std::vector<PointT>& p)
    {
      // Lock the tree while writing
      boost::unique_lock < boost::shared_mutex > lock (read_write_mutex_);
      boost::uint64_t pt_added = root_->addDataToLeaf_and_genLOD (p, false);
      return (pt_added);
    }
////////////////////////////////////////////////////////////////////////////////

    template<typename Container, typename PointT> void
    octree_base<Container, PointT>::queryBBIncludes (const double min[3], const double max[3], size_t query_depth, std::list<PointT>& v) const
    {
      boost::shared_lock < boost::shared_mutex > lock (read_write_mutex_);
      v.clear ();
      root_->queryBBIncludes (min, max, query_depth, v);
    }
////////////////////////////////////////////////////////////////////////////////

    template<typename Container, typename PointT> void
    octree_base<Container, PointT>::queryBBIncludes_subsample (const double min[3], const double max[3],
                                                               size_t query_depth, const double percent,
                                                               std::list<PointT>& v) const
    {
      boost::shared_lock < boost::shared_mutex > lock (read_write_mutex_);
      v.clear ();
      root_->queryBBIncludes_subsample (min, max, query_depth, percent, v);
    }
////////////////////////////////////////////////////////////////////////////////

    template<typename Container, typename PointT> void
    octree_base<Container, PointT>::queryBBIntersects (const double min[3], const double max[3],
                                                       const boost::uint32_t query_depth,
                                                       std::list<std::string>& bin_name) const
    {
      boost::shared_lock < boost::shared_mutex > lock (read_write_mutex_);
      bin_name.clear ();
#pragma warning(push)
#pragma warning(disable : 4267)
      root_->queryBBIntersects (min, max, query_depth, bin_name);
#pragma warning(pop)
    }
////////////////////////////////////////////////////////////////////////////////

    template<typename Container, typename PointT> void
    octree_base<Container, PointT>::writeVPythonVisual (const char* file)
    {
      std::ofstream f (file);

      f << "from visual import *\n\n";

      root_->writeVPythonVisual (f);
    }
////////////////////////////////////////////////////////////////////////////////

    template<typename Container, typename PointT> void
    octree_base<Container, PointT>::flushToDisk ()
    {
      root_->flushToDisk ();
    }
////////////////////////////////////////////////////////////////////////////////

    template<typename Container, typename PointT> void
    octree_base<Container, PointT>::flushToDiskLazy ()
    {
      root_->flushToDiskLazy ();
    }
////////////////////////////////////////////////////////////////////////////////

    template<typename Container, typename PointT> void
    octree_base<Container, PointT>::saveIdx ()
    {
      root_->saveIdx (true);
    }
////////////////////////////////////////////////////////////////////////////////

    template<typename Container, typename PointT> void
    octree_base<Container, PointT>::convertToXYZ ()
    {
      saveToFile ();
      root_->convertToXYZ ();
    }
////////////////////////////////////////////////////////////////////////////////

    template<typename Container, typename PointT> void
    octree_base<Container, PointT>::DeAllocEmptyNodeCache ()
    {
      DeAllocEmptyNodeCache (root_ );
    }
////////////////////////////////////////////////////////////////////////////////

    template<typename Container, typename PointT> void
    octree_base<Container, PointT>::DeAllocEmptyNodeCache (octree_base_node<Container, PointT>* current)
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

    template<typename Container, typename PointT> void
    octree_base<Container, PointT>::buildLOD ()
    {
      if (root_ == NULL)
      {
        std::cerr << "root is null, aborting buildLOD" << std::endl;
        return;
      }
      boost::unique_lock < boost::shared_mutex > lock (read_write_mutex_);

      const int current_dims = 1;
      octree_base_node<Container, PointT>* current_branch[current_dims] = {root_};
      buildLOD (current_branch, current_dims);
    }
////////////////////////////////////////////////////////////////////////////////

//loads chunks of up to 2e7 pts at a time
    template<typename Container, typename PointT> void
    octree_base<Container, PointT>::buildLOD (octree_base_node<Container, PointT>** current_branch,
                                              const int current_dims)
    {
      //stop if this brach DNE
      if (!current_branch[current_dims - 1])
      {
        return;
      }

      static const boost::uint64_t loadcount = boost::uint64_t (2e7);
      if ((current_branch[current_dims - 1]->numchildren () == 0)
          && (!current_branch[current_dims - 1]->hasUnloadedChildren ()))//at leaf: subsample, remove, and copy to higher nodes
      {
        //this node's idx is (current_dims-1)
        octree_base_node<Container, PointT>* leaf = current_branch[current_dims - 1];

        boost::uint64_t leaf_start_size = leaf->payload->size ();
        if (leaf_start_size > 0)//skip empty
        {
          for (boost::uint64_t startp = 0; startp < leaf_start_size; startp += loadcount)//load up to 5e7 pts at a time
          {
            //there are (current_dims-1) nodes above this one, indexed 0 thru (current_dims-2)
            for (size_t level = (current_dims - 1); level >= 1; level--)
            {
              //the target
              octree_base_node<Container, PointT>* target_parent = current_branch[level - 1];

              //the percent to copy
              double percent = pow (double (octree_base_node<Container, PointT>::sample_precent),
                                    double (current_dims - level));//each level up the chain gets sample_precent^l of the leaf's data

              //read in percent of node
              std::vector<PointT> v;
              if ((startp + loadcount) < leaf_start_size)
              {
                leaf->payload->readRangeSubSample (startp, loadcount, percent, v);
              }
              else
              {
                leaf->payload->readRangeSubSample (startp, leaf_start_size - startp, percent, v);
              }

              //write to the target
              if (!v.empty ())
              {
                target_parent->payload->insertRange (&(v.front ()), v.size ());
                this->count_point (target_parent->depth, v.size ());
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
        octree_base_node<Container, PointT>** next_branch = new octree_base_node<Container, PointT>*[next_dims];
        memcpy (next_branch, current_branch, current_dims * sizeof(octree_base_node<Container, PointT>**));

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
////////////////////////////////////////////////////////////////////////////////
#if 0
    template<typename Container, typename PointT> boost::uint64_t
    octree_base<Container, PointT>::addPointsFromInputCloud ()
    {

/*
  boost::unique_lock < boost::shared_mutex > lock (read_write_mutex_);
  boost::uint64_t pt_added = root_->addDataToLeaf ( input_->points, false );

  return pt_added;
*/

      //ensure the octree is empty

      //if indicies is not null, then add by list of indices
      //otherwise, add all the points in the cloud to the octree
    }
#endif
////////////////////////////////////////////////////////////////////////////////

  }//namespace outofcore
}//namespace pcl

#endif //PCL_OUTOFCORE_OCTREE_BASE_IMPL_H_
