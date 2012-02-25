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
#ifndef PCL_OCTREE_BASE_NODE_IMPL_H_
#define PCL_OCTREE_BASE_NODE_IMPL_H_

// C++
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <exception>

// Boost
#include <boost/thread.hpp>
#include <boost/filesystem.hpp>

// PCL (Urban Robotics)
#include "pcl/outofcore/octree_base_node.h"
#include "pcl/outofcore/octree_exceptions.h"

// JSON
#include <pcl/outofcore/cJSON.h>

namespace pcl
{
  namespace outofcore
  {

    template<typename Container, typename PointT>
    const std::string octree_base_node<Container, PointT>::node_index_basename = "node";

    template<typename Container, typename PointT>
    const std::string octree_base_node<Container, PointT>::node_container_basename = "node";

    template<typename Container, typename PointT>
    const std::string octree_base_node<Container, PointT>::node_index_extension = ".oct_idx";

    template<typename Container, typename PointT>
    const std::string octree_base_node<Container, PointT>::node_container_extension = ".oct_dat";

    template<typename Container, typename PointT>
    boost::mutex octree_base_node<Container, PointT>::rng_mutex_;

    template<typename Container, typename PointT>
    boost::mt19937 octree_base_node<Container, PointT>::rand_gen_;

    template<typename Container, typename PointT>
    const double octree_base_node<Container, PointT>::sample_precent = .125;

    template<typename Container, typename PointT>
    octree_base_node<Container, PointT>::octree_base_node (const boost::filesystem::path& path, octree_base_node<Container, PointT>* super, bool loadAll)
    {
      if (super == NULL)
      {
        thisdir_ = path.parent_path ();

        if (!boost::filesystem::exists (thisdir_))
        {
          std::cerr << "could not find dir!" << thisdir_ << "\n";
          throw(OctreeException::OCT_MISSING_DIR);
        }

        thisnodeindex_ = path;

        depth_ = 0;
        root_ = this;
      }
      else
      {
        thisdir_ = path;
        depth_ = super->depth_ + 1;
        root_ = super->root_;

        boost::filesystem::directory_iterator diterend;
        bool loaded = false;
        for (boost::filesystem::directory_iterator diter (thisdir_); diter != diterend; ++diter)
        {
          const boost::filesystem::path& file = *diter;
          if (!boost::filesystem::is_directory (file))
          {
            if (boost::filesystem::extension (file) == node_index_extension)
            {
              thisnodeindex_ = file;
              loaded = true;
              break;
            }
          }
        }

        if (!loaded)
        {
          std::cerr << "could not find index!\n";
          throw(OctreeException::OCT_MISSING_IDX);
        }

      }

      loadFromFile (thisnodeindex_, super);

      if (loadAll)
      {
        loadChildren (true);
      }
    }
//////////////////////////////////////////////////////////////////////////////// 

    template<typename Container, typename PointT> void
    octree_base_node<Container, PointT>::init_root_node (const double bbmin[3], const double bbmax[3], octree_base<Container, PointT> * const tree, const boost::filesystem::path& rootname)
    {
      parent_ = NULL;
      root_ = this;
      m_tree_ = tree;
      depth_ = 0;

      // Allocate space for 8 child nodes
      memset (children_, 0, 8 * sizeof(octree_base_node<Container, PointT>*));
      numchild_ = 0;

      // Set bounding box and mid point
      memcpy (min_, bbmin, 3 * sizeof(double));
      memcpy (max_, bbmax, 3 * sizeof(double));
      midx_ = (max_[0] + min_[0]) / double (2);
      midy_ = (max_[1] + min_[1]) / double (2);
      midz_ = (max_[2] + min_[2]) / double (2);

      // Get root path
      const boost::filesystem::path dir = rootname.parent_path ();

      // If the root directory doesn't exist create it
      if (!boost::filesystem::exists (dir))
      {
        boost::filesystem::create_directory (dir);
      }
      // If the root directory is a file
      else if (!boost::filesystem::is_directory (dir))
      {
        //boost::filesystem::remove_all(dir);
        //boost::filesystem::create_directory(dir);
        std::cerr << "need empty dir structure! --- dir exists and is a file" << std::endl;
        throw(OctreeException::OCT_BAD_PATH);
      }

      // Create a unique id for node file name
      /** \todo: getRandomUUIDString shouldn't be in class octree_disk_container; also this is pretty slow*/
      std::string uuid;
      octree_disk_container<PointT>::getRandomUUIDString (uuid);
      std::string node_container_name = uuid + std::string ("_") + node_container_basename + node_container_extension;

      // Setup all file paths related to this node
      thisdir_ = boost::filesystem::path (dir);
      thisnodeindex_ = thisdir_ / rootname.filename ();
      thisnodestorage_ = thisdir_ / boost::filesystem::path (node_container_name);
      boost::filesystem::create_directory (thisdir_);

      // Create data container, ie octree_disk_container, octree_ram_container
      payload_ = new Container (thisnodestorage_);
    }
////////////////////////////////////////////////////////////////////////////////

    template<typename Container, typename PointT>
    octree_base_node<Container, PointT>::octree_base_node (const double bbmin[3], const double bbmax[3],
                                                           const double node_dim_meters,
                                                           octree_base<Container, PointT> * const tree,
                                                           const boost::filesystem::path& rootname)
    {
      init_root_node(bbmin, bbmax, tree, rootname);

      // Calculate the max depth but don't create nodes
      tree->maxDepth_ = calcDepthForDim (bbmin, bbmax, node_dim_meters);
      saveIdx (false);
      //createChildrenToDim(node_dim_meters);
    }
////////////////////////////////////////////////////////////////////////////////

    template<typename Container, typename PointT>
    octree_base_node<Container, PointT>::octree_base_node (const int maxdepth, const double bbmin[3], const double bbmax[3], octree_base<Container, PointT> * const tree, const boost::filesystem::path& rootname)
    {
      init_root_node(bbmin, bbmax, tree, rootname);

      // Set max depth but don't create nodes
      tree->maxDepth_ = maxdepth;
      saveIdx (false);
    }
////////////////////////////////////////////////////////////////////////////////

    template<typename Container, typename PointT>
    octree_base_node<Container, PointT>::~octree_base_node ()
    {
      // Recursively delete all children and this nodes data
      recFreeChildren ();
      delete payload_;
    }
////////////////////////////////////////////////////////////////////////////////

    template<typename Container, typename PointT> inline bool
    octree_base_node<Container, PointT>::hasUnloadedChildren () const
    {
      unsigned int numchild_Dirs = 0;
      // Check nodes directory for children directories 0-7
      for (int i = 0; i < 8; i++)
      {
        boost::filesystem::path childdir = thisdir_ / boost::filesystem::path (boost::lexical_cast<std::string> (i));
        if (boost::filesystem::exists (childdir))
        {
          numchild_Dirs++;
        }
      }

      // If found directories is less than the current nodes loaded children
      if (numchild_Dirs > numchild_)
        return (true);

      return (false);
    }
////////////////////////////////////////////////////////////////////////////////

    template<typename Container, typename PointT> void
    octree_base_node<Container, PointT>::loadChildren (bool recursive)
    {
      /** todo: hasChildrenLoaded checks \ref numchild_ against how many child
          directories live on disk.  This just bails if anything is loaded? */
      if (numchild_ != 0)
      {
        std::cerr << "Calling loadChildren on a node that already has loaded children! - skipping";
        return;
      }

      // Create a new node for each child directory that exists
      for (int i = 0; i < 8; i++)
      {
        boost::filesystem::path childdir = thisdir_ / boost::filesystem::path (boost::lexical_cast<std::string> (i));
        if (boost::filesystem::exists (childdir))
        {
          this->children_[i] = new octree_base_node<Container, PointT> (childdir, this, recursive);
          numchild_++;
        }
      }
    }
////////////////////////////////////////////////////////////////////////////////

    template<typename Container, typename PointT> void
    octree_base_node<Container, PointT>::recFreeChildren ()
    {
      if (numchild_ == 0)
      {
        return;
      }

      for (size_t i = 0; i < 8; i++)
      {
        if (children_[i])
        {
          octree_base_node<Container, PointT>* current = children_[i];
          delete current;
        }
      }
      memset (children_, 0, 8 * sizeof(octree_base_node<Container, PointT>*));
      numchild_ = 0;
    }
////////////////////////////////////////////////////////////////////////////////

    template<typename Container, typename PointT> boost::uint64_t
    octree_base_node<Container, PointT>::addDataToLeaf (const std::vector<PointT>& p, const bool skipBBCheck)
    {
      //quit if there are no points to add
      if (p.empty ())
      {
        return (0);
      }

      //if this depth is the max depth of the tree, then add the point
      if (this->depth_ == root_->m_tree_->maxDepth_)
        return (addDataAtMaxDepth(p, skipBBCheck));

      if (numchild_ < 8)
        if (hasUnloadedChildren ())
          loadChildren (false);

      std::vector < std::vector<const PointT*> > c;
      c.resize (8);
      for (size_t i = 0; i < 8; i++)
      {
        c[i].reserve (p.size () / 8);
      }

      const size_t len = p.size ();
      for (size_t i = 0; i < len; i++)
      {
        const PointT& pt = p[i];

        if (!skipBBCheck)
        {
          if (!this->pointWithinBB (pt))
          {
            //	std::cerr << "failed to place point!!!" << std::endl;
            continue;
          }
        }

        if ((pt.z >= midz_))
        {
          if ((pt.y >= midy_))
          {
            if ((pt.x >= midx_))
            {
              c[7].push_back (&pt);
              continue;
            }
            else
            {
              c[6].push_back (&pt);
              continue;
            }
          }
          else
          {
            if ((pt.x >= midx_))
            {
              c[5].push_back (&pt);
              continue;
            }
            else
            {
              c[4].push_back (&pt);
              continue;
            }
          }
        }
        else
        {
          if ((pt.y >= midy_))
          {
            if ((pt.x >= midx_))
            {
              c[3].push_back (&pt);
              continue;
            }
            else
            {
              c[2].push_back (&pt);
              continue;
            }
          }
          else
          {
            if ((pt.x >= midx_))
            {
              c[1].push_back (&pt);
              continue;
            }
            else
            {
              c[0].push_back (&pt);
              continue;
            }
          }
        }
      }

      boost::uint64_t points_added = 0;
      for (int i = 0; i < 8; i++)
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


//return number of points added
    template<typename Container, typename PointT> boost::uint64_t
    octree_base_node<Container, PointT>::addDataToLeaf (const std::vector<const PointT*>& p, const bool skipBBCheck)
    {
      if (p.empty ())
      {
        return (0);
      }

      if (this->depth_ == root_->m_tree_->maxDepth_)
      {
        if (skipBBCheck)//trust me, just add the points
        {
          root_->m_tree_->count_point (this->depth_, p.size ());
          payload_->insertRange (p.data (), p.size ());
          return (p.size ());
        }
        else//check which points belong to this node, throw away the rest
        {
          std::vector<const PointT*> buff;
          BOOST_FOREACH(const PointT* pt, p)
          {
            if(pointWithinBB(*pt))
            {
              buff.push_back(pt);
            }
          }

          if (!buff.empty ())
          {
            root_->m_tree_->count_point (this->depth_, buff.size ());
            payload_->insertRange (buff.data (), buff.size ());
          }
          return (buff.size ());
        }
      }
      else
      {
        if (numchild_ < 8)
        {
          if (hasUnloadedChildren ())
          {
            loadChildren (false);
          }
        }

        std::vector < std::vector<const PointT*> > c;
        c.resize (8);
        for (int i = 0; i < 8; i++)
        {
          c[i].reserve (p.size () / 8);
        }

        const size_t len = p.size ();
        for (size_t i = 0; i < len; i++)
        {
          //const PointT& pt = p[i];
          if (!skipBBCheck)
          {
            if (!this->pointWithinBB (*p[i]))
            {
              //	std::cerr << "failed to place point!!!" << std::endl;
              continue;
            }
          }

          if ((p[i]->z >= midz_))
          {
            if ((p[i]->y >= midy_))
            {
              if ((p[i]->x >= midx_))
              {
                c[7].push_back (p[i]);
                continue;
              }
              else
              {
                c[6].push_back (p[i]);
                continue;
              }
            }
            else
            {
              if ((p[i]->x >= midx_))
              {
                c[5].push_back (p[i]);
                continue;
              }
              else
              {
                c[4].push_back (p[i]);
                continue;
              }
            }
          }
          else
          {
            if ((p[i]->y >= midy_))
            {
              if ((p[i]->x >= midx_))
              {
                c[3].push_back (p[i]);
                continue;
              }
              else
              {
                c[2].push_back (p[i]);
                continue;
              }
            }
            else
            {
              if ((p[i]->x >= midx_))
              {
                c[1].push_back (p[i]);
                continue;
              }
              else
              {
                c[0].push_back (p[i]);
                continue;
              }
            }
          }
        }
        boost::uint64_t points_added = 0;
        for (int i = 0; i < 8; i++)
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
      // std::cerr << "failed to place point!!!" << std::endl;
      return (0);
    }
////////////////////////////////////////////////////////////////////////////////

/** todo: This seems like a lot of work to get a random uniform sample? */
/** todo: Need to refactor this further as to not pass in a BBCheck */
    template<typename Container, typename PointT> void
    octree_base_node<Container, PointT>::randomSample(const std::vector<PointT>& p, std::vector<PointT>& insertBuff, const bool skipBBCheck)
    {
//    std::cout << "randomSample" << std::endl;
      std::vector<PointT> sampleBuff;
      if (!skipBBCheck)
      {
        BOOST_FOREACH (const PointT& pt, p)
        if(pointWithinBB(pt))
          sampleBuff.push_back(pt);
      }
      else
      {
        sampleBuff = p;
      }

      // Derive percentage from specified sample_precent and tree depth
      const double percent = pow(sample_precent, double((root_->m_tree_->maxDepth_ - depth_)));
      const boost::uint64_t samplesize = (boost::uint64_t)(percent * double(sampleBuff.size()));
      const boost::uint64_t inputsize = sampleBuff.size();

      if(samplesize > 0)
      {
        // Resize buffer to sample size
        insertBuff.resize(samplesize);

        // Create random number generator
        boost::mutex::scoped_lock lock(rng_mutex_);
        boost::uniform_int<boost::uint64_t> buffdist(0, inputsize-1);
        boost::variate_generator<boost::mt19937&, boost::uniform_int<boost::uint64_t> > buffdie(rand_gen_, buffdist);

        // Randomly pick sampled points
        for(boost::uint64_t i = 0; i < samplesize; ++i)
        {
          boost::uint64_t buffstart = buffdie();
          insertBuff[i] = ( sampleBuff[buffstart] );
        }
      }
      // Have to do it the slow way
      else
      {
        boost::mutex::scoped_lock lock(rng_mutex_);
        boost::bernoulli_distribution<double> buffdist(percent);
        boost::variate_generator<boost::mt19937&, boost::bernoulli_distribution<double> > buffcoin(rand_gen_, buffdist);

        for(boost::uint64_t i = 0; i < inputsize; ++i)
          if(buffcoin())
            insertBuff.push_back( p[i] );
      }
    }
////////////////////////////////////////////////////////////////////////////////

    template<typename Container, typename PointT> boost::uint64_t
    octree_base_node<Container, PointT>::addDataAtMaxDepth (const std::vector<PointT>& p, const bool skipBBCheck)
    {
      // Trust me, just add the points
      if(skipBBCheck)
      {
        // Increment point count for node
        root_->m_tree_->count_point (this->depth_, p.size ());

        // Insert point data
        payload_->insertRange (p.data (), p.size ());
        return (p.size ());
      }

      // Add points found within the current nodes bounding box
      else
      {
        std::vector<PointT> buff;
        const size_t len = p.size ();

        for (size_t i = 0; i < len; i++)
          if (pointWithinBB (p[i]))
            buff.push_back (p[i]);

        if (!buff.empty ())
        {
          root_->m_tree_->count_point (this->depth_, buff.size ());
          payload_->insertRange (buff.data (), buff.size ());
        }
        return (buff.size ());
      }
    }
////////////////////////////////////////////////////////////////////////////////


    template<typename Container, typename PointT> void
    octree_base_node<Container, PointT>::subdividePoints (const std::vector<PointT>& p,
                                                          std::vector< std::vector<PointT> >& c,
                                                          const bool skipBBCheck)
    {
      // Reserve space for children nodes
      c.resize(8);
      for(int i = 0; i < 8; i++)
        c[i].reserve(p.size() / 8);

      const size_t len = p.size();
      for(size_t i = 0; i < len; i++)
      {
        const PointT& pt = p[i];

        if(!skipBBCheck)
          if(!this->pointWithinBB(pt))
            continue;

        subdividePoint(pt, c);
      }
    }
////////////////////////////////////////////////////////////////////////////////


    template<typename Container, typename PointT> void
    octree_base_node<Container, PointT>::subdividePoint (const PointT& pt, std::vector< std::vector<PointT> >& c)
    {
      if((pt.z >= midz_))
      {
        if((pt.y >= midy_))
        {
          if((pt.x >= midx_))
          {
            c[7].push_back(pt);
            return;
          }
          else
          {
            c[6].push_back(pt);
            return;
          }
        }
        else
        {
          if((pt.x >= midx_))
          {
            c[5].push_back(pt);
            return;
          }
          else
          {
            c[4].push_back(pt);
            return;
          }
        }
      }
      else
      {
        if((pt.y >= midy_))
        {
          if((pt.x >= midx_))
          {
            c[3].push_back(pt);
            return;
          }
          else
          {
            c[2].push_back(pt);
            return;
          }
        }
        else
        {
          if((pt.x >= midx_))
          {
            c[1].push_back(pt);
            return;
          }
          else
          {
            c[0].push_back(pt);
            return;
          }
        }
      }
    }
////////////////////////////////////////////////////////////////////////////////


    template<typename Container, typename PointT> boost::uint64_t
    octree_base_node<Container, PointT>::addDataToLeaf_and_genLOD (const std::vector<PointT>& p, const bool skipBBCheck)
    {
      // If there's no points return
      if (p.empty ())
        return (0);

      /// \todo: Why is skipBBCheck set to false when adding points at max depth
      //  when adding data and generating sampled LOD 
      // If the max depth has been reached
      if (this->depth_ == root_->m_tree_->maxDepth_)
        return (addDataAtMaxDepth(p, false));

      // Create child nodes of the current node but not grand children+
      if (numchild_ < 8)
        if (hasUnloadedChildren ())
          loadChildren (false);

      // Randomly sample data
      std::vector<PointT> insertBuff;
      randomSample(p, insertBuff, skipBBCheck);

      if(!insertBuff.empty())
      {
        // Increment point count for node
        root_->m_tree_->count_point (this->depth_, insertBuff.size());
        // Insert sampled point data
        payload_->insertRange ( &(insertBuff.front ()), insertBuff.size());
      }

      //subdivide vec to pass data down lower
      std::vector< std::vector<PointT> > c;
      subdividePoints(p, c, skipBBCheck);

      /// \todo: Perhaps do a quick loop through the lists here and dealloc the reserved mem for empty lists
      boost::uint64_t points_added = 0;
      for(int i = 0; i < 8; i++)
      {
        // If child doesn't have points
        if(c[i].empty())
          continue;

        // If child doesn't exist
        if(!children_[i])
          createChild(i);

        /// \todo: Why are there no bounding box checks on the way down?
        // Recursively build children
        points_added += children_[i]->addDataToLeaf_and_genLOD(c[i], true);
        c[i].clear();
      }

      return (points_added);

      // todo: Make sure I didn't break anything by removing the if/else above
      // std::cerr << "failed to place point!!!" << std::endl;
      //return 0;
    }
////////////////////////////////////////////////////////////////////////////////


// todo: Do we need to support std::vector<PointT*>
//template<typename Container, typename PointT>
//  boost::uint64_t
//  octree_base_node<Container, PointT>::addDataToLeaf_and_genLOD (const std::vector<PointT*>& p,
//                                                                     const bool skipBBCheck)
//  {
//    if (p.empty ())
//    {
//      return 0;
//    }
//
//    if (this->depth == root_->m_tree_->maxDepth_)
//    {
//      //if(skipBBCheck)//trust me, just add the points
//      if (false)//trust me, just add the points
//      {
//        root_->m_tree_->count_point (this->depth, p.size ());
//        payload->insertRange (p.data (), p.size ());
//        return p.size ();
//      }
//      else//check which points belong to this node, throw away the rest
//      {
//        std::vector<PointT> buff;
//
//        const size_t len = p.size ();
//        for (size_t i = 0; i < len; i++)
//        {
//          //const PointT& pt = p[i];
//          if (pointWithinBB (p[i]))
//          {
//            buff.push_back (p[i]);
//          }
//          else
//          {
//            //	std::cerr << "failed to place point";
//          }
//        }
//
//        root_->m_tree_->count_point (this->depth, buff.size ());
//        payload->insertRange (buff.data (), buff.size ());
//        return buff.size ();
//      }
//    }
//    else
//    {
//      if (numchild_ < 8)
//      {
//        if (hasUnloadedChildren ())
//        {
//          loadChildren (false);
//        }
//      }
//
//      //add code to subsample here
//      std::vector<PointT> insertBuff;
//      {
//        std::vector<PointT> sampleBuff;
//        if (!skipBBCheck)
//        {
//BOOST_FOREACH        (const PointT& pt, p)
//        {
//          if(pointWithinBB(pt))
//          {
//            sampleBuff.push_back(pt);
//          }
//        }
//      }
//      else
//      {
//        sampleBuff = p;
//      }
//
//      const double percent = pow(sample_precent, double((root->m_tree_->maxDepth_ - depth)));
//      const boost::uint64_t samplesize = (boost::uint64_t)(percent * double(sampleBuff.size()));
//      const boost::uint64_t inputsize = sampleBuff.size();
//      if(samplesize > 0)
//      {
//        insertBuff.resize(samplesize);
//
//        boost::mutex::scoped_lock lock(rng_mutex_);
//        boost::uniform_int<boost::uint64_t> buffdist(0, inputsize-1);
//        boost::variate_generator<boost::mt19937&, boost::uniform_int<boost::uint64_t> > buffdie(rand_gen_, buffdist);
//
//        for(boost::uint64_t i = 0; i < samplesize; ++i)
//        {
//          boost::uint64_t buffstart = buffdie();
//          insertBuff[i] = ( sampleBuff[buffstart] );
//        }
//      }
//      else//have to do it the slow way
//
//      {
//        boost::mutex::scoped_lock lock(rng_mutex_);
//        boost::bernoulli_distribution<double> buffdist(percent);
//        boost::variate_generator<boost::mt19937&, boost::bernoulli_distribution<double> > buffcoin(rand_gen_, buffdist);
//
//        for(boost::uint64_t i = 0; i < inputsize; ++i)
//        {
//          if(buffcoin())
//          {
//            insertBuff.push_back( p[i] );
//          }
//        }
//      }
//    }
//    if(!insertBuff.empty())
//    {
//      root_->m_tree_->count_point(this->depth, insertBuff.size());
//      payload->insertRange(&(insertBuff.front()), insertBuff.size());
//    }
//    //end subsample
//
//    //subdivide vec to pass data down lower
//    std::vector<PointT*> c;
//    c.resize(8);
//    for(int i = 0; i < 8; i++)
//    {
//      c[i].reserve(p.size() / 8);
//    }
//
//    const size_t len = p.size();
//    for(size_t i = 0; i < len; i++)
//    {
//      //const PointT& pt = p[i];
//
//      if(!skipBBCheck)
//      {
//        if(!this->pointWithinBB(p[i]))
//        {
//          //	std::cerr << "\nfailed to place point!!!\n" << std::endl;
//          continue;
//        }
//      }
//
//      if((p[i].z >= midz_))
//      {
//        if((p[i].y >= midy_))
//        {
//          if((p[i].x >= midx_))
//          {
//            c[7].push_back(p[i]);
//            continue;
//          }
//          else
//          {
//            c[6].push_back(p[i]);
//            continue;
//          }
//        }
//        else
//        {
//          if((p[i].x >= midx_))
//          {
//            c[5].push_back(p[i]);
//            continue;
//          }
//          else
//          {
//            c[4].push_back(p[i]);
//            continue;
//          }
//        }
//      }
//      else
//      {
//        if((p[i].y >= midy_))
//        {
//          if((p[i].x >= midx_))
//          {
//            c[3].push_back(p[i]);
//            continue;
//          }
//          else
//          {
//            c[2].push_back(p[i]);
//            continue;
//          }
//        }
//        else
//        {
//          if((p[i].x >= midx_))
//          {
//            c[1].push_back(p[i]);
//            continue;
//          }
//          else
//          {
//            c[0].push_back(p[i]);
//            continue;
//          }
//        }
//      }
//    }
//
//    //perhaps do a quick loop through the lists here and dealloc the reserved mem for empty lists
//    boost::uint64_t points_added = 0;
//    for(int i = 0; i < 8; i++)
//    {
//      if(c[i].empty()) continue;
//      if(!children[i]) createChild(i);
//      points_added += children[i]->addDataToLeaf_and_genLOD(c[i], true);
//      c[i].clear();
//    }
//    return points_added;
//  }
//  // std::cerr << "failed to place point!!!" << std::endl;
//  return 0;
//}

/*
  Child order:

  bottom stack (low z):
  1 3
  0 2

  top stack (high z):
  5 7
  4 6
*/

    template<typename Container, typename PointT> void
    octree_base_node<Container, PointT>::createChild (const int idx)
    {
      //if already has 8 children, return
      if (children_[idx] || (numchild_ == 8))
        return;

      const double zstart = min_[2];
      const double ystart = min_[1];
      const double xstart = min_[0];

      const double zstep = (max_[2] - min_[2]) / double (2);
      const double ystep = (max_[1] - min_[1]) / double (2);
      const double xstep = (max_[0] - min_[0]) / double (2);

      double childbb_min[3];
      double childbb_max[3];
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

      childbb_min[2] = zstart + double (z) * zstep;
      childbb_max[2] = zstart + double (z + 1) * zstep;

      childbb_min[1] = ystart + double (y) * ystep;
      childbb_max[1] = ystart + double (y + 1) * ystep;

      childbb_min[0] = xstart + double (x) * xstep;
      childbb_max[0] = xstart + double (x + 1) * xstep;

      boost::filesystem::path childdir = thisdir_ / boost::filesystem::path (boost::lexical_cast<std::string> (idx));
      children_[idx] = new octree_base_node<Container, PointT> (childbb_min, childbb_max, childdir.string ().c_str (), this);

      numchild_++;
    }
////////////////////////////////////////////////////////////////////////////////

    template<typename Container, typename PointT> void
    octree_base_node<Container, PointT>::createChildren ()
    {
      const double zstart = min_[2];
      const double ystart = min_[1];
      const double xstart = min_[0];

      const double zstep = (max_[2] - min_[2]) / double (2);
      const double ystep = (max_[1] - min_[1]) / double (2);
      const double xstep = (max_[0] - min_[0]) / double (2);

      int i = 0;

      double childbb_min[3];
      double childbb_max[3];
      for (int z = 0; z < 2; z++)
      {
        childbb_min[2] = zstart + double (z) * zstep;
        childbb_max[2] = zstart + double (z + 1) * zstep;

        for (int y = 0; y < 2; y++)
        {
          childbb_min[1] = ystart + double (y) * ystep;
          childbb_max[1] = ystart + double (y + 1) * ystep;

          for (int x = 0; x < 2; x++)
          {
            childbb_min[0] = xstart + double (x) * xstep;
            childbb_max[0] = xstart + double (x + 1) * xstep;

            boost::filesystem::path childdir = thisdir_ / boost::filesystem::path (boost::lexical_cast<std::string> (i));
            children_[i] = new octree_base_node<Container, PointT> (childbb_min, childbb_max, childdir.string ().c_str (), this);
            i++;
          }
        }
      }
      numchild_ = 8;
    }
////////////////////////////////////////////////////////////////////////////////

//template<typename Container, typename PointT>
//void octree_base_node<Container, PointT>::createChildrenToDim(const double dim)
//{
//	double side[3];
//
//	for (int i = 0; i < 3; i++) {
//		side[i] = maxbb[i] - minbb[i];
//	};
//
//
//	if( (side[0] < dim) || (side[1] < dim) || (side[2] < dim) )
//	{
//		this->root->maxDepth_ = this->depth;
//		return;
//	}
//
//	if(numchild_ == 0)
//	{
//		createChildren();
//	}
//
//	for(size_t i = 0; i < numchild_; i++)
//	{
//		children[i]->createChildrenToDim(dim);
//	}
//}

    template<typename Container, typename PointT> int
    octree_base_node<Container, PointT>::calcDepthForDim (const double minbb[3], const double maxbb[3], const double dim)
    {
      double volume = 1;
      double diagonal = 0;

      for (int i = 0; i < 3; i++)
      {
        double side = maxbb[i] - minbb[i];
        diagonal += side * side;
        volume *= side;
      };

      diagonal = sqrt (diagonal);
      double dim_volume = dim * dim * dim;

      if ((diagonal <= dim) || (volume <= dim_volume))
      {
        return (0);
      }
      else
      {
        double zstart = minbb[2];
        double ystart = minbb[1];
        double xstart = minbb[0];

        double zstep = (maxbb[2] - minbb[2]) / double (2);
        double ystep = (maxbb[1] - minbb[1]) / double (2);
        double xstep = (maxbb[0] - minbb[0]) / double (2);

        double childbb_min[3];
        double childbb_max[3];

        childbb_min[0] = xstart;
        childbb_min[1] = ystart;
        childbb_min[2] = zstart;

        childbb_max[0] = xstart + double (1) * xstep;
        childbb_max[1] = ystart + double (1) * ystep;
        childbb_max[2] = zstart + double (1) * zstep;

        return (1 + calcDepthForDim (childbb_min, childbb_max, dim));
      }

    }
////////////////////////////////////////////////////////////////////////////////

    template<typename Container, typename PointT> inline bool
    octree_base_node<Container, PointT>::pointWithinBB (const PointT& p) const
    {
      if (((min_[0] <= p.x) && (p.x <= max_[0])) &&
          ((min_[1] <= p.y) && (p.y <= max_[1])) &&
          ((min_[2] <= p.z) && (p.z <= max_[2])))
      {
        return (true);
    
      }
      return (false);
    }
////////////////////////////////////////////////////////////////////////////////

    template<typename Container, typename PointT> void
    octree_base_node<Container, PointT>::queryBBIntersects (const double minbb[3], const double maxbb[3], const boost::uint32_t query_depth, std::list<std::string>& file_names)
    {
      //
      //  Do a quick check to see if the caller passed in a lat/lon/alt bounding box.
      //  If so, convert to UTM
      //
      double my_min[3];
      double my_max[3];

      memcpy (my_min, minbb, 3 * sizeof(double));
      memcpy (my_max, maxbb, 3 * sizeof(double));

      //	printf("DEBUG! - automatic LLA detection removed\n");
      //	if (false) {
      //		if ((minbb[1] >= -180) && (maxbb[1] <= 180) && (minbb[0] >= -90) && (maxbb[0] <= 90)) {
      //			char   zone[256];
      //			double min_northing;
      //			double min_easting;
      //			double max_northing;
      //			double max_easting;
      //			int    utmZone;
      //			CgeoMath::ll2utm(UTM_WGS_84, minbb[0], minbb[1], &utmZone, zone, &min_northing, &min_easting);
      //			CgeoMath::ll2utm(UTM_WGS_84, maxbb[0], maxbb[1], &utmZone, zone, &max_northing, &max_easting);
      //
      //			my_min[0] = min_easting;
      //			my_min[1] = min_northing;
      //			my_max[0] = max_easting;
      //			my_max[1] = max_northing;
      //		};
      //	}


      if (intersectsWithBB (my_min, my_max))
      {
        if (this->depth_ < query_depth)
        {
          if (numchild_ > 0)
          {
            for (size_t i = 0; i < 8; i++)
            {
              if (children_[i])
                children_[i]->queryBBIntersects (my_min, my_max, query_depth, file_names);
            }
          }
          else if (hasUnloadedChildren ())
          {
            loadChildren (false);

            for (size_t i = 0; i < 8; i++)
            {
              if (children_[i])
                children_[i]->queryBBIntersects (my_min, my_max, query_depth, file_names);
            }
          }
          return;
        }

        if (!payload_->empty ())
        {
          file_names.push_back (payload_->path ());
        }
      }
    }
////////////////////////////////////////////////////////////////////////////////

//read from lowest level
    template<typename Container, typename PointT> void
    octree_base_node<Container, PointT>::queryBBIncludes (const double minbb[3], const double maxbb[3], size_t query_depth, std::list<PointT>& v)
    {

      //if the queried bounding box has any intersection with this node's bounding box
      if (intersectsWithBB (minbb, maxbb))
      {
        //if we aren't at the max desired depth
        if (this->depth_ < query_depth)
        {
          //if this node doesn't have any children, we are at the max depth for this query
          if ((numchild_ == 0) && (hasUnloadedChildren ()))
          {
            loadChildren (false);
          }

          //if this node has children
          if (numchild_ > 0)
          {
            //recursively store any points that falls into the queried bounding box into v and return
            for (size_t i = 0; i < 8; i++)
            {
              if (children_[i])
                children_[i]->queryBBIncludes (minbb, maxbb, query_depth, v);
            }
            return;
          }
        }
        //otherwise if we are at the max depth
        else
        {
          //if this node's bounding box falls completely within the queried bounding box
          if (withinBB (minbb, maxbb))
          {
            //get all the points from the payload and return
            std::vector<PointT> payload_cache;
            payload_->readRange (0, payload_->size (), payload_cache);
            v.insert (v.end (), payload_cache.begin (), payload_cache.end ());
            return;
          }
          //otherwise queried bounding box only partially intersects this
          //node's bounding box, so we have to check all the points in
          //this box for intersection with queried bounding box
          else
          {
            //read _all_ the points in from the disk container
            std::vector<PointT> payload_cache;
            payload_->readRange (0, payload_->size (), payload_cache);
        
            boost::uint64_t len = payload_->size ();
            //iterate through each of them
            for (boost::uint64_t i = 0; i < len; i++)
            {
              const PointT& p = payload_cache[i];
              //if it falls within this bounding box
              if (pointWithinBB (minbb, maxbb, p))
              {
                //store it in the list
                v.push_back (p);
              }
            }
          }
        }
      }
  
    }
////////////////////////////////////////////////////////////////////////////////

    template<typename Container, typename PointT> void
    octree_base_node<Container, PointT>::queryBBIncludes_subsample (const double minbb[3], const double maxbb[3], int query_depth, const double percent, std::list<PointT>& v)
    {
      //check if the queried bounding box has any intersection with this node's bounding box
      if (intersectsWithBB (minbb, maxbb))
      {
        //if we are not at the max depth for queried nodes
        if (this->depth_ < query_depth)
        {
          //check if we don't have children
          if ((numchild_ == 0) && (hasUnloadedChildren ()))
          {
            loadChildren (false);
          }
          //if we do have children
          if (numchild_ > 0)
          {
            //recursively add their valid points within the queried bounding box to the list v
            for (size_t i = 0; i < 8; i++)
            {
              if (children_[i])
                children_[i]->queryBBIncludes_subsample (minbb, maxbb, query_depth, percent, v);
            }
            return;
          }
        }
        //otherwise we are at the max depth, so we add all our points or some of our points
        else
        {
          //if this node's bounding box falls completely within the queried bounding box
          if (withinBB (minbb, maxbb))
          {
            //add a random sample of all the points
            std::vector<PointT> payload_cache;
            payload_->readRangeSubSample (0, payload_->size (), percent, payload_cache);
            v.insert (v.end (), payload_cache.begin (), payload_cache.end ());
            return;
          }
          //otherwise the queried bounding box only partially intersects with this node's bounding box
          else
          {
            //brute force selection of all valid points
            std::vector<PointT> payload_cache_within_region;
            {
              std::vector<PointT> payload_cache;
              payload_->readRange (0, payload_->size (), payload_cache);
              for (size_t i = 0; i < payload_->size (); i++)
              {
                const PointT& p = payload_cache[i];
                if (pointWithinBB (minbb, maxbb, p))
                {
                  payload_cache_within_region.push_back (p);
                }
              }
            }//force the payload cache to deconstruct here

            //** \todo: this is likely to be very slow.*/

            //use STL random_shuffle and push back a random selection of the points onto our list
            std::random_shuffle (payload_cache_within_region.begin (), payload_cache_within_region.end ());
            size_t numpick = percent * payload_cache_within_region.size ();

            for (size_t i = 0; i < numpick; i++)
            {
              v.push_back (payload_cache_within_region[i]);
            }
          }
        }
      }
    }
////////////////////////////////////////////////////////////////////////////////

//dir is current level. we put this nodes files into it
    template<typename Container, typename PointT>
    octree_base_node<Container, PointT>::octree_base_node (const double bbmin[3], const double bbmax[3], const char* dir, octree_base_node<Container,PointT>* super)
    {
      if (super == NULL)
      {
        std::cerr << "super is null - don't make a root node this way!" << std::endl;
        throw(OctreeException::OCT_BAD_PARENT);
      }

      this->parent_ = super;
      root_ = super->root_;
      depth_ = super->depth_ + 1;

      memset (children_, 0, 8 * sizeof(octree_base_node<Container, PointT>*));
      numchild_ = 0;

      memcpy (min_, bbmin, 3 * sizeof(double));
      memcpy (max_, bbmax, 3 * sizeof(double));
      midx_ = (max_[0] + min_[0]) / double (2);
      midy_ = (max_[1] + min_[1]) / double (2);
      midz_ = (max_[2] + min_[2]) / double (2);

      std::string uuid_idx;
      std::string uuid_cont;
      octree_disk_container<PointT>::getRandomUUIDString (uuid_idx);
      octree_disk_container<PointT>::getRandomUUIDString (uuid_cont);

      std::string node_index_name = uuid_idx + std::string ("_") + node_index_basename + node_index_extension;
      std::string node_container_name = uuid_cont + std::string ("_") + node_container_basename
      + node_container_extension;

      thisdir_ = boost::filesystem::path (dir);
      thisnodeindex_ = thisdir_ / boost::filesystem::path (node_index_name);
      thisnodestorage_ = thisdir_ / boost::filesystem::path (node_container_name);

      boost::filesystem::create_directory (thisdir_);

      payload_ = new Container (thisnodestorage_);
      saveIdx (false);
    }
////////////////////////////////////////////////////////////////////////////////

    template<typename Container, typename PointT>
    void
    octree_base_node<Container, PointT>::copyAllCurrentAndChildPointsRec (std::list<PointT>& v)
    {
      if ((numchild_ == 0) && (hasUnloadedChildren ()))
      {
        loadChildren (false);
      }

      for (size_t i = 0; i < numchild_; i++)
      {
        children_[i]->copyAllCurrentAndChildPointsRec (v);
      }

      std::vector<PointT> payload_cache;
      payload_->readRange (0, payload_->size (), payload_cache);

      {
        //boost::mutex::scoped_lock lock(queryBBIncludes_vector_mutex);
        v.insert (v.end (), payload_cache.begin (), payload_cache.end ());
      }
    }
////////////////////////////////////////////////////////////////////////////////

    template<typename Container, typename PointT> void
    octree_base_node<Container, PointT>::copyAllCurrentAndChildPointsRec_sub (std::list<PointT>& v, const double percent)
    {
      if ((numchild_ == 0) && (hasUnloadedChildren ()))
      {
        loadChildren (false);
      }

      for (size_t i = 0; i < 8; i++)
      {
        if (children_[i])
          children_[i]->copyAllCurrentAndChildPointsRec_sub (v, percent);
      }

      std::vector<PointT> payload_cache;
      payload_->readRangeSubSample (0, payload_->size (), percent, payload_cache);

      for (size_t i = 0; i < payload_cache.size (); i++)
      {
        v.push_back (payload_cache[i]);
      }
    }
////////////////////////////////////////////////////////////////////////////////

    template<typename Container, typename PointT> inline bool
    octree_base_node<Container, PointT>::intersectsWithBB (const double minbb[3], const double maxbb[3]) const
    {
      if (((min_[0] <= minbb[0]) && (minbb[0] <= max_[0])) || ((minbb[0] <= min_[0]) && (min_[0] <= maxbb[0])))
      {
        if (((min_[1] <= minbb[1]) && (minbb[1] <= max_[1])) || ((minbb[1] <= min_[1]) && (min_[1] <= maxbb[1])))
        {
          if (((min_[2] <= minbb[2]) && (minbb[2] <= max_[2])) || ((minbb[2] <= min_[2]) && (min_[2] <= maxbb[2])))
          {
            return (true);
          }
        }
      }

      return (false);
    }
////////////////////////////////////////////////////////////////////////////////

    template<typename Container, typename PointT> inline bool
    octree_base_node<Container, PointT>::withinBB (const double minbb[3], const double maxbb[3]) const
    {

      if ((minbb[0] <= min_[0]) && (max_[0] <= maxbb[0]))
      {
        if ((minbb[1] <= min_[1]) && (max_[1] <= maxbb[1]))
        {
          if ((minbb[2] <= min_[2]) && (max_[2] <= maxbb[2]))
          {
            return (true);
          }
        }
      }

      return (false);
    }
////////////////////////////////////////////////////////////////////////////////

    template<typename Container, typename PointT> inline bool
    octree_base_node<Container, PointT>::pointWithinBB (const double minbb[3], const double maxbb[3],
                                                        const PointT& p)
    {
      if ((minbb[0] <= p.x) && (p.x <= maxbb[0]))
      {
        if ((minbb[1] <= p.y) && (p.y <= maxbb[1]))
        {
          if ((minbb[2] <= p.z) && (p.z <= maxbb[2]))
          {
            return (true);
          }
        }
      }
      return (false);
    }
////////////////////////////////////////////////////////////////////////////////

    template<typename Container, typename PointT> void
    octree_base_node<Container, PointT>::writeVPythonVisual (std::ofstream& file)
    {
      double l = max_[0] - min_[0];
      double h = max_[1] - min_[1];
      double w = max_[2] - min_[2];
      file << "box( pos=(" << min_[0] << ", " << min_[1] << ", " << min_[2] << "), length=" << l << ", height=" << h
           << ", width=" << w << " )\n";

      for (size_t i = 0; i < numchild_; i++)
      {
        children_[i]->writeVPythonVisual (file);
      }
    }

    template<typename Container, typename PointT> void
    octree_base_node<Container, PointT>::flush_DeAlloc_this_only ()
    {
      payload_->flush (true);
    }

    template<typename Container, typename PointT> void
    octree_base_node<Container, PointT>::flushToDisk ()
    {
      payload_->flush (true);
      for (size_t i = 0; i < 8; i++)
      {
        if (children_[i])
          children_[i]->flushToDisk ();
      }
    }
////////////////////////////////////////////////////////////////////////////////

    template<typename Container, typename PointT> void
    octree_base_node<Container, PointT>::flushToDiskLazy ()
    {
      if (numchild_ > 0)//only flush if not leaf
      {
        payload_->flush (true);
        for (size_t i = 0; i < numchild_; i++)
        {
          if (children_[i])
            children_[i]->flushToDiskLazy ();
        }
      }
    }
////////////////////////////////////////////////////////////////////////////////

    template<typename Container, typename PointT> void
    octree_base_node<Container, PointT>::saveToFile (const boost::filesystem::path& path)
    {
      boost::shared_ptr<cJSON> idx (cJSON_CreateObject (), cJSON_Delete);

      cJSON* version = cJSON_CreateNumber (2.0);
      cJSON* bbmin = cJSON_CreateDoubleArray (min_, 3);
      cJSON* bbmax = cJSON_CreateDoubleArray (max_, 3);

      cJSON* bin = cJSON_CreateString (thisnodestorage_.filename ().string ().c_str ());

      cJSON_AddItemToObject (idx.get (), "version", version);
      cJSON_AddItemToObject (idx.get (), "bbmin", bbmin);
      cJSON_AddItemToObject (idx.get (), "bbmax", bbmax);
      cJSON_AddItemToObject (idx.get (), "bin", bin);

      char* idx_txt = cJSON_Print (idx.get ());

      std::ofstream f (path.string ().c_str (), std::ios::out | std::ios::trunc);
      f << idx_txt;
      f.close ();

      free (idx_txt);
    }
////////////////////////////////////////////////////////////////////////////////

    template<typename Container, typename PointT> void
    octree_base_node<Container, PointT>::loadFromFile (const boost::filesystem::path& path, octree_base_node<Container, PointT>* super)
    {
      //load CJSON
      std::vector<char> idx_input;
      boost::uintmax_t len = boost::filesystem::file_size (path);
      idx_input.resize (len + 1);

      std::ifstream f (thisnodeindex_.string ().c_str (), std::ios::in);
      f.read (&(idx_input.front ()), len);
      idx_input.back () = '\0';

      //Parse
      boost::shared_ptr<cJSON> idx (cJSON_Parse (&(idx_input.front ())), cJSON_Delete);
      cJSON* version = cJSON_GetObjectItem (idx.get (), "version");
      cJSON* bbmin = cJSON_GetObjectItem (idx.get (), "bbmin");
      cJSON* bbmax = cJSON_GetObjectItem (idx.get (), "bbmax");
      cJSON* bin = cJSON_GetObjectItem (idx.get (), "bin");

      //Validate
      if (!((version) && (bbmin) && (bbmax) && (bin)))
      {
        std::cerr << "index " << path << " failed to parse!" << std::endl;
        throw OctreeException (OctreeException::OCT_PARSE_FAILURE);
      }
      if ((version->type != cJSON_Number) || (bbmin->type != cJSON_Array) || (bbmax->type != cJSON_Array) || (bin->type != cJSON_String))
      {
        std::cerr << "index " << path << " failed to parse!" << std::endl;
        throw OctreeException (OctreeException::OCT_PARSE_FAILURE);
      }
      if (version->valuedouble != 2.0)
      {
        std::cerr << "index " << path << " failed to parse!" << std::endl;
        throw OctreeException (OctreeException::OCT_PARSE_FAILURE);
      }

      //	version->valuedouble;
      for (int i = 0; i < 3; i++)
      {
        min_[i] = cJSON_GetArrayItem (bbmin, i)->valuedouble;
        max_[i] = cJSON_GetArrayItem (bbmax, i)->valuedouble;
      }

      thisnodestorage_ = thisdir_ / bin->valuestring;
      this->payload_ = new Container (thisnodestorage_);

      midx_ = (max_[0] + min_[0]) / double (2);
      midy_ = (max_[1] + min_[1]) / double (2);
      midz_ = (max_[2] + min_[2]) / double (2);

      this->parent_ = super;
      memset (children_, 0, 8 * sizeof(octree_base_node<Container, PointT>*));
      this->numchild_ = 0;
    }
////////////////////////////////////////////////////////////////////////////////

    template<typename Container, typename PointT>
    void
    octree_base_node<Container, PointT>::saveIdx (bool recursive)
    {
      saveToFile (thisnodeindex_);

      if (recursive)
      {
        for (size_t i = 0; i < 8; i++)
        {
          if (children_[i])
            children_[i]->saveIdx (true);
        }
      }
    }
////////////////////////////////////////////////////////////////////////////////

    template<typename Container, typename PointT> void
    octree_base_node<Container, PointT>::convertToXYZ ()
    {
      std::string fname = boost::filesystem::basename (thisnodestorage_) + std::string (".dat.xyz");
      boost::filesystem::path xyzfile = thisdir_ / fname;
      payload_->convertToXYZ (xyzfile);

      if (hasUnloadedChildren ())
      {
        loadChildren (false);
      }

      for (size_t i = 0; i < 8; i++)
      {
        if (children_[i])
          children_[i]->convertToXYZ ();
      }
    }
////////////////////////////////////////////////////////////////////////////////

    template<typename Container, typename PointT> octree_base_node<Container, PointT>*
    makenode_norec (const boost::filesystem::path& path, octree_base_node<Container, PointT>* super)
    {
      octree_base_node<Container, PointT>* thisnode = new octree_base_node<octree_disk_container < PointT > , PointT > ();
//octree_disk_node ();

      if (super == NULL)
      {
        thisnode->thisdir_ = path.parent_path ();

        if (!boost::filesystem::exists (thisnode->thisdir_))
        {
          std::cerr << "could not find dir!" << thisnode->thisdir_ << std::endl;
          throw(OctreeException::OCT_BAD_PATH);
        }

        thisnode->thisnodeindex_ = path;

        thisnode->depth_ = 0;
        thisnode->root_ = thisnode;
      }
      else
      {
        thisnode->thisdir_ = path;
        thisnode->depth_ = super->depth_ + 1;
        thisnode->root_ = super->root_;

        if (thisnode->depth_ > thisnode->root->maxDepth_)
        {
          thisnode->root->maxDepth_ = thisnode->depth_;
        }

        boost::filesystem::directory_iterator diterend;
        bool loaded = false;
        for (boost::filesystem::directory_iterator diter (thisnode->thisdir_); diter != diterend; ++diter)
        {
          const boost::filesystem::path& file = *diter;
          if (!boost::filesystem::is_directory (file))
          {
            if (boost::filesystem::extension (file) == octree_base_node<Container, PointT>::node_index_extension)
            {
              thisnode->thisnodeindex_ = file;
              loaded = true;
              break;
            }
          }
        }

        if (!loaded)
        {
          std::cerr << "could not find index!\n";
          throw(OctreeException::OCT_MISSING_IDX);
        }

      }
      thisnode->maxDepth_ = 0;

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

        thisnode->payload_ = new Container (thisnode->thisnodestorage_);
      }

      thisnode->parent_ = super;
      memset (thisnode->children_, 0, 8 * sizeof(octree_base_node<octree_disk_container < PointT > , PointT >*));//octree_disk_node*));
      thisnode->numchild_ = 0;

      return (thisnode);
    }
////////////////////////////////////////////////////////////////////////////////

//accelerate search
    template<typename Container, typename PointT> void
    queryBBIntersects_noload2 (const boost::filesystem::path& rootnode, const double min[3], const double max[3], const boost::uint32_t query_depth, std::list<std::string>& bin_name)
    {
      //this class already has a private "root" member
      //it also has min[3] and max[3] members
      octree_base_node<Container, PointT>* root = makenode_norec<Container, PointT> (rootnode, NULL);
      if (root == NULL)
      {
        std::cout << "test";
      }
      if (root->intersectsWithBB (min, max))
      {
        if (query_depth == root->maxDepth_)
        {
          if (!root->payload_->empty ())
          {
            bin_name.push_back (root->thisnodestorage_.string ());
          }
          return;
        }

        for (int i = 0; i < 8; i++)
        {
          boost::filesystem::path childdir = root->thisdir_
          / boost::filesystem::path (boost::lexical_cast<std::string> (i));
          if (boost::filesystem::exists (childdir))
          {
            root->children_[i] = makenode_norec (childdir, root);
            root->numchild_++;
            queryBBIntersects_noload (root->children_[i], min, max, root->maxDepth_ - query_depth, bin_name);
          }
        }
      }
      delete root;
    }
////////////////////////////////////////////////////////////////////////////////

    template<typename Container, typename PointT> void
    queryBBIntersects_noload (octree_base_node<Container, PointT>* current, const double min[3], const double max[3], const boost::uint32_t query_depth, std::list<std::string>& bin_name)
    {
      if (current->intersectsWithBB (min, max))
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
            boost::filesystem::path childdir = current->thisdir_ / boost::filesystem::path (boost::lexical_cast<std::string> (i));
            if (boost::filesystem::exists (childdir))
            {
              current->children_[i] = makenode_norec<Container, PointT> (childdir, current);
              current->numchild_++;
              queryBBIntersects_noload (current->children_[i], min, max, query_depth, bin_name);
            }
          }
        }
      }
    }
////////////////////////////////////////////////////////////////////////////////

  }//namespace outofcore
}//namespace pcl

//#define PCL_INSTANTIATE....

#endif //PCL_OCTREE_BASE_NODE_IMPL_H_
