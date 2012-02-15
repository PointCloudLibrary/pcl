/*
  Copyright (c) 2012, Urban Robotics Inc
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
  * Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in the
  documentation and/or other materials provided with the distribution.
  * Neither the name of Urban Robotics Inc nor the
  names of its contributors may be used to endorse or promote products
  derived from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/*
  This code defines the octree used for point storage at Urban Robotics. Please
  contact Jacob Schloss <jacob.schloss@urbanrobotics.net> with any questions.
  http://www.urbanrobotics.net/
*/

#pragma once

// C++
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <exception>

// Boost
#pragma warning(push)
#pragma warning(disable : 4311 4312)
#include "boost/thread.hpp"
#pragma warning(pop)
#include "boost/filesystem.hpp"
#include "boost/foreach.hpp"

// PCL (Urban Robotics)
#include "pcl/outofcore/octree_base.h"
#include "pcl/outofcore/octree_base_node.h"
#include "pcl/outofcore/octree_exceptions.h"

#include "pcl/outofcore/pointCloudTools.h"

#include "pcl/outofcore/impl/octree_disk_container.hpp"
#include "pcl/outofcore/impl/octree_ram_container.hpp"

// JSON
#include "pcl/outofcore/cJSON.h"


template<typename Container, typename PointType>
const std::string octree_base_node<Container, PointType>::node_index_basename = "node";

template<typename Container, typename PointType>
const std::string octree_base_node<Container, PointType>::node_container_basename = "node";

template<typename Container, typename PointType>
const std::string octree_base_node<Container, PointType>::node_index_extension = ".oct_idx";

template<typename Container, typename PointType>
const std::string octree_base_node<Container, PointType>::node_container_extension = ".oct_dat";

template<typename Container, typename PointType>
boost::mutex octree_base_node<Container, PointType>::rng_mutex;

template<typename Container, typename PointType>
boost::mt19937 octree_base_node<Container, PointType>::rand_gen;//(rngseed);

template<typename Container, typename PointType>
const double octree_base_node<Container, PointType>::sample_precent = .125;

template<typename Container, typename PointType>
octree_base_node<Container, PointType>::octree_base_node (const boost::filesystem::path& path,
                                                          octree_base_node<Container, PointType>* super,
                                                          bool loadAll)
{
  if (super == NULL)
  {
    thisdir = path.parent_path ();

    if (!boost::filesystem::exists (thisdir))
    {
      std::cerr << "could not find dir!" << thisdir << "\n";
      throw(OctreeException::OCT_MISSING_DIR);
    }

    thisnodeindex = path;

    depth = 0;
    root = this;
  }
  else
  {
    thisdir = path;
    depth = super->depth + 1;
    root = super->root;

    boost::filesystem::directory_iterator diterend;
    bool loaded = false;
    for (boost::filesystem::directory_iterator diter (thisdir); diter != diterend; ++diter)
    {
      const boost::filesystem::path& file = *diter;
      if (!boost::filesystem::is_directory (file))
      {
        if (boost::filesystem::extension (file) == node_index_extension)
        {
          thisnodeindex = file;
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

  loadFromFile (thisnodeindex, super);

  if (loadAll)
  {
    loadChildren (true);
  }
}

template<typename Container, typename PointType> void
octree_base_node<Container, PointType>::init_root_node (const double bbmin[3], const double bbmax[3],
                                                        octree_base<Container, PointType> * const tree,
                                                        const boost::filesystem::path& rootname)
{
  parent = NULL;
  root = this;
  m_tree = tree;
  depth = 0;

  // Allocate space for 8 child nodes
  memset (children, 0, 8 * sizeof(octree_base_node<Container, PointType>*));
  numchild = 0;

  // Set bounding box and mid point
  memcpy (min, bbmin, 3 * sizeof(double));
  memcpy (max, bbmax, 3 * sizeof(double));
  midx = (max[0] + min[0]) / double (2);
  midy = (max[1] + min[1]) / double (2);
  midz = (max[2] + min[2]) / double (2);

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
  /** \todo: getRandomUUIDString shouldn't be in class octree_disk_container */
  std::string uuid;
  octree_disk_container<PointType>::getRandomUUIDString (uuid);
  std::string node_container_name = uuid + std::string ("_") + node_container_basename + node_container_extension;

  // Setup all file paths related to this node
  thisdir = boost::filesystem::path (dir);
  thisnodeindex = thisdir / rootname.filename ();
  thisnodestorage = thisdir / boost::filesystem::path (node_container_name);
  boost::filesystem::create_directory (thisdir);

  // Create data container, ie octree_disk_container, octree_ram_container
  payload = new Container (thisnodestorage);
}

template<typename Container, typename PointType>
octree_base_node<Container, PointType>::octree_base_node (const double bbmin[3], const double bbmax[3],
                                                          const double node_dim_meters,
                                                          octree_base<Container, PointType> * const tree,
                                                          const boost::filesystem::path& rootname)
{
  init_root_node(bbmin, bbmax, tree, rootname);

  // Calculate the max depth but don't create nodes
  tree->maxDepth = calcDepthForDim (bbmin, bbmax, node_dim_meters);
  saveIdx (false);
  //createChildrenToDim(node_dim_meters);
}

template<typename Container, typename PointType>
octree_base_node<Container, PointType>::octree_base_node (
                                                          const int maxdepth,
                                                          const double bbmin[3], const double bbmax[3],
                                                          octree_base<Container, PointType> * const tree,
                                                          const boost::filesystem::path& rootname)
{
  init_root_node(bbmin, bbmax, tree, rootname);

  // Set max depth but don't create nodes
  tree->maxDepth = maxdepth;
  saveIdx (false);
}

template<typename Container, typename PointType>
octree_base_node<Container, PointType>::~octree_base_node ()
{
  // Recursively delete all children and this nodes data
  recFreeChildren ();
  delete payload;
}

template<typename Container, typename PointType> inline bool
octree_base_node<Container, PointType>::hasUnloadedChildren () const
{
  unsigned int numChildDirs = 0;
  // Check nodes directory for children directories 0-7
  for (int i = 0; i < 8; i++)
  {
    boost::filesystem::path childdir = thisdir / boost::filesystem::path (boost::lexical_cast<std::string> (i));
    if (boost::filesystem::exists (childdir))
    {
      numChildDirs++;
    }
  }

  // If found directories is less than the current nodes loaded children
  if (numChildDirs > numchild)
    return true;

  return false;
}

template<typename Container, typename PointType> void
octree_base_node<Container, PointType>::loadChildren (bool recursive)
{
  // todo: hasChildrenLoaded checks numChild against how many child
  //       directories live on disk.  This just bails if anything is loaded?
  if (numchild != 0)
  {
    std::cerr << "Calling loadChildren on a node that already has loaded children! - skipping";
    return;
  }

  // Create a new node for each child directory that exists
  for (int i = 0; i < 8; i++)
  {
    boost::filesystem::path childdir = thisdir / boost::filesystem::path (boost::lexical_cast<std::string> (i));
    if (boost::filesystem::exists (childdir))
    {
      this->children[i] = new octree_base_node<Container, PointType> (childdir, this, recursive);
      numchild++;
    }
  }
}

template<typename Container, typename PointType> void
octree_base_node<Container, PointType>::recFreeChildren ()
{
  if (numchild == 0)
  {
    return;
  }

  for (size_t i = 0; i < 8; i++)
  {
    if (children[i])
    {
      octree_base_node<Container, PointType>* current = children[i];
      delete current;
    }
  }
  memset (children, 0, 8 * sizeof(octree_base_node<Container, PointType>*));
  numchild = 0;
}


template<typename Container, typename PointType> boost::uint64_t
octree_base_node<Container, PointType>::addDataToLeaf (const std::vector<PointType>& p, 
                                                       const bool skipBBCheck)
{
  if (p.empty ())
  {
    return 0;
  }

  if (this->depth == root->m_tree->maxDepth)
    return addDataAtMaxDepth(p, skipBBCheck);

  if (numchild < 8)
    if (hasUnloadedChildren ())
      loadChildren (false);

  std::vector < std::vector<const PointType*> > c;
  c.resize (8);
  for (size_t i = 0; i < 8; i++)
  {
    c[i].reserve (p.size () / 8);
  }

  const size_t len = p.size ();
  for (size_t i = 0; i < len; i++)
  {
    const PointType& pt = p[i];

    if (!skipBBCheck)
    {
      if (!this->pointWithinBB (pt))
      {
        //	std::cerr << "failed to place point!!!" << std::endl;
        continue;
      }
    }

    if ((pt.z >= midz))
    {
      if ((pt.y >= midy))
      {
        if ((pt.x >= midx))
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
        if ((pt.x >= midx))
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
      if ((pt.y >= midy))
      {
        if ((pt.x >= midx))
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
        if ((pt.x >= midx))
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
    if (!children[i])
      createChild (i);
    points_added += children[i]->addDataToLeaf (c[i], true);
    c[i].clear ();
  }
  return points_added;

}



//return number of points added
template<typename Container, typename PointType> boost::uint64_t
octree_base_node<Container, PointType>::addDataToLeaf (const std::vector<const PointType*>& p,
                                                       const bool skipBBCheck)
{
  if (p.empty ())
  {
    return 0;
  }

  if (this->depth == root->m_tree->maxDepth)
  {
    if (skipBBCheck)//trust me, just add the points
    {
      root->m_tree->count_point (this->depth, p.size ());
      payload->insertRange (p.data (), p.size ());
      return p.size ();
    }
    else//check which points belong to this node, throw away the rest
    {
      std::vector<const PointType*> buff;
      BOOST_FOREACH(const PointType* pt, p)
      {
        if(pointWithinBB(*pt))
        {
          buff.push_back(pt);
        }
      }

      if (!buff.empty ())
      {
        root->m_tree->count_point (this->depth, buff.size ());
        payload->insertRange (buff.data (), buff.size ());
      }
      return buff.size ();
    }
  }
  else
  {
    if (numchild < 8)
    {
      if (hasUnloadedChildren ())
      {
        loadChildren (false);
      }
    }

    std::vector < std::vector<const PointType*> > c;
    c.resize (8);
    for (int i = 0; i < 8; i++)
    {
      c[i].reserve (p.size () / 8);
    }

    const size_t len = p.size ();
    for (size_t i = 0; i < len; i++)
    {
      //const PointType& pt = p[i];
      if (!skipBBCheck)
      {
        if (!this->pointWithinBB (*p[i]))
        {
          //	std::cerr << "failed to place point!!!" << std::endl;
          continue;
        }
      }

      if ((p[i]->z >= midz))
      {
        if ((p[i]->y >= midy))
        {
          if ((p[i]->x >= midx))
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
          if ((p[i]->x >= midx))
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
        if ((p[i]->y >= midy))
        {
          if ((p[i]->x >= midx))
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
          if ((p[i]->x >= midx))
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
      if (!children[i])
        createChild (i);
      points_added += children[i]->addDataToLeaf (c[i], true);
      c[i].clear ();
    }
    return points_added;
  }
  // std::cerr << "failed to place point!!!" << std::endl;
  return 0;
}

// todo: This seems like a lot of work to get a random uniform sample?
// todo: Need to refactor this further as to not pass in a BBCheck
template<typename Container, typename PointType> void
octree_base_node<Container, PointType>::randomSample(const std::vector<PointType>& p, 
                                                     std::vector<PointType>& insertBuff, 
                                                     const bool skipBBCheck)
{
//    std::cout << "randomSample" << std::endl;
  std::vector<PointType> sampleBuff;
  if (!skipBBCheck)
  {
    BOOST_FOREACH (const PointType& pt, p)
    if(pointWithinBB(pt))
      sampleBuff.push_back(pt);
  }
  else
  {
    sampleBuff = p;
  }

  // Derive percentage from specified sample_precent and tree depth
  const double percent = pow(sample_precent, double((root->m_tree->maxDepth - depth)));
  const boost::uint64_t samplesize = (boost::uint64_t)(percent * double(sampleBuff.size()));
  const boost::uint64_t inputsize = sampleBuff.size();

  if(samplesize > 0)
  {
    // Resize buffer to sample size
    insertBuff.resize(samplesize);

    // Create random number generator
    boost::mutex::scoped_lock lock(rng_mutex);
    boost::uniform_int<boost::uint64_t> buffdist(0, inputsize-1);
    boost::variate_generator<boost::mt19937&, boost::uniform_int<boost::uint64_t> > buffdie(rand_gen, buffdist);

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
    boost::mutex::scoped_lock lock(rng_mutex);
    boost::bernoulli_distribution<double> buffdist(percent);
    boost::variate_generator<boost::mt19937&, boost::bernoulli_distribution<double> > buffcoin(rand_gen, buffdist);

    for(boost::uint64_t i = 0; i < inputsize; ++i)
      if(buffcoin())
        insertBuff.push_back( p[i] );
  }
}


template<typename Container, typename PointType> boost::uint64_t
octree_base_node<Container, PointType>::addDataAtMaxDepth (const std::vector<PointType>& p, const bool skipBBCheck)
{
  //    std::cout << "addDataAtMaxDepth" << std::endl;
  // Trust me, just add the points
  if(skipBBCheck)
  {
    // Increment point count for node
    root->m_tree->count_point (this->depth, p.size ());
    // Insert point data
    payload->insertRange (p.data (), p.size ());
    return p.size ();
  }
  // Add points found within the current nodes bounding box
  else
  {
    std::vector<PointType> buff;
    const size_t len = p.size ();

    for (size_t i = 0; i < len; i++)
      if (pointWithinBB (p[i]))
        buff.push_back (p[i]);

    if (!buff.empty ())
    {
      root->m_tree->count_point (this->depth, buff.size ());
      payload->insertRange (buff.data (), buff.size ());
    }
    return buff.size ();
  }
}

template<typename Container, typename PointType> void
octree_base_node<Container, PointType>::subdividePoints (const std::vector<PointType>& p,
                                                         std::vector< std::vector<PointType> >& c,
                                                         const bool skipBBCheck)
{
  // Reserve space for children nodes
  c.resize(8);
  for(int i = 0; i < 8; i++)
    c[i].reserve(p.size() / 8);

  const size_t len = p.size();
  for(size_t i = 0; i < len; i++)
  {
    const PointType& pt = p[i];

    if(!skipBBCheck)
      if(!this->pointWithinBB(pt))
        continue;

    subdividePoint(pt, c);
  }
}

template<typename Container, typename PointType> void
octree_base_node<Container, PointType>::subdividePoint (const PointType& pt,
                                                        std::vector< std::vector<PointType> >& c)
{
  if((pt.z >= midz))
  {
    if((pt.y >= midy))
    {
      if((pt.x >= midx))
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
      if((pt.x >= midx))
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
    if((pt.y >= midy))
    {
      if((pt.x >= midx))
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
      if((pt.x >= midx))
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

template<typename Container, typename PointType> boost::uint64_t
octree_base_node<Container, PointType>::addDataToLeaf_and_genLOD (const std::vector<PointType>& p, const bool skipBBCheck)
{
  // If there's no points return
  if (p.empty ())
    return 0;

  // todo: Why is skipBBCheck set to false when adding points at max depth
  //       when adding data and generating sampled LOD
  // If the max depth has been reached
  if (this->depth == root->m_tree->maxDepth)
    return addDataAtMaxDepth(p, false);

  // Create child nodes of the current node but not grand children+
  if (numchild < 8)
    if (hasUnloadedChildren ())
      loadChildren (false);

  // Randomly sample data
  std::vector<PointType> insertBuff;
  randomSample(p, insertBuff, skipBBCheck);

  if(!insertBuff.empty())
  {
    // Increment point count for node
    root->m_tree->count_point (this->depth, insertBuff.size());
    // Insert sampled point data
    payload->insertRange ( &(insertBuff.front ()), insertBuff.size());
  }

  //subdivide vec to pass data down lower
  std::vector< std::vector<PointType> > c;
  subdividePoints(p, c, skipBBCheck);

  // todo: Perhaps do a quick loop through the lists here and dealloc the
  //       reserved mem for empty lists
  boost::uint64_t points_added = 0;
  for(int i = 0; i < 8; i++)
  {
    // If child doesn't have points
    if(c[i].empty())
      continue;

    // If child doesn't exist
    if(!children[i])
      createChild(i);

    // todo: Why are there no bounding box checks on the way down?
    // Recursively build children
    points_added += children[i]->addDataToLeaf_and_genLOD(c[i], true);
    c[i].clear();
  }

  return points_added;

  // todo: Make sure I didn't break anything by removing the if/else above
  // std::cerr << "failed to place point!!!" << std::endl;
  //return 0;
}

// todo: Do we need to support std::vector<PointType*>
//template<typename Container, typename PointType>
//  boost::uint64_t
//  octree_base_node<Container, PointType>::addDataToLeaf_and_genLOD (const std::vector<PointType*>& p,
//                                                                     const bool skipBBCheck)
//  {
//    if (p.empty ())
//    {
//      return 0;
//    }
//
//    if (this->depth == root->m_tree->maxDepth)
//    {
//      //if(skipBBCheck)//trust me, just add the points
//      if (false)//trust me, just add the points
//      {
//        root->m_tree->count_point (this->depth, p.size ());
//        payload->insertRange (p.data (), p.size ());
//        return p.size ();
//      }
//      else//check which points belong to this node, throw away the rest
//      {
//        std::vector<PointType> buff;
//
//        const size_t len = p.size ();
//        for (size_t i = 0; i < len; i++)
//        {
//          //const PointType& pt = p[i];
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
//        root->m_tree->count_point (this->depth, buff.size ());
//        payload->insertRange (buff.data (), buff.size ());
//        return buff.size ();
//      }
//    }
//    else
//    {
//      if (numchild < 8)
//      {
//        if (hasUnloadedChildren ())
//        {
//          loadChildren (false);
//        }
//      }
//
//      //add code to subsample here
//      std::vector<PointType> insertBuff;
//      {
//        std::vector<PointType> sampleBuff;
//        if (!skipBBCheck)
//        {
//BOOST_FOREACH        (const PointType& pt, p)
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
//      const double percent = pow(sample_precent, double((root->m_tree->maxDepth - depth)));
//      const boost::uint64_t samplesize = (boost::uint64_t)(percent * double(sampleBuff.size()));
//      const boost::uint64_t inputsize = sampleBuff.size();
//      if(samplesize > 0)
//      {
//        insertBuff.resize(samplesize);
//
//        boost::mutex::scoped_lock lock(rng_mutex);
//        boost::uniform_int<boost::uint64_t> buffdist(0, inputsize-1);
//        boost::variate_generator<boost::mt19937&, boost::uniform_int<boost::uint64_t> > buffdie(rand_gen, buffdist);
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
//        boost::mutex::scoped_lock lock(rng_mutex);
//        boost::bernoulli_distribution<double> buffdist(percent);
//        boost::variate_generator<boost::mt19937&, boost::bernoulli_distribution<double> > buffcoin(rand_gen, buffdist);
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
//      root->m_tree->count_point(this->depth, insertBuff.size());
//      payload->insertRange(&(insertBuff.front()), insertBuff.size());
//    }
//    //end subsample
//
//    //subdivide vec to pass data down lower
//    std::vector<PointType*> c;
//    c.resize(8);
//    for(int i = 0; i < 8; i++)
//    {
//      c[i].reserve(p.size() / 8);
//    }
//
//    const size_t len = p.size();
//    for(size_t i = 0; i < len; i++)
//    {
//      //const PointType& pt = p[i];
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
//      if((p[i].z >= midz))
//      {
//        if((p[i].y >= midy))
//        {
//          if((p[i].x >= midx))
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
//          if((p[i].x >= midx))
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
//        if((p[i].y >= midy))
//        {
//          if((p[i].x >= midx))
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
//          if((p[i].x >= midx))
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

template<typename Container, typename PointType> void
octree_base_node<Container, PointType>::createChild (const int idx)
{
  if (children[idx] || (numchild == 8))
    return;

  const double zstart = min[2];
  const double ystart = min[1];
  const double xstart = min[0];

  const double zstep = (max[2] - min[2]) / double (2);
  const double ystep = (max[1] - min[1]) / double (2);
  const double xstep = (max[0] - min[0]) / double (2);

  double childbb_min[3];
  double childbb_max[3];
  /*
    int x,y,z;
    if(idx > 3)
    {
    x = ((idx == 6) || (idx == 7)) ? 1 : 0;
    y = ((idx == 5) || (idx == 7)) ? 1 : 0;
    z = 1;
    }
    else
    {
    x = ((idx == 2) || (idx == 3)) ? 1 : 0;
    y = ((idx == 1) || (idx == 3)) ? 1 : 0;
    z = 0;
    }
  */

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

  boost::filesystem::path childdir = thisdir / boost::filesystem::path (boost::lexical_cast<std::string> (idx));
  children[idx] = new octree_base_node<Container, PointType> (childbb_min, childbb_max, childdir.string ().c_str (),this);

  numchild++;
}

template<typename Container, typename PointType> void
octree_base_node<Container, PointType>::createChildren ()
{
  const double zstart = min[2];
  const double ystart = min[1];
  const double xstart = min[0];

  const double zstep = (max[2] - min[2]) / double (2);
  const double ystep = (max[1] - min[1]) / double (2);
  const double xstep = (max[0] - min[0]) / double (2);

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

        boost::filesystem::path childdir = thisdir / boost::filesystem::path (boost::lexical_cast<std::string> (i));
        children[i] = new octree_base_node<Container, PointType> (childbb_min, childbb_max,
                                                                  childdir.string ().c_str (), this);
        i++;
      }
    }
  }
  numchild = 8;
}

//template<typename Container, typename PointType>
//void octree_base_node<Container, PointType>::createChildrenToDim(const double dim)
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
//		this->root->maxDepth = this->depth;
//		return;
//	}
//
//	if(numchild == 0)
//	{
//		createChildren();
//	}
//
//	for(size_t i = 0; i < numchild; i++)
//	{
//		children[i]->createChildrenToDim(dim);
//	}
//}

template<typename Container, typename PointType>
int
octree_base_node<Container, PointType>::calcDepthForDim (const double minbb[3], const double maxbb[3],
                                                         const double dim)
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
    return 0;
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

    return 1 + calcDepthForDim (childbb_min, childbb_max, dim);
  }

}

template<typename Container, typename PointType>
inline bool
octree_base_node<Container, PointType>::pointWithinBB (const PointType& p) const
{
  if (((min[0] <= p.x) && (p.x <= max[0])) &&
      ((min[1] <= p.y) && (p.y <= max[1])) &&
      ((min[2] <= p.z) && (p.z <= max[2])))
  {
    return true;
  }
  return false;
}

template<typename Container, typename PointType> void
octree_base_node<Container, PointType>::queryBBIntersects (const double minbb[3], 
                                                           const double maxbb[3],
                                                           const boost::uint32_t query_depth,
                                                           std::list<std::string>& file_names)
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
    if (this->depth < query_depth)
    {
      if (numchild > 0)
      {
        for (size_t i = 0; i < 8; i++)
        {
          if (children[i])
            children[i]->queryBBIntersects (my_min, my_max, query_depth, file_names);
        }
      }
      else if (hasUnloadedChildren ())
      {
        loadChildren (false);

        for (size_t i = 0; i < 8; i++)
        {
          if (children[i])
            children[i]->queryBBIntersects (my_min, my_max, query_depth, file_names);
        }
      }
      return;
    }

    if (!payload->empty ())
    {
      file_names.push_back (payload->path ());
    }
  }
}

//read from lowest level
template<typename Container, typename PointType> void
octree_base_node<Container, PointType>::queryBBIncludes (const double minbb[3], 
                                                         const double maxbb[3],
                                                         size_t query_depth, 
                                                         std::list<PointType>& v)
{

  if (intersectsWithBB (minbb, maxbb))
  {
    if (this->depth < query_depth)
    {
      if ((numchild == 0) && (hasUnloadedChildren ()))
      {
        loadChildren (false);
      }

      if (numchild > 0)
      {
        for (size_t i = 0; i < 8; i++)
        {
          if (children[i])
            children[i]->queryBBIncludes (minbb, maxbb, query_depth, v);
        }
        return;
      }
    }
    else
    {
      if (withinBB (minbb, maxbb))
      {
        std::vector<PointType> payload_cache;
        payload->readRange (0, payload->size (), payload_cache);
        v.insert (v.end (), payload_cache.begin (), payload_cache.end ());
        return;
      }
      else
      {
        std::vector<PointType> payload_cache;
        payload->readRange (0, payload->size (), payload_cache);

        boost::uint64_t len = payload->size ();
        for (boost::uint64_t i = 0; i < len; i++)
        {
          const PointType& p = payload_cache[i];
          if (pointWithinBB (minbb, maxbb, p))
          {
            v.push_back (p);
          }
        }
      }
    }
  }
}

template<typename Container, typename PointType> void
octree_base_node<Container, PointType>::queryBBIncludes_subsample (const double minbb[3], const double maxbb[3], 
                                                                   int query_depth, const double percent, std::list<PointType>& v)
{
  if (intersectsWithBB (minbb, maxbb))
  {
    if (this->depth < query_depth)
    {
      if ((numchild == 0) && (hasUnloadedChildren ()))
      {
        loadChildren (false);
      }

      if (numchild > 0)
      {
        for (size_t i = 0; i < 8; i++)
        {
          if (children[i])
            children[i]->queryBBIncludes_subsample (minbb, maxbb, query_depth, percent, v);
        }
        return;
      }
    }
    else
    {
      if (withinBB (minbb, maxbb))
      {
        std::vector<PointType> payload_cache;
        payload->readRangeSubSample (0, payload->size (), percent, payload_cache);
        v.insert (v.end (), payload_cache.begin (), payload_cache.end ());
        return;
      }
      else
      {
        std::vector<PointType> payload_cache_within_region;
        {
          std::vector<PointType> payload_cache;
          payload->readRange (0, payload->size (), payload_cache);
          for (size_t i = 0; i < payload->size (); i++)
          {
            const PointType& p = payload_cache[i];
            if (pointWithinBB (minbb, maxbb, p))
            {
              payload_cache_within_region.push_back (p);
            }
          }
        }//force the payload cache to deconstruct here

        //TODO: this is likely to be very slow.
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

//dir is current level. we put this nodes files into it
template<typename Container, typename PointType>
octree_base_node<Container, PointType>::octree_base_node (const double bbmin[3], const double bbmax[3],
                                                          const char* dir, 
                                                          octree_base_node<Container,PointType>* super)
{
  if (super == NULL)
  {
    std::cerr << "super is null - don't make a root node this way!" << std::endl;
    throw(OctreeException::OCT_BAD_PARENT);
  }

  this->parent = super;
  root = super->root;
  depth = super->depth + 1;

  memset (children, 0, 8 * sizeof(octree_base_node<Container, PointType>*));
  numchild = 0;

  memcpy (min, bbmin, 3 * sizeof(double));
  memcpy (max, bbmax, 3 * sizeof(double));
  midx = (max[0] + min[0]) / double (2);
  midy = (max[1] + min[1]) / double (2);
  midz = (max[2] + min[2]) / double (2);

  std::string uuid_idx;
  std::string uuid_cont;
  octree_disk_container<PointType>::getRandomUUIDString (uuid_idx);
  octree_disk_container<PointType>::getRandomUUIDString (uuid_cont);

  std::string node_index_name = uuid_idx + std::string ("_") + node_index_basename + node_index_extension;
  std::string node_container_name = uuid_cont + std::string ("_") + node_container_basename
  + node_container_extension;

  thisdir = boost::filesystem::path (dir);
  thisnodeindex = thisdir / boost::filesystem::path (node_index_name);
  thisnodestorage = thisdir / boost::filesystem::path (node_container_name);

  boost::filesystem::create_directory (thisdir);

  payload = new Container (thisnodestorage);
  saveIdx (false);
}

template<typename Container, typename PointType>
void
octree_base_node<Container, PointType>::copyAllCurrentAndChildPointsRec (std::list<PointType>& v)
{
  if ((numchild == 0) && (hasUnloadedChildren ()))
  {
    loadChildren (false);
  }

  for (size_t i = 0; i < numchild; i++)
  {
    children[i]->copyAllCurrentAndChildPointsRec (v);
  }

  std::vector<PointType> payload_cache;
  payload->readRange (0, payload->size (), payload_cache);

  {
    //boost::mutex::scoped_lock lock(queryBBIncludes_vector_mutex);
    v.insert (v.end (), payload_cache.begin (), payload_cache.end ());
  }
}

template<typename Container, typename PointType> void
octree_base_node<Container, PointType>::copyAllCurrentAndChildPointsRec_sub (std::list<PointType>& v,
                                                                             const double percent)
{
  if ((numchild == 0) && (hasUnloadedChildren ()))
  {
    loadChildren (false);
  }

  for (size_t i = 0; i < 8; i++)
  {
    if (children[i])
      children[i]->copyAllCurrentAndChildPointsRec_sub (v, percent);
  }

  std::vector<PointType> payload_cache;
  payload->readRangeSubSample (0, payload->size (), percent, payload_cache);

  for (size_t i = 0; i < payload_cache.size (); i++)
  {
    v.push_back (payload_cache[i]);
  }
}

template<typename Container, typename PointType> inline bool
octree_base_node<Container, PointType>::intersectsWithBB (const double minbb[3], const double maxbb[3]) const
{
  if (((min[0] <= minbb[0]) && (minbb[0] <= max[0])) || ((minbb[0] <= min[0]) && (min[0] <= maxbb[0])))
  {
    if (((min[1] <= minbb[1]) && (minbb[1] <= max[1])) || ((minbb[1] <= min[1]) && (min[1] <= maxbb[1])))
    {
      if (((min[2] <= minbb[2]) && (minbb[2] <= max[2])) || ((minbb[2] <= min[2]) && (min[2] <= maxbb[2])))
      {
        return true;
      }
    }
  }

  return false;
}

template<typename Container, typename PointType> inline bool
octree_base_node<Container, PointType>::withinBB (const double minbb[3], const double maxbb[3]) const
{

  if ((minbb[0] <= min[0]) && (max[0] <= maxbb[0]))
  {
    if ((minbb[1] <= min[1]) && (max[1] <= maxbb[1]))
    {
      if ((minbb[2] <= min[2]) && (max[2] <= maxbb[2]))
      {
        return true;
      }
    }
  }

  return false;
}

template<typename Container, typename PointType> inline bool
octree_base_node<Container, PointType>::pointWithinBB (const double minbb[3], const double maxbb[3],
                                                       const PointType& p)
{
  if ((minbb[0] <= p.x) && (p.x <= maxbb[0]))
  {
    if ((minbb[1] <= p.y) && (p.y <= maxbb[1]))
    {
      if ((minbb[2] <= p.z) && (p.z <= maxbb[2]))
      {
        return true;
      }
    }
  }
  return false;
}

template<typename Container, typename PointType> void
octree_base_node<Container, PointType>::writeVPythonVisual (std::ofstream& file)
{
  double l = max[0] - min[0];
  double h = max[1] - min[1];
  double w = max[2] - min[2];
  file << "box( pos=(" << min[0] << ", " << min[1] << ", " << min[2] << "), length=" << l << ", height=" << h
       << ", width=" << w << " )\n";

  for (size_t i = 0; i < numchild; i++)
  {
    children[i]->writeVPythonVisual (file);
  }
}

template<typename Container, typename PointType> void
octree_base_node<Container, PointType>::flush_DeAlloc_this_only ()
{
  payload->flush (true);
}

template<typename Container, typename PointType> void
octree_base_node<Container, PointType>::flushToDisk ()
{
  payload->flush (true);
  for (size_t i = 0; i < 8; i++)
  {
    if (children[i])
      children[i]->flushToDisk ();
  }
}

template<typename Container, typename PointType> void
octree_base_node<Container, PointType>::flushToDiskLazy ()
{
  if (numchild > 0)//only flush if not leaf
  {
    payload->flush (true);
    for (size_t i = 0; i < numchild; i++)
    {
      if (children[i])
        children[i]->flushToDiskLazy ();
    }
  }
}

template<typename Container, typename PointType> void
octree_base_node<Container, PointType>::saveToFile (const boost::filesystem::path& path)
{
  boost::shared_ptr<cJSON> idx (cJSON_CreateObject (), cJSON_Delete);

  cJSON* version = cJSON_CreateNumber (2.0);
  cJSON* bbmin = cJSON_CreateDoubleArray (min, 3);
  cJSON* bbmax = cJSON_CreateDoubleArray (max, 3);

  cJSON* bin = cJSON_CreateString (thisnodestorage.filename ().string ().c_str ());

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

template<typename Container, typename PointType> void
octree_base_node<Container, PointType>::loadFromFile (const boost::filesystem::path& path,
                                                      octree_base_node<Container, PointType>* super)
{
  //load CJSON
  std::vector<char> idx_input;
  boost::uintmax_t len = boost::filesystem::file_size (path);
  idx_input.resize (len + 1);

  std::ifstream f (thisnodeindex.string ().c_str (), std::ios::in);
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
  if ((version->type != cJSON_Number) || (bbmin->type != cJSON_Array) || (bbmax->type != cJSON_Array) || (bin->type
                                                                                                          != cJSON_String))
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
    min[i] = cJSON_GetArrayItem (bbmin, i)->valuedouble;
    max[i] = cJSON_GetArrayItem (bbmax, i)->valuedouble;
  }

  thisnodestorage = thisdir / bin->valuestring;
  this->payload = new Container (thisnodestorage);

  midx = (max[0] + min[0]) / double (2);
  midy = (max[1] + min[1]) / double (2);
  midz = (max[2] + min[2]) / double (2);

  this->parent = super;
  memset (children, 0, 8 * sizeof(octree_base_node<Container, PointType>*));
  this->numchild = 0;
}

template<typename Container, typename PointType>
void
octree_base_node<Container, PointType>::saveIdx (bool recursive)
{
  saveToFile (thisnodeindex);

  if (recursive)
  {
    for (size_t i = 0; i < 8; i++)
    {
      if (children[i])
        children[i]->saveIdx (true);
    }
  }
}

template<typename Container, typename PointType> void
octree_base_node<Container, PointType>::convertToXYZ ()
{
  std::string fname = boost::filesystem::basename (thisnodestorage) + std::string (".dat.xyz");
  boost::filesystem::path xyzfile = thisdir / fname;
  payload->convertToXYZ (xyzfile);

  if (hasUnloadedChildren ())
  {
    loadChildren (false);
  }

  for (size_t i = 0; i < 8; i++)
  {
    if (children[i])
      children[i]->convertToXYZ ();
  }
}

template<typename Container, typename PointType> octree_base_node<Container, PointType>*
makenode_norec (const boost::filesystem::path& path, octree_base_node<Container, PointType>* super)
{
  octree_base_node<Container, PointType>* thisnode = new octree_disk_node ();

  if (super == NULL)
  {
    thisnode->thisdir = path.parent_path ();

    if (!boost::filesystem::exists (thisnode->thisdir))
    {
      std::cerr << "could not find dir!" << thisnode->thisdir << std::endl;
      throw(OctreeException::OCT_BAD_PATH);
    }

    thisnode->thisnodeindex = path;

    thisnode->depth = 0;
    thisnode->root = thisnode;
  }
  else
  {
    thisnode->thisdir = path;
    thisnode->depth = super->depth + 1;
    thisnode->root = super->root;

    if (thisnode->depth > thisnode->root->maxDepth)
    {
      thisnode->root->maxDepth = thisnode->depth;
    }

    boost::filesystem::directory_iterator diterend;
    bool loaded = false;
    for (boost::filesystem::directory_iterator diter (thisnode->thisdir); diter != diterend; ++diter)
    {
      const boost::filesystem::path& file = *diter;
      if (!boost::filesystem::is_directory (file))
      {
        if (boost::filesystem::extension (file) == octree_base_node<Container, PointType>::node_index_extension)
        {
          thisnode->thisnodeindex = file;
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
  thisnode->maxDepth = 0;

  {
    std::ifstream f (thisnode->thisnodeindex.string ().c_str (), std::ios::in);

    f >> thisnode->min[0];
    f >> thisnode->min[1];
    f >> thisnode->min[2];
    f >> thisnode->max[0];
    f >> thisnode->max[1];
    f >> thisnode->max[2];

    std::string filename;
    f >> filename;
    thisnode->thisnodestorage = thisnode->thisdir / filename;

    f.close ();

    thisnode->payload = new Container (thisnode->thisnodestorage);
  }

  thisnode->parent = super;
  memset (thisnode->children, 0, 8 * sizeof(octree_disk_node*));
  thisnode->numchild = 0;

  return thisnode;
}

//accelerate search
template<typename Container, typename PointType> void
queryBBIntersects_noload2 (const boost::filesystem::path& rootnode, const double min[3], 
                           const double max[3], const boost::uint32_t query_depth, 
                           std::list<std::string>& bin_name)
{
  octree_base_node<Container, PointType>* root = makenode_norec<Container, PointType> (rootnode, NULL);
  if (root == NULL)
  {
    std::cout << "test";
  }
  if (root->intersectsWithBB (min, max))
  {
    if (query_depth == root->maxDepth)
    {
      if (!root->payload->empty ())
      {
        bin_name.push_back (root->thisnodestorage.string ());
      }
      return;
    }

    for (int i = 0; i < 8; i++)
    {
      boost::filesystem::path childdir = root->thisdir
      / boost::filesystem::path (boost::lexical_cast<std::string> (i));
      if (boost::filesystem::exists (childdir))
      {
        root->children[i] = makenode_norec (childdir, root);
        root->numchild++;
        queryBBIntersects_noload (root->children[i], min, max, root->maxDepth - query_depth, bin_name);
      }
    }
  }
  delete root;
}

template<typename Container, typename PointType> void
queryBBIntersects_noload (octree_base_node<Container, PointType>* current, const double min[3], const double max[3], const boost::uint32_t query_depth, std::list<std::string>& bin_name)
{
  if (current->intersectsWithBB (min, max))
  {
    if (current->depth == query_depth)
    {
      if (!current->payload->empty ())
      {
        bin_name.push_back (current->thisnodestorage.string ());
      }
    }
    else
    {
      for (int i = 0; i < 8; i++)
      {
        boost::filesystem::path childdir = current->thisdir / boost::filesystem::path (
                                                                                       boost::lexical_cast<
                                                                                       std::string> (i));
        if (boost::filesystem::exists (childdir))
        {
          current->children[i] = makenode_norec<Container, PointType> (childdir, current);
          current->numchild++;
          queryBBIntersects_noload (current->children[i], min, max, query_depth, bin_name);
        }
      }
    }
  }
}
