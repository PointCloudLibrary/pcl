#ifndef PCL_OUTOFCORE_OCTREE_BASE_NODE_H_
#define PCL_OUTOFCORE_OCTREE_BASE_NODE_H_

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

// Boost
#include "boost/filesystem.hpp"

#pragma warning(push)
#pragma warning(disable: 4311 4312)
#include <boost/thread.hpp>
#pragma warning(pop)

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int.hpp>
#include <boost/random/bernoulli_distribution.hpp>

// PCL (Urban Robotics)
#include "pcl/outofcore/octree_base.h"
#include "pcl/outofcore/pointCloudTools.h"
// Forward Declarations
template<typename Container, typename PointT>
class octree_base_node;

template<typename Container, typename PointT>
class octree_base;

template<typename Container, typename PointT> octree_base_node<Container, PointT>*
makenode_norec (const boost::filesystem::path& path, octree_base_node<Container, PointT>* super);

//read from specified level
template<typename Container, typename PointT> void
queryBBIntersects_noload (const boost::filesystem::path& rootnode, const double min[3], const double max[3], const boost::uint32_t query_depth, std::list<std::string>& bin_name);

template<typename Container, typename PointT> void
queryBBIntersects_noload (octree_base_node<Container, PointT>* current, const double min[3], const double max[3], const boost::uint32_t query_depth, std::list<std::string>& bin_name);


template<typename Container, typename PointT>
class octree_base_node
{
  friend class octree_base<Container, PointT> ;

  friend octree_base_node<Container, PointT>*
  makenode_norec<Container, PointT> (const boost::filesystem::path& path, octree_base_node<Container, PointT>* super);

  friend void
  queryBBIntersects_noload<Container, PointT> (const boost::filesystem::path& rootnode, const double min[3], const double max[3], const boost::uint32_t query_depth, std::list<std::string>& bin_name);

  friend void
  queryBBIntersects_noload<Container, PointT> (octree_base_node<Container, PointT>* current, const double min[3], const double max[3], const boost::uint32_t query_depth, std::list<std::string>& bin_name);

  public:
    const static std::string node_index_basename;
    const static std::string node_container_basename;
    const static std::string node_index_extension;
    const static std::string node_container_extension;
    const static double sample_precent;

    octree_base_node ()
    {
      parent = NULL;
      root_ = NULL;
      depth = 0;

      memset (children, 0, 8 * sizeof(octree_base_node<Container, PointT>*));
      numchild = 0;

      midx = 0;
      midy = 0;
      midz = 0;
      memset (min, 0, 3 * sizeof(double));
      memset (max, 0, 3 * sizeof(double));

    }

    /** \brief Load from disk 
     * If creating root, path is full name. If creating any other
     * node, path is dir; throws OCT_MISSING_DIR and OCT_MISSING_IDX
     *
     * \param path
     * \param super
     * \param loadAll
     */
    octree_base_node (const boost::filesystem::path& path, octree_base_node<Container, PointT>* super, bool loadAll);

    /** \brief Create root node and directory
     *
     * Initializes the root node and performs initial filesystem checks for the octree; 
     * throws OctreeException::OCT_BAD_PATH if root directory is an existing file
     *
     * \param bbmin triple of x,y,z minima for bounding box
     * \param bbmax triple of x,y,z maxima for bounding box
     * \param tree adress of the tree data structure that will hold this initial root node
     * \param rootname Root directory for location of on-disk octree storage; if directory 
     * doesn't exist, it is created; if "rootname" is an existing file, throws 
     * OctreeException::OCT_BAD_PATH
     */
    void init_root_node (const double bbmin[3], const double bbmax[3], octree_base<Container, PointT> * const tree, const boost::filesystem::path& rootname);

    /** \brief Create root node and directory setting voxel size*/
    octree_base_node (const double bbmin[3], const double bbmax[3], const double node_dim_meters, octree_base<Container, PointT> * const tree, const boost::filesystem::path& rootname);

    /** \brief Create root node and directory setting setting max depth*/
    octree_base_node (const int maxdepth, const double bbmin[3], const double bbmax[3], octree_base<Container, PointT> * const tree, const boost::filesystem::path& rootname);

    /** \brief Will recursively delete all children calling recFreeChildrein */
    ~octree_base_node ();

    //query
    inline void
    getBB (double minCoord[3], double maxCoord[3]) const
    {
      memcpy (minCoord, min, 3 * sizeof(double));
      memcpy (maxCoord, max, 3 * sizeof(double));
    }

    void
    createChild (const int idx);

    void
    createChildren ();//add empty children

    //void
    //createChildrenToDim(const double dim);//add empty children until their bounding box is less than dim meters on a side

    int
    calcDepthForDim (const double minbb[3], const double maxbb[3], const double dim);

    void
    freeChildren ();

    void
    recFreeChildren ();

    //point extraction
    void
    queryBBIncludes (const double minbb[3], const double maxbb[3], size_t query_depth, std::list<PointT>& v);

    void
    queryBBIncludes_subsample (const double minbb[3], const double maxbb[3], int query_depth, const double percent, std::list<PointT>& v);

    //bin extraction
    //query_depth == 0 is root
    //query_depth == tree->depth is leaf
    void
    queryBBIntersects (const double minbb[3], const double maxbb[3], const boost::uint32_t query_depth, std::list<std::string>& file_names);

    //bb check
    inline bool
    intersectsWithBB (const double minbb[3], const double maxbb[3]) const;

    inline bool
    withinBB (const double min[3], const double max[3]) const;

    static inline bool
    pointWithinBB (const double minbb[3], const double maxbb[3], const PointT& p);

    /** \brief Check whether specified point is within bounds of current node */
    inline bool
    pointWithinBB (const PointT& p) const;

    // todo: All of the add data methods check whether the point being added is
    //       within the bounds of the octree created.  This requires a traversal
    //       of the cloud to find the min/max before proceeding to adding the
    //       points.

    //add point to this node if we are a leaf, or find the leaf below us that is supposed to take the point
    boost::uint64_t
    addDataToLeaf (const std::vector<PointT>& p, const bool skipBBCheck);

    boost::uint64_t
    addDataToLeaf (const std::vector<const PointT*>& p, const bool skipBBCheck);

    /** \brief Recursively add points to the leaf and children subsampling LODs
     * on the way down.
     *
     * rng_mutex lock occurs
     */
    boost::uint64_t
    addDataToLeaf_and_genLOD (const std::vector<PointT>& p,
                              const bool skipBBCheck);

// todo: Do we need to support std::vector<PointT*>
//    boost::uint64_t
//    addDataToLeaf_and_genLOD (const std::vector<PointT*>& p,
//                              const bool skipBBCheck);

    /** \brief Add data when at max depth of tree */
    boost::uint64_t
    addDataAtMaxDepth (const std::vector<PointT>& p, const bool skipBBCheck);

    /** \brief Randomly sample point data */
    void
    randomSample(const std::vector<PointT>& p, std::vector<PointT>& insertBuff, const bool skipBBCheck);

    /** \brief Subdivide points to pass to child nodes */
    void
    subdividePoints (const std::vector<PointT>& p, std::vector< std::vector<PointT> >& c, const bool skipBBCheck);


    /** \brief Subdivide a single point into a spefici child node */
    void
    subdividePoint (const PointT& pt, std::vector< std::vector<PointT> >& c);

    /** \brief Number of points in the payload */
    inline boost::uint64_t
    size () const
    {
      return payload->size ();
    }

    /** \brief Number of children (0 or 8). */
    inline size_t
    numchildren () const
    {
      return numchild;
    }

    /** \brief Flush payload's cache to disk */
    void
    flushToDisk ();

    void
    flushToDiskLazy ();

    void
    flush_DeAlloc_this_only ();

    //write a vpyton vis script
    void
    writeVPythonVisual (std::ofstream& file);

    /** \brief Serialize */
    void
    saveToFile (const boost::filesystem::path& path);

    void
    loadFromFile (const boost::filesystem::path& path, octree_base_node* super);

    void
    saveIdx (bool recursive);

    void
    convertToXYZ ();

  private:
    //no copy construction right now
    octree_base_node (const octree_base_node& rval);

    octree_base_node&
    operator= (const octree_base_node& rval);

    //empty node in tree
    octree_base_node (const double bbmin[3], const double bbmax[3], const char* dir, octree_base_node<Container, PointT>* super);

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

    boost::filesystem::path thisdir;//the dir containing the node's data and its children
    boost::filesystem::path thisnodeindex;//the node's index file, node.idx
    boost::filesystem::path thisnodestorage;//the node's storage file, node.dat

    /** \brief The tree we belong to */
    octree_base<Container, PointT>* m_tree_;//
    /** \brief The root node of the tree we belong to */
    octree_base_node* root_;//
    /** \brief super-node */
    octree_base_node* parent;
    /** \brief Depth in the tree, root is 0, root's children are 1, ... */
    size_t depth;
    /** \brief The children of this node */
    octree_base_node* children[8];
    /** \brief number of children this node has. Between 0 and 8 inclusive */
    size_t numchild;

    /** \brief what holds the points. currently a custom class, but in theory
     * you could use an stl container if you rewrote some of this class. I used
     * to use deques for this...
     */
    Container* payload;

    /** \brief The bounding box and middle point */
    double min[3];
    double max[3];
    double midx;
    double midy;
    double midz;

    /** \brief Random number generator mutex */
    static boost::mutex rng_mutex;

    /** \brief Mersenne Twister: A 623-dimensionally equidistributed uniform
     * pseudo-random number generator
     */
    static boost::mt19937 rand_gen;

    /** \brief Random number generator seed */
    const static boost::uint32_t rngseed = 0xAABBCCDD;

};
#endif //PCL_OUTOFCORE_OCTREE_BASE_NODE_H_
