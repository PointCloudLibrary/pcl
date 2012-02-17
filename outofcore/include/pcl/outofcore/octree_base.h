#ifndef PCL_OUTOFCORE_OCTREE_BASE_H_
#define PCL_OUTOFCORE_OCTREE_BASE_H_

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

// PCL (Urban Robotics)
#include "pcl/outofcore/octree_base_node.h"
#include "pcl/outofcore/pointCloudTools.h"


template<typename Container, typename PointT>
class octree_base
{
  friend class octree_base_node<Container, PointT> ;

  public:

    // Constructors
    // -----------------------------------------------------------------------

    /** \brief Load an existing tree
     *
     * If loadAll is set, the BB and point count for every node is loaded,
     * otherwise only the root node is actually created, and the rest will be
     * generated on insertion or query.
     *
     * \param rootname boost::filesystem::path to existing tree
     * \param loadAll Load entire tree
     */
    octree_base (const boost::filesystem::path& rootname, const bool loadAll);

    /** \brief Create a new tree
     *
     * Create a new tree rootname with specified bounding box.
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
     * \param coord_sys
     */
    octree_base (const double min[3], const double max[3], const double node_dim_meters, const boost::filesystem::path& rootname, const std::string& coord_sys);

    /** \brief Create a new tree
     *
     * Create a new tree rootname with specified bounding box.
     *
     * \param maxdepth Specifies a fixed number of LODs to generate
     * \param min Bounding box min
     * \param max Bounding box max
     * \param rootname must end in ".oct_idx" (THIS SHOULD CHANGE)
     * \param coord_sys
     */
    octree_base (const int maxdepth, const double min[3], const double max[3], const boost::filesystem::path& rootname, const std::string& coord_sys);

    ~octree_base ();

    // Accessors
    // -----------------------------------------------------------------------

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

    /** \brief Access node's PointT */
    Container
    get (const size_t* indexes, const size_t len) const;

    /** \brief Access node's PointT */
    Container&
    get (const size_t* indexes, const size_t len);

    /** \brief Get number of points at specified LOD */
    inline boost::uint64_t
    getNumPoints (const boost::uint64_t depth) const
    {
      return lodPoints_[depth];
    }

    /** \brief Get number of points at each LOD */
    inline const std::vector<boost::uint64_t>&
    getNumPoints () const
    {
      return lodPoints_;
    }

    /** \brief Get number of LODs
     *
     * Assume fully balanced tree -- all nodes have 8 children, and all branches
     * are same depth
     */
    boost::uint64_t
    getDepth () const
    {
      return maxDepth_;
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

      double y_len = root_->max[1] - root_->min[1];
      double x_len = root_->max[0] - root_->min[0];

      y = y_len * pow (.5, double (root_->maxDepth_));
      x = x_len * pow (.5, double (root_->maxDepth_));

      return true;
    }

    /** \brief Get coord system tag in the metadata */
    const std::string&
    get_coord_system ()
    {
      return coord_system_;
    }

    // Mutators
    // -----------------------------------------------------------------------

    /** \brief Generate LODs for the tree */
    void
    buildLOD ();

    /** \brief Recursively add points to the tree */
    boost::uint64_t
    addDataToLeaf (const std::vector<PointT>& p);

    /** \brief Recursively add points to the tree subsampling LODs on the way.
     *
     * shared read_write_mutex lock occurs
     */
    boost::uint64_t
    addDataToLeaf_and_genLOD (const std::vector<PointT>& p);



    // DB Access
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

    //get Points in BB, only points inside BB
    void
    queryBBIncludes (const double min[3], const double max[3], size_t query_depth, std::list<PointT>& v) const;

    void
    queryBBIncludes_subsample (const double min[3], const double max[3], size_t query_depth, const double percent, std::list<PointT>& v) const;

    // Serializers
    // -----------------------------------------------------------------------
    bool
    saveToDisk (const char* path);

    /** \brief Save the index files for each node.  You do not need to call this
     * explicitly
     */
    void
    saveIdx ();

    /** \brief Save each .bin file as an XYZ file */
    void
    convertToXYZ ();

    /** \brief Write a python script using the vpython module containing all
     * the bounding boxes
     */
    void
    writeVPythonVisual (const char* file);

    // The following are DEPRECATED since I found that writeback caches did not
    // scale well, and they are currently disabled in the backend
    /** \brief DEPRECATED - Flush all nodes' cache */
    void
    flushToDisk ();

    /** \brief DEPRECATED - Flush all non leaf nodes' cache */
    void
    flushToDiskLazy ();

    /** \brief DEPRECATED - Flush empty nodes only */
    void
    DeAllocEmptyNodeCache ();

  private:

    octree_base (octree_base& rval);
    octree_base (const octree_base& rval);

    octree_base&
    operator= (octree_base& rval);

    octree_base&
    operator= (const octree_base& rval);

    //flush empty nodes only
    void
    DeAllocEmptyNodeCache (octree_base_node<Container, PointT>* current);

    /** \brief Write octree definition .octree to disk */
    void
    saveToFile ();

    void
    loadFromFile ();

    //recursive portion of lod builder
    void
    buildLOD (octree_base_node<Container, PointT>** current_branch, const int current_dims);

    /** \brief Increment current depths (LOD for branch nodes) point count */
    void
    count_point (boost::uint64_t depth, boost::uint64_t inc)
    {
      lodPoints_[depth] += inc;
    }

    octree_base_node<Container, PointT>* root_;
    mutable boost::shared_mutex read_write_mutex;
    std::vector<boost::uint64_t> lodPoints_;
    boost::uint64_t maxDepth_;
    boost::filesystem::path treepath_;
    std::string coord_system_;

    const static std::string tree_extension_;
 
  };
#endif // PCL_OUTOFCORE_OCTREE_BASE_H_
