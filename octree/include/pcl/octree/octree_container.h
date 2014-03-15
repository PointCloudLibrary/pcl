/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
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

#ifndef PCL_OCTREE_CONTAINER_H
#define PCL_OCTREE_CONTAINER_H

#include <vector>
#include <cstddef>
#include <set>

#include <boost/blank.hpp>

#include <pcl/pcl_macros.h>
#include <pcl/octree/octree_leaf_data.h>
#include <pcl/octree/centroid_point.h>

namespace pcl
{
  namespace octree
  {

    /** \brief An octree leaf container class that serves as a base to construct
      * specialized leaf node container classes.
      *
      * Has a single template parameter UserDataT, which allows the user to
      * store some arbitrary piece of information inside a leaf.
      *
      * Deriving container classes should specialize LeafContainerTraits, see
      * its documentation. */
    template <typename UserDataT = boost::blank>
    class OctreeLeafContainer
    {

      public:

        /// Point type stored in this leaf container. In fact, nothing is known
        /// about point type so far. A deriving leaf container that actually
        /// stores points will have to redefine it. Here semantically it would
        /// be better to typedef point_type to e.g. void, however it would
        /// result in issues with LeafContainerTraits on older boost versions
        /// ( < 1.53). So we typedef to some non-sense.
        typedef PointXY point_type;

        /// User data type stored in this leaf container.
        typedef UserDataT data_type;

        /** \brief Empty constructor. */
        OctreeLeafContainer ()
        {
        }

        /** \brief Empty deconstructor. */
        virtual
        ~OctreeLeafContainer ()
        {
        }

        /** \brief Deep copy of the leaf - copies all internal data. */
        virtual OctreeLeafContainer*
        deepCopy () const
        {
          OctreeLeafContainer* new_container = new OctreeLeafContainer;
          new_container->user_data_ = user_data_;
          return new_container;
        }

        /** \brief Equal comparison operator.
          *
          * \param[in] other OctreeLeafContainer to compare with
          *
          * \note This function always returns false. A deriving container
          * class should override this if it has any reasonable way to compare
          * itself to another container. */
        virtual bool
        operator== (const OctreeLeafContainer&) const
        {
          return false;
        }

        /** \brief Inequal comparison operator.
          *
          * \param[in] other OctreeLeafContainer to compare with */
        bool
        operator!= (const OctreeLeafContainer& other) const
        {
          return (!operator== (other));
        }

        /** \brief Clear the data contained in the node. */
        virtual void
        reset ()
        {
          user_data_ = UserDataT ();
        }

        /** \brief Returns reference to the internal user data member. */
        UserDataT&
        getUserData ()
        {
          return user_data_;
        }

        size_t
        getSize () const
        {
          return (0);
        }

        /** DEPRECATED This is maintained because of octree_pointcloud.hpp:521 TODO Investigate...
         * \brief Empty getPointIndices implementation as this leaf node does not store any data. \
         */
        void
        getPointIndices (std::vector<int>&) const
        {
        }

      protected:

        UserDataT user_data_;

    };

    /** An empty octree leaf container.
      *
      * We need this to be a separate type (not a typedef) because otherwise we
      * might run into issues with traits. */
    class OctreeEmptyContainer : public OctreeLeafContainer<boost::blank>
    {

        typedef OctreeLeafContainer<boost::blank> OctreeLeafContainerT;

      public:

        OctreeEmptyContainer ()
        : OctreeLeafContainerT ()
        {
        }

    };

    /** \brief @b Octree adjacency leaf container class - stores set of pointers to neighbors, number of points added, and a DataT value.
     * \note This class implements a leaf node that stores pointers to neighboring leaves.
     * \note This class also has a virtual computeData function, which is called by OctreePointCloudAdjacency::addPointsFromInputCloud.
     * \note If you are not happy with the default data type (which is AveragePoint<PointInT>) and its behavior (averaging of XYZ,
     * normal, and RGB fields), then you should implement your own and either:
     *
     * - make sure it has add() and compute() functions (like AveragePoint does)
     *   or
     * - make explicit instantiations of addPoint() and computeData() functions of this class (supervoxel_clustering.hpp for an example).
     */
    template <typename OctreeLeafContainerT = OctreeEmptyContainer>
    class OctreeAdjacencyContainer : public OctreeLeafContainerT
    {

      public:

        typedef OctreeAdjacencyContainer<OctreeLeafContainerT> OctreeAdjacencyContainerT;

        typedef std::set<OctreeAdjacencyContainerT*> NeighborSetT;

        /** Iterator for neighbors of this leaf */
        typedef typename NeighborSetT::iterator iterator;
        /** Const iterator for neighbors of this leaf */
        typedef typename NeighborSetT::const_iterator const_iterator;
        inline iterator begin () { return (neighbors_.begin ()); }
        inline iterator end ()   { return (neighbors_.end ()); }
        inline const_iterator begin () const { return (neighbors_.begin ()); }
        inline const_iterator end () const  { return (neighbors_.end ()); }

        OctreeAdjacencyContainer ()
        : OctreeLeafContainerT ()
        {
        }

        /** \brief Deep copy of the leaf - copies all internal data. */
        virtual OctreeAdjacencyContainerT*
        deepCopy () const
        {
          OctreeAdjacencyContainerT *new_data = new OctreeAdjacencyContainerT;
          new_data->neighbors_ = this->neighbors_;
          new_data->user_data_ = this->user_data_;
          return (new_data);
        }

        /** \brief Add a new neighbor to the container.
          * \param[in] neighbor the new neighbor to add */
        void
        addNeighbor (OctreeAdjacencyContainerT *neighbor)
        {
          neighbors_.insert (neighbor);
        }

        /** \brief Remove neighbor from neighbor set.
          * \param[in] neighbor the neighbor to remove */
        void
        removeNeighbor (OctreeAdjacencyContainerT *neighbor)
        {
          neighbors_.erase (neighbor);
        }

        /** Returns the number of neighbors this leaf has. */
        inline size_t
        getNumNeighbors () const
        {
          return (neighbors_.size ());
        }

        /** \brief Sets the whole neighbor set.
          * \param[in] neighbor_arg the new set */
        void
        setNeighbors (const NeighborSetT &neighbor_arg)
        {
          neighbors_ = neighbor_arg;
        }

        /** \brief Clears the set of neighbors */
        void
        resetNeighbors ()
        {
          neighbors_.clear ();
        }

      protected:

        NeighborSetT neighbors_;

    };

    /** \brief An octree leaf container that stores indices of the points that
      * are inserted into it. */
    template <typename UserDataT = boost::blank>
    class OctreeIndicesContainer : public OctreeLeafContainer<UserDataT>
    {

        typedef OctreeLeafContainer<UserDataT> OctreeLeafContainerT;

      public:

        OctreeIndicesContainer ()
        : OctreeLeafContainerT ()
        {
        }

        /** Add a new point (index). */
        void
        insertPointIndex (int index_arg)
        {
          indices_.push_back (index_arg);
        }

        std::vector<int>
        getPointIndicesVector () const
        {
          return (indices_);
        }

        void
        getPointIndices (std::vector<int>& indices) const
        {
          indices.insert (indices.end (), indices_.begin (), indices_.end ());
        }

        size_t
        getSize () const
        {
          return (indices_.size ());
        }

        virtual void
        reset ()
        {
          indices_.clear ();
        }

      protected:

        std::vector<int> indices_;

    };

    /** \brief An octree leaf container that stores the index of the very last
      * point that was added to it. */
    template <typename UserDataT = boost::blank>
    class OctreeIndexContainer : public OctreeLeafContainer<UserDataT>
    {

        typedef OctreeLeafContainer<UserDataT> OctreeLeafContainerT;

      public:

        OctreeIndexContainer ()
        : OctreeLeafContainerT ()
        , last_index_ (-1)
        {
        }

        /** Add a new point (index). */
        void
        insertPointIndex (int index_arg)
        {
          last_index_ = index_arg;
        }

        /** \brief Retrieve the index of the last point. */
        int
        getPointIndex () const
        {
          return (last_index_);
        }

        virtual void
        reset ()
        {
          last_index_ = -1;
        }

      protected:

        int last_index_;

    };

    /** \brief An octree leaf container that stores the number of points
      * that were added to it. */
    template <typename UserDataT = boost::blank>
    class OctreeDensityContainer : public OctreeLeafContainer<UserDataT>
    {

        typedef OctreeLeafContainer<UserDataT> OctreeLeafContainerT;

      public:

        OctreeDensityContainer ()
        : OctreeLeafContainerT ()
        , num_points_ (0)
        {
        }

        /** Add a new point. */
        void
        insertPoint ()
        {
          ++num_points_;
        }

        /** \brief Get the number of points that have been inserted. */
        size_t
        getSize () const
        {
          return (num_points_);
        }

        virtual void
        reset ()
        {
          num_points_ = 0;
        }

      protected:

        size_t num_points_;

    };

    /** \brief An octree leaf container that computes the cenrtoid of the points
      * that were inserted into it. */
    template <typename PointT = pcl::PointXYZ,
              typename UserDataT = boost::blank>
    class OctreeCentroidContainer : public OctreeLeafContainer<UserDataT>
    {

        typedef OctreeLeafContainer<UserDataT> OctreeLeafContainerT;

      public:

        typedef PointT point_type;
        typedef UserDataT data_type;

        OctreeCentroidContainer ()
        : OctreeLeafContainerT ()
        , num_points_ (0)
        {
        }

        /** Add a new point. */
        template <typename T> void
        insertPoint (const T& point_arg)
        {
          ++num_points_;
          xyz_.add (point_arg);
          normal_.add (point_arg);
          color_.add (point_arg);
        }

        /** \brief Retrieve the computed average (centroid) point.
          *
          * This container maintains the sum of all fields of the
          * inserted points. When this function is called it will divide the
          * sum by the actual number of inserted points. */
        template <typename T> void
        getCentroid (T& point_arg) const
        {
          xyz_.get (point_arg);
          normal_.get (point_arg);
          color_.get (point_arg);
        }

        /** \brief Get the number of points that have been inserted. */
        size_t
        getSize () const
        {
          return (num_points_);
        }

        virtual void
        reset ()
        {
          xyz_ = detail::xyz_accumulator ();
          normal_ = detail::normal_accumulator ();
          color_ = detail::color_accumulator ();
          num_points_ = 0;
        }

      protected:

        detail::xyz_accumulator xyz_;
        detail::normal_accumulator normal_;
        detail::color_accumulator color_;

        size_t num_points_;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    };

    /** Typedef to preserve existing interface. */
    typedef OctreeEmptyContainer OctreeContainerEmpty;

    /** Typedef to preserve existing interface. */
    typedef OctreeIndexContainer<boost::blank> OctreeContainerPointIndex;

    /** Typedef to preserve existing interface. */
    typedef OctreeIndicesContainer<boost::blank> OctreeContainerPointIndices;

    /** Typedef to preserve existing interface. */
    typedef OctreeDensityContainer<boost::blank> OctreePointCloudDensityContainer;

    /** Wrapper class to preserve existing interface. */
    template <typename PointT>
    class OctreePointCloudVoxelCentroidContainer : public OctreeCentroidContainer<PointT, boost::blank>
    {

      public:

        OctreePointCloudVoxelCentroidContainer ()
        : OctreeCentroidContainer<PointT, boost::blank> ()
        {
        }

    };

    /** LeafContainerTraits serves as an adapter for different leaf containers
      * that provides a uniform interface for point insertion.
      *
      * Various containers may or may not store points and/or their indices, so
      * their functions for point insertion have different signatures. This
      * adapter provides a single insert() function that takes leaf container,
      * point index, and point value, and calls appropriate insert function of
      * the container forwarding arguments as needed. Example usage:
      *
      * \code
      * int idx;
      * PointXYZ p;
      * OctreeIndexContainer container;
      * LeafContainerTraits<OctreeIndexContainer>::insert (container, idx, p);
      * \endcode
      *
      * This traits structure has to be specialized for each leaf container
      * type. Note that it is not necessary to specialize for a container that
      * derives from a container that already has a specialization. */
    template <typename LeafContainerT, typename Enable = void>
    struct LeafContainerTraits;

    template<typename LeafContainerT>
    struct LeafContainerTraits<LeafContainerT,
                               typename boost::enable_if<
                                 boost::is_base_of<
                                   OctreeEmptyContainer
                                 , LeafContainerT
                                 >
                               >::type>
    {
      template <typename PointT>
      static void insert (LeafContainerT&, int, const PointT&)
      {
      }
    };

    template<typename LeafContainerT>
    struct LeafContainerTraits<LeafContainerT,
                               typename boost::enable_if<
                                 boost::is_base_of<
                                   OctreeIndicesContainer<typename LeafContainerT::data_type>
                                 , LeafContainerT
                                 >
                               >::type>
    {
      template <typename PointT>
      static void insert (LeafContainerT& container, int idx, const PointT&)
      {
        container.insertPointIndex (idx);
      }
    };

    template <typename LeafContainerT>
    struct LeafContainerTraits<LeafContainerT,
                               typename boost::enable_if<
                                 boost::is_base_of<
                                   OctreeIndexContainer<typename LeafContainerT::data_type>
                                 , LeafContainerT
                                 >
                               >::type>
    {
      template <typename PointT>
      static void insert (LeafContainerT& container, int idx, const PointT&)
      {
        container.insertPointIndex (idx);
      }
    };

    template <typename LeafContainerT>
    struct LeafContainerTraits<LeafContainerT,
                               typename boost::enable_if<
                                 boost::is_base_of<
                                   OctreeDensityContainer<typename LeafContainerT::data_type>
                                 , LeafContainerT
                                 >
                               >::type>
    {
      template <typename PointT>
      static void insert (LeafContainerT& container, int, const PointT&)
      {
        container.insertPoint ();
      }
    };

    template <typename LeafContainerT>
    struct LeafContainerTraits<LeafContainerT,
                               typename boost::enable_if<
                                 boost::is_base_of<
                                   OctreeCentroidContainer<
                                     typename LeafContainerT::point_type
                                   , typename LeafContainerT::data_type
                                   >
                                 , LeafContainerT
                                 >
                               >::type>
    {
      static void insert (LeafContainerT& container, int, const typename LeafContainerT::point_type& point)
      {
        container.insertPoint (point);
      }
    };

  }

}

#endif

