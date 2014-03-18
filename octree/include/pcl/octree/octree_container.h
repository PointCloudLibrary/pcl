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

#ifndef PCL_OCTREE_OCTREE_CONTAINER_H
#define PCL_OCTREE_OCTREE_CONTAINER_H

#include <vector>
#include <cstddef>
#include <set>

#include <boost/blank.hpp>

#include <pcl/point_types.h>
#include <pcl/pcl_macros.h>

namespace pcl
{

  namespace octree
  {

    /** An octree leaf container class that serves as a base to construct
      * specialized leaf node container classes.
      *
      * Has a single template parameter UserDataT, which allows the user to
      * store some arbitrary piece of information inside a leaf.
      *
      * \note Deriving container classes should specialize LeafContainerTraits,
      * see its documentation. */
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

        /** Empty virtual destructor. */
        virtual
        ~OctreeLeafContainer ()
        {
        }

        /** Deep copy of the leaf, allocates memory and copies the user data
          * that is stored internally. */
        virtual OctreeLeafContainer*
        deepCopy () const
        {
          OctreeLeafContainer* new_container = new OctreeLeafContainer;
          new_container->user_data_ = user_data_;
          return (new_container);
        }

        /** Equal comparison operator.
          *
          * \param[in] other OctreeLeafContainer to compare with
          *
          * \note This function always returns false. A deriving container
          * class should override this if it has any reasonable way to compare
          * itself to another container. */
        virtual bool
        operator== (const OctreeLeafContainer&) const
        {
          return (false);
        }

        /** Inequal comparison operator.
          *
          * \param[in] other OctreeLeafContainer to compare with */
        bool
        operator!= (const OctreeLeafContainer& other) const
        {
          return (!operator== (other));
        }

        /** Clear the data contained in the node. */
        virtual void
        reset ()
        {
          user_data_ = UserDataT ();
        }

        /** Returns a reference to the internal user data member. */
        UserDataT&
        getUserData ()
        {
          return (user_data_);
        }

        /** Returns the size of the container.
          *
          * This base implementation always returns zero. A deriving container
          * class may override if it has a concept of size.
          *
          * This function is retained for compatibility with older versions. */
        virtual size_t
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
    };

    /** Typedef to preserve existing interface. */
    typedef OctreeEmptyContainer OctreeContainerEmpty;

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

  }

}

#endif

