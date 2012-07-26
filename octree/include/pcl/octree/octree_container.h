/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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
 * $Id: octree_nodes.h 5596 2012-04-17 15:09:31Z jkammerl $
 */

#ifndef PCL_OCTREE_CONTAINER_H
#define PCL_OCTREE_CONTAINER_H

#include <string.h>
#include <vector>
#include <cstddef>

#include <pcl/pcl_macros.h>

using namespace std;

namespace pcl
{
  namespace octree
  {
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
       /** \brief @b Octree container class that can serve as a base to construct own leaf node container classes.
        *  \author Julius Kammerl (julius@kammerl.de)
        */
       template<typename DataT>
         class OctreeContainerBase
         {
         public:
           /** \brief Empty constructor. */
           OctreeContainerBase ()
           {
           }

           /** \brief Empty constructor. */
           OctreeContainerBase (const OctreeContainerBase&)
           {
           }

           /** \brief Empty deconstructor. */
           virtual
           ~OctreeContainerBase ()
           {
           }

           /** \brief Octree deep copy method */
           virtual OctreeContainerBase *
           deepCopy () const
           {
             return (new OctreeContainerBase (*this));
           }

           /** \brief Empty setData data implementation.
            */

           void
           setData (const DataT&)
           {
           }

           /** \brief Empty getData data vector implementation.
            */
           void
           getData (DataT&) const
           {
           }


           /** \brief Empty getData data vector implementation
            */
           void
           getData (std::vector<DataT>&) const
           {
           }

           /** \brief Get size of container
            * \return number of elements in leaf node container.
            */
           size_t
           getSize () const
           {
             return 0;
           }
         };

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief @b Octree container class that does not store any information.
     * \note Can be used for occupancy trees that are used for checking only the existence of leaf nodes in the tree
     * \author Julius Kammerl (julius@kammerl.de)
     */
    template<typename DataT>
      class OctreeContainerEmpty
      {
      public:
        /** \brief Empty constructor. */
        OctreeContainerEmpty ()
        {
        }

        /** \brief Empty constructor. */
        OctreeContainerEmpty (const OctreeContainerEmpty&)
        {
        }

        /** \brief Empty deconstructor. */
        virtual
        ~OctreeContainerEmpty ()
        {
        }

        /** \brief Octree deep copy method */
        virtual OctreeContainerEmpty *
        deepCopy () const
        {
          return (new OctreeContainerEmpty (*this));
        }

        /** \brief Empty setData data implementation. This leaf node does not store any data.
         */

        void
        setData (const DataT&)
        {
        }

        /** \brief Empty getData data vector implementation as this leaf node does not store any data.
         */
        void
        getData (DataT&) const
        {
        }


        /** \brief Empty getData data vector implementation as this leaf node does not store any data. \
         */
        void
        getData (std::vector<DataT>&) const
        {
        }

        /** \brief Get size of container (number of DataT objects)
         * \return number of DataT elements in leaf node container.
         */
        size_t
        getSize () const
        {
          return 0;
        }

        /** \brief Empty reset leaf node implementation as this leaf node does not store any data. */
        virtual void
        reset ()
        {
        }
      };

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief @b Octree container class that does store a single DataT element.
     * \note Enables the octree to store a single DataT element within its leaf nodes.
     * \author Julius Kammerl (julius@kammerl.de)
     */
    template<typename DataT>
      class OctreeContainerDataT
      {
      public:
        /** \brief Empty constructor. */
        OctreeContainerDataT () :
            data_ (),
            isEmpty_(true)
        {
          reset ();
        }

        /** \brief Empty constructor. */
        OctreeContainerDataT (const OctreeContainerDataT& source) :
            data_ (source.data_), isEmpty_ (source.isEmpty_)
        {
        }

        /** \brief Empty deconstructor. */
        virtual
        ~OctreeContainerDataT ()
        {
        }

        /** \brief Octree deep copy method */
        virtual OctreeContainerDataT*
        deepCopy () const
        {
          return (new OctreeContainerDataT (*this));
        }

        /** \brief Copies a DataT element to leaf node memorye.
         * \param[in] data_arg reference to DataT element to be stored within leaf node.
         */
        void
        setData (const DataT& data_arg)
        {
          this->data_ = data_arg;
          isEmpty_ = false;
        }

        /** \brief Adds leaf node DataT element to dataVector vector of type DataT.
         * \param[in] dataVector_arg: reference to DataT type to obtain the most recently added leaf node DataT element.
         */
        void
        getData (DataT& dataVector_arg) const
        {
          if (!isEmpty_)
            dataVector_arg = this->data_;
        }

        /** \brief Adds leaf node DataT element to dataVector vector of type DataT.
         * \param[in] dataVector_arg: reference to DataT vector that is to be extended with leaf node DataT elements.
         */
        void
        getData (vector<DataT>& dataVector_arg) const
        {
          if (!isEmpty_)
            dataVector_arg.push_back (this->data_);
        }

        /** \brief Get size of container (number of DataT objects)
         * \return number of DataT elements in leaf node container.
         */
        size_t
        getSize () const
        {
          return isEmpty_ ? 0 : 1;
        }

        /** \brief Reset leaf node memory to zero. */
        virtual void
        reset ()
        {
          isEmpty_ = true;
        }
      protected:
        /** \brief Leaf node DataT storage. */
        DataT data_;

        /** \brief Bool indicating if leaf node is empty or not. */
        bool isEmpty_;
      };

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief @b Octree container class that does store a vector of DataT elements.
     * \note Enables the octree to store multiple DataT elements within its leaf nodes.
     * \author Julius Kammerl (julius@kammerl.de)
     */
    template<typename DataT>
      class OctreeContainerDataTVector
      {
      public:
        /** \brief Empty constructor. */
        OctreeContainerDataTVector () :
            leafDataTVector_ ()
        {
        }

        /** \brief Empty constructor. */
        OctreeContainerDataTVector (const OctreeContainerDataTVector& source) :
            leafDataTVector_ (source.leafDataTVector_)
        {
        }

        /** \brief Empty deconstructor. */
        virtual
        ~OctreeContainerDataTVector ()
        {
        }

        /** \brief Octree deep copy method */
        virtual OctreeContainerDataTVector *
        deepCopy () const
        {
          return (new OctreeContainerDataTVector (*this));
        }

        /** \brief Pushes a DataT element to internal DataT vector.
         * \param[in] data_arg reference to DataT element to be stored within leaf node.
         */
        void
        setData (const DataT& data_arg)
        {
          leafDataTVector_.push_back (data_arg);
        }

        /** \brief Receive the most recent DataT element that was pushed to the internal DataT vector.
         * \param[in] data_arg reference to DataT type to obtain the most recently added leaf node DataT element.
         */
        void
        getData (DataT& data_arg) const
        {
          if (leafDataTVector_.size () > 0)
            data_arg = leafDataTVector_.back ();
        }

        /** \brief Concatenate the internal DataT vector to vector argument dataVector_arg.
         * \param[in] dataVector_arg: reference to DataT vector that is to be extended with leaf node DataT elements.
         */
        void
        getData (std::vector<DataT>& dataVector_arg) const
        {
          dataVector_arg.insert (dataVector_arg.end (),
              leafDataTVector_.begin (), leafDataTVector_.end ());
        }

        /** \brief Return const reference to internal DataT vector
         * \return  const reference to internal DataT vector
         */
        const std::vector<DataT>& getDataTVector () const
        {
          return leafDataTVector_;
        }

        /** \brief Get size of container (number of DataT objects)
         * \return number of DataT elements in leaf node container.
         */
        size_t
        getSize () const
        {
          return leafDataTVector_.size ();
        }

        /** \brief Reset leaf node. Clear DataT vector.*/
        virtual void
        reset ()
        {
          leafDataTVector_.clear ();
        }

      protected:
        /** \brief Leaf node DataT vector. */
        vector<DataT> leafDataTVector_;
      };

  }
}

#endif
