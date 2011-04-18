/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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
 * Author: Julius Kammerl (julius@kammerl.de)
 */

#ifndef OCTREE_NODE_H
#define OCTREE_NODE_H

// maximum depth of octree as we are using "unsigned int" octree keys / bit masks
#define OCT_MAXTREEDEPTH ( sizeof(unsigned int) * 8  )

namespace pcl
{
  namespace octree
  {

    // enum of node types within the octree
    enum node_type_t
    {
      BRANCH_NODE, LEAF_NODE
    };

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief @b Abstract octree node class
     * \note Every octree node should implement the getNodeType () method
     * \author Julius Kammerl (julius@kammerl.de)
     */
    class OctreeNode
    {
    public:

      /** \brief Pure virtual method for receiving the type of octree node (branch or leaf)  */
      virtual node_type_t
      getNodeType () const = 0;
    };

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief @b Abstract octree leaf class
     * \note Octree leafs may collect data of type DataT
     * \author Julius Kammerl (julius@kammerl.de)
     */
    template<typename DataT>
      class OctreeLeafAbstract : public OctreeNode
      {
      public:

        typedef DataT leaf_data_t;

        /** \brief Empty constructor. */
        OctreeLeafAbstract ()
        {
        }
        /** \brief Empty deconstructor. */
        ~OctreeLeafAbstract ()
        {
        }

        /** \brief Get the type of octree node. Returns LEAVE_NODE type */
        virtual node_type_t
        getNodeType () const
        {
          return LEAF_NODE;
        }

        /** \brief Pure virtual method for storing data into the octree node
         *  \param data_arg: reference to DataT element to be stored.
         */
        virtual void
        setData (const leaf_data_t& data_arg) = 0;

        /** \brief Pure virtual method for retrieving a single DataT element from the octree leaf node
         *  \param data_arg: reference to return pointer of leaf node DataT element.
         */
        virtual void
        getData (const DataT*& data_arg) = 0;

        /** \brief Pure virtual method for retrieving a vector of DataT elements from the octree laef node
         *  \param dataVector_arg: reference to DataT vector that is extended with leaf node DataT elements.
         */
        virtual void
        getData (std::vector<leaf_data_t>& dataVector_arg) = 0;

        /** \brief Pure virtual method for resetting the data storage of the octree leaf node */
        virtual void
        reset () = 0;

      };

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief @b Octree leaf class that does not store any information.
     * \note Can be used for occupancy trees that are used for checking only the existence of leaf nodes in the tree
     * \author Julius Kammerl (julius@kammerl.de)
     */
    template<typename DataT>
      class OctreeLeafEmpty : public OctreeLeafAbstract<DataT>
      {
      public:
        /** \brief Empty constructor. */
        OctreeLeafEmpty ()
        {
        }

        /** \brief Empty deconstructor. */
        ~OctreeLeafEmpty ()
        {
        }

        /** \brief Empty setData data implementation. This leaf node does not store any data.
         *  \param data_arg: reference to dummy DataT element to be stored.
         */

        virtual void
        setData (const DataT& data_arg)
        {
        }

        /** \brief Returns a null pointer as this leaf node does not store any data.
         *  \param data_arg: reference to return pointer of leaf node DataT element (will be set to NULL).
         */
        virtual void
        getData (const DataT*& data_arg)
        {
          data_arg = NULL;
        }

        /** \brief Empty getData data vector implementation as this leaf node does not store any data. \
       *  \param dataVector_arg: reference to dummy DataT vector that is extended with leaf node DataT elements.
         */
        virtual void
        getData (std::vector<DataT>& dataVector_arg)
        {
        }

        /** \brief Empty reset leaf node implementation as this leaf node does not store any data. */
        virtual void
        reset ()
        {
        }
      };

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief @b Octree leaf class that does store a single DataT element.
     * \note Enables the octree to store a single DataT element within its leaf nodes.
     * \author Julius Kammerl (julius@kammerl.de)
     */
    template<typename DataT>
      class OctreeLeafDataT : public OctreeLeafAbstract<DataT>
      {
      public:

        /** \brief Empty constructor. */
        OctreeLeafDataT ()
        {
        }

        /** \brief Empty deconstructor. */
        ~OctreeLeafDataT ()
        {
        }

        /** \brief Copies a DataT element to leaf node memorye.
         *  \param data_arg: reference to DataT element to be stored within leaf node.
         * */
        virtual void
        setData (const DataT& data_arg)
        {
          this->data_ = data_arg;
        }

        /** \brief Retrieve a pointer to the leaf node DataT element.
         *  \param data_arg: reference to return pointer of leaf node DataT element.
         * */
        virtual void
        getData (const DataT*& data_arg)
        {
          data_arg = &data_;
        }

        /** \brief Adds leaf node DataT element to dataVector vector of type DataT.
         *  \param dataVector_arg: reference to DataT vector that is to be extended with leaf node DataT elements.
         * */
        virtual void
        getData (std::vector<DataT>& dataVector_arg)
        {
          dataVector_arg.push_back (this->data_);
        }

        /** \brief Reset leaf node memory to zero. */
        virtual void
        reset ()
        {
          memset (&data_, 0, sizeof(data_));
        }

      protected:
        /** \brief Leaf node DataT storage. */
        DataT data_;

      };

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief @b Octree leaf class that does store a vector of DataT elements.
     * \note Enables the octree to store multiple DataT elements within its leaf nodes.
     * \author Julius Kammerl (julius@kammerl.de)
     */
    template<typename DataT>
      class OctreeLeafDataTVector : public OctreeLeafAbstract<DataT>
      {

      public:
        /** \brief Empty constructor. */
        OctreeLeafDataTVector ()
        {
        }
        /** \brief Empty deconstructor. */
        ~OctreeLeafDataTVector ()
        {
        }

        /** \brief Pushes a DataT element to internal DataT vector.
         *  \param data_arg: reference to DataT element to be stored within leaf node.
         * */
        virtual void
        setData (const DataT& data_arg)
        {
          leafDataTVector_.push_back (data_arg);
        }

        /** \brief Receive the most recent DataT element that was pushed to the internal DataT vector.
         *  \param data_arg: reference to return pointer of most recently added leaf node DataT element.
         * */
        virtual void
        getData (const DataT*& data_arg)
        {
          DataT* result = NULL;

          if (leafDataTVector_.size () > 0)
            result = (DataT*)&leafDataTVector_.back ();

          result = result;

        }

        /** \brief Concatenate the internal DataT vector to vector argument dataVector_arg.
         * \param dataVector_arg: reference to DataT vector that is to be extended with leaf node DataT elements.
         * */
        virtual void
        getData (std::vector<DataT>& dataVector_arg)
        {

          dataVector_arg.insert (dataVector_arg.end (), leafDataTVector_.begin (), leafDataTVector_.end ());
        }

        /** \brief Reset leaf node. Clear DataT vector.*/
        virtual void
        reset ()
        {
          leafDataTVector_.clear ();
        }

      protected:
        /** \brief Leaf node DataT vector. */
        std::vector<DataT> leafDataTVector_;

      };
  }
}

#endif
