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
 * $Id$
 */

#ifndef OCTREE_NODE_H
#define OCTREE_NODE_H

// maximum depth of octree as we are using "unsigned int" octree keys / bit masks
#define OCT_MAXTREEDEPTH ( sizeof(unsigned int) * 8  )

#include <string.h>

#include <pcl/pcl_macros.h>

namespace pcl
{
  namespace octree
  {

    // enum of node types within the octree
    enum node_type_t
    {
      BRANCH_NODE, LEAF_NODE, LEAF_BRANCH
    };

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief @b Abstract octree node class
      * \note Every octree node should implement the getNodeType () method
      * \author Julius Kammerl (julius@kammerl.de)
      */
    class PCL_EXPORTS OctreeNode
    {
      public:

        virtual ~OctreeNode () {}
        /** \brief Pure virtual method for receiving the type of octree node (branch or leaf)  */
        virtual node_type_t 
        getNodeType () const = 0;

        /** \brief Pure virtual method to perform a deep copy of the octree */
        virtual OctreeNode* 
        deepCopy () const = 0;
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

      /** \brief pure virtual octree deep copy method */
      virtual OctreeNode* deepCopy () const = 0;

      /** \brief Get the type of octree node. Returns LEAVE_NODE type */
      inline virtual node_type_t getNodeType () const {return LEAF_NODE;}

      /** \brief Pure virtual method for storing data into the octree node
       *  \param data_arg: reference to DataT element to be stored.
       */
      virtual void
        setData (const leaf_data_t& data_arg) = 0;

      /** \brief Pure virtual method for retrieving a single DataT element from the octree leaf node
       *  \param data_arg: reference to return pointer of leaf node DataT element.
       */
      virtual void
        getData (const DataT*& data_arg) const = 0;

      /** \brief Pure virtual method for retrieving a vector of DataT elements from the octree laef node
       *  \param dataVector_arg: reference to DataT vector that is extended with leaf node DataT elements.
       */
      virtual void
        getData (std::vector<leaf_data_t>& dataVector_arg) const = 0;

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

      /** \brief Octree deep copy method */
      virtual OctreeNode *
      deepCopy () const
      {
        return (OctreeNode*) new OctreeLeafEmpty (*this);
      }

      /** \brief Empty setData data implementation. This leaf node does not store any data.
       *  \param data_arg: reference to dummy DataT element to be stored.
       */

      virtual void
        setData (const DataT& data_arg)
      {
      }

      /** \brief Returns a null pointer as this leaf node does not store any data.
       *  \param data_arg: reference to return pointer of leaf node DataT element (will be set to 0).
       */
      virtual void
        getData (const DataT*& data_arg) const
      {
        data_arg = 0;
      }

      /** \brief Empty getData data vector implementation as this leaf node does not store any data. \
       *  \param dataVector_arg: reference to dummy DataT vector that is extended with leaf node DataT elements.
       */
      virtual void
        getData (std::vector<DataT>& dataVector_arg) const
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
        OctreeLeafDataT () : data_ ()
        {
        }

        /** \brief Empty deconstructor. */
        ~OctreeLeafDataT ()
        {
        }

        /** \brief Octree deep copy method */
        virtual OctreeNode*
        deepCopy () const
        {
          return (OctreeNode*) new OctreeLeafDataT (*this);
        }

        /** \brief Copies a DataT element to leaf node memorye.
          * \param[in] data_arg reference to DataT element to be stored within leaf node.
          */
        virtual void
        setData (const DataT& data_arg)
        {
          this->data_ = data_arg;
        }

        /** \brief Retrieve a pointer to the leaf node DataT element.
          * \param[in] data_arg reference to return pointer of leaf node DataT element.
          */
        virtual void
        getData (const DataT*& data_arg) const
        {
          data_arg = &data_;
        }

        /** \brief Adds leaf node DataT element to dataVector vector of type DataT.
          * \param[in] dataVector_arg: reference to DataT vector that is to be extended with leaf node DataT elements.
          */
        virtual void
        getData (std::vector<DataT>& dataVector_arg) const
        {
          dataVector_arg.push_back (this->data_);
        }

        /** \brief Reset leaf node memory to zero. */
        virtual void
        reset ()
        {
          memset (&data_, 0, sizeof (data_));
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
        OctreeLeafDataTVector () : leafDataTVector_ ()
        {
        }

        /** \brief Empty deconstructor. */
        ~OctreeLeafDataTVector ()
        {
        }

        /** \brief Octree deep copy method */
        virtual OctreeNode *
        deepCopy () const
        {
          return ((OctreeNode*) new OctreeLeafDataTVector (*this));
        }

        /** \brief Pushes a DataT element to internal DataT vector.
          * \param[in] data_arg reference to DataT element to be stored within leaf node.
          */
        virtual void
        setData (const DataT& data_arg)
        {
          leafDataTVector_.push_back (data_arg);
        }

        /** \brief Receive the most recent DataT element that was pushed to the internal DataT vector.
          * \param[in] data_arg reference to return pointer of most recently added leaf node DataT element.
          */
        virtual void
        getData (const DataT*& data_arg) const
        {
          DataT* result = 0;

          if (leafDataTVector_.size () > 0)
            result = (DataT*)&leafDataTVector_.back ();

          data_arg = result;
        }

        /** \brief Concatenate the internal DataT vector to vector argument dataVector_arg.
          * \param[in] dataVector_arg: reference to DataT vector that is to be extended with leaf node DataT elements.
          */
        virtual void
        getData (std::vector<DataT>& dataVector_arg) const
        {
          dataVector_arg.insert (dataVector_arg.end (), leafDataTVector_.begin (), leafDataTVector_.end ());
        }

        /** \brief Receive const reference to internal DataT Vector
          * \return reference to internal DataT Vector
          */
        virtual const std::vector<DataT>&
        getIdxVector ()
        {
          return leafDataTVector_;
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

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**\brief @b Octree branch class.
     * \note It stores 8 pointers to its child nodes.
     * \author Julius Kammerl (julius@kammerl.de)
     */
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    class OctreeBranch : public OctreeNode
    {
      public:
        typedef const OctreeNode *octree_node_ptr;

        /** \brief Constructor for initializing child node pointer array. */
        OctreeBranch ()
        {
          memset (this->subNodes_, 0, sizeof (this->subNodes_));
        }
        
        /** \brief Copy constructor */
        OctreeBranch (const OctreeBranch& source)
        {
          memset (subNodes_, 0, sizeof (subNodes_));
          // iterate over the 8 child nodes
          for (unsigned int i=0; i<8; i++)
          {
            if (source.subNodes_[i] != 0)
              subNodes_[i] = source.subNodes_[i]->deepCopy ();
          }
        }

        /** \brief Octree deep copy function */
        virtual OctreeNode*
        deepCopy () const
        {
          return ((OctreeNode*) new OctreeBranch (*this));
        }

        /** \brief Copy operator
          * \param[in] o the octree branch to copy into this
          */
        inline OctreeBranch&
        operator = (const OctreeBranch& o)
        {
          *this = o;
          return (*this);
        }

        /** \brief Empty deconstructor. */
        virtual ~OctreeBranch () {}

        /** \brief Get the type of octree node. Returns BRANCH_NODE type
          * \return Returns BRANCH_NODE type.
          */
        inline virtual node_type_t
        getNodeType () const {return BRANCH_NODE;}

        /** \brief access operator. 
          * \return const OctreeNode pointer
          */
        inline const octree_node_ptr& 
        operator[] (size_t pos) const { return (subNodes_[pos]); }

        /** \brief access operator. 
          * \return OctreeNode pointer
          */
        inline octree_node_ptr& operator[] (size_t pos) {return subNodes_[pos];}

        /** \brief Reset child node pointer array. */
        inline void 
        reset () { memset (subNodes_, 0, sizeof (subNodes_)); }

      private:
        /** \brief Child node pointer array of size 8.  */
        octree_node_ptr subNodes_[8];
    };


  }
}

#endif
