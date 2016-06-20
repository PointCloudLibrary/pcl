/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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

#ifndef PCL_OCTREE_TREE_BASE_H
#define PCL_OCTREE_TREE_BASE_H

#include <vector>

#include <pcl/octree/octree_nodes.h>
#include <pcl/octree/octree_container.h>
#include <pcl/octree/octree_key.h>
#include <pcl/octree/octree_iterator.h>

namespace pcl
{
  namespace octree
  {
    /** \brief Octree class
      * \note The tree depth defines the maximum amount of octree voxels / leaf nodes (should be initially defined).
      * \note All leaf nodes are addressed by integer indices.
      * \note Note: The tree depth equates to the bit length of the voxel indices.
     * \ingroup octree
     * \author Julius Kammerl (julius@kammerl.de)
     */
    template<typename LeafContainerT = int,
        typename BranchContainerT = OctreeContainerEmpty >
    class OctreeBase
    {

      public:

        typedef OctreeBase<LeafContainerT, BranchContainerT> OctreeT;

        typedef OctreeBranchNode<BranchContainerT> BranchNode;
        typedef OctreeLeafNode<LeafContainerT> LeafNode;

        typedef BranchContainerT BranchContainer;
        typedef LeafContainerT LeafContainer;

        // iterators are friends
        friend class OctreeIteratorBase<OctreeT> ;
        friend class OctreeDepthFirstIterator<OctreeT> ;
        friend class OctreeBreadthFirstIterator<OctreeT> ;
        friend class OctreeLeafNodeIterator<OctreeT> ;

        // Octree default iterators
        typedef OctreeDepthFirstIterator<OctreeT> Iterator;
        typedef const OctreeDepthFirstIterator<OctreeT> ConstIterator;
        Iterator begin(unsigned int max_depth_arg = 0) {return Iterator(this, max_depth_arg);};
        const Iterator end() {return Iterator();};

        // Octree leaf node iterators
        typedef OctreeLeafNodeIterator<OctreeT> LeafNodeIterator;
        typedef const OctreeLeafNodeIterator<OctreeT> ConstLeafNodeIterator;
        LeafNodeIterator leaf_begin(unsigned int max_depth_arg = 0) {return LeafNodeIterator(this, max_depth_arg);};
        const LeafNodeIterator leaf_end() {return LeafNodeIterator();};

        // Octree depth-first iterators
        typedef OctreeDepthFirstIterator<OctreeT> DepthFirstIterator;
        typedef const OctreeDepthFirstIterator<OctreeT> ConstDepthFirstIterator;
        DepthFirstIterator depth_begin(unsigned int max_depth_arg = 0) {return DepthFirstIterator(this, max_depth_arg);};
        const DepthFirstIterator depth_end() {return DepthFirstIterator();};

        // Octree breadth-first iterators
        typedef OctreeBreadthFirstIterator<OctreeT> BreadthFirstIterator;
        typedef const OctreeBreadthFirstIterator<OctreeT> ConstBreadthFirstIterator;
        BreadthFirstIterator breadth_begin(unsigned int max_depth_arg = 0) {return BreadthFirstIterator(this, max_depth_arg);};
        const BreadthFirstIterator breadth_end() {return BreadthFirstIterator();};


        /** \brief Empty constructor. */
        OctreeBase ();

        /** \brief Empty deconstructor. */
        virtual
        ~OctreeBase ();

        /** \brief Copy constructor. */
        OctreeBase (const OctreeBase& source) :
          leaf_count_ (source.leaf_count_),
          branch_count_ (source.branch_count_),
          root_node_ (new (BranchNode) (*(source.root_node_))),
          depth_mask_ (source.depth_mask_),
          octree_depth_ (source.octree_depth_),
          dynamic_depth_enabled_(source.dynamic_depth_enabled_),
          max_key_ (source.max_key_)
        {
        }

        /** \brief Copy operator. */
        OctreeBase&
        operator = (const OctreeBase &source)
        {
          leaf_count_ = source.leaf_count_;
          branch_count_ = source.branch_count_;
          root_node_ = new (BranchNode) (*(source.root_node_));
          depth_mask_ = source.depth_mask_;
          max_key_ = source.max_key_;
          octree_depth_ = source.octree_depth_;
          return (*this);
        }

        /** \brief Set the maximum amount of voxels per dimension.
          * \param[in] max_voxel_index_arg maximum amount of voxels per dimension
          */
        void
        setMaxVoxelIndex (unsigned int max_voxel_index_arg);

        /** \brief Set the maximum depth of the octree.
         *  \param max_depth_arg: maximum depth of octree
         * */
        void
        setTreeDepth (unsigned int max_depth_arg);

        /** \brief Get the maximum depth of the octree.
         *  \return depth_arg: maximum depth of octree
         * */
        unsigned int
        getTreeDepth () const
        {
          return this->octree_depth_;
        }

        /** \brief Create new leaf node at (idx_x_arg, idx_y_arg, idx_z_arg).
         *  \note If leaf node already exist, this method returns the existing node
         *  \param idx_x_arg: index of leaf node in the X axis.
         *  \param idx_y_arg: index of leaf node in the Y axis.
         *  \param idx_z_arg: index of leaf node in the Z axis.
         *  \return pointer to new leaf node container.
         * */
        LeafContainerT*
        createLeaf (unsigned int idx_x_arg, unsigned int idx_y_arg, unsigned int idx_z_arg);

        /** \brief Find leaf node at (idx_x_arg, idx_y_arg, idx_z_arg).
         *  \note If leaf node already exist, this method returns the existing node
         *  \param idx_x_arg: index of leaf node in the X axis.
         *  \param idx_y_arg: index of leaf node in the Y axis.
         *  \param idx_z_arg: index of leaf node in the Z axis.
         *  \return pointer to leaf node container if found, null pointer otherwise.
         * */
        LeafContainerT*
        findLeaf (unsigned int idx_x_arg, unsigned int idx_y_arg, unsigned int idx_z_arg);

        /** \brief idx_x_arg for the existence of leaf node at (idx_x_arg, idx_y_arg, idx_z_arg).
         *  \param idx_x_arg: index of leaf node in the X axis.
         *  \param idx_y_arg: index of leaf node in the Y axis.
         *  \param idx_z_arg: index of leaf node in the Z axis.
         *  \return "true" if leaf node search is successful, otherwise it returns "false".
         * */
        bool
        existLeaf (unsigned int idx_x_arg, unsigned int idx_y_arg, unsigned int idx_z_arg) const ;

        /** \brief Remove leaf node at (idx_x_arg, idx_y_arg, idx_z_arg).
         *  \param idx_x_arg: index of leaf node in the X axis.
         *  \param idx_y_arg: index of leaf node in the Y axis.
         *  \param idx_z_arg: index of leaf node in the Z axis.
         * */
        void
        removeLeaf (unsigned int idx_x_arg, unsigned int idx_y_arg, unsigned int idx_z_arg);

        /** \brief Return the amount of existing leafs in the octree.
         *  \return amount of registered leaf nodes.
         * */
        std::size_t
        getLeafCount () const
        {
          return leaf_count_;
        }

        /** \brief Return the amount of existing branch nodes in the octree.
         *  \return amount of branch nodes.
         * */
        std::size_t
        getBranchCount () const
        {
          return branch_count_;
        }

        /** \brief Delete the octree structure and its leaf nodes.
         * */
        void
        deleteTree ( );

        /** \brief Serialize octree into a binary output vector describing its branch node structure.
         *  \param binary_tree_out_arg: reference to output vector for writing binary tree structure.
         * */
        void
        serializeTree (std::vector<char>& binary_tree_out_arg);

        /** \brief Serialize octree into a binary output vector describing its branch node structure and push all LeafContainerT elements stored in the octree to a vector.
         * \param binary_tree_out_arg: reference to output vector for writing binary tree structure.
         * \param leaf_container_vector_arg: pointer to all LeafContainerT objects in the octree
         * */
        void
        serializeTree (std::vector<char>& binary_tree_out_arg, std::vector<LeafContainerT*>& leaf_container_vector_arg);

        /** \brief Outputs a vector of all LeafContainerT elements that are stored within the octree leaf nodes.
         *  \param leaf_container_vector_arg: pointers to LeafContainerT vector that receives a copy of all LeafContainerT objects in the octree.
         * */
        void
        serializeLeafs (std::vector<LeafContainerT*>& leaf_container_vector_arg);

        /** \brief Deserialize a binary octree description vector and create a corresponding octree structure. Leaf nodes are initialized with getDataTByKey(..).
         *  \param binary_tree_input_arg: reference to input vector for reading binary tree structure.
         * */
        void
        deserializeTree (std::vector<char>& binary_tree_input_arg);

        /** \brief Deserialize a binary octree description and create a corresponding octree structure. Leaf nodes are initialized with LeafContainerT elements from the dataVector.
         *  \param binary_tree_input_arg: reference to input vector for reading binary tree structure.
         *  \param leaf_container_vector_arg: pointer to container vector.
         * */
        void
        deserializeTree (std::vector<char>& binary_tree_input_arg, std::vector<LeafContainerT*>& leaf_container_vector_arg);

      protected:
        
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Protected octree methods based on octree keys
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        /** \brief Create a leaf node
         *  \param key_arg: octree key addressing a leaf node.
         *  \return pointer to leaf node
         * */
        LeafContainerT*
        createLeaf (const OctreeKey& key_arg)
        {

          LeafNode* leaf_node;
          BranchNode* leaf_node_parent;

          createLeafRecursive (key_arg, depth_mask_ ,root_node_, leaf_node, leaf_node_parent);

          LeafContainerT* ret = leaf_node->getContainerPtr();

          return ret;
        }

        /** \brief Find leaf node
         *  \param key_arg: octree key addressing a leaf node.
         *  \return pointer to leaf node. If leaf node is not found, this pointer returns 0.
         * */
        LeafContainerT*
        findLeaf (const OctreeKey& key_arg) const
        {
          LeafContainerT* result = 0;
          findLeafRecursive (key_arg, depth_mask_, root_node_, result);
          return result;
        }

        /** \brief Check for existance of a leaf node in the octree
         *  \param key_arg: octree key addressing a leaf node.
         *  \return "true" if leaf node is found; "false" otherwise
         * */
        bool
        existLeaf (const OctreeKey& key_arg) const
        {
          return (findLeaf(key_arg) != 0);
        }

        /** \brief Remove leaf node from octree
         *  \param key_arg: octree key addressing a leaf node.
         * */
        void
        removeLeaf (const OctreeKey& key_arg)
        {
          if (key_arg <= max_key_)
            deleteLeafRecursive (key_arg, depth_mask_, root_node_);
        }

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Branch node access functions
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        /** \brief Retrieve root node */
        OctreeNode*
        getRootNode () const
        {
          return this->root_node_;
        }

        /** \brief Check if branch is pointing to a particular child node
         *  \param branch_arg: reference to octree branch class
         *  \param child_idx_arg: index to child node
         *  \return "true" if pointer to child node exists; "false" otherwise
         * */
        bool
        branchHasChild (const BranchNode& branch_arg,
                        unsigned char child_idx_arg) const
        {
          // test occupancyByte for child existence
          return (branch_arg.getChildPtr(child_idx_arg) != 0);
        }

        /** \brief Retrieve a child node pointer for child node at child_idx.
         * \param branch_arg: reference to octree branch class
         * \param child_idx_arg: index to child node
         * \return pointer to octree child node class
         */
        OctreeNode*
        getBranchChildPtr (const BranchNode& branch_arg,
                           unsigned char child_idx_arg) const
        {
          return branch_arg.getChildPtr(child_idx_arg);
        }

        /** \brief Assign new child node to branch
         *  \param branch_arg: reference to octree branch class
         *  \param child_idx_arg: index to child node
         *  \param new_child_arg: pointer to new child node
         * */
        void setBranchChildPtr (BranchNode& branch_arg,
                                unsigned char child_idx_arg,
                                OctreeNode* new_child_arg)
        {
          branch_arg[child_idx_arg] = new_child_arg;
        }

        /** \brief Generate bit pattern reflecting the existence of child node pointers
         *  \param branch_arg: reference to octree branch class
         *  \return a single byte with 8 bits of child node information
         * */
        char
        getBranchBitPattern (const BranchNode& branch_arg) const
        {
          unsigned char i;
          char node_bits;

          // create bit pattern
          node_bits = 0;
          for (i = 0; i < 8; i++) {
            const OctreeNode* child = branch_arg.getChildPtr(i);
            node_bits |= static_cast<char> ((!!child) << i);
          }

          return (node_bits);
        }

        /** \brief Delete child node and all its subchilds from octree
         *  \param branch_arg: reference to octree branch class
         *  \param child_idx_arg: index to child node
         * */
        void
        deleteBranchChild (BranchNode& branch_arg, unsigned char child_idx_arg)
        {
          if (branch_arg.hasChild(child_idx_arg))
          {
            OctreeNode* branch_child = branch_arg[child_idx_arg];
            
            switch (branch_child->getNodeType ())
            {
              case BRANCH_NODE:
              {
                // free child branch recursively
                deleteBranch (*static_cast<BranchNode*> (branch_child));
                // delete branch node
                delete branch_child;
              }
                break;

              case LEAF_NODE:
              {
                // delete leaf node
                delete branch_child;
                break;
              }
              default:
                break;
            }

            // set branch child pointer to 0
            branch_arg[child_idx_arg] = 0;
          }
        }

        /** \brief Delete branch and all its subchilds from octree
         *  \param branch_arg: reference to octree branch class
         * */
        void
        deleteBranch (BranchNode& branch_arg)
        {
          char i;

          // delete all branch node children
          for (i = 0; i < 8; i++)
            deleteBranchChild (branch_arg, i);
        }

        /** \brief Create and add a new branch child to a branch class
         *  \param branch_arg: reference to octree branch class
         *  \param child_idx_arg: index to child node
         *  \return pointer of new branch child to this reference
         * */
        BranchNode* createBranchChild (BranchNode& branch_arg,
                                       unsigned char child_idx_arg)
        {
          BranchNode* new_branch_child = new BranchNode();
          branch_arg[child_idx_arg] = static_cast<OctreeNode*> (new_branch_child);

          return new_branch_child;
        }

        /** \brief Create and add a new leaf child to a branch class
         *  \param branch_arg: reference to octree branch class
         *  \param child_idx_arg: index to child node
         *  \return pointer of new leaf child to this reference
         * */
        LeafNode*
        createLeafChild (BranchNode& branch_arg, unsigned char child_idx_arg)
        {
          LeafNode* new_leaf_child = new LeafNode();
          branch_arg[child_idx_arg] = static_cast<OctreeNode*> (new_leaf_child);

          return new_leaf_child;
        }

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Recursive octree methods
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        /** \brief Create a leaf node at octree key. If leaf node does already exist, it is returned.
         *  \param key_arg: reference to an octree key
         *  \param depth_mask_arg: depth mask used for octree key analysis and for branch depth indicator
         *  \param branch_arg: current branch node
         *  \param return_leaf_arg: return pointer to leaf node
         *  \param parent_of_leaf_arg: return pointer to parent of leaf node
         *  \return depth mask at which leaf node was created
         **/
        unsigned int
        createLeafRecursive (const OctreeKey& key_arg,
                             unsigned int depth_mask_arg,
                             BranchNode* branch_arg,
                             LeafNode*& return_leaf_arg,
                             BranchNode*& parent_of_leaf_arg);

        /** \brief Recursively search for a given leaf node and return a pointer.
         *  \note  If leaf node does not exist, a 0 pointer is returned.
         *  \param key_arg: reference to an octree key
         *  \param depth_mask_arg: depth mask used for octree key analysis and for branch depth indicator
         *  \param branch_arg: current branch node
         *  \param result_arg: pointer to leaf node class
         **/
        void
        findLeafRecursive (const OctreeKey& key_arg,
                           unsigned int depth_mask_arg,
                           BranchNode* branch_arg,
                           LeafContainerT*& result_arg) const;

        /** \brief Recursively search and delete leaf node
         *  \param key_arg: reference to an octree key
         *  \param depth_mask_arg: depth mask used for octree key analysis and branch depth indicator
         *  \param branch_arg: current branch node
         *  \return "true" if branch does not contain any childs; "false" otherwise. This indicates if current branch can be deleted, too.
         **/
        bool
        deleteLeafRecursive (const OctreeKey& key_arg,
                             unsigned int depth_mask_arg,
                             BranchNode* branch_arg);

        /** \brief Recursively explore the octree and output binary octree description together with a vector of leaf node LeafContainerTs.
         *  \param branch_arg: current branch node
         *  \param key_arg: reference to an octree key
         *  \param binary_tree_out_arg: binary output vector
         *  \param leaf_container_vector_arg: writes LeafContainerT pointers to this LeafContainerT* vector.
         **/
        void
        serializeTreeRecursive (const BranchNode* branch_arg,
                                OctreeKey& key_arg,
                                std::vector<char>* binary_tree_out_arg,
                                typename std::vector<LeafContainerT*>* leaf_container_vector_arg) const;

         /** \brief Recursive method for deserializing octree structure
          *  \param branch_arg: current branch node
          *  \param depth_mask_arg: depth mask used for octree key analysis and branch depth indicator
          *  \param key_arg: reference to an octree key
          *  \param binary_tree_input_it_arg: iterator to binary input vector
          *  \param binary_tree_input_it_end_arg: end iterator of binary input vector
          *  \param leaf_container_vector_it_arg: iterator pointing to current LeafContainerT object to be added to a leaf node
          *  \param leaf_container_vector_it_end_arg: iterator pointing to last object in LeafContainerT input vector.
         **/
        void
        deserializeTreeRecursive (BranchNode* branch_arg, unsigned int depth_mask_arg, OctreeKey& key_arg,
                                  typename std::vector<char>::const_iterator& binary_tree_input_it_arg,
                                  typename std::vector<char>::const_iterator& binary_tree_input_it_end_arg,
                                  typename std::vector<LeafContainerT*>::const_iterator* leaf_container_vector_it_arg,
                                  typename std::vector<LeafContainerT*>::const_iterator* leaf_container_vector_it_end_arg);


         //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Serialization callbacks
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        /** \brief Callback executed for every leaf node during serialization
         **/
        virtual void
        serializeTreeCallback (LeafContainerT&, const OctreeKey &) const
        {

        }

        /** \brief Callback executed for every leaf node during deserialization
         **/
        virtual void
        deserializeTreeCallback (LeafContainerT&, const OctreeKey&)
        {

        }

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Helpers
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        /** \brief Helper function to calculate the binary logarithm
         * \param n_arg: some value
         * \return binary logarithm (log2) of argument n_arg
         */
        double
        Log2 (double n_arg)
        {
          return log( n_arg ) / log( 2.0 );
        }

        /** \brief Test if octree is able to dynamically change its depth. This is required for adaptive bounding box adjustment.
         *  \return "true"
         **/
        bool
        octreeCanResize ()
        {
          return (true);
        }

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Globals
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        /** \brief Amount of leaf nodes   **/
        std::size_t leaf_count_;

        /** \brief Amount of branch nodes   **/
        std::size_t branch_count_;

        /** \brief Pointer to root branch node of octree   **/
        BranchNode* root_node_;

        /** \brief Depth mask based on octree depth   **/
        unsigned int depth_mask_;

        /** \brief Octree depth */
        unsigned int octree_depth_;

        /** \brief Enable dynamic_depth **/
        bool dynamic_depth_enabled_;

        /** \brief key range */
        OctreeKey max_key_;
    };
  }
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/octree/impl/octree_base.hpp>
#endif

#endif

