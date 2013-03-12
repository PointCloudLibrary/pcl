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
 *
 */

/*
 * simple_octree.h
 *
 *  Created on: Mar 11, 2013
 *      Author: papazov
 */

#ifndef SIMPLE_OCTREE_H_
#define SIMPLE_OCTREE_H_

#include <pcl/pcl_exports.h>
#include <set>

namespace pcl
{
  namespace recognition
  {
    template<typename NodeData, typename NodeDataCreator, typename Scalar = float>
    class PCL_EXPORTS SimpleOctree
    {
      public:
        class Node
        {
          public:
            Node ();

            virtual~ Node ();

            inline void
            setCenter (const Scalar *c);

            inline void
            setBounds (const Scalar *b);

            inline const Scalar*
            getCenter () const { return center_;}

            inline const Scalar*
            getBounds () const { return bounds_;}

            inline void
            getBounds (Scalar b[6]) const { memcpy (b, bounds_, 6*sizeof (Scalar));}

            inline Node*
            getChild (int id) { return &children_[id];}

            inline Node*
            getChildren () { return children_;}

            inline void
            setData (const NodeData& src){ *data_ = src;}

            inline NodeData&
            getData (){ return *data_;}

            inline const NodeData&
            getData () const { return *data_;}

            inline Node*
            getParent (){ return parent_;}

            inline float
            getRadius () const { return radius_;}

            inline bool
            hasData (){ return static_cast<bool> (data_);}

            inline bool
            hasChildren (){ return static_cast<bool> (children_);}

            inline const std::set<Node*>&
            getNeighbors () const { return (full_leaf_neighbors_);}

            inline void
            deleteChildren ();

            inline void
            deleteData ();

            friend class SimpleOctree;

          protected:
            void
            setData (NodeData* data){ if ( data_ ) delete data_; data_ = data;}

            inline bool
            createChildren ();

            /** \brief Make this and 'node' neighbors by inserting each node in the others node neighbor set. Nothing happens
              * of either of the nodes has no data. */
            inline void
            makeNeighbors (Node* node);

            inline void
            setParent (Node* parent){ parent_ = parent;}

            /** \brief Computes the "radius" of the node which is half the diagonal length. */
            inline void
            computeRadius ();

          protected:
            NodeData *data_;
            Scalar center_[3], bounds_[6];
            Node *parent_, *children_;
            Scalar radius_;
            std::set<Node*> full_leaf_neighbors_;
        };

      public:
        SimpleOctree ();

        virtual ~SimpleOctree ();

        void
        clear ();

        /** \brief Creates an empty octree with bounds at least as large as the ones provided as input and with leaf
          * size equal to 'voxel_size'. */
        void
        build (const Scalar* bounds, Scalar voxel_size, NodeDataCreator* node_data_creator);

        /** \brief Creates the leaf containing p = (x, y, z) and returns a pointer to it, however, only if p lies within
          * the octree bounds! A more general version which allows p to be out of bounds is not implemented yet. The method
          * returns NULL if p is not within the root bounds. If the leaf containing p already exists nothing happens and
          * method just returns a pointer to the leaf. Note that for a new created leaf, the method also creates its data
          * object. */
        inline Node*
        createLeaf (Scalar x, Scalar y, Scalar z);

        /** \brief Since the leaves are aligned in a rectilinear grid, each leaf has a unique id. The method returns the full
          * leaf, i.e., the one having a data object, with id [i, j, k] or NULL is no such leaf exists. */
        inline Node*
        getFullLeaf (int i, int j, int k);

        /** \brief Returns a pointer to the full leaf, i.e., one having a data pbject, containing p = (x, y, z) or NULL if no such leaf exists. */
        inline Node*
        getFullLeaf (Scalar x, Scalar y, Scalar z);

        inline std::vector<Node*>&
        getFullLeaves () { return full_leaves_;}

        inline const std::vector<Node*>&
        getFullLeaves () const { return full_leaves_;}

        inline Node*
        getRoot (){ return root_;}

        inline const Scalar*
        getBounds () const { return (bounds_);}

        inline void
        getBounds (Scalar b[6]) const { memcpy (b, bounds_, 6*sizeof (Scalar));}

        inline Scalar
        getVoxelSize () const { return voxel_size_;}

      protected:
        inline void
        insertNeighbors (Node* node);

      protected:
        Scalar voxel_size_, bounds_[6];
        int tree_levels_;
        Node* root_;
        std::vector<Node*> full_leaves_;
        NodeDataCreator* node_data_creator_;
    };
  } // namespace recognition
} // namespace pcl

#include <pcl/recognition/impl/ransac_based/simple_octree.hpp>

#endif /* SIMPLE_OCTREE_H_ */
