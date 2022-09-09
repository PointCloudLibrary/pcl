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
 * orr_octree.h
 *
 *  Created on: Oct 23, 2012
 *      Author: papazov
 */

#pragma once

#include "auxiliary.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/pcl_exports.h>
#include <vector>
#include <list>
#include <set>

//#define PCL_REC_ORR_OCTREE_VERBOSE

namespace pcl
{
  namespace recognition
  {
    /** \brief That's a very specialized and simple octree class. That's the way it is intended to
      * be, that's why no templates and stuff like this.
      *
      * \author Chavdar Papazov
      * \ingroup recognition
      */
    class PCL_EXPORTS ORROctree
    {
      public:
        using PointCloudIn = pcl::PointCloud<pcl::PointXYZ>;
        using PointCloudOut = pcl::PointCloud<pcl::PointXYZ>;
        using PointCloudN = pcl::PointCloud<pcl::Normal>;

        class Node
        {
          public:
            class Data
            {
              public:
                Data (int id_x, int id_y, int id_z, int lin_id, void* user_data = nullptr)
                : id_x_ (id_x),
                  id_y_ (id_y),
                  id_z_ (id_z),
                  lin_id_ (lin_id),
                  num_points_ (0),
                  user_data_ (user_data)
                {
                  n_[0] = n_[1] = n_[2] = p_[0] = p_[1] = p_[2] = 0.0f;
                }

                virtual~ Data () = default;

                inline void
                addToPoint (float x, float y, float z)
                {
                  p_[0] += x; p_[1] += y; p_[2] += z;
                  ++num_points_;
                }

                inline void
                computeAveragePoint ()
                {
                  if ( num_points_ < 2 )
                    return;

                  aux::mult3 (p_, 1.0f/static_cast<float> (num_points_));
                  num_points_ = 1;
                }

                inline void
                addToNormal (float x, float y, float z) { n_[0] += x; n_[1] += y; n_[2] += z;}

                inline const float*
                getPoint () const { return p_;}

                inline float*
                getPoint (){ return p_;}

                inline const float*
                getNormal () const { return n_;}

                inline float*
                getNormal (){ return n_;}

                inline void
                get3dId (int id[3]) const
                {
                  id[0] = id_x_;
                  id[1] = id_y_;
                  id[2] = id_z_;
                }

                inline int
                get3dIdX () const {return id_x_;}

                inline int
                get3dIdY () const {return id_y_;}

                inline int
                get3dIdZ () const {return id_z_;}

                inline int
                getLinearId () const { return lin_id_;}

                inline void
                setUserData (void* user_data){ user_data_ = user_data;}

                inline void*
                getUserData () const { return user_data_;}

                inline void
                insertNeighbor (Node* node){ neighbors_.insert (node);}

                inline const std::set<Node*>&
                getNeighbors () const { return (neighbors_);}

              protected:
                float n_[3], p_[3];
                int id_x_, id_y_, id_z_, lin_id_, num_points_;
                std::set<Node*> neighbors_;
                void *user_data_;
            };

            Node ()
            : data_ (nullptr),
              parent_ (nullptr),
              children_(nullptr)
            {}

            virtual~ Node ()
            {
              this->deleteChildren ();
              this->deleteData ();
            }

            inline void
            setCenter(const float *c) { center_[0] = c[0]; center_[1] = c[1]; center_[2] = c[2];}

            inline void
            setBounds(const float *b) { bounds_[0] = b[0]; bounds_[1] = b[1]; bounds_[2] = b[2]; bounds_[3] = b[3]; bounds_[4] = b[4]; bounds_[5] = b[5];}

            inline void
            setParent(Node* parent) { parent_ = parent;}

            inline void
            setData(Node::Data* data) { data_ = data;}

            /** \brief Computes the "radius" of the node which is half the diagonal length. */
            inline void
            computeRadius()
            {
              float v[3] = {0.5f*(bounds_[1]-bounds_[0]), 0.5f*(bounds_[3]-bounds_[2]), 0.5f*(bounds_[5]-bounds_[4])};
              radius_ = static_cast<float> (aux::length3 (v));
            }

            inline const float*
            getCenter() const { return center_;}

            inline const float*
            getBounds() const { return bounds_;}

            inline void
            getBounds(float b[6]) const
            {
              std::copy(bounds_, bounds_ + 6, b);
            }

            inline Node*
            getChild (int id) { return &children_[id];}

            inline Node*
            getChildren () { return children_;}

            inline Node::Data*
            getData (){ return data_;}

            inline const Node::Data*
            getData () const { return data_;}

            inline void
            setUserData (void* user_data){ data_->setUserData (user_data);}

            inline Node*
            getParent (){ return parent_;}

            inline bool
            hasData (){ return static_cast<bool> (data_);}

            inline bool
            hasChildren (){ return static_cast<bool> (children_);}

            /** \brief Computes the "radius" of the node which is half the diagonal length. */
            inline float
            getRadius () const{ return radius_;}

            bool
            createChildren ();

            inline void
            deleteChildren ()
              {
                delete[] children_;
                children_ = nullptr;
              }

            inline void
            deleteData ()
              {
                delete data_;
                data_ = nullptr;
              }

            /** \brief Make this and 'node' neighbors by inserting each node in the others node neighbor set. Nothing happens
              * of either of the nodes has no data. */
            inline void
            makeNeighbors (Node* node)
            {
              if ( !this->getData () || !node->getData () )
                return;

              this->getData ()->insertNeighbor (node);
              node->getData ()->insertNeighbor (this);
            }

          protected:
            Node::Data *data_;
            float center_[3], bounds_[6], radius_;
            Node *parent_, *children_;
        };

        ORROctree ();
        virtual ~ORROctree (){ this->clear ();}

        void
        clear ();

        /** \brief Creates an octree which encloses 'points' and with leaf size equal to 'voxel_size'.
          * 'enlarge_bounds' makes sure that no points from the input will lie on the octree boundary
          * by enlarging the bounds by that factor. For example, enlarge_bounds = 1 means that the
          * bounds will be enlarged by 100%. The default value is fine. */
        void
        build (const PointCloudIn& points, float voxel_size, const PointCloudN* normals = nullptr, float enlarge_bounds = 0.00001f);

        /** \brief Creates an empty octree with bounds at least as large as the ones provided as input and with leaf
          * size equal to 'voxel_size'. */
        void
        build (const float* bounds, float voxel_size);

        /** \brief Creates the leaf containing p = (x, y, z) and returns a pointer to it, however, only if p lies within
          * the octree bounds! A more general version which allows p to be out of bounds is not implemented yet. The method
          * returns NULL if p is not within the root bounds. If the leaf containing p already exists nothing happens and
          * method just returns a pointer to the leaf. */
        inline ORROctree::Node*
        createLeaf (float x, float y, float z)
        {
          // Make sure that the input point is within the octree bounds
          if ( x < bounds_[0] || x > bounds_[1] ||
               y < bounds_[2] || y > bounds_[3] ||
               z < bounds_[4] || z > bounds_[5] )
          {
            return (nullptr);
          }

          ORROctree::Node* node = root_;
          const float *c;
          int id;

          // Go down to the right leaf
          for ( int l = 0 ; l < tree_levels_ ; ++l )
          {
            node->createChildren ();
            c = node->getCenter ();
            id = 0;

            if ( x >= c[0] ) id |= 4;
            if ( y >= c[1] ) id |= 2;
            if ( z >= c[2] ) id |= 1;

            node = node->getChild (id);
          }

          if ( !node->getData () )
          {
            auto* data = new Node::Data (
                static_cast<int> ((node->getCenter ()[0] - bounds_[0])/voxel_size_),
                static_cast<int> ((node->getCenter ()[1] - bounds_[2])/voxel_size_),
                static_cast<int> ((node->getCenter ()[2] - bounds_[4])/voxel_size_),
                static_cast<int> (full_leaves_.size ()));

            node->setData (data);
            this->insertNeighbors (node);
            full_leaves_.push_back (node);
          }

          return (node);
        }

    	/** \brief This method returns a super set of the full leavess which are intersected by the sphere
    	  * with radius 'radius' and centered at 'p'. Pointers to the intersected full leaves are saved in
    	  * 'out'. The method computes a super set in the sense that in general not all leaves saved in 'out'
    	  * are really intersected by the sphere. The intersection test is based on the leaf radius (since
    	  * its faster than checking all leaf corners and sides), so we report more leaves than we should,
    	  * but still, this is a fair approximation. */
        void
        getFullLeavesIntersectedBySphere (const float* p, float radius, std::list<ORROctree::Node*>& out) const;

        /** \brief Randomly chooses and returns a full leaf that is intersected by the sphere with center 'p'
          * and 'radius'. Returns NULL if no leaf is intersected by that sphere. */
        ORROctree::Node*
        getRandomFullLeafOnSphere (const float* p, float radius) const;

        /** \brief Since the leaves are aligned in a rectilinear grid, each leaf has a unique id. The method returns the leaf
          * with id [i, j, k] or NULL is no such leaf exists. */
        ORROctree::Node*
        getLeaf (int i, int j, int k)
        {
          float offset = 0.5f*voxel_size_;
          float p[3] = {bounds_[0] + offset + static_cast<float> (i)*voxel_size_,
                        bounds_[2] + offset + static_cast<float> (j)*voxel_size_,
                        bounds_[4] + offset + static_cast<float> (k)*voxel_size_};

          return (this->getLeaf (p[0], p[1], p[2]));
        }

        /** \brief Returns a pointer to the leaf containing p = (x, y, z) or NULL if no such leaf exists. */
        inline ORROctree::Node*
        getLeaf (float x, float y, float z)
        {
          // Make sure that the input point is within the octree bounds
          if ( x < bounds_[0] || x > bounds_[1] ||
               y < bounds_[2] || y > bounds_[3] ||
               z < bounds_[4] || z > bounds_[5] )
          {
            return (nullptr);
          }

          ORROctree::Node* node = root_;
          const float *c;
          int id;

          // Go down to the right leaf
          for ( int l = 0 ; l < tree_levels_ ; ++l )
          {
            if ( !node->hasChildren () )
              return (nullptr);

            c = node->getCenter ();
            id = 0;

            if ( x >= c[0] ) id |= 4;
            if ( y >= c[1] ) id |= 2;
            if ( z >= c[2] ) id |= 1;

            node = node->getChild (id);
          }

          return (node);
        }

        /** \brief Deletes the branch 'node' is part of. */
        void
        deleteBranch (Node* node);

        /** \brief Returns a vector with all octree leaves which contain at least one point. */
        inline std::vector<ORROctree::Node*>&
        getFullLeaves () { return full_leaves_;}

        inline const std::vector<ORROctree::Node*>&
        getFullLeaves () const { return full_leaves_;}

        void
        getFullLeavesPoints (PointCloudOut& out) const;

        void
        getNormalsOfFullLeaves (PointCloudN& out) const;

        inline ORROctree::Node*
        getRoot (){ return root_;}

        inline const float*
        getBounds () const
        {
          return (bounds_);
        }

        inline void
        getBounds (float b[6]) const
        {
          std::copy(bounds_, bounds_ + 6, b);
        }

        inline float
        getVoxelSize () const { return voxel_size_;}

        inline void
        insertNeighbors (Node* node)
        {
          const float* c = node->getCenter ();
          float s = 0.5f*voxel_size_;
          Node *neigh;

          neigh = this->getLeaf (c[0]+s, c[1]+s, c[2]+s); if ( neigh ) node->makeNeighbors (neigh);
          neigh = this->getLeaf (c[0]+s, c[1]+s, c[2]  ); if ( neigh ) node->makeNeighbors (neigh);
          neigh = this->getLeaf (c[0]+s, c[1]+s, c[2]-s); if ( neigh ) node->makeNeighbors (neigh);
          neigh = this->getLeaf (c[0]+s, c[1]  , c[2]+s); if ( neigh ) node->makeNeighbors (neigh);
          neigh = this->getLeaf (c[0]+s, c[1]  , c[2]  ); if ( neigh ) node->makeNeighbors (neigh);
          neigh = this->getLeaf (c[0]+s, c[1]  , c[2]-s); if ( neigh ) node->makeNeighbors (neigh);
          neigh = this->getLeaf (c[0]+s, c[1]-s, c[2]+s); if ( neigh ) node->makeNeighbors (neigh);
          neigh = this->getLeaf (c[0]+s, c[1]-s, c[2]  ); if ( neigh ) node->makeNeighbors (neigh);
          neigh = this->getLeaf (c[0]+s, c[1]-s, c[2]-s); if ( neigh ) node->makeNeighbors (neigh);

          neigh = this->getLeaf (c[0]  , c[1]+s, c[2]+s); if ( neigh ) node->makeNeighbors (neigh);
          neigh = this->getLeaf (c[0]  , c[1]+s, c[2]  ); if ( neigh ) node->makeNeighbors (neigh);
          neigh = this->getLeaf (c[0]  , c[1]+s, c[2]-s); if ( neigh ) node->makeNeighbors (neigh);
          neigh = this->getLeaf (c[0]  , c[1]  , c[2]+s); if ( neigh ) node->makeNeighbors (neigh);
        //neigh = this->getLeaf (c[0]  , c[1]  , c[2]  ); if ( neigh ) node->makeNeighbors (neigh);
          neigh = this->getLeaf (c[0]  , c[1]  , c[2]-s); if ( neigh ) node->makeNeighbors (neigh);
          neigh = this->getLeaf (c[0]  , c[1]-s, c[2]+s); if ( neigh ) node->makeNeighbors (neigh);
          neigh = this->getLeaf (c[0]  , c[1]-s, c[2]  ); if ( neigh ) node->makeNeighbors (neigh);
          neigh = this->getLeaf (c[0]  , c[1]-s, c[2]-s); if ( neigh ) node->makeNeighbors (neigh);

          neigh = this->getLeaf (c[0]-s, c[1]+s, c[2]+s); if ( neigh ) node->makeNeighbors (neigh);
          neigh = this->getLeaf (c[0]-s, c[1]+s, c[2]  ); if ( neigh ) node->makeNeighbors (neigh);
          neigh = this->getLeaf (c[0]-s, c[1]+s, c[2]-s); if ( neigh ) node->makeNeighbors (neigh);
          neigh = this->getLeaf (c[0]-s, c[1]  , c[2]+s); if ( neigh ) node->makeNeighbors (neigh);
          neigh = this->getLeaf (c[0]-s, c[1]  , c[2]  ); if ( neigh ) node->makeNeighbors (neigh);
          neigh = this->getLeaf (c[0]-s, c[1]  , c[2]-s); if ( neigh ) node->makeNeighbors (neigh);
          neigh = this->getLeaf (c[0]-s, c[1]-s, c[2]+s); if ( neigh ) node->makeNeighbors (neigh);
          neigh = this->getLeaf (c[0]-s, c[1]-s, c[2]  ); if ( neigh ) node->makeNeighbors (neigh);
          neigh = this->getLeaf (c[0]-s, c[1]-s, c[2]-s); if ( neigh ) node->makeNeighbors (neigh);
        }

      protected:
        float voxel_size_, bounds_[6];
        int tree_levels_;
        Node* root_;
        std::vector<Node*> full_leaves_;
    };
  } // namespace recognition
} // namespace pcl
