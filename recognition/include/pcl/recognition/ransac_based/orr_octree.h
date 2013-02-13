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

#ifndef PCL_RECOGNITION_ORR_OCTREE_H_
#define PCL_RECOGNITION_ORR_OCTREE_H_

#include "auxiliary.h"
//#include <pcl/common/random.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/pcl_exports.h>
#include <cstdlib>
#include <ctime>
#include <vector>
#include <list>

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
        typedef pcl::PointCloud<pcl::PointXYZ> PointCloudIn;
        typedef pcl::PointCloud<pcl::PointXYZ> PointCloudOut;
        typedef pcl::PointCloud<pcl::Normal> PointCloudN;

        class Node
        {
          public:
            class Data
            {
              public:
                Data (void* user_data = NULL)
                : id_x_ (-1),
                  id_y_ (-1),
                  id_z_ (-1),
                  num_points_(0),
                  user_data_ (user_data)
                {
                  n_[0] = n_[1] = n_[2] = p_[0] = p_[1] = p_[2] = 0.0f;
                }

                virtual~ Data (){}

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
                set3dId (int x, int y, int z){ id_x_ = x; id_y_ = y; id_z_ = z;}

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

                inline void
                setUserData (void* user_data)
                {
                  user_data_ = user_data;
                }

                inline void*
                getUserData () const
                {
                  return user_data_;
                }

              protected:
                float n_[3], p_[3];
                int id_x_, id_y_, id_z_, num_points_;
                void *user_data_;
            };

            Node ()
            : data_ (NULL),
              parent_ (NULL),
              children_(NULL)
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
            getCenter() const { return  center_;}

            inline const float*
            getBounds() const { return bounds_;}

            inline Node*
            getChild (int id) { return &children_[id];}

            inline Node*
            getChildren () { return children_;}

            inline Node::Data*
            getData() { return data_;}

            inline Node*
            getParent (){ return parent_;}

            inline bool
            hasData (){ return static_cast<bool> (data_);}

            inline bool
            hasChildren (){ return static_cast<bool> (children_);}

            /** \brief Computes the "radius" of the node which is half the diagonal length. */
            inline float
            getRadius (){ return radius_;}

            bool
            createChildren ();

            inline void
            deleteChildren ()
            {
              if ( children_ )
              {
                delete[] children_;
                children_ = NULL;
              }
            }

            inline void
            deleteData ()
            {
              if ( data_ )
              {
                delete data_;
                data_ = NULL;
              }
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
        build (const PointCloudIn& points, float voxel_size, const PointCloudN* normals = NULL, float enlarge_bounds = 0.00001f);

        /** \brief Creates an empty octree with bounds at least as large as the ones provided as input and with leaf
          * size equal to 'voxel_size'. */
        void
        build (const float* bounds, float voxel_size);

        /** \brief Creates the leaf containing p = (x, y, z) and returns a pointer to it, however, only if p lies within
          * the octree bounds! A more general version which allows p to be out of bounds is not implemented yet. The method
          * returns NULL if p is not within the root bounds. If the leaf containing p already exists nothing happens and
          * method just returns a pointer to the leaf. If 'leaf_was_created' != NULL, the method sets this boolean to true
          * if a new leaf was created or to false if p ends up in an existing leaf. */
        inline Node*
        createLeaf (float x, float y, float z)
        {
          // Make sure that the input point is within the octree bounds
          if ( x < bounds_[0] || x > bounds_[1] ||
               y < bounds_[2] || y > bounds_[3] ||
               z < bounds_[4] || z > bounds_[5] )
          {
            return (NULL);
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
            Node::Data* data = new Node::Data ();
            // Compute the 3d integer id of the leaf
            // Compute the 3d integer id of the leaf
            data->set3dId(
              static_cast<int> ((node->getCenter ()[0] - bounds_[0])/voxel_size_),
              static_cast<int> ((node->getCenter ()[1] - bounds_[2])/voxel_size_),
              static_cast<int> ((node->getCenter ()[2] - bounds_[4])/voxel_size_));
            // Save the data
            node->setData (data);
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
        getRandomFullLeafOnSphere (const float* p, float radius);

        /** \brief Deletes the branch 'node' is part of. */
        void
        deleteBranch (Node* node);

        /** \brief Returns a vector with all octree leaves which contain at least one point. */
        inline std::vector<ORROctree::Node*>&
        getFullLeaves () { return full_leaves_;}

        inline const std::vector<ORROctree::Node*>&
        getFullLeaves () const { return full_leaves_;}

        void
        getFullLeafPoints (PointCloudOut& out) const;

        void
        getNormalsOfFullLeaves (PointCloudN& out) const;

        inline ORROctree::Node*
        getRoot (){ return root_;}

        inline const float*
        getBounds () const
        {
          return (bounds_);
        }

        inline float
        getVoxelSize () const { return voxel_size_;}

      protected:
        float voxel_size_, bounds_[6];
        int tree_levels_;
        Node* root_;
        std::vector<Node*> full_leaves_;
//        pcl::common::UniformGenerator<int> randgen_;
    };
  } // namespace recognition
} // namespace pcl

#endif /* PCL_RECOGNITION_ORR_OCTREE_H_ */
