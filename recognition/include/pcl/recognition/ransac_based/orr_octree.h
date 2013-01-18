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

#define PCL_REC_ORR_OCTREE_VERBOSE

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
                Data (){ n_[0] = n_[1] = n_[2] = p_[0] = p_[1] = p_[2] = 0.0f;}
                virtual~ Data (){}

                inline void
                addToPoint (float x, float y, float z) { p_[0] += x; p_[1] += y; p_[2] += z;}

                inline void
                addToNormal (float x, float y, float z) { n_[0] += x; n_[1] += y; n_[2] += z;}

                inline void
                addPointId (int id) { point_ids_.push_back(id);}

                inline size_t
                getNumberOfPoints () const { return point_ids_.size();}

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

                inline int
                get3dIdX () const {return id_x_;}

                inline int
                get3dIdY () const {return id_y_;}

                inline int
                get3dIdZ () const {return id_z_;}

              protected:
                float n_[3], p_[3];
                std::list<int> point_ids_;
                int id_x_, id_y_, id_z_;
            };

            Node (): data_ (NULL), children_(NULL) {}
            virtual~ Node () { this->deleteChildren (); this->deleteData ();}

            void setCenter(const float *c) { center_[0] = c[0]; center_[1] = c[1]; center_[2] = c[2];}
            void setBounds(const float *b) { bounds_[0] = b[0]; bounds_[1] = b[1]; bounds_[2] = b[2]; bounds_[3] = b[3]; bounds_[4] = b[4]; bounds_[5] = b[5];}
            void setParent(Node* parent) { parent_ = parent;}
            void setData(Node::Data* data) { data_ = data;}
            /** \brief Computes the "radius" of the node which is half the diagonal length. */
            inline void
            computeRadius()
            {
              float v[3] = {0.5f*(bounds_[1]-bounds_[0]), 0.5f*(bounds_[3]-bounds_[2]), 0.5f*(bounds_[5]-bounds_[4])};
              radius_ = static_cast<float> (aux::vecLength3 (v));
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

            inline void
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
            int child_nr_;
        };

        ORROctree ();
        virtual ~ORROctree (){ this->clear ();}

        void
        clear ();

        void
        build (const PointCloudIn& points, float voxelsize, const PointCloudN* normals = NULL);

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
        std::vector<ORROctree::Node*>&
        getFullLeaves () { return full_leaves_;}

        const std::vector<ORROctree::Node*>&
        getFullLeaves () const { return full_leaves_;}

        void
        getFullLeafPoints (PointCloudOut& out);

        void
        getNormalsOfFullLeaves (PointCloudN& out);

        ORROctree::Node*
        getRoot (){ return root_;}

        float
        getVoxelSize () const { return voxel_size_;}

      protected:
        float voxel_size_, bounds_[6];
        int tree_levels_;
        Node* root_;
        std::vector<Node*> full_leaves_;
//        pcl::common::UniformGenerator<int> randgen_;

        const PointCloudIn *points_;
    };
  } // namespace recognition
} // namespace pcl

#endif /* PCL_RECOGNITION_ORR_OCTREE_H_ */
