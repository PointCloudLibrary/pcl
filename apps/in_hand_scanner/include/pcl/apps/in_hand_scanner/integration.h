/*
 * Software License Agreement (BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2009-2012, Willow Garage, Inc.
 * Copyright (c) 2012-, Open Perception, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of the copyright holder(s) nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

#ifndef PCL_IN_HAND_SCANNER_INTEGRATION_H
#define PCL_IN_HAND_SCANNER_INTEGRATION_H

#include <pcl/apps/in_hand_scanner/common_types.h>

////////////////////////////////////////////////////////////////////////////////
// Forward declarations
////////////////////////////////////////////////////////////////////////////////

namespace pcl
{
  template <typename PointT>
  class KdTree;
} // End namespace pcl

////////////////////////////////////////////////////////////////////////////////
// Integration
////////////////////////////////////////////////////////////////////////////////

namespace pcl
{
  namespace ihs
  {

    class Integration
    {

      public:

        typedef pcl::PointXYZ              PointXYZ;
        typedef pcl::PointCloud <PointXYZ> CloudXYZ;
        typedef CloudXYZ::Ptr              CloudXYZPtr;
        typedef CloudXYZ::ConstPtr         CloudXYZConstPtr;

        typedef pcl::ihs::PointProcessed         PointProcessed;
        typedef pcl::ihs::CloudProcessed         CloudProcessed;
        typedef pcl::ihs::CloudProcessedPtr      CloudProcessedPtr;
        typedef pcl::ihs::CloudProcessedConstPtr CloudProcessedConstPtr;

        typedef pcl::ihs::PointModel         PointModel;
        typedef pcl::ihs::CloudModel         CloudModel;
        typedef pcl::ihs::CloudModelPtr      CloudModelPtr;
        typedef pcl::ihs::CloudModelConstPtr CloudModelConstPtr;

        typedef pcl::ihs::Mesh            Mesh;
        typedef pcl::ihs::MeshPtr         MeshPtr;
        typedef pcl::ihs::MeshConstPtr    MeshConstPtr;
        typedef Mesh::Vertex              Vertex;
        typedef Mesh::VertexIndex         VertexIndex;
        typedef Mesh::VertexIndexes       VertexIndexes;
        typedef Mesh::VertexConstIterator VertexConstIterator;

        typedef pcl::ihs::Transformation Transformation;

        typedef pcl::KdTree <PointXYZ>           KdTree;
        typedef boost::shared_ptr <KdTree>       KdTreePtr;
        typedef boost::shared_ptr <const KdTree> KdTreeConstPtr;

      public:

        Integration ();

        bool
        reconstructMesh (const CloudProcessedConstPtr& cloud_data,
                         const MeshPtr&                mesh_model) const;

        bool
        merge (const CloudProcessedConstPtr& cloud_data,
               const MeshPtr&                mesh_model,
               const Transformation&         T) const;

        void
        age (const MeshPtr& mesh, const bool cleanup=true) const;

      private:

        uint8_t
        trimRGB (const float val) const;

        // 2 - 1
        // | / |
        // 3 - 0
        void
        addToMesh (const CloudModel::const_iterator& it_pt_0,
                   const CloudModel::const_iterator& it_pt_1,
                   const CloudModel::const_iterator& it_pt_2,
                   const CloudModel::const_iterator& it_pt_3,
                   const VertexIndexes::iterator&    it_vi_0,
                   const VertexIndexes::iterator&    it_vi_1,
                   const VertexIndexes::iterator&    it_vi_2,
                   const VertexIndexes::iterator&    it_vi_3,
                   const MeshPtr&                    mesh) const;

        void
        addToMesh (const CloudModel::const_iterator& it_pt_0,
                   const CloudModel::const_iterator& it_pt_1,
                   const CloudModel::const_iterator& it_pt_2,
                   const VertexIndexes::iterator&    it_vi_0,
                   const VertexIndexes::iterator&    it_vi_1,
                   const VertexIndexes::iterator&    it_vi_2,
                   const MeshPtr&                    mesh) const;

        bool
        distanceThreshold (const PointModel& pt_0,
                           const PointModel& pt_1,
                           const PointModel& pt_2) const;

        bool
        distanceThreshold (const PointModel& pt_0,
                           const PointModel& pt_1,
                           const PointModel& pt_2,
                           const PointModel& pt_3) const;

      private:

        // Nearest neighbor search
        KdTreePtr kd_tree_;

        // Maximum squared distance below which points are averaged out
        float squared_distance_max_;

        // Minium dot product between normals above which points are averaged out
        float dot_normal_min_;

        // Minimum weight above which points are added.
        float weight_min_;

        // A point dies if it has not been updated in the last age_max_ frames and the visibility confidence is below visconf_min
        unsigned int age_max_;
        float        visconf_min_; // [0 1]
    };

  } // End namespace ihs
} // End namespace pcl

#endif // PCL_IN_HAND_SCANNER_INTEGRATION_H
