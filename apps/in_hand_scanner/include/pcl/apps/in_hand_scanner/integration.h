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

#ifndef PCL_APPS_IN_HAND_SCANNER_INTEGRATION_H
#define PCL_APPS_IN_HAND_SCANNER_INTEGRATION_H

#include <stdint.h>

#include <pcl/pcl_exports.h>
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
    class PCL_EXPORTS Integration
    {
      public:

        typedef pcl::PointXYZ              PointXYZ;
        typedef pcl::PointCloud <PointXYZ> CloudXYZ;
        typedef CloudXYZ::Ptr              CloudXYZPtr;
        typedef CloudXYZ::ConstPtr         CloudXYZConstPtr;

        typedef pcl::PointXYZRGBNormal              PointXYZRGBNormal;
        typedef pcl::PointCloud <PointXYZRGBNormal> CloudXYZRGBNormal;
        typedef CloudXYZRGBNormal::Ptr              CloudXYZRGBNormalPtr;
        typedef CloudXYZRGBNormal::ConstPtr         CloudXYZRGBNormalConstPtr;

        typedef pcl::ihs::PointIHS         PointIHS;
        typedef pcl::ihs::CloudIHS         CloudIHS;
        typedef pcl::ihs::CloudIHSPtr      CloudIHSPtr;
        typedef pcl::ihs::CloudIHSConstPtr CloudIHSConstPtr;

        typedef pcl::ihs::Mesh            Mesh;
        typedef pcl::ihs::MeshPtr         MeshPtr;
        typedef pcl::ihs::MeshConstPtr    MeshConstPtr;
        typedef Mesh::VertexIndex         VertexIndex;
        typedef Mesh::VertexIndices       VertexIndices;

        typedef pcl::KdTree <PointXYZ>           KdTree;
        typedef boost::shared_ptr <KdTree>       KdTreePtr;
        typedef boost::shared_ptr <const KdTree> KdTreeConstPtr;

        Integration ();

        bool
        reconstructMesh (const CloudXYZRGBNormalConstPtr& cloud_data,
                         MeshPtr&                         mesh_model) const;

        bool
        merge (const CloudXYZRGBNormalConstPtr& cloud_data,
               MeshPtr&                         mesh_model,
               const Eigen::Matrix4f&           T) const;

        void
        age (const MeshPtr& mesh, const bool cleanup=true) const;

        void  setSquaredDistanceThreshold (const float squared_distance_max);
        float getSquaredDistanceThreshold () const;

        void  setAngleThreshold (const float dot_normal_min);
        float getAngleThreshold () const;

        void  setMinimumWeight (const float weight_min);
        float getMinimumWeight () const;

        void         setMaximumAge (const unsigned int age_max);
        unsigned int getMaximumAge () const;

        void         setMinimumCount (const unsigned int count_min);
        unsigned int getMinimumCount () const;

      private:

        // - Frequency 3 Icosahedron where each vertex corresponds to a viewing direction
        // - First vertex aligned to z-axis
        // - Removed vertices with z < 0.3
        // -> 31 directions, fitting nicely into a 32 bit integer
        // -> Very oblique angles are not considered
        class Dome
        {
          public:

            static const int                                NumDirections = 31;
            typedef Eigen::Matrix <float, 4, NumDirections> Vertices;

          public:

            Dome ();

            const Vertices&
            getVertices () const;

          private:

            Vertices vertices_;
        };

        uint8_t
        trimRGB (const float val) const;

        // 2 - 1
        // | / |
        // 3 - 0
        void
        addToMesh (const PointIHS& pt_0,
                   const PointIHS& pt_1,
                   const PointIHS& pt_2,
                   const PointIHS& pt_3,
                   VertexIndex&    vi_0,
                   VertexIndex&    vi_1,
                   VertexIndex&    vi_2,
                   VertexIndex&    vi_3,
                   const MeshPtr&  mesh) const;

        void
        addToMesh (const PointIHS& pt_0,
                   const PointIHS& pt_1,
                   const PointIHS& pt_2,
                   VertexIndex&    vi_0,
                   VertexIndex&    vi_1,
                   VertexIndex&    vi_2,
                   const MeshPtr&  mesh) const;

        bool
        distanceThreshold (const PointIHS& pt_0,
                           const PointIHS& pt_1,
                           const PointIHS& pt_2) const;

        bool
        distanceThreshold (const PointIHS& pt_0,
                           const PointIHS& pt_1,
                           const PointIHS& pt_2,
                           const PointIHS& pt_3) const;

        void
        addDirection (const Eigen::Vector4f& normal,
                      const Eigen::Vector4f& direction,
                      uint32_t&              directions) const;

        unsigned int
        countDirections (const unsigned int directions) const;

        // Nearest neighbor search.
        KdTreePtr kd_tree_;

        // Maximum squared distance below which points are averaged out.
        float squared_distance_max_;

        // Minium dot product between normals above which points are averaged out.
        float dot_normal_min_;

        // Minimum weight above which points are added.
        float weight_min_;

        // A point dies if it has not been updated in the last age_max_ frames and the number of observed directions are below count_min_.
        unsigned int age_max_;
        unsigned int count_min_;

        // Dome to check from which direction the point has been observed
        Dome dome_;
    };
  } // End namespace ihs
} // End namespace pcl

#endif // PCL_APPS_IN_HAND_SCANNER_INTEGRATION_H
