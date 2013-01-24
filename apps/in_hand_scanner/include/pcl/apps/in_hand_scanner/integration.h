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
    /** \brief Integrate several clouds into a common mesh.
      * \author Martin Saelzle
      * \ingroup apps
      */
    class PCL_EXPORTS Integration
    {
      public:

        typedef pcl::PointXYZRGBNormal              PointXYZRGBNormal;
        typedef pcl::PointCloud <PointXYZRGBNormal> CloudXYZRGBNormal;
        typedef CloudXYZRGBNormal::Ptr              CloudXYZRGBNormalPtr;
        typedef CloudXYZRGBNormal::ConstPtr         CloudXYZRGBNormalConstPtr;

        typedef pcl::ihs::Mesh            Mesh;
        typedef pcl::ihs::MeshPtr         MeshPtr;
        typedef pcl::ihs::MeshConstPtr    MeshConstPtr;
        typedef Mesh::VertexIndex         VertexIndex;
        typedef Mesh::VertexIndices       VertexIndices;

        /** \brief Constructor. */
        Integration ();

        /** \brief Reconstructs a mesh from an organized cloud.
          * \param[in] cloud_data Input cloud. Must be organized.
          * \param[in] mesh_model Reconstructed mesh.
          * \return true if success.
          */
        bool
        reconstructMesh (const CloudXYZRGBNormalConstPtr& cloud_data,
                         MeshPtr&                         mesh_model) const;

        /** \brief Merge the organized cloud into the mesh.
          * \param[in] cloud_data Input cloud. Must be organized.
          * \param[in,out] mesh_model Mesh with new points integrated.
          * \param[in] T Transformation that aligns the data cloud with the model mesh.
          * \return true if success.
          */
        bool
        merge (const CloudXYZRGBNormalConstPtr& cloud_data,
               MeshPtr&                         mesh_model,
               const Eigen::Matrix4f&           T) const;

        /** \brief Outlier rejection. In each merge step points that have not been observed again age by one iteration. Points that are observed again get an age of 0. Once a point reaches the maximum age it is decided if the point is removed or kept in the mesh. A point is removed if it has not been observed from a minimum number of directions.
          * \param[in,out] mesh The mesh which should be processed.
          * \param[in] cleanup Calls mesh.cleanup () if true.
          */
        void
        age (const MeshPtr& mesh, const bool cleanup=true) const;

        /** \brief Removes unfit vertices regardless of their age. Unfit vertices are those that have not been observed from enough directions.
          * \param[in,out] mesh The which should be processed.
          * \param[in] cleanup Calls mesh.cleanup () if true.
          */
        void
        removeUnfitVertices (const MeshPtr& mesh, const bool cleanup=true) const;

        /** @{ */
        /** \brief Corresponding points are averaged out if their distance is below a distance threshold. Else the points are added to the mesh as new vertices (Set in cm^2).
          * \note Must be greater than zero.
          */
        void  setMaxSquaredDistance (const float squared_distance);
        float getMaxSquaredDistance () const;
        /** @} */

        /** @{ */
        /** \brief Corresponding points are only averaged out if the angle between the normals is smaller than an angle threshold.
          * \note Must be between 0 and 180. Values outside this range are clamped to the nearest valid value.
          */
        void  setMaxAngle (const float angle);
        float getMaxAngle () const;
        /** @} */

        /** @{  */
        /** \brief Once a point reaches the maximum age it is decided if the point is removed or kept in the mesh.
          * \note Must be greater than zero.
          */
        void         setMaxAge (const unsigned int age);
        unsigned int getMaxAge () const;
        /** @} */

        /** @{  */
        /** \brief A point is removed if it has not been observed from a minimum number of directions.
          * \note Must be greater than zero.
          */
        void         setMinDirections (const unsigned int directions);
        unsigned int getMinDirections () const;
        /** @} */

      private:

        typedef pcl::PointXYZ              PointXYZ;
        typedef pcl::PointCloud <PointXYZ> CloudXYZ;
        typedef CloudXYZ::Ptr              CloudXYZPtr;
        typedef CloudXYZ::ConstPtr         CloudXYZConstPtr;

        typedef pcl::ihs::PointIHS         PointIHS;
        typedef pcl::ihs::CloudIHS         CloudIHS;
        typedef pcl::ihs::CloudIHSPtr      CloudIHSPtr;
        typedef pcl::ihs::CloudIHSConstPtr CloudIHSConstPtr;

        typedef pcl::KdTree <PointXYZ>           KdTree;
        typedef boost::shared_ptr <KdTree>       KdTreePtr;
        typedef boost::shared_ptr <const KdTree> KdTreeConstPtr;

        uint8_t
        trimRGB (const float val) const;

        /** \brief Adds two triangles between points 0-1-3 and 1-2-3 to the mesh. */
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

        /** \brief Adds a triangle between the points 0-1-2 to the mesh. */
        void
        addToMesh (const PointIHS& pt_0,
                   const PointIHS& pt_1,
                   const PointIHS& pt_2,
                   VertexIndex&    vi_0,
                   VertexIndex&    vi_1,
                   VertexIndex&    vi_2,
                   const MeshPtr&  mesh) const;

        /** \brief Returns true if the distance between the three points is below a threshold. */
        bool
        distanceThreshold (const PointIHS& pt_0,
                           const PointIHS& pt_1,
                           const PointIHS& pt_2) const;

        /** \brief Returns true if the distance between the four points is below a threshold. */
        bool
        distanceThreshold (const PointIHS& pt_0,
                           const PointIHS& pt_1,
                           const PointIHS& pt_2,
                           const PointIHS& pt_3) const;

        ////////////////////////////////////////////////////////////////////////
        // Members
        ////////////////////////////////////////////////////////////////////////

        /** \brief Nearest neighbor search. */
        KdTreePtr kd_tree_;

        /** \brief Maximum squared distance below which points are averaged out. */
        float max_squared_distance_;

        /** \brief Maximum angle between normals below which points are averaged out. In degrees. */
        float max_angle_;

        /** \brief Minimum weight above which points are added. */
        float min_weight_;

        /** \brief Once a point reaches the maximum age it is decided if the point is removed or kept in the mesh. */
        unsigned int max_age_;

        /** \brief A point is removed if it has not been observed from a minimum number of directions. */
        unsigned int min_directions_;
    };
  } // End namespace ihs
} // End namespace pcl

#endif // PCL_APPS_IN_HAND_SCANNER_INTEGRATION_H
