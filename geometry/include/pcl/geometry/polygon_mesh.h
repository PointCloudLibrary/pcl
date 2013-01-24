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

#ifndef PCL_GEOMETRY_POLYGON_MESH_H
#define PCL_GEOMETRY_POLYGON_MESH_H

#include <pcl/geometry/mesh_base.h>

namespace pcl
{
  namespace geometry
  {
    /** \brief Tag describing the type of the mesh. */
    struct PolygonMeshTag {};

    /** \brief General half-edge mesh that can store any polygon with a minimum number of vertices of 3.
      * \tparam MeshTraitsT Please have a look at pcl::geometry::DefaultMeshTraits.
      * \author Martin Saelzle
      * \ingroup geometry
      */
    template <class MeshTraitsT>
    class PolygonMesh : public pcl::geometry::MeshBase <PolygonMesh <MeshTraitsT>, MeshTraitsT, PolygonMeshTag>
    {
      public:

        typedef pcl::geometry::MeshBase <PolygonMesh <MeshTraitsT>, MeshTraitsT, PolygonMeshTag> Base;

        typedef PolygonMesh <MeshTraitsT>      Self;
        typedef boost::shared_ptr <Self>       Ptr;
        typedef boost::shared_ptr <const Self> ConstPtr;

        typedef typename Base::VertexData   VertexData;
        typedef typename Base::HalfEdgeData HalfEdgeData;
        typedef typename Base::EdgeData     EdgeData;
        typedef typename Base::FaceData     FaceData;
        typedef typename Base::IsManifold   IsManifold;
        typedef typename Base::MeshTag      MeshTag;

        typedef typename Base::HasVertexData   HasVertexData;
        typedef typename Base::HasHalfEdgeData HasHalfEdgeData;
        typedef typename Base::HasEdgeData     HasEdgeData;
        typedef typename Base::HasFaceData     HasFaceData;

        typedef typename Base::VertexDataCloud   VertexDataCloud;
        typedef typename Base::HalfEdgeDataCloud HalfEdgeDataCloud;
        typedef typename Base::EdgeDataCloud     EdgeDataCloud;
        typedef typename Base::FaceDataCloud     FaceDataCloud;

        // Indices
        typedef typename Base::VertexIndex   VertexIndex;
        typedef typename Base::HalfEdgeIndex HalfEdgeIndex;
        typedef typename Base::EdgeIndex     EdgeIndex;
        typedef typename Base::FaceIndex     FaceIndex;

        typedef typename Base::VertexIndices   VertexIndices;
        typedef typename Base::HalfEdgeIndices HalfEdgeIndices;
        typedef typename Base::EdgeIndices     EdgeIndices;
        typedef typename Base::FaceIndices     FaceIndices;

        // Circulators
        typedef typename Base::VertexAroundVertexCirculator           VertexAroundVertexCirculator;
        typedef typename Base::OutgoingHalfEdgeAroundVertexCirculator OutgoingHalfEdgeAroundVertexCirculator;
        typedef typename Base::IncomingHalfEdgeAroundVertexCirculator IncomingHalfEdgeAroundVertexCirculator;
        typedef typename Base::FaceAroundVertexCirculator             FaceAroundVertexCirculator;
        typedef typename Base::VertexAroundFaceCirculator             VertexAroundFaceCirculator;
        typedef typename Base::InnerHalfEdgeAroundFaceCirculator      InnerHalfEdgeAroundFaceCirculator;
        typedef typename Base::OuterHalfEdgeAroundFaceCirculator      OuterHalfEdgeAroundFaceCirculator;
        typedef typename Base::FaceAroundFaceCirculator               FaceAroundFaceCirculator;

        /** \brief Constructor. */
        PolygonMesh ()
          : Base (),
            add_triangle_ (3),
            add_quad_ (4)
        {
        }

        /** \brief The base method of addFace is hidden because of the overloads in this class. */
        using Base::addFace;

        /** \brief Add a triangle to the mesh. Data is only added if it is associated with the elements. The last vertex is connected with the first one.
          * \param[in] idx_v_0        Index to the first vertex.
          * \param[in] idx_v_1        Index to the second vertex.
          * \param[in] idx_v_2        Index to the third vertex.
          * \param[in] face_data      Data that is set for the face.
          * \param[in] half_edge_data Data that is set for all added half-edges.
          * \param[in] edge_data      Data that is set for all added edges.
          * \return Index to the new face. Failure is signaled by returning an invalid face index.
          * \warning The vertices must be valid and unique (each vertex may be contained only once). Not complying with this requirement results in undefined behavior!
          */
        inline FaceIndex
        addFace (const VertexIndex&   idx_v_0,
                 const VertexIndex&   idx_v_1,
                 const VertexIndex&   idx_v_2,
                 const FaceData&      face_data      = FaceData (),
                 const EdgeData&      edge_data      = EdgeData (),
                 const HalfEdgeData&  half_edge_data = HalfEdgeData ())
        {
          add_triangle_ [0] = idx_v_0;
          add_triangle_ [1] = idx_v_1;
          add_triangle_ [2] = idx_v_2;

          return (this->addFaceImplBase (add_triangle_, face_data, edge_data, half_edge_data));
        }

        /** \brief Add a quad to the mesh. Data is only added if it is associated with the elements. The last vertex is connected with the first one.
          * \param[in] idx_v_0        Index to the first vertex.
          * \param[in] idx_v_1        Index to the second vertex.
          * \param[in] idx_v_2        Index to the third vertex.
          * \param[in] idx_v_3        Index to the fourth vertex.
          * \param[in] face_data      Data that is set for the face.
          * \param[in] half_edge_data Data that is set for all added half-edges.
          * \param[in] edge_data      Data that is set for all added edges.
          * \return Index to the new face. Failure is signaled by returning an invalid face index.
          * \warning The vertices must be valid and unique (each vertex may be contained only once). Not complying with this requirement results in undefined behavior!
          */
        inline FaceIndex
        addFace (const VertexIndex&   idx_v_0,
                 const VertexIndex&   idx_v_1,
                 const VertexIndex&   idx_v_2,
                 const VertexIndex&   idx_v_3,
                 const FaceData&      face_data      = FaceData (),
                 const EdgeData&      edge_data      = EdgeData (),
                 const HalfEdgeData&  half_edge_data = HalfEdgeData ())
        {
          add_quad_ [0] = idx_v_0;
          add_quad_ [1] = idx_v_1;
          add_quad_ [2] = idx_v_2;
          add_quad_ [3] = idx_v_3;

          return (this->addFaceImplBase (add_quad_, face_data, edge_data, half_edge_data));
        }

      private:

        // NOTE: Can't use the typedef of Base as a friend.
        friend class pcl::geometry::MeshBase <PolygonMesh <MeshTraitsT>, MeshTraitsT, pcl::geometry::PolygonMeshTag>;

        /** \brief addFace for the polygon mesh. */
        inline FaceIndex
        addFaceImpl (const VertexIndices& vertices,
                     const FaceData&      face_data,
                     const EdgeData&      edge_data,
                     const HalfEdgeData&  half_edge_data)
        {
          return (this->addFaceImplBase (vertices, face_data, edge_data, half_edge_data));
        }

        ////////////////////////////////////////////////////////////////////////
        // Members
        ////////////////////////////////////////////////////////////////////////

        /** \brief Storage for adding a triangle. */
        VertexIndices add_triangle_;

        /** \brief Storage for adding a quad. */
        VertexIndices add_quad_;

      public:

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
  } // End namespace geom
} // End namespace pcl

#endif // PCL_GEOMETRY_POLYGON_MESH_H
