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

#pragma once

#include <pcl/geometry/mesh_base.h>
#include <pcl/memory.h>
#include <pcl/pcl_macros.h>

namespace pcl {
namespace geometry {
/** \brief Tag describing the type of the mesh. */
struct PolygonMeshTag {};

/** \brief General half-edge mesh that can store any polygon with a minimum number of
 * vertices of 3.
 * \tparam MeshTraitsT Please have a look at
 * pcl::geometry::DefaultMeshTraits.
 * \author Martin Saelzle
 * \ingroup geometry
 */
template <class MeshTraitsT>
class PolygonMesh : public pcl::geometry::MeshBase<PolygonMesh<MeshTraitsT>,
                                                   MeshTraitsT,
                                                   PolygonMeshTag> {
public:
  using Base =
      pcl::geometry::MeshBase<PolygonMesh<MeshTraitsT>, MeshTraitsT, PolygonMeshTag>;

  using Self = PolygonMesh<MeshTraitsT>;
  using Ptr = shared_ptr<Self>;
  using ConstPtr = shared_ptr<const Self>;

  using VertexData = typename Base::VertexData;
  using HalfEdgeData = typename Base::HalfEdgeData;
  using EdgeData = typename Base::EdgeData;
  using FaceData = typename Base::FaceData;
  using IsManifold = typename Base::IsManifold;
  using MeshTag = typename Base::MeshTag;

  using HasVertexData = typename Base::HasVertexData;
  using HasHalfEdgeData = typename Base::HasHalfEdgeData;
  using HasEdgeData = typename Base::HasEdgeData;
  using HasFaceData = typename Base::HasFaceData;

  using VertexDataCloud = typename Base::VertexDataCloud;
  using HalfEdgeDataCloud = typename Base::HalfEdgeDataCloud;
  using EdgeDataCloud = typename Base::EdgeDataCloud;
  using FaceDataCloud = typename Base::FaceDataCloud;

  // Indices
  using VertexIndex = typename Base::VertexIndex;
  using HalfEdgeIndex = typename Base::HalfEdgeIndex;
  using EdgeIndex = typename Base::EdgeIndex;
  using FaceIndex = typename Base::FaceIndex;

  using VertexIndices = typename Base::VertexIndices;
  using HalfEdgeIndices = typename Base::HalfEdgeIndices;
  using EdgeIndices = typename Base::EdgeIndices;
  using FaceIndices = typename Base::FaceIndices;

  // Circulators
  using VertexAroundVertexCirculator = typename Base::VertexAroundVertexCirculator;
  using OutgoingHalfEdgeAroundVertexCirculator =
      typename Base::OutgoingHalfEdgeAroundVertexCirculator;
  using IncomingHalfEdgeAroundVertexCirculator =
      typename Base::IncomingHalfEdgeAroundVertexCirculator;
  using FaceAroundVertexCirculator = typename Base::FaceAroundVertexCirculator;
  using VertexAroundFaceCirculator = typename Base::VertexAroundFaceCirculator;
  using InnerHalfEdgeAroundFaceCirculator =
      typename Base::InnerHalfEdgeAroundFaceCirculator;
  using OuterHalfEdgeAroundFaceCirculator =
      typename Base::OuterHalfEdgeAroundFaceCirculator;
  using FaceAroundFaceCirculator = typename Base::FaceAroundFaceCirculator;

  /** \brief Constructor. */
  PolygonMesh() : Base(), add_triangle_(3), add_quad_(4) {}

  /** \brief The base method of addFace is hidden because of the overloads in this
   * class. */
  using Base::addFace;

  /** \brief Add a triangle to the mesh. Data is only added if it is associated with the
   * elements. The last vertex is connected with the first one.
   * \param[in] idx_v_0 Index
   * to the first vertex.
   * \param[in] idx_v_1        Index to the second vertex.
   * \param[in] idx_v_2        Index to the third vertex.
   * \param[in] face_data      Data that is set for the face.
   * \param[in] half_edge_data Data that is set for all added half-edges.
   * \param[in] edge_data      Data that is set for all added edges.
   * \return Index to the new face. Failure is signaled by returning an invalid face
   * index.
   * \warning The vertices must be valid and unique (each vertex may be contained
   * only once). Not complying with this requirement results in undefined behavior!
   */
  inline FaceIndex
  addFace(const VertexIndex& idx_v_0,
          const VertexIndex& idx_v_1,
          const VertexIndex& idx_v_2,
          const FaceData& face_data = FaceData(),
          const EdgeData& edge_data = EdgeData(),
          const HalfEdgeData& half_edge_data = HalfEdgeData())
  {
    add_triangle_[0] = idx_v_0;
    add_triangle_[1] = idx_v_1;
    add_triangle_[2] = idx_v_2;

    return (this->addFaceImplBase(add_triangle_, face_data, edge_data, half_edge_data));
  }

  /** \brief Add a quad to the mesh. Data is only added if it is associated with the
   * elements. The last vertex is connected with the first one.
   * \param[in] idx_v_0 Index to the first vertex.
   * \param[in] idx_v_1        Index to the second vertex.
   * \param[in] idx_v_2        Index to the third vertex.
   * \param[in] idx_v_3        Index to the fourth vertex.
   * \param[in] face_data      Data that is set for the face.
   * \param[in] half_edge_data Data that is set for all added half-edges.
   * \param[in] edge_data      Data that is set for all added edges.
   * \return Index to the new face. Failure is signaled by returning an invalid face
   * index.
   * \warning The vertices must be valid and unique (each vertex may be contained
   * only once). Not complying with this requirement results in undefined behavior!
   */
  inline FaceIndex
  addFace(const VertexIndex& idx_v_0,
          const VertexIndex& idx_v_1,
          const VertexIndex& idx_v_2,
          const VertexIndex& idx_v_3,
          const FaceData& face_data = FaceData(),
          const EdgeData& edge_data = EdgeData(),
          const HalfEdgeData& half_edge_data = HalfEdgeData())
  {
    add_quad_[0] = idx_v_0;
    add_quad_[1] = idx_v_1;
    add_quad_[2] = idx_v_2;
    add_quad_[3] = idx_v_3;

    return (this->addFaceImplBase(add_quad_, face_data, edge_data, half_edge_data));
  }

private:
  // NOTE: Can't use the typedef of Base as a friend.
  friend class pcl::geometry::
      MeshBase<PolygonMesh<MeshTraitsT>, MeshTraitsT, pcl::geometry::PolygonMeshTag>;

  /** \brief addFace for the polygon mesh. */
  inline FaceIndex
  addFaceImpl(const VertexIndices& vertices,
              const FaceData& face_data,
              const EdgeData& edge_data,
              const HalfEdgeData& half_edge_data)
  {
    return (this->addFaceImplBase(vertices, face_data, edge_data, half_edge_data));
  }

  ////////////////////////////////////////////////////////////////////////
  // Members
  ////////////////////////////////////////////////////////////////////////

  /** \brief Storage for adding a triangle. */
  VertexIndices add_triangle_;

  /** \brief Storage for adding a quad. */
  VertexIndices add_quad_;

public:
  PCL_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace geometry
} // End namespace pcl
