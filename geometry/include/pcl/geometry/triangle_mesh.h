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

#include <utility>

namespace pcl {
namespace geometry {
/** \brief Tag describing the type of the mesh. */
struct TriangleMeshTag {};

/** \brief Half-edge mesh that can only store triangles.
 * \tparam MeshTraitsT Please have a look at pcl::geometry::DefaultMeshTraits.
 * \author Martin Saelzle
 * \ingroup geometry
 */
template <class MeshTraitsT>
class TriangleMesh : public pcl::geometry::MeshBase<TriangleMesh<MeshTraitsT>,
                                                    MeshTraitsT,
                                                    TriangleMeshTag> {
public:
  using Base =
      pcl::geometry::MeshBase<TriangleMesh<MeshTraitsT>, MeshTraitsT, TriangleMeshTag>;

  using Self = TriangleMesh<MeshTraitsT>;
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
  using FaceIndexPair = std::pair<FaceIndex, FaceIndex>;

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
  TriangleMesh() : Base(), add_triangle_(3), inner_he_atp_(4), is_new_atp_(4) {}

  /** \brief The base method of addFace is hidden because of the overloads in this
   * class. */
  using Base::addFace;

  /** \brief Add a triangle to the mesh. Data is only added if it is associated with the
   * elements. The last vertex is connected with the first one.
   * \param[in] idx_v_0 Index to the first vertex.
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

  /** \brief Add two triangles for the four given input vertices. When using a manifold
   * triangle mesh it is not possible to connect two bounded regions without going
   * through a non-manifold intermediate step. This method first tries to add the
   * triangles individually and if this fails connects the whole configuration at once
   * (if possible).
   * \param[in] vertices       Indices to the vertices of the new face.  (The size
   * must be equal to four).
   * \param[in] face_data      Data that is set for the face.
   * \param[in] half_edge_data Data that is set for all added half-edges.
   * \param[in] edge_data      Data that is set for all added edges.
   * \return Pair of face indices. The first index is valid if one triangle was added.
   * Both indices are valid if two triangles were added.
   * \warning The vertices must be valid and unique (each vertex may be contained only
   * once). Not complying with this requirement results in undefined behavior!
   */
  FaceIndexPair
  addTrianglePair(const VertexIndices& vertices,
                  const FaceData& face_data = FaceData(),
                  const EdgeData& edge_data = EdgeData(),
                  const HalfEdgeData& half_edge_data = HalfEdgeData())
  {
    if (vertices.size() != 4) {
      return (std::make_pair(FaceIndex(), FaceIndex()));
    }
    return (this->addTrianglePair(vertices[0],
                                  vertices[1],
                                  vertices[2],
                                  vertices[3],
                                  face_data,
                                  edge_data,
                                  half_edge_data));
  }

  /** \brief Add two triangles for the four given input vertices. When using a manifold
   * triangle mesh it is not possible to connect two bounded regions without going
   * through a non-manifold intermediate step. This method first tries to add the
   * triangles individually and if this fails connects the whole configuration at once
   * (if possible).
   * \param[in] idx_v_0        Index to the first vertex.
   * \param[in] idx_v_1        Index to the second vertex.
   * \param[in] idx_v_2        Index to the third vertex.
   * \param[in] idx_v_3        Index to the fourth vertex.
   * \param[in] face_data      Data that is set for the face.
   * \param[in] half_edge_data Data that is set for all added half-edges.
   * \param[in] edge_data      Data that is set for all added edges.
   * \return Pair of face indices. The first index is valid if one triangle was added.
   * Both indices are valid if two triangles were added.
   * \warning The vertices must be valid and unique (each vertex may be contained only
   * once). Not complying with this requirement results in undefined behavior!
   */
  inline FaceIndexPair
  addTrianglePair(const VertexIndex& idx_v_0,
                  const VertexIndex& idx_v_1,
                  const VertexIndex& idx_v_2,
                  const VertexIndex& idx_v_3,
                  const FaceData& face_data = FaceData(),
                  const EdgeData& edge_data = EdgeData(),
                  const HalfEdgeData& half_edge_data = HalfEdgeData())
  {
    // Try to add two faces
    // 3 - 2
    // | / |
    // 0 - 1
    FaceIndex idx_face_0 = this->addFace(idx_v_0, idx_v_1, idx_v_2, face_data);
    FaceIndex idx_face_1 = this->addFace(idx_v_0, idx_v_2, idx_v_3, face_data);

    if (idx_face_0.isValid()) {
      return (std::make_pair(idx_face_0, idx_face_1));
    }
    if (idx_face_1.isValid()) {
      idx_face_0 = this->addFace(
          idx_v_0, idx_v_1, idx_v_2, face_data); // might be possible to add now
      return (std::make_pair(idx_face_1, idx_face_0));
    }

    // Try to add two faces
    // 3 - 2
    // | \ |
    // 0 - 1
    idx_face_0 = this->addFace(idx_v_1, idx_v_2, idx_v_3, face_data);
    idx_face_1 = this->addFace(idx_v_0, idx_v_1, idx_v_3, face_data);

    if (idx_face_0.isValid()) {
      return (std::make_pair(idx_face_0, idx_face_1));
    }
    if (idx_face_1.isValid()) {
      idx_face_0 = this->addFace(
          idx_v_1, idx_v_2, idx_v_3, face_data); // might be possible to add now
      return (std::make_pair(idx_face_1, idx_face_0));
    }

    if (!IsManifold::value) {
      return (std::make_pair(FaceIndex(), FaceIndex()));
    }

    // Check manifoldness
    if (!Base::checkTopology1(
            idx_v_0, idx_v_1, inner_he_atp_[0], is_new_atp_[0], IsManifold()) ||
        !Base::checkTopology1(
            idx_v_1, idx_v_2, inner_he_atp_[1], is_new_atp_[1], IsManifold()) ||
        !Base::checkTopology1(
            idx_v_2, idx_v_3, inner_he_atp_[2], is_new_atp_[2], IsManifold()) ||
        !Base::checkTopology1(
            idx_v_3, idx_v_0, inner_he_atp_[3], is_new_atp_[3], IsManifold())) {
      return (std::make_pair(FaceIndex(), FaceIndex()));
    }

    // Connect the triangle pair
    if (!is_new_atp_[0] && is_new_atp_[1] && !is_new_atp_[2] && is_new_atp_[3]) {
      return (this->connectTrianglePair(inner_he_atp_[0],
                                        inner_he_atp_[2],
                                        idx_v_0,
                                        idx_v_1,
                                        idx_v_2,
                                        idx_v_3,
                                        face_data,
                                        edge_data,
                                        half_edge_data));
    }
    if (is_new_atp_[0] && !is_new_atp_[1] && is_new_atp_[2] && !is_new_atp_[3]) {
      return (this->connectTrianglePair(inner_he_atp_[1],
                                        inner_he_atp_[3],
                                        idx_v_1,
                                        idx_v_2,
                                        idx_v_3,
                                        idx_v_0,
                                        face_data,
                                        edge_data,
                                        half_edge_data));
    }
    return (std::make_pair(FaceIndex(), FaceIndex()));
  }

private:
  // NOTE: Can't use the typedef of Base as a friend.
  friend class pcl::geometry::
      MeshBase<TriangleMesh<MeshTraitsT>, MeshTraitsT, pcl::geometry::TriangleMeshTag>;

  /** \brief addFace for the triangular mesh. */
  inline FaceIndex
  addFaceImpl(const VertexIndices& vertices,
              const FaceData& face_data,
              const EdgeData& edge_data,
              const HalfEdgeData& half_edge_data)
  {
    if (vertices.size() == 3)
      return (this->addFaceImplBase(vertices, face_data, edge_data, half_edge_data));
    return (FaceIndex());
  }

  /** \brief Connect the triangles a-b-c and a-c-d. The edges a-b and c-d must be old
   * and the edges b-c and d-a must be new. */
  // d - c
  // | / |
  // a - b
  FaceIndexPair
  connectTrianglePair(const HalfEdgeIndex& idx_he_ab,
                      const HalfEdgeIndex& idx_he_cd,
                      const VertexIndex& idx_v_a,
                      const VertexIndex& idx_v_b,
                      const VertexIndex& idx_v_c,
                      const VertexIndex& idx_v_d,
                      const FaceData& face_data,
                      const EdgeData& edge_data,
                      const HalfEdgeData& he_data)
  {
    // Add new half-edges
    const HalfEdgeIndex idx_he_bc = Base::addEdge(idx_v_b, idx_v_c, he_data, edge_data);
    const HalfEdgeIndex idx_he_da = Base::addEdge(idx_v_d, idx_v_a, he_data, edge_data);
    const HalfEdgeIndex idx_he_ca = Base::addEdge(idx_v_c, idx_v_a, he_data, edge_data);

    const HalfEdgeIndex idx_he_cb = Base::getOppositeHalfEdgeIndex(idx_he_bc);
    const HalfEdgeIndex idx_he_ad = Base::getOppositeHalfEdgeIndex(idx_he_da);
    const HalfEdgeIndex idx_he_ac = Base::getOppositeHalfEdgeIndex(idx_he_ca);

    // Get the existing half-edges
    const HalfEdgeIndex idx_he_ab_prev =
        Base::getPrevHalfEdgeIndex(idx_he_ab); // No reference!
    const HalfEdgeIndex idx_he_ab_next =
        Base::getNextHalfEdgeIndex(idx_he_ab); // No reference!

    const HalfEdgeIndex idx_he_cd_prev =
        Base::getPrevHalfEdgeIndex(idx_he_cd); // No reference!
    const HalfEdgeIndex idx_he_cd_next =
        Base::getNextHalfEdgeIndex(idx_he_cd); // No reference!

    // Connect the outer half-edges
    Base::connectPrevNext(idx_he_ab_prev, idx_he_ad);
    Base::connectPrevNext(idx_he_ad, idx_he_cd_next);
    Base::connectPrevNext(idx_he_cd_prev, idx_he_cb);
    Base::connectPrevNext(idx_he_cb, idx_he_ab_next);

    // Connect the inner half-edges
    Base::connectPrevNext(idx_he_ab, idx_he_bc);
    Base::connectPrevNext(idx_he_bc, idx_he_ca);
    Base::connectPrevNext(idx_he_ca, idx_he_ab);

    Base::connectPrevNext(idx_he_ac, idx_he_cd);
    Base::connectPrevNext(idx_he_cd, idx_he_da);
    Base::connectPrevNext(idx_he_da, idx_he_ac);

    // Connect the vertices to the boundary half-edges
    Base::setOutgoingHalfEdgeIndex(idx_v_a, idx_he_ad);
    Base::setOutgoingHalfEdgeIndex(idx_v_b, idx_he_ab_next);
    Base::setOutgoingHalfEdgeIndex(idx_v_c, idx_he_cb);
    Base::setOutgoingHalfEdgeIndex(idx_v_d, idx_he_cd_next);

    // Add and connect the faces
    HalfEdgeIndices inner_he_abc;
    inner_he_abc.reserve(3);
    inner_he_abc.push_back(idx_he_ab);
    inner_he_abc.push_back(idx_he_bc);
    inner_he_abc.push_back(idx_he_ca);

    HalfEdgeIndices inner_he_acd;
    inner_he_acd.reserve(3);
    inner_he_acd.push_back(idx_he_ac);
    inner_he_acd.push_back(idx_he_cd);
    inner_he_acd.push_back(idx_he_da);

    const FaceIndex idx_f_abc = Base::connectFace(inner_he_abc, face_data);
    const FaceIndex idx_f_acd = Base::connectFace(inner_he_acd, face_data);

    return (std::make_pair(idx_f_abc, idx_f_acd));
  }

  ////////////////////////////////////////////////////////////////////////
  // Members
  ////////////////////////////////////////////////////////////////////////

  /** \brief Storage for adding a triangle. */
  VertexIndices add_triangle_;

  /** \brief Storage for addTrianglePair. */
  HalfEdgeIndices inner_he_atp_;

  /** \brief Storage for addTrianglePair. */
  std::vector<bool> is_new_atp_;

public:
  PCL_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace geometry
} // End namespace pcl
