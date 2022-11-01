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

#include <pcl/geometry/mesh_circulators.h>
#include <pcl/geometry/mesh_elements.h>
#include <pcl/geometry/mesh_indices.h>
#include <pcl/geometry/mesh_traits.h>
#include <pcl/memory.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>

#include <type_traits>
#include <vector>

////////////////////////////////////////////////////////////////////////////////
// Global variables used during testing
////////////////////////////////////////////////////////////////////////////////

#ifdef PCL_GEOMETRY_MESH_BASE_TEST_DELETE_FACE_MANIFOLD_2
namespace pcl {
namespace geometry {
bool g_pcl_geometry_mesh_base_test_delete_face_manifold_2_success;
} // End namespace geometry
} // End namespace pcl
#endif

////////////////////////////////////////////////////////////////////////////////
// Forward declarations
////////////////////////////////////////////////////////////////////////////////

namespace pcl {
namespace geometry {
template <class MeshT>
class MeshIO;
} // End namespace geometry
} // End namespace pcl

////////////////////////////////////////////////////////////////////////////////
// MeshBase
////////////////////////////////////////////////////////////////////////////////

namespace pcl {
namespace geometry {
/**
 * \brief Base class for the half-edge mesh.
 * \tparam DerivedT Has to implement the method 'addFaceImpl'. Please have a look at
 * pcl::geometry::TriangleMesh, pcl::geometry::QuadMesh and pcl::geometry::PolygonMesh.
 * \tparam MeshTraitsT Please have a look at pcl::geometry::DefaultMeshTraits.
 * \tparam MeshTagT Tag describing the type of the mesh, e.g. TriangleMeshTag,
 * QuadMeshTag, PolygonMeshTag.
 * \author Martin Saelzle
 * \ingroup geometry
 * \todo Add documentation
 */
template <class DerivedT, class MeshTraitsT, class MeshTagT>
class MeshBase {
public:
  using Self = MeshBase<DerivedT, MeshTraitsT, MeshTagT>;
  using Ptr = shared_ptr<Self>;
  using ConstPtr = shared_ptr<const Self>;

  using Derived = DerivedT;

  // These have to be defined in the traits class.
  using VertexData = typename MeshTraitsT::VertexData;
  using HalfEdgeData = typename MeshTraitsT::HalfEdgeData;
  using EdgeData = typename MeshTraitsT::EdgeData;
  using FaceData = typename MeshTraitsT::FaceData;
  using IsManifold = typename MeshTraitsT::IsManifold;

  // Check if the mesh traits are defined correctly.
  static_assert(std::is_convertible<IsManifold, bool>::value,
                "MeshTraitsT::IsManifold is not convertible to bool");

  using MeshTag = MeshTagT;

  // Data
  using HasVertexData =
      std::integral_constant<bool,
                             !std::is_same<VertexData, pcl::geometry::NoData>::value>;
  using HasHalfEdgeData =
      std::integral_constant<bool,
                             !std::is_same<HalfEdgeData, pcl::geometry::NoData>::value>;
  using HasEdgeData =
      std::integral_constant<bool,
                             !std::is_same<EdgeData, pcl::geometry::NoData>::value>;
  using HasFaceData =
      std::integral_constant<bool,
                             !std::is_same<FaceData, pcl::geometry::NoData>::value>;

  using VertexDataCloud = pcl::PointCloud<VertexData>;
  using HalfEdgeDataCloud = pcl::PointCloud<HalfEdgeData>;
  using EdgeDataCloud = pcl::PointCloud<EdgeData>;
  using FaceDataCloud = pcl::PointCloud<FaceData>;

  // Indices
  using VertexIndex = pcl::geometry::VertexIndex;
  using HalfEdgeIndex = pcl::geometry::HalfEdgeIndex;
  using EdgeIndex = pcl::geometry::EdgeIndex;
  using FaceIndex = pcl::geometry::FaceIndex;

  using VertexIndices = std::vector<VertexIndex>;
  using HalfEdgeIndices = std::vector<HalfEdgeIndex>;
  using EdgeIndices = std::vector<EdgeIndex>;
  using FaceIndices = std::vector<FaceIndex>;

  // Circulators
  using VertexAroundVertexCirculator =
      pcl::geometry::VertexAroundVertexCirculator<const Self>;
  using OutgoingHalfEdgeAroundVertexCirculator =
      pcl::geometry::OutgoingHalfEdgeAroundVertexCirculator<const Self>;
  using IncomingHalfEdgeAroundVertexCirculator =
      pcl::geometry::IncomingHalfEdgeAroundVertexCirculator<const Self>;
  using FaceAroundVertexCirculator =
      pcl::geometry::FaceAroundVertexCirculator<const Self>;
  using VertexAroundFaceCirculator =
      pcl::geometry::VertexAroundFaceCirculator<const Self>;
  using InnerHalfEdgeAroundFaceCirculator =
      pcl::geometry::InnerHalfEdgeAroundFaceCirculator<const Self>;
  using OuterHalfEdgeAroundFaceCirculator =
      pcl::geometry::OuterHalfEdgeAroundFaceCirculator<const Self>;
  using FaceAroundFaceCirculator = pcl::geometry::FaceAroundFaceCirculator<const Self>;

  /** \brief Constructor. */
  MeshBase()
  : vertex_data_cloud_()
  , half_edge_data_cloud_()
  , edge_data_cloud_()
  , face_data_cloud_()
  {}

  ////////////////////////////////////////////////////////////////////////
  // addVertex / addFace / deleteVertex / deleteEdge / deleteFace / cleanUp
  ////////////////////////////////////////////////////////////////////////

  /**
   * \brief Add a vertex to the mesh.
   * \param[in] vertex_data Data that is stored in the vertex. This is only added if the
   * mesh has data associated with the vertices.
   * \return Index to the new vertex.
   */
  inline VertexIndex
  addVertex(const VertexData& vertex_data = VertexData())
  {
    vertices_.push_back(Vertex());
    this->addData(vertex_data_cloud_, vertex_data, HasVertexData());
    return (VertexIndex(static_cast<int>(this->sizeVertices() - 1)));
  }

  /**
   * \brief Add a face to the mesh. Data is only added if it is associated with the
   * elements. The last vertex is connected with the first one.
   * \param[in] vertices Indices to the vertices of the new face.
   * \param[in] face_data      Data that is set for the face.
   * \param[in] half_edge_data Data that is set for all added half-edges.
   * \param[in] edge_data      Data that is set for all added edges.
   * \return Index to the new face. Failure is signaled by returning an invalid face
   * index.
   * \warning The vertices must be valid and unique (each vertex may be contained
   * only once). Not complying with this requirement results in undefined behavior!
   */
  inline FaceIndex
  addFace(const VertexIndices& vertices,
          const FaceData& face_data = FaceData(),
          const EdgeData& edge_data = EdgeData(),
          const HalfEdgeData& half_edge_data = HalfEdgeData())
  {
    // NOTE: The derived class has to implement addFaceImpl. If needed it can use the
    // general method addFaceImplBase.
    return (static_cast<Derived*>(this)->addFaceImpl(
        vertices, face_data, edge_data, half_edge_data));
  }

  /**
   * \brief Mark the given vertex and all connected half-edges and faces as deleted.
   * \note Call cleanUp () to finally delete all mesh-elements.
   */
  void
  deleteVertex(const VertexIndex& idx_vertex)
  {
    assert(this->isValid(idx_vertex));
    if (this->isDeleted(idx_vertex))
      return;

    delete_faces_vertex_.clear();
    FaceAroundVertexCirculator circ = this->getFaceAroundVertexCirculator(idx_vertex);
    const FaceAroundVertexCirculator circ_end = circ;
    do {
      if (circ.getTargetIndex().isValid()) // Check for boundary.
      {
        delete_faces_vertex_.push_back(circ.getTargetIndex());
      }
    } while (++circ != circ_end);

    for (const auto& delete_me : delete_faces_vertex_) {
      this->deleteFace(delete_me);
    }
  }

  /**
   * \brief Mark the given half-edge, the opposite half-edge and the associated faces
   * as deleted.
   * \note Call cleanUp () to finally delete all mesh-elements.
   */
  void
  deleteEdge(const HalfEdgeIndex& idx_he)
  {
    assert(this->isValid(idx_he));
    if (this->isDeleted(idx_he))
      return;

    HalfEdgeIndex opposite = this->getOppositeHalfEdgeIndex(idx_he);

    if (this->isBoundary(idx_he))
      this->markDeleted(idx_he);
    else
      this->deleteFace(this->getFaceIndex(idx_he));
    if (this->isBoundary(opposite))
      this->markDeleted(opposite);
    else
      this->deleteFace(this->getFaceIndex(opposite));
  }

  /**
   * \brief Mark the given edge (both half-edges) and the associated faces as deleted.
   * \note Call cleanUp () to finally delete all mesh-elements.
   */
  inline void
  deleteEdge(const EdgeIndex& idx_edge)
  {
    assert(this->isValid(idx_edge));
    this->deleteEdge(pcl::geometry::toHalfEdgeIndex(idx_edge));
    assert(this->isDeleted(
        pcl::geometry::toHalfEdgeIndex(idx_edge, false))); // Bug in this class!
  }

  /**
   * \brief Mark the given face as deleted. More faces are deleted if the manifold mesh
   * would become non-manifold.
   * \note Call cleanUp () to finally delete all mesh-elements.
   */
  inline void
  deleteFace(const FaceIndex& idx_face)
  {
    assert(this->isValid(idx_face));
    if (this->isDeleted(idx_face))
      return;

    this->deleteFace(idx_face, IsManifold());
  }

  /**
   * \brief Removes all mesh elements and data that are marked as deleted.
   * \note This removes all isolated vertices as well.
   */
  void
  cleanUp()
  {
    // Copy the non-deleted mesh elements and store the index to their new position
    const VertexIndices new_vertex_indices =
        this->remove<Vertices, VertexDataCloud, VertexIndices, HasVertexData>(
            vertices_, vertex_data_cloud_);
    const HalfEdgeIndices new_half_edge_indices =
        this->remove<HalfEdges, HalfEdgeDataCloud, HalfEdgeIndices, HasHalfEdgeData>(
            half_edges_, half_edge_data_cloud_);
    const FaceIndices new_face_indices =
        this->remove<Faces, FaceDataCloud, FaceIndices, HasFaceData>(faces_,
                                                                     face_data_cloud_);

    // Remove deleted edge data
    if (HasEdgeData::value) {
      auto it_ed_old = edge_data_cloud_.begin();
      auto it_ed_new = edge_data_cloud_.begin();

      for (auto it_ind = new_half_edge_indices.cbegin(),
                it_ind_end = new_half_edge_indices.cend();
           it_ind != it_ind_end;
           it_ind += 2, ++it_ed_old) {
        if (it_ind->isValid()) {
          *it_ed_new++ = *it_ed_old;
        }
      }
      edge_data_cloud_.resize(this->sizeEdges());
    }

    // Adjust the indices
    for (auto it = vertices_.begin(); it != vertices_.end(); ++it) {
      if (it->idx_outgoing_half_edge_.isValid()) {
        it->idx_outgoing_half_edge_ =
            new_half_edge_indices[it->idx_outgoing_half_edge_.get()];
      }
    }

    for (auto it = half_edges_.begin(); it != half_edges_.end(); ++it) {
      it->idx_terminating_vertex_ =
          new_vertex_indices[it->idx_terminating_vertex_.get()];
      it->idx_next_half_edge_ = new_half_edge_indices[it->idx_next_half_edge_.get()];
      it->idx_prev_half_edge_ = new_half_edge_indices[it->idx_prev_half_edge_.get()];
      if (it->idx_face_.isValid()) {
        it->idx_face_ = new_face_indices[it->idx_face_.get()];
      }
    }

    for (auto it = faces_.begin(); it != faces_.end(); ++it) {
      it->idx_inner_half_edge_ = new_half_edge_indices[it->idx_inner_half_edge_.get()];
    }
  }

  ////////////////////////////////////////////////////////////////////////
  // Vertex connectivity
  ////////////////////////////////////////////////////////////////////////

  /** \brief Get the outgoing half-edge index to a given vertex. */
  inline HalfEdgeIndex
  getOutgoingHalfEdgeIndex(const VertexIndex& idx_vertex) const
  {
    assert(this->isValid(idx_vertex));
    return (this->getVertex(idx_vertex).idx_outgoing_half_edge_);
  }

  /** \brief Get the incoming half-edge index to a given vertex. */
  inline HalfEdgeIndex
  getIncomingHalfEdgeIndex(const VertexIndex& idx_vertex) const
  {
    assert(this->isValid(idx_vertex));
    return (this->getOppositeHalfEdgeIndex(this->getOutgoingHalfEdgeIndex(idx_vertex)));
  }

  ////////////////////////////////////////////////////////////////////////
  // Half-edge connectivity
  ////////////////////////////////////////////////////////////////////////

  /** \brief Get the terminating vertex index to a given half-edge. */
  inline VertexIndex
  getTerminatingVertexIndex(const HalfEdgeIndex& idx_half_edge) const
  {
    assert(this->isValid(idx_half_edge));
    return (this->getHalfEdge(idx_half_edge).idx_terminating_vertex_);
  }

  /** \brief Get the originating vertex index to a given half-edge. */
  inline VertexIndex
  getOriginatingVertexIndex(const HalfEdgeIndex& idx_half_edge) const
  {
    assert(this->isValid(idx_half_edge));
    return (
        this->getTerminatingVertexIndex(this->getOppositeHalfEdgeIndex(idx_half_edge)));
  }

  /** \brief Get the opposite half-edge index to a given half-edge. */
  inline HalfEdgeIndex
  getOppositeHalfEdgeIndex(const HalfEdgeIndex& idx_half_edge) const
  {
    assert(this->isValid(idx_half_edge));
    // Check if the index is even or odd and return the other index.
    return (HalfEdgeIndex(idx_half_edge.get() & 1 ? idx_half_edge.get() - 1
                                                  : idx_half_edge.get() + 1));
  }

  /** \brief Get the next half-edge index to a given half-edge. */
  inline HalfEdgeIndex
  getNextHalfEdgeIndex(const HalfEdgeIndex& idx_half_edge) const
  {
    assert(this->isValid(idx_half_edge));
    return (this->getHalfEdge(idx_half_edge).idx_next_half_edge_);
  }

  /** \brief Get the previous half-edge index to a given half-edge. */
  inline HalfEdgeIndex
  getPrevHalfEdgeIndex(const HalfEdgeIndex& idx_half_edge) const
  {
    assert(this->isValid(idx_half_edge));
    return (this->getHalfEdge(idx_half_edge).idx_prev_half_edge_);
  }

  /** \brief Get the face index to a given half-edge. */
  inline FaceIndex
  getFaceIndex(const HalfEdgeIndex& idx_half_edge) const
  {
    assert(this->isValid(idx_half_edge));
    return (this->getHalfEdge(idx_half_edge).idx_face_);
  }

  /** \brief Get the face index to a given half-edge. */
  inline FaceIndex
  getOppositeFaceIndex(const HalfEdgeIndex& idx_half_edge) const
  {
    assert(this->isValid(idx_half_edge));
    return (this->getFaceIndex(this->getOppositeHalfEdgeIndex(idx_half_edge)));
  }

  ////////////////////////////////////////////////////////////////////////
  // Face connectivity
  ////////////////////////////////////////////////////////////////////////

  /** \brief Get the inner half-edge index to a given face. */
  inline HalfEdgeIndex
  getInnerHalfEdgeIndex(const FaceIndex& idx_face) const
  {
    assert(this->isValid(idx_face));
    return (this->getFace(idx_face).idx_inner_half_edge_);
  }

  /** \brief Get the outer half-edge inex to a given face. */
  inline HalfEdgeIndex
  getOuterHalfEdgeIndex(const FaceIndex& idx_face) const
  {
    assert(this->isValid(idx_face));
    return (this->getOppositeHalfEdgeIndex(this->getInnerHalfEdgeIndex(idx_face)));
  }

  ////////////////////////////////////////////////////////////////////////
  // Circulators
  ////////////////////////////////////////////////////////////////////////

  /** \see pcl::geometry::VertexAroundVertexCirculator */
  inline VertexAroundVertexCirculator
  getVertexAroundVertexCirculator(const VertexIndex& idx_vertex) const
  {
    assert(this->isValid(idx_vertex));
    return (VertexAroundVertexCirculator(idx_vertex, this));
  }

  /** \see pcl::geometry::VertexAroundVertexCirculator */
  inline VertexAroundVertexCirculator
  getVertexAroundVertexCirculator(const HalfEdgeIndex& idx_outgoing_half_edge) const
  {
    assert(this->isValid(idx_outgoing_half_edge));
    return (VertexAroundVertexCirculator(idx_outgoing_half_edge, this));
  }

  /** \see pcl::geometry::OutgoingHalfEdgeAroundVertexCirculator */
  inline OutgoingHalfEdgeAroundVertexCirculator
  getOutgoingHalfEdgeAroundVertexCirculator(const VertexIndex& idx_vertex) const
  {
    assert(this->isValid(idx_vertex));
    return (OutgoingHalfEdgeAroundVertexCirculator(idx_vertex, this));
  }

  /** \see pcl::geometry::OutgoingHalfEdgeAroundVertexCirculator */
  inline OutgoingHalfEdgeAroundVertexCirculator
  getOutgoingHalfEdgeAroundVertexCirculator(
      const HalfEdgeIndex& idx_outgoing_half_edge) const
  {
    assert(this->isValid(idx_outgoing_half_edge));
    return (OutgoingHalfEdgeAroundVertexCirculator(idx_outgoing_half_edge, this));
  }

  /** \see pcl::geometry::IncomingHalfEdgeAroundVertexCirculator */
  inline IncomingHalfEdgeAroundVertexCirculator
  getIncomingHalfEdgeAroundVertexCirculator(const VertexIndex& idx_vertex) const
  {
    assert(this->isValid(idx_vertex));
    return (IncomingHalfEdgeAroundVertexCirculator(idx_vertex, this));
  }

  /** \see pcl::geometry::IncomingHalfEdgeAroundVertexCirculator */
  inline IncomingHalfEdgeAroundVertexCirculator
  getIncomingHalfEdgeAroundVertexCirculator(
      const HalfEdgeIndex& idx_incoming_half_edge) const
  {
    assert(this->isValid(idx_incoming_half_edge));
    return (IncomingHalfEdgeAroundVertexCirculator(idx_incoming_half_edge, this));
  }

  /** \see pcl::geometry::FaceAroundVertexCirculator */
  inline FaceAroundVertexCirculator
  getFaceAroundVertexCirculator(const VertexIndex& idx_vertex) const
  {
    assert(this->isValid(idx_vertex));
    return (FaceAroundVertexCirculator(idx_vertex, this));
  }

  /** \see pcl::geometry::FaceAroundVertexCirculator */
  inline FaceAroundVertexCirculator
  getFaceAroundVertexCirculator(const HalfEdgeIndex& idx_outgoing_half_edge) const
  {
    assert(this->isValid(idx_outgoing_half_edge));
    return (FaceAroundVertexCirculator(idx_outgoing_half_edge, this));
  }

  /** \see pcl::geometry::VertexAroundFaceCirculator */
  inline VertexAroundFaceCirculator
  getVertexAroundFaceCirculator(const FaceIndex& idx_face) const
  {
    assert(this->isValid(idx_face));
    return (VertexAroundFaceCirculator(idx_face, this));
  }

  /** \see pcl::geometry::VertexAroundFaceCirculator */
  inline VertexAroundFaceCirculator
  getVertexAroundFaceCirculator(const HalfEdgeIndex& idx_inner_half_edge) const
  {
    assert(this->isValid(idx_inner_half_edge));
    return (VertexAroundFaceCirculator(idx_inner_half_edge, this));
  }

  /** \see pcl::geometry::InnerHalfEdgeAroundFaceCirculator */
  inline InnerHalfEdgeAroundFaceCirculator
  getInnerHalfEdgeAroundFaceCirculator(const FaceIndex& idx_face) const
  {
    assert(this->isValid(idx_face));
    return (InnerHalfEdgeAroundFaceCirculator(idx_face, this));
  }

  /** \see pcl::geometry::InnerHalfEdgeAroundFaceCirculator */
  inline InnerHalfEdgeAroundFaceCirculator
  getInnerHalfEdgeAroundFaceCirculator(const HalfEdgeIndex& idx_inner_half_edge) const
  {
    assert(this->isValid(idx_inner_half_edge));
    return (InnerHalfEdgeAroundFaceCirculator(idx_inner_half_edge, this));
  }

  /** \see pcl::geometry::OuterHalfEdgeAroundFaceCirculator */
  inline OuterHalfEdgeAroundFaceCirculator
  getOuterHalfEdgeAroundFaceCirculator(const FaceIndex& idx_face) const
  {
    assert(this->isValid(idx_face));
    return (OuterHalfEdgeAroundFaceCirculator(idx_face, this));
  }

  /** \see pcl::geometry::OuterHalfEdgeAroundFaceCirculator */
  inline OuterHalfEdgeAroundFaceCirculator
  getOuterHalfEdgeAroundFaceCirculator(const HalfEdgeIndex& idx_inner_half_edge) const
  {
    assert(this->isValid(idx_inner_half_edge));
    return (OuterHalfEdgeAroundFaceCirculator(idx_inner_half_edge, this));
  }

  /** \see pcl::geometry::FaceAroundFaceCirculator */
  inline FaceAroundFaceCirculator
  getFaceAroundFaceCirculator(const FaceIndex& idx_face) const
  {
    assert(this->isValid(idx_face));
    return (FaceAroundFaceCirculator(idx_face, this));
  }

  /** \see pcl::geometry::FaceAroundFaceCirculator */
  inline FaceAroundFaceCirculator
  getFaceAroundFaceCirculator(const HalfEdgeIndex& idx_inner_half_edge) const
  {
    assert(this->isValid(idx_inner_half_edge));
    return (FaceAroundFaceCirculator(idx_inner_half_edge, this));
  }

  //////////////////////////////////////////////////////////////////////////
  // isEqualTopology
  //////////////////////////////////////////////////////////////////////////

  /** \brief Check if the other mesh has the same topology as this mesh. */
  bool
  isEqualTopology(const Self& other) const
  {
    if (this->sizeVertices() != other.sizeVertices())
      return (false);
    if (this->sizeHalfEdges() != other.sizeHalfEdges())
      return (false);
    if (this->sizeFaces() != other.sizeFaces())
      return (false);

    for (std::size_t i = 0; i < this->sizeVertices(); ++i) {
      if (this->getOutgoingHalfEdgeIndex(VertexIndex(i)) !=
          other.getOutgoingHalfEdgeIndex(VertexIndex(i)))
        return (false);
    }

    for (std::size_t i = 0; i < this->sizeHalfEdges(); ++i) {
      if (this->getTerminatingVertexIndex(HalfEdgeIndex(i)) !=
          other.getTerminatingVertexIndex(HalfEdgeIndex(i)))
        return (false);

      if (this->getNextHalfEdgeIndex(HalfEdgeIndex(i)) !=
          other.getNextHalfEdgeIndex(HalfEdgeIndex(i)))
        return (false);

      if (this->getPrevHalfEdgeIndex(HalfEdgeIndex(i)) !=
          other.getPrevHalfEdgeIndex(HalfEdgeIndex(i)))
        return (false);

      if (this->getFaceIndex(HalfEdgeIndex(i)) != other.getFaceIndex(HalfEdgeIndex(i)))
        return (false);
    }

    for (std::size_t i = 0; i < this->sizeFaces(); ++i) {
      if (this->getInnerHalfEdgeIndex(FaceIndex(i)) !=
          other.getInnerHalfEdgeIndex(FaceIndex(i)))
        return (false);
    }

    return (true);
  }

  ////////////////////////////////////////////////////////////////////////
  // isValid
  ////////////////////////////////////////////////////////////////////////

  /** \brief Check if the given vertex index is a valid index into the mesh. */
  inline bool
  isValid(const VertexIndex& idx_vertex) const
  {
    return (idx_vertex >= VertexIndex(0) &&
            idx_vertex < VertexIndex(int(vertices_.size())));
  }

  /** \brief Check if the given half-edge index is a valid index into the mesh.  */
  inline bool
  isValid(const HalfEdgeIndex& idx_he) const
  {
    return (idx_he >= HalfEdgeIndex(0) && idx_he < HalfEdgeIndex(half_edges_.size()));
  }

  /** \brief Check if the given edge index is a valid index into the mesh. */
  inline bool
  isValid(const EdgeIndex& idx_edge) const
  {
    return (idx_edge >= EdgeIndex(0) && idx_edge < EdgeIndex(half_edges_.size() / 2));
  }

  /** \brief Check if the given face index is a valid index into the mesh.  */
  inline bool
  isValid(const FaceIndex& idx_face) const
  {
    return (idx_face >= FaceIndex(0) && idx_face < FaceIndex(faces_.size()));
  }

  ////////////////////////////////////////////////////////////////////////
  // isDeleted
  ////////////////////////////////////////////////////////////////////////

  /** \brief Check if the given vertex is marked as deleted. */
  inline bool
  isDeleted(const VertexIndex& idx_vertex) const
  {
    assert(this->isValid(idx_vertex));
    return (!this->getOutgoingHalfEdgeIndex(idx_vertex).isValid());
  }

  /** \brief Check if the given half-edge is marked as deleted. */
  inline bool
  isDeleted(const HalfEdgeIndex& idx_he) const
  {
    assert(this->isValid(idx_he));
    return (!this->getTerminatingVertexIndex(idx_he).isValid());
  }

  /** \brief Check if the given edge (any of the two half-edges) is marked as deleted.
   */
  inline bool
  isDeleted(const EdgeIndex& idx_edge) const
  {
    assert(this->isValid(idx_edge));
    return (this->isDeleted(pcl::geometry::toHalfEdgeIndex(idx_edge, true)) ||
            this->isDeleted(pcl::geometry::toHalfEdgeIndex(idx_edge, false)));
  }

  /** \brief Check if the given face is marked as deleted. */
  inline bool
  isDeleted(const FaceIndex& idx_face) const
  {
    assert(this->isValid(idx_face));
    return (!this->getInnerHalfEdgeIndex(idx_face).isValid());
  }

  ////////////////////////////////////////////////////////////////////////
  // isIsolated
  ////////////////////////////////////////////////////////////////////////

  /** \brief Check if the given vertex is isolated (not connected to other elements). */
  inline bool
  isIsolated(const VertexIndex& idx_vertex) const
  {
    assert(this->isValid(idx_vertex));
    return (!this->getOutgoingHalfEdgeIndex(idx_vertex).isValid());
  }

  ////////////////////////////////////////////////////////////////////////
  // isBoundary
  ////////////////////////////////////////////////////////////////////////

  /** \brief Check if the given vertex lies on the boundary. Isolated vertices are
   * considered to be on the boundary. */
  inline bool
  isBoundary(const VertexIndex& idx_vertex) const
  {
    assert(this->isValid(idx_vertex));
    if (this->isIsolated(idx_vertex))
      return (true);
    return (this->isBoundary(this->getOutgoingHalfEdgeIndex(idx_vertex)));
  }

  /** \brief Check if the given half-edge lies on the bounddary. */
  inline bool
  isBoundary(const HalfEdgeIndex& idx_he) const
  {
    assert(this->isValid(idx_he));
    return (!this->getFaceIndex(idx_he).isValid());
  }

  /** \brief Check if the given edge lies on the boundary (any of the two half-edges
   * lies on the boundary. */
  inline bool
  isBoundary(const EdgeIndex& idx_edge) const
  {
    assert(this->isValid(idx_edge));
    const HalfEdgeIndex& idx = pcl::geometry::toHalfEdgeIndex(idx_edge);
    return (this->isBoundary(idx) ||
            this->isBoundary(this->getOppositeHalfEdgeIndex(idx)));
  }

  /**
   * \brief Check if the given face lies on the boundary. There are two versions of
   * this method, selected by the template parameter.
   * \tparam CheckVerticesT Check if any vertex lies on the boundary (true) or
   * check if any edge lies on the boundary (false).
   */
  template <bool CheckVerticesT>
  inline bool
  isBoundary(const FaceIndex& idx_face) const
  {
    assert(this->isValid(idx_face));
    return (this->isBoundary(idx_face, std::integral_constant<bool, CheckVerticesT>()));
  }

  /** \brief Check if the given face lies on the boundary. This method uses isBoundary
   * \c true which checks if any vertex lies on the boundary. */
  inline bool
  isBoundary(const FaceIndex& idx_face) const
  {
    assert(this->isValid(idx_face));
    return (this->isBoundary(idx_face, std::true_type()));
  }

  ////////////////////////////////////////////////////////////////////////
  // isManifold
  ////////////////////////////////////////////////////////////////////////

  /** \brief Check if the given vertex is manifold. Isolated vertices are manifold. */
  inline bool
  isManifold(const VertexIndex& idx_vertex) const
  {
    assert(this->isValid(idx_vertex));
    if (this->isIsolated(idx_vertex))
      return (true);
    return (this->isManifold(idx_vertex, IsManifold()));
  }

  /** \brief Check if the mesh is manifold. */
  inline bool
  isManifold() const
  {
    return (this->isManifold(IsManifold()));
  }

  ////////////////////////////////////////////////////////////////////////
  // size
  ////////////////////////////////////////////////////////////////////////

  /** \brief Get the number of the vertices. */
  inline std::size_t
  sizeVertices() const
  {
    return (vertices_.size());
  }

  /** \brief Get the number of the half-edges. */
  inline std::size_t
  sizeHalfEdges() const
  {
    assert(half_edges_.size() % 2 == 0); // This would be a bug in the mesh.
    return (half_edges_.size());
  }

  /** \brief Get the number of the edges. */
  inline std::size_t
  sizeEdges() const
  {
    assert(half_edges_.size() % 2 == 0); // This would be a bug in the mesh.
    return (half_edges_.size() / 2);
  }

  /** \brief Get the number of the faces. */
  inline std::size_t
  sizeFaces() const
  {
    return (faces_.size());
  }

  ////////////////////////////////////////////////////////////////////////
  // empty
  ////////////////////////////////////////////////////////////////////////

  /** \brief Check if the mesh is empty. */
  inline bool
  empty() const
  {
    return (this->emptyVertices() && this->emptyEdges() && this->emptyFaces());
  }

  /** \brief Check if the vertices are empty. */
  inline bool
  emptyVertices() const
  {
    return (vertices_.empty());
  }

  /** \brief Check if the edges are empty. */
  inline bool
  emptyEdges() const
  {
    return (half_edges_.empty());
  }

  /** \brief Check if the faces are empty. */
  inline bool
  emptyFaces() const
  {
    return (faces_.empty());
  }

  ////////////////////////////////////////////////////////////////////////
  // reserve
  ////////////////////////////////////////////////////////////////////////

  /** \brief Reserve storage space n vertices. */
  inline void
  reserveVertices(const std::size_t n)
  {
    vertices_.reserve(n);
    this->reserveData(vertex_data_cloud_, n, HasVertexData());
  }

  /** \brief Reserve storage space for n edges (2*n storage space is reserved for the
   * half-edges). */
  inline void
  reserveEdges(const std::size_t n)
  {
    half_edges_.reserve(2 * n);
    this->reserveData(half_edge_data_cloud_, 2 * n, HasHalfEdgeData());
    this->reserveData(edge_data_cloud_, n, HasEdgeData());
  }

  /** \brief Reserve storage space for n faces. */
  inline void
  reserveFaces(const std::size_t n)
  {
    faces_.reserve(n);
    this->reserveData(face_data_cloud_, n, HasFaceData());
  }

  ////////////////////////////////////////////////////////////////////////
  // resize
  ////////////////////////////////////////////////////////////////////////

  /** \brief Resize the the vertices to n elements. */
  inline void
  resizeVertices(const std::size_t n, const VertexData& data = VertexData())
  {
    vertices_.resize(n, Vertex());
    this->resizeData(vertex_data_cloud_, n, data, HasVertexData());
  }

  /** \brief Resize the edges to n elements (half-edges will hold 2*n elements). */
  inline void
  resizeEdges(const std::size_t n,
              const EdgeData& edge_data = EdgeData(),
              const HalfEdgeData he_data = HalfEdgeData())
  {
    half_edges_.resize(2 * n, HalfEdge());
    this->resizeData(half_edge_data_cloud_, 2 * n, he_data, HasHalfEdgeData());
    this->resizeData(edge_data_cloud_, n, edge_data, HasEdgeData());
  }

  /** \brief Resize the faces to n elements. */
  inline void
  resizeFaces(const std::size_t n, const FaceData& data = FaceData())
  {
    faces_.resize(n, Face());
    this->resizeData(face_data_cloud_, n, data, HasFaceData());
  }

  ////////////////////////////////////////////////////////////////////////
  // clear
  ////////////////////////////////////////////////////////////////////////

  /** \brief Clear all mesh elements and data. */
  void
  clear()
  {
    vertices_.clear();
    half_edges_.clear();
    faces_.clear();

    this->clearData(vertex_data_cloud_, HasVertexData());
    this->clearData(half_edge_data_cloud_, HasHalfEdgeData());
    this->clearData(edge_data_cloud_, HasEdgeData());
    this->clearData(face_data_cloud_, HasFaceData());
  }

  ////////////////////////////////////////////////////////////////////////
  // get / set the vertex data cloud
  ////////////////////////////////////////////////////////////////////////

  /** \brief Get access to the stored vertex data.
   * \warning Please make sure to NOT add or remove elements from the cloud.
   */
  inline VertexDataCloud&
  getVertexDataCloud()
  {
    return (vertex_data_cloud_);
  }

  /** \brief Get the stored vertex data. */
  inline VertexDataCloud
  getVertexDataCloud() const
  {
    return (vertex_data_cloud_);
  }

  /** \brief Change the stored vertex data.
   * \param[in] vertex_data_cloud The new vertex data. Must be the same as the current
   * data.
   * \return true if the cloud could be set.
   */
  inline bool
  setVertexDataCloud(const VertexDataCloud& vertex_data_cloud)
  {
    if (vertex_data_cloud.size() == vertex_data_cloud_.size()) {
      vertex_data_cloud_ = vertex_data_cloud;
      return (true);
    }
    return (false);
  }

  ////////////////////////////////////////////////////////////////////////
  // get / set the half-edge data cloud
  ////////////////////////////////////////////////////////////////////////

  /** \brief Get access to the stored half-edge data.
   * \warning Please make sure to NOT add or remove elements from the cloud.
   */
  inline HalfEdgeDataCloud&
  getHalfEdgeDataCloud()
  {
    return (half_edge_data_cloud_);
  }

  /** \brief Get the stored half-edge data. */
  inline HalfEdgeDataCloud
  getHalfEdgeDataCloud() const
  {
    return (half_edge_data_cloud_);
  }

  /** \brief Change the stored half-edge data.
   * \param[in] half_edge_data_cloud The new half-edge data. Must be the same as the
   * current data.
   * \return true if the cloud could be set.
   */
  inline bool
  setHalfEdgeDataCloud(const HalfEdgeDataCloud& half_edge_data_cloud)
  {
    if (half_edge_data_cloud.size() == half_edge_data_cloud_.size()) {
      half_edge_data_cloud_ = half_edge_data_cloud;
      return (true);
    }
    return (false);
  }

  ////////////////////////////////////////////////////////////////////////
  // get / set the edge data cloud
  ////////////////////////////////////////////////////////////////////////

  /** \brief Get access to the stored edge data.
   * \warning Please make sure to NOT add or remove elements from the cloud.
   */
  inline EdgeDataCloud&
  getEdgeDataCloud()
  {
    return (edge_data_cloud_);
  }

  /** \brief Get the stored edge data. */
  inline EdgeDataCloud
  getEdgeDataCloud() const
  {
    return (edge_data_cloud_);
  }

  /** \brief Change the stored edge data.
   * \param[in] edge_data_cloud The new edge data. Must be the same as the current data.
   * \return true if the cloud could be set.
   */
  inline bool
  setEdgeDataCloud(const EdgeDataCloud& edge_data_cloud)
  {
    if (edge_data_cloud.size() == edge_data_cloud_.size()) {
      edge_data_cloud_ = edge_data_cloud;
      return (true);
    }
    return (false);
  }

  ////////////////////////////////////////////////////////////////////////
  // get / set the face data cloud
  ////////////////////////////////////////////////////////////////////////

  /** \brief Get access to the stored face data.
   * \warning Please make sure to NOT add or remove elements from the cloud.
   */
  inline FaceDataCloud&
  getFaceDataCloud()
  {
    return (face_data_cloud_);
  }

  /** \brief Get the stored face data. */
  inline FaceDataCloud
  getFaceDataCloud() const
  {
    return (face_data_cloud_);
  }

  /** \brief Change the stored face data.
   * \param[in] face_data_cloud The new face data. Must be the same as the current data.
   * \return true if the cloud could be set.
   */
  inline bool
  setFaceDataCloud(const FaceDataCloud& face_data_cloud)
  {
    if (face_data_cloud.size() == face_data_cloud_.size()) {
      face_data_cloud_ = face_data_cloud;
      return (true);
    }
    return (false);
  }

  ////////////////////////////////////////////////////////////////////////
  // getVertexIndex / getHalfEdgeIndex / getEdgeIndex / getFaceIndex
  ////////////////////////////////////////////////////////////////////////

  /** \brief Get the index associated to the given vertex data.
   * \return Invalid index if the mesh does not have associated vertex data.
   */
  inline VertexIndex
  getVertexIndex(const VertexData& vertex_data) const
  {
    if (HasVertexData::value) {
      assert(&vertex_data >= &vertex_data_cloud_.front() &&
             &vertex_data <= &vertex_data_cloud_.back());
      return (VertexIndex(std::distance(&vertex_data_cloud_.front(), &vertex_data)));
    }
    return (VertexIndex());
  }

  /** \brief Get the index associated to the given half-edge data. */
  inline HalfEdgeIndex
  getHalfEdgeIndex(const HalfEdgeData& half_edge_data) const
  {
    if (HasHalfEdgeData::value) {
      assert(&half_edge_data >= &half_edge_data_cloud_.front() &&
             &half_edge_data <= &half_edge_data_cloud_.back());
      return (HalfEdgeIndex(
          std::distance(&half_edge_data_cloud_.front(), &half_edge_data)));
    }
    return (HalfEdgeIndex());
  }

  /** \brief Get the index associated to the given edge data. */
  inline EdgeIndex
  getEdgeIndex(const EdgeData& edge_data) const
  {
    if (HasEdgeData::value) {
      assert(&edge_data >= &edge_data_cloud_.front() &&
             &edge_data <= &edge_data_cloud_.back());
      return (EdgeIndex(std::distance(&edge_data_cloud_.front(), &edge_data)));
    }
    return (EdgeIndex());
  }

  /** \brief Get the index associated to the given face data. */
  inline FaceIndex
  getFaceIndex(const FaceData& face_data) const
  {
    if (HasFaceData::value) {
      assert(&face_data >= &face_data_cloud_.front() &&
             &face_data <= &face_data_cloud_.back());
      return (FaceIndex(std::distance(&face_data_cloud_.front(), &face_data)));
    }
    return (FaceIndex());
  }

protected:
  ////////////////////////////////////////////////////////////////////////
  // Types
  ////////////////////////////////////////////////////////////////////////

  // Elements
  using Vertex = pcl::geometry::Vertex;
  using HalfEdge = pcl::geometry::HalfEdge;
  using Face = pcl::geometry::Face;

  using Vertices = std::vector<Vertex>;
  using HalfEdges = std::vector<HalfEdge>;
  using Faces = std::vector<Face>;

  using VertexIterator = typename Vertices::iterator;
  using HalfEdgeIterator = typename HalfEdges::iterator;
  using FaceIterator = typename Faces::iterator;

  using VertexConstIterator = typename Vertices::const_iterator;
  using HalfEdgeConstIterator = typename HalfEdges::const_iterator;
  using FaceConstIterator = typename Faces::const_iterator;

  /** \brief General implementation of addFace. */
  FaceIndex
  addFaceImplBase(const VertexIndices& vertices,
                  const FaceData& face_data,
                  const EdgeData& edge_data,
                  const HalfEdgeData& half_edge_data)
  {
    const int n = static_cast<int>(vertices.size());
    if (n < 3)
      return (FaceIndex());

    // Check for topological errors
    inner_he_.resize(n);
    free_he_.resize(n);
    is_new_.resize(n);
    make_adjacent_.resize(n);
    for (int i = 0; i < n; ++i) {
      if (!this->checkTopology1(vertices[i],
                                vertices[(i + 1) % n],
                                inner_he_[i],
                                is_new_[i],
                                IsManifold())) {
        return (FaceIndex());
      }
    }
    for (int i = 0; i < n; ++i) {
      int j = (i + 1) % n;
      if (!this->checkTopology2(inner_he_[i],
                                inner_he_[j],
                                is_new_[i],
                                is_new_[j],
                                this->isIsolated(vertices[j]),
                                make_adjacent_[i],
                                free_he_[i],
                                IsManifold())) {
        return (FaceIndex());
      }
    }

    // Reconnect the existing half-edges if needed
    if (!IsManifold::value) {
      for (int i = 0; i < n; ++i) {
        if (make_adjacent_[i]) {
          this->makeAdjacent(inner_he_[i], inner_he_[(i + 1) % n], free_he_[i]);
        }
      }
    }

    // Add new half-edges if needed
    for (int i = 0; i < n; ++i) {
      if (is_new_[i]) {
        inner_he_[i] = this->addEdge(
            vertices[i], vertices[(i + 1) % n], half_edge_data, edge_data);
      }
    }

    // Connect
    for (int i = 0; i < n; ++i) {
      int j = (i + 1) % n;
      if (is_new_[i] && is_new_[j])
        this->connectNewNew(inner_he_[i], inner_he_[j], vertices[j], IsManifold());
      else if (is_new_[i] && !is_new_[j])
        this->connectNewOld(inner_he_[i], inner_he_[j], vertices[j]);
      else if (!is_new_[i] && is_new_[j])
        this->connectOldNew(inner_he_[i], inner_he_[j], vertices[j]);
      else
        this->connectOldOld(inner_he_[i], inner_he_[j], vertices[j], IsManifold());
    }
    return (this->connectFace(inner_he_, face_data));
  }

  ////////////////////////////////////////////////////////////////////////
  // addEdge
  ////////////////////////////////////////////////////////////////////////

  /** \brief Add an edge between the two given vertices and connect them with the
   * vertices.
   * \param[in]  idx_v_a   The first vertex index
   * \param[in]  idx_v_b   The second vertex index
   * \param[in]  he_data   Data associated with the half-edges. This is only added if
   * the mesh has data associated with the half-edges.
   * \param[in] edge_data Data associated with the edge. This is only added if the mesh
   * has data associated with the edges.
   * \return Index to the half-edge from vertex a to vertex b.
   */
  HalfEdgeIndex
  addEdge(const VertexIndex& idx_v_a,
          const VertexIndex& idx_v_b,
          const HalfEdgeData& he_data,
          const EdgeData& edge_data)
  {
    half_edges_.push_back(HalfEdge(idx_v_b));
    half_edges_.push_back(HalfEdge(idx_v_a));

    this->addData(half_edge_data_cloud_, he_data, HasHalfEdgeData());
    this->addData(half_edge_data_cloud_, he_data, HasHalfEdgeData());
    this->addData(edge_data_cloud_, edge_data, HasEdgeData());

    return (HalfEdgeIndex(static_cast<int>(half_edges_.size() - 2)));
  }

  ////////////////////////////////////////////////////////////////////////
  // topology checks
  ////////////////////////////////////////////////////////////////////////

  /** \brief Check if the edge between the two vertices can be added.
   * \param[in]  idx_v_a   Index to the first vertex.
   * \param[in]  idx_v_b   Index to the second vertex.
   * \param[out] idx_he_ab Index to the half-edge ab if is_new_ab=false.
   * \param[out] is_new_ab true if the edge between the vertices exists already. Must be
   * initialized with true!
   * \return true if the half-edge may be added.
   */
  bool
  checkTopology1(const VertexIndex& idx_v_a,
                 const VertexIndex& idx_v_b,
                 HalfEdgeIndex& idx_he_ab,
                 std::vector<bool>::reference is_new_ab,
                 std::true_type /*is_manifold*/) const
  {
    is_new_ab = true;
    if (this->isIsolated(idx_v_a))
      return (true);

    idx_he_ab = this->getOutgoingHalfEdgeIndex(idx_v_a);

    if (!this->isBoundary(idx_he_ab))
      return (false);
    if (this->getTerminatingVertexIndex(idx_he_ab) == idx_v_b)
      is_new_ab = false;
    return (true);
  }

  /** \brief Non manifold version of checkTopology1 */
  bool
  checkTopology1(const VertexIndex& idx_v_a,
                 const VertexIndex& idx_v_b,
                 HalfEdgeIndex& idx_he_ab,
                 std::vector<bool>::reference is_new_ab,
                 std::false_type /*is_manifold*/) const
  {
    is_new_ab = true;
    if (this->isIsolated(idx_v_a))
      return (true);
    if (!this->isBoundary(this->getOutgoingHalfEdgeIndex(idx_v_a)))
      return (false);

    VertexAroundVertexCirculator circ =
        this->getVertexAroundVertexCirculator(this->getOutgoingHalfEdgeIndex(idx_v_a));
    const VertexAroundVertexCirculator circ_end = circ;

    do {
      if (circ.getTargetIndex() == idx_v_b) {
        idx_he_ab = circ.getCurrentHalfEdgeIndex();
        if (!this->isBoundary(idx_he_ab))
          return (false);

        is_new_ab = false;
        return (true);
      }
    } while (++circ != circ_end);

    return (true);
  }

  /** \brief Check if the face may be added (mesh does not become non-manifold). */
  inline bool
  checkTopology2(const HalfEdgeIndex& /*idx_he_ab*/,
                 const HalfEdgeIndex& /*idx_he_bc*/,
                 const bool is_new_ab,
                 const bool is_new_bc,
                 const bool is_isolated_b,
                 std::vector<bool>::reference /*make_adjacent_ab_bc*/,
                 HalfEdgeIndex& /*idx_free_half_edge*/,
                 std::true_type /*is_manifold*/) const
  {
    return !(is_new_ab && is_new_bc && !is_isolated_b);
  }

  /** \brief Check if the half-edge bc is the next half-edge of ab.
   * \param[in]  idx_he_ab Index to the half-edge between the vertices a and b.
   * \param[in]  idx_he_bc Index to the half-edge between the vertices b and c.
   * \param[in]  is_new_ab Half-edge ab is new.
   * \param[in]  is_new_bc Half-edge bc is new.
   * \param[out] make_adjacent_ab_bc Half-edges ab and bc need to be made adjacent.
   * \param[out] idx_free_half_edge  Free half-edge (needed for makeAdjacent)
   * \return true if addFace may be continued.
   */
  inline bool
  checkTopology2(const HalfEdgeIndex& idx_he_ab,
                 const HalfEdgeIndex& idx_he_bc,
                 const bool is_new_ab,
                 const bool is_new_bc,
                 const bool /*is_isolated_b*/,
                 std::vector<bool>::reference make_adjacent_ab_bc,
                 HalfEdgeIndex& idx_free_half_edge,
                 std::false_type /*is_manifold*/) const
  {
    if (is_new_ab || is_new_bc) {
      make_adjacent_ab_bc = false;
      return (true); // Make adjacent is only needed for two old half-edges
    }

    if (this->getNextHalfEdgeIndex(idx_he_ab) == idx_he_bc) {
      make_adjacent_ab_bc = false;
      return (true); // already adjacent
    }

    make_adjacent_ab_bc = true;

    // Find the next boundary half edge
    IncomingHalfEdgeAroundVertexCirculator circ =
        this->getIncomingHalfEdgeAroundVertexCirculator(
            this->getOppositeHalfEdgeIndex(idx_he_bc));

    do
      ++circ;
    while (!this->isBoundary(circ.getTargetIndex()));
    idx_free_half_edge = circ.getTargetIndex();

    // This would detach the faces around the vertex from each other.
    return (circ.getTargetIndex() != idx_he_ab);
  }

  /** \brief Make the half-edges bc the next half-edge of ab.
   * \param[in]      idx_he_ab          Index to the half-edge between the vertices a
   * and b.
   * \param[in]      idx_he_bc          Index to the half-edge between the
   * vertices b and c.
   * \param[in, out] idx_free_half_edge Free half-edge needed to re-connect the
   * half-edges around vertex b.
   */
  void
  makeAdjacent(const HalfEdgeIndex& idx_he_ab,
               const HalfEdgeIndex& idx_he_bc,
               HalfEdgeIndex& idx_free_half_edge)
  {
    // Re-link. No references!
    const HalfEdgeIndex idx_he_ab_next = this->getNextHalfEdgeIndex(idx_he_ab);
    const HalfEdgeIndex idx_he_bc_prev = this->getPrevHalfEdgeIndex(idx_he_bc);
    const HalfEdgeIndex idx_he_free_next =
        this->getNextHalfEdgeIndex(idx_free_half_edge);

    this->connectPrevNext(idx_he_ab, idx_he_bc);
    this->connectPrevNext(idx_free_half_edge, idx_he_ab_next);
    this->connectPrevNext(idx_he_bc_prev, idx_he_free_next);
  }

  ////////////////////////////////////////////////////////////////////////
  // connect
  ////////////////////////////////////////////////////////////////////////

  /** \brief Add a face to the mesh and connect it to the half-edges.
   * \param[in] inner_he  Inner half-edges of the face.
   * \param[in] face_data Data that is stored in the face. This is only added if the
   * mesh has data associated with the faces.
   * \return Index to the new face.
   */
  FaceIndex
  connectFace(const HalfEdgeIndices& inner_he, const FaceData& face_data)
  {
    faces_.push_back(Face(inner_he.back()));
    this->addData(face_data_cloud_, face_data, HasFaceData());

    const FaceIndex idx_face(static_cast<int>(this->sizeFaces() - 1));

    for (const auto& idx_half_edge : inner_he) {
      this->setFaceIndex(idx_half_edge, idx_face);
    }

    return (idx_face);
  }

  /** \brief Connect the next and prev indices of the two half-edges with each other. */
  inline void
  connectPrevNext(const HalfEdgeIndex& idx_he_ab, const HalfEdgeIndex& idx_he_bc)
  {
    this->setNextHalfEdgeIndex(idx_he_ab, idx_he_bc);
    this->setPrevHalfEdgeIndex(idx_he_bc, idx_he_ab);
  }

  /** \brief Both half-edges are new (manifold version). */
  void
  connectNewNew(const HalfEdgeIndex& idx_he_ab,
                const HalfEdgeIndex& idx_he_bc,
                const VertexIndex& idx_v_b,
                std::true_type /*is_manifold*/)
  {
    const HalfEdgeIndex idx_he_ba = this->getOppositeHalfEdgeIndex(idx_he_ab);
    const HalfEdgeIndex idx_he_cb = this->getOppositeHalfEdgeIndex(idx_he_bc);

    this->connectPrevNext(idx_he_ab, idx_he_bc);
    this->connectPrevNext(idx_he_cb, idx_he_ba);

    this->setOutgoingHalfEdgeIndex(idx_v_b, idx_he_ba);
  }

  /** \brief Both half-edges are new (non-manifold version). */
  void
  connectNewNew(const HalfEdgeIndex& idx_he_ab,
                const HalfEdgeIndex& idx_he_bc,
                const VertexIndex& idx_v_b,
                std::false_type /*is_manifold*/)
  {
    if (this->isIsolated(idx_v_b)) {
      this->connectNewNew(idx_he_ab, idx_he_bc, idx_v_b, std::true_type());
    }
    else {
      const HalfEdgeIndex idx_he_ba = this->getOppositeHalfEdgeIndex(idx_he_ab);
      const HalfEdgeIndex idx_he_cb = this->getOppositeHalfEdgeIndex(idx_he_bc);

      // No references!
      const HalfEdgeIndex idx_he_b_out = this->getOutgoingHalfEdgeIndex(idx_v_b);
      const HalfEdgeIndex idx_he_b_out_prev = this->getPrevHalfEdgeIndex(idx_he_b_out);

      this->connectPrevNext(idx_he_ab, idx_he_bc);
      this->connectPrevNext(idx_he_cb, idx_he_b_out);
      this->connectPrevNext(idx_he_b_out_prev, idx_he_ba);
    }
  }

  /** \brief The first half-edge is new. */
  void
  connectNewOld(const HalfEdgeIndex& idx_he_ab,
                const HalfEdgeIndex& idx_he_bc,
                const VertexIndex& idx_v_b)
  {
    const HalfEdgeIndex idx_he_ba = this->getOppositeHalfEdgeIndex(idx_he_ab);
    const HalfEdgeIndex idx_he_bc_prev =
        this->getPrevHalfEdgeIndex(idx_he_bc); // No reference!

    this->connectPrevNext(idx_he_ab, idx_he_bc);
    this->connectPrevNext(idx_he_bc_prev, idx_he_ba);

    this->setOutgoingHalfEdgeIndex(idx_v_b, idx_he_ba);
  }

  /** \brief The second half-edge is new. */
  void
  connectOldNew(const HalfEdgeIndex& idx_he_ab,
                const HalfEdgeIndex& idx_he_bc,
                const VertexIndex& idx_v_b)
  {
    const HalfEdgeIndex idx_he_cb = this->getOppositeHalfEdgeIndex(idx_he_bc);
    const HalfEdgeIndex idx_he_ab_next =
        this->getNextHalfEdgeIndex(idx_he_ab); // No reference!

    this->connectPrevNext(idx_he_ab, idx_he_bc);
    this->connectPrevNext(idx_he_cb, idx_he_ab_next);

    this->setOutgoingHalfEdgeIndex(idx_v_b, idx_he_ab_next);
  }

  /** \brief Both half-edges are old (manifold version). */
  void
  connectOldOld(const HalfEdgeIndex& /*idx_he_ab*/,
                const HalfEdgeIndex& /*idx_he_bc*/,
                const VertexIndex& /*idx_v_b*/,
                std::true_type /*is_manifold*/)
  {}

  /** \brief Both half-edges are old (non-manifold version). */
  void
  connectOldOld(const HalfEdgeIndex& /*idx_he_ab*/,
                const HalfEdgeIndex& idx_he_bc,
                const VertexIndex& idx_v_b,
                std::false_type /*is_manifold*/)
  {
    const HalfEdgeIndex& idx_he_b_out = this->getOutgoingHalfEdgeIndex(idx_v_b);

    // The outgoing half edge MUST be a boundary half-edge (if there is one)
    if (idx_he_b_out == idx_he_bc) // he_bc is no longer on the boundary
    {
      OutgoingHalfEdgeAroundVertexCirculator circ =
          this->getOutgoingHalfEdgeAroundVertexCirculator(idx_he_b_out);
      const OutgoingHalfEdgeAroundVertexCirculator circ_end = circ;

      while (++circ != circ_end) {
        if (this->isBoundary(circ.getTargetIndex())) {
          this->setOutgoingHalfEdgeIndex(idx_v_b, circ.getTargetIndex());
          return;
        }
      }
    }
  }

  ////////////////////////////////////////////////////////////////////////
  // addData
  ////////////////////////////////////////////////////////////////////////

  /** \brief Add mesh data. */
  template <class DataT>
  inline void
  addData(pcl::PointCloud<DataT>& cloud, const DataT& data, std::true_type /*has_data*/)
  {
    cloud.push_back(data);
  }

  /** \brief Does nothing. */
  template <class DataT>
  inline void
  addData(pcl::PointCloud<DataT>& /*cloud*/,
          const DataT& /*data*/,
          std::false_type /*has_data*/)
  {}

  ////////////////////////////////////////////////////////////////////////
  // deleteFace
  ////////////////////////////////////////////////////////////////////////

  /** \brief Manifold version of deleteFace. If the mesh becomes non-manifold due to the
   * delete operation the faces around the non-manifold vertex are deleted until the
   * mesh becomes manifold again. */
  void
  deleteFace(const FaceIndex& idx_face, std::true_type /*is_manifold*/)
  {
    assert(this->isValid(idx_face));
    delete_faces_face_.clear();
    delete_faces_face_.push_back(idx_face);

    while (!delete_faces_face_.empty()) {
      const FaceIndex idx_face_cur = delete_faces_face_.back();
      delete_faces_face_.pop_back();

      // This calls the non-manifold version of deleteFace, which will call the manifold
      // version of reconnect.
      this->deleteFace(idx_face_cur, std::false_type());
    }
  }

  /** \brief Non-manifold version of deleteFace. */
  void
  deleteFace(const FaceIndex& idx_face, std::false_type /*is_manifold*/)
  {
    assert(this->isValid(idx_face));
    if (this->isDeleted(idx_face))
      return;

    // Store all half-edges in the face
    inner_he_.clear();
    is_boundary_.clear();
    InnerHalfEdgeAroundFaceCirculator circ =
        this->getInnerHalfEdgeAroundFaceCirculator(idx_face);
    const InnerHalfEdgeAroundFaceCirculator circ_end = circ;
    do {
      inner_he_.push_back(circ.getTargetIndex());
      is_boundary_.push_back(
          this->isBoundary(this->getOppositeHalfEdgeIndex(circ.getTargetIndex())));
    } while (++circ != circ_end);
    assert(inner_he_.size() >= 3); // Minimum should be a triangle.

    const int n = static_cast<int>(inner_he_.size());
    int j;

    if (IsManifold::value) {
      for (int i = 0; i < n; ++i) {
        j = (i + 1) % n;
        this->reconnect(inner_he_[i], inner_he_[j], is_boundary_[i], is_boundary_[j]);
      }
      for (int i = 0; i < n; ++i) {
        this->getHalfEdge(inner_he_[i]).idx_face_.invalidate();
      }
    }
    else {
      for (int i = 0; i < n; ++i) {
        j = (i + 1) % n;
        this->reconnect(inner_he_[i], inner_he_[j], is_boundary_[i], is_boundary_[j]);
        this->getHalfEdge(inner_he_[i]).idx_face_.invalidate();
      }
    }

    this->markDeleted(idx_face);
  }

  ////////////////////////////////////////////////////////////////////////
  // reconnect
  ////////////////////////////////////////////////////////////////////////

  /** \brief Deconnect the input half-edges from the mesh and adjust the indices of the
   * connected half-edges. */
  void
  reconnect(const HalfEdgeIndex& idx_he_ab,
            const HalfEdgeIndex& idx_he_bc,
            const bool is_boundary_ba,
            const bool is_boundary_cb)
  {
    const HalfEdgeIndex idx_he_ba = this->getOppositeHalfEdgeIndex(idx_he_ab);
    const HalfEdgeIndex idx_he_cb = this->getOppositeHalfEdgeIndex(idx_he_bc);
    const VertexIndex idx_v_b = this->getTerminatingVertexIndex(idx_he_ab);

    if (is_boundary_ba && is_boundary_cb) // boundary - boundary
    {
      const HalfEdgeIndex& idx_he_cb_next = this->getNextHalfEdgeIndex(idx_he_cb);

      if (idx_he_cb_next == idx_he_ba) // Vertex b is isolated
      {
        this->markDeleted(idx_v_b);
      }
      else {
        this->connectPrevNext(this->getPrevHalfEdgeIndex(idx_he_ba), idx_he_cb_next);
        this->setOutgoingHalfEdgeIndex(idx_v_b, idx_he_cb_next);
      }

      this->markDeleted(idx_he_ab);
      this->markDeleted(idx_he_ba);
    }
    else if (is_boundary_ba && !is_boundary_cb) // boundary - no boundary
    {
      this->connectPrevNext(this->getPrevHalfEdgeIndex(idx_he_ba), idx_he_bc);
      this->setOutgoingHalfEdgeIndex(idx_v_b, idx_he_bc);

      this->markDeleted(idx_he_ab);
      this->markDeleted(idx_he_ba);
    }
    else if (!is_boundary_ba && is_boundary_cb) // no boundary - boundary
    {
      const HalfEdgeIndex& idx_he_cb_next = this->getNextHalfEdgeIndex(idx_he_cb);
      this->connectPrevNext(idx_he_ab, idx_he_cb_next);
      this->setOutgoingHalfEdgeIndex(idx_v_b, idx_he_cb_next);
    }
    else // no boundary - no boundary
    {
      this->reconnectNBNB(idx_he_bc, idx_he_cb, idx_v_b, IsManifold());
    }
  }

  /** \brief Both edges are not on the boundary. Manifold version. */
  void
  reconnectNBNB(const HalfEdgeIndex& idx_he_bc,
                const HalfEdgeIndex& idx_he_cb,
                const VertexIndex& idx_v_b,
                std::true_type /*is_manifold*/)
  {
    if (this->isBoundary(idx_v_b)) {
      // Deletion of this face makes the mesh non-manifold
      // -> delete the neighboring faces until it is manifold again
      IncomingHalfEdgeAroundVertexCirculator circ =
          this->getIncomingHalfEdgeAroundVertexCirculator(idx_he_cb);

      while (!this->isBoundary(circ.getTargetIndex())) {
        delete_faces_face_.push_back(this->getFaceIndex((circ++).getTargetIndex()));

#ifdef PCL_GEOMETRY_MESH_BASE_TEST_DELETE_FACE_MANIFOLD_2
        if (circ == this->getIncomingHalfEdgeAroundVertexCirculator(
                        idx_he_cb)) // Abort infinity loop
        {
          // In a manifold mesh we can't invalidate the face while reconnecting!
          // See the implementation of
          // deleteFace (const FaceIndex&  idx_face,
          //             std::false_type /*is_manifold*/)
          pcl::geometry::g_pcl_geometry_mesh_base_test_delete_face_manifold_2_success =
              false;
          return;
        }
#endif
      }
    }
    else {
      this->setOutgoingHalfEdgeIndex(idx_v_b, idx_he_bc);
    }
  }

  /** \brief Both edges are not on the boundary. Non-manifold version. */
  void
  reconnectNBNB(const HalfEdgeIndex& idx_he_bc,
                const HalfEdgeIndex& /*idx_he_cb*/,
                const VertexIndex& idx_v_b,
                std::false_type /*is_manifold*/)
  {
    if (!this->isBoundary(idx_v_b)) {
      this->setOutgoingHalfEdgeIndex(idx_v_b, idx_he_bc);
    }
  }

  ////////////////////////////////////////////////////////////////////////
  // markDeleted
  ////////////////////////////////////////////////////////////////////////

  /** \brief Mark the given vertex as deleted. */
  inline void
  markDeleted(const VertexIndex& idx_vertex)
  {
    assert(this->isValid(idx_vertex));
    this->getVertex(idx_vertex).idx_outgoing_half_edge_.invalidate();
  }

  /** \brief Mark the given half-edge as deleted. */
  inline void
  markDeleted(const HalfEdgeIndex& idx_he)
  {
    assert(this->isValid(idx_he));
    this->getHalfEdge(idx_he).idx_terminating_vertex_.invalidate();
  }

  /** \brief Mark the given edge (both half-edges) as deleted. */
  inline void
  markDeleted(const EdgeIndex& idx_edge)
  {
    assert(this->isValid(idx_edge));
    this->markDeleted(pcl::geometry::toHalfEdgeIndex(idx_edge, true));
    this->markDeleted(pcl::geometry::toHalfEdgeIndex(idx_edge, false));
  }

  /** \brief Mark the given face as deleted. */
  inline void
  markDeleted(const FaceIndex& idx_face)
  {
    assert(this->isValid(idx_face));
    this->getFace(idx_face).idx_inner_half_edge_.invalidate();
  }

  ////////////////////////////////////////////////////////////////////////
  // For cleanUp
  ////////////////////////////////////////////////////////////////////////

  /** \brief Removes mesh elements and data that are marked as deleted from the
   * container.
   * \tparam ElementContainerT e.g. std::vector \<Vertex\>
   * \tparam DataContainerT    e.g. std::vector \<VertexData\>
   * \tparam IndexContainerT   e.g.  std::vector \<VertexIndex\>
   * \tparam HasDataT          Integral constant specifying if the mesh has data
   * associated with the elements.
   * \param[in, out] elements Container for the mesh elements. Resized to the new size.
   * \param[in, out] data_cloud Container for the mesh data. Resized to the new size.
   * \return Container with the same size as the old input data. Holds the indices to
   * the new elements for each non-deleted element and an invalid index if it is
   * deleted.
   */
  template <class ElementContainerT,
            class DataContainerT,
            class IndexContainerT,
            class HasDataT>
  IndexContainerT
  remove(ElementContainerT& elements, DataContainerT& data_cloud)
  {
    using Index = typename IndexContainerT::value_type;
    using Element = typename ElementContainerT::value_type;

    if (HasDataT::value)
      assert(elements.size() == data_cloud.size());
    else
      assert(data_cloud.empty()); // Bug in this class!

    IndexContainerT new_indices(elements.size(),
                                typename IndexContainerT::value_type());
    Index ind_old(0), ind_new(0);

    typename ElementContainerT::const_iterator it_e_old = elements.begin();
    auto it_e_new = elements.begin();

    typename DataContainerT::const_iterator it_d_old = data_cloud.begin();
    auto it_d_new = data_cloud.begin();

    auto it_ind_new = new_indices.begin();
    typename IndexContainerT::const_iterator it_ind_new_end = new_indices.end();

    while (it_ind_new != it_ind_new_end) {
      if (!this->isDeleted(ind_old)) {
        *it_ind_new = ind_new++;

        // TODO: Test for self assignment?
        *it_e_new++ = *it_e_old;
        this->assignIf(it_d_old, it_d_new, HasDataT());
        this->incrementIf(it_d_new, HasDataT());
      }
      ++ind_old;
      ++it_e_old;
      this->incrementIf(it_d_old, HasDataT());
      ++it_ind_new;
    }

    elements.resize(ind_new.get(), Element());
    if (HasDataT::value) {
      data_cloud.resize(ind_new.get());
    }
    else if (it_d_old != data_cloud.begin() || it_d_new != data_cloud.begin()) {
      std::cerr << "TODO: Bug in MeshBase::remove!\n";
      assert(false);
      exit(EXIT_FAILURE);
    }

    return (new_indices);
  }

  /** \brief Increment the iterator. */
  template <class IteratorT>
  inline void
  incrementIf(IteratorT& it, std::true_type /*has_data*/) const
  {
    ++it;
  }

  /** \brief Does nothing. */
  template <class IteratorT>
  inline void
  incrementIf(IteratorT& /*it*/, std::false_type /*has_data*/) const
  {}

  /** \brief Assign the source iterator to the target iterator. */
  template <class ConstIteratorT, class IteratorT>
  inline void
  assignIf(const ConstIteratorT source,
           IteratorT target,
           std::true_type /*has_data*/) const
  {
    *target = *source;
  }

  /** \brief Does nothing. */
  template <class ConstIteratorT, class IteratorT>
  inline void
  assignIf(const ConstIteratorT /*source*/,
           IteratorT /*target*/,
           std::false_type /*has_data*/) const
  {}

  ////////////////////////////////////////////////////////////////////////
  // Vertex / Half-edge / Face connectivity
  ////////////////////////////////////////////////////////////////////////

  /** \brief Set the outgoing half-edge index to a given vertex. */
  inline void
  setOutgoingHalfEdgeIndex(const VertexIndex& idx_vertex,
                           const HalfEdgeIndex& idx_outgoing_half_edge)
  {
    assert(this->isValid(idx_vertex));
    this->getVertex(idx_vertex).idx_outgoing_half_edge_ = idx_outgoing_half_edge;
  }

  /** \brief Set the terminating vertex index to a given half-edge. */
  inline void
  setTerminatingVertexIndex(const HalfEdgeIndex& idx_half_edge,
                            const VertexIndex& idx_terminating_vertex)
  {
    assert(this->isValid(idx_half_edge));
    this->getHalfEdge(idx_half_edge).idx_terminating_vertex_ = idx_terminating_vertex;
  }

  /** \brief Set the next half_edge index to a given half-edge. */
  inline void
  setNextHalfEdgeIndex(const HalfEdgeIndex& idx_half_edge,
                       const HalfEdgeIndex& idx_next_half_edge)
  {
    assert(this->isValid(idx_half_edge));
    this->getHalfEdge(idx_half_edge).idx_next_half_edge_ = idx_next_half_edge;
  }

  /** \brief Set the previous half-edge index to a given half-edge. */
  inline void
  setPrevHalfEdgeIndex(const HalfEdgeIndex& idx_half_edge,
                       const HalfEdgeIndex& idx_prev_half_edge)
  {
    assert(this->isValid(idx_half_edge));
    this->getHalfEdge(idx_half_edge).idx_prev_half_edge_ = idx_prev_half_edge;
  }

  /** \brief Set the face index to a given half-edge. */
  inline void
  setFaceIndex(const HalfEdgeIndex& idx_half_edge, const FaceIndex& idx_face)
  {
    assert(this->isValid(idx_half_edge));
    this->getHalfEdge(idx_half_edge).idx_face_ = idx_face;
  }

  /** \brief Set the inner half-edge index to a given face. */
  inline void
  setInnerHalfEdgeIndex(const FaceIndex& idx_face,
                        const HalfEdgeIndex& idx_inner_half_edge)
  {
    assert(this->isValid(idx_face));
    this->getFace(idx_face).idx_inner_half_edge_ = idx_inner_half_edge;
  }

  ////////////////////////////////////////////////////////////////////////
  // isBoundary / isManifold
  ////////////////////////////////////////////////////////////////////////

  /** \brief Check if any vertex of the face lies on the boundary. */
  bool
  isBoundary(const FaceIndex& idx_face, std::true_type /*check_vertices*/) const
  {
    VertexAroundFaceCirculator circ = this->getVertexAroundFaceCirculator(idx_face);
    const VertexAroundFaceCirculator circ_end = circ;

    do {
      if (this->isBoundary(circ.getTargetIndex())) {
        return (true);
      }
    } while (++circ != circ_end);

    return (false);
  }

  /** \brief Check if any edge of the face lies on the boundary. */
  bool
  isBoundary(const FaceIndex& idx_face, std::false_type /*check_vertices*/) const
  {
    OuterHalfEdgeAroundFaceCirculator circ =
        this->getOuterHalfEdgeAroundFaceCirculator(idx_face);
    const OuterHalfEdgeAroundFaceCirculator circ_end = circ;

    do {
      if (this->isBoundary(circ.getTargetIndex())) {
        return (true);
      }
    } while (++circ != circ_end);

    return (false);
  }

  /** \brief Always manifold. */
  inline bool
  isManifold(const VertexIndex&, std::true_type /*is_manifold*/) const
  {
    return (true);
  }

  /** \brief Check if the given vertex is manifold. */
  bool
  isManifold(const VertexIndex& idx_vertex, std::false_type /*is_manifold*/) const
  {
    OutgoingHalfEdgeAroundVertexCirculator circ =
        this->getOutgoingHalfEdgeAroundVertexCirculator(idx_vertex);
    const OutgoingHalfEdgeAroundVertexCirculator circ_end = circ;

    if (!this->isBoundary((circ++).getTargetIndex()))
      return (true);
    do {
      if (this->isBoundary(circ.getTargetIndex()))
        return (false);
    } while (++circ != circ_end);

    return (true);
  }

  /** \brief Always manifold. */
  inline bool
  isManifold(std::true_type /*is_manifold*/) const
  {
    return (true);
  }

  /** \brief Check if all vertices in the mesh are manifold. */
  bool
  isManifold(std::false_type /*is_manifold*/) const
  {
    for (std::size_t i = 0; i < this->sizeVertices(); ++i) {
      if (!this->isManifold(VertexIndex(i)))
        return (false);
    }
    return (true);
  }

  ////////////////////////////////////////////////////////////////////////
  // reserveData / resizeData / clearData
  ////////////////////////////////////////////////////////////////////////

  /** \brief Reserve storage space for the mesh data. */
  template <class DataCloudT>
  inline void
  reserveData(DataCloudT& cloud, const std::size_t n, std::true_type /*has_data*/) const
  {
    cloud.reserve(n);
  }

  /** \brief Does nothing */
  template <class DataCloudT>
  inline void
  reserveData(DataCloudT& /*cloud*/,
              const std::size_t /*n*/,
              std::false_type /*has_data*/) const
  {}

  /** \brief Resize the mesh data. */
  template <class DataCloudT>
  inline void
  resizeData(DataCloudT& data_cloud,
             const std::size_t n,
             const typename DataCloudT::value_type& data,
             std::true_type /*has_data*/) const
  {
    data_cloud.resize(n, data);
  }

  /** \brief Does nothing. */
  template <class DataCloudT>
  inline void
  resizeData(DataCloudT& /*data_cloud*/,
             const std::size_t /*n*/,
             const typename DataCloudT::value_type& /*data*/,
             std::false_type /*has_data*/) const
  {}

  /** \brief Clear the mesh data. */
  template <class DataCloudT>
  inline void
  clearData(DataCloudT& cloud, std::true_type /*has_data*/) const
  {
    cloud.clear();
  }

  /** \brief Does nothing. */
  template <class DataCloudT>
  inline void
  clearData(DataCloudT& /*cloud*/, std::false_type /*has_data*/) const
  {}

  ////////////////////////////////////////////////////////////////////////
  // get / set Vertex
  ////////////////////////////////////////////////////////////////////////

  /** \brief Get the vertex for the given index. */
  inline Vertex&
  getVertex(const VertexIndex& idx_vertex)
  {
    assert(this->isValid(idx_vertex));
    return (vertices_[idx_vertex.get()]);
  }

  /** \brief Get the vertex for the given index. */
  inline Vertex
  getVertex(const VertexIndex& idx_vertex) const
  {
    assert(this->isValid(idx_vertex));
    return (vertices_[idx_vertex.get()]);
  }

  /** \brief Set the vertex at the given index. */
  inline void
  setVertex(const VertexIndex& idx_vertex, const Vertex& vertex)
  {
    assert(this->isValid(idx_vertex));
    vertices_[idx_vertex.get()] = vertex;
  }

  ////////////////////////////////////////////////////////////////////////
  // get / set HalfEdge
  ////////////////////////////////////////////////////////////////////////

  /** \brief Get the half-edge for the given index. */
  inline HalfEdge&
  getHalfEdge(const HalfEdgeIndex& idx_he)
  {
    assert(this->isValid(idx_he));
    return (half_edges_[idx_he.get()]);
  }

  /** \brief Get the half-edge for the given index. */
  inline HalfEdge
  getHalfEdge(const HalfEdgeIndex& idx_he) const
  {
    assert(this->isValid(idx_he));
    return (half_edges_[idx_he.get()]);
  }

  /** \brief Set the half-edge at the given index. */
  inline void
  setHalfEdge(const HalfEdgeIndex& idx_he, const HalfEdge& half_edge)
  {
    assert(this->isValid(idx_he));
    half_edges_[idx_he.get()] = half_edge;
  }

  ////////////////////////////////////////////////////////////////////////
  // get / set Face
  ////////////////////////////////////////////////////////////////////////

  /** \brief Get the face for the given index. */
  inline Face&
  getFace(const FaceIndex& idx_face)
  {
    assert(this->isValid(idx_face));
    return (faces_[idx_face.get()]);
  }

  /** \brief Get the face for the given index. */
  inline Face
  getFace(const FaceIndex& idx_face) const
  {
    assert(this->isValid(idx_face));
    return (faces_[idx_face.get()]);
  }

  /** \brief Set the face at the given index. */
  inline void
  setFace(const FaceIndex& idx_face, const Face& face)
  {
    assert(this->isValid(idx_face));
    faces_[idx_face.get()] = face;
  }

private:
  ////////////////////////////////////////////////////////////////////////
  // Members
  ////////////////////////////////////////////////////////////////////////

  /** \brief Data stored for the vertices. */
  VertexDataCloud vertex_data_cloud_;

  /** \brief Data stored for the half-edges. */
  HalfEdgeDataCloud half_edge_data_cloud_;

  /** \brief Data stored for the edges. */
  EdgeDataCloud edge_data_cloud_;

  /** \brief Data stored for the faces. */
  FaceDataCloud face_data_cloud_;

  /** \brief Connectivity information for the vertices. */
  Vertices vertices_;

  /** \brief Connectivity information for the half-edges. */
  HalfEdges half_edges_;

  /** \brief Connectivity information for the faces. */
  Faces faces_;

  // NOTE: It is MUCH faster to store these variables permamently.

  /** \brief Storage for addFaceImplBase and deleteFace. */
  HalfEdgeIndices inner_he_;

  /** \brief Storage for addFaceImplBase. */
  HalfEdgeIndices free_he_;

  /** \brief Storage for addFaceImplBase. */
  std::vector<bool> is_new_;

  /** \brief Storage for addFaceImplBase. */
  std::vector<bool> make_adjacent_;

  /** \brief Storage for deleteFace. */
  std::vector<bool> is_boundary_;

  /** \brief Storage for deleteVertex. */
  FaceIndices delete_faces_vertex_;

  /** \brief Storage for deleteFace. */
  FaceIndices delete_faces_face_;

public:
  template <class MeshT>
  friend class pcl::geometry::MeshIO;

  PCL_MAKE_ALIGNED_OPERATOR_NEW
};
} // End namespace geometry
} // End namespace pcl
