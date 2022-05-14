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

#include <pcl/geometry/mesh_indices.h>

namespace pcl {
namespace geometry {
template <class DerivedT, class MeshTraitsT, class MeshTagT>
class MeshBase;

template <class MeshT>
class MeshIO;
} // End namespace geometry
} // End namespace pcl

////////////////////////////////////////////////////////////////////////////////
// Vertex
////////////////////////////////////////////////////////////////////////////////

namespace pcl {
namespace geometry {
/** \brief A vertex is a node in the mesh.
 * \author Martin Saelzle
 * \ingroup geometry
 */
class Vertex {
private:
  using HalfEdgeIndex = pcl::geometry::HalfEdgeIndex;

  /** \brief Constructor.
   * \param[in] idx_outgoing_half_edge Index to the outgoing half-edge. Defaults to an
   * invalid index.
   */
  explicit Vertex(const HalfEdgeIndex& idx_outgoing_half_edge = HalfEdgeIndex())
  : idx_outgoing_half_edge_(idx_outgoing_half_edge)
  {}

  /** \brief Index to the outgoing half-edge. The vertex is considered to be deleted if
   * it stores an invalid outgoing half-edge index. */
  HalfEdgeIndex idx_outgoing_half_edge_;

  template <class DerivedT, class MeshTraitsT, class MeshTagT>
  friend class pcl::geometry::MeshBase;

  template <class MeshT>
  friend class pcl::geometry::MeshIO;
};
} // End namespace geometry
} // End namespace pcl

////////////////////////////////////////////////////////////////////////////////
// HalfEdge
////////////////////////////////////////////////////////////////////////////////

namespace pcl {
namespace geometry {
/** \brief An edge is a connection between two vertices. In a half-edge mesh the edge is
 * split into two half-edges with opposite orientation. Each half-edge stores the index
 * to the terminating vertex, the next half-edge, the previous half-edge and the face it
 * belongs to. The opposite half-edge is accessed implicitly.
 * \author Martin Saelzle
 * \ingroup geometry
 */
class HalfEdge {
private:
  using VertexIndex = pcl::geometry::VertexIndex;
  using HalfEdgeIndex = pcl::geometry::HalfEdgeIndex;
  using FaceIndex = pcl::geometry::FaceIndex;

  /** \brief Constructor.
   * \param[in] idx_terminating_vertex Index to the terminating vertex. Defaults to an
   * invalid index.
   * \param[in] idx_next_half_edge     Index to the next half-edge.
   * Defaults to an invalid index.
   * \param[in] idx_prev_half_edge     Index to the
   * previous half-edge. Defaults to an invalid index.
   * \param[in] idx_face Index to the
   * face. Defaults to an invalid index.
   */
  explicit HalfEdge(const VertexIndex& idx_terminating_vertex = VertexIndex(),
                    const HalfEdgeIndex& idx_next_half_edge = HalfEdgeIndex(),
                    const HalfEdgeIndex& idx_prev_half_edge = HalfEdgeIndex(),
                    const FaceIndex& idx_face = FaceIndex())
  : idx_terminating_vertex_(idx_terminating_vertex)
  , idx_next_half_edge_(idx_next_half_edge)
  , idx_prev_half_edge_(idx_prev_half_edge)
  , idx_face_(idx_face)
  {}

  /** \brief Index to the terminating vertex. The half-edge is considered to be deleted
   * if it stores an invalid terminating vertex index. */
  VertexIndex idx_terminating_vertex_;

  /** \brief Index to the next half-edge. */
  HalfEdgeIndex idx_next_half_edge_;

  /** \brief Index to the previous half-edge. */
  HalfEdgeIndex idx_prev_half_edge_;

  /** \brief Index to the face. The half-edge is considered to be on the boundary if it
   * stores an invalid face index. */
  FaceIndex idx_face_;

  template <class DerivedT, class MeshTraitsT, class MeshTagT>
  friend class pcl::geometry::MeshBase;

  template <class MeshT>
  friend class pcl::geometry::MeshIO;
};
} // End namespace geometry
} // End namespace pcl

////////////////////////////////////////////////////////////////////////////////
// Face
////////////////////////////////////////////////////////////////////////////////

namespace pcl {
namespace geometry {
/** \brief A face is a closed loop of edges.
 * \author Martin Saelzle
 * \ingroup geometry
 */
class Face {
private:
  using HalfEdgeIndex = pcl::geometry::HalfEdgeIndex;

  /** \brief Constructor.
   * \param[in] inner_half_edge_idx Index to the outgoing half-edge. Defaults to an
   * invalid index
   */
  explicit Face(const HalfEdgeIndex& idx_inner_half_edge = HalfEdgeIndex())
  : idx_inner_half_edge_(idx_inner_half_edge)
  {}

  /** \brief Index to the inner half-edge. The face is considered to be deleted if it
   * stores an invalid inner half-edge index. */
  HalfEdgeIndex idx_inner_half_edge_;

  template <class DerivedT, class MeshTraitsT, class MeshTagT>
  friend class pcl::geometry::MeshBase;

  template <class MeshT>
  friend class pcl::geometry::MeshIO;
};
} // End namespace geometry
} // End namespace pcl
