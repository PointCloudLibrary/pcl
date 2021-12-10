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

#include <pcl/PolygonMesh.h>
#include <pcl/conversions.h>

namespace pcl {
namespace geometry {
/** \brief Convert a half-edge mesh to a face-vertex mesh.
 * \param[in] half_edge_mesh The input mesh.
 * \param[out] face_vertex_mesh The output mesh.
 * \author Martin Saelzle
 * \ingroup geometry
 */
template <class HalfEdgeMeshT>
void
toFaceVertexMesh(const HalfEdgeMeshT& half_edge_mesh,
                 pcl::PolygonMesh& face_vertex_mesh)
{
  using HalfEdgeMesh = HalfEdgeMeshT;
  using VAFC = typename HalfEdgeMesh::VertexAroundFaceCirculator;
  using FaceIndex = typename HalfEdgeMesh::FaceIndex;

  pcl::Vertices polygon;
  pcl::toPCLPointCloud2(half_edge_mesh.getVertexDataCloud(), face_vertex_mesh.cloud);

  face_vertex_mesh.polygons.reserve(half_edge_mesh.sizeFaces());
  for (std::size_t i = 0; i < half_edge_mesh.sizeFaces(); ++i) {
    VAFC circ = half_edge_mesh.getVertexAroundFaceCirculator(FaceIndex(i));
    const VAFC circ_end = circ;
    polygon.vertices.clear();
    do {
      polygon.vertices.push_back(circ.getTargetIndex().get());
    } while (++circ != circ_end);
    face_vertex_mesh.polygons.push_back(polygon);
  }
}

/** \brief Convert a face-vertex mesh to a half-edge mesh.
 * \param[in] face_vertex_mesh The input mesh.
 * \param[out] half_edge_mesh The output mesh. It must have data associated with the
 * vertices.
 * \return The number of faces that could NOT be added to the half-edge mesh.
 * \author Martin Saelzle
 * \ingroup geometry
 */
template <class HalfEdgeMeshT>
int
toHalfEdgeMesh(const pcl::PolygonMesh& face_vertex_mesh, HalfEdgeMeshT& half_edge_mesh)
{
  using HalfEdgeMesh = HalfEdgeMeshT;
  using VertexDataCloud = typename HalfEdgeMesh::VertexDataCloud;
  using VertexIndices = typename HalfEdgeMesh::VertexIndices;

  static_assert(HalfEdgeMesh::HasVertexData::value,
                "Output mesh must have data associated with the vertices!");

  VertexDataCloud vertices;
  pcl::fromPCLPointCloud2(face_vertex_mesh.cloud, vertices);

  half_edge_mesh.reserveVertices(vertices.size());
  half_edge_mesh.reserveEdges(3 * face_vertex_mesh.polygons.size());
  half_edge_mesh.reserveFaces(face_vertex_mesh.polygons.size());

  for (const auto& vertex : vertices) {
    half_edge_mesh.addVertex(vertex);
  }

  assert(half_edge_mesh.sizeVertices() == vertices.size());

  int count_not_added = 0;
  VertexIndices vi;
  vi.reserve(3); // Minimum number (triangle)
  for (const auto& polygon : face_vertex_mesh.polygons) {
    vi.clear();
    for (const auto& vertex : polygon.vertices) {
      vi.emplace_back(vertex);
    }

    if (!half_edge_mesh.addFace(vi).isValid()) {
      ++count_not_added;
    }
  }

  return (count_not_added);
}
} // End namespace geometry
} // End namespace pcl
