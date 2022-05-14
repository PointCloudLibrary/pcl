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

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

namespace pcl {
namespace geometry {
/** \brief Read / write the half-edge mesh from / to a file.
 * \tparam MeshT e.g. pcl::geometry::TriangleMesh or pcl::geometry::PolygonMesh
 * \author Martin Saelzle
 * \ingroup geometry
 * \todo
 *  - Only writes the topology (not the mesh data).
 *  - Supports only ascii.
 *  - Does not consider the mesh traits (e.g. manifold or not)
 */
template <class MeshT>
class MeshIO {
public:
  using Mesh = MeshT;

  using Vertex = typename Mesh::Vertex;
  using HalfEdge = typename Mesh::HalfEdge;
  using Face = typename Mesh::Face;

  using Vertices = typename Mesh::Vertices;
  using HalfEdges = typename Mesh::HalfEdges;
  using Faces = typename Mesh::Faces;

  using VertexIndex = typename Mesh::VertexIndex;
  using HalfEdgeIndex = typename Mesh::HalfEdgeIndex;
  using FaceIndex = typename Mesh::FaceIndex;

  /** \brief Constructor. */
  MeshIO() {}

  /** \brief Read the mesh from a file with the given filename.
   * \param[in] filename Path to the file.
   * \param[out] mesh The loaded mesh.
   * \return true if success.
   */
  bool
  read(const std::string& filename, Mesh& mesh) const
  {
    std::ifstream file(filename.c_str());

    if (!file.is_open()) {
      std::cerr << "Error in MeshIO::read: Could not open the file '" << filename
                << "'\n";
      return (false);
    }

    // Read the header
    std::string line;
    unsigned int line_number = 1;
    int n_v = -1, n_he = -1, n_f = -1;

    if (!std::getline(file, line) || line != "PCL half-edge mesh") {
      std::cerr << "Error loading '" << filename << "' (line " << line_number
                << "): Wrong file format.\n";
      return (false);
    }
    ++line_number;

    if (!std::getline(file, line)) {
      std::cerr << "Error loading '" << filename << "'' (line " << line_number
                << "): Number of vertices / half-edges / faces not found.\n";
      return (false);
    }
    {
      std::istringstream iss(line);
      if (!(iss >> n_v >> n_he >> n_f) || iss.good()) // Don't allow more than 3 en
      {
        std::cerr << "Error loading '" << filename << "'' (line " << line_number
                  << "): Could not read the number of vertices / half-edges / faces.\n";
        return (false);
      }
    }
    if (n_v < 0 || n_he < 0 || n_f < 0) {
      std::cerr << "Error loading '" << filename << "'' (line " << line_number
                << "): Invalid number of vertices / half-edges / faces.\n";
      return (false);
    }
    ++line_number;

    // Read the vertices.
    {
      mesh.vertices_.reserve(n_v);
      HalfEdgeIndex idx_ohe; // Outgoing half-edge;

      for (int i = 0; i < n_v; ++i, ++line_number) {
        if (!std::getline(file, line)) {
          std::cerr << "Error loading '" << filename << "'' (line " << line_number
                    << "): Could not read the line.\n";
          return (false);
        }

        std::istringstream iss(line);
        if (!(iss >> idx_ohe) || iss.good()) {
          std::cerr << "Error loading '" << filename << "'' (line " << line_number
                    << "): Could not read the vertex.\n";
          return (false);
        }
        mesh.vertices_.push_back(Vertex(idx_ohe));
      }
    }

    // Read the half-edges.
    {
      mesh.half_edges_.reserve(n_he);
      VertexIndex idx_tv;    // Terminating vertex.
      HalfEdgeIndex idx_nhe; // Next half-edge;
      HalfEdgeIndex idx_phe; // Previous half-edge.
      FaceIndex idx_f;       // Face.

      for (int i = 0; i < n_he; ++i, ++line_number) {
        if (!std::getline(file, line)) {
          std::cerr << "Error loading '" << filename << "'' (line " << line_number
                    << "): Could not read the line.\n";
          return (false);
        }

        std::istringstream iss(line);
        if (!(iss >> idx_tv >> idx_nhe >> idx_phe >> idx_f) || iss.good()) {
          std::cerr << "Error loading '" << filename << "'' (line " << line_number
                    << "): Could not read the half-edge.\n";
          return (false);
        }
        mesh.half_edges_.push_back(HalfEdge(idx_tv, idx_nhe, idx_phe, idx_f));
      }
    }

    // Read the faces.
    {
      mesh.faces_.reserve(n_f);
      HalfEdgeIndex idx_ihe; // Inner half-edge.

      for (int i = 0; i < n_f; ++i, ++line_number) {
        if (!std::getline(file, line)) {
          std::cerr << "Error loading '" << filename << "'' (line " << line_number
                    << "): Could not read the line.\n";
          return (false);
        }

        std::istringstream iss(line);
        if (!(iss >> idx_ihe) || iss.good()) {
          std::cerr << "Error loading '" << filename << "'' (line " << line_number
                    << "): Could not read the face.\n";
          return (false);
        }
        mesh.faces_.push_back(Face(idx_ihe));
      }
    }

    // Set the data
    if (Mesh::HasVertexData::value)
      mesh.vertex_data_cloud_.resize(n_v);
    if (Mesh::HasHalfEdgeData::value)
      mesh.half_edge_data_cloud_.resize(n_he);
    if (Mesh::HasEdgeData::value)
      mesh.edge_data_cloud_.resize(n_he / 2);
    if (Mesh::HasFaceData::value)
      mesh.face_data_cloud_.resize(n_f);

    return (true);
  }

  /** \brief Write the mesh to a file with the given filename.
   * \param[in] filename Path to the file.
   * \param[in] mesh The saved mesh.
   * \return true if success
   */
  bool
  write(const std::string& filename, const Mesh& mesh) const
  {
    std::ofstream file(filename.c_str());

    // Write the header
    if (!file.is_open()) {
      std::cerr << "Error in MeshIO::write: Could not open the file '" << filename
                << "'\n";
      return (false);
    }

    file << "PCL half-edge mesh\n";
    file << mesh.sizeVertices() << " " << mesh.sizeHalfEdges() << " "
         << mesh.sizeFaces() << "\n";

    // Write the vertices
    for (typename Vertices::const_iterator it = mesh.vertices_.begin();
         it != mesh.vertices_.end();
         ++it) {
      file << it->idx_outgoing_half_edge_ << "\n";
    }

    // Write the half-edges
    for (typename HalfEdges::const_iterator it = mesh.half_edges_.begin();
         it != mesh.half_edges_.end();
         ++it) {
      file << it->idx_terminating_vertex_ << " " << it->idx_next_half_edge_ << " "
           << it->idx_prev_half_edge_ << " " << it->idx_face_ << "\n";
    }

    // Write the faces
    for (typename Faces::const_iterator it = mesh.faces_.begin();
         it != mesh.faces_.end();
         ++it) {
      file << it->idx_inner_half_edge_ << "\n";
    }

    return (true);
  }
};

} // End namespace geometry
} // End namespace pcl
