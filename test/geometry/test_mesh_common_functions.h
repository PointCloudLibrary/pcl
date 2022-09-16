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

#include <iostream>
#include <vector>

////////////////////////////////////////////////////////////////////////////////

// Abort circulating if the number of evaluations is too damn high.
const unsigned int max_number_polygon_vertices  = 100;
const unsigned int max_number_boundary_vertices = 100;

////////////////////////////////////////////////////////////////////////////////

/** \brief Check if the faces of the mesh are equal to the reference faces (defined by a vector of vertices). */
template <class MeshT> bool
hasFaces (const MeshT& mesh, const std::vector <typename MeshT::VertexIndices> &faces, const bool verbose = false)
{
  using VAFC = typename MeshT::VertexAroundFaceCirculator;
  using VertexIndices = typename MeshT::VertexIndices;
  using FaceIndex = typename MeshT::FaceIndex;

  if (mesh.sizeFaces () != faces.size ())
  {
    if (verbose)
    {
      std::cerr << "Incorrect number of faces: " << mesh.sizeFaces () << " != " << faces.size () << "\n";
    }
    return (false);
  }

  VertexIndices vi;
  for (std::size_t i = 0; i < mesh.sizeFaces (); ++i)
  {
    if (verbose) std::cerr << "Face " << std::setw (2) << i << ": ";
    VAFC       circ     = mesh.getVertexAroundFaceCirculator (FaceIndex (i));
    const VAFC circ_end = circ;
    vi.clear ();
    unsigned int counter = 0;
    do
    {
      if (verbose) std::cerr << std::setw (2) << circ.getTargetIndex () << " ";

      // Avoid an infinite loop if connectivity is wrong
      if (++counter > max_number_polygon_vertices)
      {
        if (verbose) std::cerr << "... Infinite loop aborted.\n";
        return (false);
      }
      vi.push_back (circ.getTargetIndex ());
    } while (++circ != circ_end);

    if (vi.size () != faces [i].size ())
    {
      std::cerr << "Wrong size!\n";
      return (false);
    }
    if (verbose) std::cerr << "\texpected: ";
    for (std::size_t j = 0; j < vi.size (); ++j)
    {
      if (verbose) std::cerr << std::setw (2) << faces [i][j] << " ";
      if (vi [j] != faces [i][j])
      {
        return (false);
      }
    }
    if (verbose) std::cerr << "\n";
  }
  return (true);
}

////////////////////////////////////////////////////////////////////////////////

/** \brief Same as the other version of hasFaces with the difference that it checks for the vertex data instead of the vertex indices.
 *  \note This method assumes that the vertex data is of type 'int'.
 */
template <class MeshT> bool
hasFaces (const MeshT& mesh, const std::vector <std::vector <int> > &faces, const bool verbose = false)
{
  using VAFC = typename MeshT::VertexAroundFaceCirculator;
  using FaceIndex = typename MeshT::FaceIndex;
  using VertexDataCloud = typename MeshT::VertexDataCloud;

  if (mesh.sizeFaces () != faces.size ())
  {
    if (verbose)
    {
      std::cerr << "Incorrect number of faces: " << mesh.sizeFaces () << " != " << faces.size () << "\n";
    }
    return (false);
  }

  const VertexDataCloud& vdc = mesh.getVertexDataCloud ();
  std::vector <int> vv;
  for (std::size_t i = 0; i < mesh.sizeFaces (); ++i)
  {
    if (verbose) std::cerr << "Face " << std::setw (2) << i << ": ";
    VAFC       circ     = mesh.getVertexAroundFaceCirculator (FaceIndex (i));
    const VAFC circ_end = circ;
    vv.clear ();
    unsigned int counter = 0;
    do
    {
      if (verbose) std::cerr << std::setw (2) << vdc [circ.getTargetIndex ().get ()] << " ";

      // Avoid an infinite loop if connectivity is wrong
      if (++counter > max_number_polygon_vertices)
      {
        if (verbose) std::cerr << "... Infinite loop aborted.\n";
        return (false);
      }
      vv.push_back (vdc [circ.getTargetIndex ().get ()]);
    } while (++circ != circ_end);

    if (vv.size () != faces [i].size ())
    {
      std::cerr << "Wrong size!\n";
      return (false);
    }
    if (verbose) std::cerr << "\texpected: ";
    for (std::size_t j=0; j<vv.size (); ++j)
    {
      if (verbose) std::cerr << std::setw (2) << faces [i][j] << " ";
      if (vv [j] != faces [i][j])
      {
        if (verbose) std::cerr << "\n";
        return (false);
      }
    }
    if (verbose) std::cerr << "\n";
  }
  return (true);
}

////////////////////////////////////////////////////////////////////////////////

/** \brief Circulate around the boundary and retrieve all vertices. */
template <class MeshT> typename MeshT::VertexIndices
getBoundaryVertices (const MeshT& mesh, const typename MeshT::VertexIndex& first, const bool verbose = false)
{
  using VAFC = typename MeshT::VertexAroundFaceCirculator;
  using HalfEdgeIndex = typename MeshT::HalfEdgeIndex;
  using VertexIndices = typename MeshT::VertexIndices;

  const HalfEdgeIndex boundary_he = mesh.getOutgoingHalfEdgeIndex (first);
  if (!mesh.isBoundary (boundary_he))
  {
    if (verbose) std::cerr << "Vertex " << first << "with outgoing half_edge "
                           << mesh.getOriginatingVertexIndex (boundary_he) << "-"
                           << mesh.getTerminatingVertexIndex (boundary_he) << " is not on the boundary!\n";
    return (VertexIndices ());
  }

  VAFC       circ     = mesh.getVertexAroundFaceCirculator (boundary_he);
  const VAFC circ_end = circ;

  VertexIndices boundary_vertices;

  unsigned int counter = 0;
  do
  {
    if (verbose) std::cerr << circ.getTargetIndex () << " ";
    // Avoid an infinite loop if connectivity is wrong
    if (++counter > max_number_boundary_vertices)
    {
      if (verbose) std::cerr << "... Infinite loop aborted.\n";
      return (VertexIndices ());
    }
    boundary_vertices.push_back (circ.getTargetIndex ());
  } while (++circ != circ_end);
  if (verbose) std::cerr << "\n";
  return (boundary_vertices);
}

////////////////////////////////////////////////////////////////////////////////

/** \brief Same as the other version of getBoundaryVertices with the difference that it retrieves the vertex data instead of the vertex indices. */
template <class MeshT> std::vector <int>
getBoundaryVertices (const MeshT& mesh, const int first, const bool verbose = false)
{
  using VAFC = typename MeshT::VertexAroundFaceCirculator;
  using VertexIndex = typename MeshT::VertexIndex;
  using HalfEdgeIndex = typename MeshT::HalfEdgeIndex;

  const HalfEdgeIndex boundary_he = mesh.getOutgoingHalfEdgeIndex (VertexIndex (first));
  if (!mesh.isBoundary (boundary_he))
  {
    if (verbose) std::cerr << "Vertex " << first << "with outgoing half_edge "
                           << mesh.getOriginatingVertexIndex (boundary_he) << "-"
                           << mesh.getTerminatingVertexIndex (boundary_he) << " is not on the boundary!\n";
    return (std::vector <int> ());
  }

  VAFC       circ     = mesh.getVertexAroundFaceCirculator (boundary_he);
  const VAFC circ_end = circ;

  std::vector <int> boundary_vertices;

  unsigned int counter = 0;
  do
  {
    if (verbose) std::cerr << mesh.getVertexDataCloud () [circ.getTargetIndex ().get ()] << " ";
    // Avoid an infinite loop if connectivity is wrong
    if (++counter > max_number_boundary_vertices)
    {
      if (verbose) std::cerr << "... Infinite loop aborted.\n";
      return (std::vector <int> ());
    }
    boundary_vertices.push_back (mesh.getVertexDataCloud () [circ.getTargetIndex ().get ()]);
  } while (++circ != circ_end);
  if (verbose) std::cerr << "\n";
  return (boundary_vertices);
}

////////////////////////////////////////////////////////////////////////////////

/** \brief Check if the 'actual' is a circular permutation of 'expected' (only clockwise).
  * \example [0 1 2 3] [1 2 3 0] [2 3 0 1] [3 0 1 2] are all equal.
  */
template <class ContainerT> bool
isCircularPermutation (const ContainerT& expected, const ContainerT& actual, const bool verbose = false)
{
  const auto n = static_cast <unsigned int> (expected.size ());
  EXPECT_EQ (n, actual.size ());
  if (n != actual.size ())
  {
    if (verbose) std::cerr << "expected.size () != actual.size (): " << n << " != " << actual.size () << "\n";
    return (false);
  }

  for (unsigned int i=0; i<n; ++i)
  {
    bool all_equal = true;
    for (unsigned int j=0; j<n; ++j)
    {
      if (verbose) std::cerr << actual [(i+j)%n] << " " << expected [j];
      if (actual [(i+j)%n] != expected [j])
      {
        all_equal = false;
      }
      if (verbose) std::cerr << " | ";
    }
    if (all_equal)
    {
      if (verbose) std::cerr << " SUCCESS\n";
      return (true);
    }
    if (verbose) std::cerr << "\n";
  }
  return (false);
}

////////////////////////////////////////////////////////////////////////////////

/** \brief Check if both the inner and outer input vector are a circular permutation. */
template <class ContainerT> bool
isCircularPermutationVec (const std::vector <ContainerT> &expected, const std::vector <ContainerT> &actual, const bool verbose = false)
{
  const auto n = static_cast<unsigned int> (expected.size ());
  EXPECT_EQ (n, actual.size ());
  if (n != actual.size ())
  {
    if (verbose) std::cerr << "expected.size () != actual.size (): " << n << " != " << actual.size () << "\n";
    return (false);
  }

  for (unsigned int i=0; i<n; ++i)
  {
    bool all_equal = true;
    for (unsigned int j=0; j<n; ++j)
    {
      if (verbose) std::cerr << "\n";
      if (!isCircularPermutation (expected [j], actual [(i+j)%n], verbose))
      {
        all_equal = false;
      }
    }
    if (verbose) std::cerr << "\n";
    if (all_equal)
    {
      return (true);
    }
  }
  return (false);
}

////////////////////////////////////////////////////////////////////////////////

/** \brief Search for the half-edge between the two input vertices.
  * \return The half-edge index if the vertex are connected and an invalid index if not.
  */
template <class MeshT> typename MeshT::HalfEdgeIndex
findHalfEdge (const MeshT&                       mesh,
              const typename MeshT::VertexIndex& idx_v_0,
              const typename MeshT::VertexIndex& idx_v_1)
{
  using HalfEdgeIndex = typename MeshT::HalfEdgeIndex;
  using VAVC = typename MeshT::VertexAroundVertexCirculator;

  if (mesh.isIsolated (idx_v_0) || mesh.isIsolated (idx_v_1))
  {
    return (HalfEdgeIndex ());
  }

  VAVC       circ     = mesh.getVertexAroundVertexCirculator (idx_v_0);
  const VAVC circ_end = circ;

  do
  {
    if (circ.getTargetIndex () == idx_v_1)
    {
      return (circ.getCurrentHalfEdgeIndex ());
    }
  } while (++circ != circ_end);

  return (HalfEdgeIndex ());
}

////////////////////////////////////////////////////////////////////////////////

/** \brief Check if the given half-edge goes from vertex a to vertex b. */
template <class MeshT> bool
checkHalfEdge (const MeshT&                        mesh,
               const typename MeshT::HalfEdgeIndex ind_he_ab,
               const typename MeshT::VertexIndex   ind_v_a,
               const typename MeshT::VertexIndex   ind_v_b)
{
  if (mesh.getOriginatingVertexIndex (ind_he_ab) != ind_v_a) return (false);
  if (mesh.getTerminatingVertexIndex (ind_he_ab) != ind_v_b) return (false);
  return (true);
}

////////////////////////////////////////////////////////////////////////////////
