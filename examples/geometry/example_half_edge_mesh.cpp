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

#include <iostream>

#include <pcl/geometry/polygon_mesh.h>

////////////////////////////////////////////////////////////////////////////////

// User data for the vertices. Here I just store an Id. In a 3D application this would be, for example
//  - x, y, z
//  - nx, ny, nz
//  - r, g, b
// ...
class MyVertexData
{
  public:

    MyVertexData (const int id = -1) : id_ (id) {}

    int  id () const {return (id_);}
    int& id ()       {return (id_);}

  private:

    int id_;
};

std::ostream&
operator << (std::ostream& os, const MyVertexData& vd)
{
  return (os << vd.id ());
}

////////////////////////////////////////////////////////////////////////////////

// Declare the mesh.
using Mesh = pcl::geometry::PolygonMesh<pcl::geometry::DefaultMeshTraits<MyVertexData> >;

using VertexIndex = Mesh::VertexIndex;
using HalfEdgeIndex = Mesh::HalfEdgeIndex;
using FaceIndex = Mesh::FaceIndex;

using VertexIndices = Mesh::VertexIndices;
using HalfEdgeIndices = Mesh::HalfEdgeIndices;
using FaceIndices = Mesh::FaceIndices;

using VAVC = Mesh::VertexAroundVertexCirculator;
using OHEAVC = Mesh::OutgoingHalfEdgeAroundVertexCirculator;
using IHEAVC = Mesh::IncomingHalfEdgeAroundVertexCirculator;
using FAVC = Mesh::FaceAroundVertexCirculator;
using VAFC = Mesh::VertexAroundFaceCirculator;
using IHEAFC = Mesh::InnerHalfEdgeAroundFaceCirculator;
using OHEAFC = Mesh::OuterHalfEdgeAroundFaceCirculator;

////////////////////////////////////////////////////////////////////////////////

// Some output functions
void
printVertices (const Mesh& mesh)
{
  std::cout << "Vertices:\n   ";
  for (std::size_t i=0; i<mesh.sizeVertices (); ++i)
  {
    std::cout << mesh.getVertexDataCloud () [i] << " ";
  }
  std::cout << std::endl;
}

void
printEdge (const Mesh& mesh, const HalfEdgeIndex& idx_he)
{
  std::cout << "  "
            << mesh.getVertexDataCloud () [mesh.getOriginatingVertexIndex (idx_he).get ()]
            << " "
            << mesh.getVertexDataCloud () [mesh.getTerminatingVertexIndex (idx_he).get ()]
            << std::endl;
}

void
printFace (const Mesh& mesh, const FaceIndex& idx_face)
{
  // Circulate around all vertices in the face
  VAFC       circ     = mesh.getVertexAroundFaceCirculator (idx_face);
  const VAFC circ_end = circ;
  std::cout << "  ";
  do
  {
    std::cout << mesh.getVertexDataCloud () [circ.getTargetIndex ().get ()] << " ";
  } while (++circ != circ_end);
  std::cout << std::endl;
}

void
printFaces (const Mesh& mesh)
{
  std::cout << "Faces:\n";
  for (std::size_t i=0; i<mesh.sizeFaces (); ++i)
  {
    printFace (mesh, FaceIndex (i));
  }
}

////////////////////////////////////////////////////////////////////////////////

int main ()
{
  Mesh          mesh;
  VertexIndices vi;

  // Create a closed circle around vertex 0 //
  //   2 - 1                                //
  //  / \ / \     7 <- Isolated vertex      //
  // 3 - 0   6                              //
  //  \ / \ /                               //
  //   4 - 5                                //
  for (unsigned int i=0; i<8; ++i)
  {
    vi.push_back (mesh.addVertex (MyVertexData (i)));
  }

  // General method to add faces.
  VertexIndices tmp;
  tmp.push_back (vi [0]);
  tmp.push_back (vi [1]);
  tmp.push_back (vi [2]);
  mesh.addFace (tmp);
  tmp.clear ();

  // Convenience method: Works only for triangles
  mesh.addFace (vi [0], vi [2], vi [3]);
  mesh.addFace (vi [0], vi [3], vi [4]);
  mesh.addFace (vi [0], vi [4], vi [5]);

  // Convenience method: Works only for quads
  mesh.addFace (vi [0], vi [5], vi [6], vi [1]);

  printVertices (mesh);
  printFaces (mesh);

  //////////////////////////////////////////////////////////////////////////////

  std::cout << "Outgoing half-edges of vertex 0:" << std::endl;
  OHEAVC       circ_oheav     = mesh.getOutgoingHalfEdgeAroundVertexCirculator (vi[0]);
  const OHEAVC circ_oheav_end = circ_oheav;
  do
  {
    printEdge (mesh,circ_oheav.getTargetIndex ());
  } while (++circ_oheav != circ_oheav_end);

  //////////////////////////////////////////////////////////////////////////////

  std::cout << "Circulate around the boundary half-edges:" << std::endl;
  const HalfEdgeIndex& idx_he_boundary = mesh.getOutgoingHalfEdgeIndex (vi [6]);
  IHEAFC       circ_iheaf     = mesh.getInnerHalfEdgeAroundFaceCirculator (idx_he_boundary);
  const IHEAFC circ_iheaf_end = circ_iheaf;
  do
  {
    printEdge (mesh, circ_iheaf.getTargetIndex ());
  } while (++circ_iheaf != circ_iheaf_end);

  //////////////////////////////////////////////////////////////////////////////

  std::cout << std::endl << "Deleting face 1 (0 2 3) and 3 (0 4 5) ...\n";
  std::cout << "(If the mesh is set to manifold further faces are removed automatically)\n\n";
  mesh.deleteFace (FaceIndex (1));
  mesh.deleteFace (FaceIndex (3));

  mesh.cleanUp (); // Removes the isolated vertex (7) as well!
  vi.clear ();     // The vertex indices are no longer synchronized with the mesh!

  printVertices (mesh);
  printFaces    (mesh);

  //////////////////////////////////////////////////////////////////////////

  std::cout << "Circulate around all faces of vertex 0:\n";

  FAVC       circ_fav     = mesh.getFaceAroundVertexCirculator (vi [0]);
  const FAVC circ_fav_end = circ_fav;
  do
  {
    // Very important: Some half_edges are on the boundary
    //  -> have an invalid face index
    if (!mesh.isBoundary (circ_fav.getCurrentHalfEdgeIndex ()))
    {
      printFace (mesh, circ_fav.getTargetIndex ());
    }
    else
    {
      std::cout << "  invalid face -> boundary half-edge\n";
    }
  } while (++circ_fav!=circ_fav_end);

  return (0);
}
