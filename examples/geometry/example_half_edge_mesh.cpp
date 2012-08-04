/*
 * Software License Agreement (BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2009-2012, Willow Garage, Inc.
 * Copyright (c) 2012-, Open Perception, Inc.
 * Copyright (c) Martin Saelzle, respective authors.
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

#include <pcl/geometry/impl/triangle_mesh.hpp>

////////////////////////////////////////////////////////////////////////////////

// User data for the vertexes. Here I just store an Id. In a 3D application this would be, for example
//  - x, y, z
//  - nx, ny, nz
//  - r, g, b
class MyVertexData
{
  public:

    MyVertexData (const int id = -1) : id_ (id) {}

    int  id () const {return (id_);}
    int& id ()       {return (id_);}

    // In the current implementation only operator == has to be defined.
    bool
    operator == (const MyVertexData& other) const
    {
      return (this->id ()==other.id ());
    }

  private:

    int id_;
};

std::ostream&
operator << (std::ostream& os, const MyVertexData& vd)
{
  return (os << vd.id ());
}

////////////////////////////////////////////////////////////////////////////////

// Declare a manifold (true) or non-manifold (false) mesh holding MyVertexData.
// MyFaceData and MyHalfEdgeData could also be set in the template parameters.
typedef pcl::TriangleMesh<true, MyVertexData> Mesh;

////////////////////////////////////////////////////////////////////////////////

// Declare some typedefs
typedef Mesh::Vertex   Vertex;
typedef Mesh::HalfEdge HalfEdge;
typedef Mesh::Face     Face;

typedef Mesh::Vertexes  Vertexes;
typedef Mesh::HalfEdges HalfEdges;
typedef Mesh::Faces     Faces;

typedef Mesh::VertexIndex   VertexIndex;
typedef Mesh::HalfEdgeIndex HalfEdgeIndex;
typedef Mesh::FaceIndex     FaceIndex;

typedef Mesh::VertexIndexes   VertexIndexes;
typedef Mesh::HalfEdgeIndexes HalfEdgeIndexes;
typedef Mesh::FaceIndexes     FaceIndexes;

typedef Mesh::VertexConstIterator   VertexConstIterator;
typedef Mesh::HalfEdgeConstIterator HalfEdgeConstIterator;
typedef Mesh::FaceConstIterator     FaceConstIterator;

typedef Mesh::VertexAroundVertexConstCirculator           VAVCC;
typedef Mesh::OutgoingHalfEdgeAroundVertexConstCirculator OHEAVCC;
typedef Mesh::IncomingHalfEdgeAroundVertexConstCirculator IHEAVCC;
typedef Mesh::FaceAroundVertexConstCirculator             FAVCC;
typedef Mesh::VertexAroundFaceConstCirculator             VAFCC;
typedef Mesh::InnerHalfEdgeAroundFaceConstCirculator      IHEAFCC;
typedef Mesh::OuterHalfEdgeAroundFaceConstCirculator      OHEAFCC;
typedef Mesh::HalfEdgeAroundBoundaryConstCirculator       HEABCC;

////////////////////////////////////////////////////////////////////////////////

// Some output functions
void
printVertexes (const Mesh& mesh)
{
  std::cout << "Vertexes:\n   ";
  for (VertexConstIterator it = mesh.beginVertexes (); it!=mesh.endVertexes (); ++it)
  {
    std::cout << *it << " ";
  }
  std::cout << std::endl;
}

void
printFace (const Mesh& mesh, const Face& face)
{
  // Circulate around all vertexes in the face
  VAFCC       circ     = mesh.getVertexAroundFaceConstCirculator (face);
  const VAFCC circ_end = circ;
  std::cout << "  ";
  do
  {
    std::cout << *circ++ << " ";
  } while (circ != circ_end);
  std::cout << std::endl;
}

void
printFaces (const Mesh& mesh)
{
  std::cout << "Faces:\n";
  for (FaceConstIterator it = mesh.beginFaces (); it!=mesh.endFaces (); ++it)
  {
    printFace (mesh, *it);
  }
}

////////////////////////////////////////////////////////////////////////////////

int main ()
{
  Mesh          mesh;
  VertexIndexes vi;

  // Create a closed circle around vertex 0 //
  //   1 - 6                                //
  //  / \ / \                               //
  // 2 - 0 - 5                              //
  //  \ / \ /                               //
  //   3 - 4                                //
  vi.push_back (mesh.addVertex (MyVertexData (0)));
  vi.push_back (mesh.addVertex (MyVertexData (1)));
  vi.push_back (mesh.addVertex (MyVertexData (2)));
  vi.push_back (mesh.addVertex (MyVertexData (3)));
  vi.push_back (mesh.addVertex (MyVertexData (4)));
  vi.push_back (mesh.addVertex (MyVertexData (5)));
  vi.push_back (mesh.addVertex (6)); // Converted to MyVertexData
  vi.push_back (mesh.addVertex (7)); // Will not be connected -> isolated vertex

  mesh.addFace (vi[0], vi[1], vi[2]);
  mesh.addFace (vi[0], vi[2], vi[3]);
  mesh.addFace (vi[0], vi[3], vi[4]);
  mesh.addFace (vi[0], vi[4], vi[5]);
  mesh.addFace (vi[0], vi[5], vi[6]);
  mesh.addFace (0, 6, 1); // Converted to VertexIndex

  printVertexes (mesh);
  printFaces (mesh);

  //////////////////////////////////////////////////////////////////////////////

  std::cout << "Outgoing half-edges of vertex 0:" << std::endl;
  OHEAVCC       circ_oheav     = mesh.getOutgoingHalfEdgeAroundVertexConstCirculator (vi[0]);
  const OHEAVCC circ_oheav_end = circ_oheav;
  do
  {
    std::cout << "  "
              << circ_oheav->getOriginatingVertex (mesh) << " "
              << circ_oheav->getTerminatingVertex (mesh)
              << std::endl;
    ++circ_oheav;
  } while (circ_oheav!=circ_oheav_end);

  //////////////////////////////////////////////////////////////////////////////

  std::cout << "Circulate around the boundary half-edges:" << std::endl;
  const HalfEdgeIndex& idx_he_boundary = mesh.getElement (vi[0]).
                                         getOutgoingHalfEdge (mesh).
                                         getTerminatingVertex (mesh).
                                         getOutgoingHalfEdgeIndex ();
  HEABCC       circ_heab     = mesh.getHalfEdgeAroundBoundaryConstCirculator (idx_he_boundary);
  const HEABCC circ_heab_end = circ_heab;
  do
  {
    std::cout << "  "
              << circ_heab->getOriginatingVertex (mesh) << " "
              << circ_heab->getTerminatingVertex (mesh)
              << std::endl;
    ++circ_heab;
  } while (circ_heab!=circ_heab_end);

  //////////////////////////////////////////////////////////////////////////////

  std::cout << std::endl << "Deleting face 1 and 4 ...\n";
  std::cout << "(If the mesh is set to manifold further faces are removed automatically)\n\n";
  mesh.deleteFace (FaceIndex (1));
  mesh.deleteFace (4); // Converted to FaceIndex

  mesh.cleanUp (false); // Delete (true) or don't delete (false) isolated vertexes
  vi.clear (); // The vertex indexes are no longer synchronized with the mesh!

  printVertexes (mesh);
  printFaces (mesh);

  //////////////////////////////////////////////////////////////////////////

  std::cout << "Circulate around all faces of vertex 0:\n";

  FAVCC       circ_fav     = mesh.getFaceAroundVertexConstCirculator (mesh.frontVertexes ());
  const FAVCC circ_fav_end = circ_fav;
  do
  {
    // Very important: Some half_edges are on the boundary
    //  -> have an invalid face index
    if (circ_fav.isValid ())
    {
      printFace (mesh, *circ_fav);
    }
    else
    {
      std::cout << "  invalid face -> boundary half-edge\n";
    }
    ++circ_fav;
  } while (circ_fav!=circ_fav_end);

  return (0);
}
