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

#include <sstream>
#include <vector>

#include <gtest/gtest.h>

#include <pcl/geometry/polygon_mesh.h>
#include <pcl/geometry/triangle_mesh.h>
#include <pcl/geometry/quad_mesh.h>

////////////////////////////////////////////////////////////////////////////////

// Abort circulating if the number of evaluation exceeds circ_max. Must be greater than the number of vertices in a face.
const unsigned int max_number_polygon_vertices  = 10;
const unsigned int max_number_boundary_vertices = 100;

typedef pcl::geometry::VertexIndex   VertexIndex;
typedef pcl::geometry::HalfEdgeIndex HalfEdgeIndex;
typedef pcl::geometry::EdgeIndex     EdgeIndex;
typedef pcl::geometry::FaceIndex     FaceIndex;

typedef std::vector <VertexIndex>   VertexIndices;
typedef std::vector <HalfEdgeIndex> HalfEdgeIndices;
typedef std::vector <FaceIndex>     FaceIndices;

template <bool IsManifoldT>
struct MeshTraits
{
    typedef int                                          VertexData;
    typedef pcl::geometry::NoData                        HalfEdgeData;
    typedef pcl::geometry::NoData                        EdgeData;
    typedef pcl::geometry::NoData                        FaceData;
    typedef boost::integral_constant <bool, IsManifoldT> IsManifold;
};

typedef pcl::geometry::TriangleMesh <MeshTraits <true > >  ManifoldTriangleMesh;
typedef pcl::geometry::TriangleMesh <MeshTraits <false> >  NonManifoldTriangleMesh;
typedef pcl::geometry::QuadMesh     <MeshTraits <true > >  ManifoldQuadMesh;
typedef pcl::geometry::QuadMesh     <MeshTraits <false> >  NonManifoldQuadMesh;
typedef pcl::geometry::PolygonMesh  <MeshTraits <true > >  ManifoldPolygonMesh;
typedef pcl::geometry::PolygonMesh  <MeshTraits <false> >  NonManifoldPolygonMesh;

template <class MeshT> class TestTriangleMesh : public testing::Test {protected: typedef MeshT Mesh;};
template <class MeshT> class TestQuadMesh     : public testing::Test {protected: typedef MeshT Mesh;};
template <class MeshT> class TestPolygonMesh  : public testing::Test {protected: typedef MeshT Mesh;};

typedef testing::Types <ManifoldQuadMesh    , NonManifoldQuadMesh    > QuadMeshTypes;
typedef testing::Types <ManifoldTriangleMesh, NonManifoldTriangleMesh> TriangleMeshTypes;
typedef testing::Types <ManifoldPolygonMesh , NonManifoldPolygonMesh > PolygonMeshTypes;

TYPED_TEST_CASE (TestTriangleMesh, TriangleMeshTypes);
TYPED_TEST_CASE (TestQuadMesh    , QuadMeshTypes);
TYPED_TEST_CASE (TestPolygonMesh , PolygonMeshTypes);

////////////////////////////////////////////////////////////////////////////////

/** \brief Check if the given half-edge goes from vertex a to vertex b. */
template <class MeshT> bool
checkHalfEdge (const MeshT& mesh, const HalfEdgeIndex ind_he_ab, const VertexIndex ind_v_a, const VertexIndex ind_v_b)
{
  if (mesh.getOriginatingVertexIndex (ind_he_ab) != ind_v_a) return (false);
  if (mesh.getTerminatingVertexIndex (ind_he_ab) != ind_v_b) return (false);
  return (true);
}

////////////////////////////////////////////////////////////////////////////////

/** \brief Check if the faces of the mesh are equal to the reference faces (defined by a vector of vertices). */
template <class MeshT> bool
hasFaces (const MeshT& mesh, const std::vector <typename MeshT::VertexIndices> faces, const bool verbose = false)
{
  typedef typename MeshT::VertexAroundFaceCirculator VAFC;
  typedef typename MeshT::VertexIndices              VertexIndices;
  typedef typename MeshT::FaceIndex                  FaceIndex;

  if (mesh.sizeFaces () != faces.size ())
  {
    if (verbose)
    {
      std::cerr << "Incorrect number of faces: " << mesh.sizeFaces () << " != " << faces.size () << "\n";
    }
    return (false);
  }

  VertexIndices vi;
  for (unsigned int i=0; i<mesh.sizeFaces (); ++i)
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
    for (unsigned int j=0; j<vi.size (); ++j)
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

/** \brief Same as the other version of hasFaces with the difference that it checks for the vertex data instead of the vertex indices. */
template <class MeshT> bool
hasFaces (const MeshT& mesh, const std::vector <std::vector <int> > faces, const bool verbose = false)
{
  typedef typename MeshT::VertexAroundFaceCirculator VAFC;
  typedef typename MeshT::FaceIndex                  FaceIndex;
  typedef typename MeshT::VertexDataCloud            VertexDataCloud;

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
  for (unsigned int i=0; i<mesh.sizeFaces (); ++i)
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
    for (unsigned int j=0; j<vv.size (); ++j)
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
template <class MeshT> VertexIndices
getBoundaryVertices (const MeshT& mesh, const VertexIndex& first, const bool verbose = false)
{
  typedef typename MeshT::VertexAroundFaceCirculator VAFC;

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
  typedef typename MeshT::VertexAroundFaceCirculator VAFC;

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

/** \brief Check if the size of the mesh elements is correct. */
template <class MeshT> void
checkSizeElements (const MeshT& mesh, const size_t n_v, const size_t n_e, const size_t n_f)
{
  ASSERT_EQ (n_v, mesh.sizeVertices ());
  ASSERT_EQ (n_e, mesh.sizeEdges ());
  ASSERT_EQ (n_f, mesh.sizeFaces ());
}

////////////////////////////////////////////////////////////////////////////////

/** \brief Check if the size of the mesh data is correct. */
template <class MeshT> void
checkSizeData (const MeshT& mesh, const size_t n_v, const size_t n_he, const size_t n_e, const size_t n_f)
{
  ASSERT_EQ (n_v , mesh.getVertexDataCloud   ().size ());
  ASSERT_EQ (n_he, mesh.getHalfEdgeDataCloud ().size ());
  ASSERT_EQ (n_e , mesh.getEdgeDataCloud     ().size ());
  ASSERT_EQ (n_f , mesh.getFaceDataCloud     ().size ());
}

////////////////////////////////////////////////////////////////////////////////

/** \brief Check if the input is a circular permutation of the expected (only clockwise).
  * \example [0 1 2 3] [1 2 3 0] [2 3 0 1] [3 0 1 2] are all equal.
  */
template <class ContainerT> bool
isCircularPermutation (const ContainerT& actual, const ContainerT& expected)
{
  const unsigned int n = static_cast<unsigned int> (expected.size ());
  EXPECT_EQ (n, actual.size ());
  if (n != actual.size ()) return (false);

  for (unsigned int i=0; i<n; ++i)
  {
    bool all_equal = true;
    for (unsigned int j=0; j<n; ++j)
    {
      if (actual [(i+j)%n] != expected [j])
      {
        all_equal = false;
      }
    }
    if (all_equal)
    {
      return (true);
    }
  }
  return (false);
}

/** \brief Search for the half-edge between the two input vertices.
  * \return The half-edge index if the vertex are connected and an invalid index if not.
  */
template <class MeshT> HalfEdgeIndex
findHalfEdge (const MeshT& mesh, const VertexIndex& idx_v_0, const VertexIndex& idx_v_1)
{
  if (mesh.isIsolated (idx_v_0) || mesh.isIsolated (idx_v_1))
  {
    return (HalfEdgeIndex ());
  }
  typedef typename MeshT::VertexAroundVertexCirculator VAVC;
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

TYPED_TEST (TestTriangleMesh, CorrectNumberOfVertices)
{
  // Make sure that only quads can be added
  typedef typename TestFixture::Mesh Mesh;

  for (unsigned int n=1; n<=5; ++n)
  {
    Mesh mesh;
    VertexIndices vi;
    for (unsigned int i=0; i<n; ++i)
    {
      vi.push_back (VertexIndex (i));
      mesh.addVertex (i);
    }

    const FaceIndex index = mesh.addFace (vi);
    if (n==3) EXPECT_TRUE  (index.isValid ()) << "Number of vertices in the face: " << n;
    else      EXPECT_FALSE (index.isValid ()) << "Number of vertices in the face: " << n;
  }
}

////////////////////////////////////////////////////////////////////////////////

TYPED_TEST (TestTriangleMesh, OneTriangle)
{
  typedef typename TestFixture::Mesh Mesh;

  //   2   //
  //  / \  //
  // 0 - 1 //
  Mesh mesh;
  VertexIndices vi;
  for (unsigned int i=0; i<3; ++i) vi.push_back (mesh.addVertex (i));

  const FaceIndex fi = mesh.addFace (vi);
  ASSERT_TRUE (fi.isValid ());

  const HalfEdgeIndex he_10 = mesh.getOutgoingHalfEdgeIndex (vi [1]);
  const HalfEdgeIndex he_21 = mesh.getOutgoingHalfEdgeIndex (vi [2]);
  const HalfEdgeIndex he_02 = mesh.getOutgoingHalfEdgeIndex (vi [0]);

  const HalfEdgeIndex he_01 = mesh.getOppositeHalfEdgeIndex (he_10);
  const HalfEdgeIndex he_12 = mesh.getOppositeHalfEdgeIndex (he_21);
  const HalfEdgeIndex he_20 = mesh.getOppositeHalfEdgeIndex (he_02);

  EXPECT_TRUE (checkHalfEdge (mesh, he_10, vi [1], vi[0]));
  EXPECT_TRUE (checkHalfEdge (mesh, he_21, vi [2], vi[1]));
  EXPECT_TRUE (checkHalfEdge (mesh, he_02, vi [0], vi[2]));

  EXPECT_TRUE (checkHalfEdge (mesh, he_01, vi [0], vi[1]));
  EXPECT_TRUE (checkHalfEdge (mesh, he_12, vi [1], vi[2]));
  EXPECT_TRUE (checkHalfEdge (mesh, he_20, vi [2], vi[0]));

  EXPECT_EQ (he_01, mesh.getIncomingHalfEdgeIndex (vi [1]));
  EXPECT_EQ (he_12, mesh.getIncomingHalfEdgeIndex (vi [2]));
  EXPECT_EQ (he_20, mesh.getIncomingHalfEdgeIndex (vi [0]));

  EXPECT_EQ (he_12, mesh.getNextHalfEdgeIndex (he_01));
  EXPECT_EQ (he_20, mesh.getNextHalfEdgeIndex (he_12));
  EXPECT_EQ (he_01, mesh.getNextHalfEdgeIndex (he_20));

  EXPECT_EQ (he_20, mesh.getPrevHalfEdgeIndex (he_01));
  EXPECT_EQ (he_01, mesh.getPrevHalfEdgeIndex (he_12));
  EXPECT_EQ (he_12, mesh.getPrevHalfEdgeIndex (he_20));

  EXPECT_EQ (he_02, mesh.getNextHalfEdgeIndex (he_10));
  EXPECT_EQ (he_21, mesh.getNextHalfEdgeIndex (he_02));
  EXPECT_EQ (he_10, mesh.getNextHalfEdgeIndex (he_21));

  EXPECT_EQ (he_21, mesh.getPrevHalfEdgeIndex (he_10));
  EXPECT_EQ (he_10, mesh.getPrevHalfEdgeIndex (he_02));
  EXPECT_EQ (he_02, mesh.getPrevHalfEdgeIndex (he_21));

  EXPECT_EQ (fi, mesh.getFaceIndex (he_01));
  EXPECT_EQ (fi, mesh.getFaceIndex (he_12));
  EXPECT_EQ (fi, mesh.getFaceIndex (he_20));

  EXPECT_FALSE (mesh.getOppositeFaceIndex (he_01).isValid ());
  EXPECT_FALSE (mesh.getOppositeFaceIndex (he_12).isValid ());
  EXPECT_FALSE (mesh.getOppositeFaceIndex (he_20).isValid ());

  EXPECT_FALSE (mesh.getFaceIndex (he_10).isValid ());
  EXPECT_FALSE (mesh.getFaceIndex (he_21).isValid ());
  EXPECT_FALSE (mesh.getFaceIndex (he_02).isValid ());

  EXPECT_EQ (fi, mesh.getOppositeFaceIndex (he_10));
  EXPECT_EQ (fi, mesh.getOppositeFaceIndex (he_21));
  EXPECT_EQ (fi, mesh.getOppositeFaceIndex (he_02));

  EXPECT_EQ (he_20, mesh.getInnerHalfEdgeIndex (fi));
  EXPECT_EQ (he_02, mesh.getOuterHalfEdgeIndex (fi));

  EXPECT_FALSE (mesh.isBoundary (he_01));
  EXPECT_FALSE (mesh.isBoundary (he_12));
  EXPECT_FALSE (mesh.isBoundary (he_20));

  EXPECT_TRUE (mesh.isBoundary (he_10));
  EXPECT_TRUE (mesh.isBoundary (he_21));
  EXPECT_TRUE (mesh.isBoundary (he_02));

  EXPECT_TRUE (mesh.isBoundary (vi [0]));
  EXPECT_TRUE (mesh.isBoundary (vi [1]));
  EXPECT_TRUE (mesh.isBoundary (vi [2]));

  EXPECT_TRUE (mesh.isBoundary (fi));
}

////////////////////////////////////////////////////////////////////////////////

TYPED_TEST (TestTriangleMesh, TwoTriangles)
{
  typedef typename TestFixture::Mesh Mesh;

  // 3 - 2   //
  //  \ / \  //
  //   0 - 1 //
  Mesh mesh;
  VertexIndex vi_0 = mesh.addVertex (0);
  VertexIndex vi_1 = mesh.addVertex (1);
  VertexIndex vi_2 = mesh.addVertex (2);
  VertexIndex vi_3 = mesh.addVertex (3);

  // First face
  VertexIndices vi;
  vi.push_back (vi_0);
  vi.push_back (vi_1);
  vi.push_back (vi_2);
  const FaceIndex fi_0 = mesh.addFace (vi);
  ASSERT_TRUE (fi_0.isValid ());

  const HalfEdgeIndex he_10 = mesh.getOutgoingHalfEdgeIndex (vi_1);
  const HalfEdgeIndex he_21 = mesh.getOutgoingHalfEdgeIndex (vi_2);
  const HalfEdgeIndex he_02 = mesh.getOutgoingHalfEdgeIndex (vi_0);

  const HalfEdgeIndex he_01 = mesh.getOppositeHalfEdgeIndex (he_10);
  const HalfEdgeIndex he_12 = mesh.getOppositeHalfEdgeIndex (he_21);
  const HalfEdgeIndex he_20 = mesh.getOppositeHalfEdgeIndex (he_02);

  // Second face
  vi.clear ();
  vi.push_back (vi_0);
  vi.push_back (vi_2);
  vi.push_back (vi_3);
  const FaceIndex fi_1 = mesh.addFace (vi);
  ASSERT_TRUE (fi_1.isValid ());

  const HalfEdgeIndex he_03 = mesh.getOutgoingHalfEdgeIndex (vi_0);
  const HalfEdgeIndex he_32 = mesh.getOutgoingHalfEdgeIndex (vi_3);

  const HalfEdgeIndex he_30 = mesh.getOppositeHalfEdgeIndex (he_03);
  const HalfEdgeIndex he_23 = mesh.getOppositeHalfEdgeIndex (he_32);

  // Tests
  EXPECT_TRUE (checkHalfEdge (mesh, he_01, vi_0, vi_1));
  EXPECT_TRUE (checkHalfEdge (mesh, he_12, vi_1, vi_2));
  EXPECT_TRUE (checkHalfEdge (mesh, he_20, vi_2, vi_0));

  EXPECT_TRUE (checkHalfEdge (mesh, he_02, vi_0, vi_2));
  EXPECT_TRUE (checkHalfEdge (mesh, he_23, vi_2, vi_3));
  EXPECT_TRUE (checkHalfEdge (mesh, he_30, vi_3, vi_0));

  EXPECT_TRUE (checkHalfEdge (mesh, he_03, vi_0, vi_3));
  EXPECT_TRUE (checkHalfEdge (mesh, he_32, vi_3, vi_2));
  EXPECT_TRUE (checkHalfEdge (mesh, he_21, vi_2, vi_1));
  EXPECT_TRUE (checkHalfEdge (mesh, he_10, vi_1, vi_0));

  EXPECT_EQ (he_12, mesh.getNextHalfEdgeIndex (he_01));
  EXPECT_EQ (he_20, mesh.getNextHalfEdgeIndex (he_12));
  EXPECT_EQ (he_01, mesh.getNextHalfEdgeIndex (he_20));

  EXPECT_EQ (he_20, mesh.getPrevHalfEdgeIndex (he_01));
  EXPECT_EQ (he_01, mesh.getPrevHalfEdgeIndex (he_12));
  EXPECT_EQ (he_12, mesh.getPrevHalfEdgeIndex (he_20));

  EXPECT_EQ (he_23, mesh.getNextHalfEdgeIndex (he_02));
  EXPECT_EQ (he_30, mesh.getNextHalfEdgeIndex (he_23));
  EXPECT_EQ (he_02, mesh.getNextHalfEdgeIndex (he_30));

  EXPECT_EQ (he_30, mesh.getPrevHalfEdgeIndex (he_02));
  EXPECT_EQ (he_02, mesh.getPrevHalfEdgeIndex (he_23));
  EXPECT_EQ (he_23, mesh.getPrevHalfEdgeIndex (he_30));

  EXPECT_EQ (he_03, mesh.getNextHalfEdgeIndex (he_10));
  EXPECT_EQ (he_32, mesh.getNextHalfEdgeIndex (he_03));
  EXPECT_EQ (he_21, mesh.getNextHalfEdgeIndex (he_32));
  EXPECT_EQ (he_10, mesh.getNextHalfEdgeIndex (he_21));

  EXPECT_EQ (he_21, mesh.getPrevHalfEdgeIndex (he_10));
  EXPECT_EQ (he_10, mesh.getPrevHalfEdgeIndex (he_03));
  EXPECT_EQ (he_03, mesh.getPrevHalfEdgeIndex (he_32));
  EXPECT_EQ (he_32, mesh.getPrevHalfEdgeIndex (he_21));

  EXPECT_EQ (fi_0, mesh.getFaceIndex (he_01));
  EXPECT_EQ (fi_0, mesh.getFaceIndex (he_12));
  EXPECT_EQ (fi_0, mesh.getFaceIndex (he_20));

  EXPECT_EQ (fi_1, mesh.getFaceIndex (he_02));
  EXPECT_EQ (fi_1, mesh.getFaceIndex (he_23));
  EXPECT_EQ (fi_1, mesh.getFaceIndex (he_30));

  EXPECT_FALSE (mesh.getFaceIndex (he_10).isValid ());
  EXPECT_FALSE (mesh.getFaceIndex (he_03).isValid ());
  EXPECT_FALSE (mesh.getFaceIndex (he_32).isValid ());
  EXPECT_FALSE (mesh.getFaceIndex (he_21).isValid ());

  EXPECT_FALSE (mesh.getOppositeFaceIndex (he_01).isValid ());
  EXPECT_FALSE (mesh.getOppositeFaceIndex (he_12).isValid ());
  EXPECT_FALSE (mesh.getOppositeFaceIndex (he_23).isValid ());
  EXPECT_FALSE (mesh.getOppositeFaceIndex (he_30).isValid ());

  EXPECT_EQ (fi_0, mesh.getOppositeFaceIndex (he_21));
  EXPECT_EQ (fi_0, mesh.getOppositeFaceIndex (he_10));
  EXPECT_EQ (fi_1, mesh.getOppositeFaceIndex (he_03));
  EXPECT_EQ (fi_1, mesh.getOppositeFaceIndex (he_32));

  EXPECT_EQ (he_20, mesh.getInnerHalfEdgeIndex (fi_0));
  EXPECT_EQ (he_30, mesh.getInnerHalfEdgeIndex (fi_1));

  EXPECT_EQ (he_02, mesh.getOuterHalfEdgeIndex (fi_0));
  EXPECT_EQ (he_03, mesh.getOuterHalfEdgeIndex (fi_1));

  EXPECT_FALSE (mesh.isBoundary (he_01));
  EXPECT_FALSE (mesh.isBoundary (he_12));
  EXPECT_FALSE (mesh.isBoundary (he_20));

  EXPECT_FALSE (mesh.isBoundary (he_02));
  EXPECT_FALSE (mesh.isBoundary (he_23));
  EXPECT_FALSE (mesh.isBoundary (he_30));

  EXPECT_TRUE (mesh.isBoundary (he_10));
  EXPECT_TRUE (mesh.isBoundary (he_03));
  EXPECT_TRUE (mesh.isBoundary (he_32));
  EXPECT_TRUE (mesh.isBoundary (he_21));

  EXPECT_TRUE (mesh.isBoundary (vi_0));
  EXPECT_TRUE (mesh.isBoundary (vi_1));
  EXPECT_TRUE (mesh.isBoundary (vi_2));
  EXPECT_TRUE (mesh.isBoundary (vi_3));

  EXPECT_TRUE (mesh.isBoundary (fi_0));
  EXPECT_TRUE (mesh.isBoundary (fi_1));
}

////////////////////////////////////////////////////////////////////////////////

TYPED_TEST (TestTriangleMesh, ThreeTriangles)
{
  typedef typename TestFixture::Mesh Mesh;

  //  1 ----- 3  //
  //  \ \   / /  //
  //   \  0  /   //
  //    \ | /    //
  //      2      //
  Mesh mesh;
  VertexIndex vi_0 = mesh.addVertex (0);
  VertexIndex vi_1 = mesh.addVertex (1);
  VertexIndex vi_2 = mesh.addVertex (2);
  VertexIndex vi_3 = mesh.addVertex (3);

  // First face
  VertexIndices vi;
  vi.push_back (vi_0);
  vi.push_back (vi_1);
  vi.push_back (vi_2);
  const FaceIndex fi_0 = mesh.addFace (vi);
  ASSERT_TRUE (fi_0.isValid ());

  const HalfEdgeIndex he_10 = mesh.getOutgoingHalfEdgeIndex (vi_1);
  const HalfEdgeIndex he_21 = mesh.getOutgoingHalfEdgeIndex (vi_2);
  const HalfEdgeIndex he_02 = mesh.getOutgoingHalfEdgeIndex (vi_0);

  const HalfEdgeIndex he_01 = mesh.getOppositeHalfEdgeIndex (he_10);
  const HalfEdgeIndex he_12 = mesh.getOppositeHalfEdgeIndex (he_21);
  const HalfEdgeIndex he_20 = mesh.getOppositeHalfEdgeIndex (he_02);

  // Second face
  vi.clear ();
  vi.push_back (vi_0);
  vi.push_back (vi_2);
  vi.push_back (vi_3);
  const FaceIndex fi_1 = mesh.addFace (vi);
  ASSERT_TRUE (fi_1.isValid ());

  const HalfEdgeIndex he_03 = mesh.getOutgoingHalfEdgeIndex (vi_0);
  const HalfEdgeIndex he_32 = mesh.getOutgoingHalfEdgeIndex (vi_3);

  const HalfEdgeIndex he_30 = mesh.getOppositeHalfEdgeIndex (he_03);
  const HalfEdgeIndex he_23 = mesh.getOppositeHalfEdgeIndex (he_32);

  // Third face
  vi.clear ();
  vi.push_back (vi_0);
  vi.push_back (vi_3);
  vi.push_back (vi_1);
  const FaceIndex fi_2 = mesh.addFace (vi);
  ASSERT_TRUE (fi_2.isValid ());

  const HalfEdgeIndex he_13 = mesh.getOutgoingHalfEdgeIndex (vi_1);

  const HalfEdgeIndex he_31 = mesh.getOppositeHalfEdgeIndex (he_13);

  // Tests
  EXPECT_TRUE (checkHalfEdge (mesh, he_01, vi_0, vi_1));
  EXPECT_TRUE (checkHalfEdge (mesh, he_12, vi_1, vi_2));
  EXPECT_TRUE (checkHalfEdge (mesh, he_20, vi_2, vi_0));

  EXPECT_TRUE (checkHalfEdge (mesh, he_02, vi_0, vi_2));
  EXPECT_TRUE (checkHalfEdge (mesh, he_23, vi_2, vi_3));
  EXPECT_TRUE (checkHalfEdge (mesh, he_30, vi_3, vi_0));

  EXPECT_TRUE (checkHalfEdge (mesh, he_03, vi_0, vi_3));
  EXPECT_TRUE (checkHalfEdge (mesh, he_31, vi_3, vi_1));
  EXPECT_TRUE (checkHalfEdge (mesh, he_10, vi_1, vi_0));

  EXPECT_TRUE (checkHalfEdge (mesh, he_32, vi_3, vi_2));
  EXPECT_TRUE (checkHalfEdge (mesh, he_21, vi_2, vi_1));
  EXPECT_TRUE (checkHalfEdge (mesh, he_13, vi_1, vi_3));

  EXPECT_EQ (he_12, mesh.getNextHalfEdgeIndex (he_01));
  EXPECT_EQ (he_20, mesh.getNextHalfEdgeIndex (he_12));
  EXPECT_EQ (he_01, mesh.getNextHalfEdgeIndex (he_20));

  EXPECT_EQ (he_23, mesh.getNextHalfEdgeIndex (he_02));
  EXPECT_EQ (he_30, mesh.getNextHalfEdgeIndex (he_23));
  EXPECT_EQ (he_02, mesh.getNextHalfEdgeIndex (he_30));

  EXPECT_EQ (he_31, mesh.getNextHalfEdgeIndex (he_03));
  EXPECT_EQ (he_10, mesh.getNextHalfEdgeIndex (he_31));
  EXPECT_EQ (he_03, mesh.getNextHalfEdgeIndex (he_10));

  EXPECT_EQ (he_21, mesh.getNextHalfEdgeIndex (he_32));
  EXPECT_EQ (he_13, mesh.getNextHalfEdgeIndex (he_21));
  EXPECT_EQ (he_32, mesh.getNextHalfEdgeIndex (he_13));

  EXPECT_EQ (he_20, mesh.getPrevHalfEdgeIndex (he_01));
  EXPECT_EQ (he_01, mesh.getPrevHalfEdgeIndex (he_12));
  EXPECT_EQ (he_12, mesh.getPrevHalfEdgeIndex (he_20));

  EXPECT_EQ (he_30, mesh.getPrevHalfEdgeIndex (he_02));
  EXPECT_EQ (he_02, mesh.getPrevHalfEdgeIndex (he_23));
  EXPECT_EQ (he_23, mesh.getPrevHalfEdgeIndex (he_30));

  EXPECT_EQ (he_10, mesh.getPrevHalfEdgeIndex (he_03));
  EXPECT_EQ (he_03, mesh.getPrevHalfEdgeIndex (he_31));
  EXPECT_EQ (he_31, mesh.getPrevHalfEdgeIndex (he_10));

  EXPECT_EQ (he_13, mesh.getPrevHalfEdgeIndex (he_32));
  EXPECT_EQ (he_32, mesh.getPrevHalfEdgeIndex (he_21));
  EXPECT_EQ (he_21, mesh.getPrevHalfEdgeIndex (he_13));

  EXPECT_EQ (fi_0, mesh.getFaceIndex (he_01));
  EXPECT_EQ (fi_0, mesh.getFaceIndex (he_12));
  EXPECT_EQ (fi_0, mesh.getFaceIndex (he_20));

  EXPECT_EQ (fi_1, mesh.getFaceIndex (he_02));
  EXPECT_EQ (fi_1, mesh.getFaceIndex (he_23));
  EXPECT_EQ (fi_1, mesh.getFaceIndex (he_30));

  EXPECT_EQ (fi_2, mesh.getFaceIndex (he_03));
  EXPECT_EQ (fi_2, mesh.getFaceIndex (he_31));
  EXPECT_EQ (fi_2, mesh.getFaceIndex (he_10));

  EXPECT_FALSE (mesh.getFaceIndex (he_32).isValid ());
  EXPECT_FALSE (mesh.getFaceIndex (he_21).isValid ());
  EXPECT_FALSE (mesh.getFaceIndex (he_13).isValid ());

  EXPECT_EQ (fi_2, mesh.getOppositeFaceIndex (he_01));
  EXPECT_FALSE    (mesh.getOppositeFaceIndex (he_12).isValid ());
  EXPECT_EQ (fi_1, mesh.getOppositeFaceIndex (he_20));

  EXPECT_EQ (fi_0, mesh.getOppositeFaceIndex (he_02));
  EXPECT_FALSE    (mesh.getOppositeFaceIndex (he_23).isValid ());
  EXPECT_EQ (fi_2, mesh.getOppositeFaceIndex (he_30));

  EXPECT_EQ (fi_1, mesh.getOppositeFaceIndex (he_03));
  EXPECT_FALSE    (mesh.getOppositeFaceIndex (he_31).isValid ());
  EXPECT_EQ (fi_0, mesh.getOppositeFaceIndex (he_10));

  EXPECT_EQ (fi_1, mesh.getOppositeFaceIndex (he_32));
  EXPECT_EQ (fi_0, mesh.getOppositeFaceIndex (he_21));
  EXPECT_EQ (fi_2, mesh.getOppositeFaceIndex (he_13));

  EXPECT_EQ (he_20, mesh.getInnerHalfEdgeIndex (fi_0));
  EXPECT_EQ (he_30, mesh.getInnerHalfEdgeIndex (fi_1));
  EXPECT_EQ (he_10, mesh.getInnerHalfEdgeIndex (fi_2));

  EXPECT_EQ (he_02, mesh.getOuterHalfEdgeIndex (fi_0));
  EXPECT_EQ (he_03, mesh.getOuterHalfEdgeIndex (fi_1));
  EXPECT_EQ (he_01, mesh.getOuterHalfEdgeIndex (fi_2));

  EXPECT_FALSE (mesh.isBoundary (he_01));
  EXPECT_FALSE (mesh.isBoundary (he_12));
  EXPECT_FALSE (mesh.isBoundary (he_20));

  EXPECT_FALSE (mesh.isBoundary (he_02));
  EXPECT_FALSE (mesh.isBoundary (he_23));
  EXPECT_FALSE (mesh.isBoundary (he_30));

  EXPECT_FALSE (mesh.isBoundary (he_03));
  EXPECT_FALSE (mesh.isBoundary (he_31));
  EXPECT_FALSE (mesh.isBoundary (he_10));

  EXPECT_TRUE  (mesh.isBoundary (he_32));
  EXPECT_TRUE  (mesh.isBoundary (he_21));
  EXPECT_TRUE  (mesh.isBoundary (he_13));

  EXPECT_FALSE (mesh.isBoundary (vi_0));
  EXPECT_TRUE  (mesh.isBoundary (vi_1));
  EXPECT_TRUE  (mesh.isBoundary (vi_2));
  EXPECT_TRUE  (mesh.isBoundary (vi_3));

  EXPECT_TRUE (mesh.isBoundary (fi_0));
  EXPECT_TRUE (mesh.isBoundary (fi_1));
  EXPECT_TRUE (mesh.isBoundary (fi_2));
}

////////////////////////////////////////////////////////////////////////////////

TEST (TestTriangleMesh2, addTrianglePair)
{
  ManifoldTriangleMesh mesh;
  VertexIndices vi;
  for (unsigned int i=0; i<16; ++i)
  {
    vi.push_back (mesh.addVertex ());
  }

  // 00 - 01 - 02 - 03 // X means that both connections / and \ are possible.
  //  | X  | X  | X  | //
  // 04 - 05 - 06 - 07 //
  //  | X  | X  | X  | //
  // 08 - 09 - 10 - 11 //
  //  | X  | X  | X  | //
  // 12 - 13 - 14 - 15 //
  VertexIndices tmp;
  std::vector <VertexIndices> faces;
  tmp.push_back (vi [ 0]); tmp.push_back (vi [ 4]); tmp.push_back (vi [ 5]); tmp.push_back (vi [ 1]); faces.push_back (tmp); tmp.clear ();
  tmp.push_back (vi [ 2]); tmp.push_back (vi [ 6]); tmp.push_back (vi [ 7]); tmp.push_back (vi [ 3]); faces.push_back (tmp); tmp.clear ();
  tmp.push_back (vi [ 1]); tmp.push_back (vi [ 5]); tmp.push_back (vi [ 6]); tmp.push_back (vi [ 2]); faces.push_back (tmp); tmp.clear ();
  tmp.push_back (vi [ 8]); tmp.push_back (vi [12]); tmp.push_back (vi [13]); tmp.push_back (vi [ 9]); faces.push_back (tmp); tmp.clear ();
  tmp.push_back (vi [10]); tmp.push_back (vi [14]); tmp.push_back (vi [15]); tmp.push_back (vi [11]); faces.push_back (tmp); tmp.clear ();
  tmp.push_back (vi [ 9]); tmp.push_back (vi [13]); tmp.push_back (vi [14]); tmp.push_back (vi [10]); faces.push_back (tmp); tmp.clear ();
  tmp.push_back (vi [ 4]); tmp.push_back (vi [ 8]); tmp.push_back (vi [ 9]); tmp.push_back (vi [ 5]); faces.push_back (tmp); tmp.clear ();
  tmp.push_back (vi [ 6]); tmp.push_back (vi [10]); tmp.push_back (vi [11]); tmp.push_back (vi [ 7]); faces.push_back (tmp); tmp.clear ();
  tmp.push_back (vi [ 5]); tmp.push_back (vi [ 9]); tmp.push_back (vi [10]); tmp.push_back (vi [ 6]); faces.push_back (tmp); tmp.clear ();

  for (unsigned int i=0; i<faces.size (); ++i)
  {
    std::pair <FaceIndex, FaceIndex> fip;
    fip = mesh.addTrianglePair (faces [i]);
    ASSERT_TRUE (fip.first.isValid ());
    ASSERT_TRUE (fip.second.isValid ());
  }

  for (unsigned int i=0; i<faces.size (); ++i)
  {
    VertexIndices actual_1, actual_2;

    ManifoldTriangleMesh::VertexAroundFaceCirculator circ     = mesh.getVertexAroundFaceCirculator (FaceIndex (2*i));
    ManifoldTriangleMesh::VertexAroundFaceCirculator circ_end = circ;
    do
    {
      actual_1.push_back (circ.getTargetIndex ());
    } while (++circ != circ_end);

    circ     = mesh.getVertexAroundFaceCirculator (FaceIndex (2*i + 1));
    circ_end = circ;
    do
    {
      actual_2.push_back (circ.getTargetIndex ());
    } while (++circ != circ_end);

    VertexIndices expected_1, expected_2, expected_3, expected_4;
    tmp = faces [i];
    expected_1.push_back (tmp [0]); expected_1.push_back (tmp [1]); expected_1.push_back (tmp [2]);
    expected_2.push_back (tmp [0]); expected_2.push_back (tmp [2]); expected_2.push_back (tmp [3]);
    expected_3.push_back (tmp [0]); expected_3.push_back (tmp [1]); expected_3.push_back (tmp [3]);
    expected_4.push_back (tmp [1]); expected_4.push_back (tmp [2]); expected_4.push_back (tmp [3]);

    EXPECT_TRUE ((isCircularPermutation (expected_1, actual_1) && isCircularPermutation (expected_2, actual_2)) ||
                 (isCircularPermutation (expected_2, actual_1) && isCircularPermutation (expected_1, actual_2)) ||
                 (isCircularPermutation (expected_3, actual_1) && isCircularPermutation (expected_4, actual_2)) ||
                 (isCircularPermutation (expected_4, actual_1) && isCircularPermutation (expected_3, actual_2)));
  }
}

////////////////////////////////////////////////////////////////////////////////

TYPED_TEST (TestQuadMesh, CorrectNumberOfVertices)
{
  // Make sure that only quads can be added
  typedef typename TestFixture::Mesh Mesh;

  for (unsigned int n=1; n<=5; ++n)
  {
    Mesh mesh;
    VertexIndices vi;
    for (unsigned int i=0; i<n; ++i)
    {
      vi.push_back (VertexIndex (i));
      mesh.addVertex (i);
    }

    const FaceIndex index = mesh.addFace (vi);
    if (n==4) EXPECT_TRUE  (index.isValid ()) << "Number of vertices in the face: " << n;
    else      EXPECT_FALSE (index.isValid ()) << "Number of vertices in the face: " << n;
  }
}

////////////////////////////////////////////////////////////////////////////////

TYPED_TEST (TestQuadMesh, OneQuad)
{
  typedef typename TestFixture::Mesh Mesh;

  // 3 - 2 //
  // |   | //
  // 0 - 1 //
  Mesh mesh;
  VertexIndices vi;
  for (unsigned int i=0; i<4; ++i) vi.push_back (mesh.addVertex (i));

  const FaceIndex fi = mesh.addFace (vi);
  ASSERT_TRUE (fi.isValid ());

  const HalfEdgeIndex he_10 = mesh.getOutgoingHalfEdgeIndex (vi [1]);
  const HalfEdgeIndex he_21 = mesh.getOutgoingHalfEdgeIndex (vi [2]);
  const HalfEdgeIndex he_32 = mesh.getOutgoingHalfEdgeIndex (vi [3]);
  const HalfEdgeIndex he_03 = mesh.getOutgoingHalfEdgeIndex (vi [0]);

  const HalfEdgeIndex he_01 = mesh.getOppositeHalfEdgeIndex (he_10);
  const HalfEdgeIndex he_12 = mesh.getOppositeHalfEdgeIndex (he_21);
  const HalfEdgeIndex he_23 = mesh.getOppositeHalfEdgeIndex (he_32);
  const HalfEdgeIndex he_30 = mesh.getOppositeHalfEdgeIndex (he_03);

  EXPECT_TRUE (checkHalfEdge (mesh, he_10, vi [1], vi[0]));
  EXPECT_TRUE (checkHalfEdge (mesh, he_21, vi [2], vi[1]));
  EXPECT_TRUE (checkHalfEdge (mesh, he_32, vi [3], vi[2]));
  EXPECT_TRUE (checkHalfEdge (mesh, he_03, vi [0], vi[3]));

  EXPECT_TRUE (checkHalfEdge (mesh, he_01, vi [0], vi[1]));
  EXPECT_TRUE (checkHalfEdge (mesh, he_12, vi [1], vi[2]));
  EXPECT_TRUE (checkHalfEdge (mesh, he_23, vi [2], vi[3]));
  EXPECT_TRUE (checkHalfEdge (mesh, he_30, vi [3], vi[0]));

  EXPECT_EQ (he_01, mesh.getIncomingHalfEdgeIndex (vi [1]));
  EXPECT_EQ (he_12, mesh.getIncomingHalfEdgeIndex (vi [2]));
  EXPECT_EQ (he_23, mesh.getIncomingHalfEdgeIndex (vi [3]));
  EXPECT_EQ (he_30, mesh.getIncomingHalfEdgeIndex (vi [0]));

  EXPECT_EQ (he_12, mesh.getNextHalfEdgeIndex (he_01));
  EXPECT_EQ (he_23, mesh.getNextHalfEdgeIndex (he_12));
  EXPECT_EQ (he_30, mesh.getNextHalfEdgeIndex (he_23));
  EXPECT_EQ (he_01, mesh.getNextHalfEdgeIndex (he_30));

  EXPECT_EQ (he_30, mesh.getPrevHalfEdgeIndex (he_01));
  EXPECT_EQ (he_01, mesh.getPrevHalfEdgeIndex (he_12));
  EXPECT_EQ (he_12, mesh.getPrevHalfEdgeIndex (he_23));
  EXPECT_EQ (he_23, mesh.getPrevHalfEdgeIndex (he_30));

  EXPECT_EQ (he_03, mesh.getNextHalfEdgeIndex (he_10));
  EXPECT_EQ (he_32, mesh.getNextHalfEdgeIndex (he_03));
  EXPECT_EQ (he_21, mesh.getNextHalfEdgeIndex (he_32));
  EXPECT_EQ (he_10, mesh.getNextHalfEdgeIndex (he_21));

  EXPECT_EQ (he_21, mesh.getPrevHalfEdgeIndex (he_10));
  EXPECT_EQ (he_10, mesh.getPrevHalfEdgeIndex (he_03));
  EXPECT_EQ (he_03, mesh.getPrevHalfEdgeIndex (he_32));
  EXPECT_EQ (he_32, mesh.getPrevHalfEdgeIndex (he_21));

  EXPECT_EQ (fi, mesh.getFaceIndex (he_01));
  EXPECT_EQ (fi, mesh.getFaceIndex (he_12));
  EXPECT_EQ (fi, mesh.getFaceIndex (he_23));
  EXPECT_EQ (fi, mesh.getFaceIndex (he_30));

  EXPECT_FALSE (mesh.getOppositeFaceIndex (he_01).isValid ());
  EXPECT_FALSE (mesh.getOppositeFaceIndex (he_12).isValid ());
  EXPECT_FALSE (mesh.getOppositeFaceIndex (he_23).isValid ());
  EXPECT_FALSE (mesh.getOppositeFaceIndex (he_30).isValid ());

  EXPECT_FALSE (mesh.getFaceIndex (he_10).isValid ());
  EXPECT_FALSE (mesh.getFaceIndex (he_21).isValid ());
  EXPECT_FALSE (mesh.getFaceIndex (he_32).isValid ());
  EXPECT_FALSE (mesh.getFaceIndex (he_03).isValid ());

  EXPECT_EQ (fi, mesh.getOppositeFaceIndex (he_10));
  EXPECT_EQ (fi, mesh.getOppositeFaceIndex (he_21));
  EXPECT_EQ (fi, mesh.getOppositeFaceIndex (he_32));
  EXPECT_EQ (fi, mesh.getOppositeFaceIndex (he_03));

  EXPECT_EQ (he_30, mesh.getInnerHalfEdgeIndex (fi));
  EXPECT_EQ (he_03, mesh.getOuterHalfEdgeIndex (fi));

  EXPECT_FALSE (mesh.isBoundary (he_01));
  EXPECT_FALSE (mesh.isBoundary (he_12));
  EXPECT_FALSE (mesh.isBoundary (he_23));
  EXPECT_FALSE (mesh.isBoundary (he_30));

  EXPECT_TRUE (mesh.isBoundary (he_10));
  EXPECT_TRUE (mesh.isBoundary (he_21));
  EXPECT_TRUE (mesh.isBoundary (he_32));
  EXPECT_TRUE (mesh.isBoundary (he_03));

  EXPECT_TRUE (mesh.isBoundary (vi [0]));
  EXPECT_TRUE (mesh.isBoundary (vi [1]));
  EXPECT_TRUE (mesh.isBoundary (vi [2]));
  EXPECT_TRUE (mesh.isBoundary (vi [3]));

  EXPECT_TRUE (mesh.isBoundary (fi));
}

////////////////////////////////////////////////////////////////////////////////

TYPED_TEST (TestQuadMesh, NineQuads)
{
  typedef typename TestFixture::Mesh Mesh;
  const int int_max = std::numeric_limits <int>::max ();

  // Order
  //    -   -   -   //
  //  | 0 | 1 | 2 | //
  //    -   -   -   //
  //  | 3 | 4 | 5 | //
  //    -   -   -   //
  //  | 6 | 7 | 8 | //
  //    -   -   -   //

  // Add the configuration in different orders. Some of them create non-manifold states.
  std::vector <std::vector <int> > order_vec;
  std::vector <int> non_manifold; // When is the first non-manifold quad added?
  std::vector <int> order_tmp;

  // Configuration 0
  order_tmp.push_back (0);
  order_tmp.push_back (1);
  order_tmp.push_back (2);
  order_tmp.push_back (3);
  order_tmp.push_back (5);
  order_tmp.push_back (4);
  order_tmp.push_back (7);
  order_tmp.push_back (6);
  order_tmp.push_back (8);
  order_vec.push_back (order_tmp);
  non_manifold.push_back (int_max);
  order_tmp.clear ();

  // Configuration 1
  order_tmp.push_back (0);
  order_tmp.push_back (1);
  order_tmp.push_back (6);
  order_tmp.push_back (8);
  order_tmp.push_back (2);
  order_tmp.push_back (3);
  order_tmp.push_back (7);
  order_tmp.push_back (5);
  order_tmp.push_back (4);
  order_vec.push_back (order_tmp);
  non_manifold.push_back (int_max);
  order_tmp.clear ();

  // Configuration 2
  order_tmp.push_back (0);
  order_tmp.push_back (1);
  order_tmp.push_back (6);
  order_tmp.push_back (8);
  order_tmp.push_back (5);
  order_tmp.push_back (2);
  order_tmp.push_back (3);
  order_tmp.push_back (7);
  order_tmp.push_back (4);
  order_vec.push_back (order_tmp);
  non_manifold.push_back (4);
  order_tmp.clear ();

  // Configuration 3
  order_tmp.push_back (1);
  order_tmp.push_back (3);
  order_tmp.push_back (5);
  order_tmp.push_back (7);
  order_tmp.push_back (4);
  order_tmp.push_back (0);
  order_tmp.push_back (2);
  order_tmp.push_back (6);
  order_tmp.push_back (8);
  order_vec.push_back (order_tmp);
  non_manifold.push_back (1);
  order_tmp.clear ();

  // Configuration 4
  order_tmp.push_back (1);
  order_tmp.push_back (3);
  order_tmp.push_back (5);
  order_tmp.push_back (7);
  order_tmp.push_back (0);
  order_tmp.push_back (2);
  order_tmp.push_back (6);
  order_tmp.push_back (8);
  order_tmp.push_back (4);
  order_vec.push_back (order_tmp);
  non_manifold.push_back (1);
  order_tmp.clear ();

  // Configuration 5
  order_tmp.push_back (0);
  order_tmp.push_back (4);
  order_tmp.push_back (8);
  order_tmp.push_back (2);
  order_tmp.push_back (6);
  order_tmp.push_back (1);
  order_tmp.push_back (7);
  order_tmp.push_back (5);
  order_tmp.push_back (3);
  order_vec.push_back (order_tmp);
  non_manifold.push_back (1);
  order_tmp.clear ();

  // 00 - 01 - 02 - 03 //
  //  |    |    |    | //
  // 04 - 05 - 06 - 07 //
  //  |    |    |    | //
  // 08 - 09 - 10 - 11 //
  //  |    |    |    | //
  // 12 - 13 - 14 - 15 //
  typedef VertexIndex VI;
  std::vector <VertexIndices> faces;
  VertexIndices vi;
  vi.push_back (VI ( 0)); vi.push_back (VI ( 4)); vi.push_back (VI ( 5)); vi.push_back (VI ( 1)); faces.push_back (vi); vi.clear ();
  vi.push_back (VI ( 1)); vi.push_back (VI ( 5)); vi.push_back (VI ( 6)); vi.push_back (VI ( 2)); faces.push_back (vi); vi.clear ();
  vi.push_back (VI ( 2)); vi.push_back (VI ( 6)); vi.push_back (VI ( 7)); vi.push_back (VI ( 3)); faces.push_back (vi); vi.clear ();
  vi.push_back (VI ( 4)); vi.push_back (VI ( 8)); vi.push_back (VI ( 9)); vi.push_back (VI ( 5)); faces.push_back (vi); vi.clear ();
  vi.push_back (VI ( 5)); vi.push_back (VI ( 9)); vi.push_back (VI (10)); vi.push_back (VI ( 6)); faces.push_back (vi); vi.clear ();
  vi.push_back (VI ( 6)); vi.push_back (VI (10)); vi.push_back (VI (11)); vi.push_back (VI ( 7)); faces.push_back (vi); vi.clear ();
  vi.push_back (VI ( 8)); vi.push_back (VI (12)); vi.push_back (VI (13)); vi.push_back (VI ( 9)); faces.push_back (vi); vi.clear ();
  vi.push_back (VI ( 9)); vi.push_back (VI (13)); vi.push_back (VI (14)); vi.push_back (VI (10)); faces.push_back (vi); vi.clear ();
  vi.push_back (VI (10)); vi.push_back (VI (14)); vi.push_back (VI (15)); vi.push_back (VI (11)); faces.push_back (vi); vi.clear ();

  ASSERT_EQ (order_vec.size (), non_manifold.size ());
  ASSERT_EQ (9, faces.size ());
  for (unsigned int i=0; i<order_vec.size (); ++i)
  {
    std::stringstream ss;
    ss << "Configuration " << i;
    SCOPED_TRACE (ss.str ());

    const std::vector <int> order = order_vec [i];

    EXPECT_EQ (9, order.size ()); // No assert so the other cases can run as well
    if (9 != order.size ()) continue;

    Mesh mesh;
    for (unsigned int j=0; j<16; ++j) mesh.addVertex (j);

    std::vector <VertexIndices> ordered_faces;

    for (unsigned int j=0; j<faces.size (); ++j)
    {
      ordered_faces.push_back (faces [order [j]]);
    }
    bool check_has_faces = true;
    for (unsigned int j=0; j<faces.size (); ++j)
    {
      const FaceIndex index = mesh.addFace (ordered_faces [j]);

      if (j < non_manifold [i] || !Mesh::IsManifold::value)
      {
        EXPECT_TRUE (index.isValid ());
      }
      else
      {
        EXPECT_FALSE (index.isValid ());
        check_has_faces = false;
        break;
      }
    }
    if (check_has_faces)
    {
      EXPECT_TRUE (hasFaces (mesh, ordered_faces));
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////

// NOTE: It is the responsibility of the user to ensure that all vertex indices are valid.

//TYPED_TEST (TestPolygonMesh, OutOfRange)
//{
//  typedef typename TestFixture::Mesh Mesh;

//  Mesh mesh;
//  VertexIndices vi;
//  for (unsigned int i=0; i<3; ++i)
//  {
//    vi.push_back (VertexIndex (i));
//  }
//  EXPECT_FALSE (mesh.addFace (vi).isValid ());

//  mesh.addVertex (0);
//  EXPECT_FALSE (mesh.addFace (vi).isValid ());

//  mesh.addVertex (1);
//  EXPECT_FALSE (mesh.addFace (vi).isValid ());

//  mesh.addVertex (2);
//  EXPECT_TRUE (mesh.addFace (vi).isValid ());
//}

//////////////////////////////////////////////////////////////////////////////////

TYPED_TEST (TestPolygonMesh, CorrectNumberOfVertices)
{
  // Make sure that only quads can be added
  typedef typename TestFixture::Mesh Mesh;

  for (unsigned int n=1; n<=5; ++n)
  {
    Mesh mesh;
    VertexIndices vi;
    for (unsigned int i=0; i<n; ++i)
    {
      vi.push_back (VertexIndex (i));
      mesh.addVertex (i);
    }

    const FaceIndex index = mesh.addFace (vi);
    if (n>=3) EXPECT_TRUE  (index.isValid ()) << "Number of vertices in the face: " << n;
    else      EXPECT_FALSE (index.isValid ()) << "Number of vertices in the face: " << n;
  }
}

//////////////////////////////////////////////////////////////////////////////////

TYPED_TEST (TestPolygonMesh, ThreePolygons)
{
  typedef typename TestFixture::Mesh Mesh;

  //   1 - 6   //
  //  / \   \  //
  // 2 - 0   5 //
  //  \   \ /  //
  //   3 - 4   //
  Mesh mesh;
  for (unsigned int i=0; i<7; ++i) mesh.addVertex (i);

  std::vector <VertexIndices> faces;
  VertexIndices vi;
  vi.push_back (VertexIndex (0));
  vi.push_back (VertexIndex (1));
  vi.push_back (VertexIndex (2));
  faces.push_back (vi);
  vi.clear ();

  vi.push_back (VertexIndex (0));
  vi.push_back (VertexIndex (2));
  vi.push_back (VertexIndex (3));
  vi.push_back (VertexIndex (4));
  faces.push_back (vi);
  vi.clear ();

  vi.push_back (VertexIndex (0));
  vi.push_back (VertexIndex (4));
  vi.push_back (VertexIndex (5));
  vi.push_back (VertexIndex (6));
  vi.push_back (VertexIndex (1));
  faces.push_back (vi);
  vi.clear ();

  for (unsigned int i=0; i<faces.size (); ++i)
  {
    ASSERT_TRUE (mesh.addFace (faces [i]).isValid ());
  }

  ASSERT_TRUE (hasFaces (mesh, faces));

  mesh.deleteFace (FaceIndex (1));
  mesh.cleanUp ();

  std::vector <std::vector <int> > expected;
  std::vector <int> tmp;
  tmp.push_back (0);
  tmp.push_back (1);
  tmp.push_back (2);
  expected.push_back (tmp);
  tmp.clear ();

  tmp.push_back (0);
  tmp.push_back (4);
  tmp.push_back (5);
  tmp.push_back (6);
  tmp.push_back (1);
  expected.push_back (tmp);
  tmp.clear ();

  ASSERT_TRUE (hasFaces (mesh, expected));
  std::vector <int> expected_boundary;
  expected_boundary.push_back (2);
  expected_boundary.push_back (1);
  expected_boundary.push_back (6);
  expected_boundary.push_back (5);
  expected_boundary.push_back (4);
  expected_boundary.push_back (0);
  ASSERT_EQ (expected_boundary, getBoundaryVertices (mesh, 0));
}

////////////////////////////////////////////////////////////////////////////////

TEST (TestAddDeleteFace, NonManifold1)
{
  // The triangle 0-1-2 is added last //
  //     5                            //
  //    / \                           //
  //   0 - 2                          //
  //  / \ / \                         //
  // 3 - 1 - 4                        //
  NonManifoldTriangleMesh mesh;
  for (unsigned int i=0; i<6; ++i) mesh.addVertex (i);

  typedef VertexIndex VI;
  VertexIndices vi;
  std::vector <VertexIndices> faces;
  vi.push_back (VI (0)); vi.push_back (VI (3)); vi.push_back (VI (1)); faces.push_back (vi); vi.clear (); // 0
  vi.push_back (VI (2)); vi.push_back (VI (1)); vi.push_back (VI (4)); faces.push_back (vi); vi.clear (); // 1
  vi.push_back (VI (0)); vi.push_back (VI (2)); vi.push_back (VI (5)); faces.push_back (vi); vi.clear (); // 2
  for (unsigned int i=0; i<faces.size (); ++i)
  {
    ASSERT_TRUE (mesh.addFace (faces [i]).isValid ());
  }
  EXPECT_TRUE (hasFaces (mesh, faces));

  // Check if the whole boundary is reached.
  VertexIndices boundary_expected;
  boundary_expected.push_back (VI (0));
  boundary_expected.push_back (VI (5));
  boundary_expected.push_back (VI (2));
  boundary_expected.push_back (VI (4));
  boundary_expected.push_back (VI (1));
  boundary_expected.push_back (VI (3));

  VertexIndices boundary_vertices = getBoundaryVertices (mesh, VI (3));
  EXPECT_EQ (boundary_expected, boundary_vertices);

  // Close the gaps.
  vi.push_back (VI (0)); vi.push_back (VI (1)); vi.push_back (VI (2)); faces.push_back (vi); vi.clear (); // 3
  ASSERT_TRUE (mesh.addFace (faces [3]).isValid ());
  EXPECT_TRUE (hasFaces (mesh, faces));

  // Delete faces
  mesh.deleteFace (FaceIndex (3));
  mesh.deleteFace (FaceIndex (1));
  mesh.cleanUp ();

  std::vector <std::vector <int> > expected_data;
  std::vector <int> data;
  data.push_back (0); data.push_back (3); data.push_back (1); expected_data.push_back (data); data.clear ();
  data.push_back (0); data.push_back (2); data.push_back (5); expected_data.push_back (data); data.clear ();

  std::vector <int> boundary_data_expected;
  boundary_data_expected.push_back (3);
  boundary_data_expected.push_back (0);
  boundary_data_expected.push_back (5);
  boundary_data_expected.push_back (2);
  boundary_data_expected.push_back (0);
  boundary_data_expected.push_back (1);

  ASSERT_TRUE (hasFaces (mesh, expected_data));
  ASSERT_EQ (boundary_data_expected, getBoundaryVertices (mesh, 1));
}

////////////////////////////////////////////////////////////////////////////////

TEST (TestAddDeleteFace, NonManifold2)
{
  NonManifoldTriangleMesh mesh;
  for (unsigned int i=0; i<9; ++i) mesh.addVertex (i);
  typedef VertexIndex VI;
  VertexIndices vi;
  std::vector <VertexIndices> faces;

  // 2 - 1 //
  //  \ /  //
  //   0   //
  //  / \  //
  // 3 - 4 //
  vi.push_back (VI (0)); vi.push_back (VI (1)); vi.push_back (VI (2)); faces.push_back (vi); vi.clear ();
  vi.push_back (VI (0)); vi.push_back (VI (3)); vi.push_back (VI (4)); faces.push_back (vi); vi.clear ();
  for (unsigned int i=0; i<faces.size (); ++i)
  {
    ASSERT_TRUE (mesh.addFace (faces [i]).isValid ());
  }
  EXPECT_TRUE (hasFaces (mesh, faces));

  // (*) Adding the next two faces would destroy the connectivity around vertex 0. E.g. a VertexAroundVertexCirculator would not be able to access all the vertices (1, 2, 3, 4) anymore.
  vi.push_back (VI (3)); vi.push_back (VI (0)); vi.push_back (VI (4));
  EXPECT_FALSE (mesh.addFace (vi).isValid ());
  vi.clear ();

  vi.push_back (VI (1)); vi.push_back (VI (0)); vi.push_back (VI (2));
  EXPECT_FALSE (mesh.addFace (vi).isValid ());
  vi.clear ();

  {
    // Check if the whole boundary is reached.
    VertexIndices boundary_expected;
    boundary_expected.push_back (VI (0));
    boundary_expected.push_back (VI (0));
    boundary_expected.push_back (VI (1));
    boundary_expected.push_back (VI (2));
    boundary_expected.push_back (VI (3));
    boundary_expected.push_back (VI (4));

    VertexIndices boundary_vertices = getBoundaryVertices (mesh, VI (2));
    std::sort (boundary_vertices.begin (), boundary_vertices.end ());
    EXPECT_EQ (boundary_expected, boundary_vertices);
  }

  // New config //
  //   2 - 1    //
  //    \ /     //
  // 3 - 0 - 6  //
  //  \ / \ /   //
  //   4   5    //
  vi.push_back (VI (0)); vi.push_back (VI (5)); vi.push_back (VI (6)); faces.push_back (vi); vi.clear ();
  EXPECT_TRUE (mesh.addFace (faces [2]).isValid ());
  EXPECT_TRUE (hasFaces (mesh, faces));

  // Same as (*)
  vi.push_back (VI (1)); vi.push_back (VI (0)); vi.push_back (VI (2));
  EXPECT_FALSE (mesh.addFace (vi).isValid ());
  vi.clear ();

  vi.push_back (VI (3)); vi.push_back (VI (0)); vi.push_back (VI (4));
  EXPECT_FALSE (mesh.addFace (vi).isValid ());
  vi.clear ();

  vi.push_back (VI (5)); vi.push_back (VI (0)); vi.push_back (VI (6));
  EXPECT_FALSE (mesh.addFace (vi).isValid ());
  vi.clear ();

  {
    // Check if the whole boundary is reached.
    VertexIndices boundary_expected;
    boundary_expected.push_back (VI (0));
    boundary_expected.push_back (VI (0));
    boundary_expected.push_back (VI (0));
    boundary_expected.push_back (VI (1));
    boundary_expected.push_back (VI (2));
    boundary_expected.push_back (VI (3));
    boundary_expected.push_back (VI (4));
    boundary_expected.push_back (VI (5));
    boundary_expected.push_back (VI (6));

    VertexIndices boundary_vertices = getBoundaryVertices (mesh, VI (2));
    std::sort (boundary_vertices.begin (), boundary_vertices.end ());
    EXPECT_EQ (boundary_expected, boundary_vertices);
  }

  // New config //
  // 2---1   8  //
  //  \  |  /|  //
  //   \ | / |  //
  //    \|/  |  //
  // 3---0---7  //
  // |  /|\     //
  // | / | \    //
  // |/  |  \   //
  // 4   5---6  //
  vi.push_back (VI (0)); vi.push_back (VI (7)); vi.push_back (VI (8)); faces.push_back (vi); vi.clear ();
  EXPECT_TRUE (mesh.addFace (faces [3]).isValid ());
  EXPECT_TRUE (hasFaces (mesh, faces));

  // Same as (*)
  vi.push_back (VI (1)); vi.push_back (VI (0)); vi.push_back (VI (2));
  EXPECT_FALSE (mesh.addFace (vi).isValid ());
  vi.clear ();

  vi.push_back (VI (3)); vi.push_back (VI (0)); vi.push_back (VI (4));
  EXPECT_FALSE (mesh.addFace (vi).isValid ());
  vi.clear ();

  vi.push_back (VI (5)); vi.push_back (VI (0)); vi.push_back (VI (6));
  EXPECT_FALSE (mesh.addFace (vi).isValid ());
  vi.clear ();

  vi.push_back (VI (7)); vi.push_back (VI (0)); vi.push_back (VI (8));
  EXPECT_FALSE (mesh.addFace (vi).isValid ());
  vi.clear ();

  // Check if the whole boundary is reached.
  {
    VertexIndices boundary_expected;
    boundary_expected.push_back (VI (0));
    boundary_expected.push_back (VI (0));
    boundary_expected.push_back (VI (0));
    boundary_expected.push_back (VI (0));
    boundary_expected.push_back (VI (1));
    boundary_expected.push_back (VI (2));
    boundary_expected.push_back (VI (3));
    boundary_expected.push_back (VI (4));
    boundary_expected.push_back (VI (5));
    boundary_expected.push_back (VI (6));
    boundary_expected.push_back (VI (7));
    boundary_expected.push_back (VI (8));

    VertexIndices boundary_vertices = getBoundaryVertices (mesh, VI (2));
    std::sort (boundary_vertices.begin (), boundary_vertices.end ());
    EXPECT_EQ (boundary_expected, boundary_vertices);
  }

  // Close the gaps //
  // 2---1---8      //
  // |\  |  /|      //
  // | \0|5/ |      //
  // | 6\|/3 |      //
  // 3---0---7      //
  // | 1/|\7 |      //
  // | /4|2\ |      //
  // |/  |  \|      //
  // 4---5---6      //
  vi.push_back (VI (0)); vi.push_back (VI (4)); vi.push_back (VI (5)); faces.push_back (vi); vi.clear ();
  vi.push_back (VI (0)); vi.push_back (VI (8)); vi.push_back (VI (1)); faces.push_back (vi); vi.clear ();
  vi.push_back (VI (0)); vi.push_back (VI (2)); vi.push_back (VI (3)); faces.push_back (vi); vi.clear ();
  vi.push_back (VI (0)); vi.push_back (VI (6)); vi.push_back (VI (7)); faces.push_back (vi); vi.clear ();
  for (unsigned int i=4; i<faces.size (); ++i)
  {
    EXPECT_TRUE (mesh.addFace (faces [i]).isValid ());
  }
  EXPECT_TRUE (hasFaces (mesh, faces));

  VertexIndices boundary_expected;
  for (unsigned int i=8; i>0; --i)
  {
    boundary_expected.push_back (VI (i));
  }
  VertexIndices boundary_vertices = getBoundaryVertices (mesh, VI (1));
  EXPECT_EQ (boundary_expected, boundary_vertices);

  // Copy vertex indices to data
  std::vector <std::vector <int> > expected (faces.size ());
  for (unsigned int i=0; i<faces.size (); ++i)
  {
    std::vector <int> tmp (faces [i].size ());
    for (unsigned int j=0; j<faces [i].size (); ++j)
    {
      tmp [j] = faces [i][j].get ();
    }
    expected [i] = tmp;
  }

  // Delete all faces
  while (expected.size ())
  {
    mesh.deleteFace (FaceIndex (0));
    mesh.cleanUp ();
    expected.erase (expected.begin ());
    ASSERT_TRUE (hasFaces (mesh, expected));
  }

  ASSERT_TRUE (mesh.empty ());
}

////////////////////////////////////////////////////////////////////////////////

TEST (TestAddDeleteFace, Manifold)
{
  typedef ManifoldTriangleMesh Mesh;
  typedef VertexIndex          VI;
  Mesh mesh;
  for (unsigned int i=0; i<7; ++i) mesh.addVertex (i);

  //   2 - 1   //
  //  / \ / \  //
  // 3 - 0 - 6 //
  //  \ / \ /  //
  //   4 - 5   //
  std::vector <VertexIndices> faces;
  std::vector <std::vector <int> > expected;
  VertexIndices vi;
  vi.push_back (VI (0)); vi.push_back (VI (1)); vi.push_back (VI (2)); faces.push_back (vi); vi.clear ();
  vi.push_back (VI (0)); vi.push_back (VI (2)); vi.push_back (VI (3)); faces.push_back (vi); vi.clear ();
  vi.push_back (VI (0)); vi.push_back (VI (3)); vi.push_back (VI (4)); faces.push_back (vi); vi.clear ();
  vi.push_back (VI (0)); vi.push_back (VI (4)); vi.push_back (VI (5)); faces.push_back (vi); vi.clear ();
  vi.push_back (VI (0)); vi.push_back (VI (5)); vi.push_back (VI (6)); faces.push_back (vi); vi.clear ();
  vi.push_back (VI (0)); vi.push_back (VI (6)); vi.push_back (VI (1)); faces.push_back (vi); vi.clear ();

  for (unsigned int i=0; i<faces.size (); ++i)
  {
    std::vector <int> tmp (faces [i].size ());
    for (unsigned int j=0; j<faces [i].size (); ++j)
    {
      tmp [j] = faces [i][j].get ();
    }
    expected.push_back (tmp);
    ASSERT_TRUE (mesh.addFace (faces [i]).isValid ()) << "Face " << i;
  }
  ASSERT_TRUE (hasFaces (mesh, faces));
  ASSERT_TRUE (hasFaces (mesh, expected));

  mesh.deleteFace (FaceIndex (0)); // 012
  mesh.deleteFace (FaceIndex (3)); // 045 -> makes mesh non-manifold -> delete additional faces.
  mesh.cleanUp ();

  EXPECT_TRUE (mesh.isManifold ());

  // Only the faces 0-5-6 and 0-6-1 remain after deletion.
  // NOTE: It would be also valid if the faces 0-2-3 and 0-3-4 remain.
  ASSERT_TRUE (hasFaces (mesh, std::vector <std::vector <int> > (expected.begin ()+4, expected.end ())));

  //   9 - 10    //
  //  / \ / \    //
  // 8 - 2 - 1   //
  //  \ / \ / \  //
  //   3 - 0 - 6 //
  //  / \ / \ /  //
  // 7 - 4 - 5   //
  mesh.clear ();
  expected.clear ();
  for (unsigned int i=0; i<11; ++i) mesh.addVertex (i);
  vi.push_back (VI ( 3)); vi.push_back (VI (7)); vi.push_back (VI (4)); faces.push_back (vi); vi.clear ();
  vi.push_back (VI ( 3)); vi.push_back (VI (2)); vi.push_back (VI (8)); faces.push_back (vi); vi.clear ();
  vi.push_back (VI ( 8)); vi.push_back (VI (2)); vi.push_back (VI (9)); faces.push_back (vi); vi.clear ();
  vi.push_back (VI (10)); vi.push_back (VI (9)); vi.push_back (VI (2)); faces.push_back (vi); vi.clear ();
  vi.push_back (VI (10)); vi.push_back (VI (2)); vi.push_back (VI (1)); faces.push_back (vi); vi.clear ();
  for (unsigned int i=0; i<faces.size (); ++i)
  {
    ASSERT_TRUE (mesh.addFace (faces [i]).isValid ()) << "Face " << i;
  }
  ASSERT_TRUE (hasFaces (mesh, faces));

  mesh.deleteFace (FaceIndex (0)); // 0-1-2 -> deletes 10-2-1
  mesh.deleteFace (FaceIndex (3)); // 0-4-5 -> deletes  0-3-4, 3-7-4 and 0-2-3
  mesh.cleanUp ();

  std::vector <int> tmp;
  tmp.push_back ( 0); tmp.push_back (5); tmp.push_back (6); expected.push_back (tmp); tmp.clear ();
  tmp.push_back ( 0); tmp.push_back (6); tmp.push_back (1); expected.push_back (tmp); tmp.clear ();
  tmp.push_back ( 3); tmp.push_back (2); tmp.push_back (8); expected.push_back (tmp); tmp.clear ();
  tmp.push_back ( 8); tmp.push_back (2); tmp.push_back (9); expected.push_back (tmp); tmp.clear ();
  tmp.push_back (10); tmp.push_back (9); tmp.push_back (2); expected.push_back (tmp); tmp.clear ();
  EXPECT_TRUE (mesh.isManifold ());
  ASSERT_TRUE (hasFaces (mesh, expected));

  std::vector <int> expected_boundary;
  expected_boundary.push_back (6);
  expected_boundary.push_back (5);
  expected_boundary.push_back (0);
  expected_boundary.push_back (1);
  EXPECT_EQ (expected_boundary, getBoundaryVertices (mesh, 1));

  expected_boundary.clear ();
  expected_boundary.push_back ( 2);
  expected_boundary.push_back ( 3);
  expected_boundary.push_back ( 8);
  expected_boundary.push_back ( 9);
  expected_boundary.push_back (10);
  EXPECT_EQ (expected_boundary, getBoundaryVertices (mesh, 8));
}

////////////////////////////////////////////////////////////////////////////////

TEST (TestDelete, VertexAndEdge)
{
  typedef NonManifoldTriangleMesh Mesh;
  typedef VertexIndex             VI;
  Mesh mesh;
  for (unsigned int i=0; i<7; ++i) mesh.addVertex (i);

  //   2 - 1          2 - 1  //
  //  / \ / \        / \ /   //
  // 3 - 0 - 6  ->  3 - 0    //
  //  \ / \ /        \ / \   //
  //   4 - 5          4 - 5  //
  std::vector <VertexIndices> faces;
  std::vector <std::vector <int> > expected;
  VertexIndices vi;
  vi.push_back (VI (0)); vi.push_back (VI (1)); vi.push_back (VI (2)); faces.push_back (vi); vi.clear ();
  vi.push_back (VI (0)); vi.push_back (VI (2)); vi.push_back (VI (3)); faces.push_back (vi); vi.clear ();
  vi.push_back (VI (0)); vi.push_back (VI (3)); vi.push_back (VI (4)); faces.push_back (vi); vi.clear ();
  vi.push_back (VI (0)); vi.push_back (VI (4)); vi.push_back (VI (5)); faces.push_back (vi); vi.clear ();
  vi.push_back (VI (0)); vi.push_back (VI (5)); vi.push_back (VI (6)); faces.push_back (vi); vi.clear ();
  vi.push_back (VI (0)); vi.push_back (VI (6)); vi.push_back (VI (1)); faces.push_back (vi); vi.clear ();

  for (unsigned int i=0; i<faces.size (); ++i)
  {
    std::vector <int> tmp (faces [i].size ());
    for (unsigned int j=0; j<faces [i].size (); ++j)
    {
      tmp [j] = faces [i][j].get ();
    }
    expected.push_back (tmp);
    ASSERT_TRUE (mesh.addFace (faces [i]).isValid ()) << "Face " << i;
  }
  ASSERT_TRUE (hasFaces (mesh, faces));
  ASSERT_TRUE (hasFaces (mesh, expected));

  // Search edge 0-6
  Mesh::VertexAroundVertexCirculator       circ     = mesh.getVertexAroundVertexCirculator (VI (0));
  const Mesh::VertexAroundVertexCirculator circ_end = circ;
  HalfEdgeIndex idx_he_06;
  do
  {
    if (circ.getTargetIndex () == VI (6))
    {
      idx_he_06 = circ.getCurrentHalfEdgeIndex ();
      break;
    }
  } while (++circ != circ_end);

  // Delete
  Mesh mesh_deleted_vertex    = mesh;
  Mesh mesh_deleted_half_edge = mesh;
  Mesh mesh_deleted_edge      = mesh;
  expected.pop_back ();
  expected.pop_back ();

  mesh_deleted_vertex.deleteVertex (VI (6));
  mesh_deleted_vertex.cleanUp ();

  mesh_deleted_half_edge.deleteEdge (idx_he_06);
  mesh_deleted_half_edge.cleanUp ();

  mesh_deleted_edge.deleteEdge (pcl::geometry::toEdgeIndex (idx_he_06));
  mesh_deleted_edge.cleanUp ();

  EXPECT_TRUE (hasFaces (mesh_deleted_vertex,    expected));
  EXPECT_TRUE (hasFaces (mesh_deleted_half_edge, expected));
  EXPECT_TRUE (hasFaces (mesh_deleted_edge,      expected));
}

////////////////////////////////////////////////////////////////////////////////

TEST (TestMesh, IsBoundaryIsManifold)
{
  // The triangle 0-1-2 is added last //
  //     5                            //
  //    / \                           //
  //   0 - 2                          //
  //  / \ / \                         //
  // 3 - 1 - 4                        //
  NonManifoldTriangleMesh mesh;
  for (unsigned int i=0; i<6; ++i) mesh.addVertex (i);

  typedef VertexIndex VI;
  VertexIndices vi;
  std::vector <VertexIndices> faces;
  vi.push_back (VI (0)); vi.push_back (VI (3)); vi.push_back (VI (1)); faces.push_back (vi); vi.clear (); // 0
  vi.push_back (VI (2)); vi.push_back (VI (1)); vi.push_back (VI (4)); faces.push_back (vi); vi.clear (); // 1
  vi.push_back (VI (0)); vi.push_back (VI (2)); vi.push_back (VI (5)); faces.push_back (vi); vi.clear (); // 2
  for (unsigned int i=0; i<faces.size (); ++i)
  {
    ASSERT_TRUE (mesh.addFace (faces [i]).isValid ());
  }
  EXPECT_TRUE (hasFaces (mesh, faces));

  // Check if the whole boundary is reached.
  VertexIndices boundary_expected;
  boundary_expected.push_back (VI (0));
  boundary_expected.push_back (VI (5));
  boundary_expected.push_back (VI (2));
  boundary_expected.push_back (VI (4));
  boundary_expected.push_back (VI (1));
  boundary_expected.push_back (VI (3));

  VertexIndices boundary_vertices = getBoundaryVertices (mesh, VI (3));
  EXPECT_EQ (boundary_expected, boundary_vertices);
  EXPECT_FALSE (mesh.isManifold (VI (0)));
  EXPECT_FALSE (mesh.isManifold (VI (1)));
  EXPECT_FALSE (mesh.isManifold (VI (2)));
  EXPECT_TRUE  (mesh.isManifold (VI (3)));
  EXPECT_TRUE  (mesh.isManifold (VI (4)));
  EXPECT_TRUE  (mesh.isManifold (VI (5)));
  ASSERT_FALSE (mesh.isManifold ());

  for (unsigned int i=0; i<mesh.sizeEdges (); ++i)
  {
    ASSERT_TRUE (mesh.isBoundary (EdgeIndex (i)));
  }

  // Make manifold
  vi.push_back (VI (0)); vi.push_back (VI (1)); vi.push_back (VI (2)); faces.push_back (vi); vi.clear (); // 3
  ASSERT_TRUE  (mesh.addFace (faces [3]).isValid ());
  EXPECT_TRUE  (hasFaces (mesh, faces));

  EXPECT_TRUE  (mesh.isManifold ());
  EXPECT_TRUE  (mesh.isBoundary (FaceIndex (3)));         // This version checks the vertices
  EXPECT_FALSE (mesh.isBoundary <false> (FaceIndex (3))); // This version checks the edges

  const HalfEdgeIndex idx_he_01 = findHalfEdge (mesh, VI (0), VI (1));
  const HalfEdgeIndex idx_he_12 = findHalfEdge (mesh, VI (1), VI (2));
  const HalfEdgeIndex idx_he_20 = findHalfEdge (mesh, VI (2), VI (0));

  ASSERT_TRUE (idx_he_01.isValid ());
  ASSERT_TRUE (idx_he_12.isValid ());
  ASSERT_TRUE (idx_he_20.isValid ());

  EXPECT_FALSE (mesh.isBoundary (pcl::geometry::toEdgeIndex (idx_he_01)));
  EXPECT_FALSE (mesh.isBoundary (pcl::geometry::toEdgeIndex (idx_he_12)));
  EXPECT_FALSE (mesh.isBoundary (pcl::geometry::toEdgeIndex (idx_he_20)));

  // Isolated vertex
  mesh.addVertex (6);
  ASSERT_TRUE (mesh.isManifold (VI (6)));
  ASSERT_TRUE (mesh.isBoundary (VI (6)));
}

////////////////////////////////////////////////////////////////////////////////

TEST (TestMesh, MeshData)
{
  typedef pcl::geometry::NoData NoData;
  typedef pcl::geometry::DefaultMeshTraits <int   , NoData , NoData, NoData> TraitsV;
  typedef pcl::geometry::DefaultMeshTraits <NoData, int    , NoData, NoData> TraitsHE;
  typedef pcl::geometry::DefaultMeshTraits <NoData, NoData , int   , NoData> TraitsE;
  typedef pcl::geometry::DefaultMeshTraits <NoData, NoData , NoData, int   > TraitsF;
  typedef pcl::geometry::DefaultMeshTraits <int   , int    , int   , int   > TraitsAD;

  typedef pcl::geometry::PolygonMesh <TraitsV>  MeshV;
  typedef pcl::geometry::PolygonMesh <TraitsHE> MeshHE;
  typedef pcl::geometry::PolygonMesh <TraitsE>  MeshE;
  typedef pcl::geometry::PolygonMesh <TraitsF>  MeshF;
  typedef pcl::geometry::PolygonMesh <TraitsAD> MeshAD;

  EXPECT_TRUE  (MeshV::HasVertexData::value);
  EXPECT_FALSE (MeshV::HasHalfEdgeData::value);
  EXPECT_FALSE (MeshV::HasEdgeData::value);
  EXPECT_FALSE (MeshV::HasFaceData::value);

  EXPECT_FALSE (MeshHE::HasVertexData::value);
  EXPECT_TRUE  (MeshHE::HasHalfEdgeData::value);
  EXPECT_FALSE (MeshHE::HasEdgeData::value);
  EXPECT_FALSE (MeshHE::HasFaceData::value);

  EXPECT_FALSE (MeshE::HasVertexData::value);
  EXPECT_FALSE (MeshE::HasHalfEdgeData::value);
  EXPECT_TRUE  (MeshE::HasEdgeData::value);
  EXPECT_FALSE (MeshE::HasFaceData::value);

  EXPECT_FALSE (MeshF::HasVertexData::value);
  EXPECT_FALSE (MeshF::HasHalfEdgeData::value);
  EXPECT_FALSE (MeshF::HasEdgeData::value);
  EXPECT_TRUE  (MeshF::HasFaceData::value);

  EXPECT_TRUE (MeshAD::HasVertexData::value);
  EXPECT_TRUE (MeshAD::HasHalfEdgeData::value);
  EXPECT_TRUE (MeshAD::HasEdgeData::value);
  EXPECT_TRUE (MeshAD::HasFaceData::value);

  // 3 - 2 - 4 //
  //  \ / \ /  //
  //   0 - 1   //
  VertexIndices vi_0, vi_1, vi_2;
  vi_0.push_back (VertexIndex (0)); vi_0.push_back (VertexIndex (1)); vi_0.push_back (VertexIndex (2));
  vi_1.push_back (VertexIndex (0)); vi_1.push_back (VertexIndex (2)); vi_1.push_back (VertexIndex (3));
  vi_2.push_back (VertexIndex (4)); vi_2.push_back (VertexIndex (2)); vi_2.push_back (VertexIndex (1));

  // Mesh data.
  int    vd_0  (10), vd_1  (11), vd_2  (12), vd_3 (13), vd_4 (14);
  int    hed_0 (20), hed_1 (21), hed_2 (22);
  int    ed_0  (30), ed_1  (31), ed_2  (32);
  int    fd_0  (40), fd_1  (41), fd_2  (42);
  NoData nd;

  {
    SCOPED_TRACE ("Mesh with vertex data");
    MeshV mesh;

    EXPECT_TRUE (mesh.addVertex (vd_0).isValid ());
    EXPECT_TRUE (mesh.addVertex (vd_1).isValid ());
    EXPECT_TRUE (mesh.addVertex (vd_2).isValid ());
    EXPECT_TRUE (mesh.addVertex (vd_3).isValid ());
    EXPECT_TRUE (mesh.addVertex (vd_4).isValid ());

    EXPECT_TRUE (mesh.addFace (vi_0, nd, nd, nd).isValid ());
    EXPECT_TRUE (mesh.addFace (vi_1, nd, nd, nd).isValid ());
    EXPECT_TRUE (mesh.addFace (vi_2, nd, nd, nd).isValid ());

    checkSizeElements (mesh, 5,    7, 3);
    checkSizeData     (mesh, 5, 0, 0, 0);

    EXPECT_EQ (vd_0, mesh.getVertexDataCloud () [0]);
    EXPECT_EQ (vd_1, mesh.getVertexDataCloud () [1]);
    EXPECT_EQ (vd_2, mesh.getVertexDataCloud () [2]);
    EXPECT_EQ (vd_3, mesh.getVertexDataCloud () [3]);
    EXPECT_EQ (vd_4, mesh.getVertexDataCloud () [4]);

    mesh.deleteFace (FaceIndex (1));
    mesh.cleanUp ();

    checkSizeElements (mesh, 4,    5, 2);
    checkSizeData     (mesh, 4, 0, 0, 0);

    EXPECT_EQ (vd_0, mesh.getVertexDataCloud () [0]);
    EXPECT_EQ (vd_1, mesh.getVertexDataCloud () [1]);
    EXPECT_EQ (vd_2, mesh.getVertexDataCloud () [2]);
    EXPECT_EQ (vd_4, mesh.getVertexDataCloud () [3]);
  }

  {
    SCOPED_TRACE ("Mesh with edge data");
    MeshE mesh;

    EXPECT_TRUE (mesh.addVertex ().isValid ());
    EXPECT_TRUE (mesh.addVertex ().isValid ());
    EXPECT_TRUE (mesh.addVertex ().isValid ());
    EXPECT_TRUE (mesh.addVertex ().isValid ());
    EXPECT_TRUE (mesh.addVertex ().isValid ());

    EXPECT_TRUE (mesh.addFace (vi_0, nd, ed_0, nd).isValid ());
    EXPECT_TRUE (mesh.addFace (vi_1, nd, ed_1, nd).isValid ());
    EXPECT_TRUE (mesh.addFace (vi_2, nd, ed_2, nd).isValid ());

    checkSizeElements (mesh, 5,    7, 3);
    checkSizeData     (mesh, 0, 0, 7, 0);

    EXPECT_EQ (ed_0, mesh.getEdgeDataCloud () [0]);
    EXPECT_EQ (ed_0, mesh.getEdgeDataCloud () [1]);
    EXPECT_EQ (ed_0, mesh.getEdgeDataCloud () [2]);
    EXPECT_EQ (ed_1, mesh.getEdgeDataCloud () [3]);
    EXPECT_EQ (ed_1, mesh.getEdgeDataCloud () [4]);
    EXPECT_EQ (ed_2, mesh.getEdgeDataCloud () [5]);
    EXPECT_EQ (ed_2, mesh.getEdgeDataCloud () [6]);

    mesh.deleteFace (FaceIndex (1));
    mesh.cleanUp ();

    checkSizeElements (mesh, 4,    5, 2);
    checkSizeData     (mesh, 0, 0, 5, 0);

    EXPECT_EQ (ed_0, mesh.getEdgeDataCloud () [0]);
    EXPECT_EQ (ed_0, mesh.getEdgeDataCloud () [1]);
    EXPECT_EQ (ed_0, mesh.getEdgeDataCloud () [2]);
    EXPECT_EQ (ed_2, mesh.getEdgeDataCloud () [3]);
    EXPECT_EQ (ed_2, mesh.getEdgeDataCloud () [4]);
  }

  {
    SCOPED_TRACE ("Mesh with half-edge data");
    MeshHE mesh;

    EXPECT_TRUE (mesh.addVertex ().isValid ());
    EXPECT_TRUE (mesh.addVertex ().isValid ());
    EXPECT_TRUE (mesh.addVertex ().isValid ());
    EXPECT_TRUE (mesh.addVertex ().isValid ());
    EXPECT_TRUE (mesh.addVertex ().isValid ());

    EXPECT_TRUE (mesh.addFace (vi_0, nd, nd, hed_0).isValid ());
    EXPECT_TRUE (mesh.addFace (vi_1, nd, nd, hed_1).isValid ());
    EXPECT_TRUE (mesh.addFace (vi_2, nd, nd, hed_2).isValid ());

    checkSizeElements (mesh, 5,     7, 3);
    checkSizeData     (mesh, 0, 14, 0, 0);

    EXPECT_EQ (hed_0, mesh.getHalfEdgeDataCloud () [ 0]);
    EXPECT_EQ (hed_0, mesh.getHalfEdgeDataCloud () [ 1]);
    EXPECT_EQ (hed_0, mesh.getHalfEdgeDataCloud () [ 2]);
    EXPECT_EQ (hed_0, mesh.getHalfEdgeDataCloud () [ 3]);
    EXPECT_EQ (hed_0, mesh.getHalfEdgeDataCloud () [ 4]);
    EXPECT_EQ (hed_0, mesh.getHalfEdgeDataCloud () [ 5]);
    EXPECT_EQ (hed_1, mesh.getHalfEdgeDataCloud () [ 6]);
    EXPECT_EQ (hed_1, mesh.getHalfEdgeDataCloud () [ 7]);
    EXPECT_EQ (hed_1, mesh.getHalfEdgeDataCloud () [ 8]);
    EXPECT_EQ (hed_1, mesh.getHalfEdgeDataCloud () [ 9]);
    EXPECT_EQ (hed_2, mesh.getHalfEdgeDataCloud () [10]);
    EXPECT_EQ (hed_2, mesh.getHalfEdgeDataCloud () [11]);
    EXPECT_EQ (hed_2, mesh.getHalfEdgeDataCloud () [12]);
    EXPECT_EQ (hed_2, mesh.getHalfEdgeDataCloud () [13]);

    mesh.deleteFace (FaceIndex (1));
    mesh.cleanUp ();

    checkSizeElements (mesh, 4,     5, 2);
    checkSizeData     (mesh, 0, 10, 0, 0);

    EXPECT_EQ (hed_0, mesh.getHalfEdgeDataCloud () [0]);
    EXPECT_EQ (hed_0, mesh.getHalfEdgeDataCloud () [1]);
    EXPECT_EQ (hed_0, mesh.getHalfEdgeDataCloud () [2]);
    EXPECT_EQ (hed_0, mesh.getHalfEdgeDataCloud () [3]);
    EXPECT_EQ (hed_0, mesh.getHalfEdgeDataCloud () [4]);
    EXPECT_EQ (hed_0, mesh.getHalfEdgeDataCloud () [5]);
    EXPECT_EQ (hed_2, mesh.getHalfEdgeDataCloud () [6]);
    EXPECT_EQ (hed_2, mesh.getHalfEdgeDataCloud () [7]);
    EXPECT_EQ (hed_2, mesh.getHalfEdgeDataCloud () [8]);
    EXPECT_EQ (hed_2, mesh.getHalfEdgeDataCloud () [9]);
  }

  {
    SCOPED_TRACE ("Mesh with face data");
    MeshF mesh;

    EXPECT_TRUE (mesh.addVertex ().isValid ());
    EXPECT_TRUE (mesh.addVertex ().isValid ());
    EXPECT_TRUE (mesh.addVertex ().isValid ());
    EXPECT_TRUE (mesh.addVertex ().isValid ());
    EXPECT_TRUE (mesh.addVertex ().isValid ());

    EXPECT_TRUE (mesh.addFace (vi_0, fd_0, nd, nd).isValid ());
    EXPECT_TRUE (mesh.addFace (vi_1, fd_1, nd, nd).isValid ());
    EXPECT_TRUE (mesh.addFace (vi_2, fd_2, nd, nd).isValid ());

    checkSizeElements (mesh, 5,    7, 3);
    checkSizeData     (mesh, 0, 0, 0, 3);

    EXPECT_EQ (fd_0, mesh.getFaceDataCloud () [0]);
    EXPECT_EQ (fd_1, mesh.getFaceDataCloud () [1]);
    EXPECT_EQ (fd_2, mesh.getFaceDataCloud () [2]);

    mesh.deleteFace (FaceIndex (1));
    mesh.cleanUp ();

    checkSizeElements (mesh, 4,    5, 2);
    checkSizeData     (mesh, 0, 0, 0, 2);

    EXPECT_EQ (fd_0, mesh.getFaceDataCloud () [0]);
    EXPECT_EQ (fd_2, mesh.getFaceDataCloud () [1]);
  }

  {
    SCOPED_TRACE ("Mesh with all data");
    MeshAD mesh;

    EXPECT_TRUE (mesh.addVertex (vd_0).isValid ());
    EXPECT_TRUE (mesh.addVertex (vd_1).isValid ());
    EXPECT_TRUE (mesh.addVertex (vd_2).isValid ());
    EXPECT_TRUE (mesh.addVertex (vd_3).isValid ());
    EXPECT_TRUE (mesh.addVertex (vd_4).isValid ());

    EXPECT_TRUE (mesh.addFace (vi_0, fd_0, ed_0, hed_0).isValid ());
    EXPECT_TRUE (mesh.addFace (vi_1, fd_1, ed_1, hed_1).isValid ());
    EXPECT_TRUE (mesh.addFace (vi_2, fd_2, ed_2, hed_2).isValid ());

    checkSizeElements (mesh, 5,     7, 3);
    checkSizeData     (mesh, 5, 14, 7, 3);

    EXPECT_EQ (vd_0, mesh.getVertexDataCloud () [0]);
    EXPECT_EQ (vd_1, mesh.getVertexDataCloud () [1]);
    EXPECT_EQ (vd_2, mesh.getVertexDataCloud () [2]);
    EXPECT_EQ (vd_3, mesh.getVertexDataCloud () [3]);
    EXPECT_EQ (vd_4, mesh.getVertexDataCloud () [4]);

    EXPECT_EQ (ed_0, mesh.getEdgeDataCloud () [0]);
    EXPECT_EQ (ed_0, mesh.getEdgeDataCloud () [1]);
    EXPECT_EQ (ed_0, mesh.getEdgeDataCloud () [2]);
    EXPECT_EQ (ed_1, mesh.getEdgeDataCloud () [3]);
    EXPECT_EQ (ed_1, mesh.getEdgeDataCloud () [4]);
    EXPECT_EQ (ed_2, mesh.getEdgeDataCloud () [5]);
    EXPECT_EQ (ed_2, mesh.getEdgeDataCloud () [6]);

    EXPECT_EQ (hed_0, mesh.getHalfEdgeDataCloud () [ 0]);
    EXPECT_EQ (hed_0, mesh.getHalfEdgeDataCloud () [ 1]);
    EXPECT_EQ (hed_0, mesh.getHalfEdgeDataCloud () [ 2]);
    EXPECT_EQ (hed_0, mesh.getHalfEdgeDataCloud () [ 3]);
    EXPECT_EQ (hed_0, mesh.getHalfEdgeDataCloud () [ 4]);
    EXPECT_EQ (hed_0, mesh.getHalfEdgeDataCloud () [ 5]);
    EXPECT_EQ (hed_1, mesh.getHalfEdgeDataCloud () [ 6]);
    EXPECT_EQ (hed_1, mesh.getHalfEdgeDataCloud () [ 7]);
    EXPECT_EQ (hed_1, mesh.getHalfEdgeDataCloud () [ 8]);
    EXPECT_EQ (hed_1, mesh.getHalfEdgeDataCloud () [ 9]);
    EXPECT_EQ (hed_2, mesh.getHalfEdgeDataCloud () [10]);
    EXPECT_EQ (hed_2, mesh.getHalfEdgeDataCloud () [11]);
    EXPECT_EQ (hed_2, mesh.getHalfEdgeDataCloud () [12]);
    EXPECT_EQ (hed_2, mesh.getHalfEdgeDataCloud () [13]);

    EXPECT_EQ (fd_0, mesh.getFaceDataCloud () [0]);
    EXPECT_EQ (fd_1, mesh.getFaceDataCloud () [1]);
    EXPECT_EQ (fd_2, mesh.getFaceDataCloud () [2]);

    mesh.deleteFace (FaceIndex (1));
    mesh.cleanUp ();

    checkSizeElements (mesh, 4,     5, 2);
    checkSizeData     (mesh, 4, 10, 5, 2);

    EXPECT_EQ (vd_0, mesh.getVertexDataCloud () [0]);
    EXPECT_EQ (vd_1, mesh.getVertexDataCloud () [1]);
    EXPECT_EQ (vd_2, mesh.getVertexDataCloud () [2]);
    EXPECT_EQ (vd_4, mesh.getVertexDataCloud () [3]);

    EXPECT_EQ (ed_0, mesh.getEdgeDataCloud () [0]);
    EXPECT_EQ (ed_0, mesh.getEdgeDataCloud () [1]);
    EXPECT_EQ (ed_0, mesh.getEdgeDataCloud () [2]);
    EXPECT_EQ (ed_2, mesh.getEdgeDataCloud () [3]);
    EXPECT_EQ (ed_2, mesh.getEdgeDataCloud () [4]);

    EXPECT_EQ (hed_0, mesh.getHalfEdgeDataCloud () [0]);
    EXPECT_EQ (hed_0, mesh.getHalfEdgeDataCloud () [1]);
    EXPECT_EQ (hed_0, mesh.getHalfEdgeDataCloud () [2]);
    EXPECT_EQ (hed_0, mesh.getHalfEdgeDataCloud () [3]);
    EXPECT_EQ (hed_0, mesh.getHalfEdgeDataCloud () [4]);
    EXPECT_EQ (hed_0, mesh.getHalfEdgeDataCloud () [5]);
    EXPECT_EQ (hed_2, mesh.getHalfEdgeDataCloud () [6]);
    EXPECT_EQ (hed_2, mesh.getHalfEdgeDataCloud () [7]);
    EXPECT_EQ (hed_2, mesh.getHalfEdgeDataCloud () [8]);
    EXPECT_EQ (hed_2, mesh.getHalfEdgeDataCloud () [9]);

    EXPECT_EQ (fd_0, mesh.getFaceDataCloud () [0]);
    EXPECT_EQ (fd_2, mesh.getFaceDataCloud () [1]);

    // Change the stored data.
    MeshAD::VertexDataCloud   vdc;
    MeshAD::HalfEdgeDataCloud hedc;
    MeshAD::EdgeDataCloud     edc;
    MeshAD::FaceDataCloud     fdc;
    vdc.push_back  (100);
    hedc.push_back (200);
    edc.push_back  (300);
    fdc.push_back  (400);

    // Wrong size
    EXPECT_FALSE (mesh.setVertexDataCloud   (vdc));
    EXPECT_FALSE (mesh.setHalfEdgeDataCloud (hedc));
    EXPECT_FALSE (mesh.setEdgeDataCloud     (edc));
    EXPECT_FALSE (mesh.setFaceDataCloud     (fdc));

    // Correct size
    for (unsigned int i=1; i< 4; ++i) vdc.push_back  (100+i);
    for (unsigned int i=1; i<10; ++i) hedc.push_back (200+i);
    for (unsigned int i=1; i< 5; ++i) edc.push_back  (300+i);
    for (unsigned int i=1; i< 2; ++i) fdc.push_back  (400+i);

    EXPECT_TRUE (mesh.setVertexDataCloud   (vdc));
    EXPECT_TRUE (mesh.setHalfEdgeDataCloud (hedc));
    EXPECT_TRUE (mesh.setEdgeDataCloud     (edc));
    EXPECT_TRUE (mesh.setFaceDataCloud     (fdc));

    MeshAD::VertexDataCloud&   vdc_new  = mesh.getVertexDataCloud ();
    MeshAD::HalfEdgeDataCloud& hedc_new = mesh.getHalfEdgeDataCloud ();
    MeshAD::EdgeDataCloud&     edc_new  = mesh.getEdgeDataCloud ();
    MeshAD::FaceDataCloud&     fdc_new  = mesh.getFaceDataCloud ();

    EXPECT_EQ (vdc.size () , vdc_new.size ());
    EXPECT_EQ (hedc.size (), hedc_new.size ());
    EXPECT_EQ (edc.size () , edc_new.size ());
    EXPECT_EQ (fdc.size () , fdc_new.size ());

    for (unsigned int i=0; i<vdc_new.size  (); ++i) EXPECT_EQ (vdc  [i], vdc_new  [i]) << "Index " << i;
    for (unsigned int i=0; i<hedc_new.size (); ++i) EXPECT_EQ (hedc [i], hedc_new [i]) << "Index " << i;
    for (unsigned int i=0; i<edc_new.size  (); ++i) EXPECT_EQ (edc  [i], edc_new  [i]) << "Index " << i;
    for (unsigned int i=0; i<fdc_new.size  (); ++i) EXPECT_EQ (fdc  [i], fdc_new  [i]) << "Index " << i;

    vdc_new  [0] = 0;
    hedc_new [0] = 1;
    edc_new  [0] = 2;
    fdc_new  [0] = 3;

    EXPECT_EQ (0, mesh.getVertexDataCloud   () [0]);
    EXPECT_EQ (1, mesh.getHalfEdgeDataCloud () [0]);
    EXPECT_EQ (2, mesh.getEdgeDataCloud     () [0]);
    EXPECT_EQ (3, mesh.getFaceDataCloud     () [0]);
  }
}

////////////////////////////////////////////////////////////////////////////////

int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
