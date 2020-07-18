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

#include <pcl/test/gtest.h>

#include "test_mesh_common_functions.h"
#include <pcl/geometry/polygon_mesh.h>

////////////////////////////////////////////////////////////////////////////////

using NoData = pcl::geometry::NoData;
using TraitsV = pcl::geometry::DefaultMeshTraits <int   , NoData , NoData, NoData>;
using TraitsHE = pcl::geometry::DefaultMeshTraits <NoData, int    , NoData, NoData>;
using TraitsE = pcl::geometry::DefaultMeshTraits <NoData, NoData , int   , NoData>;
using TraitsF = pcl::geometry::DefaultMeshTraits <NoData, NoData , NoData, int   >;
using TraitsAD = pcl::geometry::DefaultMeshTraits <int   , int    , int   , int   >;

using MeshV = pcl::geometry::PolygonMesh<TraitsV>;
using MeshHE = pcl::geometry::PolygonMesh<TraitsHE>;
using MeshE = pcl::geometry::PolygonMesh<TraitsE>;
using MeshF = pcl::geometry::PolygonMesh<TraitsF>;
using MeshAD = pcl::geometry::PolygonMesh<TraitsAD>;

using VertexIndex = pcl::geometry::VertexIndex;
using HalfEdgeIndex = pcl::geometry::HalfEdgeIndex;
using EdgeIndex = pcl::geometry::EdgeIndex;
using FaceIndex = pcl::geometry::FaceIndex;

using VertexIndices = std::vector<VertexIndex>;
using HalfEdgeIndices = std::vector<HalfEdgeIndex>;
using FaceIndices = std::vector<FaceIndex>;

////////////////////////////////////////////////////////////////////////////////

/** \brief Check if the size of the mesh elements is correct. */
template <class MeshT> void
checkSizeElements (const MeshT& mesh, const std::size_t n_v, const std::size_t n_e, const std::size_t n_f)
{
  ASSERT_EQ (n_v, mesh.sizeVertices ());
  ASSERT_EQ (n_e, mesh.sizeEdges ());
  ASSERT_EQ (n_f, mesh.sizeFaces ());
}

////////////////////////////////////////////////////////////////////////////////

/** \brief Check if the size of the mesh data is correct. */
template <class MeshT> void
checkSizeData (const MeshT& mesh, const std::size_t n_v, const std::size_t n_he, const std::size_t n_e, const std::size_t n_f)
{
  ASSERT_EQ (n_v , mesh.getVertexDataCloud   ().size ());
  ASSERT_EQ (n_he, mesh.getHalfEdgeDataCloud ().size ());
  ASSERT_EQ (n_e , mesh.getEdgeDataCloud     ().size ());
  ASSERT_EQ (n_f , mesh.getFaceDataCloud     ().size ());
}

////////////////////////////////////////////////////////////////////////////////

TEST (TestMesh, MeshData)
{
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
  int vd_0  (10), vd_1  (11), vd_2  (12), vd_3 (13), vd_4 (14);
  int hed_0 (20), hed_1 (21), hed_2 (22);
  int ed_0  (30), ed_1  (31), ed_2  (32);
  int fd_0  (40), fd_1  (41), fd_2  (42);
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

    for (std::size_t i = 0; i < vdc_new.size  (); ++i) EXPECT_EQ (vdc  [i], vdc_new  [i]) << "Index " << i;
    for (std::size_t i = 0; i < hedc_new.size (); ++i) EXPECT_EQ (hedc [i], hedc_new [i]) << "Index " << i;
    for (std::size_t i = 0; i < edc_new.size  (); ++i) EXPECT_EQ (edc  [i], edc_new  [i]) << "Index " << i;
    for (std::size_t i = 0; i < fdc_new.size  (); ++i) EXPECT_EQ (fdc  [i], fdc_new  [i]) << "Index " << i;

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
