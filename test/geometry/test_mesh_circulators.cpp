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

#include <pcl/geometry/triangle_mesh.h>
#include <pcl/memory.h>

#include "test_mesh_common_functions.h"

////////////////////////////////////////////////////////////////////////////////

class TestMeshCirculators : public ::testing::Test
{
  protected:

    using VertexIndex = pcl::geometry::VertexIndex;
    using HalfEdgeIndex = pcl::geometry::HalfEdgeIndex;
    using EdgeIndex = pcl::geometry::EdgeIndex;
    using FaceIndex = pcl::geometry::FaceIndex;

    using VertexIndices = std::vector<VertexIndex>;
    using HalfEdgeIndices = std::vector<HalfEdgeIndex>;
    using FaceIndices = std::vector<FaceIndex>;

    struct MeshTraits
    {
      using VertexData = pcl::geometry::NoData;
      using HalfEdgeData = pcl::geometry::NoData;
      using EdgeData = pcl::geometry::NoData;
      using FaceData = pcl::geometry::NoData;
      using IsManifold = std::true_type;
    };

    using Mesh = pcl::geometry::TriangleMesh<MeshTraits>;
    using VAVC = Mesh::VertexAroundVertexCirculator;
    using OHEAVC = Mesh::OutgoingHalfEdgeAroundVertexCirculator;
    using IHEAVC = Mesh::IncomingHalfEdgeAroundVertexCirculator;
    using FAVC = Mesh::FaceAroundVertexCirculator;
    using VAFC = Mesh::VertexAroundFaceCirculator;
    using IHEAFC = Mesh::InnerHalfEdgeAroundFaceCirculator;
    using OHEAFC = Mesh::OuterHalfEdgeAroundFaceCirculator;
    using FAFC = Mesh::FaceAroundFaceCirculator;

    //   1 - 6   //
    //  / \ / \  //
    // 2 - 0 - 5 //
    //  \ / \ /  //
    //   3 - 4   //
    void
    SetUp () override
    {
      for (int i=0; i<7; ++i) mesh_.addVertex ();

      VertexIndices vi;
      using VI = VertexIndex;
      vi.push_back (VI (0)); vi.push_back (VI (1)); vi.push_back (VI (2)); faces_.push_back (vi); vi.clear ();
      vi.push_back (VI (0)); vi.push_back (VI (2)); vi.push_back (VI (3)); faces_.push_back (vi); vi.clear ();
      vi.push_back (VI (0)); vi.push_back (VI (3)); vi.push_back (VI (4)); faces_.push_back (vi); vi.clear ();
      vi.push_back (VI (0)); vi.push_back (VI (4)); vi.push_back (VI (5)); faces_.push_back (vi); vi.clear ();
      vi.push_back (VI (0)); vi.push_back (VI (5)); vi.push_back (VI (6)); faces_.push_back (vi); vi.clear ();
      vi.push_back (VI (0)); vi.push_back (VI (6)); vi.push_back (VI (1)); faces_.push_back (vi); vi.clear ();
      for (std::size_t i=0; i<faces_.size (); ++i)
      {
        ASSERT_TRUE (mesh_.addFace (faces_ [i]).isValid ()) << "Face number " << i;
      }
      for (int i=1; i<=6; ++i)
      {
        expected_123456_.push_back (VertexIndex (i));
        expected_654321_.push_back (VertexIndex (7-i));
      }
    }

    Mesh mesh_;
    std::vector <VertexIndices> faces_;

    // expected sequences
    VertexIndices expected_123456_;
    VertexIndices expected_654321_;
  public:
    PCL_MAKE_ALIGNED_OPERATOR_NEW
};

////////////////////////////////////////////////////////////////////////////////

TEST_F (TestMeshCirculators, IsValid)
{
  VAVC   circ_0; EXPECT_FALSE (circ_0.isValid ());
  OHEAVC circ_1; EXPECT_FALSE (circ_1.isValid ());
  IHEAVC circ_2; EXPECT_FALSE (circ_2.isValid ());
  FAVC   circ_3; EXPECT_FALSE (circ_3.isValid ());
  VAFC   circ_4; EXPECT_FALSE (circ_4.isValid ());
  IHEAFC circ_5; EXPECT_FALSE (circ_5.isValid ());
  OHEAFC circ_6; EXPECT_FALSE (circ_6.isValid ());
  FAFC   circ_7; EXPECT_FALSE (circ_7.isValid ());
}

////////////////////////////////////////////////////////////////////////////////

TEST_F (TestMeshCirculators, VertexAroundVertexIncrement)
{
  VertexIndices actual;
  VAVC       circ     = mesh_.getVertexAroundVertexCirculator (VertexIndex (0));
  const VAVC circ_end = circ;
  ASSERT_TRUE (circ.isValid ());
  ASSERT_EQ   (circ, circ_end);
  int counter = 0;
  do
  {
    ASSERT_LE (++counter, 6); // Avoid infinite loop if connectivity is wrong
    actual.push_back (circ.getTargetIndex ());
  } while (++circ != circ_end);
  EXPECT_TRUE (isCircularPermutation (expected_654321_, actual));
}

////////////////////////////////////////////////////////////////////////////////

TEST_F (TestMeshCirculators, VertexAroundVertexDecrement)
{
  VertexIndices actual;
  VAVC       circ     = mesh_.getVertexAroundVertexCirculator (VertexIndex (0));
  const VAVC circ_end = circ;
  ASSERT_TRUE (circ.isValid ());
  ASSERT_EQ   (circ, circ_end);
  int counter = 0;
  do
  {
    ASSERT_LE (++counter, 6); // Avoid infinite loop if connectivity is wrong
    actual.push_back (circ.getTargetIndex ());
  } while (--circ != circ_end);
  EXPECT_TRUE (isCircularPermutation (expected_123456_, actual));
}

////////////////////////////////////////////////////////////////////////////////

TEST_F (TestMeshCirculators, OutgoingHalfEdgeAroundVertexIncrement)
{
  VertexIndices actual;
  OHEAVC       circ     = mesh_.getOutgoingHalfEdgeAroundVertexCirculator (VertexIndex (0));
  const OHEAVC circ_end = circ;
  ASSERT_TRUE (circ.isValid ());
  ASSERT_EQ   (circ, circ_end);
  int counter = 0;
  do
  {
    ASSERT_LE (++counter, 6); // Avoid infinite loop if connectivity is wrong
    const HalfEdgeIndex he = circ.getTargetIndex ();
    EXPECT_EQ (VertexIndex (0), mesh_.getOriginatingVertexIndex (he));
    actual.push_back (mesh_.getTerminatingVertexIndex (he));
  } while (++circ != circ_end);
  EXPECT_TRUE (isCircularPermutation (expected_654321_, actual));
}

////////////////////////////////////////////////////////////////////////////////

TEST_F (TestMeshCirculators, OutgoingHalfEdgeAroundVertexDecrement)
{
  VertexIndices actual;
  OHEAVC       circ     = mesh_.getOutgoingHalfEdgeAroundVertexCirculator (VertexIndex (0));
  const OHEAVC circ_end = circ;
  ASSERT_TRUE (circ.isValid ());
  ASSERT_EQ   (circ, circ_end);
  int counter = 0;
  do
  {
    ASSERT_LE (++counter, 6); // Avoid infinite loop if connectivity is wrong
    const HalfEdgeIndex he = circ.getTargetIndex ();
    EXPECT_EQ (VertexIndex (0), mesh_.getOriginatingVertexIndex (he));
    actual.push_back (mesh_.getTerminatingVertexIndex (he));
  } while (--circ != circ_end);
  EXPECT_TRUE (isCircularPermutation (expected_123456_, actual));
}

////////////////////////////////////////////////////////////////////////////////

TEST_F (TestMeshCirculators, IncomingHalfEdgeAroundVertexIncrement)
{
  VertexIndices actual;
  IHEAVC       circ     = mesh_.getIncomingHalfEdgeAroundVertexCirculator (VertexIndex (0));
  const IHEAVC circ_end = circ;
  ASSERT_TRUE (circ.isValid ());
  ASSERT_EQ   (circ, circ_end);
  int counter = 0;
  do
  {
    ASSERT_LE (++counter, 6); // Avoid infinite loop if connectivity is wrong
    const HalfEdgeIndex he = circ.getTargetIndex ();
    EXPECT_EQ (VertexIndex (0), mesh_.getTerminatingVertexIndex (he));
    actual.push_back (mesh_.getOriginatingVertexIndex (he));
  } while (++circ != circ_end);
  EXPECT_TRUE (isCircularPermutation (expected_654321_, actual));
}

////////////////////////////////////////////////////////////////////////////////

TEST_F (TestMeshCirculators, IncomingHalfEdgeAroundVertexDecrement)
{
  VertexIndices actual;
  IHEAVC       circ     = mesh_.getIncomingHalfEdgeAroundVertexCirculator (VertexIndex (0));
  const IHEAVC circ_end = circ;
  ASSERT_TRUE (circ.isValid ());
  ASSERT_EQ   (circ, circ_end);
  int counter = 0;
  do
  {
    ASSERT_LE (++counter, 6); // Avoid infinite loop if connectivity is wrong
    const HalfEdgeIndex he = circ.getTargetIndex ();
    EXPECT_EQ (VertexIndex (0), mesh_.getTerminatingVertexIndex (he));
    actual.push_back (mesh_.getOriginatingVertexIndex (he));
  } while (--circ != circ_end);
  EXPECT_TRUE (isCircularPermutation (expected_123456_, actual));
}

////////////////////////////////////////////////////////////////////////////////

TEST_F (TestMeshCirculators, FaceAroundVertexIncrement)
{
  std::vector <VertexIndices> actual;
  FAVC       circ_fav     = mesh_.getFaceAroundVertexCirculator (VertexIndex (0));
  const FAVC circ_fav_end = circ_fav;
  ASSERT_TRUE (circ_fav.isValid ());
  ASSERT_EQ   (circ_fav, circ_fav_end);
  int counter_v = 0;
  do
  {
    ASSERT_LE (++counter_v, 6); // Avoid infinite loop if connectivity is wrong
    VAFC       circ_vaf     = mesh_.getVertexAroundFaceCirculator (circ_fav.getTargetIndex ());
    const VAFC circ_vaf_end = circ_vaf;
    ASSERT_TRUE (circ_vaf.isValid ());
    ASSERT_EQ   (circ_vaf, circ_vaf_end);
    VertexIndices vi;
    int counter_f = 0;
    do
    {
      ASSERT_LE (++counter_f, 3); // Avoid infinite loop if connectivity is wrong
      vi.push_back (circ_vaf.getTargetIndex ());
    } while (++circ_vaf != circ_vaf_end);
    actual.push_back (vi);
  } while (++circ_fav != circ_fav_end);
  EXPECT_TRUE (isCircularPermutationVec (std::vector <VertexIndices> (faces_.rbegin (), faces_.rend ()), actual));
}

////////////////////////////////////////////////////////////////////////////////

TEST_F (TestMeshCirculators, FaceAroundVertexDecrement)
{
  std::vector <VertexIndices> actual;
  FAVC       circ_fav     = mesh_.getFaceAroundVertexCirculator (VertexIndex (0));
  const FAVC circ_fav_end = circ_fav;
  ASSERT_TRUE (circ_fav.isValid ());
  ASSERT_EQ   (circ_fav, circ_fav_end);
  int counter_v = 0;
  do
  {
    ASSERT_LE (++counter_v, 6); // Avoid infinite loop if connectivity is wrong
    VAFC       circ_vaf     = mesh_.getVertexAroundFaceCirculator (circ_fav.getTargetIndex ());
    const VAFC circ_vaf_end = circ_vaf;
    ASSERT_TRUE (circ_vaf.isValid ());
    ASSERT_EQ   (circ_vaf, circ_vaf_end);
     VertexIndices vi;
    int counter_f = 0;
    do
    {
      ASSERT_LE (++counter_f, 3); // Avoid infinite loop if connectivity is wrong
      vi.push_back (circ_vaf.getTargetIndex ());
    } while (++circ_vaf != circ_vaf_end);
    actual.push_back (vi);
  } while (--circ_fav != circ_fav_end);
  EXPECT_TRUE (isCircularPermutationVec (faces_, actual));
}

////////////////////////////////////////////////////////////////////////////////

TEST_F (TestMeshCirculators, VertexAroundFaceIncrement)
{
  VertexIndices actual;
  for (std::size_t i=0; i<mesh_.sizeFaces (); ++i)
  {
    VAFC       circ     = mesh_.getVertexAroundFaceCirculator (FaceIndex (i));
    const VAFC circ_end = circ;
    ASSERT_TRUE (circ.isValid ());
    ASSERT_EQ   (circ, circ_end);
    actual.clear ();
    int counter = 0;
    do
    {
      ASSERT_LE (++counter, 3); // Avoid infinite loop if connectivity is wrong
      actual.push_back (circ.getTargetIndex ());
    } while (++circ != circ_end);
    EXPECT_TRUE (isCircularPermutation (faces_ [i], actual)) << "Face number " << i;
  }
}

////////////////////////////////////////////////////////////////////////////////

TEST_F (TestMeshCirculators, VertexAroundFaceDecrement)
{
  VertexIndices actual;
  for (std::size_t i=0; i<mesh_.sizeFaces (); ++i)
  {
    VAFC       circ     = mesh_.getVertexAroundFaceCirculator (FaceIndex (i));
    const VAFC circ_end = circ;
    ASSERT_TRUE (circ.isValid ());
    ASSERT_EQ   (circ, circ_end);
    actual.clear ();
    int counter = 0;
    do
    {
      ASSERT_LE (++counter, 3); // Avoid infinite loop if connectivity is wrong
      actual.push_back (circ.getTargetIndex ());
    } while (--circ != circ_end);
    EXPECT_TRUE (isCircularPermutation (VertexIndices (faces_ [i].rbegin (), faces_ [i].rend ()), actual)) << "Face number " << i;
  }
}

////////////////////////////////////////////////////////////////////////////////

TEST_F (TestMeshCirculators, InnerHalfEdgeAroundFaceForAllFacesIncrement)
{
  VertexIndices actual;
  for (std::size_t i=0; i<mesh_.sizeFaces (); ++i)
  {
    IHEAFC       circ     = mesh_.getInnerHalfEdgeAroundFaceCirculator (FaceIndex (i));
    const IHEAFC circ_end = circ;
    ASSERT_TRUE (circ.isValid ());
    ASSERT_EQ   (circ, circ_end);
    actual.clear ();
    int counter = 0;
    do
    {
      ASSERT_LE (++counter, 3); // Avoid infinite loop if connectivity is wrong
      EXPECT_FALSE (mesh_.isBoundary (circ.getTargetIndex ()));
      actual.push_back (mesh_.getTerminatingVertexIndex (circ.getTargetIndex ()));
    } while (++circ != circ_end);
    EXPECT_TRUE (isCircularPermutation (faces_ [i], actual)) << "Face number " << i;
  }
}

////////////////////////////////////////////////////////////////////////////////

TEST_F (TestMeshCirculators, InnerHalfEdgeAroundFaceForAllFacesDecrement)
{
  VertexIndices actual;
  for (std::size_t i=0; i<mesh_.sizeFaces (); ++i)
  {
    IHEAFC       circ     = mesh_.getInnerHalfEdgeAroundFaceCirculator (FaceIndex (i));
    const IHEAFC circ_end = circ;
    ASSERT_TRUE (circ.isValid ());
    ASSERT_EQ   (circ, circ_end);
    actual.clear ();
    int counter = 0;
    do
    {
      ASSERT_LE (++counter, 3); // Avoid infinite loop if connectivity is wrong
      EXPECT_FALSE (mesh_.isBoundary (circ.getTargetIndex ()));
      actual.push_back (mesh_.getTerminatingVertexIndex (circ.getTargetIndex ()));
    } while (--circ != circ_end);
    EXPECT_TRUE (isCircularPermutation (VertexIndices (faces_ [i].rbegin (), faces_ [i].rend ()), actual)) << "Face number " << i;
  }
}

////////////////////////////////////////////////////////////////////////////////

TEST_F (TestMeshCirculators, InnerHalfEdgeAroundFaceForBoundaryIncrement)
{
  VertexIndices actual;
  IHEAFC       circ     = mesh_.getInnerHalfEdgeAroundFaceCirculator (mesh_.getOutgoingHalfEdgeIndex (VertexIndex (1)));
  const IHEAFC circ_end = circ;
  ASSERT_TRUE (circ.isValid ());
  ASSERT_EQ   (circ, circ_end);
  int counter = 0;
  do
  {
    ASSERT_LE (++counter, 6); // Avoid infinite loop if connectivity is wrong
    EXPECT_TRUE (mesh_.isBoundary (circ.getTargetIndex ()));
    actual.push_back (mesh_.getTerminatingVertexIndex (circ.getTargetIndex ()));
  } while (++circ != circ_end);
  EXPECT_TRUE (isCircularPermutation (expected_654321_, actual));
}

////////////////////////////////////////////////////////////////////////////////

TEST_F (TestMeshCirculators, InnerHalfEdgeAroundFaceForBoundaryDecrement)
{
  VertexIndices actual;
  IHEAFC       circ     = mesh_.getInnerHalfEdgeAroundFaceCirculator (mesh_.getOutgoingHalfEdgeIndex (VertexIndex (1)));
  const IHEAFC circ_end = circ;
  ASSERT_TRUE (circ.isValid ());
  ASSERT_EQ   (circ, circ_end);
  int counter = 0;
  do
  {
    ASSERT_LE (++counter, 6); // Avoid infinite loop if connectivity is wrong
    EXPECT_TRUE (mesh_.isBoundary (circ.getTargetIndex ()));
    actual.push_back (mesh_.getTerminatingVertexIndex (circ.getTargetIndex ()));
  } while (--circ != circ_end);
  EXPECT_TRUE (isCircularPermutation (expected_123456_, actual));
}

////////////////////////////////////////////////////////////////////////////////

TEST_F (TestMeshCirculators, OuterHalfEdgeAroundFaceIncrement)
{
  VertexIndices actual;
  for (std::size_t i=0; i<mesh_.sizeFaces (); ++i)
  {
    OHEAFC       circ     = mesh_.getOuterHalfEdgeAroundFaceCirculator (FaceIndex (i));
    const OHEAFC circ_end = circ;
    ASSERT_TRUE (circ.isValid ());
    ASSERT_EQ   (circ, circ_end);
    int num_boundary (0), num_not_boundary (0);
    actual.clear ();
    int counter = 0;
    do
    {
      ASSERT_LE (++counter, 3); // Avoid infinite loop if connectivity is wrong
      if (mesh_.isBoundary (circ.getTargetIndex ())) num_boundary     += 1;
      else                                          num_not_boundary += 1;
      actual.push_back (mesh_.getTerminatingVertexIndex (circ.getTargetIndex ()));
    } while (++circ != circ_end);
    EXPECT_EQ   (1, num_boundary)                                  << "Face number " << i;
    EXPECT_EQ   (2, num_not_boundary)                              << "Face number " << i;
    EXPECT_TRUE (isCircularPermutation (faces_ [i], actual)) << "Face number " << i;
  }
}

////////////////////////////////////////////////////////////////////////////////

TEST_F (TestMeshCirculators, OuterHalfEdgeAroundFaceDecrement)
{
  VertexIndices actual;
  for (std::size_t i=0; i<mesh_.sizeFaces (); ++i)
  {
    OHEAFC       circ     = mesh_.getOuterHalfEdgeAroundFaceCirculator (FaceIndex (i));
    const OHEAFC circ_end = circ;
    ASSERT_TRUE (circ.isValid ());
    ASSERT_EQ   (circ, circ_end);
    int num_boundary (0), num_not_boundary (0);
    actual.clear ();
    int counter = 0;
    do
    {
      ASSERT_LE (++counter, 3); // Avoid infinite loop if connectivity is wrong
      if (mesh_.isBoundary (circ.getTargetIndex ())) num_boundary     += 1;
      else                                          num_not_boundary += 1;
      actual.push_back (mesh_.getTerminatingVertexIndex (circ.getTargetIndex ()));
    } while (--circ != circ_end);
    EXPECT_EQ   (1, num_boundary)     << "Face number " << i;
    EXPECT_EQ   (2, num_not_boundary) << "Face number " << i;
    EXPECT_TRUE (isCircularPermutation (VertexIndices (faces_ [i].rbegin (), faces_ [i].rend ()), actual)) << "Face number " << i;
  }
}

////////////////////////////////////////////////////////////////////////////////

TEST_F (TestMeshCirculators, FaceAroundFaceIncrement)
{
  FaceIndices expected, actual;
  const int n = static_cast<int> (mesh_.sizeFaces ());
  for (int i = 0; i < static_cast<int> (mesh_.sizeFaces ()); ++i)
  {
    expected.clear ();
    expected.push_back (FaceIndex (i==(n-1) ?  0    : (i+1)));
    expected.push_back (FaceIndex (i== 0    ? (n-1) : (i-1)));
    expected.push_back (FaceIndex ());

    FAFC       circ     = mesh_.getFaceAroundFaceCirculator (FaceIndex (i));
    const FAFC circ_end = circ;
    ASSERT_TRUE (circ.isValid ());
    ASSERT_EQ   (circ, circ_end);
    actual.clear ();
    int counter = 0;
    do
    {
      ASSERT_LE (++counter, 3); // Avoid infinite loop if connectivity is wrong
      actual.push_back (circ.getTargetIndex ());
    } while (++circ != circ_end);
    EXPECT_TRUE (isCircularPermutation (expected, actual)) << "Face number " << i;
  }
}

////////////////////////////////////////////////////////////////////////////////

TEST_F (TestMeshCirculators, FaceAroundFaceDecrement)
{
  FaceIndices expected, actual;
  const int n = static_cast<int> (mesh_.sizeFaces ());
  for (int i = 0; i < static_cast<int> (mesh_.sizeFaces ()); ++i)
  {
    expected.clear ();
    expected.push_back (FaceIndex (i== 0    ? (n-1) : (i-1)));
    expected.push_back (FaceIndex (i==(n-1) ?  0    : (i+1)));
    expected.push_back (FaceIndex ());

    FAFC       circ     = mesh_.getFaceAroundFaceCirculator (FaceIndex (i));
    const FAFC circ_end = circ;
    ASSERT_TRUE (circ.isValid ());
    ASSERT_EQ   (circ, circ_end);
    actual.clear ();
    int counter = 0;
    do
    {
      ASSERT_LE (++counter, 3); // Avoid infinite loop if connectivity is wrong
      actual.push_back (circ.getTargetIndex ());
    } while (--circ != circ_end);
    EXPECT_TRUE (isCircularPermutation (expected, actual)) << "Face number " << i;
  }
}

////////////////////////////////////////////////////////////////////////////////

int
main (int argc, char** argv)
{  
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
