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

#include <gtest/gtest.h>
#include <pcl/geometry/triangle_mesh.h>

////////////////////////////////////////////////////////////////////////////////

class TestMeshCirculators : public ::testing::Test
{
  protected:

    typedef pcl::geometry::VertexIndex   VertexIndex;
    typedef pcl::geometry::HalfEdgeIndex HalfEdgeIndex;
    typedef pcl::geometry::EdgeIndex     EdgeIndex;
    typedef pcl::geometry::FaceIndex     FaceIndex;

    typedef std::vector <VertexIndex>   VertexIndices;
    typedef std::vector <HalfEdgeIndex> HalfEdgeIndices;
    typedef std::vector <FaceIndex>     FaceIndices;

    struct MeshTraits
    {
      typedef pcl::geometry::NoData VertexData;
      typedef pcl::geometry::NoData HalfEdgeData;
      typedef pcl::geometry::NoData EdgeData;
      typedef pcl::geometry::NoData FaceData;
      typedef boost::true_type      IsManifold;
    };

    typedef pcl::geometry::TriangleMesh <MeshTraits>     Mesh;
    typedef Mesh::VertexAroundVertexCirculator           VAVC;
    typedef Mesh::OutgoingHalfEdgeAroundVertexCirculator OHEAVC;
    typedef Mesh::IncomingHalfEdgeAroundVertexCirculator IHEAVC;
    typedef Mesh::FaceAroundVertexCirculator             FAVC;
    typedef Mesh::VertexAroundFaceCirculator             VAFC;
    typedef Mesh::InnerHalfEdgeAroundFaceCirculator      IHEAFC;
    typedef Mesh::OuterHalfEdgeAroundFaceCirculator      OHEAFC;
    typedef Mesh::FaceAroundFaceCirculator               FAFC;

    //   1 - 6   //
    //  / \ / \  //
    // 2 - 0 - 5 //
    //  \ / \ /  //
    //   3 - 4   //
    void
    SetUp ()
    {
      std::cout << "SetUp 0\n";
      for (int i=0; i<7; ++i) mesh_.addVertex ();

      VertexIndices vi;
      typedef VertexIndex VI;
      vi.push_back (VI (0)); vi.push_back (VI (1)); vi.push_back (VI (2)); faces_.push_back (vi); vi.clear ();
      vi.push_back (VI (0)); vi.push_back (VI (2)); vi.push_back (VI (3)); faces_.push_back (vi); vi.clear ();
      vi.push_back (VI (0)); vi.push_back (VI (3)); vi.push_back (VI (4)); faces_.push_back (vi); vi.clear ();
      vi.push_back (VI (0)); vi.push_back (VI (4)); vi.push_back (VI (5)); faces_.push_back (vi); vi.clear ();
      vi.push_back (VI (0)); vi.push_back (VI (5)); vi.push_back (VI (6)); faces_.push_back (vi); vi.clear ();
      vi.push_back (VI (0)); vi.push_back (VI (6)); vi.push_back (VI (1)); faces_.push_back (vi); vi.clear ();
      for (int i=0; i<faces_.size (); ++i)
      {
        std::cout << "SetUp 1 index " << i << "\n";
        ASSERT_TRUE (mesh_.addFace (faces_ [i]).isValid ()) << "Face number " << i;
      }
      std::cout << "SetUp 2\n";
      for (int i=1; i<=6; ++i)
      {
        std::cout << "SetUp 3 index " << i << "\n";
        expected_123456_.push_back (VertexIndex (i));
        expected_654321_.push_back (VertexIndex (7-i));
      }
      std::cout << "SetUp 4\n";
    }

    /** \brief Check if the input is a circular permutation of the expected (only clockwise).
      * \example [0 1 2 3] [1 2 3 0] [2 3 0 1] [3 0 1 2] are all equal.
      */
    template <class ContainerT> bool
    isCircularPermutation (const ContainerT& actual, const ContainerT& expected)
    {
      std::cout << "iCP 0" << std::endl;
      const unsigned int n = expected.size ();
      EXPECT_EQ (n, actual.size ());
      if (n != actual.size ()) return (false);
      std::cout << "iCP 1" << std::endl;
      for (unsigned int i=0; i<n; ++i)
      {
        bool all_equal = true;
        for (unsigned int j=0; j<n; ++j)
        {
          std::cout << "iCP 2: " << ((i+j)%n) << " " << j << "\n";
          if (actual [(i+j)%n] != expected [j])
          {
            all_equal = false;
          }
        }
        if (all_equal)
        {
          std::cout << "iCP 3" << std::endl;
          return (true);
        }
      }
      std::cout << "iCP 4" << std::endl;
      return (false);
    }

    /** \brief Check if both the inner and outer input vector are a circular permutation. */
    template <class ContainerT> bool
    isCircularPermutationVec (const std::vector <ContainerT> actual, const std::vector <ContainerT> expected)
    {
      std::cout<< "iCPV 0" << std::endl;
      const unsigned int n = expected.size ();
      EXPECT_EQ (n, actual.size ());
      if (n != actual.size ()) return (false);
std::cout<< "iCPV 1" << std::endl;
      for (unsigned int i=0; i<n; ++i)
      {
        bool all_equal = true;
        for (unsigned int j=0; j<n; ++j)
        {
          std::cout << "iCPV 2: " << j << " " << ((i+1)%n) << std::endl;
          if (!this->isCircularPermutation (expected [j], actual [(i+j)%n]))
          {
            all_equal = false;
          }
        }
        if (all_equal)
        {
          std::cout<< "iCPV 3" << std::endl;
          return (true);
        }
      }
      std::cout<< "iCPV 4" << std::endl;
      return (false);
    }

    Mesh mesh_;
    std::vector <VertexIndices> faces_;

    // expected sequences
    VertexIndices expected_123456_;
    VertexIndices expected_654321_;
};

////////////////////////////////////////////////////////////////////////////////

TEST_F (TestMeshCirculators, VertexAroundVertexIncrement)
{
  std::cout << "VertexAroundVertexIncrement" << std::endl;
  VertexIndices actual;
  std::cout << "VAVI 0\n";
  VAVC       circ     = mesh_.getVertexAroundVertexCirculator (VertexIndex (0));
  std::cout << "VAVI 1\n";
  const VAVC circ_end = circ;
  std::cout << "VAVI 2\n";
  int counter = 0;
  do
  {
    std::cout << "VAVI 3 " << counter << "\n";
    ASSERT_LE (++counter, 6); // Avoid infinite loop if connectivity is wrong
    actual.push_back (circ.getTargetIndex ());
    std::cout << "VAVI 4 " << counter << "\n";
  } while (++circ != circ_end);
  std::cout << "VAVI 5 " << counter << "\n";
  EXPECT_TRUE (this->isCircularPermutation (expected_654321_, actual));
  std::cout << "VertexAroundVertexIncrement end" << std::endl;
}

////////////////////////////////////////////////////////////////////////////////

TEST_F (TestMeshCirculators, VertexAroundVertexDecrement)
{
  std::cout << "VertexAroundVertexDecrement" << std::endl;
  VertexIndices actual;
  VAVC       circ     = mesh_.getVertexAroundVertexCirculator (VertexIndex (0));
  const VAVC circ_end = circ;
  int counter = 0;
  do
  {
    ASSERT_LE (++counter, 6); // Avoid infinite loop if connectivity is wrong
    actual.push_back (circ.getTargetIndex ());
  } while (--circ != circ_end);
  EXPECT_TRUE (this->isCircularPermutation (expected_123456_, actual));
  std::cout << "VertexAroundVertexDecrement end" << std::endl;
}

////////////////////////////////////////////////////////////////////////////////

TEST_F (TestMeshCirculators, OutgoingHalfEdgeAroundVertexIncrement)
{
  std::cout << "OutgoingHalfEdgeAroundVertexIncrement" << std::endl;
  VertexIndices actual;
  OHEAVC       circ     = mesh_.getOutgoingHalfEdgeAroundVertexCirculator (VertexIndex (0));
  const OHEAVC circ_end = circ;
  int counter = 0;
  do
  {
    ASSERT_LE (++counter, 6); // Avoid infinite loop if connectivity is wrong
    const HalfEdgeIndex he = circ.getTargetIndex ();
    EXPECT_EQ (VertexIndex (0), mesh_.getOriginatingVertexIndex (he));
    actual.push_back (mesh_.getTerminatingVertexIndex (he));
  } while (++circ != circ_end);
  EXPECT_TRUE (this->isCircularPermutation (expected_654321_, actual));
  std::cout << "OutgoingHalfEdgeAroundVertexIncrement end" << std::endl;
}

////////////////////////////////////////////////////////////////////////////////

TEST_F (TestMeshCirculators, OutgoingHalfEdgeAroundVertexDecrement)
{
  std::cout << "OutgoingHalfEdgeAroundVertexDecrement\n";
  VertexIndices actual;
  OHEAVC       circ     = mesh_.getOutgoingHalfEdgeAroundVertexCirculator (VertexIndex (0));
  const OHEAVC circ_end = circ;
  int counter = 0;
  do
  {
    ASSERT_LE (++counter, 6); // Avoid infinite loop if connectivity is wrong
    const HalfEdgeIndex he = circ.getTargetIndex ();
    EXPECT_EQ (VertexIndex (0), mesh_.getOriginatingVertexIndex (he));
    actual.push_back (mesh_.getTerminatingVertexIndex (he));
  } while (--circ != circ_end);
  EXPECT_TRUE (this->isCircularPermutation (expected_123456_, actual));
  std::cout << "OutgoingHalfEdgeAroundVertexDecrement end\n";
}

////////////////////////////////////////////////////////////////////////////////

TEST_F (TestMeshCirculators, IncomingHalfEdgeAroundVertexIncrement)
{
  std::cout << "IncomingHalfEdgeAroundVertexIncrement\n";
  VertexIndices actual;
  IHEAVC       circ     = mesh_.getIncomingHalfEdgeAroundVertexCirculator (VertexIndex (0));
  const IHEAVC circ_end = circ;
  int counter = 0;
  do
  {
    ASSERT_LE (++counter, 6); // Avoid infinite loop if connectivity is wrong
    const HalfEdgeIndex he = circ.getTargetIndex ();
    EXPECT_EQ (VertexIndex (0), mesh_.getTerminatingVertexIndex (he));
    actual.push_back (mesh_.getOriginatingVertexIndex (he));
  } while (++circ != circ_end);
  EXPECT_TRUE (this->isCircularPermutation (expected_654321_, actual));
  std::cout << "IncomingHalfEdgeAroundVertexIncrement end\n";
}

////////////////////////////////////////////////////////////////////////////////

TEST_F (TestMeshCirculators, IncomingHalfEdgeAroundVertexDecrement)
{
  std::cout << "IncomingHalfEdgeAroundVertexDecrement\n";
  VertexIndices actual;
  IHEAVC       circ     = mesh_.getIncomingHalfEdgeAroundVertexCirculator (VertexIndex (0));
  const IHEAVC circ_end = circ;
  int counter = 0;
  do
  {
    ASSERT_LE (++counter, 6); // Avoid infinite loop if connectivity is wrong
    const HalfEdgeIndex he = circ.getTargetIndex ();
    EXPECT_EQ (VertexIndex (0), mesh_.getTerminatingVertexIndex (he));
    actual.push_back (mesh_.getOriginatingVertexIndex (he));
  } while (--circ != circ_end);
  EXPECT_TRUE (this->isCircularPermutation (expected_123456_, actual));
  std::cout << "IncomingHalfEdgeAroundVertexDecrement end\n";
}

////////////////////////////////////////////////////////////////////////////////

TEST_F (TestMeshCirculators, FaceAroundVertexIncrement)
{
  std::cout << "FaceAroundVertexIncrement\n";
  std::vector <VertexIndices> actual;
  FAVC       circ_fav     = mesh_.getFaceAroundVertexCirculator (VertexIndex (0));
  const FAVC circ_fav_end = circ_fav;
  int counter_v = 0;
  do
  {
    ASSERT_LE (++counter_v, 6); // Avoid infinite loop if connectivity is wrong
    VAFC       circ_vaf     = mesh_.getVertexAroundFaceCirculator (circ_fav.getTargetIndex ());
    const VAFC circ_vaf_end = circ_vaf;
    VertexIndices vi;
    int counter_f = 0;
    do
    {
      ASSERT_LE (++counter_f, 3); // Avoid infinite loop if connectivity is wrong
      vi.push_back (circ_vaf.getTargetIndex ());
    } while (++circ_vaf != circ_vaf_end);
    actual.push_back (vi);
  } while (++circ_fav != circ_fav_end);
  EXPECT_TRUE (this->isCircularPermutationVec (std::vector <VertexIndices> (faces_.rbegin (), faces_.rend ()), actual));
  std::cout << "FaceAroundVertexIncrement end\n";
}

////////////////////////////////////////////////////////////////////////////////

TEST_F (TestMeshCirculators, FaceAroundVertexDecrement)
{
  std::cout << "FaceAroundVertexDecrement\n";
  std::vector <VertexIndices> actual;
  FAVC       circ_fav     = mesh_.getFaceAroundVertexCirculator (VertexIndex (0));
  const FAVC circ_fav_end = circ_fav;
  int counter_v = 0;
  do
  {
    ASSERT_LE (++counter_v, 6); // Avoid infinite loop if connectivity is wrong
    VAFC       circ_vaf     = mesh_.getVertexAroundFaceCirculator (circ_fav.getTargetIndex ());
    const VAFC circ_vaf_end = circ_vaf;
     VertexIndices vi;
    int counter_f = 0;
    do
    {
      ASSERT_LE (++counter_f, 3); // Avoid infinite loop if connectivity is wrong
      vi.push_back (circ_vaf.getTargetIndex ());
    } while (++circ_vaf != circ_vaf_end);
    actual.push_back (vi);
  } while (--circ_fav != circ_fav_end);
  EXPECT_TRUE (this->isCircularPermutationVec (faces_, actual));
  std::cout << "FaceAroundVertexDecrement end\n";
}

////////////////////////////////////////////////////////////////////////////////

TEST_F (TestMeshCirculators, VertexAroundFaceIncrement)
{
  std::cout << "VertexAroundFaceIncrement\n";
  VertexIndices actual;
  for (unsigned int i=0; i<mesh_.sizeFaces (); ++i)
  {
    VAFC       circ     = mesh_.getVertexAroundFaceCirculator (FaceIndex (i));
    const VAFC circ_end = circ;
    actual.clear ();
    int counter = 0;
    do
    {
      ASSERT_LE (++counter, 3); // Avoid infinite loop if connectivity is wrong
      actual.push_back (circ.getTargetIndex ());
    } while (++circ != circ_end);
    EXPECT_TRUE (this->isCircularPermutation (faces_ [i], actual)) << "Face number " << i;
  }
  std::cout << "VertexAroundFaceIncrement end\n";
}

////////////////////////////////////////////////////////////////////////////////

TEST_F (TestMeshCirculators, VertexAroundFaceDecrement)
{
  std::cout << "VertexAroundFaceDecrement\n";
  VertexIndices actual;
  for (unsigned int i=0; i<mesh_.sizeFaces (); ++i)
  {
    VAFC       circ     = mesh_.getVertexAroundFaceCirculator (FaceIndex (i));
    const VAFC circ_end = circ;
    actual.clear ();
    int counter = 0;
    do
    {
      ASSERT_LE (++counter, 3); // Avoid infinite loop if connectivity is wrong
      actual.push_back (circ.getTargetIndex ());
    } while (--circ != circ_end);
    EXPECT_TRUE (this->isCircularPermutation (VertexIndices (faces_ [i].rbegin (), faces_ [i].rend ()), actual)) << "Face number " << i;
  }
  std::cout << "VertexAroundFaceDecrement end\n";
}

////////////////////////////////////////////////////////////////////////////////

TEST_F (TestMeshCirculators, InnerHalfEdgeAroundFaceForAllFacesIncrement)
{
  std::cout << "InnerHalfEdgeAroundFaceForAllFacesIncrement\n";
  VertexIndices actual;
  for (unsigned int i=0; i<mesh_.sizeFaces (); ++i)
  {
    IHEAFC       circ     = mesh_.getInnerHalfEdgeAroundFaceCirculator (FaceIndex (i));
    const IHEAFC circ_end = circ;
    actual.clear ();
    int counter = 0;
    do
    {
      ASSERT_LE (++counter, 3); // Avoid infinite loop if connectivity is wrong
      EXPECT_FALSE (mesh_.isBoundary (circ.getTargetIndex ()));
      actual.push_back (mesh_.getTerminatingVertexIndex (circ.getTargetIndex ()));
    } while (++circ != circ_end);
    EXPECT_TRUE (this->isCircularPermutation (faces_ [i], actual)) << "Face number " << i;
  }
  std::cout << "InnerHalfEdgeAroundFaceForAllFacesIncrement end\n";
}

////////////////////////////////////////////////////////////////////////////////

TEST_F (TestMeshCirculators, InnerHalfEdgeAroundFaceForAllFacesDecrement)
{
  std::cout << "InnerHalfEdgeAroundFaceForAllFacesDecrement\n";
  VertexIndices actual;
  for (unsigned int i=0; i<mesh_.sizeFaces (); ++i)
  {
    IHEAFC       circ     = mesh_.getInnerHalfEdgeAroundFaceCirculator (FaceIndex (i));
    const IHEAFC circ_end = circ;
    actual.clear ();
    int counter = 0;
    do
    {
      ASSERT_LE (++counter, 3); // Avoid infinite loop if connectivity is wrong
      EXPECT_FALSE (mesh_.isBoundary (circ.getTargetIndex ()));
      actual.push_back (mesh_.getTerminatingVertexIndex (circ.getTargetIndex ()));
    } while (--circ != circ_end);
    EXPECT_TRUE (this->isCircularPermutation (VertexIndices (faces_ [i].rbegin (), faces_ [i].rend ()), actual)) << "Face number " << i;
  }
  std::cout << "InnerHalfEdgeAroundFaceForAllFacesDecrement end\n";
}

////////////////////////////////////////////////////////////////////////////////

TEST_F (TestMeshCirculators, InnerHalfEdgeAroundFaceForBoundaryIncrement)
{
  std::cout << "InnerHalfEdgeAroundFaceForBoundaryIncrement\n";
  VertexIndices actual;
  IHEAFC       circ     = mesh_.getInnerHalfEdgeAroundFaceCirculator (mesh_.getOutgoingHalfEdgeIndex (VertexIndex (1)));
  const IHEAFC circ_end = circ;
  int counter = 0;
  do
  {
    ASSERT_LE (++counter, 6); // Avoid infinite loop if connectivity is wrong
    EXPECT_TRUE (mesh_.isBoundary (circ.getTargetIndex ()));
    actual.push_back (mesh_.getTerminatingVertexIndex (circ.getTargetIndex ()));
  } while (++circ != circ_end);
  EXPECT_TRUE (this->isCircularPermutation (expected_654321_, actual));
  std::cout << "InnerHalfEdgeAroundFaceForBoundaryIncrement end\n";
}

////////////////////////////////////////////////////////////////////////////////

TEST_F (TestMeshCirculators, InnerHalfEdgeAroundFaceForBoundaryDecrement)
{
  std::cout << "InnerHalfEdgeAroundFaceForBoundaryDecrement\n";
  VertexIndices actual;
  IHEAFC       circ     = mesh_.getInnerHalfEdgeAroundFaceCirculator (mesh_.getOutgoingHalfEdgeIndex (VertexIndex (1)));
  const IHEAFC circ_end = circ;
  int counter = 0;
  do
  {
    ASSERT_LE (++counter, 6); // Avoid infinite loop if connectivity is wrong
    EXPECT_TRUE (mesh_.isBoundary (circ.getTargetIndex ()));
    actual.push_back (mesh_.getTerminatingVertexIndex (circ.getTargetIndex ()));
  } while (--circ != circ_end);
  EXPECT_TRUE (this->isCircularPermutation (expected_123456_, actual));
  std::cout << "InnerHalfEdgeAroundFaceForBoundaryDecrement end\n";
}

////////////////////////////////////////////////////////////////////////////////

TEST_F (TestMeshCirculators, OuterHalfEdgeAroundFaceIncrement)
{
  std::cout << "OuterHalfEdgeAroundFaceIncrement\n";
  VertexIndices actual;
  for (unsigned int i=0; i<mesh_.sizeFaces (); ++i)
  {
    OHEAFC       circ     = mesh_.getOuterHalfEdgeAroundFaceCirculator (FaceIndex (i));
    const OHEAFC circ_end = circ;
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
    EXPECT_TRUE (this->isCircularPermutation (faces_ [i], actual)) << "Face number " << i;
  }
  std::cout << "OuterHalfEdgeAroundFaceIncrement end\n";
}

////////////////////////////////////////////////////////////////////////////////

TEST_F (TestMeshCirculators, OuterHalfEdgeAroundFaceDecrement)
{
  std::cout << "OuterHalfEdgeAroundFaceDecrement\n";
  VertexIndices actual;
  for (unsigned int i=0; i<mesh_.sizeFaces (); ++i)
  {
    OHEAFC       circ     = mesh_.getOuterHalfEdgeAroundFaceCirculator (FaceIndex (i));
    const OHEAFC circ_end = circ;
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
    EXPECT_TRUE (this->isCircularPermutation (VertexIndices (faces_ [i].rbegin (), faces_ [i].rend ()), actual)) << "Face number " << i;
  }
  std::cout << "OuterHalfEdgeAroundFaceDecrement end\n";
}

////////////////////////////////////////////////////////////////////////////////

TEST_F (TestMeshCirculators, FaceAroundFaceIncrement)
{
  std::cout << "FaceAroundFaceIncrement\n";
  FaceIndices expected, actual;
  const int n = mesh_.sizeFaces ();
  for (unsigned int i=0; i<mesh_.sizeFaces (); ++i)
  {
    expected.clear ();
    expected.push_back (FaceIndex (i==(n-1) ?  0    : (i+1)));
    expected.push_back (FaceIndex (i== 0    ? (n-1) : (i-1)));
    expected.push_back (FaceIndex ());

    FAFC       circ     = mesh_.getFaceAroundFaceCirculator (FaceIndex (i));
    const FAFC circ_end = circ;
    actual.clear ();
    int counter = 0;
    do
    {
      ASSERT_LE (++counter, 3); // Avoid infinite loop if connectivity is wrong
      actual.push_back (circ.getTargetIndex ());
    } while (++circ != circ_end);
    EXPECT_TRUE (this->isCircularPermutation (expected, actual)) << "Face number " << i;
  }
  std::cout << "FaceAroundFaceIncrement end\n";
}

////////////////////////////////////////////////////////////////////////////////

TEST_F (TestMeshCirculators, FaceAroundFaceDecrement)
{
  std::cout << "FaceAroundFaceDecrement\n";
  FaceIndices expected, actual;
  const int n = mesh_.sizeFaces ();
  for (unsigned int i=0; i<mesh_.sizeFaces (); ++i)
  {
    expected.clear ();
    expected.push_back (FaceIndex (i== 0    ? (n-1) : (i-1)));
    expected.push_back (FaceIndex (i==(n-1) ?  0    : (i+1)));
    expected.push_back (FaceIndex ());

    FAFC       circ     = mesh_.getFaceAroundFaceCirculator (FaceIndex (i));
    const FAFC circ_end = circ;
    actual.clear ();
    int counter = 0;
    do
    {
      ASSERT_LE (++counter, 3); // Avoid infinite loop if connectivity is wrong
      actual.push_back (circ.getTargetIndex ());
    } while (--circ != circ_end);
    EXPECT_TRUE (this->isCircularPermutation (expected, actual)) << "Face number " << i;
  }
  std::cout << "FaceAroundFaceDecrement end\n";
}

////////////////////////////////////////////////////////////////////////////////

int
main (int argc, char** argv)
{  
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
