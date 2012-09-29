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

#ifndef PCL_GEOMETRY_TRIANGLE_MESH_HPP
#define PCL_GEOMETRY_TRIANGLE_MESH_HPP

#include <pcl/geometry/impl/mesh_topology.hpp>

namespace pcl
{

  // Empty...Data is defined in mesh_base.hpp
  template <bool is_manifold,
            class VertexDataT   = pcl::EmptyVertexData,
            class FaceDataT     = pcl::EmptyFaceData,
            class HalfEdgeDataT = pcl::EmptyHalfEdgeData>
  class TriangleMesh : public pcl::MeshTopology <is_manifold, VertexDataT, FaceDataT, HalfEdgeDataT>
  {
    public:

      typedef pcl::TriangleMesh <is_manifold, VertexDataT, FaceDataT, HalfEdgeDataT> Self;
      typedef pcl::MeshTopology <is_manifold, VertexDataT, FaceDataT, HalfEdgeDataT> Base;

      typedef typename Base::ManifoldMeshTag    ManifoldMeshTag;
      typedef typename Base::NonManifoldMeshTag NonManifoldMeshTag;
      typedef typename Base::IsManifold         IsManifold;

      typedef typename Base::VertexData   VertexData;
      typedef typename Base::HalfEdgeData HalfEdgeData;
      typedef typename Base::FaceData     FaceData;

      typedef typename Base::VertexIndex       VertexIndex;
      typedef typename Base::HalfEdgeIndex     HalfEdgeIndex;
      typedef typename Base::FaceIndex         FaceIndex;
      typedef std::pair <FaceIndex, FaceIndex> FaceIndexPair;

      typedef typename Base::VertexIndexes   VertexIndexes;
      typedef typename Base::HalfEdgeIndexes HalfEdgeIndexes;
      typedef typename Base::FaceIndexes     FaceIndexes;

      typedef typename Base::Vertex   Vertex;
      typedef typename Base::HalfEdge HalfEdge;
      typedef typename Base::Face     Face;

      typedef typename Base::Vertexes  Vertexes;
      typedef typename Base::HalfEdges HalfEdges;
      typedef typename Base::Faces     Faces;

      typedef typename Base::SizeType SizeType;

      typedef typename Base::VertexIterator   VertexIterator;
      typedef typename Base::HalfEdgeIterator HalfEdgeIterator;
      typedef typename Base::FaceIterator     FaceIterator;

      typedef typename Base::VertexConstIterator   VertexConstIterator;
      typedef typename Base::HalfEdgeConstIterator HalfEdgeConstIterator;
      typedef typename Base::FaceConstIterator     FaceConstIterator;

      typedef typename Base::VertexIndexIterator   VertexIndexIterator;
      typedef typename Base::HalfEdgeIndexIterator HalfEdgeIndexIterator;
      typedef typename Base::FaceIndexIterator     FaceIndexIterator;

      typedef typename Base::VertexIndexConstIterator   VertexIndexConstIterator;
      typedef typename Base::HalfEdgeIndexConstIterator HalfEdgeIndexConstIterator;
      typedef typename Base::FaceIndexConstIterator     FaceIndexConstIterator;

      typedef typename Base::VertexAroundVertexCirculator           VertexAroundVertexCirculator;
      typedef typename Base::OutgoingHalfEdgeAroundVertexCirculator OutgoingHalfEdgeAroundVertexCirculator;
      typedef typename Base::IncomingHalfEdgeAroundVertexCirculator IncomingHalfEdgeAroundVertexCirculator;
      typedef typename Base::FaceAroundVertexCirculator             FaceAroundVertexCirculator;
      typedef typename Base::VertexAroundFaceCirculator             VertexAroundFaceCirculator;
      typedef typename Base::InnerHalfEdgeAroundFaceCirculator      InnerHalfEdgeAroundFaceCirculator;
      typedef typename Base::OuterHalfEdgeAroundFaceCirculator      OuterHalfEdgeAroundFaceCirculator;
      typedef typename Base::FaceAroundFaceCirculator               FaceAroundFaceCirculator;
      typedef typename Base::HalfEdgeAroundBoundaryCirculator       HalfEdgeAroundBoundaryCirculator;

      typedef typename Base::VertexAroundVertexConstCirculator           VertexAroundVertexConstCirculator;
      typedef typename Base::OutgoingHalfEdgeAroundVertexConstCirculator OutgoingHalfEdgeAroundVertexConstCirculator;
      typedef typename Base::IncomingHalfEdgeAroundVertexConstCirculator IncomingHalfEdgeAroundVertexConstCirculator;
      typedef typename Base::FaceAroundVertexConstCirculator             FaceAroundVertexConstCirculator;
      typedef typename Base::VertexAroundFaceConstCirculator             VertexAroundFaceConstCirculator;
      typedef typename Base::InnerHalfEdgeAroundFaceConstCirculator      InnerHalfEdgeAroundFaceConstCirculator;
      typedef typename Base::OuterHalfEdgeAroundFaceConstCirculator      OuterHalfEdgeAroundFaceConstCirculator;
      typedef typename Base::FaceAroundFaceConstCirculator               FaceAroundFaceConstCirculator;
      typedef typename Base::HalfEdgeAroundBoundaryConstCirculator       HalfEdgeAroundBoundaryConstCirculator;

    public:

      TriangleMesh ()
        : Base ()
      {
      }

    public:

      //////////////////////////////////////////////////////////////////////////
      // addFace
      //////////////////////////////////////////////////////////////////////////

      FaceIndex
      addFace (const VertexIndex&  idx_v_0,
               const VertexIndex&  idx_v_1,
               const VertexIndex&  idx_v_2,
               const FaceData&     face_data = FaceData ())
      {
        HalfEdgeIndex idx_he_01, idx_he_12, idx_he_20;
        HalfEdgeIndex idx_he_10, idx_he_21, idx_he_02;

        if (Base::getElement (idx_v_0).isIsolated () && Base::getElement (idx_v_1).isIsolated () && Base::getElement (idx_v_2).isIsolated ())
        {
          if (idx_v_0==idx_v_1 || idx_v_0==idx_v_2 || idx_v_1==idx_v_2)
          {
            return FaceIndex ();
          }

          Base::addHalfEdgePair (idx_v_0,idx_v_1, HalfEdgeData (),HalfEdgeData (), idx_he_01,idx_he_10);
          Base::addHalfEdgePair (idx_v_1,idx_v_2, HalfEdgeData (),HalfEdgeData (), idx_he_12,idx_he_21);
          Base::addHalfEdgePair (idx_v_2,idx_v_0, HalfEdgeData (),HalfEdgeData (), idx_he_20,idx_he_02);

          Base::connectHalfEdges (true,true, idx_he_01,idx_he_10, idx_he_12,idx_he_21, idx_v_1);
          Base::connectHalfEdges (true,true, idx_he_12,idx_he_21, idx_he_20,idx_he_02, idx_v_2);
          Base::connectHalfEdges (true,true, idx_he_20,idx_he_02, idx_he_01,idx_he_10, idx_v_0);

          return (this->connectFace (face_data, idx_he_01,idx_he_12,idx_he_20));
        }

        // Check for topological errors
        bool is_new_01 = true;
        bool is_new_12 = true;
        bool is_new_20 = true;

        if (!Base::firstTopologyCheck (idx_v_0,idx_v_1, idx_he_01, is_new_01) ||
            !Base::firstTopologyCheck (idx_v_1,idx_v_2, idx_he_12, is_new_12) ||
            !Base::firstTopologyCheck (idx_v_2,idx_v_0, idx_he_20, is_new_20))
        {
          return (FaceIndex ());
        }

        if (!Base::secondTopologyCheck (is_new_01,is_new_12, Base::getElement (idx_v_1).isIsolated ()) ||
            !Base::secondTopologyCheck (is_new_12,is_new_20, Base::getElement (idx_v_2).isIsolated ()) ||
            !Base::secondTopologyCheck (is_new_20,is_new_01, Base::getElement (idx_v_0).isIsolated ()))
        {
          return (FaceIndex ());
        }

        // Reconnect the existing half-edges if necessary
        if (!is_new_01 && !is_new_12) Base::makeAdjacent (idx_he_01, idx_he_12);
        if (!is_new_12 && !is_new_20) Base::makeAdjacent (idx_he_12, idx_he_20);
        if (!is_new_20 && !is_new_01) Base::makeAdjacent (idx_he_20, idx_he_01);

        // Add the new half-edges if needed
        if (is_new_01) Base::addHalfEdgePair (idx_v_0,idx_v_1, HalfEdgeData (),HalfEdgeData (), idx_he_01,idx_he_10);
        else           idx_he_10 = Base::getElement (idx_he_01).getOppositeHalfEdgeIndex ();

        if (is_new_12) Base::addHalfEdgePair (idx_v_1,idx_v_2, HalfEdgeData (),HalfEdgeData (), idx_he_12,idx_he_21);
        else           idx_he_21 = Base::getElement (idx_he_12).getOppositeHalfEdgeIndex ();

        if (is_new_20) Base::addHalfEdgePair (idx_v_2,idx_v_0, HalfEdgeData (),HalfEdgeData (), idx_he_20,idx_he_02);
        else           idx_he_02 = Base::getElement (idx_he_20).getOppositeHalfEdgeIndex ();

        // Connect the half-edges and vertexes
        Base::connectHalfEdges (is_new_01,is_new_12, idx_he_01,idx_he_10, idx_he_12,idx_he_21, idx_v_1);
        Base::connectHalfEdges (is_new_12,is_new_20, idx_he_12,idx_he_21, idx_he_20,idx_he_02, idx_v_2);
        Base::connectHalfEdges (is_new_20,is_new_01, idx_he_20,idx_he_02, idx_he_01,idx_he_10, idx_v_0);

        // Connect the face
        return (this->connectFace (face_data, idx_he_01,idx_he_12,idx_he_20));
      }

      //////////////////////////////////////////////////////////////////////////
      // addFace
      //////////////////////////////////////////////////////////////////////////

      // 3 - 2      3 - 2
      // | / |  or  | \ |
      // 0 - 1      0 - 1
      FaceIndexPair
      addFace (const VertexIndex&  idx_v_0,
               const VertexIndex&  idx_v_1,
               const VertexIndex&  idx_v_2,
               const VertexIndex&  idx_v_3,
               const FaceData&     face_data = FaceData ())
      {
        // Try to add two faces
        // 3 - 2
        // | / |
        // 0 - 1
        FaceIndex idx_face_0 = this->addFace (idx_v_0, idx_v_1, idx_v_2, face_data);
        FaceIndex idx_face_1 = this->addFace (idx_v_0, idx_v_2, idx_v_3, face_data);

        if (idx_face_0.isValid ())
        {
          return (std::make_pair (idx_face_0, idx_face_1));
        }
        else if (idx_face_1.isValid ())
        {
          idx_face_0 = this->addFace (idx_v_0, idx_v_1, idx_v_2, face_data); // might be possible to add now
          return (std::make_pair (idx_face_1, idx_face_0));
        }

        // Try to add two faces
        // 3 - 2
        // | \ |
        // 0 - 1
        idx_face_0 = this->addFace (idx_v_1, idx_v_2, idx_v_3, face_data);
        idx_face_1 = this->addFace (idx_v_0, idx_v_1, idx_v_3, face_data);

        if (idx_face_0.isValid ())
        {
          return (std::make_pair (idx_face_0, idx_face_1));
        }
        else if (idx_face_1.isValid ())
        {
          idx_face_0 = this->addFace (idx_v_1, idx_v_2, idx_v_3, face_data); // might be possible to add now
          return (std::make_pair (idx_face_1, idx_face_0));
        }

        // Connect the triangle pair if possible
        HalfEdgeIndex idx_he_01, idx_he_12, idx_he_23, idx_he_30;

        // Check manifoldness
        bool is_new_01 = true;
        bool is_new_12 = true;
        bool is_new_23 = true;
        bool is_new_30 = true;

        if (!Base::firstTopologyCheck (idx_v_0,idx_v_1, idx_he_01, is_new_01) ||
            !Base::firstTopologyCheck (idx_v_1,idx_v_2, idx_he_12, is_new_12) ||
            !Base::firstTopologyCheck (idx_v_2,idx_v_3, idx_he_23, is_new_23) ||
            !Base::firstTopologyCheck (idx_v_3,idx_v_0, idx_he_30, is_new_30))
        {
          return (std::make_pair (FaceIndex (), FaceIndex ()));
        }

        // Connect the triangle pair
        if (!is_new_01 && is_new_12 && !is_new_23 && is_new_30)
        {
          return (this->connectTrianglePair (idx_he_01, idx_he_23, idx_v_0, idx_v_1, idx_v_2, idx_v_3, face_data));
        }
        else if (is_new_01 && !is_new_12 && is_new_23 && !is_new_30)
        {
          return (this->connectTrianglePair (idx_he_12, idx_he_30, idx_v_1, idx_v_2, idx_v_3, idx_v_0, face_data));
        }
        else
        {
          assert (true); // This should not happen!
          return (std::make_pair (FaceIndex (), FaceIndex ()));
        }
      }

    private:

      //////////////////////////////////////////////////////////////////////////
      // connectTrianglePair
      //////////////////////////////////////////////////////////////////////////

      // d - c
      // | / |
      // a - b
      FaceIndexPair
      connectTrianglePair (const HalfEdgeIndex& idx_he_ab,
                           const HalfEdgeIndex& idx_he_cd,
                           const VertexIndex&   idx_v_a,
                           const VertexIndex&   idx_v_b,
                           const VertexIndex&   idx_v_c,
                           const VertexIndex&   idx_v_d,
                           const FaceData&      face_data)
      {
        // Add new half-edges
        HalfEdgeIndex idx_he_bc, idx_he_cb;
        HalfEdgeIndex idx_he_da, idx_he_ad;
        HalfEdgeIndex idx_he_ca, idx_he_ac;

        Base::addHalfEdgePair (idx_v_b,idx_v_c, HalfEdgeData (),HalfEdgeData (), idx_he_bc,idx_he_cb);
        Base::addHalfEdgePair (idx_v_d,idx_v_a, HalfEdgeData (),HalfEdgeData (), idx_he_da,idx_he_ad);
        Base::addHalfEdgePair (idx_v_c,idx_v_a, HalfEdgeData (),HalfEdgeData (), idx_he_ca,idx_he_ac);

        // Get the existing half-edges
        HalfEdge& he_ab = Base::getElement (idx_he_ab);
        HalfEdge& he_cd = Base::getElement (idx_he_cd);

        const HalfEdgeIndex idx_he_ab_boundary_prev = he_ab.getPrevHalfEdgeIndex (); // No reference!
        const HalfEdgeIndex idx_he_ab_boundary_next = he_ab.getNextHalfEdgeIndex (); // No reference!

        const HalfEdgeIndex idx_he_cd_boundary_prev = he_cd.getPrevHalfEdgeIndex (); // No reference!
        const HalfEdgeIndex idx_he_cd_boundary_next = he_cd.getNextHalfEdgeIndex (); // No reference!

        // Connect the boundary half-edges
        Base::connectPrevNext (idx_he_ab_boundary_prev, idx_he_ad);
        Base::connectPrevNext (idx_he_ad, idx_he_cd_boundary_next);
        Base::connectPrevNext (idx_he_cd_boundary_prev, idx_he_cb);
        Base::connectPrevNext (idx_he_cb, idx_he_ab_boundary_next);

        // Connect the inner half-edges
        Base::connectPrevNext (idx_he_ab, idx_he_bc);
        Base::connectPrevNext (idx_he_bc, idx_he_ca);
        Base::connectPrevNext (idx_he_ca, idx_he_ab);

        Base::connectPrevNext (idx_he_ac, idx_he_cd);
        Base::connectPrevNext (idx_he_cd, idx_he_da);
        Base::connectPrevNext (idx_he_da, idx_he_ac);

        // Connect the vertexes to the boundary half-edges
        Base::getElement (idx_v_a).setOutgoingHalfEdgeIndex (idx_he_ad);
        Base::getElement (idx_v_b).setOutgoingHalfEdgeIndex (idx_he_ab_boundary_next);
        Base::getElement (idx_v_c).setOutgoingHalfEdgeIndex (idx_he_cb);
        Base::getElement (idx_v_d).setOutgoingHalfEdgeIndex (idx_he_cd_boundary_next);

        // Add and connect the faces
        return (std::make_pair (this->connectFace (face_data, idx_he_ab, idx_he_bc, idx_he_ca),
                                this->connectFace (face_data, idx_he_ac, idx_he_cd, idx_he_da)));
      }

      //////////////////////////////////////////////////////////////////////////
      // connectFace
      //////////////////////////////////////////////////////////////////////////

      FaceIndex
      connectFace (const FaceData&      face_data,
                   const HalfEdgeIndex& idx_he_ab,
                   const HalfEdgeIndex& idx_he_bc,
                   const HalfEdgeIndex& idx_he_ca)
      {
        // Add and connect the face
        const FaceIndex idx_face = Base::pushBackFace (face_data, idx_he_ca);

        Base::getElement (idx_he_ab).setFaceIndex (idx_face);
        Base::getElement (idx_he_bc).setFaceIndex (idx_face);
        Base::getElement (idx_he_ca).setFaceIndex (idx_face);

        return (idx_face);
      }
  };

} // End namespace pcl

#endif // PCL_GEOMETRY_TRIANGLE_MESH_HPP
