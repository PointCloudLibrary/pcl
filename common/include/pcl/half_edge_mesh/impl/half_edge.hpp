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

#ifndef HALF_EDGE_HPP
#define HALF_EDGE_HPP

namespace pcl
{

  template <class HalfEdgeDataT, class MeshT>
  class HalfEdge : public HalfEdgeDataT
  {
    public:

      typedef pcl::HalfEdge <HalfEdgeDataT, MeshT> Self;

      typedef HalfEdgeDataT HalfEdgeData;
      typedef MeshT         Mesh;

      typedef typename Mesh::Vertex        Vertex;
      typedef typename Mesh::VertexIndex   VertexIndex;
      typedef typename Mesh::HalfEdgeIndex HalfEdgeIndex;
      typedef typename Mesh::Face          Face;
      typedef typename Mesh::FaceIndex     FaceIndex;

    public:

      HalfEdge (const HalfEdgeData&  half_edge_data         = HalfEdgeData  (),
                const VertexIndex&   idx_terminating_vertex = VertexIndex   (),
                const HalfEdgeIndex& idx_opposite_half_edge = HalfEdgeIndex (),
                const HalfEdgeIndex& idx_next_half_edge     = HalfEdgeIndex (),
                const HalfEdgeIndex& idx_prev_half_edge     = HalfEdgeIndex (),
                const FaceIndex&     idx_face               = FaceIndex ())
        : HalfEdgeData            (half_edge_data),
          is_deleted_             (false),
          idx_terminating_vertex_ (idx_terminating_vertex),
          idx_opposite_half_edge_ (idx_opposite_half_edge),
          idx_next_half_edge_     (idx_next_half_edge),
          idx_prev_half_edge_     (idx_prev_half_edge),
          idx_face_               (idx_face)
      {
      }

      HalfEdge (const Self& other)
        : HalfEdgeData            (other),
          is_deleted_             (other.getDeleted ()),
          idx_terminating_vertex_ (other.getTerminatingVertexIndex ()),
          idx_opposite_half_edge_ (other.getOppositeHalfEdgeIndex ()),
          idx_next_half_edge_     (other.getNextHalfEdgeIndex ()),
          idx_prev_half_edge_     (other.getPrevHalfEdgeIndex ()),
          idx_face_               (other.getFaceIndex ())
      {
      }

    public:

      //////////////////////////////////////////////////////////////////////////
      // TerminatingVertex
      //////////////////////////////////////////////////////////////////////////

      const VertexIndex&
      getTerminatingVertexIndex () const
      {
        return (idx_terminating_vertex_);
      }

      VertexIndex&
      getTerminatingVertexIndex ()
      {
        return (idx_terminating_vertex_);
      }

      void
      setTerminatingVertexIndex (const VertexIndex& idx_terminating_vertex)
      {
        idx_terminating_vertex_ = idx_terminating_vertex;
      }

      const Vertex&
      getTerminatingVertex (const Mesh& mesh) const
      {
        return (mesh.getVertex (this->getTerminatingVertexIndex ()));
      }

      Vertex&
      getTerminatingVertex (Mesh& mesh)
      {
        return (mesh.getVertex (this->getTerminatingVertexIndex ()));
      }

      //////////////////////////////////////////////////////////////////////////
      // OriginatingVertex
      //////////////////////////////////////////////////////////////////////////

      const VertexIndex&
      getOriginatingVertexIndex (const Mesh& mesh) const
      {
        return (this->getOppositeHalfEdge (mesh).getTerminatingVertexIndex ());
      }

      VertexIndex&
      getOriginatingVertexIndex (Mesh& mesh)
      {
        return (this->getOppositeHalfEdge (mesh).getTerminatingVertexIndex ());
      }

      const Vertex&
      getOriginatingVertex (const Mesh& mesh) const
      {
        return (this->getOppositeHalfEdge (mesh).getTerminatingVertex (mesh));
      }

      Vertex&
      getOriginatingVertex (Mesh& mesh)
      {
        return (this->getOppositeHalfEdge (mesh).getTerminatingVertex (mesh));
      }

      //////////////////////////////////////////////////////////////////////////
      // OppositeHalfEdge
      //////////////////////////////////////////////////////////////////////////

      const HalfEdgeIndex&
      getOppositeHalfEdgeIndex () const
      {
        return (idx_opposite_half_edge_);
      }

      HalfEdgeIndex&
      getOppositeHalfEdgeIndex ()
      {
        return (idx_opposite_half_edge_);
      }

      void
      setOppositeHalfEdgeIndex (const HalfEdgeIndex& idx_opposite_half_edge)
      {
        idx_opposite_half_edge_ = idx_opposite_half_edge;
      }

      const HalfEdge&
      getOppositeHalfEdge (const Mesh& mesh) const
      {
        return (mesh.getHalfEdge (this->getOppositeHalfEdgeIndex ()));
      }

      HalfEdge&
      getOppositeHalfEdge (Mesh& mesh)
      {
        return (mesh.getHalfEdge (this->getOppositeHalfEdgeIndex ()));
      }

      //////////////////////////////////////////////////////////////////////////
      // NextHalfEdge
      //////////////////////////////////////////////////////////////////////////

      const HalfEdgeIndex&
      getNextHalfEdgeIndex () const
      {
        return (idx_next_half_edge_);
      }

      HalfEdgeIndex&
      getNextHalfEdgeIndex ()
      {
        return (idx_next_half_edge_);
      }

      void
      setNextHalfEdgeIndex (const HalfEdgeIndex& idx_next_half_edge)
      {
        idx_next_half_edge_ = idx_next_half_edge;
      }

      const HalfEdge&
      getNextHalfEdge (const Mesh& mesh) const
      {
        return (mesh.getHalfEdge (this->getNextHalfEdgeIndex ()));
      }

      HalfEdge&
      getNextHalfEdge (Mesh& mesh)
      {
        return (mesh.getHalfEdge (this->getNextHalfEdgeIndex ()));
      }

      //////////////////////////////////////////////////////////////////////////
      // PrevHalfedge
      //////////////////////////////////////////////////////////////////////////

      const HalfEdgeIndex&
      getPrevHalfEdgeIndex () const
      {
        return (idx_prev_half_edge_);
      }

      HalfEdgeIndex&
      getPrevHalfEdgeIndex ()
      {
        return (idx_prev_half_edge_);
      }

      void
      setPrevHalfEdgeIndex (const HalfEdgeIndex& idx_prev_half_edge)
      {
        idx_prev_half_edge_ = idx_prev_half_edge;
      }

      const HalfEdge&
      getPrevHalfEdge (const Mesh& mesh) const
      {
        return (mesh.getHalfEdge (this->getPrevHalfEdgeIndex ()));
      }

      HalfEdge&
      getPrevHalfEdge (Mesh& mesh)
      {
        return (mesh.getHalfEdge (this->getPrevHalfEdgeIndex ()));
      }

      //////////////////////////////////////////////////////////////////////////
      // Face
      //////////////////////////////////////////////////////////////////////////

      const FaceIndex&
      getFaceIndex () const
      {
        return (idx_face_);
      }

      FaceIndex&
      getFaceIndex ()
      {
        return (idx_face_);
      }

      void
      setFaceIndex (const FaceIndex& idx_face)
      {
        idx_face_ = idx_face;
      }

      const Face&
      getFace (const Mesh& mesh) const
      {
        return (mesh.getFace (this->getFaceIndex ()));
      }

      Face&
      getFace (Mesh& mesh)
      {
        return (mesh.getFace (this->getFaceIndex ()));
      }

      //////////////////////////////////////////////////////////////////////////
      // OppositeFace
      //////////////////////////////////////////////////////////////////////////

      const FaceIndex&
      getOppositeFaceIndex (const Mesh& mesh) const
      {
        return (this->getOppositeHalfEdge (mesh).getFaceIndex ());
      }

      FaceIndex&
      getOppositeFaceIndex (Mesh& mesh)
      {
        return (this->getOppositeHalfEdge (mesh).getFaceIndex ());
      }

      const Face&
      getOppositeFace (const Mesh& mesh) const
      {
        return (this->getOppositeHalfEdge (mesh).getFace (mesh));
      }

      Face&
      getOppositeFace (Mesh& mesh)
      {
        return (this->getOppositeHalfEdge (mesh).getFace (mesh));
      }

      //////////////////////////////////////////////////////////////////////////
      // deleted
      //////////////////////////////////////////////////////////////////////////

      bool
      getDeleted () const
      {
        return (is_deleted_);
      }

      void
      setDeleted (const bool is_deleted)
      {
        is_deleted_ = is_deleted;
      }

      //////////////////////////////////////////////////////////////////////////
      // Isolated
      //////////////////////////////////////////////////////////////////////////

      bool
      isIsolated () const
      {
        return (!this->getTerminatingVertexIndex ().isValid ());
      }

      //////////////////////////////////////////////////////////////////////////
      // isBoundary
      //////////////////////////////////////////////////////////////////////////

      bool
      isBoundary () const
      {
        return (!this->getFaceIndex ().isValid ());
      }

      //////////////////////////////////////////////////////////////////////////
      // Operators
      //////////////////////////////////////////////////////////////////////////

      bool
      operator == (const HalfEdgeData& other) const
      {
        return (HalfEdgeData::operator == (other));
      }

      bool
      operator == (const Self& other) const
      {
        // TODO: The comparison with HalfEdgeData could be ommitted if the user does not change the connectivity in the half-edges
        //       -> use only methods provided in mesh
        //       -> make the mesh a friend class and declare the set... methods private?
        return (this->getTerminatingVertexIndex () == other.getTerminatingVertexIndex () &&
                this->getOppositeHalfEdgeIndex ()  == other.getOppositeHalfEdgeIndex ()  &&
                this->getNextHalfEdgeIndex ()      == other.getNextHalfEdgeIndex ()      &&
                this->getPrevHalfEdgeIndex ()      == other.getPrevHalfEdgeIndex ()      &&
                this->getFaceIndex ()              == other.getFaceIndex ()              &&
                this->operator                     == (HalfEdgeData (other)));
      }

      bool
      operator != (const HalfEdgeData& other) const
      {
        return (!this->operator == (other));
      }

      bool
      operator != (const Self& other) const
      {
        return (!this->operator == (other));
      }

    private:

      bool          is_deleted_;

      VertexIndex   idx_terminating_vertex_;
      HalfEdgeIndex idx_opposite_half_edge_;
      HalfEdgeIndex idx_next_half_edge_;
      HalfEdgeIndex idx_prev_half_edge_;
      FaceIndex     idx_face_;
  };

} // End namespace pcl

#endif // HALF_EDGE_HPP

