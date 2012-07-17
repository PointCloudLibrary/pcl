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

#ifndef VERTEX_HPP
#define VERTEX_HPP

namespace pcl
{

  template <class VertexDataT, class MeshT>
  class Vertex : public VertexDataT
  {
    public:

      typedef pcl::Vertex <VertexDataT, MeshT> Self;

      typedef VertexDataT VertexData;
      typedef MeshT       Mesh;

      typedef typename Mesh::VertexIndex   VertexIndex;
      typedef typename Mesh::HalfEdge      HalfEdge;
      typedef typename Mesh::HalfEdgeIndex HalfEdgeIndex;

    public:

      Vertex (const VertexData&    vertex_data             = VertexData (),
              const HalfEdgeIndex& idx_outgoing_half_edge_ = HalfEdgeIndex ())
        : VertexData              (vertex_data),
          is_deleted_             (false),
          idx_outgoing_half_edge_ (idx_outgoing_half_edge_)
      {
      }

      Vertex (const Self& other)
        : VertexData              (other),
          is_deleted_             (other.getDeleted ()),
          idx_outgoing_half_edge_ (other.getOutgoingHalfEdgeIndex ())
      {
      }

    public:

      //////////////////////////////////////////////////////////////////////////
      // OutgoingHalfEdge
      //////////////////////////////////////////////////////////////////////////

      const HalfEdgeIndex&
      getOutgoingHalfEdgeIndex () const
      {
        return (idx_outgoing_half_edge_);
      }

      HalfEdgeIndex&
      getOutgoingHalfEdgeIndex ()
      {
        return (idx_outgoing_half_edge_);
      }

      void
      setOutgoingHalfEdgeIndex (const HalfEdgeIndex& idx_outgoing_half_edge)
      {
        idx_outgoing_half_edge_ = idx_outgoing_half_edge;
      }

      const HalfEdge&
      getOutgoingHalfEdge (const Mesh& mesh) const
      {
        return (mesh.getHalfEdge (this->getOutgoingHalfEdgeIndex ()));
      }

      HalfEdge&
      getOutgoingHalfEdge (Mesh& mesh)
      {
        return (mesh.getHalfEdge (this->getOutgoingHalfEdgeIndex ()));
      }

      //////////////////////////////////////////////////////////////////////////
      // IncomingHalfEdge
      //////////////////////////////////////////////////////////////////////////

      const HalfEdgeIndex&
      getIncomingHalfEdgeIndex (const Mesh& mesh) const
      {
        return (this->getOutgoingHalfEdge (mesh).getOppositeHalfEdgeIndex ());
      }

      HalfEdgeIndex&
      getIncomingHalfEdgeIndex (Mesh& mesh)
      {
        return (this->getOutgoingHalfEdge (mesh).getOppositeHalfEdgeIndex ());
      }

      const HalfEdge&
      getIncomingHalfEdge (const Mesh& mesh) const
      {
        return (this->getOutgoingHalfEdge (mesh).getOppositeHalfEdge (mesh));
      }

      HalfEdge&
      getIncomingHalfEdge (Mesh& mesh)
      {
        return (this->getOutgoingHalfEdge (mesh).getOppositeHalfEdge (mesh));
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
      // isIsolated
      //////////////////////////////////////////////////////////////////////////

      bool
      isIsolated () const
      {
        return (!this->getOutgoingHalfEdgeIndex ().isValid ());
      }

      //////////////////////////////////////////////////////////////////////////
      // isBoundary
      //////////////////////////////////////////////////////////////////////////

      bool
      isBoundary (const Mesh& mesh) const
      {
        return (this->getOutgoingHalfEdge (mesh).isBoundary ());
      }

      //////////////////////////////////////////////////////////////////////////
      // isManifold
      //////////////////////////////////////////////////////////////////////////

      bool
      isManifold (const Mesh& mesh) const
      {
        // Outgoing half-edge is not boundary -> manifold
        // Outgoing half-edge is boundary
        //   - No other outgoing half-edge is boundary  -> manifold
        //   - Any other outgoing half-edge is boundary -> non-manifold
        typename Mesh::OutgoingHalfEdgeAroundVertexConstCirculator circ = mesh.getOutgoingHalfEdgeAroundVertexConstCirculator (*this);

        if (!circ->isBoundary ())
        {
          return (true);
        }

        const typename Mesh::OutgoingHalfEdgeAroundVertexConstCirculator circ_end = circ++;
        do
        {
          if ((*circ++).isBoundary ())
          {
            return (false);
          }
        } while (circ!=circ_end);

        return (true);
      }

      //////////////////////////////////////////////////////////////////////////
      // isConnectedTo
      //////////////////////////////////////////////////////////////////////////

      bool
      isConnectedTo (const Mesh&        mesh,
                     const VertexIndex& idx_v_other,
                     HalfEdgeIndex&     idx_connecting_half_edge) const
      {
        typename Mesh::VertexAroundVertexConstCirculator       circ     = mesh.getVertexAroundVertexConstCirculator (*this);
        const typename Mesh::VertexAroundVertexConstCirculator circ_end = circ;

        do
        {
          if (circ.getDereferencedIndex () == idx_v_other)
          {
            idx_connecting_half_edge = circ.getCurrentHalfEdgeIndex ();
            return (true);
          }
          ++circ;
        } while (circ!=circ_end);

        return (false);
      }

      //////////////////////////////////////////////////////////////////////////
      // Operators
      //////////////////////////////////////////////////////////////////////////

      bool
      operator == (const VertexData& other) const
      {
        return (VertexData::operator == (other));
      }

      bool
      operator == (const Self& other) const
      {
        // TODO: The comparison with VertexData could be ommitted if the user does not change the connectivity in the vertexes
        //       -> use only methods provided in mesh
        //       -> make the mesh a friend class and declare the set... methods private?
        return (this->getOutgoingHalfEdgeIndex () == other.getOutgoingHalfEdgeIndex () &&
                this->operator                    == (VertexData (other)));
      }

      bool
      operator != (const VertexData& other) const
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

      HalfEdgeIndex idx_outgoing_half_edge_;
  };

} // End namespace pcl

#endif // VERTEX_HPP

