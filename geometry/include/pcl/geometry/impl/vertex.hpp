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

#ifndef PCL_GEOMETRY_VERTEX_HPP
#define PCL_GEOMETRY_VERTEX_HPP

namespace pcl
{

  /** \brief A Vertex is a node in the mesh.
    *
    * \tparam VertexDataT User data that is stored in the Vertex: Must have operator == (Equality Comparable)
    * \tparam MeshT Mesh to which the Vertex belongs to (classes derived from MeshBase)
    *
    * \note It is not necessary to declare the Vertex manually. Please declare the mesh first and use the provided typedefs.
    * \author Martin Saelzle
    * \ingroup geometry
    */
  template <class VertexDataT, class MeshT>
  class Vertex : public VertexDataT
  {
      //////////////////////////////////////////////////////////////////////////
      // Types
      //////////////////////////////////////////////////////////////////////////

    public:

      typedef pcl::Vertex <VertexDataT, MeshT> Self;

      typedef VertexDataT VertexData;
      typedef MeshT       Mesh;

      typedef typename Mesh::HalfEdge      HalfEdge;
      typedef typename Mesh::HalfEdgeIndex HalfEdgeIndex;

      //////////////////////////////////////////////////////////////////////////
      // Constructor
      //////////////////////////////////////////////////////////////////////////

    public:

      /** \brief Constructor
        * \param vertex_data (optional) User data that is stored in the Vertex; defaults to VertexData ()
        * \param idx_outgoing_half_edge_ (optional) Outgoing HalfEdgeIndex; defaults to HalfEdgeIndex () (invalid index)
        */
      Vertex (const VertexData&    vertex_data             = VertexData (),
              const HalfEdgeIndex& idx_outgoing_half_edge_ = HalfEdgeIndex ())
        : VertexData              (vertex_data),
          is_deleted_             (false),
          idx_outgoing_half_edge_ (idx_outgoing_half_edge_)
      {
      }

      //////////////////////////////////////////////////////////////////////////
      // OutgoingHalfEdge
      //////////////////////////////////////////////////////////////////////////

    public:

      /** \brief Returns the outgoing HalfEdgeIndex (non-const) */
      HalfEdgeIndex&
      getOutgoingHalfEdgeIndex ()
      {
        return (idx_outgoing_half_edge_);
      }

      /** \brief Returns the outgoing HalfEdgeIndex (const) */
      const HalfEdgeIndex&
      getOutgoingHalfEdgeIndex () const
      {
        return (idx_outgoing_half_edge_);
      }

      /** \brief Set the outgoing HalfEdgeIndex */
      void
      setOutgoingHalfEdgeIndex (const HalfEdgeIndex& idx_outgoing_half_edge)
      {
        idx_outgoing_half_edge_ = idx_outgoing_half_edge;
      }

      /** \brief Returns the outgoing HalfEdge (non-const)
        * \param mesh Reference to the mesh to which the Vertex belongs to
        * \note Convenience method for Mesh::getElement
        */
      HalfEdge&
      getOutgoingHalfEdge (Mesh& mesh) const
      {
        return (mesh.getElement (this->getOutgoingHalfEdgeIndex ()));
      }

      /** \brief Returns the outgoing HalfEdge (const)
        * \param mesh Reference to the mesh to which the Vertex belongs to
        * \note Convenience method for Mesh::getElement
        */
      const HalfEdge&
      getOutgoingHalfEdge (const Mesh& mesh) const
      {
        return (mesh.getElement (this->getOutgoingHalfEdgeIndex ()));
      }

      //////////////////////////////////////////////////////////////////////////
      // IncomingHalfEdge
      //////////////////////////////////////////////////////////////////////////

    public:

      /** \brief Returns the incoming HalfEdgeIndex (non-const)
        * \param mesh Reference to the mesh to which the Vertex belongs to
        * \note Convenience method for getOutgoingHalfEdge -> getOppositeHalfEdgeIndex
        */
      HalfEdgeIndex&
      getIncomingHalfEdgeIndex (Mesh& mesh) const
      {
        return (this->getOutgoingHalfEdge (mesh).getOppositeHalfEdgeIndex ());
      }

      /** \brief Returns the incoming HalfEdgeIndex (const)
        * \param mesh Reference to the mesh to which the Vertex belongs to
        * \note Convenience method for getOutgoingHalfEdge -> getOppositeHalfEdgeIndex
        */
      const HalfEdgeIndex&
      getIncomingHalfEdgeIndex (const Mesh& mesh) const
      {
        return (this->getOutgoingHalfEdge (mesh).getOppositeHalfEdgeIndex ());
      }

      /** \brief Set the incoming HalfEdgeIndex
        * \param mesh Reference to the mesh to which the Vertex belongs to
        * \param idx_incoming_half_edge Incoming HalfEdgeIndex
        * \note Convenience method for getOutgoingHalfEdge -> setOppositeHalfEdgeIndex
        */
      void
      setIncomingHalfEdgeIndex (Mesh&                mesh,
                                const HalfEdgeIndex& idx_incoming_half_edge) const
      {
        this->getOutgoingHalfEdge (mesh).setOppositeHalfEdgeIndex (idx_incoming_half_edge);
      }

      /** \brief Returns the incoming HalfEdge (non-const)
        * \param mesh Reference to the mesh to which the Vertex belongs to
        * \note Convenience method for getOutgoingHalfEdge -> getOppositeHalfEdge
        */
      HalfEdge&
      getIncomingHalfEdge (Mesh& mesh) const
      {
        return (this->getOutgoingHalfEdge (mesh).getOppositeHalfEdge (mesh));
      }

      /** \brief Returns the incoming HalfEdge (const)
        * \param mesh Reference to the mesh to which the Vertex belongs to
        * \note Convenience method for getOutgoingHalfEdge -> getOppositeHalfEdge
        */
      const HalfEdge&
      getIncomingHalfEdge (const Mesh& mesh) const
      {
        return (this->getOutgoingHalfEdge (mesh).getOppositeHalfEdge (mesh));
      }

      //////////////////////////////////////////////////////////////////////////
      // deleted
      //////////////////////////////////////////////////////////////////////////

    public:

      /** \brief Returns true if the Vertex is marked as deleted */
      bool
      getDeleted () const
      {
        return (is_deleted_);
      }

      /** \brief Mark the Vertex as deleted (or not-deleted) */
      void
      setDeleted (const bool is_deleted)
      {
        is_deleted_ = is_deleted;
      }

      //////////////////////////////////////////////////////////////////////////
      // isIsolated
      //////////////////////////////////////////////////////////////////////////

    public:

      /** \brief Returns true if the Vertex is isolated (not connected to any HalfEdge) */
      bool
      isIsolated () const
      {
        return (!this->getOutgoingHalfEdgeIndex ().isValid ());
      }

      //////////////////////////////////////////////////////////////////////////
      // isBoundary
      //////////////////////////////////////////////////////////////////////////

    public:

      /** \brief Returns true if the outgoing HalfEdge is on the boundary.
        * \param mesh Reference to the mesh to which the Vertex belongs to
        */
      bool
      isBoundary (const Mesh& mesh) const
      {
        return (this->getOutgoingHalfEdge (mesh).isBoundary ());
      }

      //////////////////////////////////////////////////////////////////////////
      // isManifold
      //////////////////////////////////////////////////////////////////////////

    public:

      /** \brief Returns true if the vertex is manifold
        * \param mesh Reference to the mesh to which the Vertex belongs to
        *
        *  * Non-boundary vertexes are always manifold
        *  * Boundary vertexes are manifold if only one of its outgoing half-edges is on the boundary (this implies that it has only one incoming boundary half-edge)
        */
      bool
      isManifold (const Mesh& mesh) const
      {
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
      // Operators
      //////////////////////////////////////////////////////////////////////////

    public:

      /** \brief Equality comparison operator with VertexData */
      bool
      operator == (const VertexData& other) const
      {
        return (VertexData::operator == (other));
      }

      /** \brief Inequality comparison operator with VertexData */
      bool
      operator != (const VertexData& other) const
      {
        return (!this->operator == (other));
      }

      /** \brief Equality comparison operator with another Vertex */
      bool
      operator == (const Self& other) const
      {
        return (this->getOutgoingHalfEdgeIndex () == other.getOutgoingHalfEdgeIndex () &&
                this->operator                    == (VertexData (other)));
      }

      /** \brief Inequality comparison operator with another Vertex */
      bool
      operator != (const Self& other) const
      {
        return (!this->operator == (other));
      }

      //////////////////////////////////////////////////////////////////////////
      // Members
      //////////////////////////////////////////////////////////////////////////

    private:

      /** \brief Vertex is marked as deleted */
      bool          is_deleted_;

      /** \brief Index to an outgoing HalfEdge */
      HalfEdgeIndex idx_outgoing_half_edge_;
  };

} // End namespace pcl

#endif // PCL_GEOMETRY_VERTEX_HPP

