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

  /** \brief An Edge is a connection between two vertexes. In a half-edge mesh these are split into two HalfEdges with opposite orientation.
    *
    * \tparam HalfEdgeDataT User data that is stored in the HalfEdge: Must have operator == (Equality Comparable)
    * \tparam MeshT Mesh to which the HalfEdge belongs to (classes derived from MeshBase)
    *
    * \note It is not necessary to declare the HalfEdge manually. Please declare the mesh first and use the provided typedefs.
    * \author Martin Saelzle
    * \ingroup geometry
    */
  template <class HalfEdgeDataT, class MeshT>
  class HalfEdge : public HalfEdgeDataT
  {
      //////////////////////////////////////////////////////////////////////////
      // Types
      //////////////////////////////////////////////////////////////////////////

    public:

      typedef pcl::HalfEdge <HalfEdgeDataT, MeshT> Self;

      typedef HalfEdgeDataT HalfEdgeData;
      typedef MeshT         Mesh;

      typedef typename Mesh::Vertex        Vertex;
      typedef typename Mesh::VertexIndex   VertexIndex;
      typedef typename Mesh::HalfEdgeIndex HalfEdgeIndex;
      typedef typename Mesh::Face          Face;
      typedef typename Mesh::FaceIndex     FaceIndex;

      //////////////////////////////////////////////////////////////////////////
      // Constructor
      //////////////////////////////////////////////////////////////////////////

    public:

      /** \brief Constructor
        * \param half_edge_data (optional) User data that is stored in the HalfEdge; defaults to HalfEdgeData ()
        * \param idx_terminating_vertex (optional) Terminating VertexIndex; defaults to VertexIndex () (invalid index)
        * \param idx_opposite_half_edge (optional) Opposite HalfEdgeIndex; defaults to HalfEdgeIndex () (invalid index)
        * \param idx_next_half_edge (optional) Next HalfEdgeIndex; defaults to HalfEdgeIndex () (invalid index)
        * \param idx_prev_half_edge (optional) Previous HalfEdgeIndex; defaults to HalfEdgeIndex () (invalid index)
        * \param idx_face (optional) FaceIndex; defaults to FaceIndex () (invalid index)
        */
      HalfEdge (const HalfEdgeData&  half_edge_data         = HalfEdgeData  (),
                const VertexIndex&   idx_terminating_vertex = VertexIndex   (),
                const HalfEdgeIndex& idx_opposite_half_edge = HalfEdgeIndex (),
                const HalfEdgeIndex& idx_next_half_edge     = HalfEdgeIndex (),
                const HalfEdgeIndex& idx_prev_half_edge     = HalfEdgeIndex (),
                const FaceIndex&     idx_face               = FaceIndex     ())
        : HalfEdgeData            (half_edge_data),
          is_deleted_             (false),
          idx_terminating_vertex_ (idx_terminating_vertex),
          idx_opposite_half_edge_ (idx_opposite_half_edge),
          idx_next_half_edge_     (idx_next_half_edge),
          idx_prev_half_edge_     (idx_prev_half_edge),
          idx_face_               (idx_face)
      {
      }

      //////////////////////////////////////////////////////////////////////////
      // TerminatingVertex
      //////////////////////////////////////////////////////////////////////////

    public:

      /** \brief Returns the terminating VertexIndex (non-const) */
      VertexIndex&
      getTerminatingVertexIndex ()
      {
        return (idx_terminating_vertex_);
      }

      /** \brief Returns the terminating VertexIndex (const) */
      const VertexIndex&
      getTerminatingVertexIndex () const
      {
        return (idx_terminating_vertex_);
      }

      /** \brief Set the terminating VertexIndex */
      void
      setTerminatingVertexIndex (const VertexIndex& idx_terminating_vertex)
      {
        idx_terminating_vertex_ = idx_terminating_vertex;
      }

      /** \brief Returns the terminating Vertex (non-const)
        * \param mesh Reference to the mesh to which the HalfEdge belongs to
        * \note Convenience method for Mesh::getElement
        */
      Vertex&
      getTerminatingVertex (Mesh& mesh) const
      {
        return (mesh.getElement (this->getTerminatingVertexIndex ()));
      }

      /** \brief Returns the terminating Vertex (const)
        * \param mesh Reference to the mesh to which the HalfEdge belongs to
        * \note Convenience method for Mesh::getElement
        */
      const Vertex&
      getTerminatingVertex (const Mesh& mesh) const
      {
        return (mesh.getElement (this->getTerminatingVertexIndex ()));
      }

      //////////////////////////////////////////////////////////////////////////
      // OriginatingVertex
      //////////////////////////////////////////////////////////////////////////

    public:

      /** \brief Returns the originating VertexIndex (non-const)
        * \param mesh Reference to the mesh to which the HalfEdge belongs to
        * \note Convenience method for getOppositeHalfEdge -> getTerminatingVertexIndex
        */
      VertexIndex&
      getOriginatingVertexIndex (Mesh& mesh) const
      {
        return (this->getOppositeHalfEdge (mesh).getTerminatingVertexIndex ());
      }

      /** \brief Returns the originating VertexIndex (const)
        * \param mesh Reference to the mesh to which the HalfEdge belongs to
        * \note Convenience method for getOppositeHalfEdge -> getTerminatingVertexIndex
        */
      const VertexIndex&
      getOriginatingVertexIndex (const Mesh& mesh) const
      {
        return (this->getOppositeHalfEdge (mesh).getTerminatingVertexIndex ());
      }

      /** \brief Set the originating VertexIndex
        * \param mesh Reference to the mesh to which the HalfEdge belongs to
        * \param idx_originating_vertex Originating VertexIndex
        * \note Convenience method for getOppositeHalfEdge -> setTerminatingVertexIndex
        */
      void
      setOriginatingVertexIndex (Mesh&                mesh,
                                 const HalfEdgeIndex& idx_originating_vertex) const
      {
        this->getOppositeHalfEdge (mesh).setTerminatingVertexIndex (idx_originating_vertex);
      }

      /** \brief Returns the originating Vertex (non-const)
        * \param mesh Reference to the mesh to which the HalfEdge belongs to
        * \note Convenience method for getOppositeHalfEdge -> getTerminatingVertex
        */
      Vertex&
      getOriginatingVertex (Mesh& mesh) const
      {
        return (this->getOppositeHalfEdge (mesh).getTerminatingVertex (mesh));
      }

      /** \brief Returns the originating Vertex (const)
        * \param mesh Reference to the mesh to which the HalfEdge belongs to
        * \note Convenience method for getOppositeHalfEdge -> getTerminatingVertex
        */
      const Vertex&
      getOriginatingVertex (const Mesh& mesh) const
      {
        return (this->getOppositeHalfEdge (mesh).getTerminatingVertex (mesh));
      }

      //////////////////////////////////////////////////////////////////////////
      // OppositeHalfEdge
      //////////////////////////////////////////////////////////////////////////

    public:

      /** \brief Returns the opposite HalfEdgeIndex (non-const) */
      HalfEdgeIndex&
      getOppositeHalfEdgeIndex ()
      {
        return (idx_opposite_half_edge_);
      }

      /** \brief Returns the opposite HalfEdgeIndex (const) */
      const HalfEdgeIndex&
      getOppositeHalfEdgeIndex () const
      {
        return (idx_opposite_half_edge_);
      }

      /** \brief Set the opposite HalfEdgeIndex */
      void
      setOppositeHalfEdgeIndex (const HalfEdgeIndex& idx_opposite_half_edge)
      {
        idx_opposite_half_edge_ = idx_opposite_half_edge;
      }

      /** \brief Returns the opposite HalfEdge (non-const)
        * \param mesh Reference to the mesh to which the HalfEdge belongs to
        * \note Convenience method for Mesh::getElement
        */
      HalfEdge&
      getOppositeHalfEdge (Mesh& mesh) const
      {
        return (mesh.getElement (this->getOppositeHalfEdgeIndex ()));
      }

      /** \brief Returns the opposite HalfEdge (const)
        * \param mesh Reference to the mesh to which the HalfEdge belongs to
        * \note Convenience method for Mesh::getElement
        */
      const HalfEdge&
      getOppositeHalfEdge (const Mesh& mesh) const
      {
        return (mesh.getElement (this->getOppositeHalfEdgeIndex ()));
      }

      //////////////////////////////////////////////////////////////////////////
      // NextHalfEdge
      //////////////////////////////////////////////////////////////////////////

    public:

      /** \brief Returns the next HalfEdgeIndex (non-const) */
      HalfEdgeIndex&
      getNextHalfEdgeIndex ()
      {
        return (idx_next_half_edge_);
      }

      /** \brief Returns the next HalfEdgeIndex (const) */
      const HalfEdgeIndex&
      getNextHalfEdgeIndex () const
      {
        return (idx_next_half_edge_);
      }

      /** \brief Set the next HalfEdgeIndex */
      void
      setNextHalfEdgeIndex (const HalfEdgeIndex& idx_next_half_edge)
      {
        idx_next_half_edge_ = idx_next_half_edge;
      }

      /** \brief Returns the next HalfEdge (non-const)
        * \param mesh Reference to the mesh to which the HalfEdge belongs to
        * \note Convenience method for Mesh::getElement
        */
      HalfEdge&
      getNextHalfEdge (Mesh& mesh) const
      {
        return (mesh.getElement (this->getNextHalfEdgeIndex ()));
      }

      /** \brief Returns the next HalfEdge (const)
        * \param mesh Reference to the mesh to which the HalfEdge belongs to
        * \note Convenience method for Mesh::getElement
        */
      const HalfEdge&
      getNextHalfEdge (const Mesh& mesh) const
      {
        return (mesh.getElement (this->getNextHalfEdgeIndex ()));
      }

      //////////////////////////////////////////////////////////////////////////
      // PrevHalfedge
      //////////////////////////////////////////////////////////////////////////

    public:

      /** \brief Returns the previous HalfEdgeIndex (non-const) */
      HalfEdgeIndex&
      getPrevHalfEdgeIndex ()
      {
        return (idx_prev_half_edge_);
      }

      /** \brief Returns the previous HalfEdgeIndex (const) */
      const HalfEdgeIndex&
      getPrevHalfEdgeIndex () const
      {
        return (idx_prev_half_edge_);
      }

      /** \brief Set the previous HalfEdgeIndex */
      void
      setPrevHalfEdgeIndex (const HalfEdgeIndex& idx_prev_half_edge)
      {
        idx_prev_half_edge_ = idx_prev_half_edge;
      }

      /** \brief Returns the previous HalfEdge (non-const)
        * \param mesh Reference to the mesh to which the HalfEdge belongs to
        * \note Convenience method for Mesh::getElement
        */
      HalfEdge&
      getPrevHalfEdge (Mesh& mesh) const
      {
        return (mesh.getElement (this->getPrevHalfEdgeIndex ()));
      }

      /** \brief Returns the previous HalfEdge (const)
        * \param mesh Reference to the mesh to which the HalfEdge belongs to
        * \note Convenience method for Mesh::getElement
        */
      const HalfEdge&
      getPrevHalfEdge (const Mesh& mesh) const
      {
        return (mesh.getElement (this->getPrevHalfEdgeIndex ()));
      }

      //////////////////////////////////////////////////////////////////////////
      // Face
      //////////////////////////////////////////////////////////////////////////

    public:

      /** \brief Get the FaceIndex (non-const) */
      FaceIndex&
      getFaceIndex ()
      {
        return (idx_face_);
      }

      /** \brief Get the FaceIndex (const) */
      const FaceIndex&
      getFaceIndex () const
      {
        return (idx_face_);
      }

      /** \brief Set the FaceIndex */
      void
      setFaceIndex (const FaceIndex& idx_face)
      {
        idx_face_ = idx_face;
      }

      /** \brief Returns the Face (non-const)
        * \param mesh Reference to the mesh to which the HalfEdge belongs to
        * \note Convenience method for Mesh::getElement
        */
      Face&
      getFace (Mesh& mesh) const
      {
        return (mesh.getElement (this->getFaceIndex ()));
      }

      /** \brief Returns the Face (const)
        * \param mesh Reference to the mesh to which the HalfEdge belongs to
        * \note Convenience method for Mesh::getElement
        */
      const Face&
      getFace (const Mesh& mesh) const
      {
        return (mesh.getElement (this->getFaceIndex ()));
      }

      //////////////////////////////////////////////////////////////////////////
      // OppositeFace
      //////////////////////////////////////////////////////////////////////////

    public:

      /** \brief Returns the opposite FaceIndex (non-const)
        * \param mesh Reference to the mesh to which the HalfEdge belongs to
        * \note Convenience method for getOppositeFace -> getFaceIndex
        */
      FaceIndex&
      getOppositeFaceIndex (Mesh& mesh) const
      {
        return (this->getOppositeHalfEdge (mesh).getFaceIndex ());
      }

      /** \brief Returns the opposite FaceIndex (const)
        * \param mesh Reference to the mesh to which the HalfEdge belongs to
        * \note Convenience method for getOppositeFace -> getFaceIndex
        */
      const FaceIndex&
      getOppositeFaceIndex (const Mesh& mesh) const
      {
        return (this->getOppositeHalfEdge (mesh).getFaceIndex ());
      }

      /** \brief Set the opposite FaceIndex
        * \param mesh Reference to the mesh to which the HalfEdge belongs to
        * \param idx_opposite_face Opposite FaceIndex
        * \note Convenience method for getOppositeFace -> setFaceIndex
        */
      void
      setOppositeFaceIndex (Mesh&            mesh,
                            const FaceIndex& idx_opposite_face) const
      {
        this->getOppositeHalfEdge (mesh).setFaceIndex (idx_opposite_face);
      }

      /** \brief Returns the opposite Face (non-const)
        * \param mesh Reference to the mesh to which the HalfEdge belongs to
        * \note Convenience method for getOppositeHalfEdge -> getFace
        */
      Face&
      getOppositeFace (Mesh& mesh) const
      {
        return (this->getOppositeHalfEdge (mesh).getFace (mesh));
      }

      /** \brief Returns the opposite Face (const)
        * \param mesh Reference to the mesh to which the HalfEdge belongs to
        * \note Convenience method for getOppositeHalfEdge -> getFace
        */
      const Face&
      getOppositeFace (const Mesh& mesh) const
      {
        return (this->getOppositeHalfEdge (mesh).getFace (mesh));
      }

      //////////////////////////////////////////////////////////////////////////
      // deleted
      //////////////////////////////////////////////////////////////////////////

    public:

      /** \brief Returns true if the HalfEdge is marked as deleted */
      bool
      getDeleted () const
      {
        return (is_deleted_);
      }

      /** \brief Mark the HalfEdge as deleted (or not-deleted) */
      void
      setDeleted (const bool is_deleted)
      {
        is_deleted_ = is_deleted;
      }

      //////////////////////////////////////////////////////////////////////////
      // Isolated
      //////////////////////////////////////////////////////////////////////////

    public:

      /** \brief Returns true if the HalfEdge is isolated.
        *
        * The HalfEdge is isolated if any of these indexes are invalid:
        *  * Terminating VertexIndex
        *  * Opposite HalfEdgeIndex
        *  * Next HalfEdgeIndex
        *  * Previous HalfEdgeIndex
        *
        * An invalid FaceIndex is allowed (HalfEdge is on the boundary).
        */
      bool
      isIsolated () const
      {
        return (!(this->getTerminatingVertexIndex ().isValid () &&
                  this->getOppositeHalfEdgeIndex ().isValid ()  &&
                  this->getNextHalfEdgeIndex ().isValid ()      &&
                  this->getPrevHalfEdgeIndex ().isValid ()));
      }

      //////////////////////////////////////////////////////////////////////////
      // isBoundary
      //////////////////////////////////////////////////////////////////////////

    public:

      /** \brief Returns true if the HalfEdge is on the boundary. */
      bool
      isBoundary () const
      {
        return (!this->getFaceIndex ().isValid ());
      }

      //////////////////////////////////////////////////////////////////////////
      // Operators
      //////////////////////////////////////////////////////////////////////////

    public:

      /** \brief Equality comparison operator with HalfEdgeData */
      bool
      operator == (const HalfEdgeData& other) const
      {
        return (HalfEdgeData::operator == (other));
      }

      /** \brief Inequality comparison operator with HalfEdgeData */
      bool
      operator != (const HalfEdgeData& other) const
      {
        return (!this->operator == (other));
      }

      /** \brief Equality comparison operator with another HalfEdge */
      bool
      operator == (const Self& other) const
      {
        return (this->getTerminatingVertexIndex () == other.getTerminatingVertexIndex () &&
                this->getOppositeHalfEdgeIndex ()  == other.getOppositeHalfEdgeIndex ()  &&
                this->getNextHalfEdgeIndex ()      == other.getNextHalfEdgeIndex ()      &&
                this->getPrevHalfEdgeIndex ()      == other.getPrevHalfEdgeIndex ()      &&
                this->getFaceIndex ()              == other.getFaceIndex ()              &&
                this->operator                     == (HalfEdgeData (other)));
      }

      /** \brief Inequality comparison operator with another HalfEdge */
      bool
      operator != (const Self& other) const
      {
        return (!this->operator == (other));
      }

      //////////////////////////////////////////////////////////////////////////
      // Members
      //////////////////////////////////////////////////////////////////////////

    private:

      /** \brief HalfEdge is marked as deleted */
      bool          is_deleted_;

      /** \brief  Index to the terminating Vertex */
      VertexIndex   idx_terminating_vertex_;

      /** \brief  Index to the opposite HalfEdge */
      HalfEdgeIndex idx_opposite_half_edge_;

      /** \brief  Index to the next HalfEdge */
      HalfEdgeIndex idx_next_half_edge_;

      /** \brief  Index to the previous HalfEdge */
      HalfEdgeIndex idx_prev_half_edge_;

      /** \brief  Index to the Face */
      FaceIndex     idx_face_;
  };

} // End namespace pcl

#endif // HALF_EDGE_HPP

