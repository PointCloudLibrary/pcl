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

#ifndef PCL_GEOMETRY_FACE_HPP
#define PCL_GEOMETRY_FACE_HPP

namespace pcl
{

  /** \brief A Face is defined by a closed loop of edges.
    *
    * \tparam FaceDataT User data that is stored in the Face: Must have operator == (Equality Comparable)
    * \tparam MeshT Mesh to which the Face belongs to (classes derived from MeshBase)
    *
    * \note It is not necessary to declare the Face manually. Please declare the mesh first and use the provided typedefs.
    * \author Martin Saelzle
    * \ingroup geometry
    */
  template <class FaceDataT, class MeshT>
  class Face : public FaceDataT
  {
      //////////////////////////////////////////////////////////////////////////
      // Types
      //////////////////////////////////////////////////////////////////////////

    public:

      typedef pcl::Face <FaceDataT, MeshT> Self;

      typedef FaceDataT FaceData;
      typedef MeshT     Mesh;

      typedef typename Mesh::HalfEdge      HalfEdge;
      typedef typename Mesh::HalfEdgeIndex HalfEdgeIndex;

      //////////////////////////////////////////////////////////////////////////
      // Constructor
      //////////////////////////////////////////////////////////////////////////

    public:

      /** \brief Constructor
        * \param face_data (optional) User data that is stored in the Face; defaults to FaceData ()
        * \param idx_inner_half_edge (optional) Inner HalfEdgeIndex; defaults to HalfEdgeIndex () (invalid index)
        */
      Face (const FaceData&      face_data           = FaceData (),
            const HalfEdgeIndex& idx_inner_half_edge = HalfEdgeIndex ())
        : FaceData             (face_data),
          is_deleted_          (false),
          idx_inner_half_edge_ (idx_inner_half_edge)
      {
      }

      //////////////////////////////////////////////////////////////////////////
      // InnerHalfEdge
      //////////////////////////////////////////////////////////////////////////

    public:

      /** \brief Returns the inner HalfEdgeIndex (non-const) */
      HalfEdgeIndex&
      getInnerHalfEdgeIndex ()
      {
        return (idx_inner_half_edge_);
      }

      /** \brief Returns the inner HalfEdgeIndex (const) */
      const HalfEdgeIndex&
      getInnerHalfEdgeIndex () const
      {
        return (idx_inner_half_edge_);
      }

      /** \brief Set the inner HalfEdgeIndex */
      void
      setInnerHalfEdgeIndex (const HalfEdgeIndex& idx_inner_half_edge)
      {
        idx_inner_half_edge_ = idx_inner_half_edge;
      }

      /** \brief Returns the inner HalfEdge (non-const)
        * \param mesh Reference to the mesh to which the Face belongs to
        * \note Convenience method for Mesh::getElement
        */
      HalfEdge&
      getInnerHalfEdge (Mesh& mesh) const
      {
        return (mesh.getElement (this->getInnerHalfEdgeIndex ()));
      }

      /** \brief Returns the inner HalfEdge (const)
        * \param mesh Reference to the mesh to which the Face belongs to
        * \note Convenience method for Mesh::getElement
        */
      const HalfEdge&
      getInnerHalfEdge (const Mesh& mesh) const
      {
        return (mesh.getElement (this->getInnerHalfEdgeIndex ()));
      }

      //////////////////////////////////////////////////////////////////////////
      // OuterHalfEdge
      //////////////////////////////////////////////////////////////////////////

    public:

      /** \brief Returns the outer HalfEdgeIndex (non-const)
        * \param mesh Reference to the mesh to which the Face belongs to
        * \note Convenience method for getInnerHalfEdge -> getOppositeHalfEdgeIndex
        */
      HalfEdgeIndex&
      getOuterHalfEdgeIndex (Mesh& mesh) const
      {
        return (this->getInnerHalfEdge (mesh).getOppositeHalfEdgeIndex ());
      }

      /** \brief Returns the outer HalfEdgeIndex (const)
        * \param mesh Reference to the mesh to which the Face belongs to
        * \note Convenience method for getInnerHalfEdge -> getOppositeHalfEdgeIndex
        */
      const HalfEdgeIndex&
      getOuterHalfEdgeIndex (const Mesh& mesh) const
      {
        return (this->getInnerHalfEdge (mesh).getOppositeHalfEdgeIndex ());
      }

      /** \brief Set the outer HalfEdgeIndex
        * \param mesh Reference to the mesh to which the Face belongs to
        * \param idx_outer_half_edge Outer HalfEdgeIndex
        * \note Convenience method for getInnerHalfEdge -> setOppositeHalfEdgeIndex
        */
      void
      setOuterHalfEdgeIndex (Mesh&                mesh,
                             const HalfEdgeIndex& idx_outer_half_edge) const
      {
        this->getInnerHalfEdge (mesh).setOppositeHalfEdgeIndex (idx_outer_half_edge);
      }

      /** \brief Returns the outer HalfEdge (non-const)
        * \param mesh Reference to the mesh to which the Face belongs to
        * \note Convenience method for getInnerHalfEdge -> getOppositeHalfEdge
        */
      HalfEdge&
      getOuterHalfEdge (Mesh& mesh) const
      {
        return (this->getInnerHalfEdge (mesh).getOppositeHalfEdge (mesh));
      }

      /** \brief Returns the outer HalfEdge (const)
        * \param mesh Reference to the mesh to which the Face belongs to
        * \note Convenience method for getInnerHalfEdge -> getOppositeHalfEdge
        */
      const HalfEdge&
      getOuterHalfEdge (const Mesh& mesh) const
      {
        return (this->getInnerHalfEdge (mesh).getOppositeHalfEdge (mesh));
      }

      //////////////////////////////////////////////////////////////////////////
      // deleted
      //////////////////////////////////////////////////////////////////////////

    public:

      /** \brief Returns true if the Face is marked as deleted */
      bool
      getDeleted () const
      {
        return (is_deleted_);
      }

      /** \brief Mark the Face as deleted (or not-deleted) */
      void
      setDeleted (const bool is_deleted)
      {
        is_deleted_ = is_deleted;
      }

      //////////////////////////////////////////////////////////////////////////
      // Isolated
      //////////////////////////////////////////////////////////////////////////

    public:

      /** \brief Returns true if the Face is isolated (not connected to any HalfEdge) */
      bool
      isIsolated () const
      {
        return (!this->getInnerHalfEdgeIndex ().isValid ());
      }

      //////////////////////////////////////////////////////////////////////////
      // isBoundary
      //////////////////////////////////////////////////////////////////////////

    public:

      /** \brief Returns true if any Vertex in the Face is on the boundary
        * \param mesh Reference to the mesh to which the Face belongs to
        */
      bool
      isBoundary (const Mesh& mesh) const
      {
        typename Mesh::VertexAroundFaceConstCirculator       circ     = mesh.getVertexAroundFaceConstCirculator (*this);
        const typename Mesh::VertexAroundFaceConstCirculator circ_end = circ;

        do
        {
          if ((*circ++).isBoundary (mesh))
          {
            return (true);
          }
        } while (circ!=circ_end);

        return (false);
      }

      //////////////////////////////////////////////////////////////////////////
      // Operators
      //////////////////////////////////////////////////////////////////////////

    public:

      /** \brief Equality comparison operator with FaceData */
      bool
      operator == (const FaceData& other) const
      {
        return (FaceData::operator == (other));
      }

      /** \brief Inequality comparison operator with FaceData */
      bool
      operator != (const FaceData& other) const
      {
        return (!this->operator == (other));
      }

      /** \brief Equality comparison operator with another Face */
      bool
      operator == (const Self& other) const
      {
        return (this->getInnerHalfEdgeIndex () == other.getInnerHalfEdgeIndex () &&
                this->operator                 == (FaceData (other)));
      }

      /** \brief Inequality comparison operator with another Face */
      bool
      operator != (const Self& other) const
      {
        return (!this->operator == (other));
      }

      //////////////////////////////////////////////////////////////////////////
      // Members
      //////////////////////////////////////////////////////////////////////////

    private:

      /** \brief Face is marked as deleted */
      bool          is_deleted_;

      /** \brief Index to an inner HalfEdge */
      HalfEdgeIndex idx_inner_half_edge_;
  };

} // End namespace pcl

#endif // PCL_GEOMETRY_FACE_HPP

