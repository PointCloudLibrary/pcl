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

#ifndef FACE_HPP
#define FACE_HPP

namespace pcl
{

  template <class FaceDataT, class MeshT>
  class Face : public FaceDataT
  {
    public:

      typedef pcl::Face <FaceDataT, MeshT> Self;

      typedef FaceDataT FaceData;
      typedef MeshT     Mesh;

      typedef typename Mesh::VertexIndex   VertexIndex;
      typedef typename Mesh::HalfEdge      HalfEdge;
      typedef typename Mesh::HalfEdgeIndex HalfEdgeIndex;

    public:

      Face (const FaceData&      face_data           = FaceData (),
            const HalfEdgeIndex& idx_inner_half_edge = HalfEdgeIndex ())
        : FaceData             (face_data),
          is_deleted_          (false),
          idx_inner_half_edge_ (idx_inner_half_edge)
      {
      }

      Face (const Self& other)
        : FaceData             (other),
          is_deleted_          (other.getDeleted ()),
          idx_inner_half_edge_ (other.getInnerHalfEdgeIndex ())
      {
      }

    public:

      //////////////////////////////////////////////////////////////////////////
      // InnerHalfEdge
      //////////////////////////////////////////////////////////////////////////

      const HalfEdgeIndex&
      getInnerHalfEdgeIndex () const
      {
        return (idx_inner_half_edge_);
      }

      HalfEdgeIndex&
      getInnerHalfEdgeIndex ()
      {
        return (idx_inner_half_edge_);
      }

      void
      setInnerHalfEdgeIndex (const HalfEdgeIndex& idx_inner_half_edge)
      {
        idx_inner_half_edge_ = idx_inner_half_edge;
      }

      const HalfEdge&
      getInnerHalfEdge (const Mesh& mesh) const
      {
        return (mesh.getHalfEdge (this->getInnerHalfEdgeIndex ()));
      }

      HalfEdge&
      getInnerHalfEdge (Mesh& mesh)
      {
        return (mesh.getHalfEdge (this->getInnerHalfEdgeIndex ()));
      }

      //////////////////////////////////////////////////////////////////////////
      // OuterHalfEdge
      //////////////////////////////////////////////////////////////////////////

      const HalfEdgeIndex&
      getOuterHalfEdgeIndex (const Mesh& mesh) const
      {
        return (this->getInnerHalfEdge (mesh).getOppositeHalfEdgeIndex ());
      }

      HalfEdgeIndex&
      getOuterHalfEdgeIndex (Mesh& mesh)
      {
        return (this->getInnerHalfEdge (mesh).getOppositeHalfEdgeIndex ());
      }

      const HalfEdge&
      getOuterHalfEdge (const Mesh& mesh) const
      {
        return (this->getInnerHalfEdge (mesh).getOppositeHalfEdge (mesh));
      }

      HalfEdge&
      getOuterHalfEdge (Mesh& mesh)
      {
        return (this->getInnerHalfEdge (mesh).getOppositeHalfEdge (mesh));
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
        return (!this->getInnerHalfEdgeIndex ().isValid ());
      }

      //////////////////////////////////////////////////////////////////////////
      // isBoundary
      //////////////////////////////////////////////////////////////////////////

      bool
      isBoundary (const Mesh& mesh) const
      {
        typename Mesh::VertexAroundFaceConstCirculator circ = mesh.getVertexAroundFaceConstCirculator (*this);

        if      ((*circ++).isBoundary (mesh)) return (true); // first
        else if ((*circ++).isBoundary (mesh)) return (true); // second
        else if ((*circ++).isBoundary (mesh)) return (true); // third
        else                              return (false);
      }

      bool
      isBoundary (const Mesh&  mesh,
                  VertexIndex& idx_first_found_boundary_vertex) const
      {
        typename Mesh::VertexAroundFaceConstCirculator circ = mesh.getVertexAroundFaceConstCirculator (*this);

        if (circ->isBoundary (mesh)) // first
        {
          idx_first_found_boundary_vertex = circ.getDereferencedIndex ();
          return (true);
        }
        ++circ;

        if (circ->isBoundary (mesh)) // second
        {
          idx_first_found_boundary_vertex = circ.getDereferencedIndex ();
          return (true);
        }
        ++circ;

        if (circ->isBoundary (mesh)) // third
        {
          idx_first_found_boundary_vertex = circ.getDereferencedIndex ();
          return (true);
        }

        return (false);
      }

      unsigned char
      isBoundary (const Mesh&  mesh,
                  VertexIndex& idx_first_found_boundary_vertex,
                  VertexIndex& idx_second_found_boundary_vertex,
                  VertexIndex& idx_third_found_boundary_vertex) const
      {
        typename Mesh::VertexAroundFaceConstCirculator circ = mesh.getVertexAroundFaceConstCirculator (*this);

        unsigned char is_boundary = 0; // treated bitwise

        if (circ->isBoundary (mesh)) // first
        {
          idx_first_found_boundary_vertex = circ.getDereferencedIndex ();
          is_boundary |= 1;
        }
        ++circ;

        if (circ->isBoundary (mesh)) // second
        {
          idx_second_found_boundary_vertex = circ.getDereferencedIndex ();
          is_boundary |= 2;
        }
        ++circ;

        if (circ->isBoundary (mesh)) // third
        {
          idx_third_found_boundary_vertex = circ.getDereferencedIndex ();
          is_boundary |= 4;
        }

        return (is_boundary);
      }

      //////////////////////////////////////////////////////////////////////////
      // Operators
      //////////////////////////////////////////////////////////////////////////

      bool
      operator == (const FaceData& other) const
      {
        return (FaceData::operator == (other));
      }

      bool
      operator == (const Self& other) const
      {
        // TODO: The comparison with FaceData could be ommitted if the user does not change the connectivity in the faces
        //       -> use only methods provided in mesh
        //       -> make the mesh a friend class and declare the set... methods private?
        return (this->getInnerHalfEdgeIndex () == other.getInnerHalfEdgeIndex () &&
                this->operator                 == (FaceData (other)));
      }

      bool
      operator != (const FaceData& other) const
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

      HalfEdgeIndex idx_inner_half_edge_;
  };

} // End namespace pcl

#endif // FACE_HPP

