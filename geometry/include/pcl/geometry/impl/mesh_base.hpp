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

#ifndef MESH_BASE_HPP
#define MESH_BASE_HPP

#include <assert.h>
#include <utility>

#include <boost/type_traits/conditional.hpp>

#include <pcl/geometry/eigen.h>
#include <pcl/geometry/mesh_element_index.h>
#include <pcl/geometry/impl/vertex.hpp>
#include <pcl/geometry/impl/half_edge.hpp>
#include <pcl/geometry/impl/face.hpp>
#include <pcl/geometry/impl/mesh_circulators.hpp>

namespace pcl
{

  // TODO: This could be put in pcl/common
  // http://en.wikipedia.org/wiki/Substitution_failure_is_not_an_error
  template <class T>
  class HasEigenAlignedOperatorNew
  {
    private:

      typedef char Yes [1];
      typedef char No  [2];

      template <class U> static Yes&
      test (typename U::eigen_aligned_operator_new_marker_type*);

      template <class> static No&
      test (...);

    public:

      static const bool value = sizeof (test <T> (0)) == sizeof (Yes);
  };

} // End namespace pcl

namespace pcl
{

  struct EmptyVertexData   {bool operator == (const EmptyVertexData&  ) const {return (true);}};
  struct EmptyHalfEdgeData {bool operator == (const EmptyHalfEdgeData&) const {return (true);}};
  struct EmptyFaceData     {bool operator == (const EmptyFaceData&    ) const {return (true);}};

  template <class VertexDataT, class FaceDataT, class HalfEdgeDataT>
  class MeshBase
  {
    public:

      typedef pcl::MeshBase <VertexDataT, FaceDataT, HalfEdgeDataT> Self;

      typedef VertexDataT   VertexData;
      typedef HalfEdgeDataT HalfEdgeData;
      typedef FaceDataT     FaceData;

      typedef pcl::VertexIndex   VertexIndex;
      typedef pcl::HalfEdgeIndex HalfEdgeIndex;
      typedef pcl::FaceIndex     FaceIndex;

      typedef pcl::Vertex   <VertexData,   Self> Vertex;
      typedef pcl::HalfEdge <HalfEdgeData, Self> HalfEdge;
      typedef pcl::Face     <FaceData,     Self> Face;

      // TODO: Compare vector with deque
      typedef typename boost::conditional <pcl::HasEigenAlignedOperatorNew <Vertex>::value,   std::vector <Vertex,   Eigen::aligned_allocator <Vertex> >,   std::vector <Vertex> >::type   Vertexes;
      typedef typename boost::conditional <pcl::HasEigenAlignedOperatorNew <HalfEdge>::value, std::vector <HalfEdge, Eigen::aligned_allocator <HalfEdge> >, std::vector <HalfEdge> >::type HalfEdges;
      typedef typename boost::conditional <pcl::HasEigenAlignedOperatorNew <Face>::value,     std::vector <Face,     Eigen::aligned_allocator <Face> >,     std::vector <Face> >::type     Faces;

      typedef std::vector <VertexIndex>   VertexIndexes;
      typedef std::vector <HalfEdgeIndex> HalfEdgeIndexes;
      typedef std::vector <FaceIndex>     FaceIndexes;

      typedef typename Vertexes::size_type SizeType;

      // Iterators
      typedef typename Vertexes::iterator  VertexIterator;
      typedef typename HalfEdges::iterator HalfEdgeIterator;
      typedef typename Faces::iterator     FaceIterator;

      typedef typename Vertexes::const_iterator  VertexConstIterator;
      typedef typename HalfEdges::const_iterator HalfEdgeConstIterator;
      typedef typename Faces::const_iterator     FaceConstIterator;

      // Const iterators
      typedef typename VertexIndexes::iterator   VertexIndexIterator;
      typedef typename HalfEdgeIndexes::iterator HalfEdgeIndexIterator;
      typedef typename FaceIndexes::iterator     FaceIndexIterator;

      typedef typename VertexIndexes::const_iterator   VertexIndexConstIterator;
      typedef typename HalfEdgeIndexes::const_iterator HalfEdgeIndexConstIterator;
      typedef typename FaceIndexes::const_iterator     FaceIndexConstIterator;

      // Circulators
      typedef pcl::VertexAroundVertexCirculator           <Self> VertexAroundVertexCirculator;
      typedef pcl::OutgoingHalfEdgeAroundVertexCirculator <Self> OutgoingHalfEdgeAroundVertexCirculator;
      typedef pcl::IncomingHalfEdgeAroundVertexCirculator <Self> IncomingHalfEdgeAroundVertexCirculator;
      typedef pcl::FaceAroundVertexCirculator             <Self> FaceAroundVertexCirculator;
      typedef pcl::VertexAroundFaceCirculator             <Self> VertexAroundFaceCirculator;
      typedef pcl::InnerHalfEdgeAroundFaceCirculator      <Self> InnerHalfEdgeAroundFaceCirculator;
      typedef pcl::OuterHalfEdgeAroundFaceCirculator      <Self> OuterHalfEdgeAroundFaceCirculator;
      typedef pcl::FaceAroundFaceCirculator               <Self> FaceAroundFaceCirculator;
      typedef InnerHalfEdgeAroundFaceCirculator                  HalfEdgeAroundBoundaryCirculator;

      // Const circulators
      typedef pcl::VertexAroundVertexCirculator           <const Self> VertexAroundVertexConstCirculator;
      typedef pcl::OutgoingHalfEdgeAroundVertexCirculator <const Self> OutgoingHalfEdgeAroundVertexConstCirculator;
      typedef pcl::IncomingHalfEdgeAroundVertexCirculator <const Self> IncomingHalfEdgeAroundVertexConstCirculator;
      typedef pcl::FaceAroundVertexCirculator             <const Self> FaceAroundVertexConstCirculator;
      typedef pcl::VertexAroundFaceCirculator             <const Self> VertexAroundFaceConstCirculator;
      typedef pcl::InnerHalfEdgeAroundFaceCirculator      <const Self> InnerHalfEdgeAroundFaceConstCirculator;
      typedef pcl::OuterHalfEdgeAroundFaceCirculator      <const Self> OuterHalfEdgeAroundFaceConstCirculator;
      typedef pcl::FaceAroundFaceCirculator               <const Self> FaceAroundFaceConstCirculator;
      typedef InnerHalfEdgeAroundFaceConstCirculator                   HalfEdgeAroundBoundaryConstCirculator;

    public:

      MeshBase ()
        : vertexes_ (), half_edges_ (), faces_ ()
      {
      }

      MeshBase (const Self& other)
        : vertexes_   (other.beginVertexes  (), other.endVertexes ()),
          half_edges_ (other.beginHalfEdges (), other.endHalfEdges ()),
          faces_      (other.beginFaces     (), other.endFaces ())
      {
      }

    public:

      //////////////////////////////////////////////////////////////////////////
      // Begin / End
      //////////////////////////////////////////////////////////////////////////

      inline VertexIterator        beginVertexes ()        {return (vertexes_.begin ());}
      inline VertexConstIterator   beginVertexes () const  {return (vertexes_.begin ());}

      inline VertexIterator        endVertexes ()          {return (vertexes_.end ());}
      inline VertexConstIterator   endVertexes () const    {return (vertexes_.end ());}

      inline HalfEdgeIterator      beginHalfEdges ()       {return (half_edges_.begin ());}
      inline HalfEdgeConstIterator beginHalfEdges () const {return (half_edges_.begin ());}

      inline HalfEdgeIterator      endHalfEdges ()         {return (half_edges_.end ());}
      inline HalfEdgeConstIterator endHalfEdges () const   {return (half_edges_.end ());}

      inline FaceIterator          beginFaces ()           {return (faces_.begin ());}
      inline FaceConstIterator     beginFaces () const     {return (faces_.begin ());}

      inline FaceIterator          endFaces ()             {return (faces_.end ());}
      inline FaceConstIterator     endFaces () const       {return (faces_.end ());}

      //////////////////////////////////////////////////////////////////////////
      // Front / Back
      //////////////////////////////////////////////////////////////////////////

      inline Vertex&         frontVertexes ()        {return (vertexes_.front ());}
      inline const Vertex&   frontVertexes () const  {return (vertexes_.front ());}

      inline Vertex&         backVertexes ()         {return (vertexes_.back ());}
      inline const Vertex&   backVertexes () const   {return (vertexes_.back ());}

      inline HalfEdge&       frontHalfEdges ()       {return (half_edges_.front ());}
      inline const HalfEdge& frontHalfEdges () const {return (half_edges_.front ());}

      inline HalfEdge&       backHalfEdges ()        {return (half_edges_.back ());}
      inline const HalfEdge& backHalfEdges () const  {return (half_edges_.back ());}

      inline Face&           frontFaces ()           {return (faces_.front ());}
      inline const Face&     frontFaces () const     {return (faces_.front ());}

      inline Face&           backFaces ()            {return (faces_.back ());}
      inline const Face&     backFaces () const      {return (faces_.back ());}

      //////////////////////////////////////////////////////////////////////////
      // Size
      //////////////////////////////////////////////////////////////////////////

      inline SizeType sizeVertexes () const  {return (vertexes_.size ());}
      inline SizeType sizeHalfEdges () const {return (half_edges_.size ());}
      inline SizeType sizeFaces () const     {return (faces_.size ());}

      //////////////////////////////////////////////////////////////////////////
      // empty
      //////////////////////////////////////////////////////////////////////////

      inline bool empty () {return (vertexes_.empty () && half_edges_.empty () && faces_.empty ());}

      //////////////////////////////////////////////////////////////////////////
      // Reserve
      //////////////////////////////////////////////////////////////////////////

      inline void reserveVertexes (const SizeType n)  {vertexes_.reserve (n);}
      inline void reserveHalfEdges (const SizeType n) {half_edges_.reserve (n);}
      inline void reserveFaces (const SizeType n)     {faces_.reserve (n);}

      //////////////////////////////////////////////////////////////////////////
      // Clear
      //////////////////////////////////////////////////////////////////////////

      inline void clear ()
      {
        vertexes_.clear ();
        half_edges_.clear ();
        faces_.clear ();
      }

      //////////////////////////////////////////////////////////////////////////
      // Get the mesh elements
      //////////////////////////////////////////////////////////////////////////

      inline const Vertex&
      getElement (const VertexIndex& idx_vertex) const
      {
        assert (this->validateVertexIndex (idx_vertex));
        return (vertexes_ [idx_vertex.idx ()]);
      }

      inline Vertex&
      getElement (const VertexIndex& idx_vertex)
      {
        assert (this->validateVertexIndex (idx_vertex));
        return (vertexes_ [idx_vertex.idx ()]);
      }

      inline const HalfEdge&
      getElement (const HalfEdgeIndex& idx_half_edge) const
      {
        assert (this->validateHalfEdgeIndex (idx_half_edge));
        return (half_edges_ [idx_half_edge.idx ()]);
      }

      inline HalfEdge&
      getElement (const HalfEdgeIndex& idx_half_edge)
      {
        assert (this->validateHalfEdgeIndex (idx_half_edge));
        return (half_edges_ [idx_half_edge.idx ()]);
      }

      inline const Face&
      getElement (const FaceIndex& idx_face) const
      {
        assert (this->validateFaceIndex (idx_face));
        return (faces_ [idx_face.idx ()]);
      }

      inline Face&
      getElement (const FaceIndex& idx_face)
      {
        assert (this->validateFaceIndex (idx_face));
        return (faces_ [idx_face.idx ()]);
      }

      //////////////////////////////////////////////////////////////////////////
      // VertexAroundVertexCirculator
      //////////////////////////////////////////////////////////////////////////

      inline VertexAroundVertexCirculator
      getVertexAroundVertexCirculator (const Vertex& vertex)
      {
        assert (this->validateVertex (vertex));
        return (VertexAroundVertexCirculator (vertex, this));
      }

      inline VertexAroundVertexConstCirculator
      getVertexAroundVertexConstCirculator (const Vertex& vertex) const
      {
        assert (this->validateVertex (vertex));
        return (VertexAroundVertexConstCirculator (vertex, this));
      }

      inline VertexAroundVertexCirculator
      getVertexAroundVertexCirculator (const VertexIndex& idx_vertex)
      {
        assert (this->validateVertexIndex (idx_vertex));
        assert (this->validateVertex (this->getElement (idx_vertex)));
        return (VertexAroundVertexCirculator (idx_vertex, this));
      }

      inline VertexAroundVertexConstCirculator
      getVertexAroundVertexConstCirculator (const VertexIndex& idx_vertex) const
      {
        assert (this->validateVertexIndex (idx_vertex));
        assert (this->validateVertex (this->getElement (idx_vertex)));
        return (VertexAroundVertexConstCirculator (idx_vertex, this));
      }

      inline VertexAroundVertexCirculator
      getVertexAroundVertexCirculator (const HalfEdgeIndex& idx_outgoing_half_edge)
      {
        assert (this->validateHalfEdgeIndex (idx_outgoing_half_edge));
        return (VertexAroundVertexCirculator (idx_outgoing_half_edge, this));
      }

      inline VertexAroundVertexConstCirculator
      getVertexAroundVertexConstCirculator (const HalfEdgeIndex& idx_outgoing_half_edge) const
      {
        assert (this->validateHalfEdgeIndex (idx_outgoing_half_edge));
        return (VertexAroundVertexConstCirculator (idx_outgoing_half_edge, this));
      }

      //////////////////////////////////////////////////////////////////////////
      // OutgoingHalfEdgeAroundVertexCirculator
      //////////////////////////////////////////////////////////////////////////

      inline OutgoingHalfEdgeAroundVertexCirculator
      getOutgoingHalfEdgeAroundVertexCirculator (const Vertex& vertex)
      {
        assert (this->validateVertex (vertex));
        return (OutgoingHalfEdgeAroundVertexCirculator (vertex, this));
      }

      inline OutgoingHalfEdgeAroundVertexConstCirculator
      getOutgoingHalfEdgeAroundVertexConstCirculator (const Vertex& vertex) const
      {
        assert (this->validateVertex (vertex));
        return (OutgoingHalfEdgeAroundVertexConstCirculator (vertex, this));
      }

      inline OutgoingHalfEdgeAroundVertexCirculator
      getOutgoingHalfEdgeAroundVertexCirculator (const VertexIndex& idx_vertex)
      {
        assert (this->validateVertexIndex (idx_vertex));
        assert (this->validateVertex (this->getElement (idx_vertex)));
        return (OutgoingHalfEdgeAroundVertexCirculator (idx_vertex, this));
      }

      inline OutgoingHalfEdgeAroundVertexConstCirculator
      getOutgoingHalfEdgeAroundVertexConstCirculator (const VertexIndex& idx_vertex) const
      {
        assert (this->validateVertexIndex (idx_vertex));
        assert (this->validateVertex (this->getElement (idx_vertex)));
        return (OutgoingHalfEdgeAroundVertexConstCirculator (idx_vertex, this));
      }

      inline OutgoingHalfEdgeAroundVertexCirculator
      getOutgoingHalfEdgeAroundVertexCirculator (const HalfEdgeIndex& idx_outgoing_half_edge)
      {
        assert (this->validateHalfEdgeIndex (idx_outgoing_half_edge));
        return (OutgoingHalfEdgeAroundVertexCirculator (idx_outgoing_half_edge, this));
      }

      inline OutgoingHalfEdgeAroundVertexConstCirculator
      getOutgoingHalfEdgeAroundVertexConstCirculator (const HalfEdgeIndex& idx_outgoing_half_edge) const
      {
        assert (this->validateHalfEdgeIndex (idx_outgoing_half_edge));
        return (OutgoingHalfEdgeAroundVertexConstCirculator (idx_outgoing_half_edge, this));
      }

      //////////////////////////////////////////////////////////////////////////
      // IncomingHalfEdgeAroundVertexCirculator
      //////////////////////////////////////////////////////////////////////////

      inline IncomingHalfEdgeAroundVertexCirculator
      getIncomingHalfEdgeAroundVertexCirculator (const Vertex& vertex)
      {
        assert (this->validateVertex (vertex));
        return (IncomingHalfEdgeAroundVertexCirculator (vertex, this));
      }

      inline IncomingHalfEdgeAroundVertexConstCirculator
      getIncomingHalfEdgeAroundVertexConstCirculator (const Vertex& vertex) const
      {
        assert (this->validateVertex (vertex));
        return (IncomingHalfEdgeAroundVertexConstCirculator (vertex, this));
      }

      inline IncomingHalfEdgeAroundVertexCirculator
      getIncomingHalfEdgeAroundVertexCirculator (const VertexIndex& idx_vertex)
      {
        assert (this->validateVertexIndex (idx_vertex));
        assert (this->validateVertex (this->getElement (idx_vertex)));
        return (IncomingHalfEdgeAroundVertexCirculator (idx_vertex, this));
      }

      inline IncomingHalfEdgeAroundVertexConstCirculator
      getIncomingHalfEdgeAroundVertexConstCirculator (const VertexIndex& idx_vertex) const
      {
        assert (this->validateVertexIndex (idx_vertex));
        assert (this->validateVertex (this->getElement (idx_vertex)));
        return (IncomingHalfEdgeAroundVertexConstCirculator (idx_vertex, this));
      }

      inline IncomingHalfEdgeAroundVertexCirculator
      getIncomingHalfEdgeAroundVertexCirculator (const HalfEdgeIndex& idx_incoming_half_edge)
      {
        assert (this->validateHalfEdgeIndex (idx_incoming_half_edge));
        return (IncomingHalfEdgeAroundVertexCirculator (idx_incoming_half_edge, this));
      }

      inline IncomingHalfEdgeAroundVertexConstCirculator
      getIncomingHalfEdgeAroundVertexConstCirculator (const HalfEdgeIndex& idx_incoming_half_edge) const
      {
        assert (this->validateHalfEdgeIndex (idx_incoming_half_edge));
        return (IncomingHalfEdgeAroundVertexConstCirculator (idx_incoming_half_edge, this));
      }

      //////////////////////////////////////////////////////////////////////////
      // FaceAroundVertexCirculator
      //////////////////////////////////////////////////////////////////////////

      inline FaceAroundVertexCirculator
      getFaceAroundVertexCirculator (const Vertex& vertex)
      {
        assert (this->validateVertex (vertex));
        return (FaceAroundVertexCirculator (vertex, this));
      }

      inline FaceAroundVertexConstCirculator
      getFaceAroundVertexConstCirculator (const Vertex& vertex) const
      {
        assert (this->validateVertex (vertex));
        return (FaceAroundVertexConstCirculator (vertex, this));
      }

      inline FaceAroundVertexCirculator
      getFaceAroundVertexCirculator (const VertexIndex& idx_vertex)
      {
        assert (this->validateVertexIndex (idx_vertex));
        assert (this->validateVertex (this->getElement (idx_vertex)));
        return (FaceAroundVertexCirculator (idx_vertex, this));
      }

      inline FaceAroundVertexConstCirculator
      getFaceAroundVertexConstCirculator (const VertexIndex& idx_vertex) const
      {
        assert (this->validateVertexIndex (idx_vertex));
        assert (this->validateVertex (this->getElement (idx_vertex)));
        return (FaceAroundVertexConstCirculator (idx_vertex, this));
      }

      inline FaceAroundVertexCirculator
      getFaceAroundVertexCirculator (const HalfEdgeIndex& idx_outgoing_half_edge)
      {
        assert (this->validateHalfEdgeIndex (idx_outgoing_half_edge));
        return (FaceAroundVertexCirculator (idx_outgoing_half_edge, this));
      }

      inline FaceAroundVertexConstCirculator
      getFaceAroundVertexConstCirculator (const HalfEdgeIndex& idx_outgoing_half_edge) const
      {
        assert (this->validateHalfEdgeIndex (idx_outgoing_half_edge));
        return (FaceAroundVertexConstCirculator (idx_outgoing_half_edge, this));
      }

      //////////////////////////////////////////////////////////////////////////
      // VertexAroundFaceCirculator
      //////////////////////////////////////////////////////////////////////////

      inline VertexAroundFaceCirculator
      getVertexAroundFaceCirculator (const Face& face)
      {
        assert (this->validateFace (face));
        return (VertexAroundFaceCirculator (face, this));
      }

      inline VertexAroundFaceConstCirculator
      getVertexAroundFaceConstCirculator (const Face& face) const
      {
        assert (this->validateFace (face));
        return (VertexAroundFaceConstCirculator (face, this));
      }

      inline VertexAroundFaceCirculator
      getVertexAroundFaceCirculator (const FaceIndex& idx_face)
      {
        assert (this->validateFaceIndex (idx_face));
        assert (this->validateFace (this->getElement (idx_face)));
        return (VertexAroundFaceCirculator (idx_face, this));
      }

      inline VertexAroundFaceConstCirculator
      getVertexAroundFaceConstCirculator (const FaceIndex& idx_face) const
      {
        assert (this->validateFaceIndex (idx_face));
        assert (this->validateFace (this->getElement (idx_face)));
        return (VertexAroundFaceConstCirculator (idx_face, this));
      }

      inline VertexAroundFaceCirculator
      getVertexAroundFaceCirculator (const HalfEdgeIndex& idx_inner_half_edge)
      {
        assert (this->validateHalfEdgeIndex (idx_inner_half_edge));
        return (VertexAroundFaceCirculator (idx_inner_half_edge, this));
      }

      inline VertexAroundFaceConstCirculator
      getVertexAroundFaceConstCirculator (const HalfEdgeIndex& idx_inner_half_edge) const
      {
        assert (this->validateHalfEdgeIndex (idx_inner_half_edge));
        return (VertexAroundFaceConstCirculator (idx_inner_half_edge, this));
      }

      //////////////////////////////////////////////////////////////////////////
      // InnerHalfEdgeAroundFaceCirculator
      //////////////////////////////////////////////////////////////////////////

      inline InnerHalfEdgeAroundFaceCirculator
      getInnerHalfEdgeAroundFaceCirculator (const Face& face)
      {
        assert (this->validateFace (face));
        return (InnerHalfEdgeAroundFaceCirculator (face, this));
      }

      inline InnerHalfEdgeAroundFaceConstCirculator
      getInnerHalfEdgeAroundFaceConstCirculator (const Face& face) const
      {
        assert (this->validateFace (face));
        return (InnerHalfEdgeAroundFaceConstCirculator (face, this));
      }

      inline InnerHalfEdgeAroundFaceCirculator
      getInnerHalfEdgeAroundFaceCirculator (const FaceIndex& idx_face)
      {
        assert (this->validateFaceIndex (idx_face));
        assert (this->validateFace (this->getElement (idx_face)));
        return (InnerHalfEdgeAroundFaceCirculator (idx_face, this));
      }

      inline InnerHalfEdgeAroundFaceConstCirculator
      getInnerHalfEdgeAroundFaceConstCirculator (const FaceIndex& idx_face) const
      {
        assert (this->validateFaceIndex (idx_face));
        assert (this->validateFace (this->getElement (idx_face)));
        return (InnerHalfEdgeAroundFaceConstCirculator (idx_face, this));
      }

      inline InnerHalfEdgeAroundFaceCirculator
      getInnerHalfEdgeAroundFaceCirculator (const HalfEdgeIndex& idx_inner_half_edge)
      {
        assert (this->validateHalfEdgeIndex (idx_inner_half_edge));
        return (InnerHalfEdgeAroundFaceCirculator (idx_inner_half_edge, this));
      }

      inline InnerHalfEdgeAroundFaceConstCirculator
      getInnerHalfEdgeAroundFaceConstCirculator (const HalfEdgeIndex& idx_inner_half_edge) const
      {
        assert (this->validateHalfEdgeIndex (idx_inner_half_edge));
        return (InnerHalfEdgeAroundFaceConstCirculator (idx_inner_half_edge, this));
      }

      //////////////////////////////////////////////////////////////////////////
      // OuterHalfEdgeAroundFaceCirculator
      //////////////////////////////////////////////////////////////////////////

      inline OuterHalfEdgeAroundFaceCirculator
      getOuterHalfEdgeAroundFaceCirculator (const Face& face)
      {
        assert (this->validateFace (face));
        return (OuterHalfEdgeAroundFaceCirculator (face, this));
      }

      inline OuterHalfEdgeAroundFaceConstCirculator
      getOuterHalfEdgeAroundFaceConstCirculator (const Face& face) const
      {
        assert (this->validateFace (face));
        return (OuterHalfEdgeAroundFaceConstCirculator (face, this));
      }

      inline OuterHalfEdgeAroundFaceCirculator
      getOuterHalfEdgeAroundFaceCirculator (const FaceIndex& idx_face)
      {
        assert (this->validateFaceIndex (idx_face));
        assert (this->validateFace (this->getElement (idx_face)));
        return (OuterHalfEdgeAroundFaceCirculator (idx_face, this));
      }

      inline OuterHalfEdgeAroundFaceConstCirculator
      getOuterHalfEdgeAroundFaceConstCirculator (const FaceIndex& idx_face) const
      {
        assert (this->validateFaceIndex (idx_face));
        assert (this->validateFace (this->getElement (idx_face)));
        return (OuterHalfEdgeAroundFaceConstCirculator (idx_face, this));
      }

      inline OuterHalfEdgeAroundFaceCirculator
      getOuterHalfEdgeAroundFaceCirculator (const HalfEdgeIndex& idx_inner_half_edge)
      {
        assert (this->validateHalfEdgeIndex (idx_inner_half_edge));
        return (OuterHalfEdgeAroundFaceCirculator (idx_inner_half_edge, this));
      }

      inline OuterHalfEdgeAroundFaceConstCirculator
      getOuterHalfEdgeAroundFaceConstCirculator (const HalfEdgeIndex& idx_inner_half_edge) const
      {
        assert (this->validateHalfEdgeIndex (idx_inner_half_edge));
        return (OuterHalfEdgeAroundFaceConstCirculator (idx_inner_half_edge, this));
      }

      //////////////////////////////////////////////////////////////////////////
      // FaceAroundFaceCirculator
      //////////////////////////////////////////////////////////////////////////

      inline FaceAroundFaceCirculator
      getFaceAroundFaceCirculator (const Face& face)
      {
        assert (this->validateFace (face));
        return (FaceAroundFaceCirculator (face, this));
      }

      inline FaceAroundFaceConstCirculator
      getFaceAroundFaceConstCirculator (const Face& face) const
      {
        assert (this->validateFace (face));
        return (FaceAroundFaceConstCirculator (face, this));
      }

      inline FaceAroundFaceCirculator
      getFaceAroundFaceCirculator (const FaceIndex& idx_face)
      {
        assert (this->validateFaceIndex (idx_face));
        assert (this->validateFace (this->getElement (idx_face)));
        return (FaceAroundFaceCirculator (idx_face, this));
      }

      inline FaceAroundFaceConstCirculator
      getFaceAroundFaceConstCirculator (const FaceIndex& idx_face) const
      {
        assert (this->validateFaceIndex (idx_face));
        assert (this->validateFace (this->getElement (idx_face)));
        return (FaceAroundFaceConstCirculator (idx_face, this));
      }

      inline FaceAroundFaceCirculator
      getFaceAroundFaceCirculator (const HalfEdgeIndex& idx_inner_half_edge)
      {
        assert (this->validateHalfEdgeIndex (idx_inner_half_edge));
        return (FaceAroundFaceCirculator (idx_inner_half_edge, this));
      }

      inline FaceAroundFaceConstCirculator
      getFaceAroundFaceConstCirculator (const HalfEdgeIndex& idx_inner_half_edge) const
      {
        assert (this->validateHalfEdgeIndex (idx_inner_half_edge));
        return (FaceAroundFaceConstCirculator (idx_inner_half_edge, this));
      }

      //////////////////////////////////////////////////////////////////////////
      // HalfEdgeAroundBoundaryCirculator
      //////////////////////////////////////////////////////////////////////////

      inline HalfEdgeAroundBoundaryCirculator
      getHalfEdgeAroundBoundaryCirculator (const HalfEdgeIndex& idx_boundary_half_edge)
      {
        assert (this->validateHalfEdgeIndex (idx_boundary_half_edge));
        return (this->getInnerHalfEdgeAroundFaceCirculator (idx_boundary_half_edge));
      }

      inline HalfEdgeAroundBoundaryConstCirculator
      getHalfEdgeAroundBoundaryConstCirculator (const HalfEdgeIndex& idx_boundary_half_edge) const
      {
        assert (this->validateHalfEdgeIndex (idx_boundary_half_edge));
        return (this->getInnerHalfEdgeAroundFaceConstCirculator (idx_boundary_half_edge));
      }

    protected:

      //////////////////////////////////////////////////////////////////////////
      // Push back
      //////////////////////////////////////////////////////////////////////////

      inline VertexIndex
      pushBackVertex (const Vertex& vertex)
      {
        vertexes_.push_back (vertex);
        return (VertexIndex (this->sizeVertexes ()-1));
      }

      inline VertexIndex
      pushBackVertex (const VertexData&    vertex_data             = VertexData (),
                      const HalfEdgeIndex& idx_outgoing_half_edge_ = HalfEdgeIndex ())
      {
        vertexes_.push_back (Vertex (vertex_data, idx_outgoing_half_edge_));
        return (VertexIndex (this->sizeVertexes ()-1));
      }

      inline HalfEdgeIndex
      pushBackHalfEdge (const HalfEdge& half_edge)
      {
        half_edges_.push_back (half_edge);
        return (HalfEdgeIndex (this->sizeHalfEdges ()-1));
      }

      inline HalfEdgeIndex
      pushBackHalfEdge (const HalfEdgeData&  half_edge_data         = HalfEdgeData  (),
                        const VertexIndex&   idx_terminating_vertex = VertexIndex   (),
                        const HalfEdgeIndex& idx_opposite_half_edge = HalfEdgeIndex (),
                        const HalfEdgeIndex& idx_next_half_edge     = HalfEdgeIndex (),
                        const HalfEdgeIndex& idx_prev_half_edge     = HalfEdgeIndex (),
                        const FaceIndex&     idx_face               = FaceIndex     ())
      {
        half_edges_.push_back (HalfEdge (half_edge_data, idx_terminating_vertex, idx_opposite_half_edge, idx_next_half_edge, idx_prev_half_edge, idx_face));
        return (HalfEdgeIndex (this->sizeHalfEdges ()-1));
      }

      inline FaceIndex
      pushBackFace (const Face& face)
      {
        faces_.push_back (face);
        return (FaceIndex (this->sizeFaces ()-1));
      }

      inline FaceIndex
      pushBackFace (const FaceData&          face_data           = FaceData (),
                    const HalfEdgeIndex& idx_inner_half_edge = HalfEdgeIndex ())
      {
        faces_.push_back (Face (face_data, idx_inner_half_edge));
        return (FaceIndex (this->sizeFaces ()-1));
      }

      //////////////////////////////////////////////////////////////////////////
      // For the assertions
      //////////////////////////////////////////////////////////////////////////

      inline bool
      validateVertex (const Vertex& vertex) const
      {
        return (this->validateHalfEdgeIndex (vertex.getOutgoingHalfEdgeIndex ()));
      }

      inline bool
      validateVertexIndex (const VertexIndex& idx_vertex) const
      {
        if (!idx_vertex.isValid ())              return (false);
        if (idx_vertex >= this->sizeVertexes ()) return (false);
        return (true);
      }

      inline bool
      validateHalfEdgeIndex (const HalfEdgeIndex& idx_half_edge) const
      {
        if (!idx_half_edge.isValid ())               return (false);
        if (idx_half_edge >= this->sizeHalfEdges ()) return (false);
        return (true);
      }

      inline bool
      validateFace (const Face& face) const
      {
        return (this->validateHalfEdgeIndex (face.getInnerHalfEdgeIndex ()));
      }

      inline bool
      validateFaceIndex (const FaceIndex& idx_face) const
      {
        if (!idx_face.isValid ())           return (false);
        if (idx_face >= this->sizeFaces ()) return (false);
        return (true);
      }

    protected:

      Vertexes  vertexes_;
      HalfEdges half_edges_;
      Faces     faces_;
  };

} // End namespace pcl

#endif // MESH_BASE_HPP

