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

#ifndef PCL_GEOMETRY_MESH_BASE_HPP
#define PCL_GEOMETRY_MESH_BASE_HPP

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
        : vertexes_ (),
          half_edges_ (),
          faces_ ()
      {
      }

    public:

      //////////////////////////////////////////////////////////////////////////
      // Begin / End
      //////////////////////////////////////////////////////////////////////////

      VertexIterator        beginVertexes ()        {return (vertexes_.begin ());}
      VertexConstIterator   beginVertexes () const  {return (vertexes_.begin ());}

      VertexIterator        endVertexes ()          {return (vertexes_.end ());}
      VertexConstIterator   endVertexes () const    {return (vertexes_.end ());}

      HalfEdgeIterator      beginHalfEdges ()       {return (half_edges_.begin ());}
      HalfEdgeConstIterator beginHalfEdges () const {return (half_edges_.begin ());}

      HalfEdgeIterator      endHalfEdges ()         {return (half_edges_.end ());}
      HalfEdgeConstIterator endHalfEdges () const   {return (half_edges_.end ());}

      FaceIterator          beginFaces ()           {return (faces_.begin ());}
      FaceConstIterator     beginFaces () const     {return (faces_.begin ());}

      FaceIterator          endFaces ()             {return (faces_.end ());}
      FaceConstIterator     endFaces () const       {return (faces_.end ());}

      //////////////////////////////////////////////////////////////////////////
      // Front / Back
      //////////////////////////////////////////////////////////////////////////

      Vertex&         frontVertexes ()        {return (vertexes_.front ());}
      const Vertex&   frontVertexes () const  {return (vertexes_.front ());}

      Vertex&         backVertexes ()         {return (vertexes_.back ());}
      const Vertex&   backVertexes () const   {return (vertexes_.back ());}

      HalfEdge&       frontHalfEdges ()       {return (half_edges_.front ());}
      const HalfEdge& frontHalfEdges () const {return (half_edges_.front ());}

      HalfEdge&       backHalfEdges ()        {return (half_edges_.back ());}
      const HalfEdge& backHalfEdges () const  {return (half_edges_.back ());}

      Face&           frontFaces ()           {return (faces_.front ());}
      const Face&     frontFaces () const     {return (faces_.front ());}

      Face&           backFaces ()            {return (faces_.back ());}
      const Face&     backFaces () const      {return (faces_.back ());}

      //////////////////////////////////////////////////////////////////////////
      // Size
      //////////////////////////////////////////////////////////////////////////

      SizeType sizeVertexes () const  {return (vertexes_.size ());}
      SizeType sizeHalfEdges () const {return (half_edges_.size ());}
      SizeType sizeFaces () const     {return (faces_.size ());}

      //////////////////////////////////////////////////////////////////////////
      // empty
      //////////////////////////////////////////////////////////////////////////

      bool empty () {return (vertexes_.empty () && half_edges_.empty () && faces_.empty ());}

      //////////////////////////////////////////////////////////////////////////
      // Reserve
      //////////////////////////////////////////////////////////////////////////

      void reserveVertexes (const SizeType n)  {vertexes_.reserve (n);}
      void reserveHalfEdges (const SizeType n) {half_edges_.reserve (n);}
      void reserveFaces (const SizeType n)     {faces_.reserve (n);}

      //////////////////////////////////////////////////////////////////////////
      // Clear
      //////////////////////////////////////////////////////////////////////////

      void clear ()
      {
        vertexes_.clear ();
        half_edges_.clear ();
        faces_.clear ();
      }

      //////////////////////////////////////////////////////////////////////////
      // Get the mesh elements
      //////////////////////////////////////////////////////////////////////////

      const Vertex&
      getElement (const VertexIndex& idx_vertex) const
      {
        assert (this->validateMeshElementIndex (idx_vertex));
        return (vertexes_ [idx_vertex.getIndex ()]);
      }

      Vertex&
      getElement (const VertexIndex& idx_vertex)
      {
        assert (this->validateMeshElementIndex (idx_vertex));
        return (vertexes_ [idx_vertex.getIndex ()]);
      }

      const HalfEdge&
      getElement (const HalfEdgeIndex& idx_half_edge) const
      {
        assert (this->validateMeshElementIndex (idx_half_edge));
        return (half_edges_ [idx_half_edge.getIndex ()]);
      }

      HalfEdge&
      getElement (const HalfEdgeIndex& idx_half_edge)
      {
        assert (this->validateMeshElementIndex (idx_half_edge));
        return (half_edges_ [idx_half_edge.getIndex ()]);
      }

      const Face&
      getElement (const FaceIndex& idx_face) const
      {
        assert (this->validateMeshElementIndex (idx_face));
        return (faces_ [idx_face.getIndex ()]);
      }

      Face&
      getElement (const FaceIndex& idx_face)
      {
        assert (this->validateMeshElementIndex (idx_face));
        return (faces_ [idx_face.getIndex ()]);
      }

      //////////////////////////////////////////////////////////////////////////
      // VertexAroundVertexCirculator
      //////////////////////////////////////////////////////////////////////////

      VertexAroundVertexCirculator
      getVertexAroundVertexCirculator (const Vertex& vertex)
      {
        assert (this->validateMeshElement (vertex));
        return (VertexAroundVertexCirculator (vertex, this));
      }

      VertexAroundVertexConstCirculator
      getVertexAroundVertexConstCirculator (const Vertex& vertex) const
      {
        assert (this->validateMeshElement (vertex));
        return (VertexAroundVertexConstCirculator (vertex, this));
      }

      VertexAroundVertexCirculator
      getVertexAroundVertexCirculator (const VertexIndex& idx_vertex)
      {
        assert (this->validateMeshElementIndex (idx_vertex));
        assert (this->validateMeshElement (this->getElement (idx_vertex)));
        return (VertexAroundVertexCirculator (idx_vertex, this));
      }

      VertexAroundVertexConstCirculator
      getVertexAroundVertexConstCirculator (const VertexIndex& idx_vertex) const
      {
        assert (this->validateMeshElementIndex (idx_vertex));
        assert (this->validateMeshElement (this->getElement (idx_vertex)));
        return (VertexAroundVertexConstCirculator (idx_vertex, this));
      }

      VertexAroundVertexCirculator
      getVertexAroundVertexCirculator (const HalfEdgeIndex& idx_outgoing_half_edge)
      {
        assert (this->validateMeshElementIndex (idx_outgoing_half_edge));
        return (VertexAroundVertexCirculator (idx_outgoing_half_edge, this));
      }

      VertexAroundVertexConstCirculator
      getVertexAroundVertexConstCirculator (const HalfEdgeIndex& idx_outgoing_half_edge) const
      {
        assert (this->validateMeshElementIndex (idx_outgoing_half_edge));
        return (VertexAroundVertexConstCirculator (idx_outgoing_half_edge, this));
      }

      //////////////////////////////////////////////////////////////////////////
      // OutgoingHalfEdgeAroundVertexCirculator
      //////////////////////////////////////////////////////////////////////////

      OutgoingHalfEdgeAroundVertexCirculator
      getOutgoingHalfEdgeAroundVertexCirculator (const Vertex& vertex)
      {
        assert (this->validateMeshElement (vertex));
        return (OutgoingHalfEdgeAroundVertexCirculator (vertex, this));
      }

      OutgoingHalfEdgeAroundVertexConstCirculator
      getOutgoingHalfEdgeAroundVertexConstCirculator (const Vertex& vertex) const
      {
        assert (this->validateMeshElement (vertex));
        return (OutgoingHalfEdgeAroundVertexConstCirculator (vertex, this));
      }

      OutgoingHalfEdgeAroundVertexCirculator
      getOutgoingHalfEdgeAroundVertexCirculator (const VertexIndex& idx_vertex)
      {
        assert (this->validateMeshElementIndex (idx_vertex));
        assert (this->validateMeshElement (this->getElement (idx_vertex)));
        return (OutgoingHalfEdgeAroundVertexCirculator (idx_vertex, this));
      }

      OutgoingHalfEdgeAroundVertexConstCirculator
      getOutgoingHalfEdgeAroundVertexConstCirculator (const VertexIndex& idx_vertex) const
      {
        assert (this->validateMeshElementIndex (idx_vertex));
        assert (this->validateMeshElement (this->getElement (idx_vertex)));
        return (OutgoingHalfEdgeAroundVertexConstCirculator (idx_vertex, this));
      }

      OutgoingHalfEdgeAroundVertexCirculator
      getOutgoingHalfEdgeAroundVertexCirculator (const HalfEdgeIndex& idx_outgoing_half_edge)
      {
        assert (this->validateMeshElementIndex (idx_outgoing_half_edge));
        return (OutgoingHalfEdgeAroundVertexCirculator (idx_outgoing_half_edge, this));
      }

      OutgoingHalfEdgeAroundVertexConstCirculator
      getOutgoingHalfEdgeAroundVertexConstCirculator (const HalfEdgeIndex& idx_outgoing_half_edge) const
      {
        assert (this->validateMeshElementIndex (idx_outgoing_half_edge));
        return (OutgoingHalfEdgeAroundVertexConstCirculator (idx_outgoing_half_edge, this));
      }

      //////////////////////////////////////////////////////////////////////////
      // IncomingHalfEdgeAroundVertexCirculator
      //////////////////////////////////////////////////////////////////////////

      IncomingHalfEdgeAroundVertexCirculator
      getIncomingHalfEdgeAroundVertexCirculator (const Vertex& vertex)
      {
        assert (this->validateMeshElement (vertex));
        return (IncomingHalfEdgeAroundVertexCirculator (vertex, this));
      }

      IncomingHalfEdgeAroundVertexConstCirculator
      getIncomingHalfEdgeAroundVertexConstCirculator (const Vertex& vertex) const
      {
        assert (this->validateMeshElement (vertex));
        return (IncomingHalfEdgeAroundVertexConstCirculator (vertex, this));
      }

      IncomingHalfEdgeAroundVertexCirculator
      getIncomingHalfEdgeAroundVertexCirculator (const VertexIndex& idx_vertex)
      {
        assert (this->validateMeshElementIndex (idx_vertex));
        assert (this->validateMeshElement (this->getElement (idx_vertex)));
        return (IncomingHalfEdgeAroundVertexCirculator (idx_vertex, this));
      }

      IncomingHalfEdgeAroundVertexConstCirculator
      getIncomingHalfEdgeAroundVertexConstCirculator (const VertexIndex& idx_vertex) const
      {
        assert (this->validateMeshElementIndex (idx_vertex));
        assert (this->validateMeshElement (this->getElement (idx_vertex)));
        return (IncomingHalfEdgeAroundVertexConstCirculator (idx_vertex, this));
      }

      IncomingHalfEdgeAroundVertexCirculator
      getIncomingHalfEdgeAroundVertexCirculator (const HalfEdgeIndex& idx_incoming_half_edge)
      {
        assert (this->validateMeshElementIndex (idx_incoming_half_edge));
        return (IncomingHalfEdgeAroundVertexCirculator (idx_incoming_half_edge, this));
      }

      IncomingHalfEdgeAroundVertexConstCirculator
      getIncomingHalfEdgeAroundVertexConstCirculator (const HalfEdgeIndex& idx_incoming_half_edge) const
      {
        assert (this->validateMeshElementIndex (idx_incoming_half_edge));
        return (IncomingHalfEdgeAroundVertexConstCirculator (idx_incoming_half_edge, this));
      }

      //////////////////////////////////////////////////////////////////////////
      // FaceAroundVertexCirculator
      //////////////////////////////////////////////////////////////////////////

      FaceAroundVertexCirculator
      getFaceAroundVertexCirculator (const Vertex& vertex)
      {
        assert (this->validateMeshElement (vertex));
        return (FaceAroundVertexCirculator (vertex, this));
      }

      FaceAroundVertexConstCirculator
      getFaceAroundVertexConstCirculator (const Vertex& vertex) const
      {
        assert (this->validateMeshElement (vertex));
        return (FaceAroundVertexConstCirculator (vertex, this));
      }

      FaceAroundVertexCirculator
      getFaceAroundVertexCirculator (const VertexIndex& idx_vertex)
      {
        assert (this->validateMeshElementIndex (idx_vertex));
        assert (this->validateMeshElement (this->getElement (idx_vertex)));
        return (FaceAroundVertexCirculator (idx_vertex, this));
      }

      FaceAroundVertexConstCirculator
      getFaceAroundVertexConstCirculator (const VertexIndex& idx_vertex) const
      {
        assert (this->validateMeshElementIndex (idx_vertex));
        assert (this->validateMeshElement (this->getElement (idx_vertex)));
        return (FaceAroundVertexConstCirculator (idx_vertex, this));
      }

      FaceAroundVertexCirculator
      getFaceAroundVertexCirculator (const HalfEdgeIndex& idx_outgoing_half_edge)
      {
        assert (this->validateMeshElementIndex (idx_outgoing_half_edge));
        return (FaceAroundVertexCirculator (idx_outgoing_half_edge, this));
      }

      FaceAroundVertexConstCirculator
      getFaceAroundVertexConstCirculator (const HalfEdgeIndex& idx_outgoing_half_edge) const
      {
        assert (this->validateMeshElementIndex (idx_outgoing_half_edge));
        return (FaceAroundVertexConstCirculator (idx_outgoing_half_edge, this));
      }

      //////////////////////////////////////////////////////////////////////////
      // VertexAroundFaceCirculator
      //////////////////////////////////////////////////////////////////////////

      VertexAroundFaceCirculator
      getVertexAroundFaceCirculator (const Face& face)
      {
        assert (this->validateMeshElement (face));
        return (VertexAroundFaceCirculator (face, this));
      }

      VertexAroundFaceConstCirculator
      getVertexAroundFaceConstCirculator (const Face& face) const
      {
        assert (this->validateMeshElement (face));
        return (VertexAroundFaceConstCirculator (face, this));
      }

      VertexAroundFaceCirculator
      getVertexAroundFaceCirculator (const FaceIndex& idx_face)
      {
        assert (this->validateMeshElementIndex (idx_face));
        assert (this->validateMeshElement (this->getElement (idx_face)));
        return (VertexAroundFaceCirculator (idx_face, this));
      }

      VertexAroundFaceConstCirculator
      getVertexAroundFaceConstCirculator (const FaceIndex& idx_face) const
      {
        assert (this->validateMeshElementIndex (idx_face));
        assert (this->validateMeshElement (this->getElement (idx_face)));
        return (VertexAroundFaceConstCirculator (idx_face, this));
      }

      VertexAroundFaceCirculator
      getVertexAroundFaceCirculator (const HalfEdgeIndex& idx_inner_half_edge)
      {
        assert (this->validateMeshElementIndex (idx_inner_half_edge));
        return (VertexAroundFaceCirculator (idx_inner_half_edge, this));
      }

      VertexAroundFaceConstCirculator
      getVertexAroundFaceConstCirculator (const HalfEdgeIndex& idx_inner_half_edge) const
      {
        assert (this->validateMeshElementIndex (idx_inner_half_edge));
        return (VertexAroundFaceConstCirculator (idx_inner_half_edge, this));
      }

      //////////////////////////////////////////////////////////////////////////
      // InnerHalfEdgeAroundFaceCirculator
      //////////////////////////////////////////////////////////////////////////

      InnerHalfEdgeAroundFaceCirculator
      getInnerHalfEdgeAroundFaceCirculator (const Face& face)
      {
        assert (this->validateMeshElement (face));
        return (InnerHalfEdgeAroundFaceCirculator (face, this));
      }

      InnerHalfEdgeAroundFaceConstCirculator
      getInnerHalfEdgeAroundFaceConstCirculator (const Face& face) const
      {
        assert (this->validateMeshElement (face));
        return (InnerHalfEdgeAroundFaceConstCirculator (face, this));
      }

      InnerHalfEdgeAroundFaceCirculator
      getInnerHalfEdgeAroundFaceCirculator (const FaceIndex& idx_face)
      {
        assert (this->validateMeshElementIndex (idx_face));
        assert (this->validateMeshElement (this->getElement (idx_face)));
        return (InnerHalfEdgeAroundFaceCirculator (idx_face, this));
      }

      InnerHalfEdgeAroundFaceConstCirculator
      getInnerHalfEdgeAroundFaceConstCirculator (const FaceIndex& idx_face) const
      {
        assert (this->validateMeshElementIndex (idx_face));
        assert (this->validateMeshElement (this->getElement (idx_face)));
        return (InnerHalfEdgeAroundFaceConstCirculator (idx_face, this));
      }

      InnerHalfEdgeAroundFaceCirculator
      getInnerHalfEdgeAroundFaceCirculator (const HalfEdgeIndex& idx_inner_half_edge)
      {
        assert (this->validateMeshElementIndex (idx_inner_half_edge));
        return (InnerHalfEdgeAroundFaceCirculator (idx_inner_half_edge, this));
      }

      InnerHalfEdgeAroundFaceConstCirculator
      getInnerHalfEdgeAroundFaceConstCirculator (const HalfEdgeIndex& idx_inner_half_edge) const
      {
        assert (this->validateMeshElementIndex (idx_inner_half_edge));
        return (InnerHalfEdgeAroundFaceConstCirculator (idx_inner_half_edge, this));
      }

      //////////////////////////////////////////////////////////////////////////
      // OuterHalfEdgeAroundFaceCirculator
      //////////////////////////////////////////////////////////////////////////

      OuterHalfEdgeAroundFaceCirculator
      getOuterHalfEdgeAroundFaceCirculator (const Face& face)
      {
        assert (this->validateMeshElement (face));
        return (OuterHalfEdgeAroundFaceCirculator (face, this));
      }

      OuterHalfEdgeAroundFaceConstCirculator
      getOuterHalfEdgeAroundFaceConstCirculator (const Face& face) const
      {
        assert (this->validateMeshElement (face));
        return (OuterHalfEdgeAroundFaceConstCirculator (face, this));
      }

      OuterHalfEdgeAroundFaceCirculator
      getOuterHalfEdgeAroundFaceCirculator (const FaceIndex& idx_face)
      {
        assert (this->validateMeshElementIndex (idx_face));
        assert (this->validateMeshElement (this->getElement (idx_face)));
        return (OuterHalfEdgeAroundFaceCirculator (idx_face, this));
      }

      OuterHalfEdgeAroundFaceConstCirculator
      getOuterHalfEdgeAroundFaceConstCirculator (const FaceIndex& idx_face) const
      {
        assert (this->validateMeshElementIndex (idx_face));
        assert (this->validateMeshElement (this->getElement (idx_face)));
        return (OuterHalfEdgeAroundFaceConstCirculator (idx_face, this));
      }

      OuterHalfEdgeAroundFaceCirculator
      getOuterHalfEdgeAroundFaceCirculator (const HalfEdgeIndex& idx_inner_half_edge)
      {
        assert (this->validateMeshElementIndex (idx_inner_half_edge));
        return (OuterHalfEdgeAroundFaceCirculator (idx_inner_half_edge, this));
      }

      OuterHalfEdgeAroundFaceConstCirculator
      getOuterHalfEdgeAroundFaceConstCirculator (const HalfEdgeIndex& idx_inner_half_edge) const
      {
        assert (this->validateMeshElementIndex (idx_inner_half_edge));
        return (OuterHalfEdgeAroundFaceConstCirculator (idx_inner_half_edge, this));
      }

      //////////////////////////////////////////////////////////////////////////
      // FaceAroundFaceCirculator
      //////////////////////////////////////////////////////////////////////////

      FaceAroundFaceCirculator
      getFaceAroundFaceCirculator (const Face& face)
      {
        assert (this->validateMeshElement (face));
        return (FaceAroundFaceCirculator (face, this));
      }

      FaceAroundFaceConstCirculator
      getFaceAroundFaceConstCirculator (const Face& face) const
      {
        assert (this->validateMeshElement (face));
        return (FaceAroundFaceConstCirculator (face, this));
      }

      FaceAroundFaceCirculator
      getFaceAroundFaceCirculator (const FaceIndex& idx_face)
      {
        assert (this->validateMeshElementIndex (idx_face));
        assert (this->validateMeshElement (this->getElement (idx_face)));
        return (FaceAroundFaceCirculator (idx_face, this));
      }

      FaceAroundFaceConstCirculator
      getFaceAroundFaceConstCirculator (const FaceIndex& idx_face) const
      {
        assert (this->validateMeshElementIndex (idx_face));
        assert (this->validateMeshElement (this->getElement (idx_face)));
        return (FaceAroundFaceConstCirculator (idx_face, this));
      }

      FaceAroundFaceCirculator
      getFaceAroundFaceCirculator (const HalfEdgeIndex& idx_inner_half_edge)
      {
        assert (this->validateMeshElementIndex (idx_inner_half_edge));
        return (FaceAroundFaceCirculator (idx_inner_half_edge, this));
      }

      FaceAroundFaceConstCirculator
      getFaceAroundFaceConstCirculator (const HalfEdgeIndex& idx_inner_half_edge) const
      {
        assert (this->validateMeshElementIndex (idx_inner_half_edge));
        return (FaceAroundFaceConstCirculator (idx_inner_half_edge, this));
      }

      //////////////////////////////////////////////////////////////////////////
      // HalfEdgeAroundBoundaryCirculator
      //////////////////////////////////////////////////////////////////////////

      HalfEdgeAroundBoundaryCirculator
      getHalfEdgeAroundBoundaryCirculator (const HalfEdgeIndex& idx_boundary_half_edge)
      {
        assert (this->validateMeshElementIndex (idx_boundary_half_edge));
        return (this->getInnerHalfEdgeAroundFaceCirculator (idx_boundary_half_edge));
      }

      HalfEdgeAroundBoundaryConstCirculator
      getHalfEdgeAroundBoundaryConstCirculator (const HalfEdgeIndex& idx_boundary_half_edge) const
      {
        assert (this->validateMeshElementIndex (idx_boundary_half_edge));
        return (this->getInnerHalfEdgeAroundFaceConstCirculator (idx_boundary_half_edge));
      }

    protected:

      //////////////////////////////////////////////////////////////////////////
      // Push back
      //////////////////////////////////////////////////////////////////////////

      VertexIndex
      pushBackVertex (const Vertex& vertex)
      {
        vertexes_.push_back (vertex);
        return (VertexIndex (static_cast<int> (this->sizeVertexes ()) - 1));
      }

      VertexIndex
      pushBackVertex (const VertexData&    vertex_data             = VertexData (),
                      const HalfEdgeIndex& idx_outgoing_half_edge_ = HalfEdgeIndex ())
      {
        vertexes_.push_back (Vertex (vertex_data, idx_outgoing_half_edge_));
        return (VertexIndex (static_cast<int> (this->sizeVertexes ()) - 1));
      }

      HalfEdgeIndex
      pushBackHalfEdge (const HalfEdge& half_edge)
      {
        half_edges_.push_back (half_edge);
        return (HalfEdgeIndex (static_cast<int> (this->sizeHalfEdges ()) - 1));
      }

      HalfEdgeIndex
      pushBackHalfEdge (const HalfEdgeData&  half_edge_data         = HalfEdgeData  (),
                        const VertexIndex&   idx_terminating_vertex = VertexIndex   (),
                        const HalfEdgeIndex& idx_opposite_half_edge = HalfEdgeIndex (),
                        const HalfEdgeIndex& idx_next_half_edge     = HalfEdgeIndex (),
                        const HalfEdgeIndex& idx_prev_half_edge     = HalfEdgeIndex (),
                        const FaceIndex&     idx_face               = FaceIndex     ())
      {
        half_edges_.push_back (HalfEdge (half_edge_data, idx_terminating_vertex, idx_opposite_half_edge, idx_next_half_edge, idx_prev_half_edge, idx_face));
        return (HalfEdgeIndex (static_cast<int> (this->sizeHalfEdges ()) - 1));
      }

      FaceIndex
      pushBackFace (const Face& face)
      {
        faces_.push_back (face);
        return (FaceIndex (static_cast<int> (this->sizeFaces ()) - 1));
      }

      FaceIndex
      pushBackFace (const FaceData&          face_data           = FaceData (),
                    const HalfEdgeIndex& idx_inner_half_edge = HalfEdgeIndex ())
      {
        faces_.push_back (Face (face_data, idx_inner_half_edge));
        return (FaceIndex (static_cast<int> (this->sizeFaces ()) - 1));
      }

      //////////////////////////////////////////////////////////////////////////
      // For the assertions
      //////////////////////////////////////////////////////////////////////////

      bool
      validateMeshElement (const Vertex& vertex) const
      {
        return (this->validateMeshElementIndex (vertex.getOutgoingHalfEdgeIndex ()));
      }

      bool
      validateMeshElementIndex (const VertexIndex& idx_vertex) const
      {
        if (!idx_vertex.isValid ())              return (false);
        if (idx_vertex >= this->sizeVertexes ()) return (false);
        return (true);
      }

      bool
      validateMeshElementIndex (const HalfEdgeIndex& idx_half_edge) const
      {
        if (!idx_half_edge.isValid ())               return (false);
        if (idx_half_edge >= this->sizeHalfEdges ()) return (false);
        return (true);
      }

      bool
      validateMeshElement (const Face& face) const
      {
        return (this->validateMeshElementIndex (face.getInnerHalfEdgeIndex ()));
      }

      bool
      validateMeshElementIndex (const FaceIndex& idx_face) const
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

#endif // PCL_GEOMETRY_MESH_BASE_HPP

