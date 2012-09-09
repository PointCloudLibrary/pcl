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

#ifndef PCL_GEOMETRY_MESH_CIRCULATORS_HPP
#define PCL_GEOMETRY_MESH_CIRCULATORS_HPP

#include <boost/iterator/iterator_facade.hpp>
#include <boost/utility/enable_if.hpp>
#include <boost/type_traits/is_const.hpp>
#include <boost/type_traits/is_convertible.hpp>
#include <boost/type_traits/conditional.hpp>
#include <boost/type_traits/add_const.hpp>

////////////////////////////////////////////////////////////////////////////////
// VertexAroundVertexCirculator
////////////////////////////////////////////////////////////////////////////////

namespace pcl
{

  template <class MeshT>
  class VertexAroundVertexCirculator
      : public boost::iterator_facade <pcl::VertexAroundVertexCirculator <MeshT>, typename boost::conditional <boost::is_const <MeshT>::value, typename boost::add_const <typename MeshT::Vertex>::type, typename MeshT::Vertex>::type, boost::bidirectional_traversal_tag>
  {
    public:

      typedef boost::iterator_facade <pcl::VertexAroundVertexCirculator <MeshT>, typename boost::conditional <boost::is_const <MeshT>::value, typename boost::add_const <typename MeshT::Vertex>::type, typename MeshT::Vertex>::type, boost::bidirectional_traversal_tag> Base;

      typedef typename Base::value_type        value_type;
      typedef typename Base::reference         reference;
      typedef typename Base::pointer           pointer;
      typedef typename Base::difference_type   difference_type;
      typedef typename Base::iterator_category iterator_category;

      typedef MeshT                        Mesh;

      typedef typename Mesh::Vertex        Vertex;
      typedef typename Mesh::HalfEdge      HalfEdge;
      typedef typename Mesh::Face          Face;

      typedef typename Mesh::VertexIndex   VertexIndex;
      typedef typename Mesh::HalfEdgeIndex HalfEdgeIndex;
      typedef typename Mesh::FaceIndex     FaceIndex;

    private:

      struct enabler {};

    public:

      VertexAroundVertexCirculator (const Vertex& vertex,
                                    Mesh*const    mesh)
        : mesh_                   (mesh),
          idx_outgoing_half_edge_ (vertex.getOutgoingHalfEdgeIndex ())
      {
      }

      VertexAroundVertexCirculator (const VertexIndex& idx_vertex,
                                    Mesh*const         mesh)
        : mesh_                   (mesh),
          idx_outgoing_half_edge_ (mesh->getElement (idx_vertex).getOutgoingHalfEdgeIndex ())
      {
      }

      VertexAroundVertexCirculator (const HalfEdgeIndex& idx_outgoing_half_edge,
                                    Mesh*const           mesh)
        : mesh_                   (mesh),
          idx_outgoing_half_edge_ (idx_outgoing_half_edge)
      {
      }

      template <class OtherMeshT>
      VertexAroundVertexCirculator (const pcl::VertexAroundVertexCirculator <OtherMeshT>& other,
                                    typename boost::enable_if <boost::is_convertible <OtherMeshT*, Mesh*>, enabler>::type = enabler ())
        : mesh_                   (other.mesh_),
          idx_outgoing_half_edge_ (other.idx_outgoing_half_edge_)
      {
      }

    public:

      const HalfEdgeIndex&
      getCurrentHalfEdgeIndex () const
      {
        return (idx_outgoing_half_edge_);
      }

      const VertexIndex&
      getDereferencedIndex () const
      {
        return (mesh_->getElement (idx_outgoing_half_edge_).getTerminatingVertexIndex ());
      }

      bool
      isValid () const
      {
        return (this->getDereferencedIndex ().isValid ());
      }

    private:

      friend class boost::iterator_core_access;
      template <class> friend class pcl::VertexAroundVertexCirculator;

      template <class OtherMeshT> bool
      equal (const pcl::VertexAroundVertexCirculator <OtherMeshT>& other) const
      {
        return (idx_outgoing_half_edge_ == other.idx_outgoing_half_edge_);
      }

      void
      increment ()
      {
        idx_outgoing_half_edge_ = mesh_->getElement (idx_outgoing_half_edge_).getPrevHalfEdge (*mesh_).getOppositeHalfEdgeIndex ();
      }

      void
      decrement ()
      {
        idx_outgoing_half_edge_ = mesh_->getElement (idx_outgoing_half_edge_).getOppositeHalfEdge (*mesh_).getNextHalfEdgeIndex ();
      }

      reference
      dereference () const
      {
        return (mesh_->getElement (idx_outgoing_half_edge_).getTerminatingVertex (*mesh_));
      }

    private:

      Mesh*         mesh_;
      HalfEdgeIndex idx_outgoing_half_edge_;
  };

} // End namespace pcl

////////////////////////////////////////////////////////////////////////////////
// OutgoingHalfEdgeAroundVertexCirculator
////////////////////////////////////////////////////////////////////////////////

namespace pcl
{

  template <class MeshT>
  class OutgoingHalfEdgeAroundVertexCirculator
      : public boost::iterator_facade <pcl::OutgoingHalfEdgeAroundVertexCirculator <MeshT>, typename boost::conditional <boost::is_const <MeshT>::value, typename boost::add_const <typename MeshT::HalfEdge>::type, typename MeshT::HalfEdge>::type, boost::bidirectional_traversal_tag>
  {
    public:

      typedef boost::iterator_facade <pcl::OutgoingHalfEdgeAroundVertexCirculator <MeshT>, typename boost::conditional <boost::is_const <MeshT>::value, typename boost::add_const <typename MeshT::HalfEdge>::type, typename MeshT::HalfEdge>::type, boost::bidirectional_traversal_tag> Base;

      typedef typename Base::value_type        value_type;
      typedef typename Base::reference         reference;
      typedef typename Base::pointer           pointer;
      typedef typename Base::difference_type   difference_type;
      typedef typename Base::iterator_category iterator_category;

      typedef MeshT                        Mesh;

      typedef typename Mesh::Vertex        Vertex;
      typedef typename Mesh::HalfEdge      HalfEdge;
      typedef typename Mesh::Face          Face;

      typedef typename Mesh::VertexIndex   VertexIndex;
      typedef typename Mesh::HalfEdgeIndex HalfEdgeIndex;
      typedef typename Mesh::FaceIndex     FaceIndex;

    private:

      struct enabler {};

    public:

      OutgoingHalfEdgeAroundVertexCirculator (const Vertex& vertex,
                                              Mesh*const    mesh)
        : mesh_                   (mesh),
          idx_outgoing_half_edge_ (vertex.getOutgoingHalfEdgeIndex ())
      {
      }

      OutgoingHalfEdgeAroundVertexCirculator (const VertexIndex& idx_vertex,
                                              Mesh*const         mesh)
        : mesh_                   (mesh),
          idx_outgoing_half_edge_ (mesh->getElement (idx_vertex).getOutgoingHalfEdgeIndex ())
      {
      }

      OutgoingHalfEdgeAroundVertexCirculator (const HalfEdgeIndex& idx_outgoing_half_edge,
                                              Mesh*const           mesh)
        : mesh_                   (mesh),
          idx_outgoing_half_edge_ (idx_outgoing_half_edge)
      {
      }

      template <class OtherMeshT>
      OutgoingHalfEdgeAroundVertexCirculator (const pcl::OutgoingHalfEdgeAroundVertexCirculator <OtherMeshT>& other,
                                              typename boost::enable_if <boost::is_convertible <OtherMeshT*, Mesh*>, enabler>::type = enabler ())
        : mesh_                   (other.mesh_),
          idx_outgoing_half_edge_ (other.idx_outgoing_half_edge_)
      {
      }

    public:

      const HalfEdgeIndex&
      getCurrentHalfEdgeIndex () const
      {
        return (idx_outgoing_half_edge_);
      }

      const HalfEdgeIndex&
      getDereferencedIndex () const
      {
        return (this->getCurrentHalfEdgeIndex ());
      }

      bool
      isValid () const
      {
        return (this->getDereferencedIndex ().isValid ());
      }

    private:

      friend class boost::iterator_core_access;
      template <class> friend class pcl::OutgoingHalfEdgeAroundVertexCirculator;

      template <class OtherMeshT> bool
      equal (const pcl::OutgoingHalfEdgeAroundVertexCirculator <OtherMeshT>& other) const
      {
        return (idx_outgoing_half_edge_ == other.idx_outgoing_half_edge_);
      }

      void
      increment ()
      {
        idx_outgoing_half_edge_ = mesh_->getElement (idx_outgoing_half_edge_).getPrevHalfEdge (*mesh_).getOppositeHalfEdgeIndex ();
      }

      void
      decrement ()
      {
        idx_outgoing_half_edge_ = mesh_->getElement (idx_outgoing_half_edge_).getOppositeHalfEdge (*mesh_).getNextHalfEdgeIndex ();
      }

      reference
      dereference () const
      {
        return (mesh_->getElement (idx_outgoing_half_edge_));
      }

    private:

      Mesh*         mesh_;
      HalfEdgeIndex idx_outgoing_half_edge_;
  };

} // End namespace pcl

////////////////////////////////////////////////////////////////////////////////
// IncomingHalfEdgeAroundVertexCirculator
////////////////////////////////////////////////////////////////////////////////

namespace pcl
{

  template <class MeshT>
  class IncomingHalfEdgeAroundVertexCirculator
      : public boost::iterator_facade <pcl::IncomingHalfEdgeAroundVertexCirculator <MeshT>, typename boost::conditional <boost::is_const <MeshT>::value, typename boost::add_const <typename MeshT::HalfEdge>::type, typename MeshT::HalfEdge>::type, boost::bidirectional_traversal_tag>
  {
    public:

      typedef boost::iterator_facade <pcl::IncomingHalfEdgeAroundVertexCirculator <MeshT>, typename boost::conditional <boost::is_const <MeshT>::value, typename boost::add_const <typename MeshT::HalfEdge>::type, typename MeshT::HalfEdge>::type, boost::bidirectional_traversal_tag> Base;

      typedef typename Base::value_type        value_type;
      typedef typename Base::reference         reference;
      typedef typename Base::pointer           pointer;
      typedef typename Base::difference_type   difference_type;
      typedef typename Base::iterator_category iterator_category;

      typedef MeshT                        Mesh;

      typedef typename Mesh::Vertex        Vertex;
      typedef typename Mesh::HalfEdge      HalfEdge;
      typedef typename Mesh::Face          Face;

      typedef typename Mesh::VertexIndex   VertexIndex;
      typedef typename Mesh::HalfEdgeIndex HalfEdgeIndex;
      typedef typename Mesh::FaceIndex     FaceIndex;

    private:

      struct enabler {};

    public:

      IncomingHalfEdgeAroundVertexCirculator (const Vertex& vertex,
                                              Mesh*const    mesh)
        : mesh_                  (mesh),
          idx_incoming_half_edge (vertex.getOutgoingHalfEdge (*mesh_).getOppositeHalfEdgeIndex ())
      {
      }

      IncomingHalfEdgeAroundVertexCirculator (const VertexIndex& idx_vertex,
                                              Mesh*const         mesh)
        : mesh_                  (mesh),
          idx_incoming_half_edge (mesh->getElement (idx_vertex).getIncomingHalfEdgeIndex (*mesh_))
      {
      }

      IncomingHalfEdgeAroundVertexCirculator (const HalfEdgeIndex& idx_incoming_half_edge,
                                              Mesh*const           mesh)
        : mesh_                  (mesh),
          idx_incoming_half_edge (idx_incoming_half_edge)
      {
      }

      template <class OtherMeshT>
      IncomingHalfEdgeAroundVertexCirculator (const pcl::IncomingHalfEdgeAroundVertexCirculator <OtherMeshT>& other,
                                              typename boost::enable_if <boost::is_convertible <OtherMeshT*, Mesh*>, enabler>::type = enabler ())
        : mesh_                  (other.mesh_),
          idx_incoming_half_edge (other.idx_incoming_half_edge)
      {
      }

    public:

      const HalfEdgeIndex&
      getCurrentHalfEdgeIndex () const
      {
        return (idx_incoming_half_edge);
      }

      const HalfEdgeIndex&
      getDereferencedIndex () const
      {
        return (this->getCurrentHalfEdgeIndex ());
      }

      bool
      isValid () const
      {
        return (this->getDereferencedIndex ().isValid ());
      }

    private:

      friend class boost::iterator_core_access;
      template <class> friend class pcl::IncomingHalfEdgeAroundVertexCirculator;

      template <class OtherMeshT> bool
      equal (const pcl::IncomingHalfEdgeAroundVertexCirculator <OtherMeshT>& other) const
      {
        return (idx_incoming_half_edge == other.idx_incoming_half_edge);
      }

      void
      increment ()
      {
        idx_incoming_half_edge = mesh_->getElement (idx_incoming_half_edge).getOppositeHalfEdge (*mesh_).getPrevHalfEdgeIndex ();
      }

      void
      decrement ()
      {
        idx_incoming_half_edge = mesh_->getElement (idx_incoming_half_edge).getNextHalfEdge (*mesh_).getOppositeHalfEdgeIndex ();
      }

      reference
      dereference () const
      {
        return (mesh_->getElement (idx_incoming_half_edge));
      }

    private:

      Mesh*         mesh_;
      HalfEdgeIndex idx_incoming_half_edge;
  };

} // End namespace pcl

////////////////////////////////////////////////////////////////////////////////
// FaceAroundVertexCirculator
////////////////////////////////////////////////////////////////////////////////

namespace pcl
{

  template <class MeshT>
  class FaceAroundVertexCirculator
      : public boost::iterator_facade <pcl::FaceAroundVertexCirculator <MeshT>, typename boost::conditional <boost::is_const <MeshT>::value, typename boost::add_const <typename MeshT::Face>::type, typename MeshT::Face>::type, boost::bidirectional_traversal_tag>
  {
    public:

      typedef boost::iterator_facade <pcl::FaceAroundVertexCirculator <MeshT>, typename boost::conditional <boost::is_const <MeshT>::value, typename boost::add_const <typename MeshT::Face>::type, typename MeshT::Face>::type, boost::bidirectional_traversal_tag> Base;

      typedef typename Base::value_type        value_type;
      typedef typename Base::reference         reference;
      typedef typename Base::pointer           pointer;
      typedef typename Base::difference_type   difference_type;
      typedef typename Base::iterator_category iterator_category;

      typedef MeshT                        Mesh;

      typedef typename Mesh::Vertex        Vertex;
      typedef typename Mesh::HalfEdge      HalfEdge;
      typedef typename Mesh::Face          Face;

      typedef typename Mesh::VertexIndex   VertexIndex;
      typedef typename Mesh::HalfEdgeIndex HalfEdgeIndex;
      typedef typename Mesh::FaceIndex     FaceIndex;

    private:

      struct enabler {};

    public:

      FaceAroundVertexCirculator (const Vertex& vertex,
                                  Mesh*const    mesh)
        : mesh_                   (mesh),
          idx_outgoing_half_edge_ (vertex.getOutgoingHalfEdgeIndex ())
      {
      }

      FaceAroundVertexCirculator (const VertexIndex& idx_vertex,
                                  Mesh*const         mesh)
        : mesh_                   (mesh),
          idx_outgoing_half_edge_ (mesh->getElement (idx_vertex).getOutgoingHalfEdgeIndex ())
      {
      }

      FaceAroundVertexCirculator (const HalfEdgeIndex& idx_outgoing_half_edge,
                                  Mesh*const           mesh)
        : mesh_                   (mesh),
          idx_outgoing_half_edge_ (idx_outgoing_half_edge)
      {
      }

      template <class OtherMeshT>
      FaceAroundVertexCirculator (const pcl::FaceAroundVertexCirculator <OtherMeshT>& other,
                                  typename boost::enable_if <boost::is_convertible <OtherMeshT*, Mesh*>, enabler>::type = enabler ())
        : mesh_                   (other.mesh_),
          idx_outgoing_half_edge_ (other.idx_outgoing_half_edge_)
      {
      }

    public:

      const HalfEdgeIndex&
      getCurrentHalfEdgeIndex () const
      {
        return (idx_outgoing_half_edge_);
      }

      const FaceIndex&
      getDereferencedIndex () const
      {
        return (mesh_->getElement (idx_outgoing_half_edge_).getFaceIndex ());
      }

      bool
      isValid () const
      {
        return (this->getDereferencedIndex ().isValid ());
      }

    private:

      friend class boost::iterator_core_access;
      template <class> friend class pcl::FaceAroundVertexCirculator;

      template <class OtherMeshT> bool
      equal (const pcl::FaceAroundVertexCirculator <OtherMeshT>& other) const
      {
        return (idx_outgoing_half_edge_ == other.idx_outgoing_half_edge_);
      }

      void
      increment ()
      {
        idx_outgoing_half_edge_ = mesh_->getElement (idx_outgoing_half_edge_).getPrevHalfEdge (*mesh_).getOppositeHalfEdgeIndex ();
      }

      void
      decrement ()
      {
        idx_outgoing_half_edge_ = mesh_->getElement (idx_outgoing_half_edge_).getOppositeHalfEdge (*mesh_).getNextHalfEdgeIndex ();
      }

      reference
      dereference () const
      {
        return (mesh_->getElement (idx_outgoing_half_edge_).getFace (*mesh_));
      }

    private:

      Mesh*         mesh_;
      HalfEdgeIndex idx_outgoing_half_edge_;
  };

} // End namespace pcl

////////////////////////////////////////////////////////////////////////////////
// VertexAroundFaceCirculator
////////////////////////////////////////////////////////////////////////////////

namespace pcl
{

  template <class MeshT>
  class VertexAroundFaceCirculator
      : public boost::iterator_facade <pcl::VertexAroundFaceCirculator <MeshT>, typename boost::conditional <boost::is_const <MeshT>::value, typename boost::add_const <typename MeshT::Vertex>::type, typename MeshT::Vertex>::type, boost::bidirectional_traversal_tag>
  {
    public:

      typedef boost::iterator_facade <pcl::VertexAroundFaceCirculator <MeshT>, typename boost::conditional <boost::is_const <MeshT>::value, typename boost::add_const <typename MeshT::Vertex>::type, typename MeshT::Vertex>::type, boost::bidirectional_traversal_tag> Base;

      typedef typename Base::value_type        value_type;
      typedef typename Base::reference         reference;
      typedef typename Base::pointer           pointer;
      typedef typename Base::difference_type   difference_type;
      typedef typename Base::iterator_category iterator_category;

      typedef MeshT                        Mesh;

      typedef typename Mesh::Vertex        Vertex;
      typedef typename Mesh::HalfEdge      HalfEdge;
      typedef typename Mesh::Face          Face;

      typedef typename Mesh::VertexIndex   VertexIndex;
      typedef typename Mesh::HalfEdgeIndex HalfEdgeIndex;
      typedef typename Mesh::FaceIndex     FaceIndex;

    private:

      struct enabler {};

    public:

      VertexAroundFaceCirculator (const Face& face,
                                  Mesh*const  mesh)
        : mesh_                (mesh),
          idx_inner_half_edge_ (face.getInnerHalfEdgeIndex ())
      {
      }

      VertexAroundFaceCirculator (const FaceIndex& idx_face,
                                  Mesh*const       mesh)
        : mesh_                (mesh),
          idx_inner_half_edge_ (mesh->getElement (idx_face).getInnerHalfEdgeIndex ())
      {
      }

      VertexAroundFaceCirculator (const HalfEdgeIndex& idx_inner_half_edge,
                                  Mesh*const           mesh)
        : mesh_                (mesh),
          idx_inner_half_edge_ (idx_inner_half_edge)
      {
      }

      template <class OtherMeshT>
      VertexAroundFaceCirculator (const pcl::VertexAroundFaceCirculator <OtherMeshT>& other,
                                  typename boost::enable_if <boost::is_convertible <OtherMeshT*, Mesh*>, enabler>::type = enabler ())
        : mesh_                (other.mesh_),
          idx_inner_half_edge_ (other.idx_inner_half_edge_)
      {
      }

    public:

      const HalfEdgeIndex&
      getCurrentHalfEdgeIndex () const
      {
        return (idx_inner_half_edge_);
      }

      const VertexIndex&
      getDereferencedIndex () const
      {
        return (mesh_->getElement (idx_inner_half_edge_).getTerminatingVertexIndex ());
      }

      bool
      isValid () const
      {
        return (this->getDereferencedIndex ().isValid ());
      }

    private:

      friend class boost::iterator_core_access;
      template <class> friend class pcl::VertexAroundFaceCirculator;

      template <class OtherMeshT> bool
      equal (const pcl::VertexAroundFaceCirculator <OtherMeshT>& other) const
      {
        return (idx_inner_half_edge_ == other.idx_inner_half_edge_);
      }

      void
      increment ()
      {
        idx_inner_half_edge_ = mesh_->getElement (idx_inner_half_edge_).getNextHalfEdgeIndex ();
      }

      void
      decrement ()
      {
        idx_inner_half_edge_ = mesh_->getElement (idx_inner_half_edge_).getPrevHalfEdgeIndex ();
      }

      reference
      dereference () const
      {
        return (mesh_->getElement (idx_inner_half_edge_).getTerminatingVertex (*mesh_));
      }

    private:

      Mesh*         mesh_;
      HalfEdgeIndex idx_inner_half_edge_;
  };

} // End namespace pcl

////////////////////////////////////////////////////////////////////////////////
// InnerHalfEdgeAroundFaceCirculator
////////////////////////////////////////////////////////////////////////////////

namespace pcl
{

  template <class MeshT>
  class InnerHalfEdgeAroundFaceCirculator
      : public boost::iterator_facade <pcl::InnerHalfEdgeAroundFaceCirculator <MeshT>, typename boost::conditional <boost::is_const <MeshT>::value, typename boost::add_const <typename MeshT::HalfEdge>::type, typename MeshT::HalfEdge>::type, boost::bidirectional_traversal_tag>
  {
    public:

      typedef boost::iterator_facade <pcl::InnerHalfEdgeAroundFaceCirculator <MeshT>, typename boost::conditional <boost::is_const <MeshT>::value, typename boost::add_const <typename MeshT::HalfEdge>::type, typename MeshT::HalfEdge>::type, boost::bidirectional_traversal_tag> Base;

      typedef typename Base::value_type        value_type;
      typedef typename Base::reference         reference;
      typedef typename Base::pointer           pointer;
      typedef typename Base::difference_type   difference_type;
      typedef typename Base::iterator_category iterator_category;

      typedef MeshT                        Mesh;

      typedef typename Mesh::Vertex        Vertex;
      typedef typename Mesh::HalfEdge      HalfEdge;
      typedef typename Mesh::Face          Face;

      typedef typename Mesh::VertexIndex   VertexIndex;
      typedef typename Mesh::HalfEdgeIndex HalfEdgeIndex;
      typedef typename Mesh::FaceIndex     FaceIndex;

    private:

      struct enabler {};

    public:

      InnerHalfEdgeAroundFaceCirculator (const Face& face,
                                         Mesh*const  mesh)
        : mesh_                (mesh),
          idx_inner_half_edge_ (face.getInnerHalfEdgeIndex ())
      {
      }

      InnerHalfEdgeAroundFaceCirculator (const FaceIndex& idx_face,
                                         Mesh*const       mesh)
        : mesh_                (mesh),
          idx_inner_half_edge_ (mesh->getElement (idx_face).getInnerHalfEdgeIndex ())
      {
      }

      InnerHalfEdgeAroundFaceCirculator (const HalfEdgeIndex& idx_inner_half_edge,
                                         Mesh*const           mesh)
        : mesh_                (mesh),
          idx_inner_half_edge_ (idx_inner_half_edge)
      {
      }

      template <class OtherMeshT>
      InnerHalfEdgeAroundFaceCirculator (const pcl::InnerHalfEdgeAroundFaceCirculator <OtherMeshT>& other,
                                         typename boost::enable_if <boost::is_convertible <OtherMeshT*, Mesh*>, enabler>::type = enabler ())
        : mesh_                (other.mesh_),
          idx_inner_half_edge_ (other.idx_inner_half_edge_)
      {
      }

    public:

      const HalfEdgeIndex&
      getCurrentHalfEdgeIndex () const
      {
        return (idx_inner_half_edge_);
      }

      const HalfEdgeIndex&
      getDereferencedIndex () const
      {
        return (this->getCurrentHalfEdgeIndex ());
      }

      bool
      isValid () const
      {
        return (this->getDereferencedIndex ().isValid ());
      }

    private:

      friend class boost::iterator_core_access;
      template <class> friend class pcl::InnerHalfEdgeAroundFaceCirculator;

      template <class OtherMeshT> bool
      equal (const pcl::InnerHalfEdgeAroundFaceCirculator <OtherMeshT>& other) const
      {
        return (idx_inner_half_edge_ == other.idx_inner_half_edge_);
      }

      void
      increment ()
      {
        idx_inner_half_edge_ = mesh_->getElement (idx_inner_half_edge_).getNextHalfEdgeIndex ();
      }

      void
      decrement ()
      {
        idx_inner_half_edge_ = mesh_->getElement (idx_inner_half_edge_).getPrevHalfEdgeIndex ();
      }

      reference
      dereference () const
      {
        return (mesh_->getElement (idx_inner_half_edge_));
      }

    private:

      Mesh*         mesh_;
      HalfEdgeIndex idx_inner_half_edge_;
  };

} // End namespace pcl

////////////////////////////////////////////////////////////////////////////////
// OuterHalfEdgeAroundFaceCirculator
////////////////////////////////////////////////////////////////////////////////

namespace pcl
{

  template <class MeshT>
  class OuterHalfEdgeAroundFaceCirculator
      : public boost::iterator_facade <pcl::OuterHalfEdgeAroundFaceCirculator <MeshT>, typename boost::conditional <boost::is_const <MeshT>::value, typename boost::add_const <typename MeshT::HalfEdge>::type, typename MeshT::HalfEdge>::type, boost::bidirectional_traversal_tag>
  {
    public:

      typedef boost::iterator_facade <pcl::OuterHalfEdgeAroundFaceCirculator <MeshT>, typename boost::conditional <boost::is_const <MeshT>::value, typename boost::add_const <typename MeshT::HalfEdge>::type, typename MeshT::HalfEdge>::type, boost::bidirectional_traversal_tag> Base;

      typedef typename Base::value_type        value_type;
      typedef typename Base::reference         reference;
      typedef typename Base::pointer           pointer;
      typedef typename Base::difference_type   difference_type;
      typedef typename Base::iterator_category iterator_category;

      typedef MeshT                        Mesh;

      typedef typename Mesh::Vertex        Vertex;
      typedef typename Mesh::HalfEdge      HalfEdge;
      typedef typename Mesh::Face          Face;

      typedef typename Mesh::VertexIndex   VertexIndex;
      typedef typename Mesh::HalfEdgeIndex HalfEdgeIndex;
      typedef typename Mesh::FaceIndex     FaceIndex;

    private:

      struct enabler {};

    public:

      OuterHalfEdgeAroundFaceCirculator (const Face& face,
                                         Mesh*const  mesh)
        : mesh_                (mesh),
          idx_inner_half_edge_ (face.getInnerHalfEdgeIndex ())
      {
      }

      OuterHalfEdgeAroundFaceCirculator (const FaceIndex& idx_face,
                                         Mesh*const       mesh)
        : mesh_                (mesh),
          idx_inner_half_edge_ (mesh->getElement (idx_face).getInnerHalfEdgeIndex ())
      {
      }

      OuterHalfEdgeAroundFaceCirculator (const HalfEdgeIndex& idx_inner_half_edge,
                                         Mesh*const           mesh)
        : mesh_                (mesh),
          idx_inner_half_edge_ (idx_inner_half_edge)
      {
      }

      template <class OtherMeshT>
      OuterHalfEdgeAroundFaceCirculator (const pcl::OuterHalfEdgeAroundFaceCirculator <OtherMeshT>& other,
                                         typename boost::enable_if <boost::is_convertible <OtherMeshT*, Mesh*>, enabler>::type = enabler ())
        : mesh_                (other.mesh_),
          idx_inner_half_edge_ (other.idx_inner_half_edge_)
      {
      }

    public:

      const HalfEdgeIndex&
      getCurrentHalfEdgeIndex () const
      {
        return (idx_inner_half_edge_);
      }

      const HalfEdgeIndex&
      getDereferencedIndex () const
      {
        return (mesh_->getElement (idx_inner_half_edge_).getOppositeHalfEdgeIndex ());
      }

      bool
      isValid () const
      {
        return (this->getDereferencedIndex ().isValid ());
      }

    private:

      friend class boost::iterator_core_access;
      template <class> friend class pcl::OuterHalfEdgeAroundFaceCirculator;

      template <class OtherMeshT> bool
      equal (const pcl::OuterHalfEdgeAroundFaceCirculator <OtherMeshT>& other) const
      {
        return (idx_inner_half_edge_ == other.idx_inner_half_edge_);
      }

      void
      increment ()
      {
        idx_inner_half_edge_ = mesh_->getElement (idx_inner_half_edge_).getNextHalfEdgeIndex ();
      }

      void
      decrement ()
      {
        idx_inner_half_edge_ = mesh_->getElement (idx_inner_half_edge_).getPrevHalfEdgeIndex ();
      }

      reference
      dereference () const
      {
        return (mesh_->getElement (idx_inner_half_edge_).getOppositeHalfEdge (*mesh_));
      }

    private:

      Mesh*         mesh_;
      HalfEdgeIndex idx_inner_half_edge_;
  };

} // End namespace pcl

////////////////////////////////////////////////////////////////////////////////
// FaceAroundFaceCirculator
////////////////////////////////////////////////////////////////////////////////

namespace pcl
{

  template <class MeshT>
  class FaceAroundFaceCirculator
      : public boost::iterator_facade <pcl::FaceAroundFaceCirculator <MeshT>, typename boost::conditional <boost::is_const <MeshT>::value, typename boost::add_const <typename MeshT::Face>::type, typename MeshT::Face>::type, boost::bidirectional_traversal_tag>
  {
    public:

      typedef boost::iterator_facade <pcl::FaceAroundFaceCirculator <MeshT>, typename boost::conditional <boost::is_const <MeshT>::value, typename boost::add_const <typename MeshT::Face>::type, typename MeshT::Face>::type, boost::bidirectional_traversal_tag> Base;

      typedef typename Base::value_type        value_type;
      typedef typename Base::reference         reference;
      typedef typename Base::pointer           pointer;
      typedef typename Base::difference_type   difference_type;
      typedef typename Base::iterator_category iterator_category;

      typedef MeshT                        Mesh;

      typedef typename Mesh::Vertex        Vertex;
      typedef typename Mesh::HalfEdge      HalfEdge;
      typedef typename Mesh::Face          Face;

      typedef typename Mesh::VertexIndex   VertexIndex;
      typedef typename Mesh::HalfEdgeIndex HalfEdgeIndex;
      typedef typename Mesh::FaceIndex     FaceIndex;

    private:

      struct enabler {};

    public:

      FaceAroundFaceCirculator (const Face& face,
                                Mesh*const  mesh)
        : mesh_                (mesh),
          idx_inner_half_edge_ (face.getInnerHalfEdgeIndex ())
      {
      }

      FaceAroundFaceCirculator (const FaceIndex& idx_face,
                                Mesh*const       mesh)
        : mesh_                (mesh),
          idx_inner_half_edge_ (mesh->getElement (idx_face).getInnerHalfEdgeIndex ())
      {
      }

      FaceAroundFaceCirculator (const HalfEdgeIndex& idx_inner_half_edge,
                                Mesh*const           mesh)
        : mesh_                (mesh),
          idx_inner_half_edge_ (idx_inner_half_edge)
      {
      }

      template <class OtherMeshT>
      FaceAroundFaceCirculator (const pcl::FaceAroundFaceCirculator <OtherMeshT>& other,
                                typename boost::enable_if <boost::is_convertible <OtherMeshT*, Mesh*>, enabler>::type = enabler ())
        : mesh_                (other.mesh_),
          idx_inner_half_edge_ (other.idx_inner_half_edge_)
      {
      }

    public:

      const HalfEdgeIndex&
      getCurrentHalfEdgeIndex () const
      {
        return (idx_inner_half_edge_);
      }

      const FaceIndex&
      getDereferencedIndex () const
      {
        return (mesh_->getElement (idx_inner_half_edge_).getOppositeFaceIndex (*mesh_));
      }

      bool
      isValid () const
      {
        return (this->getDereferencedIndex ().isValid ());
      }

    private:

      friend class boost::iterator_core_access;
      template <class> friend class pcl::FaceAroundFaceCirculator;

      template <class OtherMeshT> bool
      equal (const pcl::FaceAroundFaceCirculator <OtherMeshT>& other) const
      {
        return (idx_inner_half_edge_ == other.idx_inner_half_edge_);
      }

      void
      increment ()
      {
        idx_inner_half_edge_ = mesh_->getElement (idx_inner_half_edge_).getNextHalfEdgeIndex ();
      }

      void
      decrement ()
      {
        idx_inner_half_edge_ = mesh_->getElement (idx_inner_half_edge_).getPrevHalfEdgeIndex ();
      }

      reference
      dereference () const
      {
        return (mesh_->getElement (idx_inner_half_edge_).getOppositeFace (*mesh_));
      }

    private:

      Mesh*         mesh_;
      HalfEdgeIndex idx_inner_half_edge_;
  };

} // End namespace pcl

#endif // PCL_GEOMETRY_MESH_CIRCULATORS_HPP

