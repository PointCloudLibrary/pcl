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

// NOTE: This file has been created with 'pcl_src/geometry/include/pcl/geometry/mesh_circulators.py'

#ifndef PCL_GEOMETRY_MESH_CIRCULATORS_H
#define PCL_GEOMETRY_MESH_CIRCULATORS_H

#include <pcl/geometry/boost.h>
#include <pcl/geometry/mesh_indices.h>

////////////////////////////////////////////////////////////////////////////////
// VertexAroundVertexCirculator
////////////////////////////////////////////////////////////////////////////////

namespace pcl
{
  namespace geometry
  {
    /** \brief Circulates counter-clockwise around a vertex and returns an index to the terminating vertex of the outgoing half-edge (the target). The best way to declare the circulator is to use the method pcl::geometry::MeshBase::getVertexAroundVertexCirculator ().
      * \tparam MeshT Mesh to which this circulator belongs to.
      * \note The circulator can't be used to change the connectivity in the mesh (only const circulators are valid).
      * \author Martin Saelzle
      * \ingroup geometry
      */
    template <class MeshT>
    class VertexAroundVertexCirculator
        : boost::equality_comparable <pcl::geometry::VertexAroundVertexCirculator <MeshT>
        , boost::unit_steppable      <pcl::geometry::VertexAroundVertexCirculator <MeshT>
        > >
    {
      public:

        typedef boost::equality_comparable <pcl::geometry::VertexAroundVertexCirculator <MeshT>
              , boost::unit_steppable      <pcl::geometry::VertexAroundVertexCirculator <MeshT> > > Base;
        typedef pcl::geometry::VertexAroundVertexCirculator <MeshT> Self;

        typedef MeshT Mesh;
        typedef typename Mesh::VertexIndex VertexIndex;
        typedef typename Mesh::HalfEdgeIndex HalfEdgeIndex;

        /** \brief Constructor resulting in an invalid circulator. */
        VertexAroundVertexCirculator ()
          : mesh_                   (NULL),
            idx_outgoing_half_edge_ ()
        {
        }

        /** \brief Construct from the vertex around which we want to circulate. */
        VertexAroundVertexCirculator (const VertexIndex& idx_vertex,
                                      Mesh*const         mesh)
          : mesh_                   (mesh),
            idx_outgoing_half_edge_ (mesh->getOutgoingHalfEdgeIndex (idx_vertex))
        {
        }

        /** \brief Construct directly from the outgoing half-edge. */
        VertexAroundVertexCirculator (const HalfEdgeIndex& idx_outgoing_half_edge,
                                      Mesh*const           mesh)
          : mesh_                   (mesh),
            idx_outgoing_half_edge_ (idx_outgoing_half_edge)
        {
        }

        /** \brief Check if the circulator is valid.
          * \warning Does NOT check if the stored mesh pointer is valid. You have to ensure this yourself when constructing the circulator. */
        inline bool
        isValid () const
        {
          return (idx_outgoing_half_edge_.isValid ());
        }

        /** \brief Comparison operators (with boost::operators): == !=
          * \warning Does NOT check if the circulators belong to the same mesh. Please check this yourself. */
        inline bool
        operator == (const Self& other) const
        {
          return (idx_outgoing_half_edge_ == other.idx_outgoing_half_edge_);
        }

        /** \brief Increment operators (with boost::operators): ++ (pre and post) */
        inline Self&
        operator ++ ()
        {
          idx_outgoing_half_edge_ = mesh_->getNextHalfEdgeIndex (mesh_->getOppositeHalfEdgeIndex (idx_outgoing_half_edge_));
          return (*this);
        }

        /** \brief Decrement operators (with boost::operators): -- (pre and post) */
        inline Self&
        operator -- ()
        {
          idx_outgoing_half_edge_ = mesh_->getOppositeHalfEdgeIndex (mesh_->getPrevHalfEdgeIndex (idx_outgoing_half_edge_));
          return (*this);
        }

        /** \brief Get the index to the target vertex. */
        inline VertexIndex
        getTargetIndex () const
        {
          return (mesh_->getTerminatingVertexIndex (idx_outgoing_half_edge_));
        }

        /** \brief Get the half-edge that is currently stored in the circulator. */
        inline HalfEdgeIndex
        getCurrentHalfEdgeIndex () const
        {
          return (idx_outgoing_half_edge_);
        }

        /** \brief The mesh to which this circulator belongs to. */
        Mesh* mesh_;

        /** \brief The outgoing half-edge of the vertex around which we want to circulate. */
        HalfEdgeIndex idx_outgoing_half_edge_;
    };
  } // End namespace geometry
} // End namespace pcl

////////////////////////////////////////////////////////////////////////////////
// OutgoingHalfEdgeAroundVertexCirculator
////////////////////////////////////////////////////////////////////////////////

namespace pcl
{
  namespace geometry
  {
    /** \brief Circulates counter-clockwise around a vertex and returns an index to the outgoing half-edge (the target). The best way to declare the circulator is to use the method pcl::geometry::MeshBase::getOutgoingHalfEdgeAroundVertexCirculator ().
      * \tparam MeshT Mesh to which this circulator belongs to.
      * \note The circulator can't be used to change the connectivity in the mesh (only const circulators are valid).
      * \author Martin Saelzle
      * \ingroup geometry
      */
    template <class MeshT>
    class OutgoingHalfEdgeAroundVertexCirculator
        : boost::equality_comparable <pcl::geometry::OutgoingHalfEdgeAroundVertexCirculator <MeshT>
        , boost::unit_steppable      <pcl::geometry::OutgoingHalfEdgeAroundVertexCirculator <MeshT>
        > >
    {
      public:

        typedef boost::equality_comparable <pcl::geometry::OutgoingHalfEdgeAroundVertexCirculator <MeshT>
              , boost::unit_steppable      <pcl::geometry::OutgoingHalfEdgeAroundVertexCirculator <MeshT> > > Base;
        typedef pcl::geometry::OutgoingHalfEdgeAroundVertexCirculator <MeshT> Self;

        typedef MeshT Mesh;
        typedef typename Mesh::VertexIndex VertexIndex;
        typedef typename Mesh::HalfEdgeIndex HalfEdgeIndex;

        /** \brief Constructor resulting in an invalid circulator. */
        OutgoingHalfEdgeAroundVertexCirculator ()
          : mesh_                   (NULL),
            idx_outgoing_half_edge_ ()
        {
        }

        /** \brief Construct from the vertex around which we want to circulate. */
        OutgoingHalfEdgeAroundVertexCirculator (const VertexIndex& idx_vertex,
                                                Mesh*const         mesh)
          : mesh_                   (mesh),
            idx_outgoing_half_edge_ (mesh->getOutgoingHalfEdgeIndex (idx_vertex))
        {
        }

        /** \brief Construct directly from the outgoing half-edge. */
        OutgoingHalfEdgeAroundVertexCirculator (const HalfEdgeIndex& idx_outgoing_half_edge,
                                                Mesh*const           mesh)
          : mesh_                   (mesh),
            idx_outgoing_half_edge_ (idx_outgoing_half_edge)
        {
        }

        /** \brief Check if the circulator is valid.
          * \warning Does NOT check if the stored mesh pointer is valid. You have to ensure this yourself when constructing the circulator. */
        inline bool
        isValid () const
        {
          return (idx_outgoing_half_edge_.isValid ());
        }

        /** \brief Comparison operators (with boost::operators): == !=
          * \warning Does NOT check if the circulators belong to the same mesh. Please check this yourself. */
        inline bool
        operator == (const Self& other) const
        {
          return (idx_outgoing_half_edge_ == other.idx_outgoing_half_edge_);
        }

        /** \brief Increment operators (with boost::operators): ++ (pre and post) */
        inline Self&
        operator ++ ()
        {
          idx_outgoing_half_edge_ = mesh_->getNextHalfEdgeIndex (mesh_->getOppositeHalfEdgeIndex (idx_outgoing_half_edge_));
          return (*this);
        }

        /** \brief Decrement operators (with boost::operators): -- (pre and post) */
        inline Self&
        operator -- ()
        {
          idx_outgoing_half_edge_ = mesh_->getOppositeHalfEdgeIndex (mesh_->getPrevHalfEdgeIndex (idx_outgoing_half_edge_));
          return (*this);
        }

        /** \brief Get the index to the outgoing half-edge. */
        inline HalfEdgeIndex
        getTargetIndex () const
        {
          return (idx_outgoing_half_edge_);
        }

        /** \brief Get the half-edge that is currently stored in the circulator. */
        inline HalfEdgeIndex
        getCurrentHalfEdgeIndex () const
        {
          return (idx_outgoing_half_edge_);
        }

        /** \brief The mesh to which this circulator belongs to. */
        Mesh* mesh_;

        /** \brief The outgoing half-edge of the vertex around which we want to circulate. */
        HalfEdgeIndex idx_outgoing_half_edge_;
    };
  } // End namespace geometry
} // End namespace pcl

////////////////////////////////////////////////////////////////////////////////
// IncomingHalfEdgeAroundVertexCirculator
////////////////////////////////////////////////////////////////////////////////

namespace pcl
{
  namespace geometry
  {
    /** \brief Circulates counter-clockwise around a vertex and returns an index to the incoming half-edge (the target). The best way to declare the circulator is to use the method pcl::geometry::MeshBase::getIncomingHalfEdgeAroundVertexCirculator ().
      * \tparam MeshT Mesh to which this circulator belongs to.
      * \note The circulator can't be used to change the connectivity in the mesh (only const circulators are valid).
      * \author Martin Saelzle
      * \ingroup geometry
      */
    template <class MeshT>
    class IncomingHalfEdgeAroundVertexCirculator
        : boost::equality_comparable <pcl::geometry::IncomingHalfEdgeAroundVertexCirculator <MeshT>
        , boost::unit_steppable      <pcl::geometry::IncomingHalfEdgeAroundVertexCirculator <MeshT>
        > >
    {
      public:

        typedef boost::equality_comparable <pcl::geometry::IncomingHalfEdgeAroundVertexCirculator <MeshT>
              , boost::unit_steppable      <pcl::geometry::IncomingHalfEdgeAroundVertexCirculator <MeshT> > > Base;
        typedef pcl::geometry::IncomingHalfEdgeAroundVertexCirculator <MeshT> Self;

        typedef MeshT Mesh;
        typedef typename Mesh::VertexIndex VertexIndex;
        typedef typename Mesh::HalfEdgeIndex HalfEdgeIndex;

        /** \brief Constructor resulting in an invalid circulator. */
        IncomingHalfEdgeAroundVertexCirculator ()
          : mesh_                   (NULL),
            idx_incoming_half_edge_ ()
        {
        }

        /** \brief Construct from the vertex around which we want to circulate. */
        IncomingHalfEdgeAroundVertexCirculator (const VertexIndex& idx_vertex,
                                                Mesh*const         mesh)
          : mesh_                   (mesh),
            idx_incoming_half_edge_ (mesh->getIncomingHalfEdgeIndex (idx_vertex))
        {
        }

        /** \brief Construct directly from the incoming half-edge. */
        IncomingHalfEdgeAroundVertexCirculator (const HalfEdgeIndex& idx_incoming_half_edge,
                                                Mesh*const           mesh)
          : mesh_                   (mesh),
            idx_incoming_half_edge_ (idx_incoming_half_edge)
        {
        }

        /** \brief Check if the circulator is valid.
          * \warning Does NOT check if the stored mesh pointer is valid. You have to ensure this yourself when constructing the circulator. */
        inline bool
        isValid () const
        {
          return (idx_incoming_half_edge_.isValid ());
        }

        /** \brief Comparison operators (with boost::operators): == !=
          * \warning Does NOT check if the circulators belong to the same mesh. Please check this yourself. */
        inline bool
        operator == (const Self& other) const
        {
          return (idx_incoming_half_edge_ == other.idx_incoming_half_edge_);
        }

        /** \brief Increment operators (with boost::operators): ++ (pre and post) */
        inline Self&
        operator ++ ()
        {
          idx_incoming_half_edge_ = mesh_->getOppositeHalfEdgeIndex (mesh_->getNextHalfEdgeIndex (idx_incoming_half_edge_));
          return (*this);
        }

        /** \brief Decrement operators (with boost::operators): -- (pre and post) */
        inline Self&
        operator -- ()
        {
          idx_incoming_half_edge_ = mesh_->getPrevHalfEdgeIndex (mesh_->getOppositeHalfEdgeIndex (idx_incoming_half_edge_));
          return (*this);
        }

        /** \brief Get the index to the incoming half-edge. */
        inline HalfEdgeIndex
        getTargetIndex () const
        {
          return (idx_incoming_half_edge_);
        }

        /** \brief Get the half-edge that is currently stored in the circulator. */
        inline HalfEdgeIndex
        getCurrentHalfEdgeIndex () const
        {
          return (idx_incoming_half_edge_);
        }

        /** \brief The mesh to which this circulator belongs to. */
        Mesh* mesh_;

        /** \brief The incoming half-edge of the vertex around which we want to circulate. */
        HalfEdgeIndex idx_incoming_half_edge_;
    };
  } // End namespace geometry
} // End namespace pcl

////////////////////////////////////////////////////////////////////////////////
// FaceAroundVertexCirculator
////////////////////////////////////////////////////////////////////////////////

namespace pcl
{
  namespace geometry
  {
    /** \brief Circulates counter-clockwise around a vertex and returns an index to the face of the outgoing half-edge (the target). The best way to declare the circulator is to use the method pcl::geometry::MeshBase::getFaceAroundVertexCirculator ().
      * \tparam MeshT Mesh to which this circulator belongs to.
      * \note The circulator can't be used to change the connectivity in the mesh (only const circulators are valid).
      * \author Martin Saelzle
      * \ingroup geometry
      */
    template <class MeshT>
    class FaceAroundVertexCirculator
        : boost::equality_comparable <pcl::geometry::FaceAroundVertexCirculator <MeshT>
        , boost::unit_steppable      <pcl::geometry::FaceAroundVertexCirculator <MeshT>
        > >
    {
      public:

        typedef boost::equality_comparable <pcl::geometry::FaceAroundVertexCirculator <MeshT>
              , boost::unit_steppable      <pcl::geometry::FaceAroundVertexCirculator <MeshT> > > Base;
        typedef pcl::geometry::FaceAroundVertexCirculator <MeshT> Self;

        typedef MeshT Mesh;
        typedef typename Mesh::FaceIndex FaceIndex;
        typedef typename Mesh::VertexIndex VertexIndex;
        typedef typename Mesh::HalfEdgeIndex HalfEdgeIndex;

        /** \brief Constructor resulting in an invalid circulator. */
        FaceAroundVertexCirculator ()
          : mesh_                   (NULL),
            idx_outgoing_half_edge_ ()
        {
        }

        /** \brief Construct from the vertex around which we want to circulate. */
        FaceAroundVertexCirculator (const VertexIndex& idx_vertex,
                                    Mesh*const         mesh)
          : mesh_                   (mesh),
            idx_outgoing_half_edge_ (mesh->getOutgoingHalfEdgeIndex (idx_vertex))
        {
        }

        /** \brief Construct directly from the outgoing half-edge. */
        FaceAroundVertexCirculator (const HalfEdgeIndex& idx_outgoing_half_edge,
                                    Mesh*const           mesh)
          : mesh_                   (mesh),
            idx_outgoing_half_edge_ (idx_outgoing_half_edge)
        {
        }

        /** \brief Check if the circulator is valid.
          * \warning Does NOT check if the stored mesh pointer is valid. You have to ensure this yourself when constructing the circulator. */
        inline bool
        isValid () const
        {
          return (idx_outgoing_half_edge_.isValid ());
        }

        /** \brief Comparison operators (with boost::operators): == !=
          * \warning Does NOT check if the circulators belong to the same mesh. Please check this yourself. */
        inline bool
        operator == (const Self& other) const
        {
          return (idx_outgoing_half_edge_ == other.idx_outgoing_half_edge_);
        }

        /** \brief Increment operators (with boost::operators): ++ (pre and post) */
        inline Self&
        operator ++ ()
        {
          idx_outgoing_half_edge_ = mesh_->getNextHalfEdgeIndex (mesh_->getOppositeHalfEdgeIndex (idx_outgoing_half_edge_));
          return (*this);
        }

        /** \brief Decrement operators (with boost::operators): -- (pre and post) */
        inline Self&
        operator -- ()
        {
          idx_outgoing_half_edge_ = mesh_->getOppositeHalfEdgeIndex (mesh_->getPrevHalfEdgeIndex (idx_outgoing_half_edge_));
          return (*this);
        }

        /** \brief Get the index to the target face. */
        inline FaceIndex
        getTargetIndex () const
        {
          return (mesh_->getFaceIndex (idx_outgoing_half_edge_));
        }

        /** \brief Get the half-edge that is currently stored in the circulator. */
        inline HalfEdgeIndex
        getCurrentHalfEdgeIndex () const
        {
          return (idx_outgoing_half_edge_);
        }

        /** \brief The mesh to which this circulator belongs to. */
        Mesh* mesh_;

        /** \brief The outgoing half-edge of the vertex around which we want to circulate. */
        HalfEdgeIndex idx_outgoing_half_edge_;
    };
  } // End namespace geometry
} // End namespace pcl

////////////////////////////////////////////////////////////////////////////////
// VertexAroundFaceCirculator
////////////////////////////////////////////////////////////////////////////////

namespace pcl
{
  namespace geometry
  {
    /** \brief Circulates clockwise around a face and returns an index to the terminating vertex of the inner half-edge (the target). The best way to declare the circulator is to use the method pcl::geometry::MeshBase::getVertexAroundFaceCirculator ().
      * \tparam MeshT Mesh to which this circulator belongs to.
      * \note The circulator can't be used to change the connectivity in the mesh (only const circulators are valid).
      * \author Martin Saelzle
      * \ingroup geometry
      */
    template <class MeshT>
    class VertexAroundFaceCirculator
        : boost::equality_comparable <pcl::geometry::VertexAroundFaceCirculator <MeshT>
        , boost::unit_steppable      <pcl::geometry::VertexAroundFaceCirculator <MeshT>
        > >
    {
      public:

        typedef boost::equality_comparable <pcl::geometry::VertexAroundFaceCirculator <MeshT>
              , boost::unit_steppable      <pcl::geometry::VertexAroundFaceCirculator <MeshT> > > Base;
        typedef pcl::geometry::VertexAroundFaceCirculator <MeshT> Self;

        typedef MeshT Mesh;
        typedef typename Mesh::VertexIndex VertexIndex;
        typedef typename Mesh::FaceIndex FaceIndex;
        typedef typename Mesh::HalfEdgeIndex HalfEdgeIndex;

        /** \brief Constructor resulting in an invalid circulator. */
        VertexAroundFaceCirculator ()
          : mesh_                (NULL),
            idx_inner_half_edge_ ()
        {
        }

        /** \brief Construct from the face around which we want to circulate. */
        VertexAroundFaceCirculator (const FaceIndex& idx_face,
                                    Mesh*const       mesh)
          : mesh_                (mesh),
            idx_inner_half_edge_ (mesh->getInnerHalfEdgeIndex (idx_face))
        {
        }

        /** \brief Construct directly from the inner half-edge. */
        VertexAroundFaceCirculator (const HalfEdgeIndex& idx_inner_half_edge,
                                    Mesh*const           mesh)
          : mesh_                (mesh),
            idx_inner_half_edge_ (idx_inner_half_edge)
        {
        }

        /** \brief Check if the circulator is valid.
          * \warning Does NOT check if the stored mesh pointer is valid. You have to ensure this yourself when constructing the circulator. */
        inline bool
        isValid () const
        {
          return (idx_inner_half_edge_.isValid ());
        }

        /** \brief Comparison operators (with boost::operators): == !=
          * \warning Does NOT check if the circulators belong to the same mesh. Please check this yourself. */
        inline bool
        operator == (const Self& other) const
        {
          return (idx_inner_half_edge_ == other.idx_inner_half_edge_);
        }

        /** \brief Increment operators (with boost::operators): ++ (pre and post) */
        inline Self&
        operator ++ ()
        {
          idx_inner_half_edge_ = mesh_->getNextHalfEdgeIndex (idx_inner_half_edge_);
          return (*this);
        }

        /** \brief Decrement operators (with boost::operators): -- (pre and post) */
        inline Self&
        operator -- ()
        {
          idx_inner_half_edge_ = mesh_->getPrevHalfEdgeIndex (idx_inner_half_edge_);
          return (*this);
        }

        /** \brief Get the index to the target vertex. */
        inline VertexIndex
        getTargetIndex () const
        {
          return (mesh_->getTerminatingVertexIndex (idx_inner_half_edge_));
        }

        /** \brief Get the half-edge that is currently stored in the circulator. */
        inline HalfEdgeIndex
        getCurrentHalfEdgeIndex () const
        {
          return (idx_inner_half_edge_);
        }

        /** \brief The mesh to which this circulator belongs to. */
        Mesh* mesh_;

        /** \brief The inner half-edge of the face around which we want to circulate. */
        HalfEdgeIndex idx_inner_half_edge_;
    };
  } // End namespace geometry
} // End namespace pcl

////////////////////////////////////////////////////////////////////////////////
// InnerHalfEdgeAroundFaceCirculator
////////////////////////////////////////////////////////////////////////////////

namespace pcl
{
  namespace geometry
  {
    /** \brief Circulates clockwise around a face and returns an index to the inner half-edge (the target). The best way to declare the circulator is to use the method pcl::geometry::MeshBase::getInnerHalfEdgeAroundFaceCirculator ().
      * \tparam MeshT Mesh to which this circulator belongs to.
      * \note The circulator can't be used to change the connectivity in the mesh (only const circulators are valid).
      * \author Martin Saelzle
      * \ingroup geometry
      */
    template <class MeshT>
    class InnerHalfEdgeAroundFaceCirculator
        : boost::equality_comparable <pcl::geometry::InnerHalfEdgeAroundFaceCirculator <MeshT>
        , boost::unit_steppable      <pcl::geometry::InnerHalfEdgeAroundFaceCirculator <MeshT>
        > >
    {
      public:

        typedef boost::equality_comparable <pcl::geometry::InnerHalfEdgeAroundFaceCirculator <MeshT>
              , boost::unit_steppable      <pcl::geometry::InnerHalfEdgeAroundFaceCirculator <MeshT> > > Base;
        typedef pcl::geometry::InnerHalfEdgeAroundFaceCirculator <MeshT> Self;

        typedef MeshT Mesh;
        typedef typename Mesh::FaceIndex FaceIndex;
        typedef typename Mesh::HalfEdgeIndex HalfEdgeIndex;

        /** \brief Constructor resulting in an invalid circulator. */
        InnerHalfEdgeAroundFaceCirculator ()
          : mesh_                (NULL),
            idx_inner_half_edge_ ()
        {
        }

        /** \brief Construct from the face around which we want to circulate. */
        InnerHalfEdgeAroundFaceCirculator (const FaceIndex& idx_face,
                                           Mesh*const       mesh)
          : mesh_                (mesh),
            idx_inner_half_edge_ (mesh->getInnerHalfEdgeIndex (idx_face))
        {
        }

        /** \brief Construct directly from the inner half-edge. */
        InnerHalfEdgeAroundFaceCirculator (const HalfEdgeIndex& idx_inner_half_edge,
                                           Mesh*const           mesh)
          : mesh_                (mesh),
            idx_inner_half_edge_ (idx_inner_half_edge)
        {
        }

        /** \brief Check if the circulator is valid.
          * \warning Does NOT check if the stored mesh pointer is valid. You have to ensure this yourself when constructing the circulator. */
        inline bool
        isValid () const
        {
          return (idx_inner_half_edge_.isValid ());
        }

        /** \brief Comparison operators (with boost::operators): == !=
          * \warning Does NOT check if the circulators belong to the same mesh. Please check this yourself. */
        inline bool
        operator == (const Self& other) const
        {
          return (idx_inner_half_edge_ == other.idx_inner_half_edge_);
        }

        /** \brief Increment operators (with boost::operators): ++ (pre and post) */
        inline Self&
        operator ++ ()
        {
          idx_inner_half_edge_ = mesh_->getNextHalfEdgeIndex (idx_inner_half_edge_);
          return (*this);
        }

        /** \brief Decrement operators (with boost::operators): -- (pre and post) */
        inline Self&
        operator -- ()
        {
          idx_inner_half_edge_ = mesh_->getPrevHalfEdgeIndex (idx_inner_half_edge_);
          return (*this);
        }

        /** \brief Get the index to the inner half-edge. */
        inline HalfEdgeIndex
        getTargetIndex () const
        {
          return (idx_inner_half_edge_);
        }

        /** \brief Get the half-edge that is currently stored in the circulator. */
        inline HalfEdgeIndex
        getCurrentHalfEdgeIndex () const
        {
          return (idx_inner_half_edge_);
        }

        /** \brief The mesh to which this circulator belongs to. */
        Mesh* mesh_;

        /** \brief The inner half-edge of the face around which we want to circulate. */
        HalfEdgeIndex idx_inner_half_edge_;
    };
  } // End namespace geometry
} // End namespace pcl

////////////////////////////////////////////////////////////////////////////////
// OuterHalfEdgeAroundFaceCirculator
////////////////////////////////////////////////////////////////////////////////

namespace pcl
{
  namespace geometry
  {
    /** \brief Circulates clockwise around a face and returns an index to the outer half-edge (the target). The best way to declare the circulator is to use the method pcl::geometry::MeshBase::getOuterHalfEdgeAroundFaceCirculator ().
      * \tparam MeshT Mesh to which this circulator belongs to.
      * \note The circulator can't be used to change the connectivity in the mesh (only const circulators are valid).
      * \author Martin Saelzle
      * \ingroup geometry
      */
    template <class MeshT>
    class OuterHalfEdgeAroundFaceCirculator
        : boost::equality_comparable <pcl::geometry::OuterHalfEdgeAroundFaceCirculator <MeshT>
        , boost::unit_steppable      <pcl::geometry::OuterHalfEdgeAroundFaceCirculator <MeshT>
        > >
    {
      public:

        typedef boost::equality_comparable <pcl::geometry::OuterHalfEdgeAroundFaceCirculator <MeshT>
              , boost::unit_steppable      <pcl::geometry::OuterHalfEdgeAroundFaceCirculator <MeshT> > > Base;
        typedef pcl::geometry::OuterHalfEdgeAroundFaceCirculator <MeshT> Self;

        typedef MeshT Mesh;
        typedef typename Mesh::FaceIndex FaceIndex;
        typedef typename Mesh::HalfEdgeIndex HalfEdgeIndex;

        /** \brief Constructor resulting in an invalid circulator. */
        OuterHalfEdgeAroundFaceCirculator ()
          : mesh_                (NULL),
            idx_inner_half_edge_ ()
        {
        }

        /** \brief Construct from the face around which we want to circulate. */
        OuterHalfEdgeAroundFaceCirculator (const FaceIndex& idx_face,
                                           Mesh*const       mesh)
          : mesh_                (mesh),
            idx_inner_half_edge_ (mesh->getInnerHalfEdgeIndex (idx_face))
        {
        }

        /** \brief Construct directly from the inner half-edge. */
        OuterHalfEdgeAroundFaceCirculator (const HalfEdgeIndex& idx_inner_half_edge,
                                           Mesh*const           mesh)
          : mesh_                (mesh),
            idx_inner_half_edge_ (idx_inner_half_edge)
        {
        }

        /** \brief Check if the circulator is valid.
          * \warning Does NOT check if the stored mesh pointer is valid. You have to ensure this yourself when constructing the circulator. */
        inline bool
        isValid () const
        {
          return (idx_inner_half_edge_.isValid ());
        }

        /** \brief Comparison operators (with boost::operators): == !=
          * \warning Does NOT check if the circulators belong to the same mesh. Please check this yourself. */
        inline bool
        operator == (const Self& other) const
        {
          return (idx_inner_half_edge_ == other.idx_inner_half_edge_);
        }

        /** \brief Increment operators (with boost::operators): ++ (pre and post) */
        inline Self&
        operator ++ ()
        {
          idx_inner_half_edge_ = mesh_->getNextHalfEdgeIndex (idx_inner_half_edge_);
          return (*this);
        }

        /** \brief Decrement operators (with boost::operators): -- (pre and post) */
        inline Self&
        operator -- ()
        {
          idx_inner_half_edge_ = mesh_->getPrevHalfEdgeIndex (idx_inner_half_edge_);
          return (*this);
        }

        /** \brief Get the index to the outer half-edge. */
        inline HalfEdgeIndex
        getTargetIndex () const
        {
          return (mesh_->getOppositeHalfEdgeIndex (idx_inner_half_edge_));
        }

        /** \brief Get the half-edge that is currently stored in the circulator. */
        inline HalfEdgeIndex
        getCurrentHalfEdgeIndex () const
        {
          return (idx_inner_half_edge_);
        }

        /** \brief The mesh to which this circulator belongs to. */
        Mesh* mesh_;

        /** \brief The inner half-edge of the face around which we want to circulate. */
        HalfEdgeIndex idx_inner_half_edge_;
    };
  } // End namespace geometry
} // End namespace pcl

////////////////////////////////////////////////////////////////////////////////
// FaceAroundFaceCirculator
////////////////////////////////////////////////////////////////////////////////

namespace pcl
{
  namespace geometry
  {
    /** \brief Circulates clockwise around a face and returns an index to the face of the outer half-edge (the target). The best way to declare the circulator is to use the method pcl::geometry::MeshBase::getFaceAroundFaceCirculator ().
      * \tparam MeshT Mesh to which this circulator belongs to.
      * \note The circulator can't be used to change the connectivity in the mesh (only const circulators are valid).
      * \author Martin Saelzle
      * \ingroup geometry
      */
    template <class MeshT>
    class FaceAroundFaceCirculator
        : boost::equality_comparable <pcl::geometry::FaceAroundFaceCirculator <MeshT>
        , boost::unit_steppable      <pcl::geometry::FaceAroundFaceCirculator <MeshT>
        > >
    {
      public:

        typedef boost::equality_comparable <pcl::geometry::FaceAroundFaceCirculator <MeshT>
              , boost::unit_steppable      <pcl::geometry::FaceAroundFaceCirculator <MeshT> > > Base;
        typedef pcl::geometry::FaceAroundFaceCirculator <MeshT> Self;

        typedef MeshT Mesh;
        typedef typename Mesh::FaceIndex FaceIndex;
        typedef typename Mesh::HalfEdgeIndex HalfEdgeIndex;

        /** \brief Constructor resulting in an invalid circulator. */
        FaceAroundFaceCirculator ()
          : mesh_                (NULL),
            idx_inner_half_edge_ ()
        {
        }

        /** \brief Construct from the face around which we want to circulate. */
        FaceAroundFaceCirculator (const FaceIndex& idx_face,
                                  Mesh*const       mesh)
          : mesh_                (mesh),
            idx_inner_half_edge_ (mesh->getInnerHalfEdgeIndex (idx_face))
        {
        }

        /** \brief Construct directly from the inner half-edge. */
        FaceAroundFaceCirculator (const HalfEdgeIndex& idx_inner_half_edge,
                                  Mesh*const           mesh)
          : mesh_                (mesh),
            idx_inner_half_edge_ (idx_inner_half_edge)
        {
        }

        /** \brief Check if the circulator is valid.
          * \warning Does NOT check if the stored mesh pointer is valid. You have to ensure this yourself when constructing the circulator. */
        inline bool
        isValid () const
        {
          return (idx_inner_half_edge_.isValid ());
        }

        /** \brief Comparison operators (with boost::operators): == !=
          * \warning Does NOT check if the circulators belong to the same mesh. Please check this yourself. */
        inline bool
        operator == (const Self& other) const
        {
          return (idx_inner_half_edge_ == other.idx_inner_half_edge_);
        }

        /** \brief Increment operators (with boost::operators): ++ (pre and post) */
        inline Self&
        operator ++ ()
        {
          idx_inner_half_edge_ = mesh_->getNextHalfEdgeIndex (idx_inner_half_edge_);
          return (*this);
        }

        /** \brief Decrement operators (with boost::operators): -- (pre and post) */
        inline Self&
        operator -- ()
        {
          idx_inner_half_edge_ = mesh_->getPrevHalfEdgeIndex (idx_inner_half_edge_);
          return (*this);
        }

        /** \brief Get the index to the target face. */
        inline FaceIndex
        getTargetIndex () const
        {
          return (mesh_->getOppositeFaceIndex (idx_inner_half_edge_));
        }

        /** \brief Get the half-edge that is currently stored in the circulator. */
        inline HalfEdgeIndex
        getCurrentHalfEdgeIndex () const
        {
          return (idx_inner_half_edge_);
        }

        /** \brief The mesh to which this circulator belongs to. */
        Mesh* mesh_;

        /** \brief The inner half-edge of the face around which we want to circulate. */
        HalfEdgeIndex idx_inner_half_edge_;
    };
  } // End namespace geometry
} // End namespace pcl

#endif // PCL_GEOMETRY_MESH_CIRCULATORS_H
