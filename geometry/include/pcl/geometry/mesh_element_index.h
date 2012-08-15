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

#ifndef PCL_GEOMETRY_MESH_ELEMENT_INDEX_H
#define PCL_GEOMETRY_MESH_ELEMENT_INDEX_H

#include <ostream>

#include <boost/operators.hpp>

#include <pcl/pcl_exports.h>

////////////////////////////////////////////////////////////////////////////////
// BaseMeshElementIndex
////////////////////////////////////////////////////////////////////////////////

namespace pcl
{

  /** \brief Base class for the mesh element indexes.
    *
    * \tparam DerivedT VertexIndex, HalfEdgeIndex, FaceIndex
    *
    * The mesh class stores mesh elements which in turn store indexes to other mesh elements. The corresponding mesh element to a given index can be accessed by the MeshBase::getElement method in the mesh.
    *
    * BaseMeshElementIndex serves as a base class for these indexes. Basically it is just a wrapper around an integer index with a few additional methods.
    *
    * \author Martin Saelzle
    * \ingroup geometry
    */
  template <class DerivedT>
  class BaseMeshElementIndex
      : boost::totally_ordered <BaseMeshElementIndex <DerivedT> // < > <= >= == !=
      , boost::unit_steppable  <BaseMeshElementIndex <DerivedT> // ++ -- (pre and post)
      , boost::additive        <BaseMeshElementIndex <DerivedT> // += + -= -
      > > >
  {
      //////////////////////////////////////////////////////////////////////////
      // Types
      //////////////////////////////////////////////////////////////////////////

    public:

      typedef pcl::BaseMeshElementIndex <DerivedT> Self;
      typedef DerivedT                             Derived;

      //////////////////////////////////////////////////////////////////////////
      // Constructor
      //////////////////////////////////////////////////////////////////////////

    public:

      /** \brief Constructor
        * \param idx (optional) The integer index; defaults to -1 (invalid index)
        */
      BaseMeshElementIndex (const int idx = -1)
        : idx_ (idx)
      {
      }

      //////////////////////////////////////////////////////////////////////////
      // Access the stored index
      //////////////////////////////////////////////////////////////////////////

    public:

      /** \brief Returns the index (non-const) */
      int&
      getIndex ()
      {
        return (idx_);
      }

      /** \brief Returns the index (const) */
      int
      getIndex () const
      {
        return (idx_);
      }

      /** \brief Set the index */
      void
      setIndex (const int idx)
      {
        idx_ = idx;
      }

      //////////////////////////////////////////////////////////////////////////
      // Valid
      //////////////////////////////////////////////////////////////////////////

    public:

      /** \brief Returns true if the index is valid */
      bool
      isValid () const
      {
        return (this->getIndex () >= 0);
      }

      /** \brief Invalidate the index (set it to -1) */
      void
      invalidate ()
      {
        this->setIndex (-1);
      }

      //////////////////////////////////////////////////////////////////////////
      // Operators
      //////////////////////////////////////////////////////////////////////////

    public:

      /** \brief Comparison operators (with boost::operators): < > <= >= */
      bool
      operator < (const Self& other) const
      {
        return (this->getIndex () < other.getIndex ());
      }

      /** \brief Comparison operators (with boost::operators): == != */
      bool
      operator == (const Self& other) const
      {
        return (this->getIndex () == other.getIndex ());
      }

      /** \brief Increment operators (with boost::operators): ++ (pre and post) */
      Self&
      operator ++ ()
      {
        ++this->getIndex ();
        return (*this);
      }

      /** \brief Decrement operators (with boost::operators): \-\- (pre and post) */
      Self&
      operator -- ()
      {
        --this->getIndex ();
        return (*this);
      }

      /** \brief Addition operators (with boost::operators): + += */
      Self&
      operator += (const Self& other)
      {
        this->getIndex () += other.getIndex ();
        return (*this);
      }

      /** \brief Subtraction operators (with boost::operators): - -= */
      Self&
      operator -= (const Self& other)
      {
        this->getIndex () -= other.getIndex ();
        return (*this);
      }

      /** \brief Assignment operator */
      Self&
      operator = (const Self& other)
      {
        this->setIndex (other.getIndex ());
        return (*this);
      }

      /** \brief Conversion operator to the derived class */
      operator Derived () const
      {
        return Derived (this->getIndex ());
      }

      //////////////////////////////////////////////////////////////////////////
      // Members
      //////////////////////////////////////////////////////////////////////////

    private:

      /** \brief Stored index */
      int idx_;
  };

} // End namespace pcl

/** \brief ostream operator */
template <class DerivedT>  std::ostream&
operator << (std::ostream& os, const pcl::BaseMeshElementIndex <DerivedT>& idx)
{
  return (os << idx.getIndex ());
}

////////////////////////////////////////////////////////////////////////////////
// VertexIndex
////////////////////////////////////////////////////////////////////////////////

namespace pcl
{

  /** \brief VertexIndex for accessing a Vertex.
    * \see BaseMeshElementIndex
    * \author Martin Saelzle
    * \ingroup geometry
    */
  class PCL_EXPORTS VertexIndex : public pcl::BaseMeshElementIndex <pcl::VertexIndex>
  {
    public:

      typedef pcl::BaseMeshElementIndex <pcl::VertexIndex> Base;
      typedef Base::Derived                                Self;

    public:

      /** \brief Constructor
        * \param idx (optional) The integer index; defaults to -1 (invalid index)
        */
      VertexIndex (const int idx = -1);
  };

} // End namespace pcl

////////////////////////////////////////////////////////////////////////////////
// HalfEdgeIndex
////////////////////////////////////////////////////////////////////////////////

namespace pcl
{

  /** \brief HalfEdgeIndex for accessing a HalfEdge.
    * \see BaseMeshElementIndex
    * \author Martin Saelzle
    * \ingroup geometry
    */
  class PCL_EXPORTS HalfEdgeIndex : public pcl::BaseMeshElementIndex <pcl::HalfEdgeIndex>
  {
    public:

      typedef pcl::BaseMeshElementIndex <pcl::HalfEdgeIndex> Base;
      typedef Base::Derived                                  Self;

    public:

      /** \brief Constructor
        * \param idx (optional) The integer index; defaults to -1 (invalid index)
        */
      HalfEdgeIndex (const int idx = -1);
  };

} // End namespace pcl

////////////////////////////////////////////////////////////////////////////////
// EdgeIndex
////////////////////////////////////////////////////////////////////////////////

//namespace pcl
//{

//  class PCL_EXPORTS EdgeIndex : public pcl::BaseMeshElementIndex <pcl::EdgeIndex>
//  {
//    public:

//      typedef pcl::BaseMeshElementIndex <pcl::EdgeIndex> Base;
//      typedef Base::Derived                              Self;

//    public:

//      EdgeIndex (const pcl::HalfEdgeIndex& idx_he);

//  };

//} // End namespace pcl

////////////////////////////////////////////////////////////////////////////////
// FaceIndex
////////////////////////////////////////////////////////////////////////////////

namespace pcl
{

  /** \brief FaceIndex for accessing Face.
    * \see BaseMeshElementIndex
    * \author Martin Saelzle
    * \ingroup geometry
    */
  class PCL_EXPORTS FaceIndex : public pcl::BaseMeshElementIndex <pcl::FaceIndex>
  {
    public:

      typedef pcl::BaseMeshElementIndex <pcl::FaceIndex> Base;
      typedef Base::Derived                              Self;

    public:

      /** \brief Constructor
        * \param idx (optional) The integer index; defaults to -1 (invalid index)
        */
      FaceIndex (const int idx = -1);
  };

} // End namespace pcl

#endif // PCL_GEOMETRY_MESH_ELEMENT_INDEX_H

