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

#ifndef MESH_ELEMENT_INDEX_H
#define MESH_ELEMENT_INDEX_H

#include <ostream>

#include <boost/operators.hpp>

////////////////////////////////////////////////////////////////////////////////
// BaseMeshElementIndex
////////////////////////////////////////////////////////////////////////////////

namespace pcl
{
  template <class DerivedT>
  class BaseMeshElementIndex
      : boost::totally_ordered <BaseMeshElementIndex <DerivedT> // < > <= >= == !=
      , boost::unit_steppable  <BaseMeshElementIndex <DerivedT> // ++ -- (pre and post)
      , boost::additive        <BaseMeshElementIndex <DerivedT> // += + -= -
      > > >
  {
    public:

      typedef pcl::BaseMeshElementIndex <DerivedT> Self;
      typedef DerivedT                             Derived;

    public:

      BaseMeshElementIndex (const int idx = -1)
        : idx_ (idx)
      {
      }

      BaseMeshElementIndex (const Self& other)
        : idx_ (other.idx ())
      {
      }

    public:

      inline int&
      idx ()
      {
        return (idx_);
      }

      inline int
      idx () const
      {
        return (idx_);
      }

      inline bool
      isValid () const
      {
        return (this->idx () >= 0);
      }

      inline void
      invalidate ()
      {
        this->idx () = -1;
      }

    public:

      inline bool
      operator < (const Self& other) const
      {
        return (this->idx () < other.idx ());
      }

      inline bool
      operator == (const Self& other) const
      {
        return (this->idx () == other.idx ());
      }

      inline Self&
      operator ++ ()
      {
        ++this->idx ();
        return (*this);
      }

      inline Self&
      operator -- ()
      {
        --this->idx ();
        return (*this);
      }

      inline Self&
      operator += (const Self& other)
      {
        this->idx () += other.idx ();
        return (*this);
      }

      inline Self&
      operator -= (const Self& other)
      {
        this->idx () -= other.idx ();
        return (*this);
      }

      inline Self&
      operator = (const Self& other)
      {
        this->idx () = other.idx ();
        return (*this);
      }

      operator Derived () const
      {
        return Derived (this->idx ());
      }

    private:

      int idx_;
  };

  template <class DerivedT> inline std::ostream&
  operator << (std::ostream& os, const BaseMeshElementIndex <DerivedT>& idx)
  {
    return (os << idx.idx ());
  }

} // End namespace pcl

////////////////////////////////////////////////////////////////////////////////
// VertexIndex
////////////////////////////////////////////////////////////////////////////////

namespace pcl
{

  class VertexIndex : public pcl::BaseMeshElementIndex <pcl::VertexIndex>
  {
    public:

      typedef pcl::BaseMeshElementIndex <pcl::VertexIndex> Base;
      typedef Base::Derived                                Self;

    public:

      VertexIndex (const int idx = -1);
      VertexIndex (const Self& other);
  };

} // End namespace pcl

////////////////////////////////////////////////////////////////////////////////
// HalfEdgeIndex
////////////////////////////////////////////////////////////////////////////////

namespace pcl
{

  class HalfEdgeIndex : public pcl::BaseMeshElementIndex <pcl::HalfEdgeIndex>
  {
    public:

      typedef pcl::BaseMeshElementIndex <pcl::HalfEdgeIndex> Base;
      typedef Base::Derived                                  Self;

    public:

      HalfEdgeIndex (const int idx = -1);
      HalfEdgeIndex (const Self& other);
  };

} // End namespace pcl

////////////////////////////////////////////////////////////////////////////////
// FaceIndex
////////////////////////////////////////////////////////////////////////////////

namespace pcl
{

  class FaceIndex : public pcl::BaseMeshElementIndex <pcl::FaceIndex>
  {
    public:

      typedef pcl::BaseMeshElementIndex <pcl::FaceIndex> Base;
      typedef Base::Derived                              Self;

    public:

      FaceIndex (const int idx = -1);
      FaceIndex (const Self& other);
  };

} // End namespace pcl

#endif // MESH_ELEMENT_INDEX_H

