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

#pragma once

#include <boost/operators.hpp>

#include <iostream>

////////////////////////////////////////////////////////////////////////////////
// MeshIndex
////////////////////////////////////////////////////////////////////////////////

namespace pcl {
namespace detail {

template <class IndexTagT>
class MeshIndex;

template <class IndexTagT>
std::istream&
operator>>(std::istream& is, MeshIndex<IndexTagT>&);

template <class IndexTagT>
class MeshIndex
: boost::totally_ordered<
      MeshIndex<IndexTagT>,                                      // < > <= >= == !=
      boost::unit_steppable<MeshIndex<IndexTagT>,                // ++ -- (pre and post)
                            boost::additive<MeshIndex<IndexTagT> // += +
                                                                 // -= -
                                            >>> {

public:
  using Base = boost::totally_ordered<
      MeshIndex<IndexTagT>,
      boost::unit_steppable<MeshIndex<IndexTagT>,
                            boost::additive<MeshIndex<IndexTagT>>>>;
  using Self = MeshIndex<IndexTagT>;

  /** \brief Constructor. Initializes with an invalid index. */
  MeshIndex() : index_(-1) {}

  /** \brief Constructor.
   * \param[in] index The integer index.
   */
  explicit MeshIndex(const int index) : index_(index) {}

  /** \brief Returns true if the index is valid. */
  inline bool
  isValid() const
  {
    return (index_ >= 0);
  }

  /** \brief Invalidate the index. */
  inline void
  invalidate()
  {
    index_ = -1;
  }

  /** \brief Get the index. */
  inline int
  get() const
  {
    return (index_);
  }

  /** \brief Set the index. */
  inline void
  set(const int index)
  {
    index_ = index;
  }

  /** \brief Comparison operators (with boost::operators): < > <= >= */
  inline bool
  operator<(const Self& other) const
  {
    return (this->get() < other.get());
  }

  /** \brief Comparison operators (with boost::operators): == != */
  inline bool
  operator==(const Self& other) const
  {
    return (this->get() == other.get());
  }

  /** \brief Increment operators (with boost::operators): ++ (pre and post) */
  inline Self&
  operator++()
  {
    ++index_;
    return (*this);
  }

  /** \brief Decrement operators (with boost::operators): \-\- (pre and post) */
  inline Self&
  operator--()
  {
    --index_;
    return (*this);
  }

  /** \brief Addition operators (with boost::operators): + += */
  inline Self&
  operator+=(const Self& other)
  {
    index_ += other.get();
    return (*this);
  }

  /** \brief Subtraction operators (with boost::operators): - -= */
  inline Self&
  operator-=(const Self& other)
  {
    index_ -= other.get();
    return (*this);
  }

private:
  /** \brief Stored index. */
  int index_;

  friend std::istream&
  operator>><>(std::istream& is, MeshIndex<IndexTagT>& index);
};

/** \brief ostream operator. */
template <class IndexTagT>
inline std::ostream&
operator<<(std::ostream& os, const MeshIndex<IndexTagT>& index)
{
  return (os << index.get());
}

/** \brief istream operator. */
template <class IndexTagT>
inline std::istream&
operator>>(std::istream& is, MeshIndex<IndexTagT>& index)
{
  return (is >> index.index_);
}

} // End namespace detail
} // End namespace pcl

namespace pcl {
namespace geometry {
/** \brief Index used to access elements in the half-edge mesh. It is basically just a
 * wrapper around an integer with a few added methods.
 * \author Martin Saelzle
 * \ingroup geometry
 */
using VertexIndex = pcl::detail::MeshIndex<struct VertexIndexTag>;
/** \brief Index used to access elements in the half-edge mesh. It is basically just a
 * wrapper around an integer with a few added methods.
 * \author Martin Saelzle
 * \ingroup geometry
 */
using HalfEdgeIndex = pcl::detail::MeshIndex<struct HalfEdgeIndexTag>;
/** \brief Index used to access elements in the half-edge mesh. It is basically just a
 * wrapper around an integer with a few added methods.
 * \author Martin Saelzle
 * \ingroup geometry
 */
using EdgeIndex = pcl::detail::MeshIndex<struct EdgeIndexTag>;
/** \brief Index used to access elements in the half-edge mesh. It is basically just a
 * wrapper around an integer with a few added methods.
 * \author Martin Saelzle
 * \ingroup geometry
 */
using FaceIndex = pcl::detail::MeshIndex<struct FaceIndexTag>;

} // End namespace geometry
} // End namespace pcl

////////////////////////////////////////////////////////////////////////////////
// Conversions
////////////////////////////////////////////////////////////////////////////////

namespace pcl {
namespace geometry {
/** \brief Convert the given half-edge index to an edge index. */
inline EdgeIndex
toEdgeIndex(const HalfEdgeIndex& index)
{
  return (index.isValid() ? EdgeIndex(index.get() / 2) : EdgeIndex());
}

/** \brief Convert the given edge index to a half-edge index.
 * \param index
 * \param[in] get_first The first half-edge of the edge is returned if this
 * variable is true; elsewise the second.
 */
inline HalfEdgeIndex
toHalfEdgeIndex(const EdgeIndex& index, const bool get_first = true)
{
  return (index.isValid()
              ? HalfEdgeIndex(index.get() * 2 + static_cast<int>(!get_first))
              : HalfEdgeIndex());
}
} // End namespace geometry
} // End namespace pcl
