/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#pragma once

#include <cstdint>
#include <cstring> // for memcpy

namespace pcl {
namespace octree {
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief @b Octree key class
 *  \note Octree keys contain integer indices for each coordinate axis in order to
 * address an octree leaf node.
 * \author Julius Kammerl (julius@kammerl.de)
 */
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class OctreeKey {
public:
  /** \brief Empty constructor. */
  OctreeKey() : x(0), y(0), z(0) {}

  /** \brief Constructor for key initialization. */
  OctreeKey(uindex_t keyX, uindex_t keyY, uindex_t keyZ) : x(keyX), y(keyY), z(keyZ) {}

  /** \brief Copy constructor. */
  OctreeKey(const OctreeKey& source) { std::memcpy(key_, source.key_, sizeof(key_)); }

  OctreeKey&
  operator=(const OctreeKey&) = default;

  /** \brief Operator== for comparing octree keys with each other.
   *  \return "true" if leaf node indices are identical; "false" otherwise.
   * */
  bool
  operator==(const OctreeKey& b) const
  {
    return ((b.x == this->x) && (b.y == this->y) && (b.z == this->z));
  }

  /** \brief Inequal comparison operator
   * \param[in] other OctreeIteratorBase to compare with
   * \return "true" if the current and other iterators are different ; "false"
   * otherwise.
   */
  bool
  operator!=(const OctreeKey& other) const
  {
    return !operator==(other);
  }

  /** \brief Operator<= for comparing octree keys with each other.
   *  \return "true" if key indices are not greater than the key indices of b  ; "false"
   * otherwise.
   * */
  bool
  operator<=(const OctreeKey& b) const
  {
    return ((b.x >= this->x) && (b.y >= this->y) && (b.z >= this->z));
  }

  /** \brief Operator>= for comparing octree keys with each other.
   *  \return "true" if key indices are not smaller than the key indices of b  ; "false"
   * otherwise.
   * */
  bool
  operator>=(const OctreeKey& b) const
  {
    return ((b.x <= this->x) && (b.y <= this->y) && (b.z <= this->z));
  }

  /** \brief push a child node to the octree key
   *  \param[in] childIndex index of child node to be added (0-7)
   * */
  inline void
  pushBranch(unsigned char childIndex)
  {
    this->x = (this->x << 1) | (!!(childIndex & (1 << 2)));
    this->y = (this->y << 1) | (!!(childIndex & (1 << 1)));
    this->z = (this->z << 1) | (!!(childIndex & (1 << 0)));
  }

  /** \brief pop child node from octree key
   * */
  inline void
  popBranch()
  {
    this->x >>= 1;
    this->y >>= 1;
    this->z >>= 1;
  }

  /** \brief get child node index using depthMask
   *  \param[in] depthMask bit mask with single bit set at query depth
   *  \return child node index
   * */
  inline unsigned char
  getChildIdxWithDepthMask(uindex_t depthMask) const
  {
    return static_cast<unsigned char>(((!!(this->x & depthMask)) << 2) |
                                      ((!!(this->y & depthMask)) << 1) |
                                      (!!(this->z & depthMask)));
  }

  /* \brief maximum depth that can be addressed */
  static const unsigned char maxDepth =
      static_cast<unsigned char>(sizeof(uindex_t) * 8);

  // Indices addressing a voxel at (X, Y, Z)

  union {
    struct {
      uindex_t x;
      uindex_t y;
      uindex_t z;
    };
    uindex_t key_[3];
  };
};
} // namespace octree
} // namespace pcl
