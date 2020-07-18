/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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

#include <pcl/common/common.h>

#include <istream>
#include <ostream>

namespace pcl {

/** 2D point with integer x- and y-coordinates. */
class PCL_EXPORTS PointXY32i {
public:
  /** Constructor. */
  inline PointXY32i() : x(0), y(0) {}

  /** Destructor. */
  inline virtual ~PointXY32i() {}

  /** Serializes the point to the specified stream.
   *
   * \param[out] stream the destination for the serialization
   */
  inline void
  serialize(std::ostream& stream) const
  {
    stream.write(reinterpret_cast<const char*>(&x), sizeof(x));
    stream.write(reinterpret_cast<const char*>(&y), sizeof(y));
  }

  /** Deserializes the point from the specified stream.
   *
   * \param[in] stream the source for the deserialization
   */
  inline void
  deserialize(std::istream& stream)
  {
    stream.read(reinterpret_cast<char*>(&x), sizeof(x));
    stream.read(reinterpret_cast<char*>(&y), sizeof(y));
  }

  /** Creates a random point within the specified window.
   *
   * \param[in] min_x the minimum value for the x-coordinate of the point
   * \param[in] max_x the maximum value for the x-coordinate of the point
   * \param[in] min_y the minimum value for the y-coordinate of the point
   * \param[in] max_y the maximum value for the y-coordinate of the point
   */
  static PointXY32i
  randomPoint(const int min_x, const int max_x, const int min_y, const int max_y);

public:
  /** The x-coordinate of the point. */
  int x;
  /** The y-coordinate of the point. */
  int y;
};

} // namespace pcl
