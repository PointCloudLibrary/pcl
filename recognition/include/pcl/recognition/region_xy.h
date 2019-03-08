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

#include <fstream>

namespace pcl
{
  /** \brief Function for reading data from a stream. */
  template <class Type>
  void read (std::istream & stream, Type & value)
  {
    stream.read (reinterpret_cast<char*> (&value), sizeof(value));
  }

  /** \brief Function for reading data arrays from a stream. */
  template <class Type>
  void read (std::istream & stream, Type * value, int nr_values)
  {
    for (int value_index = 0; value_index < nr_values; ++value_index)
    {
      read (stream, value[value_index]);
    }
  }

  /** \brief Function for writing data to a stream. */
  template <class Type>
  void write (std::ostream & stream, Type value)
  {
    stream.write (reinterpret_cast<char*> (&value), sizeof (value));
  }

  /** \brief Function for writing data arrays to a stream. */
  template <class Type>
  void write (std::ostream & stream, Type * value, int nr_values)
  {
    for (int value_index = 0; value_index < nr_values; ++value_index)
    {
      write (stream, value[value_index]);
    }
  }

  /** \brief Defines a region in XY-space.
    * \author Stefan Holzer
    */
  struct PCL_EXPORTS RegionXY
  {
    /** \brief Constructor. */
    RegionXY () : x (0), y (0), width (0), height (0) {}

    /** \brief x-position of the region. */
    int x;
    /** \brief y-position of the region. */
    int y;
    /** \brief width of the region. */
    int width;
    /** \brief height of the region. */
    int height;

    /** \brief Serializes the object to the specified stream.
      * \param[out] stream the stream the object will be serialized to. */
    void
    serialize (std::ostream & stream) const
    {
      write (stream, x);
      write (stream, y);
      write (stream, width);
      write (stream, height);
    }

    /** \brief Deserializes the object from the specified stream.
      * \param[in] stream the stream the object will be deserialized from. */
    void 
    deserialize (::std::istream & stream)
    {
      read (stream, x);
      read (stream, y);
      read (stream, width);
      read (stream, height);
    }

  };
}
