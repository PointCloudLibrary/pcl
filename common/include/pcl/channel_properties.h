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
 * $Id: channel_properties.h 4696 2012-02-23 06:12:55Z rusu $
 *
 */

#ifndef PCL_CHANNEL_PROPERTIES_H_
#define PCL_CHANNEL_PROPERTIES_H_

#include <pcl/pcl_macros.h>
#include <string>

namespace pcl
{
  /** \brief ChannelProperties stores the properties of each channel in a cloud, namely:
    *
    *   - \b name : the channel's name (e.g., "xyz", "rgb", "fpfh")
    *   - \b offset : its offset in the data array
    *   - \b size : its size in bytes
    *   - \b count : its number of elements (e.g., 1 for xyz data, 33 for an FPFH signature)
    *   - \b datatype : its data type. By convention:
    *
    *     - INT8    = 1
    *     - UINT8   = 2
    *     - INT16   = 3
    *     - UINT16  = 4
    *     - INT32   = 5
    *     - UINT32  = 6
    *     - FLOAT32 = 7
    *     - FLOAT64 = 8
    *
    * <b>This part of the API is for advanced users only, and constitutes a transition to the 2.0 API!</b>
    *
    * \author Radu B. Rusu
    */
  class ChannelProperties
  {
    public:

      /** \brief Default constructor. Sets:
        *
        *   - \ref name to ""
        *   - \ref offset to 0
        *   - \ref size to 0
        *   - \ref count to 1
        *   - \ref datatype to 7 (FLOAT32)
        */
      ChannelProperties () :
        name (""), offset (0), size (0), count (1), datatype (7)
      {
      }

      /** \brief The name of the channel (e.g., "xyz", "rgb"). */
      std::string name;

      /** \brief The byte offset where data for the channel starts in the point cloud. */
      pcl::uint32_t offset;

      /** \brief The size of bytes per each element in the channel. */
      pcl::uint32_t size;

      /** \brief The number of elements per channel (e.g., 1 for "xyz", 33 for "fpfh"). */
      pcl::uint32_t count;

      /** \brief The type of data the channel contains. 
        * By convention:
        *     - INT8    = 1
        *     - UINT8   = 2
        *     - INT16   = 3
        *     - UINT16  = 4
        *     - INT32   = 5
        *     - UINT32  = 6
        *     - FLOAT32 = 7
        *     - FLOAT64 = 8
        */
      pcl::uint8_t datatype;
   };
}

#endif  // PCL_CHANNEL_PROPERTIES_H_


