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

#include "pcl/recognition/quantizable_modality.h"
#include <cstddef>
#include <string.h>

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::QuantizableModality::
QuantizableModality ()
{
}

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::QuantizableModality::
~QuantizableModality ()
{
}

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::MaskMap::
MaskMap ()
  : data_ (0), width_ (0), height_ (0)
{
}

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::MaskMap::
MaskMap (const size_t width, const size_t height)
  : data_ (width*height), width_ (width), height_ (height)
{
}  

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::MaskMap::
~MaskMap ()
{
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::MaskMap::
resize (const size_t width, const size_t height)
{
  data_.resize (width*height);
  width_ = width;
  height_ = height;
}


//////////////////////////////////////////////////////////////////////////////////////////////
pcl::QuantizedMap::
QuantizedMap ()
  : data_ (0), width_ (0), height_ (0)
{
}

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::QuantizedMap::
QuantizedMap (const size_t width, const size_t height)
  : data_ (width*height), width_ (width), height_ (height)
{
}

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::QuantizedMap::
~QuantizedMap ()
{
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::QuantizedMap::
resize (const size_t width, const size_t height)
{
  data_.resize (width*height);
  width_ = width;
  height_ = height;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::QuantizedMap::
spreadQuantizedMap (const QuantizedMap & input_map, QuantizedMap & output_map, const int spreading_size)
{
  const size_t width = input_map.getWidth ();
  const size_t height = input_map.getHeight ();

  QuantizedMap tmp_map (width, height);

  output_map.resize (width, height);

  for (int row_index = spreading_size; row_index < height-spreading_size-1; ++row_index)
  {
    for (int col_index = spreading_size; col_index < width-spreading_size-1; ++col_index)
    {
      unsigned char value = 0;
      for (int col_index2 = col_index-spreading_size; col_index2 <= col_index+spreading_size; ++col_index2)
      {
        //if (row_index2 < 0 || row_index2 >= height || col_index2 < 0 || col_index2 >= width) continue;
        value |= input_map (col_index2, row_index);
      }
      tmp_map (col_index, row_index) = value;
    }
  }

  for (int row_index = spreading_size; row_index < height-spreading_size-1; ++row_index)
  {
    for (int col_index = spreading_size; col_index < width-spreading_size-1; ++col_index)
    {
      unsigned char value = 0;
      for (int row_index2 = row_index-spreading_size; row_index2 <= row_index+spreading_size; ++row_index2)
      {
        //if (row_index2 < 0 || row_index2 >= height || col_index2 < 0 || col_index2 >= width) continue;
        value |= tmp_map (col_index, row_index2);
      }
      output_map (col_index, row_index) = value;
    }
  }
}
