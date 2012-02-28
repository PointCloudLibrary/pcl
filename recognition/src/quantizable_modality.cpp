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
: data_ (NULL), width_ (-1), height_ (-1)
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
initialize (
   const int width,
   const int height )
{
  if (data_ != NULL && (width_ != width || height_ != height))
  {
    delete[] data_;
    data_ = NULL;
    width_ = -1;
    height_ = -1;

  }

  if (data_ == NULL)
  {
    data_ = new unsigned char[width*height];
    memset (data_, 0, sizeof (unsigned char)*width*height);

    width_ = width;
    height_ = height;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::MaskMap::
release ()
{
  if (data_ != NULL) delete[] data_;

  data_ = NULL;
  width_ = -1;
  height_ = -1;
}

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::QuantizedMap::
QuantizedMap ()
{
  data_ = NULL;
  width_ = -1;
  height_ = -1;
}

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::QuantizedMap::
QuantizedMap (
   const int width,
   const int height )
{
  initialize (width, height);
}

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::QuantizedMap::
~QuantizedMap ()
{
  delete[] data_;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::QuantizedMap::
initialize (
   const int width,
   const int height )
{
  if (data_ != NULL && (width_ != width || height_ != height))
  {
    delete[] data_;

    width_ = -1;
    height_ = -1;
    data_ = NULL;
  }
  
  if (data_ == NULL)
  {
    width_ = width;
    height_ = height;
    data_ = new unsigned char[width*height];
  }

  memset (data_, 0, sizeof (unsigned char)*width*height);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::QuantizedMap::
spreadQuantizedMap (
  QuantizedMap & inputMap,
  QuantizedMap & outputMap,
  const int spreadingSize )
{
  const int width = inputMap.getWidth ();
  const int height = inputMap.getHeight ();

  QuantizedMap tmpMap;
  tmpMap.initialize(width, height);

  outputMap.initialize(width, height);

  for (int rowIndex = spreadingSize; rowIndex < height-spreadingSize-1; ++rowIndex)
  {
    for (int colIndex = spreadingSize; colIndex < width-spreadingSize-1; ++colIndex)
    {
      unsigned char value = 0;
      for (int colIndex2 = colIndex-spreadingSize; colIndex2 <= colIndex+spreadingSize; ++colIndex2)
      {
        //if (rowIndex2 < 0 || rowIndex2 >= height || colIndex2 < 0 || colIndex2 >= width) continue;
        value |= inputMap (colIndex2, rowIndex);
      }
      tmpMap (colIndex, rowIndex) = value;
    }
  }

  for (int rowIndex = spreadingSize; rowIndex < height-spreadingSize-1; ++rowIndex)
  {
    for (int colIndex = spreadingSize; colIndex < width-spreadingSize-1; ++colIndex)
    {
      unsigned char value = 0;
      for (int rowIndex2 = rowIndex-spreadingSize; rowIndex2 <= rowIndex+spreadingSize; ++rowIndex2)
      {
        //if (rowIndex2 < 0 || rowIndex2 >= height || colIndex2 < 0 || colIndex2 >= width) continue;
        value |= tmpMap (colIndex, rowIndex2);
      }
      outputMap (colIndex, rowIndex) = value;
    }
  }
}
