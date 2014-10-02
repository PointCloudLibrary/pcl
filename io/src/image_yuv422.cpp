/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011 2011 Willow Garage, Inc.
 *    Suat Gedikli <gedikli@willowgarage.com>
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
#include <pcl/pcl_config.h>
#include <pcl/io/image_yuv422.h>

#include <pcl/io/io_exception.h>

#include <sstream>
#include <iostream>

#define CLIP_CHAR(c) static_cast<unsigned char> ((c)>255?255:(c)<0?0:(c))

using pcl::io::FrameWrapper;
using pcl::io::IOException;

pcl::io::ImageYUV422::ImageYUV422 (FrameWrapper::Ptr image_metadata)
  : Image (image_metadata)
{}


pcl::io::ImageYUV422::ImageYUV422 (FrameWrapper::Ptr image_metadata, Timestamp timestamp)
  : Image (image_metadata, timestamp)
{}


pcl::io::ImageYUV422::~ImageYUV422 () throw ()
{}

bool
pcl::io::ImageYUV422::isResizingSupported (unsigned input_width, unsigned input_height, unsigned output_width, unsigned output_height) const
{
  return (output_width <= input_width && output_height <= input_height && input_width % output_width == 0 && input_height % output_height == 0 );
}


void
pcl::io::ImageYUV422::fillRGB (unsigned width, unsigned height, unsigned char* rgb_buffer, unsigned rgb_line_step) const
{
  // 0  1   2  3
  // u  y1  v  y2

  if (wrapper_->getWidth () != width && wrapper_->getHeight () != height)
  {
    if (width > wrapper_->getWidth () || height > wrapper_->getHeight () )
      THROW_IO_EXCEPTION ("Upsampling not supported. Request was: %d x %d -> %d x %d", wrapper_->getWidth (), wrapper_->getHeight (), width, height);

    if ( wrapper_->getWidth () % width != 0 || wrapper_->getHeight () % height != 0
      || (wrapper_->getWidth () / width) & 0x01 || (wrapper_->getHeight () / height & 0x01) )
      THROW_IO_EXCEPTION ("Downsampling only possible for power of two scale in both dimensions. Request was %d x %d -> %d x %d.", wrapper_->getWidth (), wrapper_->getHeight (), width, height);
  }

  register const uint8_t* yuv_buffer = (uint8_t*) wrapper_->getData ();

  unsigned rgb_line_skip = 0;
  if (rgb_line_step != 0)
    rgb_line_skip = rgb_line_step - width * 3;

  if (wrapper_->getWidth () == width && wrapper_->getHeight () == height)
  {
    for ( register unsigned yIdx = 0; yIdx < height; ++yIdx, rgb_buffer += rgb_line_skip )
    {
      for ( register unsigned xIdx = 0; xIdx < width; xIdx += 2, rgb_buffer += 6, yuv_buffer += 4 )
      {
        int v = yuv_buffer[2] - 128;
        int u = yuv_buffer[0] - 128;

        rgb_buffer[0] =  CLIP_CHAR (yuv_buffer[1] + ((v * 18678 + 8192 ) >> 14));
        rgb_buffer[1] =  CLIP_CHAR (yuv_buffer[1] + ((v * -9519 - u * 6472 + 8192 ) >> 14));
        rgb_buffer[2] =  CLIP_CHAR (yuv_buffer[1] + ((u * 33292 + 8192 ) >> 14));

        rgb_buffer[3] =  CLIP_CHAR (yuv_buffer[3] + ((v * 18678 + 8192 ) >> 14));
        rgb_buffer[4] =  CLIP_CHAR (yuv_buffer[3] + ((v * -9519 - u * 6472 + 8192 ) >> 14));
        rgb_buffer[5] =  CLIP_CHAR (yuv_buffer[3] + ((u * 33292 + 8192 ) >> 14));
      }
    }
  }
  else
  {
    register unsigned yuv_step = wrapper_->getWidth () / width;
    register unsigned yuv_x_step = yuv_step << 1;
    register unsigned yuv_skip = (wrapper_->getHeight () / height - 1) * ( wrapper_->getWidth () << 1 );

    for ( register unsigned yIdx = 0; yIdx < wrapper_->getHeight (); yIdx += yuv_step, yuv_buffer += yuv_skip, rgb_buffer += rgb_line_skip )
    {
      for ( register unsigned xIdx = 0; xIdx < wrapper_->getWidth (); xIdx += yuv_step, rgb_buffer += 3, yuv_buffer += yuv_x_step )
      {
        int v = yuv_buffer[2] - 128;
        int u = yuv_buffer[0] - 128;

        rgb_buffer[0] =  CLIP_CHAR (yuv_buffer[1] + ((v * 18678 + 8192 ) >> 14));
        rgb_buffer[1] =  CLIP_CHAR (yuv_buffer[1] + ((v * -9519 - u * 6472 + 8192 ) >> 14));
        rgb_buffer[2] =  CLIP_CHAR (yuv_buffer[1] + ((u * 33292 + 8192 ) >> 14));
      }
    }
  }
}


void
pcl::io::ImageYUV422::fillGrayscale (unsigned width, unsigned height, unsigned char* gray_buffer, unsigned gray_line_step) const
{
  // u y1 v y2
  if (width > wrapper_->getWidth () || height > wrapper_->getHeight ())
    THROW_IO_EXCEPTION ("Upsampling not supported. Request was: %d x %d -> %d x %d", wrapper_->getWidth (), wrapper_->getHeight (), width, height);

  if (wrapper_->getWidth () % width != 0 || wrapper_->getHeight () % height != 0)
    THROW_IO_EXCEPTION ("Downsampling only possible for integer scales in both dimensions. Request was %d x %d -> %d x %d.", wrapper_->getWidth (), wrapper_->getHeight (), width, height);

  unsigned gray_line_skip = 0;
  if (gray_line_step != 0)
    gray_line_skip = gray_line_step - width;

  register unsigned yuv_step = wrapper_->getWidth () / width;
  register unsigned yuv_x_step = yuv_step << 1;
  register unsigned yuv_skip = (wrapper_->getHeight () / height - 1) * ( wrapper_->getWidth () << 1 );
  register const uint8_t* yuv_buffer = ( (uint8_t*) wrapper_->getData () + 1);

  for ( register unsigned yIdx = 0; yIdx < wrapper_->getHeight (); yIdx += yuv_step, yuv_buffer += yuv_skip, gray_buffer += gray_line_skip )
  {
    for ( register unsigned xIdx = 0; xIdx < wrapper_->getWidth (); xIdx += yuv_step, ++gray_buffer, yuv_buffer += yuv_x_step )
    {
      *gray_buffer = *yuv_buffer;
    }
  }
}

