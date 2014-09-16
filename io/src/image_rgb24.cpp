/*
 * Software License Agreement (BSD License)
 * 
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2009-2012, Willow Garage, Inc.
 * Copyright (c) 2012-, Open Perception, Inc.
 * Copyright (c) 2014, respective authors.
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
 */

#include <pcl/pcl_config.h>
#include <pcl/io/image_rgb24.h>

#include <pcl/io/io_exception.h>

using pcl::io::FrameWrapper;
using pcl::io::IOException;

pcl::io::ImageRGB24::ImageRGB24 (FrameWrapper::Ptr image_metadata)
  : Image (image_metadata)
{}


pcl::io::ImageRGB24::ImageRGB24 (FrameWrapper::Ptr image_metadata, Timestamp timestamp)
  : Image (image_metadata, timestamp)
{}


pcl::io::ImageRGB24::~ImageRGB24 () throw ()
{}

bool
pcl::io::ImageRGB24::isResizingSupported (unsigned input_width, unsigned input_height, unsigned output_width, unsigned output_height) const
{
  return (output_width <= input_width && output_height <= input_height && input_width % output_width == 0 && input_height % output_height == 0 );
}


void
pcl::io::ImageRGB24::fillGrayscale (unsigned width, unsigned height, unsigned char* gray_buffer, unsigned gray_line_step) const
{
  if (width > wrapper_->getWidth () || height > wrapper_->getHeight ())
    THROW_IO_EXCEPTION ("Up-sampling not supported. Request was %d x %d -> %d x %d.", wrapper_->getWidth (), wrapper_->getHeight (), width, height);

  if (wrapper_->getWidth () % width == 0 && wrapper_->getHeight () % height == 0)
  {
    unsigned src_step = wrapper_->getWidth () / width;
    unsigned src_skip = (wrapper_->getHeight () / height - 1) * wrapper_->getWidth ();

    if (gray_line_step == 0)
      gray_line_step = width;

    unsigned dst_skip = gray_line_step - width; // skip of padding values in bytes

    unsigned char* dst_line = gray_buffer;
    const RGB888Pixel* src_line = (RGB888Pixel*) wrapper_->getData ();

    for (unsigned yIdx = 0; yIdx < height; ++yIdx, src_line += src_skip, dst_line += dst_skip)
    {
      for (unsigned xIdx = 0; xIdx < width; ++xIdx, src_line += src_step, dst_line ++)
      {
        *dst_line = static_cast<unsigned char>((static_cast<int> (src_line->r)   * 299 +
          static_cast<int> (src_line->g) * 587 +
          static_cast<int> (src_line->b)  * 114) * 0.001);
      }
    }
  }
  else
  {
    THROW_IO_EXCEPTION ("Down-sampling only possible for integer scale. Request was %d x %d -> %d x %d.", wrapper_->getWidth (), wrapper_->getHeight (), width, height);
  }
}


void
pcl::io::ImageRGB24::fillRGB (unsigned width, unsigned height, unsigned char* rgb_buffer, unsigned rgb_line_step) const
{
  if (width > wrapper_->getWidth () || height > wrapper_->getHeight ())
    THROW_IO_EXCEPTION ("Up-sampling not supported. Request was %d x %d -> %d x %d.", wrapper_->getWidth (), wrapper_->getHeight (), width, height);

  if (width == wrapper_->getWidth () && height == wrapper_->getHeight ())
  {
    unsigned line_size = width * 3;
    if (rgb_line_step == 0 || rgb_line_step == line_size)
    {
      memcpy (rgb_buffer, wrapper_->getData (), wrapper_->getDataSize ());
    }
    else // line by line
    {
      unsigned char* rgb_line = rgb_buffer;
      const unsigned char* src_line = static_cast<const unsigned char*> (wrapper_->getData ());
      for (unsigned yIdx = 0; yIdx < height; ++yIdx, rgb_line += rgb_line_step, src_line += line_size)
      {
        memcpy (rgb_line, src_line, line_size);
      }
    }
  }
  else if (wrapper_->getWidth () % width == 0 && wrapper_->getHeight () % height == 0) // downsamplig
  {
    unsigned src_step = wrapper_->getWidth () / width;
    unsigned src_skip = (wrapper_->getHeight () / height - 1) * wrapper_->getWidth ();

    if (rgb_line_step == 0)
      rgb_line_step = width * 3;

    unsigned dst_skip = rgb_line_step - width * 3; // skip of padding values in bytes

    RGB888Pixel* dst_line = reinterpret_cast<RGB888Pixel*> (rgb_buffer);
    const RGB888Pixel* src_line = (RGB888Pixel*) wrapper_->getData ();

    for (unsigned yIdx = 0; yIdx < height; ++yIdx, src_line += src_skip)
    {
      for (unsigned xIdx = 0; xIdx < width; ++xIdx, src_line += src_step, dst_line ++)
      {
        *dst_line = *src_line;
      }

      if (dst_skip != 0)
      {
        // use bytes to skip rather than XnRGB24Pixel's, since line_step does not need to be multiple of 3
        unsigned char* temp = reinterpret_cast <unsigned char*> (dst_line);
        dst_line = reinterpret_cast <RGB888Pixel*> (temp + dst_skip);
      }
    }
  }
  else
  {
    THROW_IO_EXCEPTION ("Down-sampling only possible for integer scale. Request was %d x %d -> %d x %d.", wrapper_->getWidth (), wrapper_->getHeight (), width, height);
  }
}

