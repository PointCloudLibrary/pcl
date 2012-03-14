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
#ifdef HAVE_OPENNI

#include <pcl/io/openni_camera/openni_image_yuv_422.h>
#include <sstream>
#include <iostream>

#define CLIP_CHAR(c) static_cast<unsigned char> ((c)>255?255:(c)<0?0:(c))

using namespace std;
namespace openni_wrapper
{

ImageYUV422::ImageYUV422 (boost::shared_ptr<xn::ImageMetaData> image_meta_data) throw ()
: Image (image_meta_data)
{
}

ImageYUV422::~ImageYUV422 () throw ()
{
}

bool ImageYUV422::isResizingSupported (unsigned input_width, unsigned input_height, unsigned output_width, unsigned output_height) const
{
  return ImageYUV422::resizingSupported (input_width, input_height, output_width, output_height);
}

void ImageYUV422::fillRGB (unsigned width, unsigned height, unsigned char* rgb_buffer, unsigned rgb_line_step) const
{
  // 0  1   2  3
  // u  y1  v  y2

  if (image_md_->XRes() != width && image_md_->YRes() != height)
  {
    if (width > image_md_->XRes () || height > image_md_->YRes ())
      THROW_OPENNI_EXCEPTION ("Upsampling not supported. Request was: %d x %d -> %d x %d", image_md_->XRes (), image_md_->YRes (), width, height);

    if ( image_md_->XRes () % width != 0 || image_md_->YRes () % height != 0
        || (image_md_->XRes () / width) & 0x01 || (image_md_->YRes () / height & 0x01) )
        THROW_OPENNI_EXCEPTION ("Downsampling only possible for power of two scale in both dimensions. Request was %d x %d -> %d x %d.", image_md_->XRes (), image_md_->YRes (), width, height);
  }

  register const XnUInt8* yuv_buffer = image_md_->Data();

  unsigned rgb_line_skip = 0;
  if (rgb_line_step != 0)
    rgb_line_skip = rgb_line_step - width * 3;

  if (image_md_->XRes() == width && image_md_->YRes() == height)
  {
    for( register unsigned yIdx = 0; yIdx < height; ++yIdx, rgb_buffer += rgb_line_skip )
    {
      for( register unsigned xIdx = 0; xIdx < width; xIdx += 2, rgb_buffer += 6, yuv_buffer += 4 )
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
    register unsigned yuv_step = image_md_->XRes() / width;
    register unsigned yuv_x_step = yuv_step << 1;
    register unsigned yuv_skip = (image_md_->YRes() / height - 1) * ( image_md_->XRes() << 1 );

    for( register unsigned yIdx = 0; yIdx < image_md_->YRes(); yIdx += yuv_step, yuv_buffer += yuv_skip, rgb_buffer += rgb_line_skip )
    {
      for( register unsigned xIdx = 0; xIdx < image_md_->XRes(); xIdx += yuv_step, rgb_buffer += 3, yuv_buffer += yuv_x_step )
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

void ImageYUV422::fillGrayscale (unsigned width, unsigned height, unsigned char* gray_buffer, unsigned gray_line_step) const
{
  // u y1 v y2
  if (width > image_md_->XRes () || height > image_md_->YRes ())
    THROW_OPENNI_EXCEPTION ("Upsampling not supported. Request was: %d x %d -> %d x %d", image_md_->XRes (), image_md_->YRes (), width, height);

  if (image_md_->XRes () % width != 0 || image_md_->YRes () % height != 0)
      THROW_OPENNI_EXCEPTION ("Downsampling only possible for integer scales in both dimensions. Request was %d x %d -> %d x %d.", image_md_->XRes (), image_md_->YRes (), width, height);

  unsigned gray_line_skip = 0;
  if (gray_line_step != 0)
    gray_line_skip = gray_line_step - width;

  register unsigned yuv_step = image_md_->XRes() / width;
  register unsigned yuv_x_step = yuv_step << 1;
  register unsigned yuv_skip = (image_md_->YRes() / height - 1) * ( image_md_->XRes() << 1 );
  register const XnUInt8* yuv_buffer = (image_md_->Data() + 1);

  for( register unsigned yIdx = 0; yIdx < image_md_->YRes(); yIdx += yuv_step, yuv_buffer += yuv_skip, gray_buffer += gray_line_skip )
  {
    for( register unsigned xIdx = 0; xIdx < image_md_->XRes(); xIdx += yuv_step, ++gray_buffer, yuv_buffer += yuv_x_step )
    {
      *gray_buffer = *yuv_buffer;
    }
  }
}
}//namespace
#endif
