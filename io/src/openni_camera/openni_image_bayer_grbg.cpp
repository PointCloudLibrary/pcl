/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2011, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
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
#include <pcl/memory.h>
#ifdef HAVE_OPENNI

#include <pcl/io/openni_camera/openni_image_bayer_grbg.h>
#include <pcl/io/debayer.h>

#define AVG(a,b) static_cast<unsigned char>((int(a) + int(b)) >> 1)
#define AVG3(a,b,c) static_cast<unsigned char>((int(a) + int(b) + int(c)) / 3)
#define AVG4(a,b,c,d) static_cast<unsigned char>((int(a) + int(b) + int(c) + int(d)) >> 2)
#define WAVG4(a,b,c,d,x,y) static_cast<unsigned char>( ( (int(a) + int(b)) * int(x) + (int(c) + int(d)) * int(y) ) / ( (int(x) + (int(y))) << 1 ) )

//////////////////////////////////////////////////////////////////////////////
openni_wrapper::ImageBayerGRBG::ImageBayerGRBG (pcl::shared_ptr<xn::ImageMetaData> image_meta_data, DebayeringMethod method) noexcept
: Image (std::move(image_meta_data))
, debayering_method_ (method)
{
}

//////////////////////////////////////////////////////////////////////////////
openni_wrapper::ImageBayerGRBG::~ImageBayerGRBG () noexcept = default;

//////////////////////////////////////////////////////////////////////////////
bool 
openni_wrapper::ImageBayerGRBG::isResizingSupported (
    unsigned input_width, unsigned input_height, 
    unsigned output_width, unsigned output_height) const
{
  return (openni_wrapper::ImageBayerGRBG::resizingSupported (input_width, input_height, output_width, output_height));
}

//////////////////////////////////////////////////////////////////////////////
void 
openni_wrapper::ImageBayerGRBG::fillGrayscale (
    unsigned width, unsigned height, 
    unsigned char* gray_buffer, unsigned gray_line_step) const
{
  if (width > image_md_->XRes () || height > image_md_->YRes ())
    THROW_OPENNI_EXCEPTION ("Upsampling not supported. Request was: %d x %d -> %d x %d", image_md_->XRes (), image_md_->YRes (), width, height);

  if (gray_line_step == 0)
    gray_line_step = width;

  // padding skip for destination image
  unsigned gray_line_skip = gray_line_step - width;
  if (image_md_->XRes () == width && image_md_->YRes () == height)
  { // if no downsampling
    const XnUInt8 *bayer_pixel = image_md_->Data ();
    int line_skip = image_md_->XRes ();
    if (debayering_method_ == Bilinear)
    {
      // first line GRGRGR
      for (unsigned xIdx = 0; xIdx < width - 2; xIdx += 2, gray_buffer += 2, bayer_pixel += 2)
      {
        gray_buffer[0] = bayer_pixel[0]; // green pixel
        gray_buffer[1] = AVG3 (bayer_pixel[0], bayer_pixel[2], bayer_pixel[1 + line_skip]); // interpolated green pixel
      }
      gray_buffer[0] = bayer_pixel[0];
      gray_buffer[1] = AVG (bayer_pixel[0], bayer_pixel[1 + line_skip]);
      gray_buffer += 2 + gray_line_skip;
      bayer_pixel += 2;

      for (unsigned yIdx = 1; yIdx < height - 1; yIdx += 2)
      {
        // blue line
        gray_buffer[0] = AVG3 (bayer_pixel[-line_skip], bayer_pixel[line_skip], bayer_pixel[1]);
        gray_buffer[1] = bayer_pixel[1];
        gray_buffer += 2;
        bayer_pixel += 2;
        for (unsigned xIdx = 2; xIdx < width; xIdx += 2, gray_buffer += 2, bayer_pixel += 2)
        {
          gray_buffer[0] = AVG4 (bayer_pixel[-line_skip], bayer_pixel[line_skip], bayer_pixel[-1], bayer_pixel[1]);
          gray_buffer[1] = bayer_pixel[1];
        }
        gray_buffer += gray_line_skip;

        // red line
        for (unsigned xIdx = 0; xIdx < width - 2; xIdx += 2, gray_buffer += 2, bayer_pixel += 2)
        {
          gray_buffer[0] = bayer_pixel[0]; // green pixel
          gray_buffer[1] = AVG4 (bayer_pixel[0], bayer_pixel[2], bayer_pixel[-line_skip + 1], bayer_pixel[line_skip + 1]); // interpolated green pixel
        }
        gray_buffer[0] = bayer_pixel[0];
        gray_buffer[1] = AVG3 (bayer_pixel[-line_skip + 1], bayer_pixel[line_skip + 1], bayer_pixel[-1]);
        gray_buffer += 2 + gray_line_skip;
        bayer_pixel += 2;
      }

      // last line BGBGBG
      gray_buffer[0] = AVG (bayer_pixel[1], bayer_pixel[-line_skip]);
      gray_buffer[1] = bayer_pixel[1];
      gray_buffer += 2;
      bayer_pixel += 2;
      for (unsigned xIdx = 2; xIdx < width; xIdx += 2, gray_buffer += 2, bayer_pixel += 2)
      {
        gray_buffer[0] = AVG3 (bayer_pixel[-1], bayer_pixel[1], bayer_pixel[-line_skip]);
        gray_buffer[1] = bayer_pixel[1];
      }
    }
    else if (debayering_method_ == EdgeAware)
    {
      int dv, dh;
      // first line GRGRGR
      for (unsigned xIdx = 0; xIdx < width - 2; xIdx += 2, gray_buffer += 2, bayer_pixel += 2)
      {
        gray_buffer[0] = bayer_pixel[0]; // green pixel
        gray_buffer[1] = AVG3 (bayer_pixel[0], bayer_pixel[2], bayer_pixel[1 + line_skip]); // interpolated green pixel
      }
      gray_buffer[0] = bayer_pixel[0];
      gray_buffer[1] = AVG (bayer_pixel[0], bayer_pixel[1 + line_skip]);
      gray_buffer += 2 + gray_line_skip;
      bayer_pixel += 2;

      for (unsigned yIdx = 1; yIdx < height - 1; yIdx += 2)
      {
        // blue line
        gray_buffer[0] = AVG3 (bayer_pixel[-line_skip], bayer_pixel[line_skip], bayer_pixel[1]);
        gray_buffer[1] = bayer_pixel[1];
        gray_buffer += 2;
        bayer_pixel += 2;
        for (unsigned xIdx = 2; xIdx < width; xIdx += 2, gray_buffer += 2, bayer_pixel += 2)
        {
          dv = std::abs (bayer_pixel[-line_skip] - bayer_pixel[line_skip]);
          dh = std::abs (bayer_pixel[-1] - bayer_pixel[1]);
          if (dh > dv)
            gray_buffer[0] = AVG (bayer_pixel[-line_skip], bayer_pixel[line_skip]);
          else if (dv > dh)
            gray_buffer[0] = AVG (bayer_pixel[-1], bayer_pixel[1]);
          else
            gray_buffer[0] = AVG4 (bayer_pixel[-line_skip], bayer_pixel[line_skip], bayer_pixel[-1], bayer_pixel[1]);

          gray_buffer[1] = bayer_pixel[1];
        }
        gray_buffer += gray_line_skip;

        // red line
        for (unsigned xIdx = 0; xIdx < width - 2; xIdx += 2, gray_buffer += 2, bayer_pixel += 2)
        {
          gray_buffer[0] = bayer_pixel[0];

          dv = std::abs (bayer_pixel[1 - line_skip] - bayer_pixel[1 + line_skip]);
          dh = std::abs (bayer_pixel[0] - bayer_pixel[2]);
          if (dh > dv)
            gray_buffer[1] = AVG (bayer_pixel[1 - line_skip], bayer_pixel[1 + line_skip]);
          else if (dv > dh)
            gray_buffer[1] = AVG (bayer_pixel[0], bayer_pixel[2]);
          else
            gray_buffer[1] = AVG4 (bayer_pixel[0], bayer_pixel[2], bayer_pixel[-line_skip + 1], bayer_pixel[line_skip + 1]);
        }
        gray_buffer[0] = bayer_pixel[0];
        gray_buffer[1] = AVG3 (bayer_pixel[-line_skip + 1], bayer_pixel[line_skip + 1], bayer_pixel[-1]);
        gray_buffer += 2 + gray_line_skip;
        bayer_pixel += 2;
      }

      // last line BGBGBG
      gray_buffer[0] = AVG (bayer_pixel[1], bayer_pixel[-line_skip]);
      gray_buffer[1] = bayer_pixel[1];
      gray_buffer += 2;
      bayer_pixel += 2;
      for (unsigned xIdx = 2; xIdx < width; xIdx += 2, gray_buffer += 2, bayer_pixel += 2)
      {
        gray_buffer[0] = AVG3 (bayer_pixel[-1], bayer_pixel[1], bayer_pixel[-line_skip]);
        gray_buffer[1] = bayer_pixel[1];
      }
    }
    else if (debayering_method_ == EdgeAwareWeighted)
    {
      int dv, dh;
      // first line GRGRGR
      for (unsigned xIdx = 0; xIdx < width - 2; xIdx += 2, gray_buffer += 2, bayer_pixel += 2)
      {
        gray_buffer[0] = bayer_pixel[0]; // green pixel
        gray_buffer[1] = AVG3 (bayer_pixel[0], bayer_pixel[2], bayer_pixel[1 + line_skip]); // interpolated green pixel
      }
      gray_buffer[0] = bayer_pixel[0];
      gray_buffer[1] = AVG (bayer_pixel[0], bayer_pixel[1 + line_skip]);
      gray_buffer += 2 + gray_line_skip;
      bayer_pixel += 2;

      for (unsigned yIdx = 1; yIdx < height - 1; yIdx += 2)
      {
        // blue line
        gray_buffer[0] = AVG3 (bayer_pixel[-line_skip], bayer_pixel[line_skip], bayer_pixel[1]);
        gray_buffer[1] = bayer_pixel[1];
        gray_buffer += 2;
        bayer_pixel += 2;
        for (unsigned xIdx = 2; xIdx < width; xIdx += 2, gray_buffer += 2, bayer_pixel += 2)
        {
          dv = std::abs (bayer_pixel[-line_skip] - bayer_pixel[line_skip]);
          dh = std::abs (bayer_pixel[-1] - bayer_pixel[1]);

          if (dv == 0 && dh == 0)
            gray_buffer[0] = AVG4 (bayer_pixel[-line_skip], bayer_pixel[line_skip], bayer_pixel[-1], bayer_pixel[1]);
          else
            gray_buffer[0] = WAVG4 (bayer_pixel[-line_skip], bayer_pixel[line_skip], bayer_pixel[-1], bayer_pixel[1], dh, dv);

          gray_buffer[1] = bayer_pixel[1];
        }

        gray_buffer += gray_line_skip;

        // red line
        for (unsigned xIdx = 0; xIdx < width - 2; xIdx += 2, gray_buffer += 2, bayer_pixel += 2)
        {
          gray_buffer[0] = bayer_pixel[0];

          dv = std::abs (bayer_pixel[1 - line_skip] - bayer_pixel[1 + line_skip]);
          dh = std::abs (bayer_pixel[0] - bayer_pixel[2]);

          if (dv == 0 && dh == 0)
            gray_buffer[1] = AVG4 (bayer_pixel[1 - line_skip], bayer_pixel[1 + line_skip], bayer_pixel[0], bayer_pixel[2]);
          else
            gray_buffer[1] = WAVG4 (bayer_pixel[1 - line_skip], bayer_pixel[1 + line_skip], bayer_pixel[0], bayer_pixel[2], dh, dv);
        }
        gray_buffer[0] = bayer_pixel[0];
        gray_buffer[1] = AVG3 (bayer_pixel[-line_skip + 1], bayer_pixel[line_skip + 1], bayer_pixel[-1]);
        gray_buffer += 2 + gray_line_skip;
        bayer_pixel += 2;
      }

      // last line BGBGBG
      gray_buffer[0] = AVG (bayer_pixel[1], bayer_pixel[-line_skip]);
      gray_buffer[1] = bayer_pixel[1];
      gray_buffer += 2;
      bayer_pixel += 2;
      for (unsigned xIdx = 2; xIdx < width; xIdx += 2, gray_buffer += 2, bayer_pixel += 2)
      {
        gray_buffer[0] = AVG3 (bayer_pixel[-1], bayer_pixel[1], bayer_pixel[-line_skip]);
        gray_buffer[1] = bayer_pixel[1];
      }
    }
    else
      THROW_OPENNI_EXCEPTION ("Unknown Debayering method: %d", debayering_method_);

  }
  else // downsampling
  {
    if ((image_md_->XRes () >> 1) % width != 0 || (image_md_->YRes () >> 1) % height != 0)
      THROW_OPENNI_EXCEPTION ("Downsampling only possible for multiple of 2 in both dimensions. Request was %d x %d -> %d x %d.", image_md_->XRes (), image_md_->YRes (), width, height);

    // fast method -> simply takes each or each 2nd pixel-group to get gray values out
    unsigned bayerXStep = image_md_->XRes () / width;
    unsigned bayerYSkip = (image_md_->YRes () / height - 1) * image_md_->XRes ();

    // Downsampling and debayering at once
    const XnUInt8* bayer_buffer = image_md_->Data ();

    for (unsigned yIdx = 0; yIdx < height; ++yIdx, bayer_buffer += bayerYSkip, gray_buffer += gray_line_skip) // skip a line
    {
      for (unsigned xIdx = 0; xIdx < width; ++xIdx, ++gray_buffer, bayer_buffer += bayerXStep)
      {
        *gray_buffer = AVG (bayer_buffer[0], bayer_buffer[ image_md_->XRes () + 1]);
      }
    }
  } // downsampling
}

//////////////////////////////////////////////////////////////////////////////
void 
openni_wrapper::ImageBayerGRBG::fillRGB (
    unsigned width, unsigned height, 
    unsigned char* rgb_buffer, unsigned rgb_line_step) const
{
  if (width > image_md_->XRes () || height > image_md_->YRes ())
    THROW_OPENNI_EXCEPTION ("Upsampling only possible for multiple of 2 in both dimensions. Request was %d x %d -> %d x %d.", image_md_->XRes (), image_md_->YRes (), width, height);

  if (rgb_line_step == 0)
    rgb_line_step = width * 3;

  // padding skip for destination image
  unsigned rgb_line_skip = rgb_line_step - width * 3;

  if (image_md_->XRes () == width && image_md_->YRes () == height)
  {
    const unsigned char* bayer_pixel = image_md_->Data ();

    int bayer_line_step = image_md_->XRes ();
    int bayer_line_step2 = image_md_->XRes () << 1;

    pcl::io::DeBayer d;
    if (debayering_method_ == Bilinear)
      d.debayerBilinear (bayer_pixel, rgb_buffer, width, height, bayer_line_step, bayer_line_step2, rgb_line_step);
    else if (debayering_method_ == EdgeAware)
      d.debayerEdgeAware (bayer_pixel, rgb_buffer, width, height, bayer_line_step, bayer_line_step2, rgb_line_step);
    else if (debayering_method_ == EdgeAwareWeighted)
      d.debayerEdgeAwareWeighted (bayer_pixel, rgb_buffer, width, height, bayer_line_step, bayer_line_step2, rgb_line_step);
    else
      THROW_OPENNI_EXCEPTION ("Unknown debayering method: %d", debayering_method_);
  }
  else
  {
    if (image_md_->XRes () % width != 0 || image_md_->YRes () % height != 0)
      THROW_OPENNI_EXCEPTION ("Downsampling only possible for integer scales in both dimensions. Request was %d x %d -> %d x %d.", image_md_->XRes (), image_md_->YRes (), width, height);

    // get each or each 2nd pixel group to find rgb values!
    unsigned bayerXStep = image_md_->XRes () / width;
    unsigned bayerYSkip = (image_md_->YRes () / height - 1) * image_md_->XRes ();

    // Downsampling and debayering at once
    const XnUInt8* bayer_buffer = image_md_->Data ();

    for (unsigned yIdx = 0; yIdx < height; ++yIdx, bayer_buffer += bayerYSkip, rgb_buffer += rgb_line_skip) // skip a line
    {
      for (unsigned xIdx = 0; xIdx < width; ++xIdx, rgb_buffer += 3, bayer_buffer += bayerXStep)
      {
        rgb_buffer[ 2 ] = bayer_buffer[ image_md_->XRes () ];
        rgb_buffer[ 1 ] = AVG (bayer_buffer[0], bayer_buffer[ image_md_->XRes () + 1]);
        rgb_buffer[ 0 ] = bayer_buffer[ 1 ];
      }
    }
  }
}

#endif
