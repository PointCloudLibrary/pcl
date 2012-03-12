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

#include <pcl/io/openni_camera/openni_image_bayer_grbg.h>
#include <sstream>
#include <iostream>

#define AVG(a,b) static_cast<unsigned char>((int(a) + int(b)) >> 1)
#define AVG3(a,b,c) static_cast<unsigned char>((int(a) + int(b) + int(c)) / 3)
#define AVG4(a,b,c,d) static_cast<unsigned char>((int(a) + int(b) + int(c) + int(d)) >> 2)
#define WAVG4(a,b,c,d,x,y) static_cast<unsigned char>( ( (int(a) + int(b)) * int(x) + (int(c) + int(d)) * int(y) ) / ( (int(x) + (int(y))) << 1 ) )
using namespace std;

namespace openni_wrapper
{

ImageBayerGRBG::ImageBayerGRBG (boost::shared_ptr<xn::ImageMetaData> image_meta_data, DebayeringMethod method) throw ()
: Image (image_meta_data)
, debayering_method_ (method)
{
}

ImageBayerGRBG::~ImageBayerGRBG () throw ()
{
}

bool ImageBayerGRBG::isResizingSupported (unsigned input_width, unsigned input_height, unsigned output_width, unsigned output_height) const
{
  return ImageBayerGRBG::resizingSupported (input_width, input_height, output_width, output_height);
}

void ImageBayerGRBG::fillGrayscale (unsigned width, unsigned height, unsigned char* gray_buffer, unsigned gray_line_step) const
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
      for (register unsigned xIdx = 0; xIdx < width - 2; xIdx += 2, gray_buffer += 2, bayer_pixel += 2)
      {
        gray_buffer[0] = bayer_pixel[0]; // green pixel
        gray_buffer[1] = AVG3 (bayer_pixel[0], bayer_pixel[2], bayer_pixel[1 + line_skip]); // interpolated green pixel
      }
      gray_buffer[0] = bayer_pixel[0];
      gray_buffer[1] = AVG (bayer_pixel[0], bayer_pixel[1 + line_skip]);
      gray_buffer += 2 + gray_line_skip;
      bayer_pixel += 2;

      for (register unsigned yIdx = 1; yIdx < height - 1; yIdx += 2)
      {
        // blue line
        gray_buffer[0] = AVG3 (bayer_pixel[-line_skip], bayer_pixel[line_skip], bayer_pixel[1]);
        gray_buffer[1] = bayer_pixel[1];
        gray_buffer += 2;
        bayer_pixel += 2;
        for (register unsigned xIdx = 2; xIdx < width; xIdx += 2, gray_buffer += 2, bayer_pixel += 2)
        {
          gray_buffer[0] = AVG4 (bayer_pixel[-line_skip], bayer_pixel[line_skip], bayer_pixel[-1], bayer_pixel[1]);
          gray_buffer[1] = bayer_pixel[1];
        }
        gray_buffer += gray_line_skip;

        // red line
        for (register unsigned xIdx = 0; xIdx < width - 2; xIdx += 2, gray_buffer += 2, bayer_pixel += 2)
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
      for (register unsigned xIdx = 2; xIdx < width; xIdx += 2, gray_buffer += 2, bayer_pixel += 2)
      {
        gray_buffer[0] = AVG3 (bayer_pixel[-1], bayer_pixel[1], bayer_pixel[-line_skip]);
        gray_buffer[1] = bayer_pixel[1];
      }
    }
    else if (debayering_method_ == EdgeAware)
    {
      int dv, dh;
      // first line GRGRGR
      for (register unsigned xIdx = 0; xIdx < width - 2; xIdx += 2, gray_buffer += 2, bayer_pixel += 2)
      {
        gray_buffer[0] = bayer_pixel[0]; // green pixel
        gray_buffer[1] = AVG3 (bayer_pixel[0], bayer_pixel[2], bayer_pixel[1 + line_skip]); // interpolated green pixel
      }
      gray_buffer[0] = bayer_pixel[0];
      gray_buffer[1] = AVG (bayer_pixel[0], bayer_pixel[1 + line_skip]);
      gray_buffer += 2 + gray_line_skip;
      bayer_pixel += 2;

      for (register unsigned yIdx = 1; yIdx < height - 1; yIdx += 2)
      {
        // blue line
        gray_buffer[0] = AVG3 (bayer_pixel[-line_skip], bayer_pixel[line_skip], bayer_pixel[1]);
        gray_buffer[1] = bayer_pixel[1];
        gray_buffer += 2;
        bayer_pixel += 2;
        for (register unsigned xIdx = 2; xIdx < width; xIdx += 2, gray_buffer += 2, bayer_pixel += 2)
        {
          dv = abs (bayer_pixel[-line_skip] - bayer_pixel[line_skip]);
          dh = abs (bayer_pixel[-1] - bayer_pixel[1]);
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
        for (register unsigned xIdx = 0; xIdx < width - 2; xIdx += 2, gray_buffer += 2, bayer_pixel += 2)
        {
          gray_buffer[0] = bayer_pixel[0];

          dv = abs (bayer_pixel[1 - line_skip] - bayer_pixel[1 + line_skip]);
          dh = abs (bayer_pixel[0] - bayer_pixel[2]);
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
      for (register unsigned xIdx = 2; xIdx < width; xIdx += 2, gray_buffer += 2, bayer_pixel += 2)
      {
        gray_buffer[0] = AVG3 (bayer_pixel[-1], bayer_pixel[1], bayer_pixel[-line_skip]);
        gray_buffer[1] = bayer_pixel[1];
      }
    }
    else if (debayering_method_ == EdgeAwareWeighted)
    {
      int dv, dh;
      // first line GRGRGR
      for (register unsigned xIdx = 0; xIdx < width - 2; xIdx += 2, gray_buffer += 2, bayer_pixel += 2)
      {
        gray_buffer[0] = bayer_pixel[0]; // green pixel
        gray_buffer[1] = AVG3 (bayer_pixel[0], bayer_pixel[2], bayer_pixel[1 + line_skip]); // interpolated green pixel
      }
      gray_buffer[0] = bayer_pixel[0];
      gray_buffer[1] = AVG (bayer_pixel[0], bayer_pixel[1 + line_skip]);
      gray_buffer += 2 + gray_line_skip;
      bayer_pixel += 2;

      for (register unsigned yIdx = 1; yIdx < height - 1; yIdx += 2)
      {
        // blue line
        gray_buffer[0] = AVG3 (bayer_pixel[-line_skip], bayer_pixel[line_skip], bayer_pixel[1]);
        gray_buffer[1] = bayer_pixel[1];
        gray_buffer += 2;
        bayer_pixel += 2;
        for (register unsigned xIdx = 2; xIdx < width; xIdx += 2, gray_buffer += 2, bayer_pixel += 2)
        {
          dv = abs (bayer_pixel[-line_skip] - bayer_pixel[line_skip]);
          dh = abs (bayer_pixel[-1] - bayer_pixel[1]);

          if (dv == 0 && dh == 0)
            gray_buffer[0] = AVG4 (bayer_pixel[-line_skip], bayer_pixel[line_skip], bayer_pixel[-1], bayer_pixel[1]);
          else
            gray_buffer[0] = WAVG4 (bayer_pixel[-line_skip], bayer_pixel[line_skip], bayer_pixel[-1], bayer_pixel[1], dh, dv);

          gray_buffer[1] = bayer_pixel[1];
        }

        gray_buffer += gray_line_skip;

        // red line
        for (register unsigned xIdx = 0; xIdx < width - 2; xIdx += 2, gray_buffer += 2, bayer_pixel += 2)
        {
          gray_buffer[0] = bayer_pixel[0];

          dv = abs (bayer_pixel[1 - line_skip] - bayer_pixel[1 + line_skip]);
          dh = abs (bayer_pixel[0] - bayer_pixel[2]);

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
      for (register unsigned xIdx = 2; xIdx < width; xIdx += 2, gray_buffer += 2, bayer_pixel += 2)
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
    register unsigned bayerXStep = image_md_->XRes () / width;
    register unsigned bayerYSkip = (image_md_->YRes () / height - 1) * image_md_->XRes ();

    // Downsampling and debayering at once
    register const XnUInt8* bayer_buffer = image_md_->Data ();

    for (register unsigned yIdx = 0; yIdx < height; ++yIdx, bayer_buffer += bayerYSkip, gray_buffer += gray_line_skip) // skip a line
    {
      for (register unsigned xIdx = 0; xIdx < width; ++xIdx, ++gray_buffer, bayer_buffer += bayerXStep)
      {
        *gray_buffer = AVG (bayer_buffer[0], bayer_buffer[ image_md_->XRes () + 1]);
      }
    }
  } // downsampling
}

void ImageBayerGRBG::fillRGB (unsigned width, unsigned height, unsigned char* rgb_buffer, unsigned rgb_line_step) const
{
  if (width > image_md_->XRes () || height > image_md_->YRes ())
    THROW_OPENNI_EXCEPTION ("Upsampling only possible for multiple of 2 in both dimensions. Request was %d x %d -> %d x %d.", image_md_->XRes (), image_md_->YRes (), width, height);

  if (rgb_line_step == 0)
    rgb_line_step = width * 3;

  // padding skip for destination image
  unsigned rgb_line_skip = rgb_line_step - width * 3;

  if (image_md_->XRes () == width && image_md_->YRes () == height)
  {
    register const XnUInt8 *bayer_pixel = image_md_->Data ();
    register unsigned yIdx, xIdx;

    int bayer_line_step = image_md_->XRes ();
    int bayer_line_step2 = image_md_->XRes () << 1;

    if (debayering_method_ == Bilinear)
    {
      // first two pixel values for first two lines
      // Bayer         0 1 2
      //         0     G r g
      // line_step     b g b
      // line_step2    g r g

      rgb_buffer[3] = rgb_buffer[0] = bayer_pixel[1]; // red pixel
      rgb_buffer[1] = bayer_pixel[0]; // green pixel
      rgb_buffer[rgb_line_step + 2] = rgb_buffer[2] = bayer_pixel[bayer_line_step]; // blue;

      // Bayer         0 1 2
      //         0     g R g
      // line_step     b g b
      // line_step2    g r g
      //rgb_pixel[3] = bayer_pixel[1];
      rgb_buffer[4] = AVG3 (bayer_pixel[0], bayer_pixel[2], bayer_pixel[bayer_line_step + 1]);
      rgb_buffer[rgb_line_step + 5] = rgb_buffer[5] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2]);

      // BGBG line
      // Bayer         0 1 2
      //         0     g r g
      // line_step     B g b
      // line_step2    g r g
      rgb_buffer[rgb_line_step + 3] = rgb_buffer[rgb_line_step ] = AVG (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1]);
      rgb_buffer[rgb_line_step + 1] = AVG3 (bayer_pixel[0], bayer_pixel[bayer_line_step + 1], bayer_pixel[bayer_line_step2]);
      //rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

      // pixel (1, 1)  0 1 2
      //         0     g r g
      // line_step     b G b
      // line_step2    g r g
      //rgb_pixel[rgb_line_step + 3] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
      rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
      //rgb_pixel[rgb_line_step + 5] = AVG( bayer_pixel[line_step] , bayer_pixel[line_step+2] );

      rgb_buffer += 6;
      bayer_pixel += 2;
      // rest of the first two lines

      for (xIdx = 2; xIdx < width - 2; xIdx += 2, rgb_buffer += 6, bayer_pixel += 2)
      {
        // GRGR line
        // Bayer        -1 0 1 2
        //           0   r G r g
        //   line_step   g b g b
        // line_step2    r g r g
        rgb_buffer[0] = AVG (bayer_pixel[1], bayer_pixel[-1]);
        rgb_buffer[1] = bayer_pixel[0];
        rgb_buffer[2] = bayer_pixel[bayer_line_step + 1];

        // Bayer        -1 0 1 2
        //          0    r g R g
        //  line_step    g b g b
        // line_step2    r g r g
        rgb_buffer[3] = bayer_pixel[1];
        rgb_buffer[4] = AVG3 (bayer_pixel[0], bayer_pixel[2], bayer_pixel[bayer_line_step + 1]);
        rgb_buffer[rgb_line_step + 5] = rgb_buffer[5] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2]);

        // BGBG line
        // Bayer         -1 0 1 2
        //         0      r g r g
        // line_step      g B g b
        // line_step2     r g r g
        rgb_buffer[rgb_line_step ] = AVG4 (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1], bayer_pixel[-1], bayer_pixel[bayer_line_step2 - 1]);
        rgb_buffer[rgb_line_step + 1] = AVG4 (bayer_pixel[0], bayer_pixel[bayer_line_step2], bayer_pixel[bayer_line_step - 1], bayer_pixel[bayer_line_step + 1]);
        rgb_buffer[rgb_line_step + 2] = bayer_pixel[bayer_line_step];

        // Bayer         -1 0 1 2
        //         0      r g r g
        // line_step      g b G b
        // line_step2     r g r g
        rgb_buffer[rgb_line_step + 3] = AVG (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1]);
        rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
        //rgb_pixel[rgb_line_step + 5] = AVG( bayer_pixel[line_step] , bayer_pixel[line_step+2] );
      }

      // last two pixel values for first two lines
      // GRGR line
      // Bayer        -1 0 1
      //           0   r G r
      //   line_step   g b g
      // line_step2    r g r
      rgb_buffer[0] = AVG (bayer_pixel[1], bayer_pixel[-1]);
      rgb_buffer[1] = bayer_pixel[0];
      rgb_buffer[rgb_line_step + 5] = rgb_buffer[rgb_line_step + 2] = rgb_buffer[5] = rgb_buffer[2] = bayer_pixel[bayer_line_step];

      // Bayer        -1 0 1
      //          0    r g R
      //  line_step    g b g
      // line_step2    r g r
      rgb_buffer[3] = bayer_pixel[1];
      rgb_buffer[4] = AVG (bayer_pixel[0], bayer_pixel[bayer_line_step + 1]);
      //rgb_pixel[5] = bayer_pixel[line_step];

      // BGBG line
      // Bayer        -1 0 1
      //          0    r g r
      //  line_step    g B g
      // line_step2    r g r
      rgb_buffer[rgb_line_step ] = AVG4 (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1], bayer_pixel[-1], bayer_pixel[bayer_line_step2 - 1]);
      rgb_buffer[rgb_line_step + 1] = AVG4 (bayer_pixel[0], bayer_pixel[bayer_line_step2], bayer_pixel[bayer_line_step - 1], bayer_pixel[bayer_line_step + 1]);
      //rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

      // Bayer         -1 0 1
      //         0      r g r
      // line_step      g b G
      // line_step2     r g r
      rgb_buffer[rgb_line_step + 3] = AVG (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1]);
      rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
      //rgb_pixel[rgb_line_step + 5] = bayer_pixel[line_step];

      bayer_pixel += bayer_line_step + 2;
      rgb_buffer += rgb_line_step + 6 + rgb_line_skip;

      // main processing

      for (yIdx = 2; yIdx < height - 2; yIdx += 2)
      {
        // first two pixel values
        // Bayer         0 1 2
        //        -1     b g b
        //         0     G r g
        // line_step     b g b
        // line_step2    g r g

        rgb_buffer[3] = rgb_buffer[0] = bayer_pixel[1]; // red pixel
        rgb_buffer[1] = bayer_pixel[0]; // green pixel
        rgb_buffer[2] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[-bayer_line_step]); // blue;

        // Bayer         0 1 2
        //        -1     b g b
        //         0     g R g
        // line_step     b g b
        // line_step2    g r g
        //rgb_pixel[3] = bayer_pixel[1];
        rgb_buffer[4] = AVG4 (bayer_pixel[0], bayer_pixel[2], bayer_pixel[bayer_line_step + 1], bayer_pixel[1 - bayer_line_step]);
        rgb_buffer[5] = AVG4 (bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2], bayer_pixel[-bayer_line_step], bayer_pixel[2 - bayer_line_step]);

        // BGBG line
        // Bayer         0 1 2
        //         0     g r g
        // line_step     B g b
        // line_step2    g r g
        rgb_buffer[rgb_line_step + 3] = rgb_buffer[rgb_line_step ] = AVG (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1]);
        rgb_buffer[rgb_line_step + 1] = AVG3 (bayer_pixel[0], bayer_pixel[bayer_line_step + 1], bayer_pixel[bayer_line_step2]);
        rgb_buffer[rgb_line_step + 2] = bayer_pixel[bayer_line_step];

        // pixel (1, 1)  0 1 2
        //         0     g r g
        // line_step     b G b
        // line_step2    g r g
        //rgb_pixel[rgb_line_step + 3] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
        rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
        rgb_buffer[rgb_line_step + 5] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2]);

        rgb_buffer += 6;
        bayer_pixel += 2;
        // continue with rest of the line
        for (xIdx = 2; xIdx < width - 2; xIdx += 2, rgb_buffer += 6, bayer_pixel += 2)
        {
          // GRGR line
          // Bayer        -1 0 1 2
          //          -1   g b g b
          //           0   r G r g
          //   line_step   g b g b
          // line_step2    r g r g
          rgb_buffer[0] = AVG (bayer_pixel[1], bayer_pixel[-1]);
          rgb_buffer[1] = bayer_pixel[0];
          rgb_buffer[2] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[-bayer_line_step]);

          // Bayer        -1 0 1 2
          //          -1   g b g b
          //          0    r g R g
          //  line_step    g b g b
          // line_step2    r g r g
          rgb_buffer[3] = bayer_pixel[1];
          rgb_buffer[4] = AVG4 (bayer_pixel[0], bayer_pixel[2], bayer_pixel[bayer_line_step + 1], bayer_pixel[1 - bayer_line_step]);
          rgb_buffer[5] = AVG4 (bayer_pixel[-bayer_line_step], bayer_pixel[2 - bayer_line_step], bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2]);

          // BGBG line
          // Bayer         -1 0 1 2
          //         -1     g b g b
          //          0     r g r g
          // line_step      g B g b
          // line_step2     r g r g
          rgb_buffer[rgb_line_step ] = AVG4 (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1], bayer_pixel[-1], bayer_pixel[bayer_line_step2 - 1]);
          rgb_buffer[rgb_line_step + 1] = AVG4 (bayer_pixel[0], bayer_pixel[bayer_line_step2], bayer_pixel[bayer_line_step - 1], bayer_pixel[bayer_line_step + 1]);
          rgb_buffer[rgb_line_step + 2] = bayer_pixel[bayer_line_step];

          // Bayer         -1 0 1 2
          //         -1     g b g b
          //          0     r g r g
          // line_step      g b G b
          // line_step2     r g r g
          rgb_buffer[rgb_line_step + 3] = AVG (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1]);
          rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
          rgb_buffer[rgb_line_step + 5] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2]);
        }

        // last two pixels of the line
        // last two pixel values for first two lines
        // GRGR line
        // Bayer        -1 0 1
        //           0   r G r
        //   line_step   g b g
        // line_step2    r g r
        rgb_buffer[0] = AVG (bayer_pixel[1], bayer_pixel[-1]);
        rgb_buffer[1] = bayer_pixel[0];
        rgb_buffer[rgb_line_step + 5] = rgb_buffer[rgb_line_step + 2] = rgb_buffer[5] = rgb_buffer[2] = bayer_pixel[bayer_line_step];

        // Bayer        -1 0 1
        //          0    r g R
        //  line_step    g b g
        // line_step2    r g r
        rgb_buffer[3] = bayer_pixel[1];
        rgb_buffer[4] = AVG (bayer_pixel[0], bayer_pixel[bayer_line_step + 1]);
        //rgb_pixel[5] = bayer_pixel[line_step];

        // BGBG line
        // Bayer        -1 0 1
        //          0    r g r
        //  line_step    g B g
        // line_step2    r g r
        rgb_buffer[rgb_line_step ] = AVG4 (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1], bayer_pixel[-1], bayer_pixel[bayer_line_step2 - 1]);
        rgb_buffer[rgb_line_step + 1] = AVG4 (bayer_pixel[0], bayer_pixel[bayer_line_step2], bayer_pixel[bayer_line_step - 1], bayer_pixel[bayer_line_step + 1]);
        //rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

        // Bayer         -1 0 1
        //         0      r g r
        // line_step      g b G
        // line_step2     r g r
        rgb_buffer[rgb_line_step + 3] = AVG (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1]);
        rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
        //rgb_pixel[rgb_line_step + 5] = bayer_pixel[line_step];

        bayer_pixel += bayer_line_step + 2;
        rgb_buffer += rgb_line_step + 6 + rgb_line_skip;
      }

      //last two lines
      // Bayer         0 1 2
      //        -1     b g b
      //         0     G r g
      // line_step     b g b

      rgb_buffer[rgb_line_step + 3] = rgb_buffer[rgb_line_step ] = rgb_buffer[3] = rgb_buffer[0] = bayer_pixel[1]; // red pixel
      rgb_buffer[1] = bayer_pixel[0]; // green pixel
      rgb_buffer[rgb_line_step + 2] = rgb_buffer[2] = bayer_pixel[bayer_line_step]; // blue;

      // Bayer         0 1 2
      //        -1     b g b
      //         0     g R g
      // line_step     b g b
      //rgb_pixel[3] = bayer_pixel[1];
      rgb_buffer[4] = AVG4 (bayer_pixel[0], bayer_pixel[2], bayer_pixel[bayer_line_step + 1], bayer_pixel[1 - bayer_line_step]);
      rgb_buffer[5] = AVG4 (bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2], bayer_pixel[-bayer_line_step], bayer_pixel[2 - bayer_line_step]);

      // BGBG line
      // Bayer         0 1 2
      //        -1     b g b
      //         0     g r g
      // line_step     B g b
      //rgb_pixel[rgb_line_step    ] = bayer_pixel[1];
      rgb_buffer[rgb_line_step + 1] = AVG (bayer_pixel[0], bayer_pixel[bayer_line_step + 1]);
      rgb_buffer[rgb_line_step + 2] = bayer_pixel[bayer_line_step];

      // Bayer         0 1 2
      //        -1     b g b
      //         0     g r g
      // line_step     b G b
      //rgb_pixel[rgb_line_step + 3] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
      rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
      rgb_buffer[rgb_line_step + 5] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2]);

      rgb_buffer += 6;
      bayer_pixel += 2;
      // rest of the last two lines
      for (xIdx = 2; xIdx < width - 2; xIdx += 2, rgb_buffer += 6, bayer_pixel += 2)
      {
        // GRGR line
        // Bayer       -1 0 1 2
        //        -1    g b g b
        //         0    r G r g
        // line_step    g b g b
        rgb_buffer[0] = AVG (bayer_pixel[1], bayer_pixel[-1]);
        rgb_buffer[1] = bayer_pixel[0];
        rgb_buffer[2] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[-bayer_line_step]);

        // Bayer       -1 0 1 2
        //        -1    g b g b
        //         0    r g R g
        // line_step    g b g b
        rgb_buffer[rgb_line_step + 3] = rgb_buffer[3] = bayer_pixel[1];
        rgb_buffer[4] = AVG4 (bayer_pixel[0], bayer_pixel[2], bayer_pixel[bayer_line_step + 1], bayer_pixel[1 - bayer_line_step]);
        rgb_buffer[5] = AVG4 (bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2], bayer_pixel[-bayer_line_step], bayer_pixel[-bayer_line_step + 2]);

        // BGBG line
        // Bayer       -1 0 1 2
        //        -1    g b g b
        //         0    r g r g
        // line_step    g B g b
        rgb_buffer[rgb_line_step ] = AVG (bayer_pixel[-1], bayer_pixel[1]);
        rgb_buffer[rgb_line_step + 1] = AVG3 (bayer_pixel[0], bayer_pixel[bayer_line_step - 1], bayer_pixel[bayer_line_step + 1]);
        rgb_buffer[rgb_line_step + 2] = bayer_pixel[bayer_line_step];


        // Bayer       -1 0 1 2
        //        -1    g b g b
        //         0    r g r g
        // line_step    g b G b
        //rgb_pixel[rgb_line_step + 3] = bayer_pixel[1];
        rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
        rgb_buffer[rgb_line_step + 5] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2]);
      }

      // last two pixel values for first two lines
      // GRGR line
      // Bayer       -1 0 1
      //        -1    g b g
      //         0    r G r
      // line_step    g b g
      rgb_buffer[rgb_line_step ] = rgb_buffer[0] = AVG (bayer_pixel[1], bayer_pixel[-1]);
      rgb_buffer[1] = bayer_pixel[0];
      rgb_buffer[5] = rgb_buffer[2] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[-bayer_line_step]);

      // Bayer       -1 0 1
      //        -1    g b g
      //         0    r g R
      // line_step    g b g
      rgb_buffer[rgb_line_step + 3] = rgb_buffer[3] = bayer_pixel[1];
      rgb_buffer[4] = AVG3 (bayer_pixel[0], bayer_pixel[bayer_line_step + 1], bayer_pixel[-bayer_line_step + 1]);
      //rgb_pixel[5] = AVG( bayer_pixel[line_step], bayer_pixel[-line_step] );

      // BGBG line
      // Bayer       -1 0 1
      //        -1    g b g
      //         0    r g r
      // line_step    g B g
      //rgb_pixel[rgb_line_step    ] = AVG2( bayer_pixel[-1], bayer_pixel[1] );
      rgb_buffer[rgb_line_step + 1] = AVG3 (bayer_pixel[0], bayer_pixel[bayer_line_step - 1], bayer_pixel[bayer_line_step + 1]);
      rgb_buffer[rgb_line_step + 5] = rgb_buffer[rgb_line_step + 2] = bayer_pixel[bayer_line_step];

      // Bayer       -1 0 1
      //        -1    g b g
      //         0    r g r
      // line_step    g b G
      //rgb_pixel[rgb_line_step + 3] = bayer_pixel[1];
      rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
      //rgb_pixel[rgb_line_step + 5] = bayer_pixel[line_step];
    }
    else if (debayering_method_ == EdgeAware)
    {
      int dh, dv;

      // first two pixel values for first two lines
      // Bayer         0 1 2
      //         0     G r g
      // line_step     b g b
      // line_step2    g r g

      rgb_buffer[3] = rgb_buffer[0] = bayer_pixel[1]; // red pixel
      rgb_buffer[1] = bayer_pixel[0]; // green pixel
      rgb_buffer[rgb_line_step + 2] = rgb_buffer[2] = bayer_pixel[bayer_line_step]; // blue;

      // Bayer         0 1 2
      //         0     g R g
      // line_step     b g b
      // line_step2    g r g
      //rgb_pixel[3] = bayer_pixel[1];
      rgb_buffer[4] = AVG3 (bayer_pixel[0], bayer_pixel[2], bayer_pixel[bayer_line_step + 1]);
      rgb_buffer[rgb_line_step + 5] = rgb_buffer[5] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2]);

      // BGBG line
      // Bayer         0 1 2
      //         0     g r g
      // line_step     B g b
      // line_step2    g r g
      rgb_buffer[rgb_line_step + 3] = rgb_buffer[rgb_line_step ] = AVG (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1]);
      rgb_buffer[rgb_line_step + 1] = AVG3 (bayer_pixel[0], bayer_pixel[bayer_line_step + 1], bayer_pixel[bayer_line_step2]);
      //rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

      // pixel (1, 1)  0 1 2
      //         0     g r g
      // line_step     b G b
      // line_step2    g r g
      //rgb_pixel[rgb_line_step + 3] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
      rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
      //rgb_pixel[rgb_line_step + 5] = AVG( bayer_pixel[line_step] , bayer_pixel[line_step+2] );

      rgb_buffer += 6;
      bayer_pixel += 2;
      // rest of the first two lines
      for (xIdx = 2; xIdx < width - 2; xIdx += 2, rgb_buffer += 6, bayer_pixel += 2)
      {
        // GRGR line
        // Bayer        -1 0 1 2
        //           0   r G r g
        //   line_step   g b g b
        // line_step2    r g r g
        rgb_buffer[0] = AVG (bayer_pixel[1], bayer_pixel[-1]);
        rgb_buffer[1] = bayer_pixel[0];
        rgb_buffer[2] = bayer_pixel[bayer_line_step + 1];

        // Bayer        -1 0 1 2
        //          0    r g R g
        //  line_step    g b g b
        // line_step2    r g r g
        rgb_buffer[3] = bayer_pixel[1];
        rgb_buffer[4] = AVG3 (bayer_pixel[0], bayer_pixel[2], bayer_pixel[bayer_line_step + 1]);
        rgb_buffer[rgb_line_step + 5] = rgb_buffer[5] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2]);

        // BGBG line
        // Bayer         -1 0 1 2
        //         0      r g r g
        // line_step      g B g b
        // line_step2     r g r g
        rgb_buffer[rgb_line_step ] = AVG4 (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1], bayer_pixel[-1], bayer_pixel[bayer_line_step2 - 1]);
        rgb_buffer[rgb_line_step + 1] = AVG4 (bayer_pixel[0], bayer_pixel[bayer_line_step2], bayer_pixel[bayer_line_step - 1], bayer_pixel[bayer_line_step + 1]);
        rgb_buffer[rgb_line_step + 2] = bayer_pixel[bayer_line_step];

        // Bayer         -1 0 1 2
        //         0      r g r g
        // line_step      g b G b
        // line_step2     r g r g
        rgb_buffer[rgb_line_step + 3] = AVG (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1]);
        rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
        //rgb_pixel[rgb_line_step + 5] = AVG( bayer_pixel[line_step] , bayer_pixel[line_step+2] );
      }

      // last two pixel values for first two lines
      // GRGR line
      // Bayer        -1 0 1
      //           0   r G r
      //   line_step   g b g
      // line_step2    r g r
      rgb_buffer[0] = AVG (bayer_pixel[1], bayer_pixel[-1]);
      rgb_buffer[1] = bayer_pixel[0];
      rgb_buffer[rgb_line_step + 5] = rgb_buffer[rgb_line_step + 2] = rgb_buffer[5] = rgb_buffer[2] = bayer_pixel[bayer_line_step];

      // Bayer        -1 0 1
      //          0    r g R
      //  line_step    g b g
      // line_step2    r g r
      rgb_buffer[3] = bayer_pixel[1];
      rgb_buffer[4] = AVG (bayer_pixel[0], bayer_pixel[bayer_line_step + 1]);
      //rgb_pixel[5] = bayer_pixel[line_step];

      // BGBG line
      // Bayer        -1 0 1
      //          0    r g r
      //  line_step    g B g
      // line_step2    r g r
      rgb_buffer[rgb_line_step ] = AVG4 (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1], bayer_pixel[-1], bayer_pixel[bayer_line_step2 - 1]);
      rgb_buffer[rgb_line_step + 1] = AVG4 (bayer_pixel[0], bayer_pixel[bayer_line_step2], bayer_pixel[bayer_line_step - 1], bayer_pixel[bayer_line_step + 1]);
      //rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

      // Bayer         -1 0 1
      //         0      r g r
      // line_step      g b G
      // line_step2     r g r
      rgb_buffer[rgb_line_step + 3] = AVG (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1]);
      rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
      //rgb_pixel[rgb_line_step + 5] = bayer_pixel[line_step];

      bayer_pixel += bayer_line_step + 2;
      rgb_buffer += rgb_line_step + 6 + rgb_line_skip;
      // main processing
      for (yIdx = 2; yIdx < height - 2; yIdx += 2)
      {
        // first two pixel values
        // Bayer         0 1 2
        //        -1     b g b
        //         0     G r g
        // line_step     b g b
        // line_step2    g r g

        rgb_buffer[3] = rgb_buffer[0] = bayer_pixel[1]; // red pixel
        rgb_buffer[1] = bayer_pixel[0]; // green pixel
        rgb_buffer[2] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[-bayer_line_step]); // blue;

        // Bayer         0 1 2
        //        -1     b g b
        //         0     g R g
        // line_step     b g b
        // line_step2    g r g
        //rgb_pixel[3] = bayer_pixel[1];
        rgb_buffer[4] = AVG4 (bayer_pixel[0], bayer_pixel[2], bayer_pixel[bayer_line_step + 1], bayer_pixel[1 - bayer_line_step]);
        rgb_buffer[5] = AVG4 (bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2], bayer_pixel[-bayer_line_step], bayer_pixel[2 - bayer_line_step]);

        // BGBG line
        // Bayer         0 1 2
        //         0     g r g
        // line_step     B g b
        // line_step2    g r g
        rgb_buffer[rgb_line_step + 3] = rgb_buffer[rgb_line_step ] = AVG (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1]);
        rgb_buffer[rgb_line_step + 1] = AVG3 (bayer_pixel[0], bayer_pixel[bayer_line_step + 1], bayer_pixel[bayer_line_step2]);
        rgb_buffer[rgb_line_step + 2] = bayer_pixel[bayer_line_step];

        // pixel (1, 1)  0 1 2
        //         0     g r g
        // line_step     b G b
        // line_step2    g r g
        //rgb_pixel[rgb_line_step + 3] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
        rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
        rgb_buffer[rgb_line_step + 5] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2]);

        rgb_buffer += 6;
        bayer_pixel += 2;
        // continue with rest of the line
        for (xIdx = 2; xIdx < width - 2; xIdx += 2, rgb_buffer += 6, bayer_pixel += 2)
        {
          // GRGR line
          // Bayer        -1 0 1 2
          //          -1   g b g b
          //           0   r G r g
          //   line_step   g b g b
          // line_step2    r g r g
          rgb_buffer[0] = AVG (bayer_pixel[1], bayer_pixel[-1]);
          rgb_buffer[1] = bayer_pixel[0];
          rgb_buffer[2] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[-bayer_line_step]);

          // Bayer        -1 0 1 2
          //          -1   g b g b
          //          0    r g R g
          //  line_step    g b g b
          // line_step2    r g r g

          dh = abs (bayer_pixel[0] - bayer_pixel[2]);
          dv = abs (bayer_pixel[-bayer_line_step + 1] - bayer_pixel[bayer_line_step + 1]);

          if (dh > dv)
            rgb_buffer[4] = AVG (bayer_pixel[-bayer_line_step + 1], bayer_pixel[bayer_line_step + 1]);
          else if (dv > dh)
            rgb_buffer[4] = AVG (bayer_pixel[0], bayer_pixel[2]);
          else
            rgb_buffer[4] = AVG4 (bayer_pixel[-bayer_line_step + 1], bayer_pixel[bayer_line_step + 1], bayer_pixel[0], bayer_pixel[2]);

          rgb_buffer[3] = bayer_pixel[1];
          rgb_buffer[5] = AVG4 (bayer_pixel[-bayer_line_step], bayer_pixel[2 - bayer_line_step], bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2]);

          // BGBG line
          // Bayer         -1 0 1 2
          //         -1     g b g b
          //          0     r g r g
          // line_step      g B g b
          // line_step2     r g r g
          rgb_buffer[rgb_line_step ] = AVG4 (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1], bayer_pixel[-1], bayer_pixel[bayer_line_step2 - 1]);
          rgb_buffer[rgb_line_step + 2] = bayer_pixel[bayer_line_step];

          dv = abs (bayer_pixel[0] - bayer_pixel[bayer_line_step2]);
          dh = abs (bayer_pixel[bayer_line_step - 1] - bayer_pixel[bayer_line_step + 1]);

          if (dv > dh)
            rgb_buffer[rgb_line_step + 1] = AVG (bayer_pixel[bayer_line_step - 1], bayer_pixel[bayer_line_step + 1]);
          else if (dh > dv)
            rgb_buffer[rgb_line_step + 1] = AVG (bayer_pixel[0], bayer_pixel[bayer_line_step2]);
          else
            rgb_buffer[rgb_line_step + 1] = AVG4 (bayer_pixel[0], bayer_pixel[bayer_line_step2], bayer_pixel[bayer_line_step - 1], bayer_pixel[bayer_line_step + 1]);

          // Bayer         -1 0 1 2
          //         -1     g b g b
          //          0     r g r g
          // line_step      g b G b
          // line_step2     r g r g
          rgb_buffer[rgb_line_step + 3] = AVG (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1]);
          rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
          rgb_buffer[rgb_line_step + 5] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2]);
        }

        // last two pixels of the line
        // last two pixel values for first two lines
        // GRGR line
        // Bayer        -1 0 1
        //           0   r G r
        //   line_step   g b g
        // line_step2    r g r
        rgb_buffer[0] = AVG (bayer_pixel[1], bayer_pixel[-1]);
        rgb_buffer[1] = bayer_pixel[0];
        rgb_buffer[rgb_line_step + 5] = rgb_buffer[rgb_line_step + 2] = rgb_buffer[5] = rgb_buffer[2] = bayer_pixel[bayer_line_step];

        // Bayer        -1 0 1
        //          0    r g R
        //  line_step    g b g
        // line_step2    r g r
        rgb_buffer[3] = bayer_pixel[1];
        rgb_buffer[4] = AVG (bayer_pixel[0], bayer_pixel[bayer_line_step + 1]);
        //rgb_pixel[5] = bayer_pixel[line_step];

        // BGBG line
        // Bayer        -1 0 1
        //          0    r g r
        //  line_step    g B g
        // line_step2    r g r
        rgb_buffer[rgb_line_step ] = AVG4 (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1], bayer_pixel[-1], bayer_pixel[bayer_line_step2 - 1]);
        rgb_buffer[rgb_line_step + 1] = AVG4 (bayer_pixel[0], bayer_pixel[bayer_line_step2], bayer_pixel[bayer_line_step - 1], bayer_pixel[bayer_line_step + 1]);
        //rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

        // Bayer         -1 0 1
        //         0      r g r
        // line_step      g b G
        // line_step2     r g r
        rgb_buffer[rgb_line_step + 3] = AVG (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1]);
        rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
        //rgb_pixel[rgb_line_step + 5] = bayer_pixel[line_step];

        bayer_pixel += bayer_line_step + 2;
        rgb_buffer += rgb_line_step + 6 + rgb_line_skip;
      }

      //last two lines
      // Bayer         0 1 2
      //        -1     b g b
      //         0     G r g
      // line_step     b g b

      rgb_buffer[rgb_line_step + 3] = rgb_buffer[rgb_line_step ] = rgb_buffer[3] = rgb_buffer[0] = bayer_pixel[1]; // red pixel
      rgb_buffer[1] = bayer_pixel[0]; // green pixel
      rgb_buffer[rgb_line_step + 2] = rgb_buffer[2] = bayer_pixel[bayer_line_step]; // blue;

      // Bayer         0 1 2
      //        -1     b g b
      //         0     g R g
      // line_step     b g b
      //rgb_pixel[3] = bayer_pixel[1];
      rgb_buffer[4] = AVG4 (bayer_pixel[0], bayer_pixel[2], bayer_pixel[bayer_line_step + 1], bayer_pixel[1 - bayer_line_step]);
      rgb_buffer[5] = AVG4 (bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2], bayer_pixel[-bayer_line_step], bayer_pixel[2 - bayer_line_step]);

      // BGBG line
      // Bayer         0 1 2
      //        -1     b g b
      //         0     g r g
      // line_step     B g b
      //rgb_pixel[rgb_line_step    ] = bayer_pixel[1];
      rgb_buffer[rgb_line_step + 1] = AVG (bayer_pixel[0], bayer_pixel[bayer_line_step + 1]);
      rgb_buffer[rgb_line_step + 2] = bayer_pixel[bayer_line_step];

      // Bayer         0 1 2
      //        -1     b g b
      //         0     g r g
      // line_step     b G b
      //rgb_pixel[rgb_line_step + 3] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
      rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
      rgb_buffer[rgb_line_step + 5] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2]);

      rgb_buffer += 6;
      bayer_pixel += 2;
      // rest of the last two lines
      for (xIdx = 2; xIdx < width - 2; xIdx += 2, rgb_buffer += 6, bayer_pixel += 2)
      {
        // GRGR line
        // Bayer       -1 0 1 2
        //        -1    g b g b
        //         0    r G r g
        // line_step    g b g b
        rgb_buffer[0] = AVG (bayer_pixel[1], bayer_pixel[-1]);
        rgb_buffer[1] = bayer_pixel[0];
        rgb_buffer[2] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[-bayer_line_step]);

        // Bayer       -1 0 1 2
        //        -1    g b g b
        //         0    r g R g
        // line_step    g b g b
        rgb_buffer[rgb_line_step + 3] = rgb_buffer[3] = bayer_pixel[1];
        rgb_buffer[4] = AVG4 (bayer_pixel[0], bayer_pixel[2], bayer_pixel[bayer_line_step + 1], bayer_pixel[1 - bayer_line_step]);
        rgb_buffer[5] = AVG4 (bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2], bayer_pixel[-bayer_line_step], bayer_pixel[-bayer_line_step + 2]);

        // BGBG line
        // Bayer       -1 0 1 2
        //        -1    g b g b
        //         0    r g r g
        // line_step    g B g b
        rgb_buffer[rgb_line_step ] = AVG (bayer_pixel[-1], bayer_pixel[1]);
        rgb_buffer[rgb_line_step + 1] = AVG3 (bayer_pixel[0], bayer_pixel[bayer_line_step - 1], bayer_pixel[bayer_line_step + 1]);
        rgb_buffer[rgb_line_step + 2] = bayer_pixel[bayer_line_step];


        // Bayer       -1 0 1 2
        //        -1    g b g b
        //         0    r g r g
        // line_step    g b G b
        //rgb_pixel[rgb_line_step + 3] = bayer_pixel[1];
        rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
        rgb_buffer[rgb_line_step + 5] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2]);
      }

      // last two pixel values for first two lines
      // GRGR line
      // Bayer       -1 0 1
      //        -1    g b g
      //         0    r G r
      // line_step    g b g
      rgb_buffer[rgb_line_step ] = rgb_buffer[0] = AVG (bayer_pixel[1], bayer_pixel[-1]);
      rgb_buffer[1] = bayer_pixel[0];
      rgb_buffer[5] = rgb_buffer[2] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[-bayer_line_step]);

      // Bayer       -1 0 1
      //        -1    g b g
      //         0    r g R
      // line_step    g b g
      rgb_buffer[rgb_line_step + 3] = rgb_buffer[3] = bayer_pixel[1];
      rgb_buffer[4] = AVG3 (bayer_pixel[0], bayer_pixel[bayer_line_step + 1], bayer_pixel[-bayer_line_step + 1]);
      //rgb_pixel[5] = AVG( bayer_pixel[line_step], bayer_pixel[-line_step] );

      // BGBG line
      // Bayer       -1 0 1
      //        -1    g b g
      //         0    r g r
      // line_step    g B g
      //rgb_pixel[rgb_line_step    ] = AVG2( bayer_pixel[-1], bayer_pixel[1] );
      rgb_buffer[rgb_line_step + 1] = AVG3 (bayer_pixel[0], bayer_pixel[bayer_line_step - 1], bayer_pixel[bayer_line_step + 1]);
      rgb_buffer[rgb_line_step + 5] = rgb_buffer[rgb_line_step + 2] = bayer_pixel[bayer_line_step];

      // Bayer       -1 0 1
      //        -1    g b g
      //         0    r g r
      // line_step    g b G
      //rgb_pixel[rgb_line_step + 3] = bayer_pixel[1];
      rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
      //rgb_pixel[rgb_line_step + 5] = bayer_pixel[line_step];
    }
    else if (debayering_method_ == EdgeAwareWeighted)
    {
      int dh, dv;

      // first two pixel values for first two lines
      // Bayer         0 1 2
      //         0     G r g
      // line_step     b g b
      // line_step2    g r g

      rgb_buffer[3] = rgb_buffer[0] = bayer_pixel[1]; // red pixel
      rgb_buffer[1] = bayer_pixel[0]; // green pixel
      rgb_buffer[rgb_line_step + 2] = rgb_buffer[2] = bayer_pixel[bayer_line_step]; // blue;

      // Bayer         0 1 2
      //         0     g R g
      // line_step     b g b
      // line_step2    g r g
      //rgb_pixel[3] = bayer_pixel[1];
      rgb_buffer[4] = AVG3 (bayer_pixel[0], bayer_pixel[2], bayer_pixel[bayer_line_step + 1]);
      rgb_buffer[rgb_line_step + 5] = rgb_buffer[5] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2]);

      // BGBG line
      // Bayer         0 1 2
      //         0     g r g
      // line_step     B g b
      // line_step2    g r g
      rgb_buffer[rgb_line_step + 3] = rgb_buffer[rgb_line_step ] = AVG (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1]);
      rgb_buffer[rgb_line_step + 1] = AVG3 (bayer_pixel[0], bayer_pixel[bayer_line_step + 1], bayer_pixel[bayer_line_step2]);
      //rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

      // pixel (1, 1)  0 1 2
      //         0     g r g
      // line_step     b G b
      // line_step2    g r g
      //rgb_pixel[rgb_line_step + 3] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
      rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
      //rgb_pixel[rgb_line_step + 5] = AVG( bayer_pixel[line_step] , bayer_pixel[line_step+2] );

      rgb_buffer += 6;
      bayer_pixel += 2;
      // rest of the first two lines
      for (xIdx = 2; xIdx < width - 2; xIdx += 2, rgb_buffer += 6, bayer_pixel += 2)
      {
        // GRGR line
        // Bayer        -1 0 1 2
        //           0   r G r g
        //   line_step   g b g b
        // line_step2    r g r g
        rgb_buffer[0] = AVG (bayer_pixel[1], bayer_pixel[-1]);
        rgb_buffer[1] = bayer_pixel[0];
        rgb_buffer[2] = bayer_pixel[bayer_line_step + 1];

        // Bayer        -1 0 1 2
        //          0    r g R g
        //  line_step    g b g b
        // line_step2    r g r g
        rgb_buffer[3] = bayer_pixel[1];
        rgb_buffer[4] = AVG3 (bayer_pixel[0], bayer_pixel[2], bayer_pixel[bayer_line_step + 1]);
        rgb_buffer[rgb_line_step + 5] = rgb_buffer[5] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2]);

        // BGBG line
        // Bayer         -1 0 1 2
        //         0      r g r g
        // line_step      g B g b
        // line_step2     r g r g
        rgb_buffer[rgb_line_step ] = AVG4 (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1], bayer_pixel[-1], bayer_pixel[bayer_line_step2 - 1]);
        rgb_buffer[rgb_line_step + 1] = AVG4 (bayer_pixel[0], bayer_pixel[bayer_line_step2], bayer_pixel[bayer_line_step - 1], bayer_pixel[bayer_line_step + 1]);
        rgb_buffer[rgb_line_step + 2] = bayer_pixel[bayer_line_step];

        // Bayer         -1 0 1 2
        //         0      r g r g
        // line_step      g b G b
        // line_step2     r g r g
        rgb_buffer[rgb_line_step + 3] = AVG (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1]);
        rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
        //rgb_pixel[rgb_line_step + 5] = AVG( bayer_pixel[line_step] , bayer_pixel[line_step+2] );
      }

      // last two pixel values for first two lines
      // GRGR line
      // Bayer        -1 0 1
      //           0   r G r
      //   line_step   g b g
      // line_step2    r g r
      rgb_buffer[0] = AVG (bayer_pixel[1], bayer_pixel[-1]);
      rgb_buffer[1] = bayer_pixel[0];
      rgb_buffer[rgb_line_step + 5] = rgb_buffer[rgb_line_step + 2] = rgb_buffer[5] = rgb_buffer[2] = bayer_pixel[bayer_line_step];

      // Bayer        -1 0 1
      //          0    r g R
      //  line_step    g b g
      // line_step2    r g r
      rgb_buffer[3] = bayer_pixel[1];
      rgb_buffer[4] = AVG (bayer_pixel[0], bayer_pixel[bayer_line_step + 1]);
      //rgb_pixel[5] = bayer_pixel[line_step];

      // BGBG line
      // Bayer        -1 0 1
      //          0    r g r
      //  line_step    g B g
      // line_step2    r g r
      rgb_buffer[rgb_line_step ] = AVG4 (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1], bayer_pixel[-1], bayer_pixel[bayer_line_step2 - 1]);
      rgb_buffer[rgb_line_step + 1] = AVG4 (bayer_pixel[0], bayer_pixel[bayer_line_step2], bayer_pixel[bayer_line_step - 1], bayer_pixel[bayer_line_step + 1]);
      //rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

      // Bayer         -1 0 1
      //         0      r g r
      // line_step      g b G
      // line_step2     r g r
      rgb_buffer[rgb_line_step + 3] = AVG (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1]);
      rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
      //rgb_pixel[rgb_line_step + 5] = bayer_pixel[line_step];

      bayer_pixel += bayer_line_step + 2;
      rgb_buffer += rgb_line_step + 6 + rgb_line_skip;
      // main processing
      for (yIdx = 2; yIdx < height - 2; yIdx += 2)
      {
        // first two pixel values
        // Bayer         0 1 2
        //        -1     b g b
        //         0     G r g
        // line_step     b g b
        // line_step2    g r g

        rgb_buffer[3] = rgb_buffer[0] = bayer_pixel[1]; // red pixel
        rgb_buffer[1] = bayer_pixel[0]; // green pixel
        rgb_buffer[2] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[-bayer_line_step]); // blue;

        // Bayer         0 1 2
        //        -1     b g b
        //         0     g R g
        // line_step     b g b
        // line_step2    g r g
        //rgb_pixel[3] = bayer_pixel[1];
        rgb_buffer[4] = AVG4 (bayer_pixel[0], bayer_pixel[2], bayer_pixel[bayer_line_step + 1], bayer_pixel[1 - bayer_line_step]);
        rgb_buffer[5] = AVG4 (bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2], bayer_pixel[-bayer_line_step], bayer_pixel[2 - bayer_line_step]);

        // BGBG line
        // Bayer         0 1 2
        //         0     g r g
        // line_step     B g b
        // line_step2    g r g
        rgb_buffer[rgb_line_step + 3] = rgb_buffer[rgb_line_step ] = AVG (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1]);
        rgb_buffer[rgb_line_step + 1] = AVG3 (bayer_pixel[0], bayer_pixel[bayer_line_step + 1], bayer_pixel[bayer_line_step2]);
        rgb_buffer[rgb_line_step + 2] = bayer_pixel[bayer_line_step];

        // pixel (1, 1)  0 1 2
        //         0     g r g
        // line_step     b G b
        // line_step2    g r g
        //rgb_pixel[rgb_line_step + 3] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
        rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
        rgb_buffer[rgb_line_step + 5] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2]);

        rgb_buffer += 6;
        bayer_pixel += 2;
        // continue with rest of the line
        for (xIdx = 2; xIdx < width - 2; xIdx += 2, rgb_buffer += 6, bayer_pixel += 2)
        {
          // GRGR line
          // Bayer        -1 0 1 2
          //          -1   g b g b
          //           0   r G r g
          //   line_step   g b g b
          // line_step2    r g r g
          rgb_buffer[0] = AVG (bayer_pixel[1], bayer_pixel[-1]);
          rgb_buffer[1] = bayer_pixel[0];
          rgb_buffer[2] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[-bayer_line_step]);

          // Bayer        -1 0 1 2
          //          -1   g b g b
          //          0    r g R g
          //  line_step    g b g b
          // line_step2    r g r g

          dh = abs (bayer_pixel[0] - bayer_pixel[2]);
          dv = abs (bayer_pixel[-bayer_line_step + 1] - bayer_pixel[bayer_line_step + 1]);

          if (dv == 0 && dh == 0)
            rgb_buffer[4] = AVG4 (bayer_pixel[1 - bayer_line_step], bayer_pixel[1 + bayer_line_step], bayer_pixel[0], bayer_pixel[2]);
          else
            rgb_buffer[4] = WAVG4 (bayer_pixel[1 - bayer_line_step], bayer_pixel[1 + bayer_line_step], bayer_pixel[0], bayer_pixel[2], dh, dv);
          rgb_buffer[3] = bayer_pixel[1];
          rgb_buffer[5] = AVG4 (bayer_pixel[-bayer_line_step], bayer_pixel[2 - bayer_line_step], bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2]);

          // BGBG line
          // Bayer         -1 0 1 2
          //         -1     g b g b
          //          0     r g r g
          // line_step      g B g b
          // line_step2     r g r g
          rgb_buffer[rgb_line_step ] = AVG4 (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1], bayer_pixel[-1], bayer_pixel[bayer_line_step2 - 1]);
          rgb_buffer[rgb_line_step + 2] = bayer_pixel[bayer_line_step];

          dv = abs (bayer_pixel[0] - bayer_pixel[bayer_line_step2]);
          dh = abs (bayer_pixel[bayer_line_step - 1] - bayer_pixel[bayer_line_step + 1]);

          if (dv == 0 && dh == 0)
            rgb_buffer[rgb_line_step + 1] = AVG4 (bayer_pixel[0], bayer_pixel[bayer_line_step2], bayer_pixel[bayer_line_step - 1], bayer_pixel[bayer_line_step + 1]);
          else
            rgb_buffer[rgb_line_step + 1] = WAVG4 (bayer_pixel[0], bayer_pixel[bayer_line_step2], bayer_pixel[bayer_line_step - 1], bayer_pixel[bayer_line_step + 1], dh, dv);

          // Bayer         -1 0 1 2
          //         -1     g b g b
          //          0     r g r g
          // line_step      g b G b
          // line_step2     r g r g
          rgb_buffer[rgb_line_step + 3] = AVG (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1]);
          rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
          rgb_buffer[rgb_line_step + 5] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2]);
        }

        // last two pixels of the line
        // last two pixel values for first two lines
        // GRGR line
        // Bayer        -1 0 1
        //           0   r G r
        //   line_step   g b g
        // line_step2    r g r
        rgb_buffer[0] = AVG (bayer_pixel[1], bayer_pixel[-1]);
        rgb_buffer[1] = bayer_pixel[0];
        rgb_buffer[rgb_line_step + 5] = rgb_buffer[rgb_line_step + 2] = rgb_buffer[5] = rgb_buffer[2] = bayer_pixel[bayer_line_step];

        // Bayer        -1 0 1
        //          0    r g R
        //  line_step    g b g
        // line_step2    r g r
        rgb_buffer[3] = bayer_pixel[1];
        rgb_buffer[4] = AVG (bayer_pixel[0], bayer_pixel[bayer_line_step + 1]);
        //rgb_pixel[5] = bayer_pixel[line_step];

        // BGBG line
        // Bayer        -1 0 1
        //          0    r g r
        //  line_step    g B g
        // line_step2    r g r
        rgb_buffer[rgb_line_step ] = AVG4 (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1], bayer_pixel[-1], bayer_pixel[bayer_line_step2 - 1]);
        rgb_buffer[rgb_line_step + 1] = AVG4 (bayer_pixel[0], bayer_pixel[bayer_line_step2], bayer_pixel[bayer_line_step - 1], bayer_pixel[bayer_line_step + 1]);
        //rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

        // Bayer         -1 0 1
        //         0      r g r
        // line_step      g b G
        // line_step2     r g r
        rgb_buffer[rgb_line_step + 3] = AVG (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1]);
        rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
        //rgb_pixel[rgb_line_step + 5] = bayer_pixel[line_step];

        bayer_pixel += bayer_line_step + 2;
        rgb_buffer += rgb_line_step + 6 + rgb_line_skip;
      }

      //last two lines
      // Bayer         0 1 2
      //        -1     b g b
      //         0     G r g
      // line_step     b g b

      rgb_buffer[rgb_line_step + 3] = rgb_buffer[rgb_line_step ] = rgb_buffer[3] = rgb_buffer[0] = bayer_pixel[1]; // red pixel
      rgb_buffer[1] = bayer_pixel[0]; // green pixel
      rgb_buffer[rgb_line_step + 2] = rgb_buffer[2] = bayer_pixel[bayer_line_step]; // blue;

      // Bayer         0 1 2
      //        -1     b g b
      //         0     g R g
      // line_step     b g b
      //rgb_pixel[3] = bayer_pixel[1];
      rgb_buffer[4] = AVG4 (bayer_pixel[0], bayer_pixel[2], bayer_pixel[bayer_line_step + 1], bayer_pixel[1 - bayer_line_step]);
      rgb_buffer[5] = AVG4 (bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2], bayer_pixel[-bayer_line_step], bayer_pixel[2 - bayer_line_step]);

      // BGBG line
      // Bayer         0 1 2
      //        -1     b g b
      //         0     g r g
      // line_step     B g b
      //rgb_pixel[rgb_line_step    ] = bayer_pixel[1];
      rgb_buffer[rgb_line_step + 1] = AVG (bayer_pixel[0], bayer_pixel[bayer_line_step + 1]);
      rgb_buffer[rgb_line_step + 2] = bayer_pixel[bayer_line_step];

      // Bayer         0 1 2
      //        -1     b g b
      //         0     g r g
      // line_step     b G b
      //rgb_pixel[rgb_line_step + 3] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
      rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
      rgb_buffer[rgb_line_step + 5] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2]);

      rgb_buffer += 6;
      bayer_pixel += 2;
      // rest of the last two lines
      for (xIdx = 2; xIdx < width - 2; xIdx += 2, rgb_buffer += 6, bayer_pixel += 2)
      {
        // GRGR line
        // Bayer       -1 0 1 2
        //        -1    g b g b
        //         0    r G r g
        // line_step    g b g b
        rgb_buffer[0] = AVG (bayer_pixel[1], bayer_pixel[-1]);
        rgb_buffer[1] = bayer_pixel[0];
        rgb_buffer[2] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[-bayer_line_step]);

        // Bayer       -1 0 1 2
        //        -1    g b g b
        //         0    r g R g
        // line_step    g b g b
        rgb_buffer[rgb_line_step + 3] = rgb_buffer[3] = bayer_pixel[1];
        rgb_buffer[4] = AVG4 (bayer_pixel[0], bayer_pixel[2], bayer_pixel[bayer_line_step + 1], bayer_pixel[1 - bayer_line_step]);
        rgb_buffer[5] = AVG4 (bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2], bayer_pixel[-bayer_line_step], bayer_pixel[-bayer_line_step + 2]);

        // BGBG line
        // Bayer       -1 0 1 2
        //        -1    g b g b
        //         0    r g r g
        // line_step    g B g b
        rgb_buffer[rgb_line_step ] = AVG (bayer_pixel[-1], bayer_pixel[1]);
        rgb_buffer[rgb_line_step + 1] = AVG3 (bayer_pixel[0], bayer_pixel[bayer_line_step - 1], bayer_pixel[bayer_line_step + 1]);
        rgb_buffer[rgb_line_step + 2] = bayer_pixel[bayer_line_step];


        // Bayer       -1 0 1 2
        //        -1    g b g b
        //         0    r g r g
        // line_step    g b G b
        //rgb_pixel[rgb_line_step + 3] = bayer_pixel[1];
        rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
        rgb_buffer[rgb_line_step + 5] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2]);
      }

      // last two pixel values for first two lines
      // GRGR line
      // Bayer       -1 0 1
      //        -1    g b g
      //         0    r G r
      // line_step    g b g
      rgb_buffer[rgb_line_step ] = rgb_buffer[0] = AVG (bayer_pixel[1], bayer_pixel[-1]);
      rgb_buffer[1] = bayer_pixel[0];
      rgb_buffer[5] = rgb_buffer[2] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[-bayer_line_step]);

      // Bayer       -1 0 1
      //        -1    g b g
      //         0    r g R
      // line_step    g b g
      rgb_buffer[rgb_line_step + 3] = rgb_buffer[3] = bayer_pixel[1];
      rgb_buffer[4] = AVG3 (bayer_pixel[0], bayer_pixel[bayer_line_step + 1], bayer_pixel[-bayer_line_step + 1]);
      //rgb_pixel[5] = AVG( bayer_pixel[line_step], bayer_pixel[-line_step] );

      // BGBG line
      // Bayer       -1 0 1
      //        -1    g b g
      //         0    r g r
      // line_step    g B g
      //rgb_pixel[rgb_line_step    ] = AVG2( bayer_pixel[-1], bayer_pixel[1] );
      rgb_buffer[rgb_line_step + 1] = AVG3 (bayer_pixel[0], bayer_pixel[bayer_line_step - 1], bayer_pixel[bayer_line_step + 1]);
      rgb_buffer[rgb_line_step + 5] = rgb_buffer[rgb_line_step + 2] = bayer_pixel[bayer_line_step];

      // Bayer       -1 0 1
      //        -1    g b g
      //         0    r g r
      // line_step    g b G
      //rgb_pixel[rgb_line_step + 3] = bayer_pixel[1];
      rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
      //rgb_pixel[rgb_line_step + 5] = bayer_pixel[line_step];
    }
    else
      THROW_OPENNI_EXCEPTION ("Unknwon debayering method: %d", debayering_method_);
  }
  else
  {
    if (image_md_->XRes () % width != 0 || image_md_->YRes () % height != 0)
      THROW_OPENNI_EXCEPTION ("Downsampling only possible for integer scales in both dimensions. Request was %d x %d -> %d x %d.", image_md_->XRes (), image_md_->YRes (), width, height);

    // get each or each 2nd pixel group to find rgb values!
    register unsigned bayerXStep = image_md_->XRes () / width;
    register unsigned bayerYSkip = (image_md_->YRes () / height - 1) * image_md_->XRes ();

    // Downsampling and debayering at once
    register const XnUInt8* bayer_buffer = image_md_->Data ();

    for (register unsigned yIdx = 0; yIdx < height; ++yIdx, bayer_buffer += bayerYSkip, rgb_buffer += rgb_line_skip) // skip a line
    {
      for (register unsigned xIdx = 0; xIdx < width; ++xIdx, rgb_buffer += 3, bayer_buffer += bayerXStep)
      {
        rgb_buffer[ 2 ] = bayer_buffer[ image_md_->XRes () ];
        rgb_buffer[ 1 ] = AVG (bayer_buffer[0], bayer_buffer[ image_md_->XRes () + 1]);
        rgb_buffer[ 0 ] = bayer_buffer[ 1 ];
      }
    }
  }
}
}//namespace
#endif
