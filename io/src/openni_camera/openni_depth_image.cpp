/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011 Willow Garage, Inc.
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

#include <pcl/io/openni_camera/openni_depth_image.h>
#include <sstream>
#include <limits>
#include <iostream>

using namespace std;

namespace openni_wrapper
{

  void
  DepthImage::fillDepthImageRaw (unsigned width, unsigned height, unsigned short* depth_buffer, unsigned line_step) const
  {
    if (width > depth_md_->XRes () || height > depth_md_->YRes ())
      THROW_OPENNI_EXCEPTION ("upsampling not supported: %d x %d -> %d x %d", depth_md_->XRes (), depth_md_->YRes (), width, height);

    if (depth_md_->XRes () % width != 0 || depth_md_->YRes () % height != 0)
      THROW_OPENNI_EXCEPTION ("downsampling only supported for integer scale: %d x %d -> %d x %d", depth_md_->XRes (), depth_md_->YRes (), width, height);

    if (line_step == 0)
      line_step = width * static_cast<unsigned> (sizeof (unsigned short));

    // special case no sclaing, no padding => memcopy!
    if (width == depth_md_->XRes () && height == depth_md_->YRes () && (line_step == width * sizeof (unsigned short)))
    {
      memcpy (depth_buffer, depth_md_->Data (), depth_md_->DataSize ());
      return;
    }

    // padding skip for destination image
    unsigned bufferSkip = line_step - width * static_cast<unsigned> (sizeof (unsigned short));

    // step and padding skip for source image
    unsigned xStep = depth_md_->XRes () / width;
    unsigned ySkip = (depth_md_->YRes () / height - 1) * depth_md_->XRes ();

    // Fill in the depth image data, converting mm to m
    short bad_point = numeric_limits<short>::quiet_NaN ();
    unsigned depthIdx = 0;

    for (unsigned yIdx = 0; yIdx < height; ++yIdx, depthIdx += ySkip)
    {
      for (unsigned xIdx = 0; xIdx < width; ++xIdx, depthIdx += xStep, ++depth_buffer)
      {
        /// @todo Different values for these cases
        if ((*depth_md_)[depthIdx] == 0 ||
            (*depth_md_)[depthIdx] == no_sample_value_ ||
            (*depth_md_)[depthIdx] == shadow_value_)
          *depth_buffer = bad_point;
        else
        {
          *depth_buffer = static_cast<unsigned short> ((*depth_md_)[depthIdx]);
        }
      }
      // if we have padding
      if (bufferSkip > 0)
      {
        char* cBuffer = reinterpret_cast<char*> (depth_buffer);
        depth_buffer = reinterpret_cast<unsigned short*> (cBuffer + bufferSkip);
      }
    }
  }

  void
  DepthImage::fillDepthImage (unsigned width, unsigned height, float* depth_buffer, unsigned line_step) const
  {
    if (width > depth_md_->XRes () || height > depth_md_->YRes ())
      THROW_OPENNI_EXCEPTION ("upsampling not supported: %d x %d -> %d x %d", depth_md_->XRes (), depth_md_->YRes (), width, height);

    if (depth_md_->XRes () % width != 0 || depth_md_->YRes () % height != 0)
      THROW_OPENNI_EXCEPTION ("downsampling only supported for integer scale: %d x %d -> %d x %d", depth_md_->XRes (), depth_md_->YRes (), width, height);

    if (line_step == 0)
      line_step = width * static_cast<unsigned> (sizeof (float));

    // padding skip for destination image
    unsigned bufferSkip = line_step - width * static_cast<unsigned> (sizeof (float));

    // step and padding skip for source image
    unsigned xStep = depth_md_->XRes () / width;
    unsigned ySkip = (depth_md_->YRes () / height - 1) * depth_md_->XRes ();

    // Fill in the depth image data, converting mm to m
    float bad_point = numeric_limits<float>::quiet_NaN ();
    unsigned depthIdx = 0;

    for (unsigned yIdx = 0; yIdx < height; ++yIdx, depthIdx += ySkip)
    {
      for (unsigned xIdx = 0; xIdx < width; ++xIdx, depthIdx += xStep, ++depth_buffer)
      {
        /// @todo Different values for these cases
        if ((*depth_md_)[depthIdx] == 0 ||
            (*depth_md_)[depthIdx] == no_sample_value_ ||
            (*depth_md_)[depthIdx] == shadow_value_)
          *depth_buffer = bad_point;
        else
        {
          *depth_buffer = static_cast<float> ((*depth_md_)[depthIdx]) * 0.001f;
        }
      }
      // if we have padding
      if (bufferSkip > 0)
      {
        char* cBuffer = reinterpret_cast<char*> (depth_buffer);
        depth_buffer = reinterpret_cast<float*> (cBuffer + bufferSkip);
      }
    }
  }

  void
  DepthImage::fillDisparityImage (unsigned width, unsigned height, float* disparity_buffer, unsigned line_step) const
  {
    if (width > depth_md_->XRes () || height > depth_md_->YRes ())
      THROW_OPENNI_EXCEPTION ("upsampling not supported: %d x %d -> %d x %d", depth_md_->XRes (), depth_md_->YRes (), width, height);

    if (depth_md_->XRes () % width != 0 || depth_md_->YRes () % height != 0)
      THROW_OPENNI_EXCEPTION ("downsampling only supported for integer scale: %d x %d -> %d x %d", depth_md_->XRes (), depth_md_->YRes (), width, height);

    if (line_step == 0)
      line_step = width * static_cast<unsigned> (sizeof (float));

    unsigned xStep = depth_md_->XRes () / width;
    unsigned ySkip = (depth_md_->YRes () / height - 1) * depth_md_->XRes ();

    unsigned bufferSkip = line_step - width * static_cast<unsigned> (sizeof (float));

    // Fill in the depth image data
    // iterate over all elements and fill disparity matrix: disp[x,y] = f * b / z_distance[x,y];
    // focal length is for the native image resolution -> focal_length = focal_length_ / xStep;
    float constant = focal_length_ * baseline_ * 1000.0f / static_cast<float> (xStep);

    for (unsigned yIdx = 0, depthIdx = 0; yIdx < height; ++yIdx, depthIdx += ySkip)
    {
      for (unsigned xIdx = 0; xIdx < width; ++xIdx, depthIdx += xStep, ++disparity_buffer)
      {
        if ((*depth_md_)[depthIdx] == 0 ||
            (*depth_md_)[depthIdx] == no_sample_value_ ||
            (*depth_md_)[depthIdx] == shadow_value_)
          *disparity_buffer = 0.0;
        else
          *disparity_buffer = constant / static_cast<float> ((*depth_md_)[depthIdx]);
      }

      // if we have padding
      if (bufferSkip > 0)
      {
        char* cBuffer = reinterpret_cast<char*> (disparity_buffer);
        disparity_buffer = reinterpret_cast<float*> (cBuffer + bufferSkip);
      }
    }
  }
} // namespace
#endif //HAVE_OPENNI
