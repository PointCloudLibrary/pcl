/*
 * Copyright (c) 2013, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *      Author: Julius Kammerl (jkammerl@willowgarage.com)
 */

#include "openni2_camera/openni2_video_mode.h"

namespace openni2_wrapper
{


std::ostream& operator << (std::ostream& stream, const OpenNI2VideoMode& video_mode) {
  stream << "Resolution: " << (int)video_mode.x_resolution_ << "x" << (int)video_mode.y_resolution_ <<
                              "@" << video_mode.frame_rate_ <<
                              "Hz Format: ";

  switch (video_mode.pixel_format_)
  {
    case PIXEL_FORMAT_DEPTH_1_MM:
      stream << "Depth 1mm";
      break;
    case PIXEL_FORMAT_DEPTH_100_UM:
      stream << "Depth 100um";
      break;
    case PIXEL_FORMAT_SHIFT_9_2:
      stream << "Shift 9/2";
      break;
    case PIXEL_FORMAT_SHIFT_9_3:
      stream << "Shift 9/3";
      break;
    case PIXEL_FORMAT_RGB888:
      stream << "RGB888";
      break;
    case PIXEL_FORMAT_YUV422:
      stream << "YUV422";
      break;
    case PIXEL_FORMAT_GRAY8:
      stream << "Gray8";
      break;
    case PIXEL_FORMAT_GRAY16:
      stream << "Gray16";
      break;
    case PIXEL_FORMAT_JPEG:
      stream << "JPEG";
      break;

    default:
      break;
  }

  return stream;
}

bool operator==(const OpenNI2VideoMode& video_mode_a, const OpenNI2VideoMode& video_mode_b)
{
  return (video_mode_a.x_resolution_==video_mode_b.x_resolution_) &&
         (video_mode_a.y_resolution_==video_mode_b.y_resolution_) &&
         (video_mode_a.frame_rate_  ==video_mode_b.frame_rate_)   &&
         (video_mode_a.pixel_format_==video_mode_b.pixel_format_);
}

bool operator!=(const OpenNI2VideoMode& video_mode_a, const OpenNI2VideoMode& video_mode_b)
{
  return !(video_mode_a==video_mode_b);
}

} //namespace openni2_wrapper
