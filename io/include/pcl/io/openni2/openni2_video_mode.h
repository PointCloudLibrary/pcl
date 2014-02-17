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

#ifndef PCL_IO_OPENNI2_VIDEO_MODE_H_
#define PCL_IO_OPENNI2_VIDEO_MODE_H_

#include <cstddef>
#include <ostream>

#include <OpenNI.h>

namespace pcl
{
  namespace io
  {
    namespace openni2
    {
      // copied from OniEnums.h
      typedef enum
      {
	      // Depth
        PIXEL_FORMAT_DEPTH_1_MM = 100,
        PIXEL_FORMAT_DEPTH_100_UM = 101,
        PIXEL_FORMAT_SHIFT_9_2 = 102,
        PIXEL_FORMAT_SHIFT_9_3 = 103,

	      // Color
        PIXEL_FORMAT_RGB888 = 200,
        PIXEL_FORMAT_YUV422 = 201,
        PIXEL_FORMAT_GRAY8 = 202,
        PIXEL_FORMAT_GRAY16 = 203,
        PIXEL_FORMAT_JPEG = 204,
        PIXEL_FORMAT_YUYV = 205,
      } PixelFormat;

      struct OpenNI2VideoMode
      {
        OpenNI2VideoMode ()
          :x_resolution_(0), y_resolution_(0), frame_rate_(0)
        {}

        OpenNI2VideoMode (int xResolution, int yResolution, int frameRate)
          :x_resolution_(xResolution), y_resolution_(yResolution), frame_rate_(frameRate)
        {}

        int x_resolution_;
        int y_resolution_;
        int frame_rate_;
        PixelFormat pixel_format_;
      };

      std::ostream&
      operator<< (std::ostream& stream, const OpenNI2VideoMode& video_mode);

      bool
      operator== (const OpenNI2VideoMode& video_mode_a, const OpenNI2VideoMode& video_mode_b);

      bool
      operator!= (const OpenNI2VideoMode& video_mode_a, const OpenNI2VideoMode& video_mode_b);

    } // namespace
  }
}

#endif
