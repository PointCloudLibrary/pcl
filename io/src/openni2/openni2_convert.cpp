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

#include "pcl/io/openni2/openni2_convert.h"
#include "pcl/io/io_exception.h"

#include <boost/make_shared.hpp>

#include <string>

namespace pcl
{
  namespace io
  {
    namespace openni2
    {

      const OpenNI2DeviceInfo
      openni2_convert (const openni::DeviceInfo* pInfo)
      {
        if (!pInfo)
          THROW_IO_EXCEPTION ("openni2_convert called with zero pointer\n");

        OpenNI2DeviceInfo output;

        output.name_       = pInfo->getName ();
        output.uri_        = pInfo->getUri ();
        output.vendor_     = pInfo->getVendor ();
        output.product_id_ = pInfo->getUsbProductId ();
        output.vendor_id_  = pInfo->getUsbVendorId ();

        return (output);
      }

      const openni::VideoMode
      grabberModeToOpenniMode (const OpenNI2VideoMode& input)
      {

        openni::VideoMode output;

        output.setResolution (input.x_resolution_, input.y_resolution_);
        output.setFps (input.frame_rate_);
        output.setPixelFormat (static_cast<openni::PixelFormat>(input.pixel_format_));

        return (output);
      }


      const OpenNI2VideoMode
      openniModeToGrabberMode (const openni::VideoMode& input)
      {
        OpenNI2VideoMode output;

        output.x_resolution_ = input.getResolutionX ();
        output.y_resolution_ = input.getResolutionY ();
        output.frame_rate_ = input.getFps ();
        output.pixel_format_ = static_cast<PixelFormat>(input.getPixelFormat ());

        return (output);
      }

      const std::vector<OpenNI2VideoMode>
      openniModeToGrabberMode (const openni::Array<openni::VideoMode>& input)
      {
        std::vector<OpenNI2VideoMode> output;

        int size = input.getSize ();

        output.reserve (size);

        for (int i=0; i<size; ++i)
          output.push_back (openniModeToGrabberMode (input[i]));

        return (output);
      }

    } // namespace
  }
 } 
