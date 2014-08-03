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
 */

#ifndef PCL_IO_OPENNI2_DEVICE_H_
#define PCL_IO_OPENNI2_DEVICE_H_

#include <pcl/pcl_exports.h>
#include "openni.h"
#include "pcl/io/openni2/openni2_video_mode.h"
#include "pcl/io/io_exception.h"

#include <boost/shared_ptr.hpp>
#include <boost/cstdint.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <string>
#include <vector>

// Template frame wrappers
#include <pcl/io/image.h>
#include <pcl/io/image_depth.h>
#include <pcl/io/image_ir.h>



namespace openni
{
  class Device;
  class DeviceInfo;
  class VideoStream;
  class SensorInfo;
}

namespace pcl
{
  namespace io
  {
    namespace openni2
    {
      typedef pcl::io::DepthImage DepthImage;
      typedef pcl::io::IRImage IRImage;
      typedef pcl::io::Image Image;

      class OpenNI2FrameListener;

      class PCL_EXPORTS OpenNI2Device
      {
        public:

          typedef boost::function<void(boost::shared_ptr<Image>, void* cookie) > ImageCallbackFunction;
          typedef boost::function<void(boost::shared_ptr<DepthImage>, void* cookie) > DepthImageCallbackFunction;
          typedef boost::function<void(boost::shared_ptr<IRImage>, void* cookie) > IRImageCallbackFunction;
          typedef unsigned CallbackHandle;

          typedef boost::function<void(openni::VideoStream& stream)> StreamCallbackFunction;

          OpenNI2Device (const std::string& device_URI);
          virtual ~OpenNI2Device ();

          const std::string
          getUri () const;
          const std::string
          getVendor () const;
          const std::string
          getName () const;
          uint16_t
          getUsbVendorId () const;
          uint16_t
          getUsbProductId () const;

          const std::string
          getStringID () const;

          bool
          isValid () const;

          bool
          hasIRSensor () const;
          bool
          hasColorSensor () const;
          bool
          hasDepthSensor () const;

          void
          startIRStream ();
          void
          startColorStream ();
          void
          startDepthStream ();

          void
          stopAllStreams ();

          void
          stopIRStream ();
          void
          stopColorStream ();
          void
          stopDepthStream ();

          bool
          isIRStreamStarted ();
          bool
          isColorStreamStarted ();
          bool
          isDepthStreamStarted ();

          bool
          isImageRegistrationModeSupported () const;
          void
          setImageRegistrationMode (bool enabled);
          bool
          isDepthRegistered () const;

          const OpenNI2VideoMode
          getIRVideoMode ();
          const OpenNI2VideoMode
          getColorVideoMode ();
          const OpenNI2VideoMode
          getDepthVideoMode ();

          const std::vector<OpenNI2VideoMode>&
          getSupportedIRVideoModes () const;
          const std::vector<OpenNI2VideoMode>&
          getSupportedColorVideoModes () const;
          const std::vector<OpenNI2VideoMode>&
          getSupportedDepthVideoModes () const;

          bool
          isIRVideoModeSupported (const OpenNI2VideoMode& video_mode) const;
          bool
          isColorVideoModeSupported (const OpenNI2VideoMode& video_mode) const;
          bool
          isDepthVideoModeSupported (const OpenNI2VideoMode& video_mode) const;

          bool
          findCompatibleIRMode (const OpenNI2VideoMode& requested_mode, OpenNI2VideoMode& actual_mode) const;
          bool
          findCompatibleColorMode (const OpenNI2VideoMode& requested_mode, OpenNI2VideoMode& actual_mode) const;
          bool
          findCompatibleDepthMode (const OpenNI2VideoMode& requested_mode, OpenNI2VideoMode& actual_mode) const;

          void
          setIRVideoMode (const OpenNI2VideoMode& video_mode);
          void
          setColorVideoMode (const OpenNI2VideoMode& video_mode);
          void
          setDepthVideoMode (const OpenNI2VideoMode& video_mode);

          OpenNI2VideoMode
          getDefaultIRMode () const;
          OpenNI2VideoMode
          getDefaultColorMode () const;
          OpenNI2VideoMode
          getDefaultDepthMode () const;

          float
          getIRFocalLength () const;
          float
          getColorFocalLength () const;
          float
          getDepthFocalLength () const;

          // Baseline between sensors. Returns 0 if this value does not exist.
          float
          getBaseline();

          // Value of pixels in shadow or that have no valid measurement
          uint64_t
          getShadowValue();

          void
          setAutoExposure (bool enable);
          void
          setAutoWhiteBalance (bool enable);

          inline bool
          isSynchronized ()
          {
            return (openni_device_->getDepthColorSyncEnabled ());
          }

          inline bool
          isSynchronizationSupported ()
          {
            return (true); // Not sure how to query this from the hardware
          }

          inline bool
          isFile()
          {
            return (openni_device_->isFile());
          }

          void
          setSynchronization (bool enableSync);

          bool
          getAutoExposure () const;
          bool
          getAutoWhiteBalance () const;

          void
          setUseDeviceTimer (bool enable);

          /************************************************************************************/
          // Callbacks from openni::VideoStream to grabber. Internal interface
          void
          setColorCallback (StreamCallbackFunction color_callback);
          void
          setDepthCallback (StreamCallbackFunction depth_callback);
          void
          setIRCallback (StreamCallbackFunction ir_callback);

        protected:
          void shutdown ();

          boost::shared_ptr<openni::VideoStream>
          getIRVideoStream () const;
          boost::shared_ptr<openni::VideoStream>
          getColorVideoStream () const;
          boost::shared_ptr<openni::VideoStream>
          getDepthVideoStream () const;


          void
          processColorFrame (openni::VideoStream& stream);
          void
          processDepthFrame (openni::VideoStream& stream);
          void
          processIRFrame (openni::VideoStream& stream);


          bool
          findCompatibleVideoMode (const std::vector<OpenNI2VideoMode> supportedModes,
            const OpenNI2VideoMode& output_mode, OpenNI2VideoMode& mode) const;

          bool
          resizingSupported (size_t input_width, size_t input_height, size_t output_width, size_t output_height) const;

          // Members

          boost::shared_ptr<openni::Device> openni_device_;
          boost::shared_ptr<openni::DeviceInfo> device_info_;

          boost::shared_ptr<OpenNI2FrameListener> ir_frame_listener;
          boost::shared_ptr<OpenNI2FrameListener> color_frame_listener;
          boost::shared_ptr<OpenNI2FrameListener> depth_frame_listener;

          mutable boost::shared_ptr<openni::VideoStream> ir_video_stream_;
          mutable boost::shared_ptr<openni::VideoStream> color_video_stream_;
          mutable boost::shared_ptr<openni::VideoStream> depth_video_stream_;

          mutable std::vector<OpenNI2VideoMode> ir_video_modes_;
          mutable std::vector<OpenNI2VideoMode> color_video_modes_;
          mutable std::vector<OpenNI2VideoMode> depth_video_modes_;

          bool ir_video_started_;
          bool color_video_started_;
          bool depth_video_started_;

          /** \brief distance between the projector and the IR camera in meters*/
          float baseline_;
          /** the value for shadow (occluded pixels) */
          uint64_t shadow_value_;
          /** the value for pixels without a valid disparity measurement */
          uint64_t no_sample_value_;
      };

      PCL_EXPORTS std::ostream& operator<< (std::ostream& stream, const OpenNI2Device& device);

    } // namespace
  }
}

#endif // PCL_IO_OPENNI2_DEVICE_H_
