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

#pragma once

#include <pcl/memory.h>
#include <pcl/pcl_exports.h>

// Template frame wrappers
#include <pcl/io/image.h>
#include <pcl/io/image_depth.h>
#include <pcl/io/image_ir.h>

#include <pcl/io/io_exception.h>
#include <pcl/io/openni2/openni2_video_mode.h>

#include "openni.h"

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>


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
      using DepthImage = pcl::io::DepthImage;
      using IRImage = pcl::io::IRImage;
      using Image = pcl::io::Image;

      class OpenNI2FrameListener;

      class PCL_EXPORTS OpenNI2Device
      {
        public:
          using Ptr = shared_ptr<OpenNI2Device>;
          using ConstPtr = shared_ptr<const OpenNI2Device>;

          using ImageCallbackFunction = std::function<void(Image::Ptr, void* cookie) >;
          using DepthImageCallbackFunction = std::function<void(DepthImage::Ptr, void* cookie) >;
          using IRImageCallbackFunction = std::function<void(IRImage::Ptr, void* cookie) >;
          using CallbackHandle = unsigned;

          using StreamCallbackFunction = std::function<void(openni::VideoStream& stream)>;

          OpenNI2Device (const std::string& device_URI);
          virtual ~OpenNI2Device ();

          const std::string
          getUri () const;
          const std::string
          getVendor () const;
          const std::string
          getName () const;
          std::uint16_t
          getUsbVendorId () const;
          std::uint16_t
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
          std::uint64_t
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

          /** \brief Get absolute number of depth frames in the current stream.
          * This function returns 0 if the current device is not a file stream or
          * if the current mode has no depth stream.
          */
          int
          getDepthFrameCount ();

          /** \brief Get absolute number of color frames in the current stream.
          * This function returns 0 if the current device is not a file stream or
          * if the current mode has no color stream.
          */
          int
          getColorFrameCount ();

          /** \brief Get absolute number of ir frames in the current stream.
          * This function returns 0 if the current device is not a file stream or
          * if the current mode has no ir stream.
          */
          int
          getIRFrameCount ();

          /** \brief Set the playback speed if the device is an recorded stream.
          * If setting the device playback speed fails, because the device is no recorded stream or
          * any other reason this function returns false. Otherwise true is returned.
          * \param[in] speed The playback speed factor 1.0 means the same speed as recorded,
          * 0.5 half the speed, 2.0 double speed and so on.
          * \return True on success, false otherwise.
          */
          bool
          setPlaybackSpeed (double speed);

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

          std::shared_ptr<openni::VideoStream>
          getIRVideoStream () const;
          std::shared_ptr<openni::VideoStream>
          getColorVideoStream () const;
          std::shared_ptr<openni::VideoStream>
          getDepthVideoStream () const;


          void
          processColorFrame (openni::VideoStream& stream);
          void
          processDepthFrame (openni::VideoStream& stream);
          void
          processIRFrame (openni::VideoStream& stream);


          bool
          findCompatibleVideoMode (const std::vector<OpenNI2VideoMode>& supportedModes,
            const OpenNI2VideoMode& output_mode, OpenNI2VideoMode& mode) const;

          bool
          resizingSupported (std::size_t input_width, std::size_t input_height, std::size_t output_width, std::size_t output_height) const;

          // Members

          std::shared_ptr<openni::Device> openni_device_;
          std::shared_ptr<openni::DeviceInfo> device_info_;

          std::shared_ptr<OpenNI2FrameListener> ir_frame_listener;
          std::shared_ptr<OpenNI2FrameListener> color_frame_listener;
          std::shared_ptr<OpenNI2FrameListener> depth_frame_listener;

          mutable std::shared_ptr<openni::VideoStream> ir_video_stream_;
          mutable std::shared_ptr<openni::VideoStream> color_video_stream_;
          mutable std::shared_ptr<openni::VideoStream> depth_video_stream_;

          mutable std::vector<OpenNI2VideoMode> ir_video_modes_;
          mutable std::vector<OpenNI2VideoMode> color_video_modes_;
          mutable std::vector<OpenNI2VideoMode> depth_video_modes_;

          bool ir_video_started_;
          bool color_video_started_;
          bool depth_video_started_;

          /** \brief distance between the projector and the IR camera in meters*/
          float baseline_;
          /** the value for shadow (occluded pixels) */
          std::uint64_t shadow_value_;
          /** the value for pixels without a valid disparity measurement */
          std::uint64_t no_sample_value_;
      };

      PCL_EXPORTS std::ostream& operator<< (std::ostream& stream, const OpenNI2Device& device);

    } // namespace
  }
}
