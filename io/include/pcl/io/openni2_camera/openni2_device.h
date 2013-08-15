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

#ifndef OPENNI2_DEVICE_H
#define OPENNI2_DEVICE_H

#include "openni2_camera/openni2_video_mode.h"

#include "openni2_camera/openni2_exception.h"

#include <boost/shared_ptr.hpp>
#include <boost/cstdint.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>

#include <sensor_msgs/Image.h>

#include <string>
#include <vector>

namespace openni
{
class Device;
class DeviceInfo;
class VideoStream;
class SensorInfo;
}

namespace openni2_wrapper
{

typedef boost::function<void(sensor_msgs::ImagePtr image)> FrameCallbackFunction;

class OpenNI2FrameListener;

class OpenNI2Device
{
public:
  OpenNI2Device(const std::string& device_URI) throw (OpenNI2Exception);
  virtual ~OpenNI2Device();

  const std::string getUri() const;
  const std::string getVendor() const;
  const std::string getName() const;
  uint16_t getUsbVendorId() const;
  uint16_t getUsbProductId() const;

  const std::string getStringID() const;

  bool isValid() const;

  bool hasIRSensor() const;
  bool hasColorSensor() const;
  bool hasDepthSensor() const;

  void startIRStream();
  void startColorStream();
  void startDepthStream();

  void stopAllStreams();

  void stopIRStream();
  void stopColorStream();
  void stopDepthStream();

  bool isIRStreamStarted();
  bool isColorStreamStarted();
  bool isDepthStreamStarted();

  bool isImageRegistrationModeSupported() const;
  void setImageRegistrationMode(bool enabled) throw (OpenNI2Exception);
  void setDepthColorSync(bool enabled) throw (OpenNI2Exception);

  const OpenNI2VideoMode getIRVideoMode() throw (OpenNI2Exception);
  const OpenNI2VideoMode getColorVideoMode() throw (OpenNI2Exception);
  const OpenNI2VideoMode getDepthVideoMode() throw (OpenNI2Exception);

  const std::vector<OpenNI2VideoMode>& getSupportedIRVideoModes() const;
  const std::vector<OpenNI2VideoMode>& getSupportedColorVideoModes() const;
  const std::vector<OpenNI2VideoMode>& getSupportedDepthVideoModes() const;

  bool isIRVideoModeSupported(const OpenNI2VideoMode& video_mode) const;
  bool isColorVideoModeSupported(const OpenNI2VideoMode& video_mode) const;
  bool isDepthVideoModeSupported(const OpenNI2VideoMode& video_mode) const;

  void setIRVideoMode(const OpenNI2VideoMode& video_mode) throw (OpenNI2Exception);
  void setColorVideoMode(const OpenNI2VideoMode& video_mode) throw (OpenNI2Exception);
  void setDepthVideoMode(const OpenNI2VideoMode& video_mode) throw (OpenNI2Exception);

  void setIRFrameCallback(FrameCallbackFunction callback);
  void setColorFrameCallback(FrameCallbackFunction callback);
  void setDepthFrameCallback(FrameCallbackFunction callback);

  float getIRFocalLength (int output_y_resolution) const;
  float getColorFocalLength (int output_y_resolution) const;
  float getDepthFocalLength (int output_y_resolution) const;

  void setAutoExposure(bool enable) throw (OpenNI2Exception);
  void setAutoWhiteBalance(bool enable) throw (OpenNI2Exception);

  bool getAutoExposure() const;
  bool getAutoWhiteBalance() const;

  void setUseDeviceTimer(bool enable);

protected:
  void shutdown();

  boost::shared_ptr<openni::VideoStream> getIRVideoStream() const throw (OpenNI2Exception);
  boost::shared_ptr<openni::VideoStream> getColorVideoStream() const throw (OpenNI2Exception);
  boost::shared_ptr<openni::VideoStream> getDepthVideoStream() const throw (OpenNI2Exception);

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

  bool image_registration_activated_;

  bool use_device_time_;

};

std::ostream& operator << (std::ostream& stream, const OpenNI2Device& device);

}

#endif /* OPENNI_DEVICE_H */
