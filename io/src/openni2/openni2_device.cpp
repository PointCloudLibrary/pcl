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

#include <OpenNI.h>
#include <PS1080.h> // For XN_STREAM_PROPERTY_EMITTER_DCMOS_DISTANCE property

#include <boost/algorithm/string/replace.hpp>

#include "pcl/io/openni2/openni2_device.h"
#include "pcl/io/openni2/openni2_convert.h"
#include "pcl/io/openni2/openni2_frame_listener.h"

#include "pcl/io/io_exception.h"

#include <string>

using namespace openni;
using namespace pcl::io::openni2;

using openni::VideoMode;
using std::vector;

pcl::io::openni2::OpenNI2Device::OpenNI2Device (const std::string& device_URI) :
  ir_video_started_(false),
  color_video_started_(false),
  depth_video_started_(false)
{
  openni::Status status = openni::OpenNI::initialize ();
  if (status != openni::STATUS_OK)
    THROW_IO_EXCEPTION ("Initialize failed\n%s\n", OpenNI::getExtendedError ());

  openni_device_.reset (new openni::Device);

  if (device_URI.length () > 0)
    status = openni_device_->open (device_URI.c_str ());
  else
    status = openni_device_->open (openni::ANY_DEVICE);

  if (status != openni::STATUS_OK)
    THROW_IO_EXCEPTION ("Initialize failed\n%s\n", openni::OpenNI::getExtendedError ());

  // Get depth calculation parameters
  // Some of these are device-spefic and may not exist
  baseline_ = 0.0;
  if ( getDepthVideoStream ()->isPropertySupported (XN_STREAM_PROPERTY_EMITTER_DCMOS_DISTANCE) )
  {
    double baseline;
    getDepthVideoStream ()->getProperty (XN_STREAM_PROPERTY_EMITTER_DCMOS_DISTANCE, &baseline); // Device specific -- from PS1080.h
    baseline_ = static_cast<float> (baseline * 0.01f);  // baseline from cm -> meters
  }
  shadow_value_ = 0;     // This does not exist in OpenNI 2, and is maintained for compatibility with the OpenNI 1.x grabber
  no_sample_value_ = 0;  // This does not exist in OpenNI 2, and is maintained for compatibility with the OpenNI 1.x grabber

  // Set default resolution if not reading a file
  if (!openni_device_->isFile ())
  {
    if (openni_device_->hasSensor (openni::SENSOR_COLOR))
    {
      setColorVideoMode (getDefaultColorMode ());
    }
    if (openni_device_->hasSensor (openni::SENSOR_DEPTH))
    {
      setDepthVideoMode (getDefaultDepthMode ());
    }
    if (openni_device_->hasSensor (openni::SENSOR_IR))
    {
      setIRVideoMode (getDefaultIRMode ());
    }
  }

  if (openni_device_->isFile ())
  {
    openni_device_->getPlaybackControl ()->setSpeed (1.0f);
  }

  device_info_.reset (new openni::DeviceInfo);
  *device_info_ = openni_device_->getDeviceInfo ();

  color_frame_listener.reset (new OpenNI2FrameListener);
  depth_frame_listener.reset (new OpenNI2FrameListener);
  ir_frame_listener.reset (new OpenNI2FrameListener);
}

pcl::io::openni2::OpenNI2Device::~OpenNI2Device ()
{
  stopAllStreams ();

  shutdown ();

  openni_device_->close ();
}

const std::string
pcl::io::openni2::OpenNI2Device::getUri () const
{
  return (std::string (device_info_->getUri ()));
}

const std::string
pcl::io::openni2::OpenNI2Device::getVendor () const
{
  return (std::string (device_info_->getVendor ()));
}

const std::string
pcl::io::openni2::OpenNI2Device::getName () const
{
  return (std::string (device_info_->getName ()));
}

std::uint16_t
pcl::io::openni2::OpenNI2Device::getUsbVendorId () const
{
  return (device_info_->getUsbVendorId ());
}

std::uint16_t
pcl::io::openni2::OpenNI2Device::getUsbProductId () const
{
  return (device_info_->getUsbProductId ());
}

const std::string
pcl::io::openni2::OpenNI2Device::getStringID () const
{
  std::string ID_str = getName () + "_" + getVendor ();

  boost::replace_all (ID_str, "/", "");
  boost::replace_all (ID_str, ".", "");
  boost::replace_all (ID_str, "@", "");

  return (ID_str);
}

bool
pcl::io::openni2::OpenNI2Device::isValid () const
{
  return (openni_device_.get () != nullptr) && openni_device_->isValid ();
}

float
pcl::io::openni2::OpenNI2Device::getIRFocalLength () const
{
  auto stream = getIRVideoStream ();

  int frameWidth = stream->getVideoMode ().getResolutionX ();
  float hFov = stream->getHorizontalFieldOfView ();
  float calculatedFocalLengthX = frameWidth / (2.0f * std::tan (hFov / 2.0f));
  return (calculatedFocalLengthX);
}

float
pcl::io::openni2::OpenNI2Device::getColorFocalLength () const
{
  auto stream = getColorVideoStream ();

  int frameWidth = stream->getVideoMode ().getResolutionX ();
  float hFov = stream->getHorizontalFieldOfView ();
  float calculatedFocalLengthX = frameWidth / (2.0f * std::tan (hFov / 2.0f));
  return (calculatedFocalLengthX);
}

float
pcl::io::openni2::OpenNI2Device::getDepthFocalLength () const
{
  auto stream = getDepthVideoStream ();

  int frameWidth = stream->getVideoMode ().getResolutionX ();
  float hFov = stream->getHorizontalFieldOfView ();
  float calculatedFocalLengthX = frameWidth / (2.0f * std::tan (hFov / 2.0f));
  return (calculatedFocalLengthX);
}

float
pcl::io::openni2::OpenNI2Device::getBaseline()
{
  return (baseline_);
}

std::uint64_t
pcl::io::openni2::OpenNI2Device::getShadowValue()
{
  return (shadow_value_);
}

bool
pcl::io::openni2::OpenNI2Device::isIRVideoModeSupported (const OpenNI2VideoMode& video_mode) const
{
  getSupportedIRVideoModes ();

  bool supported = false;

  std::vector<OpenNI2VideoMode>::const_iterator it = ir_video_modes_.begin ();
  std::vector<OpenNI2VideoMode>::const_iterator it_end = ir_video_modes_.end ();

  while (it != it_end && !supported)
  {
    supported = (*it == video_mode);
    ++it;
  }

  return (supported);
}

bool
pcl::io::openni2::OpenNI2Device::isColorVideoModeSupported (const OpenNI2VideoMode& video_mode) const
{
  getSupportedColorVideoModes ();

  bool supported = false;

  std::vector<OpenNI2VideoMode>::const_iterator it = color_video_modes_.begin ();
  std::vector<OpenNI2VideoMode>::const_iterator it_end = color_video_modes_.end ();

  while (it != it_end && !supported)
  {
    supported = (*it == video_mode);
    ++it;
  }

  return (supported);
}

bool
pcl::io::openni2::OpenNI2Device::isDepthVideoModeSupported (const OpenNI2VideoMode& video_mode) const
{
  getSupportedDepthVideoModes ();

  bool supported = false;

  std::vector<OpenNI2VideoMode>::const_iterator it = depth_video_modes_.begin ();
  std::vector<OpenNI2VideoMode>::const_iterator it_end = depth_video_modes_.end ();

  while (it != it_end && !supported)
  {
    supported = (*it == video_mode);
    ++it;
  }

  return (supported);
}

bool
pcl::io::openni2::OpenNI2Device::hasIRSensor () const
{
  return (openni_device_->hasSensor (openni::SENSOR_IR));
}

bool
pcl::io::openni2::OpenNI2Device::hasColorSensor () const
{
  return (openni_device_->hasSensor (openni::SENSOR_COLOR));
}

bool
pcl::io::openni2::OpenNI2Device::hasDepthSensor () const
{
  return (openni_device_->hasSensor (openni::SENSOR_DEPTH));
}

void
pcl::io::openni2::OpenNI2Device::startIRStream ()
{
  if (auto stream = getIRVideoStream ())
  {
    stream->setMirroringEnabled (false);
    stream->addNewFrameListener (ir_frame_listener.get ());
    stream->start ();
    ir_video_started_ = true;
  }
}

void
pcl::io::openni2::OpenNI2Device::startColorStream ()
{
  if (auto stream = getColorVideoStream ())
  {
    stream->setMirroringEnabled (false);
    stream->addNewFrameListener (color_frame_listener.get ());
    stream->start ();
    color_video_started_ = true;
  }
}
void
pcl::io::openni2::OpenNI2Device::startDepthStream ()
{
  if (auto stream = getDepthVideoStream ())
  {
    stream->setMirroringEnabled (false);
    stream->addNewFrameListener (depth_frame_listener.get ());
    stream->start ();
    depth_video_started_ = true;
  }
}

void
pcl::io::openni2::OpenNI2Device::stopAllStreams ()
{
  stopIRStream ();
  stopColorStream ();
  stopDepthStream ();
}

void
pcl::io::openni2::OpenNI2Device::stopIRStream ()
{
  if (ir_video_stream_.get () != nullptr)
  {
    ir_video_stream_->stop ();
    ir_video_started_ = false;
  }
}
void
pcl::io::openni2::OpenNI2Device::stopColorStream ()
{
  if (color_video_stream_.get () != nullptr)
  {
    color_video_stream_->stop ();
    color_video_started_ = false;
  }
}
void
pcl::io::openni2::OpenNI2Device::stopDepthStream ()
{
  if (depth_video_stream_.get () != nullptr)
  {
    depth_video_stream_->stop ();
    depth_video_started_ = false;
  }
}

void
pcl::io::openni2::OpenNI2Device::shutdown ()
{
  if (ir_video_stream_.get () != nullptr)
    ir_video_stream_->destroy ();

  if (color_video_stream_.get () != nullptr)
    color_video_stream_->destroy ();

  if (depth_video_stream_.get () != nullptr)
    depth_video_stream_->destroy ();

}

bool
pcl::io::openni2::OpenNI2Device::isIRStreamStarted ()
{
  return (ir_video_started_);
}
bool
pcl::io::openni2::OpenNI2Device::isColorStreamStarted ()
{
  return (color_video_started_);
}
bool
pcl::io::openni2::OpenNI2Device::isDepthStreamStarted ()
{
  return (depth_video_started_);
}

bool
pcl::io::openni2::OpenNI2Device::isImageRegistrationModeSupported () const
{
  return (openni_device_->isImageRegistrationModeSupported (openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR));
}

void
pcl::io::openni2::OpenNI2Device::setImageRegistrationMode (bool setEnable)
{
  bool registrationSupported = isImageRegistrationModeSupported ();
  if (registrationSupported)
  {
    if (setEnable)
    {
      openni::Status rc = openni_device_->setImageRegistrationMode (openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
      if (rc != openni::STATUS_OK)
        THROW_IO_EXCEPTION ("Enabling image registration mode failed: \n%s\n", openni::OpenNI::getExtendedError ());
    }
    else
    {
      openni::Status rc = openni_device_->setImageRegistrationMode (openni::IMAGE_REGISTRATION_OFF);
      if (rc != openni::STATUS_OK)
        THROW_IO_EXCEPTION ("Disabling image registration mode failed: \n%s\n", openni::OpenNI::getExtendedError ());
    }
  }
}

bool
pcl::io::openni2::OpenNI2Device::isDepthRegistered () const
{
  return openni_device_->getImageRegistrationMode () == openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR;
}

void
pcl::io::openni2::OpenNI2Device::setSynchronization (bool enabled)
{
  openni::Status rc = openni_device_->setDepthColorSyncEnabled (enabled);
  if (rc != openni::STATUS_OK)
    THROW_IO_EXCEPTION ("Enabling depth color synchronization failed: \n%s\n", openni::OpenNI::getExtendedError ());
}

const OpenNI2VideoMode
pcl::io::openni2::OpenNI2Device::getIRVideoMode ()
{
  if (auto stream = getIRVideoStream ())
    return openniModeToGrabberMode (stream->getVideoMode ());
  THROW_IO_EXCEPTION ("Could not create video stream.");
  return {};
}

const OpenNI2VideoMode
pcl::io::openni2::OpenNI2Device::getColorVideoMode ()
{
  if (auto stream = getColorVideoStream ())
    return openniModeToGrabberMode (stream->getVideoMode ());
  THROW_IO_EXCEPTION ("Could not create video stream.");
  return {};
}

const OpenNI2VideoMode
pcl::io::openni2::OpenNI2Device::getDepthVideoMode ()
{
  if (auto stream = getDepthVideoStream ())
    return openniModeToGrabberMode (stream->getVideoMode ());
  THROW_IO_EXCEPTION ("Could not create video stream.");
  return {};
}

void
pcl::io::openni2::OpenNI2Device::setIRVideoMode (const OpenNI2VideoMode& video_mode)
{
  if (auto stream = getIRVideoStream ())
  {
    const openni::VideoMode videoMode = grabberModeToOpenniMode (video_mode);
    const openni::Status rc = stream->setVideoMode (videoMode);
    if (rc != openni::STATUS_OK)
      THROW_IO_EXCEPTION ("Couldn't set IR video mode: \n%s\n", openni::OpenNI::getExtendedError ());
  }
}

void
pcl::io::openni2::OpenNI2Device::setColorVideoMode (const OpenNI2VideoMode& video_mode)
{
  if (auto stream = getColorVideoStream ())
  {
    openni::VideoMode videoMode = grabberModeToOpenniMode (video_mode);
    const openni::Status rc = stream->setVideoMode (videoMode);
    if (rc != openni::STATUS_OK)
      THROW_IO_EXCEPTION ("Couldn't set color video mode: \n%s\n", openni::OpenNI::getExtendedError ());
  }
}

void
pcl::io::openni2::OpenNI2Device::setDepthVideoMode (const OpenNI2VideoMode& video_mode)
{
  if (auto stream = getDepthVideoStream ())
  {
    const openni::VideoMode videoMode = grabberModeToOpenniMode (video_mode);
    const openni::Status rc = stream->setVideoMode (videoMode);
    if (rc != openni::STATUS_OK)
      THROW_IO_EXCEPTION ("Couldn't set depth video mode: \n%s\n", openni::OpenNI::getExtendedError ());
  }
}

OpenNI2VideoMode
pcl::io::openni2::OpenNI2Device::getDefaultIRMode () const
{
  // Search for and return VGA@30 Hz mode
  std::vector<OpenNI2VideoMode> modeList = getSupportedIRVideoModes ();
  for (const auto &mode : modeList)
  {
    if ( (mode.x_resolution_ == 640) && (mode.y_resolution_ == 480) && (mode.frame_rate_ == 30.0) )
      return mode;
  }
  if (modeList.empty())
      THROW_IO_EXCEPTION("Device claims to have a IR sensor, but doesn't have any IR streaming mode");
  return (modeList.at (0)); // Return first mode if we can't find VGA
}

OpenNI2VideoMode
pcl::io::openni2::OpenNI2Device::getDefaultColorMode () const
{
  // Search for and return VGA@30 Hz mode
  std::vector<OpenNI2VideoMode> modeList = getSupportedColorVideoModes ();
  for (const auto &mode : modeList)
  {
    if ( (mode.x_resolution_ == 640) && (mode.y_resolution_ == 480) && (mode.frame_rate_ == 30.0) )
      return mode;
  }
  if (modeList.empty())
      THROW_IO_EXCEPTION("Device claims to have a color sensor, but doesn't have any color streaming mode");
  return (modeList.at (0)); // Return first mode if we can't find VGA
}

OpenNI2VideoMode
pcl::io::openni2::OpenNI2Device::getDefaultDepthMode () const
{
  // Search for and return VGA@30 Hz mode
  std::vector<OpenNI2VideoMode> modeList = getSupportedDepthVideoModes ();
  for (const auto &mode : modeList)
  {
    if ( (mode.x_resolution_ == 640) && (mode.y_resolution_ == 480) && (mode.frame_rate_ == 30.0) )
      return mode;
  }
  if (modeList.empty())
      THROW_IO_EXCEPTION("Device claims to have a depth sensor, but doesn't have any depth streaming mode");
  return (modeList.at (0)); // Return first mode if we can't find VGA
}

const std::vector<OpenNI2VideoMode>&
pcl::io::openni2::OpenNI2Device::getSupportedIRVideoModes () const
{
  ir_video_modes_.clear ();

  if (auto stream = getIRVideoStream ())
  {
    const openni::SensorInfo& sensor_info = stream->getSensorInfo ();
    ir_video_modes_ = openniModeToGrabberMode (sensor_info.getSupportedVideoModes ());
  }

  return (ir_video_modes_);
}

const std::vector<OpenNI2VideoMode>&
pcl::io::openni2::OpenNI2Device::getSupportedColorVideoModes () const
{
  color_video_modes_.clear ();

  if (auto stream = getColorVideoStream ())
  {
    const openni::SensorInfo& sensor_info = stream->getSensorInfo ();
    color_video_modes_ = openniModeToGrabberMode (sensor_info.getSupportedVideoModes ());
  }

  return (color_video_modes_);
}

const std::vector<OpenNI2VideoMode>&
pcl::io::openni2::OpenNI2Device::getSupportedDepthVideoModes () const
{
  depth_video_modes_.clear ();

  if (auto stream = getDepthVideoStream ())
  {
    const openni::SensorInfo& sensor_info = stream->getSensorInfo ();
    depth_video_modes_ = openniModeToGrabberMode (sensor_info.getSupportedVideoModes ());
  }

  return (depth_video_modes_);
}

bool
pcl::io::openni2::OpenNI2Device::findCompatibleIRMode (const OpenNI2VideoMode& requested_mode, OpenNI2VideoMode& actual_mode) const
{
  if ( isIRVideoModeSupported (requested_mode) )
  {
    actual_mode = requested_mode;
    return (true);
  }
  // Find a resize-compatable mode
  std::vector<OpenNI2VideoMode> supportedModes = getSupportedIRVideoModes ();
  bool found = findCompatibleVideoMode (supportedModes, requested_mode, actual_mode);
  return (found);
}

bool
pcl::io::openni2::OpenNI2Device::findCompatibleColorMode (const OpenNI2VideoMode& requested_mode, OpenNI2VideoMode& actual_mode) const
{
  if ( isColorVideoModeSupported (requested_mode) )
  {
    actual_mode = requested_mode;
    return (true);
  }
  // Find a resize-compatable mode
  std::vector<OpenNI2VideoMode> supportedModes = getSupportedColorVideoModes ();
  bool found = findCompatibleVideoMode (supportedModes, requested_mode, actual_mode);
  return (found);
}

bool
pcl::io::openni2::OpenNI2Device::findCompatibleDepthMode (const OpenNI2VideoMode& requested_mode, OpenNI2VideoMode& actual_mode) const
{
  if ( isDepthVideoModeSupported (requested_mode) )
  {
    actual_mode = requested_mode;
    return (true);
  }
  // Find a resize-compatable mode
  std::vector<OpenNI2VideoMode> supportedModes = getSupportedDepthVideoModes ();
  bool found = findCompatibleVideoMode (supportedModes, requested_mode, actual_mode);
  return (found);
}

// Generic support method for the above findCompatable...Mode calls above
bool
pcl::io::openni2::OpenNI2Device::findCompatibleVideoMode (const std::vector<OpenNI2VideoMode>& supportedModes, const OpenNI2VideoMode& requested_mode, OpenNI2VideoMode& actual_mode) const
{
  bool found = false;
  for (const auto &supportedMode : supportedModes)
  {
    if (supportedMode.frame_rate_ == requested_mode.frame_rate_
      && resizingSupported (supportedMode.x_resolution_, supportedMode.y_resolution_, requested_mode.x_resolution_, requested_mode.y_resolution_))
    {
      if (found)
      { // check whether the new mode is better -> smaller than the current one.
        if (actual_mode.x_resolution_ * actual_mode.x_resolution_ > supportedMode.x_resolution_ * supportedMode.y_resolution_ )
          actual_mode = supportedMode;
      }
      else
      {
        actual_mode = supportedMode;
        found = true;
      }
    }
  }
  return (found);
}

bool
pcl::io::openni2::OpenNI2Device::resizingSupported (std::size_t input_width, std::size_t input_height, std::size_t output_width, std::size_t output_height) const
{
  return (output_width <= input_width && output_height <= input_height && input_width % output_width == 0 && input_height % output_height == 0 );
}

void
pcl::io::openni2::OpenNI2Device::setAutoExposure (bool enable)
{
  if (auto stream = getColorVideoStream ())
  {
    openni::CameraSettings* camera_seeting = stream->getCameraSettings ();
    if (camera_seeting)
    {
      const openni::Status rc = camera_seeting->setAutoExposureEnabled (enable);
      if (rc != openni::STATUS_OK)
        THROW_IO_EXCEPTION ("Couldn't set auto exposure: \n%s\n", openni::OpenNI::getExtendedError ());
    }

  }
}

void
pcl::io::openni2::OpenNI2Device::setAutoWhiteBalance (bool enable)
{
  if (auto stream = getColorVideoStream ())
  {
    openni::CameraSettings* camera_seeting = stream->getCameraSettings ();
    if (camera_seeting)
    {
      const openni::Status rc = camera_seeting->setAutoWhiteBalanceEnabled (enable);
      if (rc != openni::STATUS_OK)
        THROW_IO_EXCEPTION ("Couldn't set auto white balance: \n%s\n", openni::OpenNI::getExtendedError ());
    }
  }
}

bool
pcl::io::openni2::OpenNI2Device::getAutoExposure () const
{
  bool ret = false;

  if (auto stream = getColorVideoStream ())
  {
    openni::CameraSettings* camera_seeting = stream->getCameraSettings ();
    if (camera_seeting)
      ret = camera_seeting->getAutoExposureEnabled ();
  }

  return (ret);
}

bool
pcl::io::openni2::OpenNI2Device::getAutoWhiteBalance () const
{
  bool ret = false;

  if (auto stream = getColorVideoStream ())
  {
    openni::CameraSettings* camera_setting = stream->getCameraSettings ();
    if (camera_setting)
      ret = camera_setting->getAutoWhiteBalanceEnabled ();
  }

  return (ret);
}

int pcl::io::openni2::OpenNI2Device::getDepthFrameCount ()
{
  if (!openni_device_->isFile () || !getDepthVideoStream ())
  {
    return 0;
  }
  return openni_device_->getPlaybackControl ()->getNumberOfFrames(*getDepthVideoStream ());
}

int OpenNI2Device::getColorFrameCount ()
{
  if (!openni_device_->isFile () || !getColorVideoStream ())
  {
    return 0;
  }
  return openni_device_->getPlaybackControl ()->getNumberOfFrames (*getColorVideoStream ());
}

int OpenNI2Device::getIRFrameCount ()
{
  if (!openni_device_->isFile () || !getIRVideoStream ())
  {
    return 0;
  }
  return openni_device_->getPlaybackControl ()->getNumberOfFrames (*getIRVideoStream ());
}

bool OpenNI2Device::setPlaybackSpeed (double speed)
{
    return openni_device_->getPlaybackControl ()->setSpeed (speed) == openni::STATUS_OK;
}

std::shared_ptr<openni::VideoStream>
pcl::io::openni2::OpenNI2Device::getIRVideoStream () const
{
  if (ir_video_stream_.get () == nullptr)
  {
    if (hasIRSensor ())
    {
      ir_video_stream_.reset (new openni::VideoStream);

      const openni::Status rc = ir_video_stream_->create (*openni_device_, openni::SENSOR_IR);
      if (rc != openni::STATUS_OK)
        THROW_IO_EXCEPTION ("Couldn't create IR video stream: \n%s\n", openni::OpenNI::getExtendedError ());
    }
  }
  return (ir_video_stream_);
}

std::shared_ptr<openni::VideoStream>
pcl::io::openni2::OpenNI2Device::getColorVideoStream () const
{
  if (color_video_stream_.get () == nullptr)
  {
    if (hasColorSensor ())
    {
      color_video_stream_.reset (new openni::VideoStream);

      const openni::Status rc = color_video_stream_->create (*openni_device_, openni::SENSOR_COLOR);
      if (rc != openni::STATUS_OK)
        THROW_IO_EXCEPTION ("Couldn't create color video stream: \n%s\n", openni::OpenNI::getExtendedError ());
    }
  }
  return (color_video_stream_);
}

std::shared_ptr<openni::VideoStream>
pcl::io::openni2::OpenNI2Device::getDepthVideoStream () const
{
  if (depth_video_stream_.get () == nullptr)
  {
    if (hasDepthSensor ())
    {
      depth_video_stream_.reset (new openni::VideoStream);

      const openni::Status rc = depth_video_stream_->create (*openni_device_, openni::SENSOR_DEPTH);
      if (rc != openni::STATUS_OK)
        THROW_IO_EXCEPTION ("Couldn't create depth video stream: \n%s\n", openni::OpenNI::getExtendedError ());
    }
  }
  return (depth_video_stream_);
}

std::ostream& pcl::io::openni2::operator<< (std::ostream& stream, const OpenNI2Device& device)
{

  stream << "Device info (" << device.getUri () << ")" << std::endl;
  stream << "   Vendor: " << device.getVendor () << std::endl;
  stream << "   Name: " << device.getName () << std::endl;
  stream << "   USB Vendor ID: " << device.getUsbVendorId () << std::endl;
  stream << "   USB Product ID: " << device.getUsbVendorId () << std::endl << std::endl;

  if (device.hasIRSensor ())
  {
    stream << "IR sensor video modes:" << std::endl;
    const std::vector<OpenNI2VideoMode>& video_modes = device.getSupportedIRVideoModes ();

    for (const auto &video_mode : video_modes)
      stream << "   - " << video_mode << std::endl;
  }
  else
  {
    stream << "No IR sensor available" << std::endl;
  }

  if (device.hasColorSensor ())
  {
    stream << "Color sensor video modes:" << std::endl;
    const std::vector<OpenNI2VideoMode>& video_modes = device.getSupportedColorVideoModes ();

    for (const auto &video_mode : video_modes)
      stream << "   - " << video_mode << std::endl;
  }
  else
  {
    stream << "No Color sensor available" << std::endl;
  }

  if (device.hasDepthSensor ())
  {
    stream << "Depth sensor video modes:" << std::endl;
    const std::vector<OpenNI2VideoMode>& video_modes = device.getSupportedDepthVideoModes ();

    for (const auto &video_mode : video_modes)
      stream << "   - " << video_mode << std::endl;
  }
  else
  {
    stream << "No Depth sensor available" << std::endl;
  }

  return (stream);

}

void
pcl::io::openni2::OpenNI2Device::setColorCallback (StreamCallbackFunction color_callback)
{
  color_frame_listener->setCallback (std::move(color_callback));
}

void
pcl::io::openni2::OpenNI2Device::setDepthCallback (StreamCallbackFunction depth_callback)
{
  depth_frame_listener->setCallback (std::move(depth_callback));
}

void
pcl::io::openni2::OpenNI2Device::setIRCallback (StreamCallbackFunction ir_callback)
{
  ir_frame_listener->setCallback (std::move(ir_callback));
}
