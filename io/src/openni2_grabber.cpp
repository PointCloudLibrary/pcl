/*
 * Software License Agreement (BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2009-2012, Willow Garage, Inc.
 * Copyright (c) 2012-, Open Perception, Inc.
 * Copyright (c) 2014, respective authors.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of the copyright holder(s) nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <pcl/pcl_config.h>
#ifdef HAVE_OPENNI2

#include <pcl/io/openni2_grabber.h>
#include <pcl/io/openni2/openni2_device_manager.h>
#include <pcl/io/openni2/openni2_metadata_wrapper.h>
#include <pcl/io/openni2/openni2_device.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/exceptions.h>
#include <iostream>
#include <boost/filesystem.hpp> // for exists

using namespace pcl::io::openni2;

namespace
{
  // Treat color as chars, float32, or uint32
  union RGBValue
  {
    struct
    {
      unsigned char Blue;
      unsigned char Green;
      unsigned char Red;
      unsigned char Alpha;
    };
    float float_value;
    std::uint32_t long_value;
  };
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
pcl::io::OpenNI2Grabber::OpenNI2Grabber (const std::string& device_id, const Mode& depth_mode, const Mode& image_mode)
  : color_resize_buffer_(0)
  , depth_resize_buffer_(0)
  , ir_resize_buffer_(0)
  , image_width_ ()
  , image_height_ ()
  , depth_width_ ()
  , depth_height_ ()
  , image_required_ (false)
  , depth_required_ (false)
  , ir_required_ (false)
  , sync_required_ (false)
  , image_signal_ (), depth_image_signal_ (), ir_image_signal_ (), image_depth_image_signal_ ()
  , ir_depth_image_signal_ (), point_cloud_signal_ (), point_cloud_i_signal_ ()
  , point_cloud_rgb_signal_ (), point_cloud_rgba_signal_ ()
  , depth_callback_handle_ (), image_callback_handle_ (), ir_callback_handle_ ()
  , running_ (false)
  , rgb_parameters_(std::numeric_limits<double>::quiet_NaN () )
  , depth_parameters_(std::numeric_limits<double>::quiet_NaN () )
{
  // initialize driver
  updateModeMaps (); // registering mapping from PCL enum modes to openni::VideoMode and vice versa
  setupDevice (device_id, depth_mode, image_mode);

  rgb_frame_id_ = "/openni2_rgb_optical_frame";
  depth_frame_id_ = "/openni2_depth_optical_frame";


  if (!device_->hasDepthSensor () )
    PCL_THROW_EXCEPTION (pcl::IOException, "Device does not provide 3D information.");

  depth_image_signal_    = createSignal<sig_cb_openni_depth_image> ();
  ir_image_signal_       = createSignal<sig_cb_openni_ir_image> ();
  point_cloud_signal_    = createSignal<sig_cb_openni_point_cloud> ();
  point_cloud_i_signal_  = createSignal<sig_cb_openni_point_cloud_i> ();
  ir_depth_image_signal_ = createSignal<sig_cb_openni_ir_depth_image> ();
  ir_sync_.addCallback ([this] (const IRImage::Ptr& ir_image,
                                const DepthImage::Ptr& depth_image,
                                unsigned long,
                                unsigned long)
  {
    irDepthImageCallback (ir_image, depth_image);
  });

  if (device_->hasColorSensor ())
  {
    // create callback signals
    image_signal_             = createSignal<sig_cb_openni_image> ();
    image_depth_image_signal_ = createSignal<sig_cb_openni_image_depth_image> ();
    point_cloud_rgb_signal_   = createSignal<sig_cb_openni_point_cloud_rgb> ();
    point_cloud_rgba_signal_  = createSignal<sig_cb_openni_point_cloud_rgba> ();
    rgb_sync_.addCallback ([this] (const Image::Ptr& image,
                                   const DepthImage::Ptr& depth_image,
                                   unsigned long,
                                   unsigned long)
    {
      imageDepthImageCallback (image, depth_image);
    });
  }

  // callbacks from the sensor to the grabber
  device_->setColorCallback ([this] (openni::VideoStream& stream) { processColorFrame (stream); });
  device_->setDepthCallback ([this] (openni::VideoStream& stream) { processDepthFrame (stream); });
  device_->setIRCallback ([this] (openni::VideoStream& stream) { processIRFrame (stream); });
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
pcl::io::OpenNI2Grabber::~OpenNI2Grabber () noexcept
{
  try
  {
    stop ();

    // release the pointer to the device object
    device_.reset ();

    // disconnect all listeners
    disconnect_all_slots<sig_cb_openni_image> ();
    disconnect_all_slots<sig_cb_openni_depth_image> ();
    disconnect_all_slots<sig_cb_openni_ir_image> ();
    disconnect_all_slots<sig_cb_openni_image_depth_image> ();
    disconnect_all_slots<sig_cb_openni_point_cloud> ();
    disconnect_all_slots<sig_cb_openni_point_cloud_rgb> ();
    disconnect_all_slots<sig_cb_openni_point_cloud_rgba> ();
    disconnect_all_slots<sig_cb_openni_point_cloud_i> ();
  }
  catch (...)
  {
    // destructor never throws
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::io::OpenNI2Grabber::checkImageAndDepthSynchronizationRequired ()
{
  // do we have anyone listening to images or color point clouds?
  if (num_slots<sig_cb_openni_point_cloud_rgb>   () > 0 ||
    num_slots<sig_cb_openni_point_cloud_rgba>  () > 0 ||
    num_slots<sig_cb_openni_image_depth_image> () > 0)
    sync_required_ = true;
  else
    sync_required_ = false;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::io::OpenNI2Grabber::checkImageStreamRequired ()
{
  // do we have anyone listening to images or color point clouds?
  if (num_slots<sig_cb_openni_image>             () > 0 ||
    num_slots<sig_cb_openni_image_depth_image> () > 0 ||
    num_slots<sig_cb_openni_point_cloud_rgba>  () > 0 ||
    num_slots<sig_cb_openni_point_cloud_rgb>   () > 0)
    image_required_ = true;
  else
    image_required_ = false;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::io::OpenNI2Grabber::checkDepthStreamRequired ()
{
  // do we have anyone listening to depth images or (color) point clouds?
  if (num_slots<sig_cb_openni_depth_image>       () > 0 ||
    num_slots<sig_cb_openni_image_depth_image> () > 0 ||
    num_slots<sig_cb_openni_ir_depth_image>    () > 0 ||
    num_slots<sig_cb_openni_point_cloud_rgb>   () > 0 ||
    num_slots<sig_cb_openni_point_cloud_rgba>  () > 0 ||
    num_slots<sig_cb_openni_point_cloud>       () > 0 ||
    num_slots<sig_cb_openni_point_cloud_i>     () > 0 )
    depth_required_ = true;
  else
    depth_required_ = false;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::io::OpenNI2Grabber::checkIRStreamRequired ()
{
  if (num_slots<sig_cb_openni_ir_image>       () > 0 ||
    num_slots<sig_cb_openni_point_cloud_i>  () > 0 ||
    num_slots<sig_cb_openni_ir_depth_image> () > 0)
    ir_required_ = true;
  else
    ir_required_ = false;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::io::OpenNI2Grabber::start ()
{
  try
  {
    // check if we need to start/stop any stream
    if (image_required_ && !device_->isColorStreamStarted () && device_->hasColorSensor ())
    {
      block_signals ();
      device_->startColorStream ();
      startSynchronization ();
    }

    if (depth_required_ && !device_->isDepthStreamStarted ())
    {
      block_signals ();
      if (device_->hasColorSensor () && device_->isImageRegistrationModeSupported () )
      {
        device_->setImageRegistrationMode (true);
      }
      device_->startDepthStream ();
      startSynchronization ();
    }

    if (ir_required_ && !device_->isIRStreamStarted () )
    {
      block_signals ();
      device_->startIRStream ();
    }
    running_ = true;
  }
  catch (IOException& ex)
  {
    PCL_THROW_EXCEPTION (pcl::IOException, "Could not start streams. Reason: " << ex.what ());
  }

  unblock_signals ();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::io::OpenNI2Grabber::stop ()
{
  try
  {
    if (device_->hasDepthSensor () && device_->isDepthStreamStarted () )
      device_->stopDepthStream ();

    if (device_->hasColorSensor () && device_->isColorStreamStarted () )
      device_->stopColorStream ();

    if (device_->hasIRSensor () && device_->isIRStreamStarted ())
      device_->stopIRStream ();

    running_ = false;
  }
  catch (IOException& ex)
  {
    PCL_THROW_EXCEPTION (pcl::IOException, "Could not stop streams. Reason: " << ex.what ());
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::io::OpenNI2Grabber::isRunning () const
{
  return (running_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::io::OpenNI2Grabber::signalsChanged ()
{
  // reevaluate which streams are required
  checkImageStreamRequired ();
  checkDepthStreamRequired ();
  checkIRStreamRequired ();
  if (ir_required_ && image_required_)
    PCL_THROW_EXCEPTION (pcl::IOException, "Can not provide IR stream and RGB stream at the same time.");

  checkImageAndDepthSynchronizationRequired ();
  if (running_)
    start ();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::string
pcl::io::OpenNI2Grabber::getName () const
{
  return (std::string ("OpenNI2Grabber"));
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::io::OpenNI2Grabber::setupDevice (const std::string& device_id, const Mode& depth_mode, const Mode& image_mode)
{
  // Initialize the openni device
  auto deviceManager = OpenNI2DeviceManager::getInstance ();

  try
  {
    if (boost::filesystem::exists (device_id))
    {
      device_ = deviceManager->getFileDevice (device_id);	// Treat as file path
    }
    else if (deviceManager->getNumOfConnectedDevices () == 0)
    {
      PCL_THROW_EXCEPTION (pcl::IOException, "No devices connected.");
    }
    else if (device_id[0] == '#')
    {
      unsigned index = atoi (device_id.c_str () + 1);
      device_ = deviceManager->getDeviceByIndex (index - 1);
    }
    else
    {
      device_ = deviceManager->getDevice (device_id);
    }
  }
  catch (const IOException& exception)
  {
    if (!device_)
      PCL_THROW_EXCEPTION (pcl::IOException, "No matching device found. " << exception.what ())
    else
      PCL_THROW_EXCEPTION (pcl::IOException, "could not retrieve device. Reason " << exception.what ())
  }
  catch (const pcl::IOException&)
  {
    throw;
  }
  catch (...)
  {
    PCL_THROW_EXCEPTION (pcl::IOException, "unknown error occurred");
  }

  using VideoMode = pcl::io::openni2::OpenNI2VideoMode;

  VideoMode depth_md;
  // Set the selected output mode
  if (depth_mode != OpenNI_Default_Mode)
  {
    VideoMode actual_depth_md;
    if (!mapMode2XnMode (depth_mode, depth_md) || !device_->findCompatibleDepthMode (depth_md, actual_depth_md))
      PCL_THROW_EXCEPTION (pcl::IOException, "could not find compatible depth stream mode " << static_cast<int> (depth_mode) );

    VideoMode current_depth_md =  device_->getDepthVideoMode ();
    if (current_depth_md.x_resolution_ != actual_depth_md.x_resolution_ || current_depth_md.y_resolution_ != actual_depth_md.y_resolution_)
      device_->setDepthVideoMode (actual_depth_md);
  }
  else
  {
    depth_md = device_->getDefaultDepthMode ();
  }

  depth_width_ = depth_md.x_resolution_;
  depth_height_ = depth_md.y_resolution_;

  if (device_->hasColorSensor ())
  {
    VideoMode image_md;
    if (image_mode != OpenNI_Default_Mode)
    {
      VideoMode actual_image_md;
      if (!mapMode2XnMode (image_mode, image_md) || !device_->findCompatibleColorMode (image_md, actual_image_md))
        PCL_THROW_EXCEPTION (pcl::IOException, "could not find compatible image stream mode " << static_cast<int> (image_mode) );

      VideoMode current_image_md =  device_->getColorVideoMode ();
      if (current_image_md.x_resolution_ != actual_image_md.x_resolution_ || current_image_md.y_resolution_ != actual_image_md.y_resolution_)
        device_->setColorVideoMode (actual_image_md);
    }
    else
    {
      image_md = device_->getDefaultColorMode ();
    }

    image_width_  = image_md.x_resolution_;
    image_height_ = image_md.y_resolution_;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::io::OpenNI2Grabber::startSynchronization ()
{
  try
  {
    if (device_->hasColorSensor () && (device_->isSynchronizationSupported () && !device_->isSynchronized () && !device_->isFile () &&
        device_->getColorVideoMode ().frame_rate_ == device_->getDepthVideoMode ().frame_rate_))
      device_->setSynchronization (true);
  }
  catch (const IOException& exception)
  {
    std::cerr << exception.what() << std::endl;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::io::OpenNI2Grabber::stopSynchronization ()
{
  try
  {
    if (device_->isSynchronizationSupported () && device_->isSynchronized ())
      device_->setSynchronization (false);
  }
  catch (const IOException& exception)
  {
    PCL_THROW_EXCEPTION (pcl::IOException, "Could not start synchronization " << exception.what ());
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::io::OpenNI2Grabber::imageCallback (Image::Ptr image, void*)
{
  if (num_slots<sig_cb_openni_point_cloud_rgb>   () > 0 ||
    num_slots<sig_cb_openni_point_cloud_rgba>  () > 0 ||
    num_slots<sig_cb_openni_image_depth_image> () > 0)
    rgb_sync_.add0 (image, image->getTimestamp ());

  int numImageSlots = image_signal_->num_slots ();
  if (numImageSlots > 0)
    image_signal_->operator ()(image);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::io::OpenNI2Grabber::depthCallback (DepthImage::Ptr depth_image, void*)
{
  if (num_slots<sig_cb_openni_point_cloud_rgb>   () > 0 ||
    num_slots<sig_cb_openni_point_cloud_rgba>  () > 0 ||
    num_slots<sig_cb_openni_image_depth_image> () > 0)
    rgb_sync_.add1 (depth_image, depth_image->getTimestamp ());

  if (num_slots<sig_cb_openni_point_cloud_i>  () > 0 ||
    num_slots<sig_cb_openni_ir_depth_image> () > 0)
    ir_sync_.add1 (depth_image, depth_image->getTimestamp ());

  if (depth_image_signal_->num_slots () > 0)
    depth_image_signal_->operator ()(depth_image);

  if (point_cloud_signal_->num_slots () > 0)
    point_cloud_signal_->operator ()(convertToXYZPointCloud (depth_image));
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::io::OpenNI2Grabber::irCallback (IRImage::Ptr ir_image, void*)
{
  if (num_slots<sig_cb_openni_point_cloud_i>  () > 0 ||
    num_slots<sig_cb_openni_ir_depth_image> () > 0)
    ir_sync_.add0(ir_image, ir_image->getTimestamp ());

  if (ir_image_signal_->num_slots () > 0)
    ir_image_signal_->operator ()(ir_image);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::io::OpenNI2Grabber::imageDepthImageCallback (const Image::Ptr &image,
  const DepthImage::Ptr &depth_image)
{
  // check if we have color point cloud slots
  if (point_cloud_rgb_signal_->num_slots () > 0)
  {
    PCL_WARN ("PointXYZRGB callbacks deprecated. Use PointXYZRGBA instead.\n");
    point_cloud_rgb_signal_->operator ()(convertToXYZRGBPointCloud<pcl::PointXYZRGB> (image, depth_image));
  }

  if (point_cloud_rgba_signal_->num_slots () > 0)
    point_cloud_rgba_signal_->operator ()(convertToXYZRGBPointCloud<pcl::PointXYZRGBA> (image, depth_image));

  if (image_depth_image_signal_->num_slots () > 0)
  {
    float reciprocalFocalLength = 1.0f / device_->getDepthFocalLength ();
    image_depth_image_signal_->operator ()(image, depth_image, reciprocalFocalLength);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::io::OpenNI2Grabber::irDepthImageCallback (const IRImage::Ptr &ir_image,
  const DepthImage::Ptr &depth_image)
{
  // check if we have color point cloud slots
  if (point_cloud_i_signal_->num_slots () > 0)
    point_cloud_i_signal_->operator ()(convertToXYZIPointCloud (ir_image, depth_image));

  if (ir_depth_image_signal_->num_slots () > 0)
  {
    float reciprocalFocalLength = 1.0f / device_->getDepthFocalLength ();
    ir_depth_image_signal_->operator ()(ir_image, depth_image, reciprocalFocalLength);
  }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
pcl::PointCloud<pcl::PointXYZ>::Ptr
pcl::io::OpenNI2Grabber::convertToXYZPointCloud (const DepthImage::Ptr& depth_image)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud <pcl::PointXYZ>);

  cloud->header.seq = depth_image->getFrameID ();
  cloud->header.stamp = depth_image->getTimestamp ();
  cloud->height = depth_height_;
  cloud->width = depth_width_;
  cloud->is_dense = false;

  cloud->points.resize (cloud->height * cloud->width);

  float constant_x = 1.0f / device_->getDepthFocalLength ();
  float constant_y = 1.0f / device_->getDepthFocalLength ();
  float centerX = ((float)cloud->width - 1.f) / 2.f;
  float centerY = ((float)cloud->height - 1.f) / 2.f;

  if (std::isfinite (depth_parameters_.focal_length_x))
    constant_x =  1.0f / static_cast<float> (depth_parameters_.focal_length_x);

  if (std::isfinite (depth_parameters_.focal_length_y))
    constant_y =  1.0f / static_cast<float> (depth_parameters_.focal_length_y);

  if (std::isfinite (depth_parameters_.principal_point_x))
    centerX =  static_cast<float> (depth_parameters_.principal_point_x);

  if (std::isfinite (depth_parameters_.principal_point_y))
    centerY =  static_cast<float> (depth_parameters_.principal_point_y);

  if ( device_->isDepthRegistered() )
    cloud->header.frame_id = rgb_frame_id_;
  else
    cloud->header.frame_id = depth_frame_id_;


  float bad_point = std::numeric_limits<float>::quiet_NaN ();

  const auto* depth_map = reinterpret_cast<const std::uint16_t*>(depth_image->getData ());
  if (depth_image->getWidth () != depth_width_ || depth_image->getHeight () != depth_height_)
  {
    // Resize the image if nessacery
    depth_resize_buffer_.resize(depth_width_ * depth_height_);

    depth_image->fillDepthImageRaw (depth_width_, depth_height_, reinterpret_cast<std::uint16_t*>(depth_resize_buffer_.data()) );
    depth_map = depth_resize_buffer_.data();
  }

  unsigned depth_idx = 0;
  for (unsigned v = 0; v < depth_height_; ++v)
  {
    for (unsigned u = 0; u < depth_width_; ++u, ++depth_idx)
    {
      pcl::PointXYZ& pt = (*cloud)[depth_idx];
      // Check for invalid measurements
      if (depth_map[depth_idx] == 0 ||
        depth_map[depth_idx] == depth_image->getNoSampleValue () ||
        depth_map[depth_idx] == depth_image->getShadowValue ())
      {
        // not valid
        pt.x = pt.y = pt.z = bad_point;
        continue;
      }
      pt.z = depth_map[depth_idx] * 0.001f;
      pt.x = (static_cast<float> (u) - centerX) * pt.z * constant_x;
      pt.y = (static_cast<float> (v) - centerY) * pt.z * constant_y;
    }
  }
  cloud->sensor_origin_.setZero ();
  cloud->sensor_orientation_.w () = 1.0f;
  cloud->sensor_orientation_.x () = 0.0f;
  cloud->sensor_orientation_.y () = 0.0f;
  cloud->sensor_orientation_.z () = 0.0f;
  return (cloud);
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> typename pcl::PointCloud<PointT>::Ptr
pcl::io::OpenNI2Grabber::convertToXYZRGBPointCloud (const Image::Ptr &image, const DepthImage::Ptr &depth_image)
{
  typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

  cloud->header.seq = depth_image->getFrameID ();
  cloud->header.stamp = depth_image->getTimestamp ();
  cloud->header.frame_id = rgb_frame_id_;
  cloud->height = std::max (image_height_, depth_height_);
  cloud->width = std::max (image_width_, depth_width_);
  cloud->is_dense = false;

  cloud->points.resize (cloud->height * cloud->width);

  // Generate default camera parameters
  float fx = device_->getDepthFocalLength (); // Horizontal focal length
  float fy = device_->getDepthFocalLength (); // Vertcal focal length
  float cx = ((float)depth_width_ - 1.f) / 2.f;  // Center x
  float cy = ((float)depth_height_- 1.f) / 2.f; // Center y

  // Load pre-calibrated camera parameters if they exist
  if (std::isfinite (depth_parameters_.focal_length_x))
    fx =  static_cast<float> (depth_parameters_.focal_length_x);

  if (std::isfinite (depth_parameters_.focal_length_y))
    fy =  static_cast<float> (depth_parameters_.focal_length_y);

  if (std::isfinite (depth_parameters_.principal_point_x))
    cx =  static_cast<float> (depth_parameters_.principal_point_x);

  if (std::isfinite (depth_parameters_.principal_point_y))
    cy =  static_cast<float> (depth_parameters_.principal_point_y);

  // Get inverse focal length for calculations below
  float fx_inv = 1.0f / fx;
  float fy_inv = 1.0f / fy;

  const auto* depth_map = reinterpret_cast<const std::uint16_t*>(depth_image->getData ());
  if (depth_image->getWidth () != depth_width_ || depth_image->getHeight () != depth_height_)
  {
    // Resize the image if nessacery
    depth_resize_buffer_.resize(depth_width_ * depth_height_);
    depth_image->fillDepthImageRaw (depth_width_, depth_height_, depth_resize_buffer_.data() );
    depth_map = depth_resize_buffer_.data();
  }

  const auto* rgb_buffer = reinterpret_cast<const std::uint8_t*>(image->getData ());
  if (image->getWidth () != image_width_ || image->getHeight () != image_height_)
  {
    // Resize the image if nessacery
    color_resize_buffer_.resize(image_width_ * image_height_ * 3);
    image->fillRGB (image_width_, image_height_, color_resize_buffer_.data(), image_width_ * 3);
    rgb_buffer = color_resize_buffer_.data();
  }


  float bad_point = std::numeric_limits<float>::quiet_NaN ();

  // set xyz to Nan and rgb to 0 (black)
  if (image_width_ != depth_width_)
  {
    PointT pt;
    pt.x = pt.y = pt.z = bad_point;
    pt.b = pt.g = pt.r = 0;
    pt.a = 255; // point has no color info -> alpha = max => transparent
    cloud->points.assign (cloud->size (), pt);
  }

  // fill in XYZ values
  unsigned step = cloud->width / depth_width_;
  unsigned skip = cloud->width - (depth_width_ * step);

  unsigned value_idx = 0;
  unsigned point_idx = 0;
  for (unsigned v = 0; v < depth_height_; ++v, point_idx += skip)
  {
    for (unsigned u = 0; u < depth_width_; ++u, ++value_idx, point_idx += step)
    {
      PointT& pt = (*cloud)[point_idx];
      /// @todo Different values for these cases
      // Check for invalid measurements

      OniDepthPixel pixel = depth_map[value_idx];
      if (pixel != 0 &&
        pixel != depth_image->getNoSampleValue () &&
        pixel != depth_image->getShadowValue () )
      {
        pt.z = depth_map[value_idx] * 0.001f;  // millimeters to meters
        pt.x = (static_cast<float> (u) - cx) * pt.z * fx_inv;
        pt.y = (static_cast<float> (v) - cy) * pt.z * fy_inv;
      }
      else
      {
        pt.x = pt.y = pt.z = bad_point;
      }
    }
  }

  // fill in the RGB values
  step = cloud->width / image_width_;
  skip = cloud->width - (image_width_ * step);

  value_idx = 0;
  point_idx = 0;
  RGBValue color;
  color.Alpha = 0xff;

  for (unsigned yIdx = 0; yIdx < image_height_; ++yIdx, point_idx += skip)
  {
    for (unsigned xIdx = 0; xIdx < image_width_; ++xIdx, point_idx += step, value_idx += 3)
    {
      PointT& pt = (*cloud)[point_idx];

      color.Red   = rgb_buffer[value_idx];
      color.Green = rgb_buffer[value_idx + 1];
      color.Blue  = rgb_buffer[value_idx + 2];

      pt.rgba = color.long_value;
    }
  }
  cloud->sensor_origin_.setZero ();
  cloud->sensor_orientation_.setIdentity ();
  return (cloud);
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
pcl::PointCloud<pcl::PointXYZI>::Ptr
pcl::io::OpenNI2Grabber::convertToXYZIPointCloud (const IRImage::Ptr &ir_image, const DepthImage::Ptr &depth_image)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);

  cloud->header.seq = depth_image->getFrameID ();
  cloud->header.stamp = depth_image->getTimestamp ();
  cloud->header.frame_id = rgb_frame_id_;
  cloud->height = depth_height_;
  cloud->width = depth_width_;
  cloud->is_dense = false;

  cloud->points.resize (cloud->height * cloud->width);


  float fx = device_->getDepthFocalLength (); // Horizontal focal length
  float fy = device_->getDepthFocalLength (); // Vertcal focal length
  float cx = ((float)cloud->width - 1.f) / 2.f;  // Center x
  float cy = ((float)cloud->height - 1.f) / 2.f; // Center y

  // Load pre-calibrated camera parameters if they exist
  if (std::isfinite (depth_parameters_.focal_length_x))
    fx =  static_cast<float> (depth_parameters_.focal_length_x);

  if (std::isfinite (depth_parameters_.focal_length_y))
    fy =  static_cast<float> (depth_parameters_.focal_length_y);

  if (std::isfinite (depth_parameters_.principal_point_x))
    cx =  static_cast<float> (depth_parameters_.principal_point_x);

  if (std::isfinite (depth_parameters_.principal_point_y))
    cy =  static_cast<float> (depth_parameters_.principal_point_y);

  float fx_inv = 1.0f / fx;
  float fy_inv = 1.0f / fy;


  const auto* depth_map = reinterpret_cast<const std::uint16_t*>(depth_image->getData ());
  if (depth_image->getWidth () != depth_width_ || depth_image->getHeight () != depth_height_)
  {
    // Resize the image if nessacery
    depth_resize_buffer_.resize(depth_width_ * depth_height_);
    depth_image->fillDepthImageRaw (depth_width_, depth_height_, depth_resize_buffer_.data() );
    depth_map = depth_resize_buffer_.data();
  }

  const auto* ir_map = reinterpret_cast<const std::uint16_t*>(ir_image->getData ());
  if (ir_image->getWidth () != depth_width_ || ir_image->getHeight () != depth_height_)
  {
    // Resize the image if nessacery
    ir_resize_buffer_.resize(depth_width_ * depth_height_);
    ir_image->fillRaw (depth_width_, depth_height_, ir_resize_buffer_.data());
    ir_map = ir_resize_buffer_.data();
  }


  std::size_t depth_idx = 0;
  float bad_point = std::numeric_limits<float>::quiet_NaN ();

  for (unsigned v = 0; v < depth_height_; ++v)
  {
    for (unsigned u = 0; u < depth_width_; ++u, ++depth_idx)
    {
      pcl::PointXYZI& pt = (*cloud)[depth_idx];
      /// @todo Different values for these cases
      // Check for invalid measurements
      if (depth_map[depth_idx] == 0 ||
        depth_map[depth_idx] == depth_image->getNoSampleValue () ||
        depth_map[depth_idx] == depth_image->getShadowValue ())
      {
        pt.x = pt.y = pt.z = bad_point;
      }
      else
      {
        pt.z = depth_map[depth_idx] * 0.001f; // millimeters to meters
        pt.x = (static_cast<float> (u) - cx) * pt.z * fx_inv;
        pt.y = (static_cast<float> (v) - cy) * pt.z * fy_inv;
      }

      pt.data_c[0] = pt.data_c[1] = pt.data_c[2] = pt.data_c[3] = 0;
      pt.intensity = static_cast<float> (ir_map[depth_idx]);
    }
  }
  cloud->sensor_origin_.setZero ();
  cloud->sensor_orientation_.setIdentity ();
  return (cloud);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::io::OpenNI2Grabber::updateModeMaps ()
{
  using VideoMode = pcl::io::openni2::OpenNI2VideoMode;

  pcl::io::openni2::OpenNI2VideoMode output_mode;

  config2oni_map_[OpenNI_SXGA_15Hz] = VideoMode (XN_SXGA_X_RES, XN_SXGA_Y_RES, 15);

  config2oni_map_[OpenNI_VGA_25Hz] = VideoMode (XN_VGA_X_RES, XN_VGA_Y_RES, 25);
  config2oni_map_[OpenNI_VGA_30Hz] = VideoMode (XN_VGA_X_RES, XN_VGA_Y_RES, 30);

  config2oni_map_[OpenNI_QVGA_25Hz] = VideoMode (XN_QVGA_X_RES, XN_QVGA_Y_RES, 25);
  config2oni_map_[OpenNI_QVGA_30Hz] = VideoMode (XN_QVGA_X_RES, XN_QVGA_Y_RES, 30);
  config2oni_map_[OpenNI_QVGA_60Hz] = VideoMode (XN_QVGA_X_RES, XN_QVGA_Y_RES, 60);

  config2oni_map_[OpenNI_QQVGA_25Hz] = VideoMode (XN_QQVGA_X_RES, XN_QQVGA_Y_RES, 25);
  config2oni_map_[OpenNI_QQVGA_30Hz] = VideoMode (XN_QQVGA_X_RES, XN_QQVGA_Y_RES, 30);
  config2oni_map_[OpenNI_QQVGA_60Hz] = VideoMode (XN_QQVGA_X_RES, XN_QQVGA_Y_RES, 60);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::io::OpenNI2Grabber::mapMode2XnMode (int mode, OpenNI2VideoMode &xnmode) const
{
  auto it = config2oni_map_.find (mode);
  if (it != config2oni_map_.end ())
  {
    xnmode = it->second;
    return (true);
  }
  return (false);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::vector<std::pair<int, pcl::io::openni2::OpenNI2VideoMode> >
pcl::io::OpenNI2Grabber::getAvailableDepthModes () const
{
pcl::io::openni2::OpenNI2VideoMode dummy;
  std::vector<std::pair<int, pcl::io::openni2::OpenNI2VideoMode> > result;
  for (const auto &config2oni : config2oni_map_)
  {
    if (device_->findCompatibleDepthMode (config2oni.second, dummy))
      result.emplace_back(config2oni);
  }

  return (result);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::vector<std::pair<int, pcl::io::openni2::OpenNI2VideoMode> >
pcl::io::OpenNI2Grabber::getAvailableImageModes () const
{
pcl::io::openni2::OpenNI2VideoMode dummy;
  std::vector<std::pair<int, pcl::io::openni2::OpenNI2VideoMode> > result;
  for (const auto &config2oni : config2oni_map_)
  {
    if (device_->findCompatibleColorMode (config2oni.second, dummy))
      result.emplace_back(config2oni);
  }

  return (result);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float
pcl::io::OpenNI2Grabber::getFramesPerSecond () const
{
  return (static_cast<float> (device_->getColorVideoMode ().frame_rate_));
}



// Convert VideoFrameRef into pcl::Image and forward to registered callbacks
void pcl::io::OpenNI2Grabber::processColorFrame (openni::VideoStream& stream)
{
  Image::Timestamp t_callback = Image::Clock::now ();

  openni::VideoFrameRef frame;
  stream.readFrame (&frame);
  FrameWrapper::Ptr frameWrapper(new Openni2FrameWrapper(frame));

  openni::PixelFormat format = frame.getVideoMode ().getPixelFormat ();
  Image::Ptr image;

  // Convert frame to PCL image type, based on pixel format
  if (format == openni::PIXEL_FORMAT_YUV422)
    image.reset (new ImageYUV422 (frameWrapper, t_callback));
  else //if (format == PixelFormat::PIXEL_FORMAT_RGB888)
    image.reset (new ImageRGB24 (frameWrapper, t_callback));

  imageCallback (image, nullptr);
}


void pcl::io::OpenNI2Grabber::processDepthFrame (openni::VideoStream& stream)
{
  openni::VideoFrameRef frame;
  stream.readFrame (&frame);
  FrameWrapper::Ptr frameWrapper(new Openni2FrameWrapper(frame));

  float focalLength = device_->getDepthFocalLength ();

  float baseline = device_->getBaseline();
  std::uint64_t no_sample_value = device_->getShadowValue();
  std::uint64_t shadow_value = no_sample_value;

  DepthImage::Ptr image (new DepthImage (frameWrapper, baseline, focalLength, shadow_value, no_sample_value));

  depthCallback (image, nullptr);
}


void pcl::io::OpenNI2Grabber::processIRFrame (openni::VideoStream& stream)
{
  openni::VideoFrameRef frame;
  stream.readFrame (&frame);

  FrameWrapper::Ptr frameWrapper(new Openni2FrameWrapper(frame));

  IRImage::Ptr image (new IRImage (frameWrapper));

  irCallback (image, nullptr);
}

#endif // HAVE_OPENNI2
