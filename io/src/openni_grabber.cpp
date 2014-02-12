/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
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
 *   * Neither the name of the copyright holder(s) nor the names of its
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

#include <pcl/io/openni_grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/io/boost.h>
#include <pcl/exceptions.h>
#include <iostream>

namespace pcl
{
  typedef union
  {
    struct
    {
      unsigned char Blue;
      unsigned char Green;
      unsigned char Red;
      unsigned char Alpha;
    };
    float float_value;
    uint32_t long_value;
  } RGBValue;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
pcl::OpenNIGrabber::OpenNIGrabber (const std::string& device_id, const Mode& depth_mode, const Mode& image_mode)
  : rgb_sync_ ()
  , ir_sync_ ()
  , device_ ()
  , rgb_frame_id_ ()
  , depth_frame_id_ ()
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
  , config2xn_map_ (), depth_callback_handle (), image_callback_handle (), ir_callback_handle ()
  , running_ (false)
  , rgb_focal_length_x_ (std::numeric_limits<double>::quiet_NaN ())
  , rgb_focal_length_y_ (std::numeric_limits<double>::quiet_NaN ())
  , rgb_principal_point_x_ (std::numeric_limits<double>::quiet_NaN ())
  , rgb_principal_point_y_ (std::numeric_limits<double>::quiet_NaN ())
  , depth_focal_length_x_ (std::numeric_limits<double>::quiet_NaN ())
  , depth_focal_length_y_ (std::numeric_limits<double>::quiet_NaN ())
  , depth_principal_point_x_ (std::numeric_limits<double>::quiet_NaN ())
  , depth_principal_point_y_ (std::numeric_limits<double>::quiet_NaN ())
{
  // initialize driver
  onInit (device_id, depth_mode, image_mode);

  if (!device_->hasDepthStream ())
    PCL_THROW_EXCEPTION (pcl::IOException, "Device does not provide 3D information.");

  depth_image_signal_    = createSignal<sig_cb_openni_depth_image> ();
  ir_image_signal_       = createSignal<sig_cb_openni_ir_image> ();
  point_cloud_signal_    = createSignal<sig_cb_openni_point_cloud> ();
  point_cloud_i_signal_  = createSignal<sig_cb_openni_point_cloud_i> ();
  ir_depth_image_signal_ = createSignal<sig_cb_openni_ir_depth_image> ();

  ir_sync_.addCallback (boost::bind (&OpenNIGrabber::irDepthImageCallback, this, _1, _2));
  if (device_->hasImageStream ())
  {
    // create callback signals
    image_signal_             = createSignal<sig_cb_openni_image> ();
    image_depth_image_signal_ = createSignal<sig_cb_openni_image_depth_image> ();
    point_cloud_rgb_signal_   = createSignal<sig_cb_openni_point_cloud_rgb> ();
    point_cloud_rgba_signal_  = createSignal<sig_cb_openni_point_cloud_rgba> ();
    rgb_sync_.addCallback (boost::bind (&OpenNIGrabber::imageDepthImageCallback, this, _1, _2));
    openni_wrapper::DeviceKinect* kinect = dynamic_cast<openni_wrapper::DeviceKinect*> (device_.get ());
    if (kinect)
      kinect->setDebayeringMethod (openni_wrapper::ImageBayerGRBG::EdgeAware);
  }

  image_callback_handle = device_->registerImageCallback (&OpenNIGrabber::imageCallback, *this);
  depth_callback_handle = device_->registerDepthCallback (&OpenNIGrabber::depthCallback, *this);
  ir_callback_handle    = device_->registerIRCallback (&OpenNIGrabber::irCallback, *this);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
pcl::OpenNIGrabber::~OpenNIGrabber () throw ()
{
  try
  {
    stop ();

    // unregister callbacks
    device_->unregisterDepthCallback (depth_callback_handle);

    if (device_->hasImageStream ())
      device_->unregisterImageCallback (image_callback_handle);

    if (device_->hasIRStream ())
      device_->unregisterIRCallback (image_callback_handle);

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

    openni_wrapper::OpenNIDriver& driver = openni_wrapper::OpenNIDriver::getInstance ();
    driver.stopAll ();
  }
  catch (...)
  {
    // destructor never throws
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::OpenNIGrabber::checkImageAndDepthSynchronizationRequired ()
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
pcl::OpenNIGrabber::checkImageStreamRequired ()
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
pcl::OpenNIGrabber::checkDepthStreamRequired ()
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
pcl::OpenNIGrabber::checkIRStreamRequired ()
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
pcl::OpenNIGrabber::start ()
{
  try
  {
    // check if we need to start/stop any stream
    if (image_required_ && !device_->isImageStreamRunning ())
    {
      block_signals ();
      device_->startImageStream ();
      //startSynchronization ();
    }

    if (depth_required_ && !device_->isDepthStreamRunning ())
    {
      block_signals ();
      if (device_->hasImageStream () && !device_->isDepthRegistered () && device_->isDepthRegistrationSupported ())
      {
        device_->setDepthRegistration (true);
      }
      device_->startDepthStream ();
      //startSynchronization ();
    }

    if (ir_required_ && !device_->isIRStreamRunning ())
    {
      block_signals ();
      device_->startIRStream ();
    }
    running_ = true;
  }
  catch (openni_wrapper::OpenNIException& ex)
  {
    PCL_THROW_EXCEPTION (pcl::IOException, "Could not start streams. Reason: " << ex.what ());
  }
  // workaround, since the first frame is corrupted
  boost::this_thread::sleep (boost::posix_time::seconds (1));
  unblock_signals ();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::OpenNIGrabber::stop ()
{
  try
  {
    if (device_->hasDepthStream () && device_->isDepthStreamRunning ())
      device_->stopDepthStream ();

    if (device_->hasImageStream () && device_->isImageStreamRunning ())
      device_->stopImageStream ();

    if (device_->hasIRStream () && device_->isIRStreamRunning ())
      device_->stopIRStream ();

    running_ = false;
  }
  catch (openni_wrapper::OpenNIException& ex)
  {
    PCL_THROW_EXCEPTION (pcl::IOException, "Could not stop streams. Reason: " << ex.what ());
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::OpenNIGrabber::isRunning () const
{
  return (running_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::OpenNIGrabber::onInit (const std::string& device_id, const Mode& depth_mode, const Mode& image_mode)
{
  updateModeMaps (); // registering mapping from config modes to XnModes and vice versa
  setupDevice (device_id, depth_mode, image_mode);

  rgb_frame_id_ = "/openni_rgb_optical_frame";

  depth_frame_id_ = "/openni_depth_optical_frame";
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::OpenNIGrabber::signalsChanged ()
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
pcl::OpenNIGrabber::getName () const
{
  return std::string ("OpenNIGrabber");
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::OpenNIGrabber::setupDevice (const std::string& device_id, const Mode& depth_mode, const Mode& image_mode)
{
  // Initialize the openni device
  openni_wrapper::OpenNIDriver& driver = openni_wrapper::OpenNIDriver::getInstance ();

  try
  {
    if (boost::filesystem::exists (device_id))
    {
      device_ = driver.createVirtualDevice (device_id, true, true);
    }
    else if (driver.getNumberDevices () == 0)
    {
      PCL_THROW_EXCEPTION (pcl::IOException, "No devices connected.");
    }
    else if (device_id[0] == '#')
    {
      unsigned index = atoi (device_id.c_str () + 1);
      //printf("[%s] searching for device with index = %d\n", getName().c_str(), index);
      device_ = driver.getDeviceByIndex (index - 1);
    }
#ifndef _WIN32
    else if (device_id.find ('@') != std::string::npos)
    {
      size_t pos = device_id.find ('@');
      unsigned bus = atoi (device_id.substr (0, pos).c_str ());
      unsigned address = atoi (device_id.substr (pos + 1, device_id.length () - pos - 1).c_str ());
      //printf("[%s] searching for device with bus@address = %d@%d\n", getName().c_str(), bus, address);
      device_ = driver.getDeviceByAddress (static_cast<unsigned char>(bus), static_cast<unsigned char>(address) );
    }
#endif
    else if (!device_id.empty ())
    {
      //printf("[%s] searching for device with serial number = %s\n", getName().c_str(), device_id.c_str());
      device_ = driver.getDeviceBySerialNumber (device_id);
    }
    else
    {
      device_ = driver.getDeviceByIndex (0);
    }
  }
  catch (const openni_wrapper::OpenNIException& exception)
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
    PCL_THROW_EXCEPTION (pcl::IOException, "unknown error occured");
  }

  XnMapOutputMode depth_md;
  // Set the selected output mode
  if (depth_mode != OpenNI_Default_Mode)
  {
    XnMapOutputMode actual_depth_md;
    if (!mapConfigMode2XnMode (depth_mode, depth_md) || !device_->findCompatibleDepthMode (depth_md, actual_depth_md))
      PCL_THROW_EXCEPTION (pcl::IOException, "could not find compatible depth stream mode " << static_cast<int> (depth_mode) );

    XnMapOutputMode current_depth_md =  device_->getDepthOutputMode ();
    if (current_depth_md.nXRes != actual_depth_md.nXRes || current_depth_md.nYRes != actual_depth_md.nYRes)
      device_->setDepthOutputMode (actual_depth_md);
  }
  else
  {
    depth_md = device_->getDefaultDepthMode ();
  }

  depth_width_ = depth_md.nXRes;
  depth_height_ = depth_md.nYRes;
  
  if (device_->hasImageStream ())
  {
    XnMapOutputMode image_md;
    if (image_mode != OpenNI_Default_Mode)
    {
      XnMapOutputMode actual_image_md;
      if (!mapConfigMode2XnMode (image_mode, image_md) || !device_->findCompatibleImageMode (image_md, actual_image_md))
        PCL_THROW_EXCEPTION (pcl::IOException, "could not find compatible image stream mode " << static_cast<int> (image_mode) );

      XnMapOutputMode current_image_md =  device_->getImageOutputMode ();
      if (current_image_md.nXRes != actual_image_md.nXRes || current_image_md.nYRes != actual_image_md.nYRes)
        device_->setImageOutputMode (actual_image_md);
    }
    else
    {
      image_md = device_->getDefaultImageMode ();
    }

    image_width_  = image_md.nXRes;
    image_height_ = image_md.nYRes;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::OpenNIGrabber::startSynchronization ()
{
  try
  {
    if (device_->isSynchronizationSupported () && !device_->isSynchronized () &&
        device_->getImageOutputMode ().nFPS == device_->getDepthOutputMode ().nFPS &&
        device_->isImageStreamRunning () && device_->isDepthStreamRunning ())
        device_->setSynchronization (true);
  }
  catch (const openni_wrapper::OpenNIException& exception)
  {
    PCL_THROW_EXCEPTION (pcl::IOException, "Could not start synchronization " << exception.what ());
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::OpenNIGrabber::stopSynchronization ()
{
  try
  {
    if (device_->isSynchronizationSupported () && device_->isSynchronized ())
      device_->setSynchronization (false);
  }
  catch (const openni_wrapper::OpenNIException& exception)
  {
    PCL_THROW_EXCEPTION (pcl::IOException, "Could not start synchronization " << exception.what ());
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::OpenNIGrabber::imageCallback (boost::shared_ptr<openni_wrapper::Image> image, void*)
{
  if (num_slots<sig_cb_openni_point_cloud_rgb>   () > 0 ||
      num_slots<sig_cb_openni_point_cloud_rgba>  () > 0 ||
      num_slots<sig_cb_openni_image_depth_image> () > 0)
    rgb_sync_.add0 (image, image->getTimeStamp ());

  if (image_signal_->num_slots () > 0)
    image_signal_->operator()(image);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::OpenNIGrabber::depthCallback (boost::shared_ptr<openni_wrapper::DepthImage> depth_image, void*)
{
  if (num_slots<sig_cb_openni_point_cloud_rgb>   () > 0 ||
      num_slots<sig_cb_openni_point_cloud_rgba>  () > 0 ||
      num_slots<sig_cb_openni_image_depth_image> () > 0)
    rgb_sync_.add1 (depth_image, depth_image->getTimeStamp ());

  if (num_slots<sig_cb_openni_point_cloud_i>  () > 0 ||
      num_slots<sig_cb_openni_ir_depth_image> () > 0)
    ir_sync_.add1 (depth_image, depth_image->getTimeStamp ());

  if (depth_image_signal_->num_slots () > 0)
    depth_image_signal_->operator()(depth_image);

  if (point_cloud_signal_->num_slots () > 0)
    point_cloud_signal_->operator()(convertToXYZPointCloud(depth_image));
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::OpenNIGrabber::irCallback (boost::shared_ptr<openni_wrapper::IRImage> ir_image, void*)
{
  if (num_slots<sig_cb_openni_point_cloud_i>  () > 0 ||
      num_slots<sig_cb_openni_ir_depth_image> () > 0)
    ir_sync_.add0(ir_image, ir_image->getTimeStamp ());

  if (ir_image_signal_->num_slots () > 0)
    ir_image_signal_->operator()(ir_image);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::OpenNIGrabber::imageDepthImageCallback (const boost::shared_ptr<openni_wrapper::Image> &image,
                                             const boost::shared_ptr<openni_wrapper::DepthImage> &depth_image)
{
  // check if we have color point cloud slots
  if (point_cloud_rgb_signal_->num_slots () > 0)
  {
    PCL_WARN ("PointXYZRGB callbacks deprecated. Use PointXYZRGBA instead.\n");
    point_cloud_rgb_signal_->operator()(convertToXYZRGBPointCloud<pcl::PointXYZRGB> (image, depth_image));
  }

  if (point_cloud_rgba_signal_->num_slots () > 0)
    point_cloud_rgba_signal_->operator()(convertToXYZRGBPointCloud<pcl::PointXYZRGBA> (image, depth_image));

  if (image_depth_image_signal_->num_slots () > 0)
  {
    float constant = 1.0f / device_->getDepthFocalLength (depth_width_);
    image_depth_image_signal_->operator()(image, depth_image, constant);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::OpenNIGrabber::irDepthImageCallback (const boost::shared_ptr<openni_wrapper::IRImage> &ir_image,
                                          const boost::shared_ptr<openni_wrapper::DepthImage> &depth_image)
{
  // check if we have color point cloud slots
  if (point_cloud_i_signal_->num_slots () > 0)
    point_cloud_i_signal_->operator()(convertToXYZIPointCloud (ir_image, depth_image));

  if (ir_depth_image_signal_->num_slots () > 0)
  {
    float constant = 1.0f / device_->getDepthFocalLength (depth_width_);
    ir_depth_image_signal_->operator()(ir_image, depth_image, constant);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
pcl::PointCloud<pcl::PointXYZ>::Ptr
pcl::OpenNIGrabber::convertToXYZPointCloud (const boost::shared_ptr<openni_wrapper::DepthImage>& depth_image) const
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud <pcl::PointXYZ>);

  cloud->height = depth_height_;
  cloud->width = depth_width_;
  cloud->is_dense = false;

  cloud->points.resize (cloud->height * cloud->width);

  register float constant_x = 1.0f / device_->getDepthFocalLength (depth_width_);
  register float constant_y = 1.0f / device_->getDepthFocalLength (depth_width_);
  register float centerX = ((float)cloud->width - 1.f) / 2.f;
  register float centerY = ((float)cloud->height - 1.f) / 2.f;

  if (pcl_isfinite (depth_focal_length_x_))
    constant_x =  1.0f / static_cast<float> (depth_focal_length_x_);

  if (pcl_isfinite (depth_focal_length_y_))
    constant_y =  1.0f / static_cast<float> (depth_focal_length_y_);
  
  if (pcl_isfinite (depth_principal_point_x_))
    centerX =  static_cast<float> (depth_principal_point_x_);
  
  if (pcl_isfinite (depth_principal_point_y_))
    centerY =  static_cast<float> (depth_principal_point_y_);

  if (device_->isDepthRegistered ())
    cloud->header.frame_id = rgb_frame_id_;
  else
    cloud->header.frame_id = depth_frame_id_;


  float bad_point = std::numeric_limits<float>::quiet_NaN ();

  // we have to use Data, since operator[] uses assert -> Debug-mode very slow!
  register const unsigned short* depth_map = depth_image->getDepthMetaData ().Data ();
  if (depth_image->getWidth() != depth_width_ || depth_image->getHeight () != depth_height_)
  {
    static unsigned buffer_size = 0;
    static boost::shared_array<unsigned short> depth_buffer ((unsigned short*)(NULL));

    if (buffer_size < depth_width_ * depth_height_)
    {
      buffer_size = depth_width_ * depth_height_;
      depth_buffer.reset (new unsigned short [buffer_size]);
    }
    depth_image->fillDepthImageRaw (depth_width_, depth_height_, depth_buffer.get ());
    depth_map = depth_buffer.get ();
  }

  register int depth_idx = 0;
  for (unsigned int v = 0; v < depth_height_; ++v)
  {
    for (register unsigned int u = 0; u < depth_width_; ++u, ++depth_idx)
    {
      pcl::PointXYZ& pt = cloud->points[depth_idx];
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
pcl::OpenNIGrabber::convertToXYZRGBPointCloud (const boost::shared_ptr<openni_wrapper::Image> &image,
                                               const boost::shared_ptr<openni_wrapper::DepthImage> &depth_image) const
{
  static unsigned rgb_array_size = 0;
  static boost::shared_array<unsigned char> rgb_array ((unsigned char*)(NULL));
  static unsigned char* rgb_buffer = 0;

  boost::shared_ptr<pcl::PointCloud<PointT> > cloud (new pcl::PointCloud<PointT>);

  cloud->header.frame_id = rgb_frame_id_;
  cloud->height = std::max (image_height_, depth_height_);
  cloud->width = std::max (image_width_, depth_width_);
  cloud->is_dense = false;

  cloud->points.resize (cloud->height * cloud->width);

  //float constant = 1.0f / device_->getImageFocalLength (depth_width_);
  register float constant_x = 1.0f / device_->getImageFocalLength (depth_width_);
  register float constant_y = 1.0f / device_->getImageFocalLength (depth_width_);
  register float centerX = ((float)cloud->width - 1.f) / 2.f;
  register float centerY = ((float)cloud->height - 1.f) / 2.f;

  if (pcl_isfinite (rgb_focal_length_x_))
    constant_x =  1.0f / static_cast<float> (rgb_focal_length_x_);

  if (pcl_isfinite (rgb_focal_length_y_))
    constant_y =  1.0f / static_cast<float> (rgb_focal_length_y_);
  
  if (pcl_isfinite (rgb_principal_point_x_))
    centerX =  static_cast<float> (rgb_principal_point_x_);
  
  if (pcl_isfinite (rgb_principal_point_y_))
    centerY =  static_cast<float> (rgb_principal_point_y_);

  register const XnDepthPixel* depth_map = depth_image->getDepthMetaData ().Data ();
  if (depth_image->getWidth () != depth_width_ || depth_image->getHeight() != depth_height_)
  {
    static unsigned buffer_size = 0;
    static boost::shared_array<unsigned short> depth_buffer ((unsigned short*)(NULL));

    if (buffer_size < depth_width_ * depth_height_)
    {
      buffer_size = depth_width_ * depth_height_;
      depth_buffer.reset (new unsigned short [buffer_size]);
    }

    depth_image->fillDepthImageRaw (depth_width_, depth_height_, depth_buffer.get ());
    depth_map = depth_buffer.get ();
  }

  // here we need exact the size of the point cloud for a one-one correspondence!
  if (rgb_array_size < image_width_ * image_height_ * 3)
  {
    rgb_array_size = image_width_ * image_height_ * 3;
    rgb_array.reset (new unsigned char [rgb_array_size]);
    rgb_buffer = rgb_array.get ();
  }
  image->fillRGB (image_width_, image_height_, rgb_buffer, image_width_ * 3);
  float bad_point = std::numeric_limits<float>::quiet_NaN ();

  // set xyz to Nan and rgb to 0 (black)  
  if (image_width_ != depth_width_)
  {
    PointT pt;
    pt.x = pt.y = pt.z = bad_point;
    pt.b = pt.g = pt.r = 0;
    pt.a = 255; // point has no color info -> alpha = max => transparent 
    cloud->points.assign (cloud->points.size (), pt);
  }
  
  // fill in XYZ values
  unsigned step = cloud->width / depth_width_;
  unsigned skip = cloud->width * step - cloud->width;
  
  int value_idx = 0;
  int point_idx = 0;
  for (unsigned int v = 0; v < depth_height_; ++v, point_idx += skip)
  {
    for (register unsigned int u = 0; u < depth_width_; ++u, ++value_idx, point_idx += step)
    {
      PointT& pt = cloud->points[point_idx];
      /// @todo Different values for these cases
      // Check for invalid measurements

      if (depth_map[value_idx] != 0 &&
          depth_map[value_idx] != depth_image->getNoSampleValue () &&
          depth_map[value_idx] != depth_image->getShadowValue ())
      {
        pt.z = depth_map[value_idx] * 0.001f;
        pt.x = (static_cast<float> (u) - centerX) * pt.z * constant_x;
        pt.y = (static_cast<float> (v) - centerY) * pt.z * constant_y;
      }
      else
      {
        pt.x = pt.y = pt.z = bad_point;
      }
    }
  }

  // fill in the RGB values
  step = cloud->width / image_width_;
  skip = cloud->width * step - cloud->width;
  
  value_idx = 0;
  point_idx = 0;
  RGBValue color;
  color.Alpha = 0;

  for (unsigned yIdx = 0; yIdx < image_height_; ++yIdx, point_idx += skip)
  {
    for (unsigned xIdx = 0; xIdx < image_width_; ++xIdx, point_idx += step, value_idx += 3)
    {
      PointT& pt = cloud->points[point_idx];
      
      color.Red   = rgb_buffer[value_idx];
      color.Green = rgb_buffer[value_idx + 1];
      color.Blue  = rgb_buffer[value_idx + 2];
      
      pt.rgba = color.long_value;
    }
  }
  cloud->sensor_origin_.setZero ();
  cloud->sensor_orientation_.w () = 1.0;
  cloud->sensor_orientation_.x () = 0.0;
  cloud->sensor_orientation_.y () = 0.0;
  cloud->sensor_orientation_.z () = 0.0;
  return (cloud);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
pcl::PointCloud<pcl::PointXYZI>::Ptr
pcl::OpenNIGrabber::convertToXYZIPointCloud (const boost::shared_ptr<openni_wrapper::IRImage> &ir_image,
                                             const boost::shared_ptr<openni_wrapper::DepthImage> &depth_image) const
{
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI> > cloud (new pcl::PointCloud<pcl::PointXYZI > ());

  cloud->header.frame_id = rgb_frame_id_;
  cloud->height = depth_height_;
  cloud->width = depth_width_;
  cloud->is_dense = false;

  cloud->points.resize (cloud->height * cloud->width);

  //float constant = 1.0f / device_->getImageFocalLength (cloud->width);
  register float constant_x = 1.0f / device_->getImageFocalLength (cloud->width);
  register float constant_y = 1.0f / device_->getImageFocalLength (cloud->width);
  register float centerX = ((float)cloud->width - 1.f) / 2.f;
  register float centerY = ((float)cloud->height - 1.f) / 2.f;

  if (pcl_isfinite (rgb_focal_length_x_))
    constant_x =  1.0f / static_cast<float> (rgb_focal_length_x_);

  if (pcl_isfinite (rgb_focal_length_y_))
    constant_y =  1.0f / static_cast<float> (rgb_focal_length_y_);

  if (pcl_isfinite (rgb_principal_point_x_))
    centerX = static_cast<float>(rgb_principal_point_x_);
  
  if (pcl_isfinite (rgb_principal_point_y_))
    centerY = static_cast<float>(rgb_principal_point_y_);

  register const XnDepthPixel* depth_map = depth_image->getDepthMetaData ().Data ();
  register const XnIRPixel* ir_map = ir_image->getMetaData ().Data ();

  if (depth_image->getWidth () != depth_width_ || depth_image->getHeight () != depth_height_)
  {
    static unsigned buffer_size = 0;
    static boost::shared_array<unsigned short> depth_buffer ((unsigned short*)(NULL));
    static boost::shared_array<unsigned short> ir_buffer ((unsigned short*)(NULL));

    if (buffer_size < depth_width_ * depth_height_)
    {
      buffer_size = depth_width_ * depth_height_;
      depth_buffer.reset (new unsigned short [buffer_size]);
      ir_buffer.reset (new unsigned short [buffer_size]);
    }

    depth_image->fillDepthImageRaw (depth_width_, depth_height_, depth_buffer.get ());
    depth_map = depth_buffer.get ();

    ir_image->fillRaw (depth_width_, depth_height_, ir_buffer.get ());
    ir_map = ir_buffer.get ();
  }

  register int depth_idx = 0;
  float bad_point = std::numeric_limits<float>::quiet_NaN ();

  for (unsigned int v = 0; v < depth_height_; ++v)
  {
    for (register unsigned int u = 0; u < depth_width_; ++u, ++depth_idx)
    {
      pcl::PointXYZI& pt = cloud->points[depth_idx];
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
        pt.z = depth_map[depth_idx] * 0.001f;
        pt.x = (static_cast<float> (u) - centerX) * pt.z * constant_x;
        pt.y = (static_cast<float> (v) - centerY) * pt.z * constant_y;
      }

      pt.data_c[0] = pt.data_c[1] = pt.data_c[2] = pt.data_c[3] = 0;
      pt.intensity = static_cast<float> (ir_map[depth_idx]);
    }
  }
  cloud->sensor_origin_.setZero ();
  cloud->sensor_orientation_.w () = 1.0;
  cloud->sensor_orientation_.x () = 0.0;
  cloud->sensor_orientation_.y () = 0.0;
  cloud->sensor_orientation_.z () = 0.0;
  return (cloud);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// TODO: delete me?
void
pcl::OpenNIGrabber::updateModeMaps ()
{
  XnMapOutputMode output_mode;

  output_mode.nXRes = XN_SXGA_X_RES;
  output_mode.nYRes = XN_SXGA_Y_RES;
  output_mode.nFPS = 15;
  config2xn_map_[OpenNI_SXGA_15Hz] = output_mode;

  output_mode.nXRes = XN_VGA_X_RES;
  output_mode.nYRes = XN_VGA_Y_RES;
  output_mode.nFPS = 25;
  config2xn_map_[OpenNI_VGA_25Hz] = output_mode;
  output_mode.nFPS = 30;
  config2xn_map_[OpenNI_VGA_30Hz] = output_mode;

  output_mode.nXRes = XN_QVGA_X_RES;
  output_mode.nYRes = XN_QVGA_Y_RES;
  output_mode.nFPS = 25;
  config2xn_map_[OpenNI_QVGA_25Hz] = output_mode;
  output_mode.nFPS = 30;
  config2xn_map_[OpenNI_QVGA_30Hz] = output_mode;
  output_mode.nFPS = 60;
  config2xn_map_[OpenNI_QVGA_60Hz] = output_mode;

  output_mode.nXRes = XN_QQVGA_X_RES;
  output_mode.nYRes = XN_QQVGA_Y_RES;
  output_mode.nFPS = 25;
  config2xn_map_[OpenNI_QQVGA_25Hz] = output_mode;
  output_mode.nFPS = 30;
  config2xn_map_[OpenNI_QQVGA_30Hz] = output_mode;
  output_mode.nFPS = 60;
  config2xn_map_[OpenNI_QQVGA_60Hz] = output_mode;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::OpenNIGrabber::mapConfigMode2XnMode (int mode, XnMapOutputMode &xnmode) const
{
  std::map<int, XnMapOutputMode>::const_iterator it = config2xn_map_.find (mode);
  if (it != config2xn_map_.end ())
  {
    xnmode = it->second;
    return (true);
  }
  else
  {
    return (false);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::vector<std::pair<int, XnMapOutputMode> >
pcl::OpenNIGrabber::getAvailableDepthModes () const
{
  XnMapOutputMode dummy;
  std::vector<std::pair<int, XnMapOutputMode> > result;
  for (std::map<int, XnMapOutputMode>::const_iterator it = config2xn_map_.begin (); it != config2xn_map_.end (); ++it)
  {
    if (device_->findCompatibleDepthMode (it->second, dummy))
      result.push_back (*it);
  }

  return (result);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::vector<std::pair<int, XnMapOutputMode> >
pcl::OpenNIGrabber::getAvailableImageModes () const
{
  XnMapOutputMode dummy;
  std::vector<std::pair<int, XnMapOutputMode> > result;
  for (std::map<int, XnMapOutputMode>::const_iterator it = config2xn_map_.begin (); it != config2xn_map_.end (); ++it)
  {
    if (device_->findCompatibleImageMode (it->second, dummy))
      result.push_back (*it);
  }

  return (result);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float
pcl::OpenNIGrabber::getFramesPerSecond () const
{
  return (static_cast<float> (device_->getDepthOutputMode ().nFPS));
}

#endif
