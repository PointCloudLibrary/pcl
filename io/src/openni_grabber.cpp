/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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
 * Author: Nico Blodow (blodow@cs.tum.edu), Suat Gedikli (gedikli@willowgarage.com)
 */

#include <pcl/pcl_config.h>
#ifdef HAVE_OPENNI

#include <pcl/io/openni_grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>
#include <pcl/io/pcl_io_exception.h>
#include <boost/shared_array.hpp>

namespace pcl
{
  typedef union
  {
    struct /*anonymous*/
    {
      unsigned char Blue;
      unsigned char Green;
      unsigned char Red;
      unsigned char Alpha;
    };
    float float_value;
    long long_value;
  } RGBValue;

  OpenNIGrabber::OpenNIGrabber (const std::string& device_id, const Params& params) throw (openni_wrapper::OpenNIException)
    : image_required_(false)
    , depth_required_(false)
    , sync_required_(false)
    , running_(false)
    , params_(params)
  {
    // initialize driver
    if (!onInit (device_id))
      THROW_PCL_IO_EXCEPTION("Device could not be initialized or no devices found.");

    if (!device_->hasDepthStream ())
      THROW_PCL_IO_EXCEPTION("Device does not provide 3D information.");

    depth_image_signal_ = createSignal <sig_cb_openni_depth_image> ();
    point_cloud_signal_ = createSignal <sig_cb_openni_point_cloud> ();

    if (device_->hasImageStream ())
    {
      // create callback signals
      image_signal_             = createSignal <sig_cb_openni_image> ();
      image_depth_image_signal_ = createSignal <sig_cb_openni_image_depth_image> ();
      point_cloud_rgb_signal_   = createSignal <sig_cb_openni_point_cloud_rgb> ();
      sync_.addCallback (boost::bind(&OpenNIGrabber::imageDepthImageCallback, this, _1, _2));
      openni_wrapper::DeviceKinect* kinect = dynamic_cast<openni_wrapper::DeviceKinect*> (device_.get());
      if (kinect)
        kinect->setDebayeringMethod(openni_wrapper::ImageBayerGRBG::EdgeAware);
    }
    
    image_callback_handle = device_->registerImageCallback (&OpenNIGrabber::imageCallback, *this);
    depth_callback_handle = device_->registerDepthCallback (&OpenNIGrabber::depthCallback, *this);
  }

  OpenNIGrabber::~OpenNIGrabber ()
  {
    stop ();
    // unregister callbacks
    device_->unregisterDepthCallback (depth_callback_handle);
    device_->unregisterImageCallback (image_callback_handle);
    
    // disconnect all listeeners
    disconnect_all_slots <sig_cb_openni_image> ();
    disconnect_all_slots <sig_cb_openni_depth_image> ();
    disconnect_all_slots <sig_cb_openni_image_depth_image> ();
    disconnect_all_slots <sig_cb_openni_point_cloud> ();
    disconnect_all_slots <sig_cb_openni_point_cloud_rgb> ();
    
    try
    {
      openni_wrapper::OpenNIDriver& driver = openni_wrapper::OpenNIDriver::getInstance ();
      driver.stopAll ();
    }
    catch (...)
    {
    }
  }

  void OpenNIGrabber::checkImageAndDepthSynchronizationRequired()
  {
    // do we have anyone listening to images or color point clouds?
    if (num_slots<sig_cb_openni_point_cloud_rgb> () > 0 ||
        num_slots<sig_cb_openni_image_depth_image> () > 0)
      sync_required_ = true;
    else
      sync_required_ = false;
  }

  void OpenNIGrabber::checkImageStreamRequired()
  {
    // do we have anyone listening to images or color point clouds?
    if (num_slots<sig_cb_openni_image> () > 0 || 
        num_slots<sig_cb_openni_image_depth_image> () > 0 ||
        num_slots<sig_cb_openni_point_cloud_rgb> () > 0)
      image_required_ = true;
    else
      image_required_ = false;
  }

  void OpenNIGrabber::checkDepthStreamRequired()
  {
    // do we have anyone listening to depth images or (color) point clouds?
    if (num_slots<sig_cb_openni_depth_image> () > 0 ||
        num_slots<sig_cb_openni_image_depth_image> () > 0 ||
        num_slots<sig_cb_openni_point_cloud_rgb> () > 0 ||
        num_slots<sig_cb_openni_point_cloud> () > 0)
      depth_required_ = true;
    else
      depth_required_ = false;
  }

  void OpenNIGrabber::start ()
  {
    // check if we need to start/stop any stream
    if (image_required_ && !device_->isImageStreamRunning ())
    {
      device_->startImageStream ();
      //startSynchronization ();
    }

    if (depth_required_ && !device_->isDepthStreamRunning ())
    {
      // TODO: turn this only on if needed ...
      if (device_->hasImageStream () && !device_->isDepthRegistered ())
      {
        device_->setDepthRegistration (true);
      }
      device_->startDepthStream ();
      //startSynchronization ();
    }
    
    running_ = true;
  }

  void OpenNIGrabber::stop ()
  {
    // stopSynchronization ();
    if (device_->hasDepthStream() && device_->isDepthStreamRunning ())
      device_->stopDepthStream();
    
    if (device_->hasImageStream() && device_->isImageStreamRunning())
      device_->stopImageStream();

    running_ = false;
  }

  bool OpenNIGrabber::isRunning () const
  {
    return running_;
  }

  bool OpenNIGrabber::onInit (const std::string& device_id)
  {
    updateModeMaps ();      // registering mapping from config modes to XnModes and vice versa
    if (!setupDevice (device_id))
      return (false);

    rgb_frame_id_ = "/openni_rgb_optical_frame";

    depth_frame_id_ = "/openni_depth_optical_frame";
    return (true);
  }
  
  void OpenNIGrabber::signalsChanged ()
  {
    // reevaluate which streams are required
    checkImageStreamRequired ();
    checkDepthStreamRequired ();
    checkImageAndDepthSynchronizationRequired ();
    if (running_)
      start ();
  }

  std::string OpenNIGrabber::getName () const
  {
    return std::string ("OpenNIGrabber");
  }

  bool OpenNIGrabber::setupDevice (const std::string& device_id)
  {
    // Initialize the openni device
    openni_wrapper::OpenNIDriver& driver = openni_wrapper::OpenNIDriver::getInstance ();

    if (driver.getNumberDevices () == 0)
    {
      printf ("[%s] No devices connected.\n", getName ().c_str ());
      return (false);
    }

    printf ("[%s] Number devices connected: %d\n", getName ().c_str (), driver.getNumberDevices ());
    for (unsigned deviceIdx = 0; deviceIdx < driver.getNumberDevices (); ++deviceIdx)
    {
      printf ("[%s] %u. device on bus %03u:%02u is a %s (%03x) from %s (%03x) with serial id \'%s\'\n"
                , getName ().c_str (), deviceIdx + 1, driver.getBus (deviceIdx), driver.getAddress (deviceIdx)
                , driver.getProductName (deviceIdx), driver.getProductID (deviceIdx), driver.getVendorName (deviceIdx)
                , driver.getVendorID (deviceIdx), driver.getSerialNumber (deviceIdx));
    }

    try {
      if (device_id[0] == '#')
      {
        unsigned index = atoi (device_id.c_str () + 1);
        printf ("[%s] searching for device with index = %d\n", getName ().c_str (), index);
        device_ = driver.getDeviceByIndex (index - 1);
      }
#ifndef _WIN32
      else if (device_id.find ('@') != std::string::npos)
      {
        size_t pos = device_id.find ('@');
        unsigned bus = atoi (device_id.substr (0, pos).c_str ());
        unsigned address = atoi (device_id.substr (pos + 1, device_id.length () - pos - 1).c_str ());
        printf ("[%s] searching for device with bus@address = %d@%d\n", getName ().c_str (), bus, address);
        device_ = driver.getDeviceByAddress (bus, address);
      }
      else if (!device_id.empty ())
      {
        printf ("[%s] searching for device with serial number = %s\n", getName ().c_str (), device_id.c_str ());
        device_ = driver.getDeviceBySerialNumber (device_id);
      }
#endif
      else
      {
        printf ("[%s] device_id is not set or has unknown format: %s! Using first device.\n", getName ().c_str (), device_id.c_str ());
        device_ = driver.getDeviceByIndex (0);
      }
    }
    catch (const openni_wrapper::OpenNIException& exception)
    {
      if (!device_)
      {
        printf ("[%s] No matching device found. %s\n", getName ().c_str (), exception.what());
        return (false);
      }
      else
      {
        printf ("[%s] could not retrieve device. Reason %s\n", getName ().c_str (), exception.what ());
        return (false);
      }
    }
    catch(...)
    {
      printf ("[%s] unknown error occured\n", getName ().c_str ());
      return (false);
    }
    printf ("[%s] Opened '%s' on bus %d:%d with serial number '%s'\n", getName ().c_str (),
              device_->getProductName (), device_->getBus (), device_->getAddress (), device_->getSerialNumber ());

    if (device_->hasImageStream ())
    {
      int image_mode = params_.image_mode;
      if (image_mode != -1)
      {
        //check if the image mode is supporte
        if (!isImageModeSupported(image_mode))
          image_mode = -1;
      }
      //default image mode
      if (image_mode == -1)
       image_mode = mapXnMode2ConfigMode (device_->getDefaultImageMode ());
      //if its still -1 one we are FAIL
      if (image_mode == -1)
        return (false);

      XnMapOutputMode image_md;
      if (!mapConfigMode2XnMode (image_mode, image_md))
        return (false);
      image_width_  = image_md.nXRes;
      image_height_ = image_md.nYRes;
    }


    int depth_mode = params_.depth_mode;
    if (depth_mode != -1)
    {
      //check if the depth mode is supported
      if (!isDepthModeSupported(depth_mode))
          depth_mode = -1; //not supported so try default
    }
    //default image mode
    if (depth_mode == -1)
      depth_mode = mapXnMode2ConfigMode(device_->getDefaultDepthMode());
    if (depth_mode == -1)
      return (false);

    XnMapOutputMode depth_md;
    if (!mapConfigMode2XnMode (depth_mode, depth_md))
      return (false);
    depth_width_  = depth_md.nXRes;
    depth_height_ = depth_md.nYRes;

    return (true);
  }

  void OpenNIGrabber::startSynchronization ()
  {   
    if (device_->isSynchronizationSupported () && !device_->isSynchronized () &&
        device_->getImageOutputMode ().nFPS == device_->getDepthOutputMode ().nFPS &&
        device_->isImageStreamRunning () && device_->isDepthStreamRunning () )
      device_->setSynchronization (true);
  }

  void OpenNIGrabber::stopSynchronization ()
  {
    if (device_->isSynchronizationSupported () && device_->isSynchronized ())
      device_->setSynchronization (false);
  }

  void OpenNIGrabber::imageCallback (boost::shared_ptr<openni_wrapper::Image> image, void* cookie)
  {
    if (num_slots<sig_cb_openni_point_cloud_rgb> () > 0 ||
        num_slots<sig_cb_openni_image_depth_image> () > 0)
        sync_.add0 (image, image->getTimeStamp());

    if (image_signal_->num_slots () > 0)
        image_signal_->operator()(image);

    return;
  }

  void OpenNIGrabber::depthCallback (boost::shared_ptr<openni_wrapper::DepthImage> depth_image, void* cookie)
  {
    if (num_slots<sig_cb_openni_point_cloud_rgb> () > 0 ||
        num_slots<sig_cb_openni_image_depth_image> () > 0)
        sync_.add1 (depth_image, depth_image->getTimeStamp());

    if (depth_image_signal_->num_slots () > 0)
        depth_image_signal_->operator()(depth_image);

    if (point_cloud_signal_->num_slots () > 0)
      point_cloud_signal_->operator()(convertToXYZPointCloud (depth_image));

    return;
  }

  void OpenNIGrabber::imageDepthImageCallback (const boost::shared_ptr<openni_wrapper::Image> &image, const boost::shared_ptr<openni_wrapper::DepthImage> &depth_image)
  {
    // check if we have color point cloud slots
    if (point_cloud_rgb_signal_->num_slots () > 0)
        point_cloud_rgb_signal_->operator()(convertToXYZRGBPointCloud (image, depth_image));

    if (image_depth_image_signal_->num_slots () > 0)
    {
      float constant = 1.0f / device_->getDepthFocalLength (depth_width_);
      image_depth_image_signal_->operator()(image, depth_image, constant);
    }
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr OpenNIGrabber::convertToXYZPointCloud (const boost::shared_ptr<openni_wrapper::DepthImage>& depth) const
  {   
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud <pcl::PointXYZ>);

    // TODO cloud->header.stamp = time;
    cloud->height       = depth_height_;
    cloud->width        = depth_width_;
    cloud->is_dense     = false;

    cloud->points.resize (cloud->height * cloud->width);

    register float constant = 1.0f / device_->getDepthFocalLength (depth_width_);

    if (device_->isDepthRegistered ())
      cloud->header.frame_id = rgb_frame_id_;
    else
      cloud->header.frame_id = depth_frame_id_;

    register int centerX = (cloud->width >> 1 );
    int centerY = (cloud->height >> 1);

    float bad_point = std::numeric_limits<float>::quiet_NaN ();

    // we have to use Data, since operator[] uses assert -> Debug-mode very slow!
    register const XnDepthPixel* depth_map = depth->getDepthMetaData ().Data();

    register int depth_idx = 0;
    for (int v = -centerY; v < centerY; ++v)
    {
      for (register int u = -centerX; u < centerX; ++u, ++depth_idx)
      {
        pcl::PointXYZ& pt = cloud->points[depth_idx];
        // Check for invalid measurements
        if (depth_map[depth_idx] == 0 ||
            depth_map[depth_idx] == depth->getNoSampleValue () ||
            depth_map[depth_idx] == depth->getShadowValue ())
        {
          // not valid
          pt.x = pt.y = pt.z = bad_point;
          continue;
        }
        pt.z = depth_map[depth_idx] * 0.001;
        pt.x = u * pt.z * constant;
        pt.y = v * pt.z * constant;
      }
    }
    return cloud;
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr OpenNIGrabber::convertToXYZRGBPointCloud (const boost::shared_ptr<openni_wrapper::Image> &image, 
                                                                        const boost::shared_ptr<openni_wrapper::DepthImage> &depth_image) const
  {
    static unsigned rgb_array_size = 0;
    static boost::shared_array<unsigned char> rgb_array(0);
    static unsigned char* rgb_buffer = 0;

    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > cloud (new pcl::PointCloud<pcl::PointXYZRGB>() );

    // do not publish if rgb image is smaller than color image -> seg fault
    if (image->getHeight () < depth_image->getHeight () || image->getWidth () < depth_image->getWidth ())
    {
      // we dont want to flood the terminal with warnings
      static unsigned warned = 0;
      if (warned % 100 == 0)
        printf ("rgb image smaller than depth image... skipping point cloud for this frame rgb:%dx%d vs. depth:%3dx%d\n"
                , image->getWidth (), image->getHeight (), depth_image->getWidth (), depth_image->getHeight () );
      ++warned;
      return cloud;
    }

    cloud->header.frame_id  = rgb_frame_id_;
    cloud->height           = depth_image->getHeight ();
    cloud->width            = depth_image->getWidth ();
    cloud->is_dense         = false;
    
    cloud->points.resize (cloud->height * cloud->width);

    float constant = 1.0f / device_->getImageFocalLength (cloud->width);
    register int centerX = (cloud->width >> 1);
    int centerY = (cloud->height >> 1);

    register const XnDepthPixel* depth_buffer = depth_image->getDepthMetaData ().Data();

    if (rgb_array_size < image_width_ * image_height_ * 3)
    {
      rgb_array_size = image_width_ * image_height_ * 3;
      rgb_array.reset (new unsigned char [rgb_array_size]);
      rgb_buffer = rgb_array.get ();
    }
    image->fillRGB (image_width_, image_height_, rgb_buffer, image_width_ * 3);

    // depth_image already has the desired dimensions, but rgb_msg may be higher res.
    register int color_idx = 0, depth_idx = 0;
    RGBValue color;
    color.Alpha = 0;

    float bad_point = std::numeric_limits<float>::quiet_NaN ();
    
    for (int v = -centerY; v < centerY; ++v)
    {
      for (register int u = -centerX; u < centerX; ++u, color_idx += 3, ++depth_idx)
      {
        pcl::PointXYZRGB& pt = cloud->points[depth_idx];
        /// @todo Different values for these cases
        // Check for invalid measurements
        if (depth_buffer[depth_idx] == 0 ||
            depth_buffer[depth_idx] == depth_image->getNoSampleValue () ||
            depth_buffer[depth_idx] == depth_image->getShadowValue ()) 
        {
          pt.x = pt.y = pt.z = bad_point;
        }
        else
        {
          pt.z = depth_buffer[depth_idx] * 0.001f;
          pt.x = u * pt.z * constant;
          pt.y = v * pt.z * constant;
        }

        // Fill in color
        color.Red   = rgb_buffer[color_idx];
        color.Green = rgb_buffer[color_idx + 1];
        color.Blue  = rgb_buffer[color_idx + 2];
        pt.rgb = color.float_value;
      }
    }
    return (cloud);
  }
  
  // TODO: delete me?
  void OpenNIGrabber::updateModeMaps ()
  {
    XnMapOutputMode output_mode;

    output_mode.nXRes = XN_SXGA_X_RES;
    output_mode.nYRes = XN_SXGA_Y_RES;
    output_mode.nFPS  = 15;
    xn2config_map_[output_mode] = OpenNI_SXGA_15Hz;
    config2xn_map_[OpenNI_SXGA_15Hz] = output_mode;

    output_mode.nXRes = XN_VGA_X_RES;
    output_mode.nYRes = XN_VGA_Y_RES;
    output_mode.nFPS  = 25;
    xn2config_map_[output_mode] = OpenNI_VGA_25Hz;
    config2xn_map_[OpenNI_VGA_25Hz] = output_mode;
    output_mode.nFPS  = 30;
    xn2config_map_[output_mode] = OpenNI_VGA_30Hz;
    config2xn_map_[OpenNI_VGA_30Hz] = output_mode;

    output_mode.nXRes = XN_QVGA_X_RES;
    output_mode.nYRes = XN_QVGA_Y_RES;
    output_mode.nFPS  = 25;
    xn2config_map_[output_mode] = OpenNI_QVGA_25Hz;
    config2xn_map_[OpenNI_QVGA_25Hz] = output_mode;
    output_mode.nFPS  = 30;
    xn2config_map_[output_mode] = OpenNI_QVGA_30Hz;
    config2xn_map_[OpenNI_QVGA_30Hz] = output_mode;
    output_mode.nFPS  = 60;
    xn2config_map_[output_mode] = OpenNI_QVGA_60Hz;
    config2xn_map_[OpenNI_QVGA_60Hz] = output_mode;

    output_mode.nXRes = XN_QQVGA_X_RES;
    output_mode.nYRes = XN_QQVGA_Y_RES;
    output_mode.nFPS  = 25;
    xn2config_map_[output_mode] = OpenNI_QQVGA_25Hz;
    config2xn_map_[OpenNI_QQVGA_25Hz] = output_mode;
    output_mode.nFPS  = 30;
    xn2config_map_[output_mode] = OpenNI_QQVGA_30Hz;
    config2xn_map_[OpenNI_QQVGA_30Hz] = output_mode;
    output_mode.nFPS  = 60;
    xn2config_map_[output_mode] = OpenNI_QQVGA_60Hz;
    config2xn_map_[OpenNI_QQVGA_60Hz] = output_mode;
  }

  // TODO
  bool 
  OpenNIGrabber::isImageModeSupported (int image_mode) const
  {
    XnMapOutputMode image_md;
    if (!mapConfigMode2XnMode (image_mode, image_md))
      return (false);
    XnMapOutputMode compatible_mode;
    if (device_->findCompatibleImageMode (image_md, compatible_mode))
      return (true);
    return (false);
  }

  // TODO
  bool 
  OpenNIGrabber::isDepthModeSupported (int depth_mode) const
  {
    XnMapOutputMode depth_md;
    if (!mapConfigMode2XnMode (depth_mode, depth_md))
      return (false);
    XnMapOutputMode compatible_mode;
    if (device_->findCompatibleDepthMode (depth_md, compatible_mode))
      return (true);
    return (false);
  }

  // TODO: delete me?
  int 
  OpenNIGrabber::mapXnMode2ConfigMode (const XnMapOutputMode& output_mode) const
  {
    std::map<XnMapOutputMode, int, modeComp>::const_iterator it = xn2config_map_.find (output_mode);

    if (it == xn2config_map_.end ())
    {
      printf ("mode %dx%d@%d could not be found\n", output_mode.nXRes, output_mode.nYRes, output_mode.nFPS);
      return (-1);
    }
    else
      return it->second;
  }

  // TODO: delete me?
  bool 
  OpenNIGrabber::mapConfigMode2XnMode (int mode, XnMapOutputMode &xnmode) const
  {
    std::map<int, XnMapOutputMode>::const_iterator it = config2xn_map_.find (mode);
    if (it != config2xn_map_.end ())
    {
      xnmode = it->second;
      return (true);
    }
    else
    {
      printf ("mode %d could not be found\n", mode);
      return (false);
    }
  }

} // namespace

#endif
