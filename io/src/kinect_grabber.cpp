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
 * Author: Nico Blodow (blodow@cs.tum.edu)
 */

#include <pcl/io/kinect_grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace pcl
{

  OpenNIGrabber::OpenNIGrabber (const std::string& device_id)
    : image_callback_registered_(false)
    , depth_image_callback_registered_(false)
    , image_required_(false)
    , depth_required_(false)
    , sync_required_(false)
    , started_(false)
  {
    // create callback signals
    createCallback <sig_cb_openni_image> (); 
    createCallback <sig_cb_openni_depth_image> (); 
    createCallback <sig_cb_openni_image_depth_image> (); 
    createCallback <sig_cb_openni_point_cloud> ();
    createCallback <sig_cb_openni_point_cloud_rgb> ();
    // initialize driver
    onInit (device_id);
  }

  OpenNIGrabber::~OpenNIGrabber ()
  {
    stop ();
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
        num_slots<sig_cb_openni_image_depth_image> () > 0||
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

  unsigned OpenNIGrabber::start ()
  {
    // check if we need to start/stop any stream
    if (image_required_ && !device_->isImageStreamRunning ())
    {
      if (!image_callback_registered_)
      {
        image_callback_handles.push_back (device_->registerImageCallback (&OpenNIGrabber::imageCallback, *this));
        image_callback_registered_ = true;
      }
      device_->startImageStream ();
      startSynchronization ();
    }
    else if (!image_required_ && device_->isImageStreamRunning ())
    {
      stopSynchronization ();
      device_->stopImageStream ();
    }

    if (depth_required_ && !device_->isDepthStreamRunning ())
    {
      if (!depth_image_callback_registered_)
      {
        depth_callback_handles.push_back (device_->registerDepthCallback (&OpenNIGrabber::depthCallback, *this));
        depth_image_callback_registered_ = true;
      }
      // TODO: turn this only on if needed ...
      if (!device_->isDepthRegistered ())
      {
        device_->setDepthRegistration (true);
      }
      device_->startDepthStream ();
      startSynchronization ();
    }
    else if ( !depth_required_ && device_->isDepthStreamRunning ())
    {
      stopSynchronization ();
      device_->stopDepthStream ();
    }
    std::cerr << "streams alive: ";
    if (image_callback_registered_) std::cerr << " image, ";
    if (depth_image_callback_registered_) std::cerr << " depth_image";
    std::cerr << std::endl;

    started_ = true;

    return 0;
  }

  void OpenNIGrabber::stop ()
  {
    std::vector<openni_wrapper::OpenNIDevice::CallbackHandle>::iterator it;
    for (it = image_callback_handles.begin (); it != image_callback_handles.end (); it++)
      device_->unregisterImageCallback (*it);
    image_callback_registered_ = false;
    
    std::vector<openni_wrapper::OpenNIDevice::CallbackHandle>::iterator jt;
    for (jt = depth_callback_handles.begin (); jt != depth_callback_handles.end (); jt++)
      device_->unregisterDepthCallback (*jt);
    depth_image_callback_registered_ = false;

    disconnect_all_slots <sig_cb_openni_image> ();
    disconnect_all_slots <sig_cb_openni_depth_image> ();
    disconnect_all_slots <sig_cb_openni_image_depth_image> ();
    disconnect_all_slots <sig_cb_openni_point_cloud> ();
    disconnect_all_slots <sig_cb_openni_point_cloud_rgb> ();

    started_ = false;
  }

  void OpenNIGrabber::onInit (const std::string& device_id)
  {
    sync.addCallback (boost::bind(&OpenNIGrabber::imageDepthImageCallback, this, _1, _2));

    updateModeMaps ();      // registering mapping from config modes to XnModes and vice versa
    setupDevice (device_id); // will change config_ to default values or user given values from param server

    rgb_frame_id_ = "/openni_rgb_optical_frame";

    depth_frame_id_ = "/openni_depth_optical_frame";
  }
  
  void OpenNIGrabber::signalsChanged ()
  {
    // reevaluate which streams are required
    checkImageStreamRequired ();
    checkDepthStreamRequired ();
    checkImageAndDepthSynchronizationRequired ();
    if (started_)
      start ();
  }

  std::string OpenNIGrabber::getName () const
  {
    return std::string ("OpenNIGrabber");
  }

  void OpenNIGrabber::setupDevice (const std::string& device_id)
  {
    // Initialize the openni device
    openni_wrapper::OpenNIDriver& driver = openni_wrapper::OpenNIDriver::getInstance ();

    if (driver.getNumberDevices () == 0)
    {
      printf ("[%s] No devices connected.\n", getName ().c_str ());
      exit (-1);
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
      if (device_id.empty ())
      {
        printf ("[%s] device_id is not set! Using first device.\n", getName ().c_str ());
        device_ = driver.getDeviceByIndex (0);
      }
      else if (device_id.find ('@') != std::string::npos)
      {
        size_t pos = device_id.find ('@');
        unsigned bus = atoi (device_id.substr (0, pos).c_str ());
        unsigned address = atoi (device_id.substr (pos + 1, device_id.length () - pos - 1).c_str ());
        printf ("[%s] searching for device with bus@address = %d@%d\n", getName ().c_str (), bus, address);
        device_ = driver.getDeviceByAddress (bus, address);
      }
      else if (device_id[0] == '#')
      {
        unsigned index = atoi (device_id.c_str () + 1);
        printf ("[%s] searching for device with index = %d\n", getName ().c_str (), index);
        device_ = driver.getDeviceByIndex (index - 1);
      }
      else
      {
        printf ("[%s] searching for device with serial number = %s\n", getName ().c_str (), device_id.c_str ());
        device_ = driver.getDeviceBySerialNumber (device_id);
      }
    }
    catch (const openni_wrapper::OpenNIException& exception)
    {
      if (!device_)
      {
        printf ("[%s] No matching device found.\n", getName ().c_str ());
        exit (-1);
      }
      else
      {
        printf ("[%s] could not retrieve device. Reason %s\n", getName ().c_str (), exception.what ());
        exit (-1);
      }
    }
    catch(...)
    {
      printf ("[%s] unknown error occured\n", getName ().c_str ());
      exit (-1);
    }
    printf ("[%s] Opened '%s' on bus %d:%d with serial number '%s'\n", getName ().c_str (),
              device_->getProductName (), device_->getBus (), device_->getAddress (), device_->getSerialNumber ());

    bool registration = false;

    int debayering_method = 0;

    int image_mode = mapXnMode2ConfigMode (device_->getDefaultImageMode ());

    int depth_mode = mapXnMode2ConfigMode (device_->getDefaultDepthMode ());

    XnMapOutputMode image_md = mapConfigMode2XnMode ( image_mode);
    image_width_  = image_md.nXRes;
    image_height_ = image_md.nYRes;

    XnMapOutputMode depth_md = mapConfigMode2XnMode ( depth_mode);
    depth_width_  = depth_md.nXRes;
    depth_height_ = depth_md.nYRes;
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
        sync.add0 (image, image->getTimeStamp());

    boost::signals2::signal<sig_cb_openni_image>* signal = find_signal <sig_cb_openni_image> ();
    if (signal && signal->num_slots () > 0)
        signal->operator()(image);

    return;
  }

  void OpenNIGrabber::depthCallback (boost::shared_ptr<openni_wrapper::DepthImage> depth_image, void* cookie)
  {
    if (num_slots<sig_cb_openni_point_cloud_rgb> () > 0 ||
        num_slots<sig_cb_openni_image_depth_image> () > 0)
        sync.add1 (depth_image, depth_image->getTimeStamp());

    boost::signals2::signal<sig_cb_openni_depth_image>* signal = find_signal <sig_cb_openni_depth_image> ();
    if (signal && signal->num_slots () > 0)
        signal->operator()(depth_image);

    boost::signals2::signal<sig_cb_openni_point_cloud>* signalXYZ = find_signal <sig_cb_openni_point_cloud> ();
    if (signalXYZ && signalXYZ->num_slots () > 0)
      signalXYZ->operator()(convertToXYZPointCloud (depth_image));

    return;
  }

  void OpenNIGrabber::imageDepthImageCallback (const boost::shared_ptr<openni_wrapper::Image> &image, const boost::shared_ptr<openni_wrapper::DepthImage> &depth_image)
  {
    // check if we have color point cloud slots
    boost::signals2::signal<sig_cb_openni_point_cloud_rgb>* signal = find_signal <sig_cb_openni_point_cloud_rgb> ();
    if (signal && signal->num_slots () > 0)
        signal->operator()(convertToXYZRGBPointCloud (image, depth_image));

    boost::signals2::signal<sig_cb_openni_image_depth_image>* signalImageDepth = find_signal <sig_cb_openni_image_depth_image> ();
    if (signalImageDepth && signalImageDepth->num_slots () > 0)
    {
      float constant = 0.001 / device_->getDepthFocalLength (depth_width_);
      signalImageDepth->operator()(image, depth_image, constant);
    }
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr OpenNIGrabber::convertToXYZPointCloud (const boost::shared_ptr<openni_wrapper::DepthImage>& depth) const
  {
    const xn::DepthMetaData& depth_md = depth->getDepthMetaData ();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud <pcl::PointXYZ>);
    
    // TODO cloud->header.stamp = time;
    cloud->height       = depth_height_;
    cloud->width        = depth_width_;
    cloud->is_dense     = false;

    cloud->points.resize (cloud->height * cloud->width);

    float constant = 0.001 / device_->getDepthFocalLength (depth_width_);

    if (device_->isDepthRegistered ())
      cloud->header.frame_id = rgb_frame_id_;
    else
      cloud->header.frame_id = depth_frame_id_;

    float centerX = (cloud->width >> 1 ) - 0.5f;
    float centerY = (cloud->height >> 1) - 0.5f;

    float bad_point = std::numeric_limits<float>::quiet_NaN ();

    unsigned depthStep = depth_md.XRes () / cloud->width;
    unsigned depthSkip = (depth_md.YRes () / cloud->height - 1) * depth_md.XRes ();
    int depth_idx = 0;
    pcl::PointCloud<pcl::PointXYZ>::iterator pt_iter = cloud->begin ();
    for (int v = 0; v < (int)cloud->height; ++v, depth_idx += depthSkip)
    {
      for (int u = 0; u < (int)cloud->width; ++u, depth_idx += depthStep, ++pt_iter)
      {
        pcl::PointXYZ& pt = *pt_iter;

        // Check for invalid measurements
        if (depth_md[depth_idx] == 0 ||
            depth_md[depth_idx] == depth->getNoSampleValue () ||
            depth_md[depth_idx] == depth->getShadowValue ())
        {
          // not valid
          pt.x = pt.y = pt.z = bad_point;
          continue;
        }

        // Fill in XYZ
        pt.x = (u - centerX) * depth_md[depth_idx] * constant;
        pt.y = (v - centerY) * depth_md[depth_idx] * constant;
        pt.z = depth_md[depth_idx] * 0.001;
      }
    }

    return cloud;
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr OpenNIGrabber::convertToXYZRGBPointCloud (const boost::shared_ptr<openni_wrapper::Image> &image, 
                                                                        const boost::shared_ptr<openni_wrapper::DepthImage> &depth_image) const
  {
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
    float centerX = (cloud->width >> 1) - 0.5f;
    float centerY = (cloud->height >> 1) - 0.5f;
    float* depth_buffer = new float [depth_width_ * depth_height_];
    depth_image->fillDepthImage (depth_width_, depth_height_, depth_buffer, depth_width_ * sizeof (float));

    unsigned char* rgb_buffer = new unsigned char [image_width_ * image_height_ * 3];
    image->fillRGB (image_width_, image_height_, rgb_buffer, image_width_ * 3);

    // depth_image already has the desired dimensions, but rgb_msg may be higher res.
    unsigned color_step = 3 * image_width_ / cloud->width;
    unsigned color_skip = 3 * (image_height_ / cloud->height - 1) * image_height_;
    int color_idx = 0, depth_idx = 0;
    pcl::PointCloud<pcl::PointXYZRGB>::iterator pt_iter = cloud->begin ();
    for (int v = 0; v < (int)cloud->height; ++v, color_idx += color_skip)
    {
      for (int u = 0; u < (int)cloud->width; ++u, color_idx += color_step, ++depth_idx, ++pt_iter)
      {
        pcl::PointXYZRGB& pt = *pt_iter;
        float Z = depth_buffer[depth_idx];

        // Check for invalid measurements
        if (std::isnan (Z))
        {
          pt.x = pt.y = pt.z = Z;
        }
        else
        {
          // Fill in XYZ
          pt.x = (u - centerX) * Z * constant;
          pt.y = (v - centerY) * Z * constant;
          pt.z = Z;
        }

        // Fill in color
        RGBValue color;
        color.Red   = rgb_buffer[color_idx];
        color.Green = rgb_buffer[color_idx + 1];
        color.Blue  = rgb_buffer[color_idx + 2];
        color.Alpha = 0;
        pt.rgb = color.float_value;
      }
    }
    delete (depth_buffer);
    delete (rgb_buffer);
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
  bool OpenNIGrabber::isImageModeSupported (int image_mode) const
  {
    XnMapOutputMode image_md = mapConfigMode2XnMode (image_mode);
    XnMapOutputMode compatible_mode;
    if (device_->findCompatibleImageMode (image_md, compatible_mode))
      return true;
    return false;
  }

  // TODO
  bool OpenNIGrabber::isDepthModeSupported (int depth_mode) const
  {
    XnMapOutputMode depth_md = mapConfigMode2XnMode (depth_mode);
    XnMapOutputMode compatible_mode;
    if (device_->findCompatibleDepthMode (depth_md, compatible_mode))
      return true;
    return false;
  }

  // TODO: delete me?
  int OpenNIGrabber::mapXnMode2ConfigMode (const XnMapOutputMode& output_mode) const
  {
    std::map<XnMapOutputMode, int, modeComp>::const_iterator it = xn2config_map_.find (output_mode);

    if (it == xn2config_map_.end ())
    {
      printf ("mode %dx%d@%d could not be found\n", output_mode.nXRes, output_mode.nYRes, output_mode.nFPS);
      exit (-1);
    }
    else
      return it->second;
  }

  // TODO: delete me?
  XnMapOutputMode OpenNIGrabber::mapConfigMode2XnMode (int mode) const
  {
    std::map<int, XnMapOutputMode>::const_iterator it = config2xn_map_.find (mode);
    if (it == config2xn_map_.end ())
    {
      printf ("mode %d could not be found\n", mode);
      exit (-1);
    }
    else
      return it->second;
  }

} // namespace


