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
 *         Radu Bogdan Rusu (rusu@willowgarage.com)
 *         Suat Gedikli (gedikli@willowgarage.com)
 *         Ethan Rublee (rublee@willowgarage.com)
 */

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h> //fps calculations
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/io/openni_camera/openni_driver.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/mouse_event.h>

#include <vtkImageViewer.h>
#include <vtkImageImport.h>

#include <mutex>
#include <vector>
#include <string>

#define SHOW_FPS 1
#if SHOW_FPS
#define FPS_CALC(_WHAT_) \
do \
{ \
    static unsigned count = 0;\
    static double last = pcl::getTime ();\
    double now = pcl::getTime (); \
    ++count; \
    if (now - last >= 1.0) \
    { \
      std::cout << "Average framerate("<< _WHAT_ << "): " << double(count)/double(now - last) << " Hz" <<  std::endl; \
      count = 0; \
      last = now; \
    } \
}while(false)
#else
#define FPS_CALC(_WHAT_) \
do \
{ \
}while(false)
#endif

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointType>
class SimpleOpenNIViewer
{
  public:
    using Cloud = pcl::PointCloud<PointType>;
    using CloudConstPtr = typename Cloud::ConstPtr;

    SimpleOpenNIViewer (pcl::OpenNIGrabber& grabber)
      : cloud_viewer_ ("PCL OpenNI Viewer")
      , grabber_ (grabber)
      , image_viewer_ ("PCL image viewer")
    {
    }

    void
    cloud_callback (const CloudConstPtr& cloud)
    {
      FPS_CALC ("cloud callback");
      std::lock_guard<std::mutex> lock (cloud_mutex_);
      cloud_ = cloud;
    }

    void
    image_callback (const openni_wrapper::Image::Ptr& image)
    {
      FPS_CALC ("image callback");
      std::lock_guard<std::mutex> lock (image_mutex_);
      image_ = image;
    }
    
    openni_wrapper::Image::Ptr
    getLatestImage ()
    {
      std::lock_guard<std::mutex> lock(image_mutex_);
      openni_wrapper::Image::Ptr temp_image;
      temp_image.swap (image_);
      return (temp_image);
    }    
    
    void 
    keyboard_callback (const pcl::visualization::KeyboardEvent& event, void* cookie)
    {
      string* message = (string*)cookie;
      std::cout << (*message) << " :: ";
      if (event.getKeyCode())
        std::cout << "the key \'" << event.getKeyCode() << "\' (" << (int)event.getKeyCode() << ") was";
      else
        std::cout << "the special key \'" << event.getKeySym() << "\' was";
      if (event.keyDown())
        std::cout << " pressed" << std::endl;
      else
        std::cout << " released" << std::endl;
    }
    
    void mouse_callback (const pcl::visualization::MouseEvent& mouse_event, void* cookie)
    {
      string* message = (string*) cookie;
      if (mouse_event.getType() == pcl::visualization::MouseEvent::MouseButtonPress && mouse_event.getButton() == pcl::visualization::MouseEvent::LeftButton)
      {
        std::cout << (*message) << " :: " << mouse_event.getX () << " , " << mouse_event.getY () << std::endl;
      }
    }
    /**
     * @brief swaps the pointer to the point cloud with Null pointer and returns the cloud pointer
     * @return boost shared pointer to point cloud
     */
    CloudConstPtr
    getLatestCloud ()
    {
      //lock while we swap our cloud and reset it.
      std::lock_guard<std::mutex> lock(cloud_mutex_);
      CloudConstPtr temp_cloud;
      temp_cloud.swap (cloud_); //here we set cloud_ to null, so that
      //it is safe to set it again from our
      //callback
      return (temp_cloud);
    }

    /**
     * @brief starts the main loop
     */
    void
    run ()
    {
      //pcl::Grabber* interface = new pcl::OpenNIGrabber(device_id_, pcl::OpenNIGrabber::OpenNI_QQVGA_30Hz, pcl::OpenNIGrabber::OpenNI_VGA_30Hz);

      std::string mouseMsg3D("Mouse coordinates in PCL Visualizer");
      std::string keyMsg3D("Key event for PCL Visualizer");
      cloud_viewer_.registerMouseCallback (&SimpleOpenNIViewer::mouse_callback, *this, (void*)(&mouseMsg3D));    
      cloud_viewer_.registerKeyboardCallback(&SimpleOpenNIViewer::keyboard_callback, *this, (void*)(&keyMsg3D));
      std::function<void (const CloudConstPtr&)> cloud_cb = [this] (const CloudConstPtr& cloud) { cloud_callback (cloud); };
      boost::signals2::connection cloud_connection = grabber_.registerCallback (cloud_cb);
      
      boost::signals2::connection image_connection;
      if (grabber_.providesCallback<void (const openni_wrapper::Image::Ptr&)>())
      {
          std::string mouseMsg2D("Mouse coordinates in image viewer");
          std::string keyMsg2D("Key event for image viewer");
          image_viewer_.registerMouseCallback (&SimpleOpenNIViewer::mouse_callback, *this, (void*)(&mouseMsg2D));
          image_viewer_.registerKeyboardCallback(&SimpleOpenNIViewer::keyboard_callback, *this, (void*)(&keyMsg2D));
          std::function<void (const openni_wrapper::Image::Ptr&)> image_cb = [this] (const openni_wrapper::Image::Ptr& img) { image_callback (img); };
          image_connection = grabber_.registerCallback (image_cb);
      }
      unsigned char* rgb_data = 0;
      unsigned rgb_data_size = 0;
      
      grabber_.start ();
      
      while (!cloud_viewer_.wasStopped(1))
      {
        if (cloud_)
        {
          FPS_CALC ("drawing cloud");
          //the call to get() sets the cloud_ to null;
          cloud_viewer_.showCloud (getLatestCloud ());
        }
        if (image_)
        {
          auto image = getLatestImage ();
          
          if (image->getEncoding() == openni_wrapper::Image::RGB)
          {
            image_viewer_.showRGBImage(image->getMetaData().Data(), image->getWidth(), image->getHeight());
          }
          else
          {
            if (rgb_data_size < image->getWidth() * image->getHeight())
            {
              rgb_data_size = image->getWidth() * image->getHeight();
              rgb_data = new unsigned char [rgb_data_size * 3];
            }
            image->fillRGB (image->getWidth(), image->getHeight(), rgb_data);
            image_viewer_.showRGBImage(rgb_data, image->getWidth(), image->getHeight());
          }
          // This will crash: image_viewer_.spinOnce (10);
        }
      }

      grabber_.stop();
      
      cloud_connection.disconnect();
      image_connection.disconnect();
      delete[] rgb_data;
    }

    pcl::visualization::CloudViewer cloud_viewer_;
    pcl::OpenNIGrabber& grabber_;
    std::mutex cloud_mutex_;
    CloudConstPtr cloud_;
    
    std::mutex image_mutex_;
    openni_wrapper::Image::Ptr image_;
    pcl::visualization::ImageViewer image_viewer_;
};

void
usage(char ** argv)
{
  std::cout << "usage: " << argv[0] << " [((<device_id> | <path-to-oni-file>) [-depthmode <mode>] [-imagemode <mode>] [-xyz] | -l [<device_id>]| -h | --help)]" << std::endl;
  std::cout << argv[0] << " -h | --help : shows this help" << std::endl;
  std::cout << argv[0] << " -l : list all available devices" << std::endl;
  std::cout << argv[0] << " -l <device-id> : list all available modes for specified device" << std::endl;

  std::cout << "                 device_id may be #1, #2, ... for the first, second etc device in the list"
#ifndef _WIN32
       << " or" << std::endl
       << "                 bus@address for the device connected to a specific usb-bus / address combination or" << std::endl
       << "                 <serial-number>"
#endif
       << std::endl;
  std::cout << std::endl;
  std::cout << "examples:" << std::endl;
  std::cout << argv[0] << " \"#1\"" << std::endl;
  std::cout << "    uses the first device." << std::endl;
  std::cout << argv[0] << " \"./temp/test.oni\"" << std::endl;
  std::cout << "    uses the oni-player device to play back oni file given by path." << std::endl;
  std::cout << argv[0] << " -l" << std::endl;
  std::cout << "    lists all available devices." << std::endl;
  std::cout << argv[0] << " -l \"#2\"" << std::endl;
  std::cout << "    lists all available modes for the second device" << std::endl;
  #ifndef _WIN32
  std::cout << argv[0] << " A00361800903049A" << std::endl;
  std::cout << "    uses the device with the serial number \'A00361800903049A\'." << std::endl;
  std::cout << argv[0] << " 1@16" << std::endl;
  std::cout << "    uses the device on address 16 at USB bus 1." << std::endl;
  #endif
  return;
}

int
main(int argc, char ** argv)
{
  std::string device_id("");
  pcl::OpenNIGrabber::Mode depth_mode = pcl::OpenNIGrabber::OpenNI_Default_Mode;
  pcl::OpenNIGrabber::Mode image_mode = pcl::OpenNIGrabber::OpenNI_Default_Mode;
  bool xyz = false;
  
  if (argc >= 2)
  {
    device_id = argv[1];
    if (device_id == "--help" || device_id == "-h")
    {
      usage(argv);
      return 0;
    }
    else if (device_id == "-l")
    {
      if (argc >= 3)
      {
        pcl::OpenNIGrabber grabber(argv[2]);
        auto device = grabber.getDevice();
        std::cout << "Supported depth modes for device: " << device->getVendorName() << " , " << device->getProductName() << std::endl;
        std::vector<std::pair<int, XnMapOutputMode > > modes = grabber.getAvailableDepthModes();
        for (const auto& mode : modes)
        {
          std::cout << mode.first << " = " << mode.second.nXRes << " x " << mode.second.nYRes << " @ " << mode.second.nFPS << std::endl;
        }

        if (device->hasImageStream ())
        {
          std::cout << std::endl << "Supported image modes for device: " << device->getVendorName() << " , " << device->getProductName() << std::endl;
          modes = grabber.getAvailableImageModes();
          for (const auto& mode : modes)
          {
            std::cout << mode.first << " = " << mode.second.nXRes << " x " << mode.second.nYRes << " @ " << mode.second.nFPS << std::endl;
          }
        }
      }
      else
      {
        openni_wrapper::OpenNIDriver& driver = openni_wrapper::OpenNIDriver::getInstance();
        if (driver.getNumberDevices() > 0)
        {
          for (unsigned deviceIdx = 0; deviceIdx < driver.getNumberDevices(); ++deviceIdx)
          {
            std::cout << "Device: " << deviceIdx + 1 << ", vendor: " << driver.getVendorName(deviceIdx) << ", product: " << driver.getProductName(deviceIdx)
              << ", connected: " << (int) driver.getBus(deviceIdx) << " @ " << (int) driver.getAddress(deviceIdx) << ", serial number: \'" << driver.getSerialNumber(deviceIdx) << "\'" << std::endl;
          }

        }
        else
          std::cout << "No devices connected." << std::endl;
        
        std::cout <<"Virtual Devices available: ONI player" << std::endl;
      }
      return 0;
    }
  }
  else
  {
    openni_wrapper::OpenNIDriver& driver = openni_wrapper::OpenNIDriver::getInstance();
    if (driver.getNumberDevices() > 0)
      std::cout << "Device Id not set, using first device." << std::endl;
  }
  
  unsigned mode;
  if (pcl::console::parse(argc, argv, "-depthmode", mode) != -1)
    depth_mode = (pcl::OpenNIGrabber::Mode) mode;

  if (pcl::console::parse(argc, argv, "-imagemode", mode) != -1)
    image_mode = (pcl::OpenNIGrabber::Mode) mode;
  
  if (pcl::console::find_argument(argc, argv, "-xyz") != -1)
    xyz = true;
  
  try
  {
    pcl::OpenNIGrabber grabber (device_id, depth_mode, image_mode);
  
    if (xyz) // only if xyz flag is set, since grabber provides at least XYZ and XYZI pointclouds
    {
      SimpleOpenNIViewer<pcl::PointXYZ> v (grabber);
      v.run ();
    }
    else if (grabber.providesCallback<pcl::OpenNIGrabber::sig_cb_openni_point_cloud_rgb> ())
    {
      SimpleOpenNIViewer<pcl::PointXYZRGBA> v (grabber);
      v.run ();
    }
    else
    {
      SimpleOpenNIViewer<pcl::PointXYZI> v (grabber);
      v.run ();
    }
  }
  catch (pcl::IOException& e)
  {
    pcl::console::print_error ("Failed to create a grabber: %s\n", e.what ());
    return (1);
  }

  return (0);
}
