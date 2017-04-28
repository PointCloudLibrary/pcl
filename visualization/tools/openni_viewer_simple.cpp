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
#include <pcl/visualization/boost.h>
#include <pcl/visualization/mouse_event.h>
#include <vtkImageViewer.h>
#include <vtkImageImport.h>
#include <vector>
#include <string>

using namespace std;

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
    typedef pcl::PointCloud<PointType> Cloud;
    typedef typename Cloud::ConstPtr CloudConstPtr;

    SimpleOpenNIViewer (pcl::OpenNIGrabber& grabber)
      : cloud_viewer_ ("PCL OpenNI Viewer")
      , grabber_ (grabber)
#if !((VTK_MAJOR_VERSION == 5)&&(VTK_MINOR_VERSION <= 4))
      , image_viewer_ ("PCL image viewer")
#endif
    {
    }

    void
    cloud_callback (const CloudConstPtr& cloud)
    {
      FPS_CALC ("cloud callback");
      boost::mutex::scoped_lock lock (cloud_mutex_);
      cloud_ = cloud;
    }

#if !((VTK_MAJOR_VERSION == 5)&&(VTK_MINOR_VERSION <= 4))
    void
    image_callback (const boost::shared_ptr<openni_wrapper::Image>& image)
    {
      FPS_CALC ("image callback");
      boost::mutex::scoped_lock lock (image_mutex_);
      image_ = image;
    }
    
    boost::shared_ptr<openni_wrapper::Image>
    getLatestImage ()
    {
      boost::mutex::scoped_lock lock(image_mutex_);
      boost::shared_ptr<openni_wrapper::Image> temp_image;
      temp_image.swap (image_);
      return (temp_image);
    }    
#endif
    
    void 
    keyboard_callback (const pcl::visualization::KeyboardEvent& event, void* cookie)
    {
      string* message = (string*)cookie;
      cout << (*message) << " :: ";
      if (event.getKeyCode())
        cout << "the key \'" << event.getKeyCode() << "\' (" << (int)event.getKeyCode() << ") was";
      else
        cout << "the special key \'" << event.getKeySym() << "\' was";
      if (event.keyDown())
        cout << " pressed" << endl;
      else
        cout << " released" << endl;
    }
    
    void mouse_callback (const pcl::visualization::MouseEvent& mouse_event, void* cookie)
    {
      string* message = (string*) cookie;
      if (mouse_event.getType() == pcl::visualization::MouseEvent::MouseButtonPress && mouse_event.getButton() == pcl::visualization::MouseEvent::LeftButton)
      {
        cout << (*message) << " :: " << mouse_event.getX () << " , " << mouse_event.getY () << endl;
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
      boost::mutex::scoped_lock lock(cloud_mutex_);
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

      string mouseMsg3D("Mouse coordinates in PCL Visualizer");
      string keyMsg3D("Key event for PCL Visualizer");
      cloud_viewer_.registerMouseCallback (&SimpleOpenNIViewer::mouse_callback, *this, (void*)(&mouseMsg3D));    
      cloud_viewer_.registerKeyboardCallback(&SimpleOpenNIViewer::keyboard_callback, *this, (void*)(&keyMsg3D));
      boost::function<void (const CloudConstPtr&) > cloud_cb = boost::bind (&SimpleOpenNIViewer::cloud_callback, this, _1);
      boost::signals2::connection cloud_connection = grabber_.registerCallback (cloud_cb);
      
#if !((VTK_MAJOR_VERSION == 5)&&(VTK_MINOR_VERSION <= 4))
      boost::signals2::connection image_connection;
      if (grabber_.providesCallback<void (const boost::shared_ptr<openni_wrapper::Image>&)>())
      {
          string mouseMsg2D("Mouse coordinates in image viewer");
          string keyMsg2D("Key event for image viewer");
          image_viewer_.registerMouseCallback (&SimpleOpenNIViewer::mouse_callback, *this, (void*)(&mouseMsg2D));
          image_viewer_.registerKeyboardCallback(&SimpleOpenNIViewer::keyboard_callback, *this, (void*)(&keyMsg2D));
          boost::function<void (const boost::shared_ptr<openni_wrapper::Image>&) > image_cb = boost::bind (&SimpleOpenNIViewer::image_callback, this, _1);
          image_connection = grabber_.registerCallback (image_cb);
      }
      unsigned char* rgb_data = 0;
      unsigned rgb_data_size = 0;
#endif        
      
      grabber_.start ();
      
      while (!cloud_viewer_.wasStopped(1))
      {
        if (cloud_)
        {
          FPS_CALC ("drawing cloud");
          //the call to get() sets the cloud_ to null;
          cloud_viewer_.showCloud (getLatestCloud ());
        }
#if !((VTK_MAJOR_VERSION == 5)&&(VTK_MINOR_VERSION <= 4))        
        if (image_)
        {
          boost::shared_ptr<openni_wrapper::Image> image = getLatestImage ();
          
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
#endif        
      }

      grabber_.stop();
      
      cloud_connection.disconnect();
#if !((VTK_MAJOR_VERSION == 5)&&(VTK_MINOR_VERSION <= 4))      
      image_connection.disconnect();
      if (rgb_data)
        delete[] rgb_data;
#endif      
    }

    pcl::visualization::CloudViewer cloud_viewer_;
    pcl::OpenNIGrabber& grabber_;
    boost::mutex cloud_mutex_;
    CloudConstPtr cloud_;
    
#if !((VTK_MAJOR_VERSION == 5)&&(VTK_MINOR_VERSION <= 4))    
    boost::mutex image_mutex_;
    boost::shared_ptr<openni_wrapper::Image> image_;
    pcl::visualization::ImageViewer image_viewer_;
#endif    
};

void
usage(char ** argv)
{
  cout << "usage: " << argv[0] << " [((<device_id> | <path-to-oni-file>) [-depthmode <mode>] [-imagemode <mode>] [-xyz] | -l [<device_id>]| -h | --help)]" << endl;
  cout << argv[0] << " -h | --help : shows this help" << endl;
  cout << argv[0] << " -l : list all available devices" << endl;
  cout << argv[0] << " -l <device-id> : list all available modes for specified device" << endl;

  cout << "                 device_id may be #1, #2, ... for the first, second etc device in the list"
#ifndef _WIN32
       << " or" << endl
       << "                 bus@address for the device connected to a specific usb-bus / address combination or" << endl
       << "                 <serial-number>"
#endif
       << endl;
  cout << endl;
  cout << "examples:" << endl;
  cout << argv[0] << " \"#1\"" << endl;
  cout << "    uses the first device." << endl;
  cout << argv[0] << " \"./temp/test.oni\"" << endl;
  cout << "    uses the oni-player device to play back oni file given by path." << endl;
  cout << argv[0] << " -l" << endl;
  cout << "    lists all available devices." << endl;
  cout << argv[0] << " -l \"#2\"" << endl;
  cout << "    lists all available modes for the second device" << endl;
  #ifndef _WIN32
  cout << argv[0] << " A00361800903049A" << endl;
  cout << "    uses the device with the serial number \'A00361800903049A\'." << endl;
  cout << argv[0] << " 1@16" << endl;
  cout << "    uses the device on address 16 at USB bus 1." << endl;
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
        boost::shared_ptr<openni_wrapper::OpenNIDevice> device = grabber.getDevice();
        cout << "Supported depth modes for device: " << device->getVendorName() << " , " << device->getProductName() << endl;
        std::vector<std::pair<int, XnMapOutputMode > > modes = grabber.getAvailableDepthModes();
        for (std::vector<std::pair<int, XnMapOutputMode > >::const_iterator it = modes.begin(); it != modes.end(); ++it)
        {
          cout << it->first << " = " << it->second.nXRes << " x " << it->second.nYRes << " @ " << it->second.nFPS << endl;
        }

        if (device->hasImageStream ())
        {
          cout << endl << "Supported image modes for device: " << device->getVendorName() << " , " << device->getProductName() << endl;
          modes = grabber.getAvailableImageModes();
          for (std::vector<std::pair<int, XnMapOutputMode > >::const_iterator it = modes.begin(); it != modes.end(); ++it)
          {
            cout << it->first << " = " << it->second.nXRes << " x " << it->second.nYRes << " @ " << it->second.nFPS << endl;
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
            cout << "Device: " << deviceIdx + 1 << ", vendor: " << driver.getVendorName(deviceIdx) << ", product: " << driver.getProductName(deviceIdx)
              << ", connected: " << (int) driver.getBus(deviceIdx) << " @ " << (int) driver.getAddress(deviceIdx) << ", serial number: \'" << driver.getSerialNumber(deviceIdx) << "\'" << endl;
          }

        }
        else
          cout << "No devices connected." << endl;
        
        cout <<"Virtual Devices available: ONI player" << endl;
      }
      return 0;
    }
  }
  else
  {
    openni_wrapper::OpenNIDriver& driver = openni_wrapper::OpenNIDriver::getInstance();
    if (driver.getNumberDevices() > 0)
      cout << "Device Id not set, using first device." << endl;
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
