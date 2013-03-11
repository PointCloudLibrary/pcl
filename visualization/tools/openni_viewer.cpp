/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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
 * $Id$
 *
 */

#define MEASURE_FUNCTION_TIME
#include <pcl/common/time.h> //fps calculations
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/boost.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>

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

void
printHelp (int, char **argv)
{
  using pcl::console::print_error;
  using pcl::console::print_info;

  print_error ("Syntax is: %s [((<device_id> | <path-to-oni-file>) [-depthmode <mode>] [-imagemode <mode>] [-xyz] | -l [<device_id>]| -h | --help)]\n", argv [0]);
  print_info ("%s -h | --help : shows this help\n", argv [0]);
  print_info ("%s -xyz : use only XYZ values and ignore RGB components (this flag is required for use with ASUS Xtion Pro) \n", argv [0]);
  print_info ("%s -l : list all available devices\n", argv [0]);
  print_info ("%s -l <device-id> :list all available modes for specified device\n", argv [0]);
  print_info ("\t\t<device_id> may be \"#1\", \"#2\", ... for the first, second etc device in the list\n");
#ifndef _WIN32
  print_info ("\t\t                   bus@address for the device connected to a specific usb-bus / address combination\n");
  print_info ("\t\t                   <serial-number>\n");
#endif
  print_info ("\n\nexamples:\n");
  print_info ("%s \"#1\"\n", argv [0]);
  print_info ("\t\t uses the first device.\n");
  print_info ("%s  \"./temp/test.oni\"\n", argv [0]);
  print_info ("\t\t uses the oni-player device to play back oni file given by path.\n");
  print_info ("%s -l\n", argv [0]);
  print_info ("\t\t list all available devices.\n");
  print_info ("%s -l \"#2\"\n", argv [0]);
  print_info ("\t\t list all available modes for the second device.\n");
  #ifndef _WIN32
  print_info ("%s A00361800903049A\n", argv [0]);
  print_info ("\t\t uses the device with the serial number \'A00361800903049A\'.\n");
  print_info ("%s 1@16\n", argv [0]);
  print_info ("\t\t uses the device on address 16 at USB bus 1.\n");
  #endif
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointType>
class OpenNIViewer
{
  public:
    typedef pcl::PointCloud<PointType> Cloud;
    typedef typename Cloud::ConstPtr CloudConstPtr;

    OpenNIViewer (pcl::Grabber& grabber)
      : cloud_viewer_ (new pcl::visualization::PCLVisualizer ("PCL OpenNI cloud"))
      , image_viewer_ ()
      , grabber_ (grabber)
      , rgb_data_ (0), rgb_data_size_ (0)
    {
    }

    void
    cloud_callback (const CloudConstPtr& cloud)
    {
      FPS_CALC ("cloud callback");
      boost::mutex::scoped_lock lock (cloud_mutex_);
      cloud_ = cloud;
    }

    void
    image_callback (const boost::shared_ptr<openni_wrapper::Image>& image)
    {
      FPS_CALC ("image callback");
      boost::mutex::scoped_lock lock (image_mutex_);
      image_ = image;
      
      if (image->getEncoding () != openni_wrapper::Image::RGB)
      {
        if (rgb_data_size_ < image->getWidth () * image->getHeight ())
        {
          if (rgb_data_)
            delete [] rgb_data_;
          rgb_data_size_ = image->getWidth () * image->getHeight ();
          rgb_data_ = new unsigned char [rgb_data_size_ * 3];
        }
        image_->fillRGB (image_->getWidth (), image_->getHeight (), rgb_data_);
      }
    }
    
    void 
    keyboard_callback (const pcl::visualization::KeyboardEvent& event, void*)
    {
      if (event.getKeyCode ())
        cout << "the key \'" << event.getKeyCode() << "\' (" << event.getKeyCode() << ") was";
      else
        cout << "the special key \'" << event.getKeySym() << "\' was";
      if (event.keyDown())
        cout << " pressed" << endl;
      else
        cout << " released" << endl;
    }
    
    void 
    mouse_callback (const pcl::visualization::MouseEvent& mouse_event, void*)
    {
      if (mouse_event.getType() == pcl::visualization::MouseEvent::MouseButtonPress && mouse_event.getButton() == pcl::visualization::MouseEvent::LeftButton)
      {
        cout << "left button pressed @ " << mouse_event.getX () << " , " << mouse_event.getY () << endl;
      }
    }

    /**
     * @brief starts the main loop
     */
    void
    run ()
    {
      cloud_viewer_->registerMouseCallback (&OpenNIViewer::mouse_callback, *this);
      cloud_viewer_->registerKeyboardCallback(&OpenNIViewer::keyboard_callback, *this);
      boost::function<void (const CloudConstPtr&) > cloud_cb = boost::bind (&OpenNIViewer::cloud_callback, this, _1);
      boost::signals2::connection cloud_connection = grabber_.registerCallback (cloud_cb);
      
      boost::signals2::connection image_connection;
      if (grabber_.providesCallback<void (const boost::shared_ptr<openni_wrapper::Image>&)>())
      {
        image_viewer_.reset (new pcl::visualization::ImageViewer ("PCL OpenNI image"));
        image_viewer_->registerMouseCallback (&OpenNIViewer::mouse_callback, *this);
        image_viewer_->registerKeyboardCallback(&OpenNIViewer::keyboard_callback, *this);
        boost::function<void (const boost::shared_ptr<openni_wrapper::Image>&) > image_cb = boost::bind (&OpenNIViewer::image_callback, this, _1);
        image_connection = grabber_.registerCallback (image_cb);
      }
      
      bool image_init = false, cloud_init = false;
      
      grabber_.start ();

      while (!cloud_viewer_->wasStopped () && (image_viewer_ && !image_viewer_->wasStopped ()))
      {
        boost::shared_ptr<openni_wrapper::Image> image;
        CloudConstPtr cloud;

        cloud_viewer_->spinOnce ();

        // See if we can get a cloud
        if (cloud_mutex_.try_lock ())
        {
          cloud_.swap (cloud);
          cloud_mutex_.unlock ();
        }

        if (cloud)
        {
          FPS_CALC ("drawing cloud");
          
          if (!cloud_init)
          {
            cloud_viewer_->setPosition (0, 0);
            cloud_viewer_->setSize (cloud->width, cloud->height);
            cloud_init = !cloud_init;
          }

          if (!cloud_viewer_->updatePointCloud (cloud, "OpenNICloud"))
          {
            cloud_viewer_->addPointCloud (cloud, "OpenNICloud");
            cloud_viewer_->resetCameraViewpoint ("OpenNICloud");
          }          
        }

        // See if we can get an image
        if (image_mutex_.try_lock ())
        {
          image_.swap (image);
          image_mutex_.unlock ();
        }

        if (image)
        {
          if (!image_init && cloud && cloud->width != 0)
          {
            image_viewer_->setPosition (cloud->width, 0);
            image_viewer_->setSize (cloud->width, cloud->height);
            image_init = !image_init;
          }

          if (image->getEncoding() == openni_wrapper::Image::RGB)
            image_viewer_->addRGBImage (image->getMetaData ().Data (), image->getWidth (), image->getHeight ());
          else
            image_viewer_->addRGBImage (rgb_data_, image->getWidth (), image->getHeight ());
          image_viewer_->spinOnce ();
        }
        
      }

      grabber_.stop ();
      
      cloud_connection.disconnect ();
      image_connection.disconnect ();
      if (rgb_data_)
        delete[] rgb_data_;
    }
    
    boost::shared_ptr<pcl::visualization::PCLVisualizer> cloud_viewer_;
    boost::shared_ptr<pcl::visualization::ImageViewer> image_viewer_;
    
    pcl::Grabber& grabber_;
    boost::mutex cloud_mutex_;
    boost::mutex image_mutex_;
    
    CloudConstPtr cloud_;
    boost::shared_ptr<openni_wrapper::Image> image_;
    unsigned char* rgb_data_;
    unsigned rgb_data_size_;
};

// Create the PCLVisualizer object
boost::shared_ptr<pcl::visualization::PCLVisualizer> cld;
boost::shared_ptr<pcl::visualization::ImageViewer> img;

/* ---[ */
int
main (int argc, char** argv)
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
      printHelp(argc, argv);
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
              << ", connected: " << driver.getBus(deviceIdx) << " @ " << driver.getAddress(deviceIdx) << ", serial number: \'" << driver.getSerialNumber(deviceIdx) << "\'" << endl;
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
    depth_mode = pcl::OpenNIGrabber::Mode (mode);

  if (pcl::console::parse(argc, argv, "-imagemode", mode) != -1)
    image_mode = pcl::OpenNIGrabber::Mode (mode);
  
  if (pcl::console::find_argument (argc, argv, "-xyz") != -1)
    xyz = true;
  
  pcl::OpenNIGrabber grabber (device_id, depth_mode, image_mode);
  
  if (xyz || !grabber.providesCallback<pcl::OpenNIGrabber::sig_cb_openni_point_cloud_rgb> ())
  {
    OpenNIViewer<pcl::PointXYZ> openni_viewer (grabber);
    openni_viewer.run ();
  }
  else
  {
    OpenNIViewer<pcl::PointXYZRGBA> openni_viewer (grabber);
    openni_viewer.run ();
  }
  
  return (0);
}
/* ]--- */
