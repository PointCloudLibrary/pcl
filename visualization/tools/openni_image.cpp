/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2011, Willow Garage, Inc.
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
 */

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h> //fps calculations
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/boost.h>
#include <pcl/visualization/common/float_image_utils.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/io/openni_camera/openni_driver.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/mouse_event.h>

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
class SimpleOpenNIViewer
{
  public:
    SimpleOpenNIViewer (pcl::OpenNIGrabber& grabber)
      : grabber_ (grabber),
        image_viewer_ ("PCL/OpenNI RGB image viewer"),
        depth_image_viewer_ ("PCL/OpenNI depth image viewer"),
        image_cld_init_ (false), depth_image_cld_init_ (false),
        rgb_data_ (0), depth_data_ (0), rgb_data_size_ (0)
    {
    }

    void
    image_callback (const boost::shared_ptr<openni_wrapper::Image> &image, 
                    const boost::shared_ptr<openni_wrapper::DepthImage> &depth_image, float)
    {
      FPS_CALC ("image callback");
      boost::mutex::scoped_lock lock (image_mutex_);

      // Copy data
      image_ = image;
      if (image->getEncoding() != openni_wrapper::Image::RGB)
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

      // Copy data
      depth_image_ = depth_image;
      if (depth_data_)
        delete[] depth_data_;
      depth_data_ = pcl::visualization::FloatImageUtils::getVisualImage (
          reinterpret_cast<const unsigned short*> (depth_image->getDepthMetaData ().Data ()),
            depth_image->getWidth (), depth_image->getHeight (),
            std::numeric_limits<unsigned short>::min (), 
            // Scale so that the colors look brigher on screen
            std::numeric_limits<unsigned short>::max () / 10, 
            true);
    }
    
    void 
    keyboard_callback (const pcl::visualization::KeyboardEvent& event, void* cookie)
    {
      string* message = static_cast<string*> (cookie);
      cout << (*message) << " :: ";
      if (event.getKeyCode ())
        cout << "the key \'" << event.getKeyCode () << "\' (" << event.getKeyCode () << ") was";
      else
        cout << "the special key \'" << event.getKeySym () << "\' was";
      if (event.keyDown ())
        cout << " pressed" << endl;
      else
        cout << " released" << endl;
    }
    
    void 
    mouse_callback (const pcl::visualization::MouseEvent& mouse_event, void* cookie)
    {
      string* message = static_cast<string*> (cookie);
      if (mouse_event.getType () == pcl::visualization::MouseEvent::MouseButtonPress && mouse_event.getButton () == pcl::visualization::MouseEvent::LeftButton)
      {
        cout << (*message) << " :: " << mouse_event.getX () << " , " << mouse_event.getY () << endl;
      }
    }

    void
    run ()
    {
      string mouseMsg2D ("Mouse coordinates in image viewer");
      string keyMsg2D ("Key event for image viewer");

      image_viewer_.registerMouseCallback (&SimpleOpenNIViewer::mouse_callback, *this, static_cast<void*> (&mouseMsg2D));
      image_viewer_.registerKeyboardCallback(&SimpleOpenNIViewer::keyboard_callback, *this, static_cast<void*> (&keyMsg2D));
      depth_image_viewer_.registerMouseCallback (&SimpleOpenNIViewer::mouse_callback, *this, static_cast<void*> (&mouseMsg2D));
      depth_image_viewer_.registerKeyboardCallback(&SimpleOpenNIViewer::keyboard_callback, *this, static_cast<void*> (&keyMsg2D));
        
      boost::function<void (const boost::shared_ptr<openni_wrapper::Image>&, const boost::shared_ptr<openni_wrapper::DepthImage>&, float) > image_cb = boost::bind (&SimpleOpenNIViewer::image_callback, this, _1, _2, _3);
      boost::signals2::connection image_connection = grabber_.registerCallback (image_cb);
      
      grabber_.start ();
           
      while (!image_viewer_.wasStopped () && !depth_image_viewer_.wasStopped ())
      {
        boost::shared_ptr<openni_wrapper::Image> image;
        boost::shared_ptr<openni_wrapper::DepthImage> depth_image;
        if (!image_cld_init_)
        {
          image_viewer_.setPosition (0, 0);
          image_cld_init_ = !image_cld_init_;
        }

        if (image_mutex_.try_lock ())
        {
          // Swap data
          image_.swap (image);
          depth_image_.swap (depth_image);

          // Unlock
          image_mutex_.unlock ();
        }

        // Add to renderer
        if (image)
        {
          if (image->getEncoding() == openni_wrapper::Image::RGB)
            image_viewer_.addRGBImage (image->getMetaData ().Data (), image->getWidth (), image->getHeight ());
          else
            image_viewer_.addRGBImage (rgb_data_, image->getWidth (), image->getHeight ());
        }

        if (depth_image)
        {
          depth_image_viewer_.addRGBImage (depth_data_, depth_image->getWidth (), depth_image->getHeight ());
          if (!depth_image_cld_init_)
          {
            depth_image_viewer_.setPosition (depth_image->getWidth (), 0);
            depth_image_cld_init_ = !depth_image_cld_init_;
          }
        }

        image_viewer_.spinOnce ();
        depth_image_viewer_.spinOnce ();
        
        boost::this_thread::sleep (boost::posix_time::microseconds (100));
      }

      grabber_.stop ();
      
      image_connection.disconnect ();
      
      if (rgb_data_)
        delete[] rgb_data_;
      if (depth_data_)
        delete[] depth_data_;
    }

    pcl::OpenNIGrabber& grabber_;
    boost::mutex image_mutex_;
    boost::shared_ptr<openni_wrapper::Image> image_;
    boost::shared_ptr<openni_wrapper::DepthImage> depth_image_;
    pcl::visualization::ImageViewer image_viewer_;
    pcl::visualization::ImageViewer depth_image_viewer_;
    bool image_cld_init_, depth_image_cld_init_;
    unsigned char* rgb_data_, *depth_data_;
    unsigned rgb_data_size_;
};

void
usage (char ** argv)
{
  cout << "usage: " << argv[0] << " [((<device_id> | <path-to-oni-file>) [-imagemode <mode>] | [-depthformat <format>] | -l [<device_id>]| -h | --help)]" << endl;
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
}

int
main (int argc, char ** argv)
{
  std::string device_id ("");
  pcl::OpenNIGrabber::Mode image_mode = pcl::OpenNIGrabber::OpenNI_Default_Mode;
  
  if (argc >= 2)
  {
    device_id = argv[1];
    if (device_id == "--help" || device_id == "-h")
    {
      usage (argv);
      return (0);
    }
    else if (device_id == "-l")
    {
      if (argc >= 3)
      {
        pcl::OpenNIGrabber grabber (argv[2]);
        boost::shared_ptr<openni_wrapper::OpenNIDevice> device = grabber.getDevice ();
        std::vector<std::pair<int, XnMapOutputMode> > modes;

        if (device->hasImageStream ())
        {
          cout << endl << "Supported image modes for device: " << device->getVendorName () << " , " << device->getProductName () << endl;
          modes = grabber.getAvailableImageModes ();
          for (std::vector<std::pair<int, XnMapOutputMode> >::const_iterator it = modes.begin (); it != modes.end (); ++it)
          {
            cout << it->first << " = " << it->second.nXRes << " x " << it->second.nYRes << " @ " << it->second.nFPS << endl;
          }
        }
      }
      else
      {
        openni_wrapper::OpenNIDriver& driver = openni_wrapper::OpenNIDriver::getInstance ();
        if (driver.getNumberDevices () > 0)
        {
          for (unsigned deviceIdx = 0; deviceIdx < driver.getNumberDevices (); ++deviceIdx)
          {
            cout << "Device: " << deviceIdx + 1 << ", vendor: " << driver.getVendorName (deviceIdx) << ", product: " << driver.getProductName (deviceIdx)
              << ", connected: " << driver.getBus (deviceIdx) << " @ " << driver.getAddress (deviceIdx) << ", serial number: \'" << driver.getSerialNumber (deviceIdx) << "\'" << endl;
          }

        }
        else
          cout << "No devices connected." << endl;
        
        cout <<"Virtual Devices available: ONI player" << endl;
      }
      return (0);
    }
  }
  else
  {
    openni_wrapper::OpenNIDriver& driver = openni_wrapper::OpenNIDriver::getInstance ();
    if (driver.getNumberDevices() > 0)
      cout << "Device Id not set, using first device." << endl;
  }
  
  unsigned mode;
  if (pcl::console::parse (argc, argv, "-imagemode", mode) != -1)
    image_mode = static_cast<pcl::OpenNIGrabber::Mode> (mode);
  
  int depthformat = openni_wrapper::OpenNIDevice::OpenNI_12_bit_depth;
  pcl::console::parse_argument (argc, argv, "-depthformat", depthformat);

  pcl::OpenNIGrabber grabber (device_id, pcl::OpenNIGrabber::OpenNI_Default_Mode, image_mode);
  // Set the depth output format
  grabber.getDevice ()->setDepthOutputFormat (static_cast<openni_wrapper::OpenNIDevice::DepthMode> (depthformat));

  SimpleOpenNIViewer v (grabber);
  v.run ();

  return (0);
}
