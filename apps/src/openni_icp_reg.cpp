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

#include <boost/thread/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h> //fps calculations
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/openni_camera/openni_driver.h>
#include <vector>
#include <pcl/registration/incremental_registration.h>

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

template <typename PointType>
class SimpleOpenNIViewer
{
public:
  typedef pcl::PointCloud<PointType> Cloud;
  typedef typename Cloud::ConstPtr CloudConstPtr;

  SimpleOpenNIViewer(pcl::OpenNIGrabber& grabber)
    : viewer("PCL OpenNI Viewer")
    , grabber_(grabber)
  {
  }

  /**
   * @brief Callback method for the grabber interface
   * @param cloud The new point cloud from Grabber
   */
  void
  cloud_cb_ (const CloudConstPtr& cloud)
  {
    FPS_CALC ("callback");
    boost::mutex::scoped_lock lock (mtx_);

  reg_.setInputCloud(cloud);
  reg_.setDownsamplingLeafSizeInput(0.03);
  reg_.setDownsamplingLeafSizeModel(0.03);
  reg_.setRegistrationDistanceThreshold(0.5);

  bool use_vanilla_icp = false;
  typename Cloud::Ptr cloud_registered (new Cloud);
  reg_.align(*cloud_registered, use_vanilla_icp);
  typename Cloud::Ptr model (new Cloud);
  pcl::PointCloud<PointType> *c = reg_.getModel();
  pcl::copyPointCloud(*c, *model);

    cloud_ = model;
  }

  /**
   * @brief swaps the pointer to the point cloud with Null pointer and returns the cloud pointer
   * @return boost shared pointer to point cloud
   */
  CloudConstPtr
  getLatestCloud ()
  {
    //lock while we swap our cloud and reset it.
    boost::mutex::scoped_lock lock(mtx_);
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
  run()
  {
    //pcl::Grabber* interface = new pcl::OpenNIGrabber(device_id_, pcl::OpenNIGrabber::OpenNI_QQVGA_30Hz, pcl::OpenNIGrabber::OpenNI_VGA_30Hz);

    boost::function<void (const CloudConstPtr&) > f = boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);

    boost::signals2::connection c = grabber_.registerCallback (f);

    grabber_.start();

    while (!viewer.wasStopped ())
    {
      if (cloud_)
      {
        FPS_CALC ("drawing");
        //the call to get() sets the cloud_ to null;
        viewer.showCloud (getLatestCloud ());
      }
    }

    grabber_.stop();
  }

  pcl::visualization::CloudViewer viewer;
  pcl::OpenNIGrabber& grabber_;
  boost::mutex mtx_;
  CloudConstPtr cloud_;
  pcl::registration::IncrementalRegistration<PointType> reg_;
};

void
usage(char ** argv)
{
  cout << "usage: " << argv[0] << " [<device_id> [<depth-mode> [<image-mode>] ] ] | [path-to-oni-file]\n";
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
  cout << "    uses the device on address 16 at usb bus 1." << endl;
  #endif
  return;
}

int
main(int argc, char ** argv)
{
  std::string arg("");
  pcl::OpenNIGrabber::Mode depth_mode = pcl::OpenNIGrabber::OpenNI_Default_Mode;
  pcl::OpenNIGrabber::Mode image_mode = pcl::OpenNIGrabber::OpenNI_Default_Mode;

  if (argc >= 2)
  {
    arg = argv[1];

    if (arg == "--help" || arg == "-h")
    {
      usage(argv);
      return 1;
    }
    else if (arg == "-l")
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

        cout << endl << "Supported image modes for device: " << device->getVendorName() << " , " << device->getProductName() << endl;
        modes = grabber.getAvailableImageModes();
        for (std::vector<std::pair<int, XnMapOutputMode > >::const_iterator it = modes.begin(); it != modes.end(); ++it)
        {
          cout << it->first << " = " << it->second.nXRes << " x " << it->second.nYRes << " @ " << it->second.nFPS << endl;
        }
        return 0;
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
        return 0;
      }
    }

    if (argc >= 3)
    {
      depth_mode = pcl::OpenNIGrabber::Mode (atoi(argv[2]));
      if (argc == 4)
      {
        image_mode = pcl::OpenNIGrabber::Mode (atoi(argv[3]));
      }
    }
  }
  else
  {
    openni_wrapper::OpenNIDriver& driver = openni_wrapper::OpenNIDriver::getInstance();
    if (driver.getNumberDevices() > 0)
      cout << "Device Id not set, using first device." << endl;
  }

  pcl::OpenNIGrabber grabber(arg, depth_mode, image_mode);
  if (grabber.providesCallback<pcl::OpenNIGrabber::sig_cb_openni_point_cloud_rgba> ())
  {
    SimpleOpenNIViewer<pcl::PointXYZRGBA> v(grabber);
    v.run();
  }
  else
  {
    SimpleOpenNIViewer<pcl::PointXYZI> v(grabber);
    v.run();
  }

  return (0);
}
