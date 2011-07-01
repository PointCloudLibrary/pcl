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
 */

#include <boost/thread/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/openni_camera/openni_driver.h>
#include <pcl/filters/passthrough.h>
#include <pcl/console/parse.h>
#include <pcl/common/time.h>

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


template <typename PointType>
class OpenNIPassthrough
{
  public:
    typedef pcl::PointCloud<PointType> Cloud;
    typedef typename Cloud::Ptr CloudPtr;
    typedef typename Cloud::ConstPtr CloudConstPtr;

    OpenNIPassthrough (const std::string& device_id = "", 
                       const std::string& field_name = "z", float min_v = 0, float max_v = 5.0)
    : viewer ("PCL OpenNI PassThrough Viewer")
    , device_id_(device_id)
    {
      pass_.setFilterFieldName (field_name);
      pass_.setFilterLimits (min_v, max_v);
    }

    void 
    cloud_cb_ (const CloudConstPtr& cloud)
    {
      boost::mutex::scoped_lock lock (mtx_);
      FPS_CALC ("computation");

      cloud_pass_.reset (new Cloud);
      // Computation goes here
      pass_.setInputCloud (cloud);
      pass_.filter (*cloud_pass_);
      cloud_  = cloud;
    }

    void
    run ()
    {
      pcl::Grabber* interface = new pcl::OpenNIGrabber (device_id_);

      boost::function<void (const CloudConstPtr&)> f = boost::bind (&OpenNIPassthrough::cloud_cb_, this, _1);
      boost::signals2::connection c = interface->registerCallback (f);
      
      interface->start ();
      
      while (!viewer.wasStopped ())
      {
        if (cloud_pass_)
        {
          boost::mutex::scoped_lock lock (mtx_);

          FPS_CALC ("visualization");
          CloudPtr temp_cloud;
          temp_cloud.swap (cloud_pass_); //here we set cloud_ to null, so that
          viewer.showCloud (temp_cloud);
        }
      }

      interface->stop ();
    }

    pcl::PassThrough<PointType> pass_;
    pcl::visualization::CloudViewer viewer;
    std::string device_id_;
    boost::mutex mtx_;
    CloudConstPtr cloud_;
    CloudPtr cloud_pass_;
};

void
usage (char ** argv)
{
  std::cout << "usage: " << argv[0] << " <device_id> <options>\n\n"
            << "where options are:\n         -minmax min-max  :: set the PassThrough min-max cutting values (default: 0-5.0)\n"
            <<                     "         -field  X        :: use field/dimension 'X' to filter data on (default: 'z')\n";

  openni_wrapper::OpenNIDriver& driver = openni_wrapper::OpenNIDriver::getInstance ();
  if (driver.getNumberDevices () > 0)
  {
    for (unsigned deviceIdx = 0; deviceIdx < driver.getNumberDevices (); ++deviceIdx)
    {
      cout << "Device: " << deviceIdx + 1 << ", vendor: " << driver.getVendorName (deviceIdx) << ", product: " << driver.getProductName (deviceIdx)
              << ", connected: " << (int)driver.getBus (deviceIdx) << " @ " << (int)driver.getAddress (deviceIdx) << ", serial number: \'" << driver.getSerialNumber (deviceIdx) << "\'" << endl;
      cout << "device_id may be #1, #2, ... for the first second etc device in the list or" << endl
           << "                 bus@address for the device connected to a specific usb-bus / address combination (wotks only in Linux) or" << endl
           << "                 <serial-number> (only in Linux and for devices which provide serial numbers)"  << endl;
    }
  }
  else
    cout << "No devices connected." << endl;
}

int 
main (int argc, char ** argv)
{
  if (argc < 2)
  {
    usage (argv);
    return 1;
  }

  std::string arg (argv[1]);
  
  if (arg == "--help" || arg == "-h")
  {
    usage (argv);
    return 1;
  }

  double min_v = 0, max_v = 5.0;
  pcl::console::parse_2x_arguments (argc, argv, "-minmax", min_v, max_v, false);
  std::string field_name ("z");
  pcl::console::parse_argument (argc, argv, "-field", field_name);

  pcl::OpenNIGrabber grabber (arg);
  if (grabber.providesCallback<pcl::OpenNIGrabber::sig_cb_openni_point_cloud_rgb> ())
  {
    OpenNIPassthrough<pcl::PointXYZRGB> v (arg, field_name, min_v, max_v);
    v.run ();
  }
  else
  {
    OpenNIPassthrough<pcl::PointXYZ> v (arg, field_name, min_v, max_v);
    v.run ();
  }

  return (0);
}
