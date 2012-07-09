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

#include <pcl/io/boost.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/openni_camera/openni_driver.h>
#include <pcl/console/parse.h>
#include <pcl/common/time.h>
#include <pcl/console/time.h>

using namespace pcl;
using namespace std;

#define FPS_CALC(_WHAT_) \
do \
{ \
    static unsigned count = 0;\
    static double last = pcl::getTime ();\
    if (++count == 100) \
    { \
      double now = pcl::getTime (); \
      std::cout << "Average framerate("<< _WHAT_ << "): " << double(count)/double(now - last) << " Hz" <<  std::endl; \
      count = 0; \
      last = now; \
    } \
}while(false)

template <typename PointType>
class OpenNIIO
{
  public:
    typedef pcl::PointCloud<PointType> Cloud;
    typedef typename Cloud::Ptr CloudPtr;
    typedef typename Cloud::ConstPtr CloudConstPtr;

    OpenNIIO (const std::string& device_id = "")
      : device_id_ (device_id), mtx_ (), cloud_ (), interface_ (), writer_ ()
    {
    }

    virtual ~OpenNIIO ()
    {
      if (interface_)
        interface_->stop ();
    }
    
    void 
    cloud_cb (const CloudConstPtr& cloud)
    {
      boost::mutex::scoped_lock lock (mtx_);
      FPS_CALC ("callback");

      cloud_ = cloud;
    }

    void
    init ()
    {
      interface_ = new pcl::OpenNIGrabber (device_id_);

      boost::function<void (const CloudConstPtr&)> f = boost::bind (&OpenNIIO::cloud_cb, this, _1);
      boost::signals2::connection c = interface_->registerCallback (f);
     
      interface_->start ();
      
      writer_.setMapSynchronization (false);    // Setting this to true will enable msync() => drop I/O performance
    }

    void
    run ()
    {
      while (true)
      {
        if (cloud_)
        {
          //boost::mutex::scoped_lock lock (mtx_);
          FPS_CALC ("write");

          CloudConstPtr temp_cloud;
          temp_cloud.swap (cloud_);
          writer_.writeBinaryCompressed ("test_binary.pcd", *temp_cloud);
        }
        boost::this_thread::sleep (boost::posix_time::milliseconds (1));
      }
    }

    void
    runEigen ()
    {
      while (true)
      {
        if (cloud_)
        {
          //boost::mutex::scoped_lock lock (mtx_);
          FPS_CALC ("write");

          CloudConstPtr temp_cloud;
          temp_cloud.swap (cloud_);
          writer_.writeBinaryCompressedEigen ("test_binary.pcd", *temp_cloud);
        }
        boost::this_thread::sleep (boost::posix_time::milliseconds (1));
      }
    }

    std::string device_id_;
    boost::mutex mtx_;
    CloudConstPtr cloud_;
    pcl::Grabber* interface_;
    PCDWriter writer_;
};

void
usage (char ** argv)
{
  std::cout << "usage: " << argv[0] << " <device_id> <options>\n\n";

  openni_wrapper::OpenNIDriver& driver = openni_wrapper::OpenNIDriver::getInstance ();
  if (driver.getNumberDevices () > 0)
  {
    for (unsigned deviceIdx = 0; deviceIdx < driver.getNumberDevices (); ++deviceIdx)
    {
      cout << "Device: " << deviceIdx + 1 << ", vendor: " << driver.getVendorName (deviceIdx) << ", product: " << driver.getProductName (deviceIdx)
              << ", connected: " << static_cast<int> (driver.getBus (deviceIdx)) << " @ " << static_cast<int> (driver.getAddress (deviceIdx)) << ", serial number: \'" << driver.getSerialNumber (deviceIdx) << "\'" << endl;
      cout << "device_id may be #1, #2, ... for the first second etc device in the list or" << endl
           << "                 bus@address for the device connected to a specific usb-bus / address combination (works only in Linux) or" << endl
           << "                 <serial-number> (only in Linux and for devices which provide serial numbers)"  << endl;
    }
  }
  else
    cout << "No devices connected." << endl;
}

int
main (int argc, char ** argv)
{
  std::string arg;
  if (argc > 1)
    arg = std::string (argv[1]);
  
  if (arg == "--help" || arg == "-h")
  {
    usage (argv);
    return 1;
  }

  pcl::OpenNIGrabber grabber ("");
  if (grabber.providesCallback<pcl::OpenNIGrabber::sig_cb_openni_point_cloud_eigen> ())
  {
    PCL_INFO ("Eigen mode enabled.\n");
    OpenNIIO<Eigen::MatrixXf> v ("");
    v.init ();
    v.runEigen ();
  }
  else if (grabber.providesCallback<pcl::OpenNIGrabber::sig_cb_openni_point_cloud_rgba> ())
  {
    PCL_INFO ("PointXYZRGBA mode enabled.\n");
    OpenNIIO<pcl::PointXYZRGBA> v ("");
    v.init ();
    v.run ();
  }
  else
  {
    PCL_INFO ("PointXYZ mode enabled.\n");
    OpenNIIO<pcl::PointXYZ> v ("");
    v.init ();
    v.run ();
  }
  return (0);
}
