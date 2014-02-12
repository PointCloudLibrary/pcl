/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, The MITRE Corporation
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
 * Author: Keven Ring <keven@mitre.org>
 */

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h> //fps calculations
#include <pcl/io/hdl_grabber.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/io/openni_camera/openni_driver.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/boost.h>
#include <pcl/visualization/mouse_event.h>
#include <vector>
#include <string>
#include <boost/algorithm/string.hpp>
#include <typeinfo>

using namespace std;
using namespace pcl;
using namespace pcl::console;
using namespace pcl::visualization;

#define SHOW_FPS 0
#if SHOW_FPS
#define FPS_CALC(_WHAT_) \
do \
{ \
    static unsigned count = 0;\
    static double last = getTime ();\
    double now = getTime (); \
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
template<typename PointType>
class SimpleHDLViewer
{
  public:
    typedef PointCloud<PointType> Cloud;
    typedef typename Cloud::ConstPtr CloudConstPtr;

    SimpleHDLViewer (Grabber& grabber,
                     PointCloudColorHandler<PointType> &handler) 
      : cloud_viewer_ (new PCLVisualizer ("PCL HDL Cloud"))
      , grabber_ (grabber)
      , handler_ (handler)
    {
    }

    void 
    cloud_callback (const CloudConstPtr& cloud)
    {
      FPS_CALC ("cloud callback");
      boost::mutex::scoped_lock lock (cloud_mutex_);
      cloud_ = cloud;
      //std::cout << cloud->points[0] << " " << cloud->size () << std::endl;
    }

    void 
    cloud_callback (const CloudConstPtr& cloud,
                    float /*startAngle*/,
                    float /*endAngle*/)
    {
      FPS_CALC ("cloud callback");
      boost::mutex::scoped_lock lock (cloud_mutex_);
      cloud_ = cloud;
    }

    void 
    keyboard_callback (const KeyboardEvent& event,
                       void* /*cookie*/)
    {
      if (event.keyUp ())
      {
        return;
      }
    }

    void 
    mouse_callback (const MouseEvent& mouse_event,
                    void* /*cookie*/)
    {
      if (mouse_event.getType () == MouseEvent::MouseButtonPress && 
          mouse_event.getButton () == MouseEvent::LeftButton)
      {
        cout << mouse_event.getX () << " , " << mouse_event.getY () << endl;
      }
    }

    void 
    run ()
    {
      cloud_viewer_->addCoordinateSystem (3.0, "global");
      cloud_viewer_->setBackgroundColor (0, 0, 0);
      cloud_viewer_->initCameraParameters ();
      cloud_viewer_->setCameraPosition (0.0, 0.0, 30.0, 0.0, 1.0, 0.0, 0);
      cloud_viewer_->setCameraClipDistances (0.0, 50.0);
      //cloud_viewer_->registerMouseCallback(&SimpleHDLViewer::mouse_callback, *this);
      //cloud_viewer_->registerKeyboardCallback (&SimpleHDLViewer::keyboard_callback, *this);

      //boost::function<void(const CloudConstPtr&, float, float)> cloud_cb = boost::bind(&SimpleHDLViewer::cloud_callback, this, _1, _2, _3);
      boost::function<void (const CloudConstPtr&)> cloud_cb = boost::bind (
          &SimpleHDLViewer::cloud_callback, this, _1);
      boost::signals2::connection cloud_connection = grabber_.registerCallback (
          cloud_cb);

      grabber_.start ();

      while (!cloud_viewer_->wasStopped ())
      {
        CloudConstPtr cloud;

        // See if we can get a cloud
        if (cloud_mutex_.try_lock ())
        {
          cloud_.swap (cloud);
          cloud_mutex_.unlock ();
        }

        if (cloud)
        {
          FPS_CALC("drawing cloud");
          handler_.setInputCloud (cloud);
          if (!cloud_viewer_->updatePointCloud (cloud, handler_, "HDL"))
            cloud_viewer_->addPointCloud (cloud, handler_, "HDL");

          cloud_viewer_->spinOnce ();
        }

        if (!grabber_.isRunning ())
          cloud_viewer_->spin ();

        boost::this_thread::sleep (boost::posix_time::microseconds (100));
      }

      grabber_.stop ();

      cloud_connection.disconnect ();
    }

    boost::shared_ptr<PCLVisualizer> cloud_viewer_;
    boost::shared_ptr<ImageViewer> image_viewer_;

    Grabber& grabber_;
    boost::mutex cloud_mutex_;
    boost::mutex image_mutex_;

    CloudConstPtr cloud_;
    PointCloudColorHandler<PointType> &handler_;
};

void
usage (char ** argv)
{
  cout << "usage: " << argv[0]
      << " [-hdlCalibration <path-to-calibration-file>] [-pcapFile <path-to-pcap-file>] [-h | --help] [-format XYZ(default)|XYZI|XYZRGB]"
      << endl;
  cout << argv[0] << " -h | --help : shows this help" << endl;
  return;
}

int 
main (int argc, char ** argv)
{
  std::string hdlCalibration, pcapFile, format ("XYZ");

  if (find_switch (argc, argv, "-h") || 
      find_switch (argc, argv, "--help"))
  {
    usage (argv);
    return (0);
  }

  parse_argument (argc, argv, "-calibrationFile", hdlCalibration);
  parse_argument (argc, argv, "-pcapFile", pcapFile);
  parse_argument (argc, argv, "-format", format);

  HDLGrabber grabber (hdlCalibration, pcapFile);

  cout << "viewer format:" << format << endl;
  if (boost::iequals (format, std::string ("XYZ")))
  {
    std::vector<double> fcolor (3); fcolor[0] = fcolor[1] = fcolor[2] = 255.0;
    pcl::console::parse_3x_arguments (argc, argv, "-fc", fcolor[0], fcolor[1], fcolor[2]);
    PointCloudColorHandlerCustom<PointXYZ> color_handler (fcolor[0], fcolor[1], fcolor[2]);

    SimpleHDLViewer<PointXYZ> v (grabber, color_handler);
    v.run ();
  }
  else if (boost::iequals (format, std::string ("XYZI")))
  {
    PointCloudColorHandlerGenericField<PointXYZI> color_handler ("intensity");

    SimpleHDLViewer<PointXYZI> v (grabber, color_handler);
    v.run ();
  }
  else if (boost::iequals (format, std::string ("XYZRGB")))
  {
    PointCloudColorHandlerRGBField<PointXYZRGBA> color_handler;

    SimpleHDLViewer<PointXYZRGBA> v (grabber, color_handler);
    v.run ();
  }
  return (0);
}

