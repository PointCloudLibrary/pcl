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
#include <pcl/console/parse.h>
#include <pcl/io/oni_grabber.h>
#include <pcl/visualization/boost.h>
#include <pcl/visualization/cloud_viewer.h>
#include <vector>
#include <pcl/common/time_trigger.h>

#define SHOW_FPS 1
#if SHOW_FPS
#define FPS_CALC(_WHAT_) \
do \
{ \
    static unsigned count = 0;\
    static double last = pcl::getTime ();\
    ++count; \
    if (pcl::getTime() - last >= 1.0) \
    { \
      double now = pcl::getTime (); \
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
class SimpleONIViewer
{
public:
  typedef pcl::PointCloud<PointType> Cloud;
  typedef typename Cloud::ConstPtr CloudConstPtr;

  SimpleONIViewer(pcl::ONIGrabber& grabber)
    : viewer("PCL OpenNI Viewer")
    , grabber_(grabber)
    , mtx_ ()
    , cloud_ ()
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
    cloud_ = cloud;
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

    boost::function<void (const CloudConstPtr&) > f = boost::bind (&SimpleONIViewer::cloud_cb_, this, _1);

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
  pcl::ONIGrabber& grabber_;
  boost::mutex mtx_;
  CloudConstPtr cloud_;
};

void
usage(char ** argv)
{
  cout << "usage: " << argv[0] << " <path-to-oni-file> [framerate]\n";
  cout << argv[0] << " -h | --help : shows this help\n";
  cout << argv[0] << " -xyz        : enable just XYZ data display\n";
  return;
}

int
main(int argc, char ** argv)
{
  std::string arg("");

  unsigned frame_rate = 0;
  if (argc >= 2)
  {
    arg = argv[1];

    if (arg == "--help" || arg == "-h")
    {
      usage(argv);
      return 1;
    }
    
    if (argc >= 3)
    {
      frame_rate = atoi(argv[2]);
    }
  }
  else
  {
    usage (argv);
    return 1;
  }

  pcl::TimeTrigger trigger;
  
  pcl::ONIGrabber* grabber = 0;
  if (frame_rate == 0)
    grabber = new  pcl::ONIGrabber(arg, true, true);
  else
  {
    grabber = new  pcl::ONIGrabber(arg, true, false);
    trigger.setInterval (1.0 / static_cast<double> (frame_rate));
    trigger.registerCallback (boost::bind(&pcl::ONIGrabber::start, grabber));
    trigger.start();
  }
  if (grabber->providesCallback<pcl::ONIGrabber::sig_cb_openni_point_cloud_rgb > () && !pcl::console::find_switch (argc, argv, "-xyz"))
  {
    SimpleONIViewer<pcl::PointXYZRGBA> v(*grabber);
    v.run();
  }
  else
  {
    SimpleONIViewer<pcl::PointXYZ> v(*grabber);
    v.run();
  }

  return (0);
}
