/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2015-, Open Perception, Inc.
 *  Copyright (c) 2015, The MITRE Corporation
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
#include <pcl/io/vlp_grabber.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/console/parse.h>

#include <boost/algorithm/string.hpp>

#include <mutex>
#include <string>

using namespace std::chrono_literals;
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
template <typename PointType>
class SimpleVLPViewer
{
  public:
    using Cloud = PointCloud<PointType>;
    using CloudConstPtr = typename Cloud::ConstPtr;
    using CloudPtr = typename Cloud::Ptr;

    SimpleVLPViewer (Grabber& grabber,
                     PointCloudColorHandler<PointType> *handler) :
        cloud_viewer_ (new PCLVisualizer ("PCL VLP Cloud")),
        grabber_ (grabber),
        handler_ (handler)
    {
    }

    void
    cloud_callback (const CloudConstPtr& cloud)
    {
      FPS_CALC("cloud callback");
      std::lock_guard<std::mutex> lock (cloud_mutex_);
      cloud_ = cloud;
    }

    void
    keyboard_callback (const KeyboardEvent& event,
                       void* /*cookie*/)
    {
      if (event.keyUp ())
      {
        switch (event.getKeyCode ())
        {
          case '0':
            delete handler_;
            handler_ = new PointCloudColorHandlerCustom<PointXYZI> (255, 255, 255);
            break;
          case '1':
            delete handler_;
            handler_ = new PointCloudColorHandlerGenericField<PointXYZI> ("x");
            break;
          case '2':
            delete handler_;
            handler_ = new PointCloudColorHandlerGenericField<PointXYZI> ("y");
            break;
          case '3':
            delete handler_;
            handler_ = new PointCloudColorHandlerGenericField<PointXYZI> ("z");
            break;
          case '4':
            delete handler_;
            handler_ = new PointCloudColorHandlerGenericField<PointXYZI> ("intensity");
            break;
          case 'a':
            cloud_viewer_->removeAllCoordinateSystems ();
            cloud_viewer_->addCoordinateSystem (1.0, "global");
            break;
          case 'A':
            cloud_viewer_->removeAllCoordinateSystems ();
            break;
        }
      }
    }

    void
    run ()
    {
      cloud_viewer_->addCoordinateSystem (1.0, "global");
      cloud_viewer_->setBackgroundColor (0, 0, 0);
      cloud_viewer_->initCameraParameters ();
      cloud_viewer_->setCameraPosition (0.0, 0.0, 30.0, 0.0, 1.0, 0.0, 0);
      cloud_viewer_->setCameraClipDistances (0.0, 50.0);
      cloud_viewer_->registerKeyboardCallback (&SimpleVLPViewer::keyboard_callback, *this);

      std::function<void (const CloudConstPtr&)> cloud_cb = [this] (const CloudConstPtr& cloud){ cloud_callback (cloud); };
      boost::signals2::connection cloud_connection = grabber_.registerCallback (cloud_cb);

      grabber_.start ();

      while (!cloud_viewer_->wasStopped ())
      {
        CloudConstPtr tmp, cloud;

        if (cloud_mutex_.try_lock ())
        {
          cloud_.swap (cloud);
          cloud_mutex_.unlock ();
        }

        if (cloud)
        {
          FPS_CALC("drawing cloud");
          handler_->setInputCloud (cloud);
          if (!cloud_viewer_->updatePointCloud (cloud, *handler_, "VLP"))
            cloud_viewer_->addPointCloud (cloud, *handler_, "VLP");

          cloud_viewer_->spinOnce ();
        }

        if (!grabber_.isRunning ())
          cloud_viewer_->spin ();

        std::this_thread::sleep_for(100us);
      }

      grabber_.stop ();

      cloud_connection.disconnect ();
    }

    PCLVisualizer::Ptr cloud_viewer_;
    ImageViewer::Ptr image_viewer_;

    Grabber& grabber_;
    std::mutex cloud_mutex_;
    std::mutex image_mutex_;

    CloudConstPtr cloud_;
    PointCloudColorHandler<PointType> *handler_;
};

void
usage (char ** argv)
{
  std::cout << "usage: " << argv[0] << " [-pcapFile <path-to-pcap-file>] [-h | --help]" << std::endl;
  std::cout << argv[0] << " -h | --help : shows this help" << std::endl;
  return;
}

int
main (int argc,
      char ** argv)
{
  std::string pcapFile;

  if (find_switch (argc, argv, "-h") || find_switch (argc, argv, "--help"))
  {
    usage (argv);
    return (0);
  }

  parse_argument (argc, argv, "-pcapFile", pcapFile);

  VLPGrabber grabber (pcapFile);

  auto *color_handler = new PointCloudColorHandlerGenericField<PointXYZI> ("intensity");

  SimpleVLPViewer<PointXYZI> v (grabber, color_handler);
  v.run ();

  return (0);
}

