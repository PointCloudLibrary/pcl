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
 */

#include <boost/thread/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h> //fps calculations
#include <pcl/io/kinect_grabber.h>
#include <pcl/visualization/cloud_viewer.h>

#define SHOW_FPS 1
#if SHOW_FPS
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
#else
#define FPS_CALC(_WHAT_) \
do \
{ \
}while(false)
#endif

template <typename PointType>
class SimpleKinectViewer
{
  public:
    typedef pcl::PointCloud<PointType> Cloud;
    typedef typename Cloud::ConstPtr CloudConstPtr;


    SimpleKinectViewer () : viewer ("PCL Kinect Viewer") {}

    
    void 
    cloud_cb_ (const CloudConstPtr& cloud)
    {
      FPS_CALC("callback");
      set(cloud);
    }

    void
    set(const CloudConstPtr& cloud)
    {
      //lock while we set our cloud;
      boost::mutex::scoped_lock lock(mtx_);
      cloud_  = cloud;
    }

    CloudConstPtr
    get(){
      //lock while we swap our cloud and reset it.
      boost::mutex::scoped_lock lock(mtx_);
      CloudConstPtr temp_cloud;
      std::swap(temp_cloud, cloud_); //here we set cloud_ to null, so that
                                     //it is safe to set it again from our
                                     //callback
      return temp_cloud;
    }

    void
    run ()
    {
      pcl::Grabber* interface = new pcl::OpenNIGrabber();

      boost::function<void (const CloudConstPtr&)> f =
        boost::bind (&SimpleKinectViewer::cloud_cb_, this, _1);

      boost::signals2::connection c = interface->registerCallback (f);
      
      interface->start ();
      
      while (!viewer.wasStopped ())
      {
        if(cloud_)
        {
          FPS_CALC("drawing");
          //the call to get() sets the cloud_ to null;
          viewer.showCloud ( get() );
        }
      }

      interface->stop ();
    }

    pcl_visualization::CloudViewer viewer;
    boost::mutex mtx_;
    CloudConstPtr cloud_;
};

void
usage(char ** argv)
{
    std::cout << "usage: " << argv[0] << " [XYZ|XYZRGB]\n";
}

int 
main (int argc, char ** argv)
{
  if(argc != 2)
  {
    usage(argv);
    return 1;
  }

  std::string arg(argv[1]);
  if(arg == "XYZRGB")
  {
      SimpleKinectViewer<pcl::PointXYZRGB> v;
      v.run ();
  }else if(arg == "XYZ")
  {
      SimpleKinectViewer<pcl::PointXYZ> v;
      v.run ();
  }
  else if( arg == "--help" || arg == "-h")
  {
    usage(argv);
    return 1;
  }

  return (0);
}
