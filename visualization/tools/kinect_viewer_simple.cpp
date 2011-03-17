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

#include <pcl/io/kinect_grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/shared_ptr.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>

pcl_visualization::CloudViewer viewer ("Simple Kinect Viewer");
boost::mutex mutex_;
pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud_;
bool init = false;

struct EventHelper 
{
  void viz_cb (pcl_visualization::PCLVisualizer& viz)
  {
    std::cerr << "viz_cb" << std::endl;
    if (!init)
    {
      std::cerr << "init" << std::endl;
      viz.setBackgroundColor (1,1,1);
      init = true;
    }
    else
      viz.removePointCloud ("KinectCloud");
    boost::mutex::scoped_lock (mutex_);
    if (cloud_)
      viz.addPointCloud (*cloud_, "KinectCloud");
  }
};

void cloud_cb (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & cloud)
{
  //std::cerr << "cloud_cb" << std::endl;
  //boost::mutex::scoped_lock (mutex_);
  //cloud_ = cloud;
  viewer.showCloud(*cloud);
}

class SimpleKinectViewer
{
  public:
    //SimpleKinectViewer () : viewer ("KinectGrabber") {}

    //void cloud_cb_ (boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > cloud)
    //{
    //    viewer.showCloud (*cloud);
    //}
    //
    //pcl_visualization::CloudViewer viewer;

    void run ()
    {
      pcl::Grabber* interface = new pcl::OpenNIGrabber();

      //boost::signals2::connection c = interface->registerCallback (boost::bind (&SimpleKinectViewer::blatestpointcloudrgb, *this, _1));
      //boost::function<void (boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> >)> f = boost::bind (&SimpleKinectViewer::blatestpointcloudrgb, this, _1);
      //boost::signals2::connection c =
      //  interface->registerCallback <void(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> >)> (boost::bind (&SimpleKinectViewer::cloud_cb_, this, _1));

      boost::signals2::connection c1 = interface->registerCallback (cloud_cb);
      
      EventHelper h;
      boost::function1<void, pcl_visualization::PCLVisualizer&> fn = boost::bind (&EventHelper::viz_cb, &h, _1);
      std::cerr << "--- " << (fn.empty()?"empty!":"not empty") << std::endl;

      //viewer.runOnVisualizationThread (fn, "viz_cb");
      interface->start ();
      
      //
      //if (c1.connected ())
      //  c1.disconnect ();
      while (!viewer.wasStopped())
      {
      }

      interface->stop ();
    }
    
};

int main ()
{
  SimpleKinectViewer v;
  v.run ();
  return 0;
}


