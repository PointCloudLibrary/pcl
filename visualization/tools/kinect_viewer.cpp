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
#include "openni_camera/openni_image.h"
#include "openni_camera/openni_depth_image.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/shared_ptr.hpp>
#include <pcl_visualization/cloud_viewer.h>

pcl_visualization::CloudViewer viewer ("blablabla");

void testimage (boost::shared_ptr<openni_wrapper::Image> image)
{
    std::cout << __PRETTY_FUNCTION__ << " " << image->getWidth () << std::endl;
}

void testdepthimage (boost::shared_ptr<openni_wrapper::DepthImage> depth_image)
{
    std::cout << __PRETTY_FUNCTION__ << " " << depth_image->getWidth () << std::endl;
}

void testpointcloud (boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > cloud)
{
    std::cout << __PRETTY_FUNCTION__ << " " << cloud->width << std::endl;
}

void testpointcloudrgb (boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > cloud)
{
    std::cout << __PRETTY_FUNCTION__ << " " << cloud->width << std::endl;
    viewer.showCloud (*cloud);
}

class bla
{
  public:
    //bla () : viewer ("KinectGrabber") {}

    void blatestpointcloudrgb (boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > cloud)
    {
        std::cout << __PRETTY_FUNCTION__ << " " << cloud->width << std::endl;
    //    viewer.showCloud (cloud);
    }

    void run ()
    {
      pcl::Grabber* interface = new pcl::OpenNIGrabber();

      //boost::signals2::connection c = interface->registerCallback (boost::bind (&bla::blatestpointcloudrgb, *this, _1));
      //boost::function<void (boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> >)> f = boost::bind (&bla::blatestpointcloudrgb, this, _1);
      //boost::signals2::connection c =
      //  interface->registerCallback <void(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> >)> (boost::bind (&bla::blatestpointcloudrgb, *this, _1).);

      boost::signals2::connection c1 = interface->registerCallback (testpointcloudrgb);
      
      interface->start ();

      //if (c1.connected ())
      //  c1.disconnect ();
      sleep (10);
      
      interface->stop ();
    }
    
    //pcl_visualization::CloudViewer viewer;
};


int main ()
{
  bla b;
  b.run ();
  return 0;
}


