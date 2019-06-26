/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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
 * $Id$
 *
 */

#include <pcl_cuda/io/cloud_to_pcl.h>
#include <pcl_cuda/io/disparity_to_cloud.h>
#include <pcl_cuda/sample_consensus/sac_model_plane.h>
#include <pcl_cuda/sample_consensus/ransac.h>
#include <pcl_cuda/time_cpu.h>

#include <pcl/io/openni_grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#include <boost/shared_ptr.hpp>

#include <functional>
#include <iostream>
#include <mutex>

class SimpleKinectTool
{
  public:
     SimpleKinectTool () : viewer ("KinectGrabber"), init_(false) {}

    void cloud_cb_ (const boost::shared_ptr<openni_wrapper::Image>& image, const boost::shared_ptr<openni_wrapper::DepthImage>& depth_image, float constant)
    {
	    pcl_cuda::PointCloudAOS<pcl_cuda::Device>::Ptr data;
    	{
    	pcl::ScopeTime t ("time:");    
      d2c.compute<pcl_cuda::Device> (depth_image, image, constant, data);
      }
      //d2c.callback (depth_image, constant, *data);

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr output (new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl_cuda::toPCL (*data, *output);

      viewer.showCloud (output);

    }
    
    void run (const std::string& device_id)
    {
      pcl::Grabber* interface = new pcl::OpenNIGrabber(device_id);

      std::function<void (const boost::shared_ptr<openni_wrapper::Image>& image, const boost::shared_ptr<openni_wrapper::DepthImage>& depth_image, float)> f = boost::bind (&SimpleKinectTool::cloud_cb_, this, _1, _2, _3);

      boost::signals2::connection c = interface->registerCallback (f);

      //viewer.runOnVisualizationThread (fn, "viz_cb");
      interface->start ();
      
      while (true)
      {
        sleep (1);
      }

      interface->stop ();
    }

    pcl_cuda::DisparityToCloud d2c;
    pcl::visualization::CloudViewer viewer;
    std::mutex mutex_;
    bool init_;
};

int main (int argc, char** argv)
{
	std::string device_id = "#1";
	if (argc == 2)
	{
		device_id = argv[1];
	}
  SimpleKinectTool v;
  v.run (device_id);
  return 0;
}
