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

#include <pcl/memory.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/cuda/io/cloud_to_pcl.h>
#include <pcl/cuda/io/disparity_to_cloud.h>
#include <pcl/cuda/time_cpu.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>

#include <functional>
#include <iostream>
#include <mutex>


using pcl::cuda::PointCloudAOS;
using pcl::cuda::Device;

class KinectViewerCuda
{
  public:
     KinectViewerCuda (bool downsample) : viewer ("KinectGrabber"), downsample_(downsample) {}

    void cloud_cb_ (const openni_wrapper::Image::Ptr& image, const openni_wrapper::DepthImage::Ptr& depth_image, float constant)
    {
      PointCloudAOS<Device>::Ptr data;
    	{
        pcl::cuda::ScopeTimeCPU t ("time:");    
        d2c.compute<Device> (depth_image, image, constant, data, downsample_);
      }

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr output (new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::cuda::toPCL (*data, *output);

      viewer.showCloud (output, "cloud");

    }
    
    void run (const std::string& device_id)
    {
      pcl::Grabber* interface = new pcl::OpenNIGrabber(device_id);

      std::function<void (const openni_wrapper::Image::Ptr& image, const openni_wrapper::DepthImage::Ptr& depth_image, float)>
        f = [this](const openni_wrapper::Image::Ptr& image, const openni_wrapper::DepthImage::Ptr& depth_image, float constant) { cloud_cb_(image, depth_image, constant); };

      boost::signals2::connection c = interface->registerCallback (f);

      interface->start ();
      
      while (true)
      {
        pcl_sleep (1);
      }

      interface->stop ();
    }

    pcl::cuda::DisparityToCloud d2c;
    pcl::visualization::CloudViewer viewer;
    std::mutex mutex_;
    bool downsample_;
};

int main (int argc, char** argv)
{
	std::string device_id = "#1";
  int downsample = false;
	if (argc >= 2)
	{
		device_id = argv[1];
	}
	if (argc >= 3)
	{
		downsample = atoi (argv[2]);
	}
  KinectViewerCuda v (downsample);
  v.run (device_id);
  return 0;
}
