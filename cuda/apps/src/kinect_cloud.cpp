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

#include <ros/ros.h>
#include <pcl/PCLPointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl_cuda/io/disparity_to_cloud.h>
#include <pcl_cuda/sample_consensus/sac_model_plane.h>
#include <pcl_cuda/sample_consensus/ransac.h>

#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <functional>

using namespace message_filters;
using namespace pcl;
using namespace pcl_cuda;

DisparityToCloud d2c;
ros::Publisher pub;

struct EventHelper
{
  void
  callback (const pcl::PCLImage::ConstPtr &depth,
            const pcl::PCLImage::ConstPtr &rgb,
            const pcl::CameraInfo::ConstPtr &info)
  {
    //using Indices = pcl_cuda::SampleConsensusModel<pcl_cuda::Host>::Indices;

    //pcl_cuda::PointCloudAOS<pcl_cuda::Host>::Ptr data (new pcl_cuda::PointCloudAOS<pcl_cuda::Host>);
    PointCloudAOS<Device>::Ptr data;
    d2c.callback (depth, rgb, info, data);

    SampleConsensusModelPlane<Device>::Ptr sac_model (new SampleConsensusModelPlane<Device> (data));
//    SampleConsensusModelPlane<Host>::Ptr sac_model (new pcl_cuda::SampleConsensusModelPlane<pcl_cuda::Host> (data));
    RandomSampleConsensus<Device> sac (sac_model);
    sac.setDistanceThreshold (0.05);

    if (!sac.computeModel (2))
      std::cerr << "Failed to compute model" << std::endl;

    // Convert data from Thrust/HOST to PCL
    pcl::PointCloud<pcl::PointXYZRGB> output;
//    pcl_cuda::toPCL (*data, output);

    output.header.stamp = ros::Time::now ();
    pub.publish (output);
  }
};

int
main (int argc, char **argv)
{
  ros::init (argc, argv, "disparity_to_cloud");
  ros::NodeHandle nh;

  // Prepare output
  pub = nh.advertise<PCLPointCloud2>("output", 1);

  // Subscribe to topics
  Synchronizer<sync_policies::ApproximateTime<PCLImage, PCLImage, CameraInfo> > sync_rgb (30);
  Synchronizer<sync_policies::ApproximateTime<PCLImage, CameraInfo> > sync (30);
  Subscriber<PCLImage> sub_depth, sub_rgb;
  Subscriber<CameraInfo> sub_info;
  sub_depth.subscribe (nh, "/camera/depth/image", 30);
  sub_rgb.subscribe (nh, "/camera/rgb/image_color", 30);
  sub_info.subscribe (nh, "/camera/rgb/camera_info", 120);
  //sub_info.subscribe (nh, "/camera/depth/camera_info", 120);

  EventHelper h;

  if (argc > 1 && atoi (argv[1]) == 1)
  {
    ROS_INFO ("[disparity_to_cloud] Using RGB to color the points.");
    sync_rgb.connectInput (sub_depth, sub_rgb, sub_info);
    //sync_rgb.registerCallback (bind (&pcl_cuda::DisparityToCloud::callback, k, _1, _2, _3));
    sync_rgb.registerCallback (std::bind (&EventHelper::callback, &h, _1, _2, _3));
  }
  else
  {
    sync.connectInput (sub_depth, sub_info);
    //sync.registerCallback (bind (&pcl_cuda::DisparityToCloud::callback, k, _1, PCLImageConstPtr (), _2));
    sync.registerCallback (std::bind (&EventHelper::callback, &h, _1, PCLImageConstPtr (), _2));
  }

  // Do this indefinitely
  ros::spin ();

  return (0);
}
