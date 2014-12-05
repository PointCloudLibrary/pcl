/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012-, Open Perception, Inc.
 *
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
 *   * Neither the name of the copyright holder(s) nor the names of its
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
 *  This is preliminary software and/or hardware and APIs are preliminary and subject to change.
 */
#include "Kinect2Grabber.hpp"

int
main (int argc,
      char *argv[])
{
  //The first constructor calibrates the cameras using images inside the specified folders.
  //The parameters are: rgb image folder, depth image folder, number of images, size of the checkboard, size of the check board squares in m
  //The second constructor load an existing calibration

  //Kinect2::Kinect2Grabber<pcl::PointXYZRGB> k2g("./images/rgb/", "./images/ir/", 16, cv::Size(6,9), 0.025 );
  pcl::Kinect2Grabber::Kinect2Grabber < pcl::PointXYZRGB > k2g ("./rgb_calibration.yaml", "./depth_calibration.yaml", "./pose_calibration.yaml");

  boost::shared_ptr < pcl::PointCloud < pcl::PointXYZRGB >> cloud;

  cloud = k2g.getCloud ();

  cloud->sensor_orientation_.w () = 0.0;
  cloud->sensor_orientation_.x () = 1.0;
  cloud->sensor_orientation_.y () = 0.0;
  cloud->sensor_orientation_.z () = 0.0;

  boost::shared_ptr < pcl::visualization::PCLVisualizer > viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField < pcl::PointXYZRGB > rgb (cloud);
  viewer->addPointCloud < pcl::PointXYZRGB > (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");

  while (!viewer->wasStopped ())
  {
    viewer->spinOnce ();
    cloud = k2g.getCloud ();
    pcl::visualization::PointCloudColorHandlerRGBField < pcl::PointXYZRGB > rgb (cloud);
    viewer->updatePointCloud < pcl::PointXYZRGB > (cloud, rgb, "sample cloud");
  }

  k2g.shutDown ();
  return 0;
}

