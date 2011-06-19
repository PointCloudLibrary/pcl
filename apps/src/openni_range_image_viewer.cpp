/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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
 * $Id: pointimage_msgto_pcd.cpp 33238 2010-03-11 00:46:58Z rusu $
 *
 */

// ROS core
#include <boost/thread/mutex.hpp>
// PCL includes
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/common/common_headers.h>

#include <pcl_visualization/pcl_visualizer.h>
#include "pcl_visualization/range_image_visualizer.h"

using namespace std;
using namespace pcl;
using namespace pcl_visualization;
typedef PointXYZ PointType;

float angular_resolution = 0.5f,  // Resolution of the range image
      noise_level = 0.0f,         // Distance in which the z-buffer averages instead of taking the minimum
      min_range  = 0.0f;          // Minimum range for the range image creation
int border_size = 0;              // Additional border around the range image
RangeImage::CoordinateFrame coordinate_frame = RangeImage::CAMERA_FRAME;
int source = 0;  // Default: Use PointCloud2

void 
printUsage (const char* progName)
{
  cout << "\n\nUsage: "<<progName<<" [options] input:=<yourInput>\n\n"
       << "Options:\n"
       << "-------------------------------------------\n"
       << "-c <int>     source coordinate frame (default "<<coordinate_frame<<")\n"
       << "-s           source used to create the range image: "
       <<              "0 PointCloud2 (default), 1 depth image, 2 disparity image .\n"
       << "-r <float>   angular resolution in degrees (default "<<angular_resolution<<")\n"
       << "-h           this help\n"
       << "\n\n";
}

sensor_msgs::PointCloud2ConstPtr cloud_msg, old_cloud_msg;
sensor_msgs::ImageConstPtr depth_image_msg, old_depth_image_msg;
stereo_msgs::DisparityImageConstPtr disparity_image_msg, old_disparity_image_msg;
sensor_msgs::CameraInfoConstPtr camera_info_msg;
boost::mutex m;

void
disparity_image_msg_cb (const stereo_msgs::DisparityImageConstPtr& msg)
{
  m.lock ();
  disparity_image_msg = msg;
  m.unlock ();
}

void
depth_image_msg_cb (const sensor_msgs::ImageConstPtr& msg)
{
  m.lock ();
  //std::cout << "Received depth image of size "<<msg->width<<"x"<<msg->height<<".\n";
  depth_image_msg = msg;
  m.unlock ();
}

void
cloud_msg_cb (const sensor_msgs::PointCloud2ConstPtr& msg)
{
  m.lock ();
  cloud_msg = msg;
  m.unlock ();
}

void
camera_info_msg_cb (const sensor_msgs::CameraInfoConstPtr& msg)
{
  m.lock ();
  //std::cout << "Received camera info: " << "width="<<msg->width<<", height="<<msg->height<<", "
                                        //<< "center_x="<<msg->P[2]<<", center_y="<<msg->P[6]<<", "
                                       //<<"fl_x="<<msg->P[0]<<", fl_y="<<msg->P[5]<<".\n";
  camera_info_msg = msg;
  m.unlock ();
}

int
main (int argc, char** argv)
{
  // Read command line arguments.
  for (char c; (c = getopt (argc, argv, "s:hc:r:")) != -1; ) 
  {
    switch (c) 
    {
      case 'c':
        coordinate_frame = RangeImage::CoordinateFrame (strtol (optarg, NULL, 0));
        break;
      case 'r':
      {
        angular_resolution = strtod (optarg, NULL);
        cout << PVARN(angular_resolution);
        break;
      }
      case 's':
      {
        source = strtol (optarg, NULL, 0);
        if (source < 0  ||  source > 2)
        {
          cout << "Source "<<source<<" is unknown.\n";
          exit (0);
        }
        cout << "Receiving "<<(source==0 ? "point clouds" : (source==1 ? "depth images" : "disparity images"))<<".\n";
        break;
      }
      case 'h':
      default:
        printUsage (argv[0]);
        exit (0);
    }
  }
  angular_resolution = deg2rad (angular_resolution);
  
  ros::init (argc, argv, "tutorial_range_image_live_viewer");
  ros::NodeHandle node_handle;
  ros::Subscriber subscriber, subscriber2;
  
  if (node_handle.resolveName("input")=="/input")
    std::cerr << "Did you forget input:=<your topic>?\n";
  
  if (source == 0)
    subscriber = node_handle.subscribe ("input", 1, cloud_msg_cb);
  else if (source == 1)
  {
    if (node_handle.resolveName("input2")=="/input2")
      std::cerr << "Did you forget input2:=<your camera_info_topic>?\n";
    subscriber = node_handle.subscribe ("input", 1, depth_image_msg_cb);
    subscriber2 = node_handle.subscribe ("input2", 1, camera_info_msg_cb);
  }
  else if (source == 2)
    subscriber  = node_handle.subscribe ("input",  1, disparity_image_msg_cb);
  
  PCLVisualizer viewer (argc, argv, "Live viewer - point cloud");
  RangeImageVisualizer range_image_widget("Live viewer - range image");
  
  RangeImagePlanar* range_image_planar;
  RangeImage* range_image;
  if (source==0)
    range_image = new RangeImage;
  else
  {
    range_image_planar = new RangeImagePlanar;
    range_image = range_image_planar;
  }
  
  while (node_handle.ok ())
  {
    usleep (10000);
    viewer.spinOnce (10);
    RangeImageVisualizer::spinOnce ();
    ros::spinOnce ();
    
    if (source==0)
    {
      // If no new message received: continue
      if (!cloud_msg || cloud_msg == old_cloud_msg)
        continue;
      old_cloud_msg = cloud_msg;
      
      Eigen::Affine3f sensor_pose(Eigen::Affine3f::Identity());
      PointCloud<PointWithViewpoint> far_ranges;
      RangeImage::extractFarRanges(*cloud_msg, far_ranges);
      if (pcl::getFieldIndex(*cloud_msg, "vp_x")>=0)
      {
        PointCloud<PointWithViewpoint> tmp_pc;
        fromROSMsg(*cloud_msg, tmp_pc);
        Eigen::Vector3f average_viewpoint = RangeImage::getAverageViewPoint(tmp_pc);
        sensor_pose = Eigen::Translation3f(average_viewpoint) * sensor_pose;
      }
      PointCloud<PointType> point_cloud;
      fromROSMsg(*cloud_msg, point_cloud);
      range_image->createFromPointCloud(point_cloud, angular_resolution, deg2rad(360.0f), deg2rad(180.0f),
                                        sensor_pose, coordinate_frame, noise_level, min_range, border_size);
    }
    else if (source==1)
    {
      // If no new message received: continue
      if (!depth_image_msg || depth_image_msg == old_depth_image_msg || !camera_info_msg)
        continue;
      old_depth_image_msg = depth_image_msg;
      range_image_planar->setDepthImage(reinterpret_cast<const float*> (&depth_image_msg->data[0]),
                                        depth_image_msg->width, depth_image_msg->height,
                                        camera_info_msg->P[2],  camera_info_msg->P[6],
                                        camera_info_msg->P[0],  camera_info_msg->P[5], angular_resolution);
    }
    else if (source==2)
    {
      // If no new message received: continue
      if (!disparity_image_msg || disparity_image_msg == old_disparity_image_msg)
        continue;
      old_disparity_image_msg = disparity_image_msg;
      range_image_planar->setDisparityImage(reinterpret_cast<const float*> (&disparity_image_msg->image.data[0]),
                                            disparity_image_msg->image.width, disparity_image_msg->image.height,
                                            disparity_image_msg->f, disparity_image_msg->T, angular_resolution);
    }

    range_image_widget.setRangeImage (*range_image);
    viewer.removePointCloud ("range image cloud");
    pcl_visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> color_handler_cloud(*range_image,
                                                                                             200, 200, 200);
    viewer.addPointCloud (*range_image, color_handler_cloud, "range image cloud");
  }
}
