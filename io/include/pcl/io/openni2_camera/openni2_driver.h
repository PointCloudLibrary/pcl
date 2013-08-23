/*
 * Copyright (c) 2013, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *      Author: Julius Kammerl (jkammerl@willowgarage.com)
 */

#ifndef OPENNI2_DRIVER_H
#define OPENNI2_DRIVER_H

#include <boost/shared_ptr.hpp>
#include <boost/cstdint.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>

//#include <pcl/io/openni2_camera/OpenNI2Config.h>	// Is this suposed to hold defaults?

#include <string>
#include <vector>

#include "pcl/io/openni2_camera/openni2_device_manager.h"
#include "pcl/io/openni2_camera/openni2_device.h"
#include "pcl/io/openni2_camera/openni2_video_mode.h"

//#include <ros/ros.h>	// ROS

namespace openni2_wrapper
{

class OpenNI2Driver
{
public:
  OpenNI2Driver(ros::NodeHandle& n, ros::NodeHandle& pnh) ;

private:
  typedef openni2_camera::OpenNI2Config Config;
  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;

  void newIRFrameCallback(sensor_msgs::ImagePtr image);
  void newColorFrameCallback(sensor_msgs::ImagePtr image);
  void newDepthFrameCallback(sensor_msgs::ImagePtr image);

  // Methods to get calibration parameters for the various cameras
  sensor_msgs::CameraInfoPtr getDefaultCameraInfo(int width, int height, double f) const;
  sensor_msgs::CameraInfoPtr getColorCameraInfo(int width, int height, ros::Time time) const;
  sensor_msgs::CameraInfoPtr getIRCameraInfo(int width, int height, ros::Time time) const;
  sensor_msgs::CameraInfoPtr getDepthCameraInfo(int width, int height, ros::Time time) const;

  void readConfigFromParameterServer();

  // resolves non-URI device IDs to URIs, e.g. '#1' is resolved to the URI of the first device
  std::string resolveDeviceURI(const std::string& device_id) throw(OpenNI2Exception);
  void initDevice();

  void advertiseROSTopics();

  void colorConnectCb();
  void depthConnectCb();
  void irConnectCb();

  void configCb(Config &config, uint32_t level);

  void applyConfigToOpenNIDevice();

  void genVideoModeTableMap();
  int lookupVideoModeFromDynConfig(int mode_nr, OpenNI2VideoMode& video_mode);

  sensor_msgs::ImageConstPtr rawToFloatingPointConversion(sensor_msgs::ImageConstPtr raw_image);

  void setIRVideoMode(const OpenNI2VideoMode& ir_video_mode);
  void setColorVideoMode(const OpenNI2VideoMode& color_video_mode);
  void setDepthVideoMode(const OpenNI2VideoMode& depth_video_mode);

  ros::NodeHandle& nh_;
  ros::NodeHandle& pnh_;

  boost::shared_ptr<OpenNI2DeviceManager> device_manager_;
  boost::shared_ptr<OpenNI2Device> device_;

  std::string device_id_;

  /** \brief reconfigure server*/
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;
  bool config_init_;

  boost::mutex connect_mutex_;
  // published topics
  image_transport::CameraPublisher pub_color_;
  image_transport::CameraPublisher pub_depth_;
  image_transport::CameraPublisher pub_depth_raw_;
  image_transport::CameraPublisher pub_ir_;
  ros::Publisher pub_projector_info_;

  /** \brief Camera info manager objects. */
  boost::shared_ptr<camera_info_manager::CameraInfoManager> color_info_manager_, ir_info_manager_;

  OpenNI2VideoMode ir_video_mode_;
  OpenNI2VideoMode color_video_mode_;
  OpenNI2VideoMode depth_video_mode_;

  std::string ir_frame_id_;
  std::string color_frame_id_;
  std::string depth_frame_id_ ;

  std::string color_info_url_, ir_info_url_;

  bool color_depth_synchronization_;
  bool depth_registration_;

  std::map<int, OpenNI2VideoMode> video_modes_lookup_;

  // dynamic reconfigure config
  double depth_ir_offset_x_;
  double depth_ir_offset_y_;
  int z_offset_mm_;
  double z_scaling_;

  ros::Duration ir_time_offset_;
  ros::Duration color_time_offset_;
  ros::Duration depth_time_offset_;

  int data_skip_;

  int data_skip_ir_counter_;
  int data_skip_color_counter_;
  int data_skip_depth_counter_;

  bool auto_exposure_;
  bool auto_white_balance_;

  bool ir_subscribers_;
  bool color_subscribers_;
  bool depth_subscribers_;
  bool depth_raw_subscribers_;

  bool use_device_time_;

  Config old_config_;
};

}

#endif
