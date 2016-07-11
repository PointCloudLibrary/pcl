/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2015-, Open Perception, Inc.
 *  Copyright (c) 2016, Intel Corporation
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
 */

#include <boost/lexical_cast.hpp>

#include <librealsense/rs.hpp>

#include <pcl/common/io.h>
#include <pcl/io/buffers.h>
#include <pcl/io/real_sense_grabber.h>
#include <pcl/io/real_sense/librealsense/real_sense_device_manager.h>


using namespace pcl::io;
using namespace pcl::io::real_sense;

pcl::RealSenseGrabber::RealSenseGrabber (const std::string& device_id, const Mode& mode, bool strict)
: Grabber ()
, is_running_ (false)
, confidence_threshold_ (6)
, temporal_filtering_type_ (RealSense_None)
, temporal_filtering_window_size_ (1)
, mode_requested_ (mode)
, strict_ (strict)
{
  if (device_id == "")
    device_ = RealSenseDeviceManager::getInstance ()->captureDevice ();
  else if (device_id[0] == '#')
    device_ = RealSenseDeviceManager::getInstance ()->captureDevice (boost::lexical_cast<int> (device_id.substr (1)) - 1);
  else
    device_ = RealSenseDeviceManager::getInstance ()->captureDevice (device_id);

  point_cloud_signal_ = createSignal<sig_cb_real_sense_point_cloud> ();
  point_cloud_rgba_signal_ = createSignal<sig_cb_real_sense_point_cloud_rgba> ();
}

void
pcl::RealSenseGrabber::start ()
{
  if (!is_running_)
  {
    need_xyz_ = num_slots<sig_cb_real_sense_point_cloud> () > 0;
    need_xyzrgba_ = num_slots<sig_cb_real_sense_point_cloud_rgba> () > 0;
    if (need_xyz_ || need_xyzrgba_)
    {
      if (temporal_filtering_type_ != RealSense_None)
      {
        temporal_filtering_type_ = RealSense_None;
        PCL_WARN ("[pcl::RealSenseGrabber::enableTemporalFiltering] This device does not support temporal filter with librealsense now\n");
      }
      selectMode ();
      device_->getDevice ()->enable_stream (rs::stream::depth, mode_selected_.depth_width, mode_selected_.depth_height, rs::format::z16, mode_selected_.fps);
    }
    if (need_xyzrgba_)
    {
      device_->getDevice ()->enable_stream (rs::stream::color, mode_selected_.color_width, mode_selected_.color_height, rs::format::rgb8, mode_selected_.fps);
    }
    frequency_.reset ();
    is_running_ = true;
    thread_ = boost::thread (&RealSenseGrabber::run, this);
  }
}

const std::string&
pcl::RealSenseGrabber::getDeviceSerialNumber () const
{
  return (device_->getSerialNumber ());
}

void
pcl::RealSenseGrabber::setConfidenceThreshold (unsigned int threshold)
{
  if (threshold > 15)
  {
    PCL_WARN ("[pcl::RealSenseGrabber::setConfidenceThreshold] Attempted to set threshold outside valid range (0-15)");
  }
  else if (!device_->getDevice ()->supports_option (rs::option::f200_confidence_threshold))
  {
    PCL_WARN ("[pcl::RealSenseGrabber::setConfidenceThreshold] This device does not support setting confidence threshold with librealsense now\n");
  }
  else
  {
    device_->getDevice ()->set_option (rs::option::f200_confidence_threshold, threshold);
  }
}

std::vector<pcl::RealSenseGrabber::Mode>
pcl::RealSenseGrabber::getAvailableModes (bool only_depth) const
{
  std::vector<Mode> modes;
  int depth_total = device_->getDevice ()->get_stream_mode_count (rs::stream::depth);
  int color_total = device_->getDevice ()->get_stream_mode_count (rs::stream::color);
  int depth_width = 0, depth_height = 0, depth_fps = 0, depth_width_old = 0, depth_fps_old = 0,
      color_width = 0, color_height = 0, color_fps = 0, color_width_old = 0, color_fps_old = 0;
  rs::format format;
  for (int i = 0; i < depth_total; i++)
  {
    device_->getDevice ()->get_stream_mode (rs::stream::depth, i, depth_width, depth_height, format, depth_fps);
    if ( depth_width == depth_width_old && depth_fps == depth_fps_old) continue;
    depth_width_old = depth_width;
    depth_fps_old = depth_fps;
    if (!only_depth)
    {
      for (int i = 0; i < color_total; i++)
      {
        device_->getDevice ()->get_stream_mode (rs::stream::color, i, color_width, color_height, format, color_fps);
        if ((color_width == color_width_old && color_fps == color_fps_old) || (depth_fps != color_fps)) continue;
        color_width_old = color_width;
        color_fps_old = color_fps;
        Mode mode;
        mode.fps = depth_fps;
        mode.depth_width = depth_width;
        mode.depth_height = depth_height;
        mode.color_width = color_width;
        mode.color_height = color_height;
        modes.push_back (mode);
      }
    }
    else
    {
      Mode mode;
      mode.fps = depth_fps;
      mode.depth_width = depth_width;
      mode.depth_height = depth_height;
      modes.push_back (mode);
    }
  }
  return modes;
}

void
pcl::RealSenseGrabber::run ()
{
  rs::device* device = device_->getDevice ();
  device->start ();
  int depth_width = mode_selected_.depth_width, depth_height = mode_selected_.depth_height, 
      depth_size = depth_width * depth_height, color_width = mode_selected_.color_width, 
      color_height = mode_selected_.color_height, color_size = color_width * color_height * 3;
  ///Buffer to store depth data
  std::vector<uint16_t> depth_data;
  ///Buffer to store color data
  std::vector<uint8_t> color_data;
  while (is_running_)
  {
    if (device->is_streaming ()) device->wait_for_frames ();
    fps_mutex_.lock ();
    frequency_.event ();
    fps_mutex_.unlock ();
    // Retrieve our images
    const uint16_t * depth_image = (const uint16_t *)device->get_frame_data (rs::stream::depth);
    const uint8_t * color_image = (const uint8_t *)device->get_frame_data (rs::stream::color);
    // Retrieve camera parameters for mapping between depth and color
    rs::intrinsics depth_intrin = device->get_stream_intrinsics (rs::stream::depth);
    rs::intrinsics color_intrin = device->get_stream_intrinsics (rs::stream::color);
    rs::extrinsics depth_to_color = device->get_extrinsics (rs::stream::depth, rs::stream::color);
    float scale = device->get_depth_scale ();
    depth_data.clear ();
    depth_data.resize (depth_size);
    pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr xyzrgba_cloud;
    float max_distance = 6; // get rid of noisy data that is past 6 meters
    static const float nan = std::numeric_limits<float>::quiet_NaN ();

    memcpy (depth_data.data (), &depth_image[0], depth_size * sizeof (uint16_t));

    if (need_xyzrgba_)
    {
      color_data.clear ();
      color_data.resize (color_size);
      memcpy (color_data.data (), &color_image[0], color_size * sizeof (uint8_t));
      xyzrgba_cloud.reset (new pcl::PointCloud<pcl::PointXYZRGBA> (depth_width, depth_height));
      xyzrgba_cloud->is_dense = false;
      if (need_xyz_)
      {
        xyz_cloud.reset (new pcl::PointCloud<pcl::PointXYZ> (depth_width, depth_height));
        xyz_cloud->is_dense = false;
      }
      for (int dy = 0; dy < depth_height; ++dy)
      {
        uint32_t i = dy * depth_width - 1;
        for (int dx = 0; dx < depth_width; ++dx)
        {
          i++;
          // Retrieve the 16-bit depth value and map it into a depth in meters
          uint16_t depth_value = depth_data[i];
          float depth_in_meters = depth_value * scale;

          // Map from pixel coordinates in the depth image to real world co-ordinates
          rs::float2 depth_pixel = {(float)dx, (float)dy};
          rs::float3 depth_point = depth_intrin.deproject (depth_pixel, depth_in_meters);
          rs::float3 color_point = depth_to_color.transform (depth_point);
          rs::float2 color_pixel = color_intrin.project (color_point);

          const int cx = (int)std::round (color_pixel.x), cy = (int)std::round (color_pixel.y);
          int red = 0, green = 0, blue = 0;
          if (cx < 0 || cy < 0 || cx >= color_width || cy >= color_height)
          {
            red = 255; green = 255; blue = 255;
          }
          else
          {
            int pos = (cy * color_width + cx) * 3;
            red =  color_data[pos];
            green = color_data[pos + 1];
            blue = color_data[pos + 2];
          }
          if (depth_value == 0 || depth_point.z > max_distance)
          {
            xyzrgba_cloud->points[i].x = xyzrgba_cloud->points[i].y = xyzrgba_cloud->points[i].z = (float) nan;
            if (need_xyz_)
            {
              xyz_cloud->points[i].x = xyz_cloud->points[i].y = xyz_cloud->points[i].z = (float) nan;
            }
            continue;
          }
          else
          {
            xyzrgba_cloud->points[i].x = -depth_point.x;
            xyzrgba_cloud->points[i].y = -depth_point.y;
            xyzrgba_cloud->points[i].z = depth_point.z;
            xyzrgba_cloud->points[i].r = red;
            xyzrgba_cloud->points[i].g = green;
            xyzrgba_cloud->points[i].b = blue;
            if (need_xyz_)
            {
              xyz_cloud->points[i].x = -depth_point.x;
              xyz_cloud->points[i].y = -depth_point.y;
              xyz_cloud->points[i].z = depth_point.z;
            }
          }
        }
      }
      point_cloud_rgba_signal_->operator () (xyzrgba_cloud);
      if (need_xyz_)
      {
        point_cloud_signal_->operator () (xyz_cloud);
      }
    }
    else if (need_xyz_)
    {
      xyz_cloud.reset (new pcl::PointCloud<pcl::PointXYZ> (depth_width, depth_height));
      xyz_cloud->is_dense = false;
      for (int dy = 0; dy < depth_height; ++dy)
      {
        uint32_t i = dy * depth_width - 1;
        for (int dx = 0; dx < depth_width; ++dx)
        {
          i++;
          // Retrieve the 16-bit depth value and map it into a depth in meters
          uint16_t depth_value = depth_data[i];
          float depth_in_meters = depth_value * scale;
          rs::float2 depth_pixel = {(float)dx, (float)dy};
          rs::float3 depth_point = depth_intrin.deproject (depth_pixel, depth_in_meters);
          if (depth_value == 0 || depth_point.z > max_distance)
          {
            xyz_cloud->points[i].x = xyz_cloud->points[i].y = xyz_cloud->points[i].z = (float) nan;
            continue;
          }
          else
          {
            xyz_cloud->points[i].x = -depth_point.x;
            xyz_cloud->points[i].y = -depth_point.y;
            xyz_cloud->points[i].z = depth_point.z;
          }
        }
      }
      point_cloud_signal_->operator () (xyz_cloud);
    }
    else
    {
      //do nothing
    }
  }
  device->stop ();
}