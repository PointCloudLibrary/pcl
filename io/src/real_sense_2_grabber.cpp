/*
* Software License Agreement (BSD License)
*
*  Point Cloud Library (PCL) - www.pointclouds.org
*  Copyright (c) 2010-2011, Willow Garage, Inc.
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
*/

#include <pcl/io/eigen.h>
#include <pcl/io/eigen.h>
#include <pcl/io/boost.h>
#include <pcl/io/grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <librealsense2/rs.hpp>
#include <pcl/io/real_sense_2_grabber.h>
#include <pcl/common/time.h>

namespace pcl
{

  RealSense2Grabber::RealSense2Grabber (const uint32_t serial_number)
    : running_ (false)
    , quit_ (false)
    , serial_number_ (serial_number)
    , signal_PointXYZ (nullptr)
    , signal_PointXYZI (nullptr)
    , signal_PointXYZRGB (nullptr)
    , signal_PointXYZRGBA (nullptr)
    , fps_ (0)
    , device_width_ (424)
    , device_height_ (240)
    , target_fps_ (30)
  {
    signal_PointXYZ = createSignal<signal_librealsense_PointXYZ> ();
    signal_PointXYZI = createSignal<signal_librealsense_PointXYZI> ();
    signal_PointXYZRGB = createSignal<signal_librealsense_PointXYZRGB> ();
    signal_PointXYZRGBA = createSignal<signal_librealsense_PointXYZRGBA> ();
  }

  RealSense2Grabber::RealSense2Grabber (const std::string& file_name)
    : file_name_(file_name)
  {
    RealSense2Grabber ();
  }


  RealSense2Grabber::~RealSense2Grabber ()
  {
    stop ();

    disconnect_all_slots<signal_librealsense_PointXYZ> ();
    disconnect_all_slots<signal_librealsense_PointXYZI> ();
    disconnect_all_slots<signal_librealsense_PointXYZRGB> ();
    disconnect_all_slots<signal_librealsense_PointXYZRGBA> ();

    thread_.join ();

    pipe_.stop ();
  }

  void 
  RealSense2Grabber::start ()
  {
    running_ = true;

    rs2::config cfg;

    // capture from file
    if (!file_name_.empty ())
    {
      cfg.enable_device_from_file (file_name_);
    }
    else
    {
      if (serial_number_)
        cfg.enable_device (std::to_string (serial_number_));

      cfg.enable_stream (RS2_STREAM_COLOR, device_width_, device_height_, RS2_FORMAT_RGB8, target_fps_);
      cfg.enable_stream (RS2_STREAM_DEPTH, device_width_, device_height_, RS2_FORMAT_ANY, target_fps_);
      cfg.enable_stream (RS2_STREAM_INFRARED, device_width_, device_height_, RS2_FORMAT_ANY, target_fps_);
    }    

    pipe_.start (cfg);

    thread_ = std::thread (&RealSense2Grabber::threadFunction, this);

  }

  void 
  RealSense2Grabber::stop ()
  {
    std::lock_guard<std::mutex> guard (mutex_);

    quit_ = true;
    running_ = false;
  }

  bool 
  RealSense2Grabber::isRunning () const
  {
    std::lock_guard<std::mutex> guard (mutex_);

    return running_;
  }

  float 
  RealSense2Grabber::getFramesPerSecond () const
  {
    return fps_;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr 
  RealSense2Grabber::convertDepthToPointXYZ (const rs2::points & points)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());

    auto sp = points.get_profile ().as<rs2::video_stream_profile> ();
    cloud->width = sp.width ();
    cloud->height = sp.height ();
    cloud->is_dense = false;
    cloud->points.resize (points.size ());

    auto vertices_ptr = points.get_vertices ();

#ifdef _OPENMP
#pragma omp parallel for 
#endif
    for (int index = 0; index < cloud->points.size (); ++index)
    {
      auto ptr = vertices_ptr + index;
      auto& p = cloud->points[index];

      p.x = ptr->x;
      p.y = ptr->y;
      p.z = ptr->z;
    }

    return cloud;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr 
  RealSense2Grabber::convertInfraredDepthToPointXYZI (const rs2::points & points, rs2::video_frame & ir)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI> ());

    auto sp = points.get_profile ().as<rs2::video_stream_profile> ();
    cloud->width = sp.width ();
    cloud->height = sp.height ();
    cloud->is_dense = false;
    cloud->points.resize (points.size ());

    auto vertices_ptr = points.get_vertices ();
    auto texture_ptr = points.get_texture_coordinates ();

    uint8_t r, g, b;

#ifdef _OPENMP
#pragma omp parallel for 
#endif
    for (int index = 0; index < cloud->points.size (); ++index)
    {
      auto ptr = vertices_ptr + index;
      auto uvptr = texture_ptr + index;
      auto& p = cloud->points[index];

      p.x = ptr->x;
      p.y = ptr->y;
      p.z = ptr->z;

      std::tie (r, g, b) = getTextureColor (ir, uvptr->u, uvptr->v);

      p.intensity = 0.299f * static_cast <float> (r) + 0.587f * static_cast <float> (g) + 0.114f * static_cast <float> (b);

    }

    return cloud;
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr 
  RealSense2Grabber::convertRGBDepthToPointXYZRGB (const rs2::points & points, rs2::video_frame & rgb)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());

    auto sp = points.get_profile ().as<rs2::video_stream_profile> ();
    cloud->width = sp.width ();
    cloud->height = sp.height ();
    cloud->is_dense = false;
    cloud->points.resize (points.size ());

    auto vertices_ptr = points.get_vertices ();
    auto texture_ptr = points.get_texture_coordinates ();

#ifdef _OPENMP
#pragma omp parallel for 
#endif
    for (int index = 0; index < cloud->points.size (); ++index)
    {
      auto ptr = vertices_ptr + index;
      auto uvptr = texture_ptr + index;
      auto& p = cloud->points[index];

      p.x = ptr->x;
      p.y = ptr->y;
      p.z = ptr->z;

      std::tie (p.r, p.g, p.b) = getTextureColor (rgb, uvptr->u, uvptr->v);

      p.a = 255;
    }

    return cloud;
  }

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr 
  RealSense2Grabber::convertRGBADepthToPointXYZRGBA (const rs2::points & points, rs2::video_frame & rgb)
  {
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA> ());

    auto sp = points.get_profile ().as<rs2::video_stream_profile> ();
    cloud->width = sp.width ();
    cloud->height = sp.height ();
    cloud->is_dense = false;
    cloud->points.resize (points.size ());

    auto vertices_ptr = points.get_vertices ();
    auto texture_ptr = points.get_texture_coordinates ();

#ifdef _OPENMP
#pragma omp parallel for 
#endif
    for (int index = 0; index < cloud->points.size (); ++index)
    {
      auto ptr = vertices_ptr + index;
      auto uvptr = texture_ptr + index;
      auto& p = cloud->points[index];

      p.x = ptr->x;
      p.y = ptr->y;
      p.z = ptr->z;

      std::tie (p.r, p.g, p.b) = getTextureColor (rgb, uvptr->u, uvptr->v);

      p.a = 255;
    }

    return cloud;
  }

  std::tuple<uint8_t, uint8_t, uint8_t> 
  RealSense2Grabber::getTextureColor (rs2::video_frame &texture, float u, float v)
  {
    auto ptr = dynamic_cast<rs2::video_frame*>(&texture);

    const int w = ptr->get_width (), h = ptr->get_height ();
    int x = std::min (std::max (int (u*w + .5f), 0), w - 1);
    int y = std::min (std::max (int (v*h + .5f), 0), h - 1);
    int idx = x * ptr->get_bytes_per_pixel() + y * ptr->get_stride_in_bytes ();
    const auto texture_data = reinterpret_cast<const uint8_t*>(ptr->get_data ());
    return std::make_tuple (texture_data[idx], texture_data[idx + 1], texture_data[idx + 2]);
  }

  void 
  RealSense2Grabber::threadFunction ()
  {
    pcl::StopWatch sw;

    rs2::frameset frames;
    rs2::depth_frame depth = NULL;
    rs2::video_frame rgb = NULL;
    rs2::video_frame ir = NULL;
    rs2::points points;

    while (!quit_)
    {
      sw.reset ();

      {
        std::lock_guard<std::mutex> guard (mutex_);

        // Wait for the next set of frames from the camera
        frames = pipe_.wait_for_frames ();

        depth = frames.get_depth_frame ();

        ir = frames.get_infrared_frame ();

        // Generate the pointcloud and texture mappings
        points = pc_.calculate (depth);

        rgb = frames.get_color_frame ();

        // Tell pointcloud object to map to this color frame
        pc_.map_to (rgb);

      }

      if (signal_PointXYZ->num_slots () > 0)
      {
        signal_PointXYZ->operator()(convertDepthToPointXYZ (points));
      }

      if (signal_PointXYZI->num_slots () > 0)
      {
        signal_PointXYZI->operator()(convertInfraredDepthToPointXYZI (points, ir));
      }

      if (signal_PointXYZRGB->num_slots () > 0)
      {
        signal_PointXYZRGB->operator()(convertRGBDepthToPointXYZRGB (points, rgb));
      }

      if (signal_PointXYZRGBA->num_slots () > 0)
      {
        signal_PointXYZRGBA->operator()(convertRGBADepthToPointXYZRGBA (points, rgb));
      }

      fps_ = 1.0f / static_cast <float> (sw.getTimeSeconds ());

    }
  }

}
