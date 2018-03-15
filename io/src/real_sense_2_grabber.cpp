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
#include <pcl/io/boost.h>
#include <pcl/io/grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <librealsense2/rs.hpp>
#include <pcl/io/real_sense_2_grabber.h>
#include <pcl/common/time.h>

pcl::RGB
getTextureColor (const rs2::video_frame& texture, float u, float v);

uint8_t
getTextureIntensity (const rs2::video_frame& texture, float u, float v);

template <typename PointT, typename Functor>
typename pcl::PointCloud<PointT>::Ptr
convertRealsensePointsToPointCloud (const rs2::points& points, Functor mapColorFunc);

pcl::PointCloud<pcl::PointXYZ>::Ptr
convertDepthToPointXYZ (const rs2::points& points);

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
convertRGBDepthToPointXYZRGB (const rs2::points& points, const rs2::video_frame& texture);

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr
convertRGBADepthToPointXYZRGBA (const rs2::points& points, const rs2::video_frame& texture);

pcl::PointCloud<pcl::PointXYZI>::Ptr
convertIntensityDepthToPointXYZRGBI (const rs2::points& points, const rs2::video_frame& texture);

namespace pcl
{
  RealSense2Grabber::RealSense2Grabber (const std::string& file_name_or_serial_number)
    : signal_PointXYZ (createSignal<signal_librealsense_PointXYZ> ())
    , signal_PointXYZI (createSignal<signal_librealsense_PointXYZI> ())
    , signal_PointXYZRGB (createSignal<signal_librealsense_PointXYZRGB> ())
    , signal_PointXYZRGBA (createSignal<signal_librealsense_PointXYZRGBA> ())
    , file_name_or_serial_number_ (file_name_or_serial_number)
    , quit_ (false)
    , running_ (false)
    , fps_ (0)
    , device_width_ (424)
    , device_height_ (240)
    , target_fps_ (30)    
  {
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
    if (isRunning ())
      return;

    running_ = true;

    rs2::config cfg;

    // capture from file
    if (file_name_or_serial_number_.rfind (".bag") == file_name_or_serial_number_.length() - 4)
    {
      cfg.enable_device_from_file (file_name_or_serial_number_);
    }
    else
    {
      if (!file_name_or_serial_number_.empty ())
        cfg.enable_device (file_name_or_serial_number_);

      cfg.enable_stream (RS2_STREAM_COLOR, device_width_, device_height_, RS2_FORMAT_RGB8, target_fps_);
      cfg.enable_stream (RS2_STREAM_DEPTH, device_width_, device_height_, RS2_FORMAT_Z16, target_fps_);
      cfg.enable_stream (RS2_STREAM_INFRARED, device_width_, device_height_, RS2_FORMAT_Y8, target_fps_);
    }    

    rs2::pipeline_profile prof = pipe_.start (cfg);

    assert (prof.get_stream (RS2_STREAM_COLOR).format () == RS2_FORMAT_RGB8);
    assert (prof.get_stream (RS2_STREAM_DEPTH).format () == RS2_FORMAT_Z16);

    ir_format_ = prof.get_stream (RS2_STREAM_INFRARED).format (); 

    thread_ = std::thread (&RealSense2Grabber::threadFunction, this);

  }

  void 
  RealSense2Grabber::stop ()
  {
    std::lock_guard<std::mutex> guard (mutex_);

    running_ = false;
    quit_ = true;    
  }

  bool 
  RealSense2Grabber::isRunning () const
  {
    return running_;
  }

  float 
  RealSense2Grabber::getFramesPerSecond () const
  {
    return fps_;
  }

  void 
  RealSense2Grabber::threadFunction ()
  {
    pcl::StopWatch sw;    

    while (!quit_)
    {
      sw.reset ();

      rs2::depth_frame depth = nullptr;
      rs2::video_frame rgb = nullptr;
      rs2::video_frame ir = nullptr;
      rs2::points points;

      {
        std::lock_guard<std::mutex> guard (mutex_);

        // Wait for the next set of frames from the camera
        auto frames = pipe_.wait_for_frames ();

        depth = frames.get_depth_frame ();

        if(signal_PointXYZI->num_slots () > 0)
          ir = frames.get_infrared_frame ();

        // Generate the pointcloud and texture mappings
        points = pc_.calculate (depth);

        if (signal_PointXYZRGB->num_slots () > 0 || signal_PointXYZRGBA->num_slots () > 0)
        {
          rgb = frames.get_color_frame ();

          // Tell pointcloud object to map to this color frame
          pc_.map_to (rgb);
        }

      }


      if (signal_PointXYZ->num_slots () > 0)
      {
        (*signal_PointXYZ)(convertDepthToPointXYZ (points));
      }

      if (signal_PointXYZI->num_slots () > 0)
      {
        (*signal_PointXYZI)(convertIntensityDepthToPointXYZRGBI (points, ir));
      }

      if (signal_PointXYZRGB->num_slots () > 0)
      {
        (*signal_PointXYZRGB)(convertRGBDepthToPointXYZRGB (points, rgb));
      }

      if (signal_PointXYZRGBA->num_slots () > 0)
      {
        (*signal_PointXYZRGBA)(convertRGBADepthToPointXYZRGBA (points, rgb));
      }

      fps_ = 1.0f / static_cast <float> (sw.getTimeSeconds ());

    }
  }

}

pcl::RGB
getTextureColor (const rs2::video_frame& texture, float u, float v)
{
  const int w = texture.get_width (), h = texture.get_height ();
  int x = std::min (std::max (int (u*w + .5f), 0), w - 1);
  int y = std::min (std::max (int (v*h + .5f), 0), h - 1);
  int idx = x * texture.get_bytes_per_pixel () + y * texture.get_stride_in_bytes ();
  const auto texture_data = reinterpret_cast<const uint8_t*>(texture.get_data ());

  pcl::RGB rgb;
  rgb.r = texture_data[idx];
  rgb.g = texture_data[idx + 1];
  rgb.b = texture_data[idx + 2];
  return rgb;
}

uint8_t
getTextureIntensity (const rs2::video_frame& texture, float u, float v)
{
  const int w = texture.get_width (), h = texture.get_height ();
  int x = std::min (std::max (int (u*w + .5f), 0), w - 1);
  int y = std::min (std::max (int (v*h + .5f), 0), h - 1);
  int idx = x * texture.get_bytes_per_pixel () + y * texture.get_stride_in_bytes ();
  const auto texture_data = reinterpret_cast<const uint8_t*>(texture.get_data ());
  return texture_data[idx];
}


template <typename PointT, typename Functor>
typename pcl::PointCloud<PointT>::Ptr
convertRealsensePointsToPointCloud (const rs2::points& points, Functor mapColorFunc)
{
  typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT> ());

  auto sp = points.get_profile ().as<rs2::video_stream_profile> ();
  cloud->width = sp.width ();
  cloud->height = sp.height ();
  cloud->is_dense = false;
  cloud->points.resize (points.size ());

  const auto cloud_vertices_ptr = points.get_vertices ();
  const auto cloud_texture_ptr = points.get_texture_coordinates ();

#ifdef _OPENMP
#pragma omp parallel for 
#endif
  for (int index = 0; index < cloud->points.size (); ++index)
  {
    const auto ptr = cloud_vertices_ptr + index;
    const auto uvptr = cloud_texture_ptr + index;
    auto& p = cloud->points[index];

    p.x = ptr->x;
    p.y = ptr->y;
    p.z = ptr->z;

    mapColorFunc (p, uvptr);
  }

  return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr
convertDepthToPointXYZ (const rs2::points& points)
{
  auto pFunc = [](pcl::PointXYZ& p, const rs2::texture_coordinate* uvptr)
  {
  };

  return convertRealsensePointsToPointCloud<pcl::PointXYZ> (points, pFunc);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
convertRGBDepthToPointXYZRGB (const rs2::points& points, const rs2::video_frame& texture)
{
  auto pFunc = [texture](pcl::PointXYZRGB& p, const rs2::texture_coordinate* uvptr)
  {
    auto clr = getTextureColor (texture, uvptr->u, uvptr->v);

    p.r = clr.r;
    p.g = clr.g;
    p.b = clr.b;
  };

  return convertRealsensePointsToPointCloud<pcl::PointXYZRGB> (points, pFunc);
}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr
convertRGBADepthToPointXYZRGBA (const rs2::points& points, const rs2::video_frame& texture)
{
  auto pFunc = [texture](pcl::PointXYZRGBA& p, const rs2::texture_coordinate* uvptr)
  {
    auto clr = getTextureColor (texture, uvptr->u, uvptr->v);

    p.r = clr.r;
    p.g = clr.g;
    p.b = clr.b;
  };

  return convertRealsensePointsToPointCloud<pcl::PointXYZRGBA> (points, pFunc);
}

pcl::PointCloud<pcl::PointXYZI>::Ptr
convertIntensityDepthToPointXYZRGBI (const rs2::points& points, const rs2::video_frame& texture)
{
  if (texture.get_profile ().format () == RS2_FORMAT_UYVY)
  {
    auto pFunc = [texture](pcl::PointXYZI& p, const rs2::texture_coordinate* uvptr)
    {
      auto clr = getTextureColor (texture, uvptr->u, uvptr->v);
      p.intensity = 0.299f * static_cast <float> (clr.r) + 0.587f * static_cast <float> (clr.g) + 0.114f * static_cast <float> (clr.b);
    };

    return convertRealsensePointsToPointCloud<pcl::PointXYZI> (points, pFunc);

  }
  else
  {
    auto pFunc = [texture](pcl::PointXYZI& p, const rs2::texture_coordinate* uvptr)
    {
      p.intensity = getTextureIntensity (texture, uvptr->u, uvptr->v);
    };

    return convertRealsensePointsToPointCloud<pcl::PointXYZI> (points, pFunc);

  }
}
