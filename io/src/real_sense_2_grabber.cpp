/*
* Software License Agreement (BSD License)
*
*  Point Cloud Library (PCL) - www.pointclouds.org
*  Copyright (c) 2018-, Open Perception, Inc.
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

#include <pcl/io/grabber.h>
#include <pcl/io/io_exception.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <librealsense2/rs.hpp>
#include <pcl/io/real_sense_2_grabber.h>
#include <pcl/common/time.h>


namespace pcl
{
  using namespace io;

  RealSense2Grabber::RealSense2Grabber ( const std::string& file_name_or_serial_number, const bool repeat_playback )
    : signal_PointXYZ ( createSignal<signal_librealsense_PointXYZ> () )
    , signal_PointXYZI ( createSignal<signal_librealsense_PointXYZI> () )
    , signal_PointXYZRGB ( createSignal<signal_librealsense_PointXYZRGB> () )
    , signal_PointXYZRGBA ( createSignal<signal_librealsense_PointXYZRGBA> () )
    , file_name_or_serial_number_ ( file_name_or_serial_number )
    , repeat_playback_ ( repeat_playback )
    , quit_ ( false )
    , running_ ( false )
    , fps_ ( 0 )
    , device_width_ ( 424 )
    , device_height_ ( 240 )
    , target_fps_ ( 30 )
  {
  }

  RealSense2Grabber::~RealSense2Grabber ()
  {
    try
    {
      stop ( );

      disconnect_all_slots<signal_librealsense_PointXYZ> ( );
      disconnect_all_slots<signal_librealsense_PointXYZI> ( );
      disconnect_all_slots<signal_librealsense_PointXYZRGB> ( );
      disconnect_all_slots<signal_librealsense_PointXYZRGBA> ( );
    } 
    catch ( ... ) {}
  }

  void
  RealSense2Grabber::start ()
  {
    if (isRunning ())
      return;

    // need at least one signal
    if (signal_PointXYZ->num_slots () == 0 &&
      signal_PointXYZI->num_slots () == 0 &&
      signal_PointXYZRGB->num_slots () == 0 &&
      signal_PointXYZRGBA->num_slots () == 0)
      return;

    running_ = true;
    quit_ = false;

    rs2::config cfg;

    // capture from file
    if (file_name_or_serial_number_.rfind ( ".bag" ) == file_name_or_serial_number_.length () - 4)
    {
      cfg.enable_device_from_file ( file_name_or_serial_number_, repeat_playback_ );
    }
    else
    {
      if (!file_name_or_serial_number_.empty ())
        cfg.enable_device ( file_name_or_serial_number_ );

      if (signal_PointXYZRGB->num_slots () > 0 || signal_PointXYZRGBA->num_slots () > 0)
      {
        cfg.enable_stream ( RS2_STREAM_COLOR, device_width_, device_height_, RS2_FORMAT_RGB8, target_fps_ );
      }

      cfg.enable_stream ( RS2_STREAM_DEPTH, device_width_, device_height_, RS2_FORMAT_Z16, target_fps_ );

      if (signal_PointXYZI->num_slots () > 0)
      {
        cfg.enable_stream ( RS2_STREAM_INFRARED, device_width_, device_height_, RS2_FORMAT_Y8, target_fps_ );
      }

    }

    rs2::pipeline_profile prof = pipe_.start ( cfg );

    if ( prof.get_stream ( RS2_STREAM_COLOR ).format ( ) != RS2_FORMAT_RGB8 ||
      prof.get_stream ( RS2_STREAM_DEPTH ).format ( ) != RS2_FORMAT_Z16 ||
      prof.get_stream ( RS2_STREAM_INFRARED ).format ( ) != RS2_FORMAT_Y8 )
      THROW_IO_EXCEPTION ( "This stream type or format not supported." );

    thread_ = std::thread ( &RealSense2Grabber::threadFunction, this );
  }

  void
  RealSense2Grabber::stop ()
  {
    if ( !isRunning ( ) )
      return;

    running_ = false;
    quit_ = true;

    thread_.join ( );

    pipe_.stop ( );
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
  RealSense2Grabber::signalsChanged ()
  {
    reInitialize ();
  }

  void
  RealSense2Grabber::threadFunction ()
  {
    pcl::StopWatch sw;

    while (!quit_)
    {
      sw.reset ();

      // Wait for the next set of frames from the camera
      auto frameset = pipe_.wait_for_frames ();

      auto depth = frameset.get_depth_frame ();

      // Generate the pointcloud and texture mappings
      auto points = pc_.calculate ( depth );

      if (signal_PointXYZ->num_slots () > 0)
      {
        (*signal_PointXYZ)(convertDepthToPointXYZ ( points ));
      }

      if (signal_PointXYZI->num_slots () > 0)
      {
        auto ir = frameset.get_infrared_frame ();
        (*signal_PointXYZI)(convertIntensityDepthToPointXYZRGBI ( points, ir ));
      }

      if (signal_PointXYZRGB->num_slots () > 0 || signal_PointXYZRGBA->num_slots () > 0)
      {
        auto rgb = frameset.get_color_frame ();

        // Tell pointcloud object to map to this color frame
        pc_.map_to ( rgb );

        if (signal_PointXYZRGB->num_slots () > 0)
        {
          (*signal_PointXYZRGB)(convertRGBDepthToPointXYZRGB ( points, rgb ));
        }

        if (signal_PointXYZRGBA->num_slots () > 0)
        {
          (*signal_PointXYZRGBA)(convertRGBADepthToPointXYZRGBA ( points, rgb ));
        }
      }

      fps_ = 1.0f / static_cast <float> (sw.getTimeSeconds ());

    }
  }

  void RealSense2Grabber::reInitialize ()
  {
    if (isRunning ())
    {
      stop ();
      start ();
    }
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr
  RealSense2Grabber::convertDepthToPointXYZ ( const rs2::points& points )
  {
    return convertRealsensePointsToPointCloud<pcl::PointXYZ> ( points, []( pcl::PointXYZ&, const rs2::texture_coordinate*) {} );
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr
  RealSense2Grabber::convertRGBDepthToPointXYZRGB ( const rs2::points& points, const rs2::video_frame& texture )
  {
    return convertRealsensePointsToPointCloud<pcl::PointXYZRGB> ( points, [&]( pcl::PointXYZRGB& p, const rs2::texture_coordinate* uvptr )
    {
      auto clr = getTextureColor ( texture, uvptr->u, uvptr->v );

      p.r = clr.r;
      p.g = clr.g;
      p.b = clr.b;
    } );
  }

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr
  RealSense2Grabber::convertRGBADepthToPointXYZRGBA ( const rs2::points& points, const rs2::video_frame& texture )
  {
    return convertRealsensePointsToPointCloud<pcl::PointXYZRGBA> ( points, [&]( pcl::PointXYZRGBA& p, const rs2::texture_coordinate* uvptr )
    {
      auto clr = getTextureColor ( texture, uvptr->u, uvptr->v );

      p.r = clr.r;
      p.g = clr.g;
      p.b = clr.b;
    } );
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr
  RealSense2Grabber::convertIntensityDepthToPointXYZRGBI ( const rs2::points& points, const rs2::video_frame& texture )
  {
    if (texture.get_profile ().format () == RS2_FORMAT_UYVY)
    {
      return convertRealsensePointsToPointCloud<pcl::PointXYZI> ( points, [&]( pcl::PointXYZI& p, const rs2::texture_coordinate* uvptr )
      {
        auto clr = getTextureColor ( texture, uvptr->u, uvptr->v );
        p.intensity = 0.299f * static_cast <float> (clr.r) + 0.587f * static_cast <float> (clr.g) + 0.114f * static_cast <float> (clr.b);
      } );
    }
    else
    {
      return convertRealsensePointsToPointCloud<pcl::PointXYZI> ( points, [&]( pcl::PointXYZI& p, const rs2::texture_coordinate* uvptr )
      {
        p.intensity = getTextureIntensity ( texture, uvptr->u, uvptr->v );
      } );
    }
  }

  template <typename PointT, typename Functor>
  typename pcl::PointCloud<PointT>::Ptr
  RealSense2Grabber::convertRealsensePointsToPointCloud ( const rs2::points& points, Functor mapColorFunc )
  {
    typename pcl::PointCloud<PointT>::Ptr cloud ( new pcl::PointCloud<PointT> () );

    auto sp = points.get_profile ().as<rs2::video_stream_profile> ();
    cloud->width = sp.width ();
    cloud->height = sp.height ();
    cloud->is_dense = false;
    cloud->resize ( points.size () );

    const auto cloud_vertices_ptr = points.get_vertices ();
    const auto cloud_texture_ptr = points.get_texture_coordinates ();

#if OPENMP_LEGACY_CONST_DATA_SHARING_RULE
#pragma omp parallel for \
  default(none) \
  shared(cloud, mapColorFunc)
#else
#pragma omp parallel for \
  default(none) \
  shared(cloud, cloud_texture_ptr, cloud_vertices_ptr, mapColorFunc)
#endif
    for (std::size_t index = 0; index < cloud->size (); ++index)
    {
      const auto ptr = cloud_vertices_ptr + index;
      const auto uvptr = cloud_texture_ptr + index;
      auto& p = (*cloud)[index];

      p.x = ptr->x;
      p.y = ptr->y;
      p.z = ptr->z;

      mapColorFunc ( p, uvptr );
    }

    return cloud;
  }

  size_t
  RealSense2Grabber::getTextureIdx (const rs2::video_frame & texture, float u, float v)
  {
    const int w = texture.get_width (), h = texture.get_height ();
    int x = std::min (std::max (int (u*w + .5f), 0), w - 1);
    int y = std::min (std::max (int (v*h + .5f), 0), h - 1);
    return x * texture.get_bytes_per_pixel () + y * texture.get_stride_in_bytes ();
  }

  pcl::RGB
  RealSense2Grabber::getTextureColor ( const rs2::video_frame& texture, float u, float v )
  {
    const auto idx = getTextureIdx (texture, u, v);
    const auto texture_data = reinterpret_cast<const std::uint8_t*>(texture.get_data ());

    pcl::RGB rgb;
    rgb.r = texture_data[idx];
    rgb.g = texture_data[idx + 1];
    rgb.b = texture_data[idx + 2];
    return rgb;
  }

  std::uint8_t
  RealSense2Grabber::getTextureIntensity ( const rs2::video_frame& texture, float u, float v )
  {
    const auto idx = getTextureIdx (texture, u, v);
    const auto texture_data = reinterpret_cast<const std::uint8_t*>(texture.get_data ());
    return texture_data[idx];
  }
}
