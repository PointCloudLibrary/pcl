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

#include <pcl/pcl_config.h>
#ifdef HAVE_FZAPI

#include <pcl/io/fotonic_grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>
#include <pcl/common/io.h>
#include <pcl/console/print.h>
#include <pcl/io/boost.h>
#include <pcl/exceptions.h>

#include <boost/thread.hpp>

#include <iostream>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
pcl::FotonicGrabber::FotonicGrabber (const FZ_DEVICE_INFO& device_info, const Mode& depth_mode, const Mode& image_mode)
  : running_ (false)
{
  // initialize device
  onInit (device_info, depth_mode, image_mode);

  point_cloud_signal_      = createSignal<sig_cb_fotonic_point_cloud> ();
  point_cloud_rgb_signal_  = createSignal<sig_cb_fotonic_point_cloud_rgb> ();
  point_cloud_rgba_signal_ = createSignal<sig_cb_fotonic_point_cloud_rgba> ();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
pcl::FotonicGrabber::~FotonicGrabber () throw ()
{
  stop ();

  disconnect_all_slots<sig_cb_fotonic_point_cloud> ();
  disconnect_all_slots<sig_cb_fotonic_point_cloud_rgb> ();
  disconnect_all_slots<sig_cb_fotonic_point_cloud_rgba> ();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::FotonicGrabber::initAPI ()
{
  FZ_Init ();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::FotonicGrabber::exitAPI ()
{
  FZ_Exit ();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::vector<FZ_DEVICE_INFO>
pcl::FotonicGrabber::enumDevices ()
{
  // enumerate devices
  int num_of_devices = 32;
  FZ_DEVICE_INFO * device_infos = new FZ_DEVICE_INFO[num_of_devices];
  FZ_Result result = FZ_EnumDevices2 (device_infos, &num_of_devices);

  // put found devices into vector
  std::vector<FZ_DEVICE_INFO> devices;
  for (int index = 0; index < num_of_devices; ++index)
  {
    FZ_DEVICE_INFO device_info;
    device_info.iDeviceType = device_infos[index].iDeviceType;
    memcpy (device_info.iReserved, device_infos[index].iReserved, sizeof (device_infos[index].iReserved[0])*64);
    memcpy (device_info.szPath, device_infos[index].szPath, sizeof (device_infos[index].szPath[0])*512);
    memcpy (device_info.szSerial, device_infos[index].szSerial, sizeof (device_infos[index].szSerial[0])*16);
    memcpy (device_info.szShortName, device_infos[index].szShortName, sizeof (device_infos[index].szShortName[0])*32);

    devices.push_back (device_info);
  }

  return (devices);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::FotonicGrabber::start ()
{
  FZ_CmdRespCode_t resp;
  FZ_Result res = FZ_IOCtl (*fotonic_device_handle_, CMD_DE_SENSOR_START, NULL, 0, &resp, NULL, NULL);

  running_ = true;

  grabber_thread_ = boost::thread(&pcl::FotonicGrabber::processGrabbing, this); 
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::FotonicGrabber::stop ()
{
  running_ = false;
  grabber_thread_.join ();

  FZ_Close (*fotonic_device_handle_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::FotonicGrabber::isRunning () const
{
  return (running_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::string
pcl::FotonicGrabber::getName () const
{
  return std::string ("FotonicGrabber");
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float
pcl::FotonicGrabber::getFramesPerSecond () const
{
  //return (static_cast<float> (device_->getDepthOutputMode ().nFPS));
  return 0.0f;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::FotonicGrabber::onInit (const FZ_DEVICE_INFO& device_info, const Mode& depth_mode, const Mode& image_mode)
{
  setupDevice (device_info, depth_mode, image_mode);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::FotonicGrabber::setupDevice (const FZ_DEVICE_INFO& device_info, const Mode& depth_mode, const Mode& image_mode)
{
  // Initialize device
  fotonic_device_handle_ = new FZ_Device_Handle_t ();

  unsigned int flags = 0;

  FZ_Result res;

  res = FZ_Open (device_info.szPath, flags, fotonic_device_handle_);
      
  // set mode (resolution, fps..)
	FZ_CmdRespCode_t resp;
  unsigned short mode = DE_MODE_640X480_30;
  res = FZ_IOCtl (*fotonic_device_handle_, CMD_DE_SET_MODE, &mode, sizeof(mode), &resp, NULL, NULL);

  res = FZ_SetFrameDataFmt (*fotonic_device_handle_, -1, -1, -1, -1, FZ_FMT_PIXEL_PER_PLANE+FZ_FMT_COMPONENT_Z+FZ_FMT_COMPONENT_XY+FZ_FMT_COMPONENT_B);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::FotonicGrabber::processGrabbing ()
{
  char * frame_buffer = new char [640*480*8];

  bool continue_grabbing = running_;
  while (continue_grabbing)
  {
    FZ_Result result = FZ_FrameAvailable (*fotonic_device_handle_);
    //std::cerr << "FZ_FrameAvailable: " << result << std::endl;

    if (result == FZ_Success)
    {
      size_t length_in_byte = 640*480*10;
      FZ_FRAME_HEADER frame_header;

      result = FZ_GetFrame (*fotonic_device_handle_, &frame_header, frame_buffer, &length_in_byte);

      if (result == FZ_Success)
      {
        //std::cerr << "frame: " << frame_header.framecounter << std::endl;
        //std::cerr << "  timestamp: " << frame_header.timestamp << std::endl;
        //std::cerr << "  format: " << frame_header.format << std::endl;
        //std::cerr << "  cols: " << frame_header.ncols << std::endl;
        //std::cerr << "  rows: " << frame_header.nrows << std::endl;
        //std::cerr << "  reportedframerate: " << frame_header.reportedframerate << std::endl;
        //std::cerr << "  bytesperpixel: " << frame_header.bytesperpixel << std::endl;

        const int width = frame_header.ncols;
        const int height = frame_header.nrows;

        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA> ());
        cloud->resize (width*height);
        cloud->width = width;
        cloud->height = height;
        cloud->is_dense = false;

        short * ptr = (short*) frame_buffer;

        for (int row_index = 0; row_index < height; ++row_index)
        {
          //if(pixelformat == FZ_PIXELFORMAT_YUV422) 
          {
            // YUV422
            FZ_YUV422_DOUBLE_PIXEL *p = (FZ_YUV422_DOUBLE_PIXEL*)ptr;
            int col = 0;
            for (int col_index = 0; col_index < width/2; ++col_index)
            {
              pcl::PointXYZRGBA & point0 = (*cloud) (col, row_index);
              ++col;
              pcl::PointXYZRGBA & point1 = (*cloud) (col, row_index);
              ++col;

              float r,g,b,u,v,u1,v1,uv1;

              u = p[col_index].u - 128.0f;
              v = p[col_index].v - 128.0f;
              v1 = 1.13983f*v;
              uv1 = -0.39465f*u - 0.58060f*v;
              u1 = 0.03211f*u;

              r = p[col_index].y1 + v1;
              g = p[col_index].y1 + uv1;
              b = p[col_index].y1 + u1;

              r = std::min (255.0f, std::max (0.0f, r));
              g = std::min (255.0f, std::max (0.0f, g));
              b = std::min (255.0f, std::max (0.0f, b));

              point0.r = unsigned(r);
              point0.g = unsigned(g);
              point0.b = unsigned(b);
               
              r = p[col_index].y2 + v1;
              g = p[col_index].y2 + uv1;
              b = p[col_index].y2 + u1;

              r = std::min (255.0f, std::max (0.0f, r));
              g = std::min (255.0f, std::max (0.0f, g));
              b = std::min (255.0f, std::max (0.0f, b));
               
              point1.r = unsigned(r);
              point1.g = unsigned(g);
              point1.b = unsigned(b);
            }

            ptr += width;
          } 

          for (int col_index = 0; col_index < width; ++col_index)
          {
            pcl::PointXYZRGBA & point = (*cloud) (col_index, row_index);

            short z = *ptr;

            point.z = static_cast<float> (z) / 1000.0f;

            ++ptr;
          }

          for (int col_index = 0; col_index < width; ++col_index)
          {
            pcl::PointXYZRGBA & point = (*cloud) (col_index, row_index);

            short x = *ptr;
            ++ptr;

            point.x = -static_cast<float> (x) / 1000.0f;
          }

          for (int col_index = 0; col_index < width; ++col_index)
          {
            pcl::PointXYZRGBA & point = (*cloud) (col_index, row_index);

            short y = *ptr;
            ++ptr;

            point.y = static_cast<float> (y) / 1000.0f;
          }
        }

        // publish cloud
        if (num_slots<sig_cb_fotonic_point_cloud_rgba> () > 0)
        {
          point_cloud_rgba_signal_->operator() (cloud);
        }
        if (num_slots<sig_cb_fotonic_point_cloud_rgb> () > 0)
        {
          pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
          cloud->resize (width*height);
          cloud->width = width;
          cloud->height = height;
          cloud->is_dense = false;

          pcl::copyPointCloud<pcl::PointXYZRGBA, pcl::PointXYZRGB> (*cloud, *tmp_cloud);

          point_cloud_rgb_signal_->operator() (tmp_cloud);
        }
        if (num_slots<sig_cb_fotonic_point_cloud> () > 0)
        {
          pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tmp (new pcl::PointCloud<pcl::PointXYZ> ());
          cloud->resize (width*height);
          cloud->width = width;
          cloud->height = height;
          cloud->is_dense = false;

          pcl::copyPointCloud<pcl::PointXYZRGBA, pcl::PointXYZ> (*cloud, *cloud_tmp);

          point_cloud_signal_->operator() (cloud_tmp);
        }

      }
    }
    else
      boost::this_thread::sleep (boost::posix_time::milliseconds(1));

    continue_grabbing = running_;
  }
}


#endif
