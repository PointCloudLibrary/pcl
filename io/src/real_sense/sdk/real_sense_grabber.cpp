/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2015-, Open Perception, Inc.
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

#include <pcl/io/real_sense_grabber.h>
#include <pcl/io/real_sense/sdk/real_sense_device_manager.h>
#include <pcl/common/io.h>
#include <pcl/io/buffers.h>
#include <pcl/io/io_exception.h>

#include <pxcimage.h>
#include <pxccapture.h>
#include <pxcprojection.h>
#include <pxcsensemanager.h>

using namespace pcl::io;
using namespace pcl::io::real_sense;

/* Helper function to convert a PXCPoint3DF32 point into a PCL point.
 * Takes care of unit conversion (PXC point coordinates are in millimeters)
 * and invalid points. */
template <typename T> inline void
convertPoint (const PXCPoint3DF32& src, T& tgt)
{
  static const float nan = std::numeric_limits<float>::quiet_NaN ();
  if (src.z == 0)
  {
    tgt.x = tgt.y = tgt.z = nan;
  }
  else
  {
    tgt.x = -src.x / 1000.0;
    tgt.y = src.y / 1000.0;
    tgt.z = src.z / 1000.0;
  }
}

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
      selectMode ();
      PXCCapture::Device::StreamProfileSet profile;
      memset (&profile, 0, sizeof (profile));
      profile.depth.frameRate.max = mode_selected_.fps;
      profile.depth.frameRate.min = mode_selected_.fps;
      profile.depth.imageInfo.width = mode_selected_.depth_width;
      profile.depth.imageInfo.height = mode_selected_.depth_height;
      profile.depth.imageInfo.format = PXCImage::PIXEL_FORMAT_DEPTH;
      profile.depth.options = PXCCapture::Device::STREAM_OPTION_ANY;
      if (need_xyzrgba_)
      {
        profile.color.frameRate.max = mode_selected_.fps;
        profile.color.frameRate.min = mode_selected_.fps;
        profile.color.imageInfo.width = mode_selected_.color_width;
        profile.color.imageInfo.height = mode_selected_.color_height;
        profile.color.imageInfo.format = PXCImage::PIXEL_FORMAT_RGB32;
        profile.color.options = PXCCapture::Device::STREAM_OPTION_ANY;
      }
      device_->getPXCDevice ().SetStreamProfileSet (&profile);
      if (!device_->getPXCDevice ().IsStreamProfileSetValid (&profile))
        THROW_IO_EXCEPTION ("Invalid stream profile for PXC device");
      frequency_.reset ();
      is_running_ = true;
      thread_ = boost::thread (&RealSenseGrabber::run, this);
    }
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
  else
  {
    confidence_threshold_ = threshold;
    device_->getPXCDevice ().SetDepthConfidenceThreshold (confidence_threshold_);
  }
}

std::vector<pcl::RealSenseGrabber::Mode>
pcl::RealSenseGrabber::getAvailableModes (bool only_depth) const
{
  std::vector<Mode> modes;
  PXCCapture::StreamType streams = only_depth
    ? PXCCapture::STREAM_TYPE_DEPTH
    : PXCCapture::STREAM_TYPE_DEPTH | PXCCapture::STREAM_TYPE_COLOR;
  for (int p = 0;; p++)
  {
    PXCCapture::Device::StreamProfileSet profiles = {};
    if (device_->getPXCDevice ().QueryStreamProfileSet (streams, p, &profiles) == PXC_STATUS_NO_ERROR)
    {
      if (!only_depth && profiles.depth.frameRate.max != profiles.color.frameRate.max)
        continue; // we need both streams to have the same framerate
      Mode mode;
      mode.fps = profiles.depth.frameRate.max;
      mode.depth_width = profiles.depth.imageInfo.width;
      mode.depth_height = profiles.depth.imageInfo.height;
      mode.color_width = profiles.color.imageInfo.width;
      mode.color_height = profiles.color.imageInfo.height;
      bool duplicate = false;
      for (size_t i = 0; i < modes.size (); ++i)
        duplicate |= modes[i] == mode;
      if (!duplicate)
        modes.push_back (mode);
    }
    else
    {
      break;
    }
  }
  return modes;
}

void
pcl::RealSenseGrabber::run ()
{
  const int WIDTH = mode_selected_.depth_width;
  const int HEIGHT = mode_selected_.depth_height;
  const int SIZE = WIDTH * HEIGHT;

  PXCProjection* projection = device_->getPXCDevice ().CreateProjection ();
  PXCCapture::Sample sample;
  std::vector<PXCPoint3DF32> vertices (SIZE);
  createDepthBuffer ();

  while (is_running_)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr xyzrgba_cloud;

    pxcStatus status;
    if (need_xyzrgba_)
      status = device_->getPXCDevice ().ReadStreams (PXCCapture::STREAM_TYPE_DEPTH | PXCCapture::STREAM_TYPE_COLOR, &sample);
    else
      status = device_->getPXCDevice ().ReadStreams (PXCCapture::STREAM_TYPE_DEPTH, &sample);

    uint64_t timestamp = pcl::getTime () * 1.0e+6;

    switch (status)
    {
    case PXC_STATUS_NO_ERROR:
    {
      fps_mutex_.lock ();
      frequency_.event ();
      fps_mutex_.unlock ();

      /* We preform the following steps to convert received data into point clouds:
       *
       *   1. Push depth image to the depth buffer
       *   2. Pull filtered depth image from the depth buffer
       *   3. Project (filtered) depth image into 3D
       *   4. Fill XYZ point cloud with computed points
       *   5. Fill XYZRGBA point cloud with computed points
       *   7. Project color image into 3D
       *   6. Assign colors to points in XYZRGBA point cloud
       *
       * Steps 1-2 are skipped if temporal filtering is disabled.
       * Step 4 is skipped if there are no subscribers for XYZ clouds.
       * Steps 5-7 are skipped if there are no subscribers for XYZRGBA clouds. */

      if (temporal_filtering_type_ != RealSense_None)
      {
        PXCImage::ImageData data;
        sample.depth->AcquireAccess (PXCImage::ACCESS_READ, &data);
        std::vector<unsigned short> data_copy (SIZE);
        memcpy (data_copy.data (), data.planes[0], SIZE * sizeof (unsigned short));
        sample.depth->ReleaseAccess (&data);

        depth_buffer_->push (data_copy);

        sample.depth->AcquireAccess (PXCImage::ACCESS_WRITE, &data);
        unsigned short* d = reinterpret_cast<unsigned short*> (data.planes[0]);
        for (size_t i = 0; i < SIZE; i++)
          d[i] = (*depth_buffer_)[i];
        sample.depth->ReleaseAccess (&data);
      }

      projection->QueryVertices (sample.depth, vertices.data ());

      if (need_xyz_)
      {
        xyz_cloud.reset (new pcl::PointCloud<pcl::PointXYZ> (WIDTH, HEIGHT));
        xyz_cloud->header.stamp = timestamp;
        xyz_cloud->is_dense = false;
        for (int i = 0; i < SIZE; i++)
          convertPoint (vertices[i], xyz_cloud->points[i]);
      }

      if (need_xyzrgba_)
      {
        PXCImage::ImageData data;
        PXCImage* mapped = projection->CreateColorImageMappedToDepth (sample.depth, sample.color);
        mapped->AcquireAccess (PXCImage::ACCESS_READ, &data);
        uint32_t* d = reinterpret_cast<uint32_t*> (data.planes[0]);
        if (need_xyz_)
        {
          // We can fill XYZ coordinates more efficiently using pcl::copyPointCloud,
          // given that they were already computed for XYZ point cloud.
          xyzrgba_cloud.reset (new pcl::PointCloud<pcl::PointXYZRGBA>);
          pcl::copyPointCloud (*xyz_cloud, *xyzrgba_cloud);
          for (int i = 0; i < HEIGHT; i++)
          {
            pcl::PointXYZRGBA* cloud_row = &xyzrgba_cloud->points[i * WIDTH];
            uint32_t* color_row = &d[i * data.pitches[0] / sizeof (uint32_t)];
            for (int j = 0; j < WIDTH; j++)
              memcpy (&cloud_row[j].rgba, &color_row[j], sizeof (uint32_t));
          }
        }
        else
        {
          xyzrgba_cloud.reset (new pcl::PointCloud<pcl::PointXYZRGBA> (WIDTH, HEIGHT));
          xyzrgba_cloud->header.stamp = timestamp;
          xyzrgba_cloud->is_dense = false;
          for (int i = 0; i < HEIGHT; i++)
          {
            PXCPoint3DF32* vertices_row = &vertices[i * WIDTH];
            pcl::PointXYZRGBA* cloud_row = &xyzrgba_cloud->points[i * WIDTH];
            uint32_t* color_row = &d[i * data.pitches[0] / sizeof (uint32_t)];
            for (int j = 0; j < WIDTH; j++)
            {
              convertPoint (vertices_row[j], cloud_row[j]);
              memcpy (&cloud_row[j].rgba, &color_row[j], sizeof (uint32_t));
            }
          }
        }
        mapped->ReleaseAccess (&data);
        mapped->Release ();
      }

      if (need_xyzrgba_)
        point_cloud_rgba_signal_->operator () (xyzrgba_cloud);
      if (need_xyz_)
        point_cloud_signal_->operator () (xyz_cloud);
      break;
    }
    case PXC_STATUS_DEVICE_LOST:
      THROW_IO_EXCEPTION ("failed to read data stream from PXC device: device lost");
    case PXC_STATUS_ALLOC_FAILED:
      THROW_IO_EXCEPTION ("failed to read data stream from PXC device: alloc failed");
    }
    sample.ReleaseImages ();
  }
  projection->Release ();
  RealSenseDevice::reset (device_);
}