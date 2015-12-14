/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception, Inc.
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

#include <pcl/common/io.h>
#include <pcl/io/depth_sense_grabber.h>
#include <pcl/io/depth_sense/depth_sense_grabber_impl.h>
#include <pcl/io/depth_sense/depth_sense_device_manager.h>

pcl::io::depth_sense::DepthSenseGrabberImpl::DepthSenseGrabberImpl (DepthSenseGrabber* parent, const std::string& device_id)
: p_ (parent)
, is_running_ (false)
, confidence_threshold_ (50)
, temporal_filtering_type_ (DepthSenseGrabber::DepthSense_None)
, color_data_ (COLOR_SIZE * 3)
, depth_buffer_ (new pcl::io::SingleBuffer<float> (SIZE))
{
  if (device_id == "")
    device_id_ = DepthSenseDeviceManager::getInstance ()->captureDevice (this);
  else if (device_id[0] == '#')
    device_id_ = DepthSenseDeviceManager::getInstance ()->captureDevice (this, boost::lexical_cast<int> (device_id.substr (1)) - 1);
  else
    device_id_ = DepthSenseDeviceManager::getInstance ()->captureDevice (this, device_id);

  point_cloud_signal_ = p_->createSignal<sig_cb_depth_sense_point_cloud> ();
  point_cloud_rgba_signal_ = p_->createSignal<sig_cb_depth_sense_point_cloud_rgba> ();
}

pcl::io::depth_sense::DepthSenseGrabberImpl::~DepthSenseGrabberImpl () throw ()
{
  stop ();

  DepthSenseDeviceManager::getInstance ()->releaseDevice (device_id_);

  p_->disconnect_all_slots<sig_cb_depth_sense_point_cloud> ();
  p_->disconnect_all_slots<sig_cb_depth_sense_point_cloud_rgba> ();
}


void
pcl::io::depth_sense::DepthSenseGrabberImpl::start ()
{
  need_xyz_ = p_->num_slots<sig_cb_depth_sense_point_cloud> () > 0;
  need_xyzrgba_ = p_->num_slots<sig_cb_depth_sense_point_cloud_rgba> () > 0;

  if (!is_running_)
  {
    DepthSenseDeviceManager::getInstance ()->reconfigureDevice (device_id_);
    DepthSenseDeviceManager::getInstance ()->startDevice (device_id_);
    frequency_.reset ();
    is_running_ = true;
  }
}

void
pcl::io::depth_sense::DepthSenseGrabberImpl::stop ()
{
  if (is_running_)
  {
    DepthSenseDeviceManager::getInstance ()->stopDevice (device_id_);
    is_running_ = false;
  }
}

float
pcl::io::depth_sense::DepthSenseGrabberImpl::getFramesPerSecond () const
{
  boost::mutex::scoped_lock lock (fps_mutex_);
  return (frequency_.getFrequency ());
}

void
pcl::io::depth_sense::DepthSenseGrabberImpl::setConfidenceThreshold (int threshold)
{
  confidence_threshold_ = threshold;
  DepthSenseDeviceManager::getInstance ()->reconfigureDevice (device_id_);
}

void
pcl::io::depth_sense::DepthSenseGrabberImpl::enableTemporalFiltering (DepthSenseGrabber::TemporalFilteringType type, size_t window_size)
{
  if (temporal_filtering_type_ != type ||
      (type != DepthSenseGrabber::DepthSense_None && depth_buffer_->size () != window_size))
  {
    bool was_running = is_running_;
    if (was_running)
      stop ();
    switch (type)
    {
      case DepthSenseGrabber::DepthSense_None:
        {
          depth_buffer_.reset (new pcl::io::SingleBuffer<float> (SIZE));
          break;
        }
      case DepthSenseGrabber::DepthSense_Median:
        {
          depth_buffer_.reset (new pcl::io::MedianBuffer<float> (SIZE, window_size));
          break;
        }
      case DepthSenseGrabber::DepthSense_Average:
        {
          depth_buffer_.reset (new pcl::io::AverageBuffer<float> (SIZE, window_size));
          break;
        }
    }
    temporal_filtering_type_ = type;
    if (was_running)
      start ();
  }
}

void
pcl::io::depth_sense::DepthSenseGrabberImpl::setCameraParameters (const DepthSense::StereoCameraParameters& parameters)
{
  projection_.reset (new DepthSense::ProjectionHelper (parameters));
}

void
pcl::io::depth_sense::DepthSenseGrabberImpl::configureDepthNode (DepthSense::DepthNode node) const
{
  DepthSense::DepthNode::Configuration config = node.getConfiguration ();
  config.frameFormat = DepthSense::FRAME_FORMAT_QVGA;
  config.framerate = FRAMERATE;
  config.mode = DepthSense::DepthNode::CAMERA_MODE_CLOSE_MODE;
  config.saturation = false;
  node.setEnableDepthMapFloatingPoint (true);
  node.setEnableUvMap (true);
  node.setEnableConfidenceMap (true);
  node.setConfiguration (config);
}

void
pcl::io::depth_sense::DepthSenseGrabberImpl::configureColorNode (DepthSense::ColorNode node) const
{
  DepthSense::ColorNode::Configuration config = node.getConfiguration ();
  config.frameFormat = DepthSense::FRAME_FORMAT_VGA;
  config.compression = DepthSense::COMPRESSION_TYPE_MJPEG;
  config.powerLineFrequency = DepthSense::POWER_LINE_FREQUENCY_50HZ;
  config.framerate = FRAMERATE;
  node.setEnableColorMap (true);
  node.setConfiguration (config);
}

void
pcl::io::depth_sense::DepthSenseGrabberImpl::onDepthDataReceived (DepthSense::DepthNode, DepthSense::DepthNode::NewSampleReceivedData data)
{
  fps_mutex_.lock ();
  frequency_.event ();
  fps_mutex_.unlock ();

  static const float nan = std::numeric_limits<float>::quiet_NaN ();

  std::vector<float> depth_data (SIZE);
  memcpy (depth_data.data (), &data.depthMapFloatingPoint[0], SIZE * sizeof (float));
  for (int i = 0; i < SIZE; i++)
    if (data.confidenceMap[i] < confidence_threshold_)
      depth_data[i] = nan;
  depth_buffer_->push (depth_data);

  pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr xyzrgba_cloud;

  if (need_xyz_)
  {
    xyz_cloud.reset (new pcl::PointCloud<pcl::PointXYZ> (WIDTH, HEIGHT));
    xyz_cloud->is_dense = false;

    computeXYZ (*xyz_cloud);

    point_cloud_signal_->operator () (xyz_cloud);
  }

  if (need_xyzrgba_)
  {
    xyzrgba_cloud.reset (new pcl::PointCloud<pcl::PointXYZRGBA> (WIDTH, HEIGHT));
    xyzrgba_cloud->is_dense = false;

    if (need_xyz_)
      copyPointCloud (*xyz_cloud, *xyzrgba_cloud);
    else
      computeXYZ (*xyzrgba_cloud);

    for (int i = 0; i < SIZE; i++)
    {
      const DepthSense::UV& uv = data.uvMap[i];
      int row = static_cast<int> (uv.v * COLOR_HEIGHT);
      int col = static_cast<int> (uv.u * COLOR_WIDTH);
      int pixel = row * COLOR_WIDTH + col;
      if (pixel >=0 && pixel < COLOR_WIDTH * COLOR_HEIGHT)
        memcpy (&xyzrgba_cloud->points[i].rgba, &color_data_[pixel * 3], 3);
    }

    point_cloud_rgba_signal_->operator () (xyzrgba_cloud);
  }
}

void
pcl::io::depth_sense::DepthSenseGrabberImpl::onColorDataReceived (DepthSense::ColorNode, DepthSense::ColorNode::NewSampleReceivedData data)
{
  if (need_xyzrgba_)
    memcpy (&color_data_[0], data.colorMap, color_data_.size ());
}

template <typename Point> void
pcl::io::depth_sense::DepthSenseGrabberImpl::computeXYZ (PointCloud<Point>& cloud)
{
  static const float nan = std::numeric_limits<float>::quiet_NaN ();

  int i = 0;
  DepthSense::FPExtended2DPoint point (DepthSense::Point2D (0, 0), 0);
  DepthSense::FPVertex vertex;
  while (point.point.y < HEIGHT)
  {
    point.point.x = 0;
    while (point.point.x < WIDTH)
    {
      point.depth = (*depth_buffer_)[i];
      if (pcl_isnan (point.depth))
      {
        cloud.points[i].x = nan;
        cloud.points[i].y = nan;
        cloud.points[i].z = nan;
      }
      else
      {
        projection_->get3DCoordinates (&point, &vertex, 1);
        cloud.points[i].x = vertex.x;
        cloud.points[i].y = vertex.y;
        cloud.points[i].z = vertex.z;
      }
      point.point.x += 1;
      ++i;
    }
    point.point.y += 1;
  }
}

