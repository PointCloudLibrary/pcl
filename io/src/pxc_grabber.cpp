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
 */

#include <pcl/pcl_config.h>
#define HAVE_PXCAPI
#ifdef HAVE_PXCAPI

#include <pcl/io/pxc_grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>
#include <pcl/common/io.h>
#include <pcl/console/print.h>
#include <pcl/io/boost.h>
#include <pcl/exceptions.h>

#include <boost/thread.hpp>

#include <iostream>
#include <queue>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
pcl::PXCGrabber::PXCGrabber ()
  : pp_ ()
  , running_ (false)
  , fps_ (0.0f)
{
  point_cloud_signal_      = createSignal<sig_cb_pxc_point_cloud> ();
  point_cloud_rgb_signal_  = createSignal<sig_cb_pxc_point_cloud_rgb> ();
  point_cloud_rgba_signal_ = createSignal<sig_cb_pxc_point_cloud_rgba> ();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
pcl::PXCGrabber::~PXCGrabber () throw ()
{
  stop ();

  disconnect_all_slots<sig_cb_pxc_point_cloud> ();
  disconnect_all_slots<sig_cb_pxc_point_cloud_rgb> ();
  disconnect_all_slots<sig_cb_pxc_point_cloud_rgba> ();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::PXCGrabber::init ()
{
  // enable rgb and 3d data and initialize
  pp_.EnableImage (PXCImage::COLOR_FORMAT_RGB32);
  pp_.EnableImage (PXCImage::COLOR_FORMAT_VERTICES);
  pp_.Init ();

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::PXCGrabber::close ()
{
  pp_.Close ();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::PXCGrabber::start ()
{
  init ();
  running_ = true;

  grabber_thread_ = boost::thread (&pcl::PXCGrabber::processGrabbing, this); 
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::PXCGrabber::stop ()
{
  running_ = false;
  grabber_thread_.join ();

  close ();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::PXCGrabber::isRunning () const
{
  return (running_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::string
pcl::PXCGrabber::getName () const
{
  return (std::string ("PXCGrabber"));
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float
pcl::PXCGrabber::getFramesPerSecond () const
{
  fps_mutex_.lock ();
  float fps = fps_;
  fps_mutex_.unlock ();

  return (fps);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::PXCGrabber::processGrabbing ()
{
  UtilCapture * cap = pp_.QueryCapture ();
  PXCCapture * capture = cap->QueryCapture ();
  PXCSession * session = pp_.QuerySession ();

  // special depth values for saturated and low-confidence pixels
  pxcF32 dvalues[2];

  // projection serializable identifier
  pxcUID prj_value;
  PXCSmartPtr<PXCProjection> projection;

  // setup the projection info for getting a nice depth map
  pxcStatus sts = cap->QueryDevice ()->QueryPropertyAsUID (PXCCapture::Device::PROPERTY_PROJECTION_SERIALIZABLE, &prj_value);
  if (sts >= PXC_STATUS_NO_ERROR) 
  {
    // create properties for checking if depth values are bad (by low confidence and by saturation)
    cap->QueryDevice ()->QueryProperty (PXCCapture::Device::PROPERTY_DEPTH_LOW_CONFIDENCE_VALUE, &dvalues[0]);
    cap->QueryDevice ()->QueryProperty (PXCCapture::Device::PROPERTY_DEPTH_SATURATION_VALUE, &dvalues[1]);

    session->DynamicCast<PXCMetadata> ()->CreateSerializable<PXCProjection> (prj_value, &projection);
  }

  pcl::StopWatch stop_watch;
  std::queue<double> capture_time_queue;
  double total_time = 0.0f;

  stop_watch.reset ();

  bool continue_grabbing = true;
  while (continue_grabbing) 
  {
    if (!pp_.AcquireFrame (true))
    {
      boost::this_thread::sleep (boost::posix_time::milliseconds (1));
      continue;
    }

    // query rgb and 3d data
    PXCImage *color_image = pp_.QueryImage (PXCImage::IMAGE_TYPE_COLOR);
    PXCImage *vertex_image = pp_.QueryImage (PXCImage::IMAGE_TYPE_DEPTH);

    // acquiure access to data
    PXCImage::ImageData ddata;
    vertex_image->AcquireAccess (PXCImage::ACCESS_READ, &ddata); 
    short * vertices = (short*) ddata.planes[0];

    PXCImage::ImageData idata;
    color_image->AcquireAccess (PXCImage::ACCESS_READ, &idata);
    unsigned char * rgb_image_data = (unsigned char*) idata.planes[0];
    
    // get image information
    PXCImage::ImageInfo vertImageInfo;
    pxcStatus stat = vertex_image->QueryInfo (&vertImageInfo);
    const int w = vertImageInfo.width;
    const int h = vertImageInfo.height;

    PXCImage::ImageInfo rgb_image_info;
    stat = color_image->QueryInfo (&rgb_image_info);
    const int image_width = rgb_image_info.width;
    const int image_height = rgb_image_info.height;

    // convert to point cloud
    const float nan_value = std::numeric_limits<float>::quiet_NaN ();

    int dwidth2 = ddata.pitches[0] / sizeof (pxcU16);
    float *uvmap = (float*)ddata.planes[2];
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA> (w, h));
    for (int j = 0, k = 0; j < h; j++)
    {
      for (int i = 0; i < w; i++, k++)
      {
        int xx = (int) (uvmap[(j*w+i)*2+0]*image_width+0.5f);
        int yy = (int) (uvmap[(j*w+i)*2+1]*image_height+0.5f);

        unsigned int idx = 3*(j*w + i);
        short x = vertices[idx];
        short y = vertices[idx+1];
        short z = vertices[idx+2];

        unsigned char r = 255;
        unsigned char g = 255;
        unsigned char b = 255;

        if (xx < 0 || xx >= image_width || yy < 0 || yy >= image_height)
        {
          r = 255;
          g = 255;
          b = 255;
        }
        else
        {
          unsigned char * rgb_ptr = rgb_image_data + 3*(yy*image_width+xx);
          r = rgb_ptr[2];
          g = rgb_ptr[1];
          b = rgb_ptr[0];
        }

        if (z == dvalues[0] || z == dvalues[1]) 
        {
          cloud->points[k].x = nan_value;
          cloud->points[k].y = nan_value;
          cloud->points[k].z = nan_value;
          cloud->points[k].r = r;
          cloud->points[k].g = g;
          cloud->points[k].b = b;
          cloud->points[k].a = 255;
        }
        else
        {
          // normalize vertices to meters
          cloud->points[k].x = x / 1000.0f;
          cloud->points[k].y = y / 1000.0f;
          cloud->points[k].z = z / 1000.0f;
          cloud->points[k].r = r;
          cloud->points[k].g = g;
          cloud->points[k].b = b;
          cloud->points[k].a = 255;
        }
      }
    }

    // publish cloud
    if (num_slots<sig_cb_pxc_point_cloud_rgba> () > 0)
    {
      point_cloud_rgba_signal_->operator() (cloud);
    }
    if (num_slots<sig_cb_pxc_point_cloud_rgb> () > 0)
    {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
      cloud->resize (w*h);
      cloud->width = w;
      cloud->height = h;
      cloud->is_dense = false;

      pcl::copyPointCloud<pcl::PointXYZRGBA, pcl::PointXYZRGB> (*cloud, *tmp_cloud);

      point_cloud_rgb_signal_->operator() (tmp_cloud);
    }
    if (num_slots<sig_cb_pxc_point_cloud> () > 0)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tmp (new pcl::PointCloud<pcl::PointXYZ> ());
      cloud->resize (w*h);
      cloud->width = w;
      cloud->height = h;
      cloud->is_dense = false;

      pcl::copyPointCloud<pcl::PointXYZRGBA, pcl::PointXYZ> (*cloud, *cloud_tmp);

      point_cloud_signal_->operator() (cloud_tmp);
    }
    
    vertex_image->ReleaseAccess (&ddata);
    color_image->ReleaseAccess (&idata);
    
    pp_.ReleaseFrame ();

    const double capture_time = stop_watch.getTimeSeconds ();
    total_time += capture_time;

    capture_time_queue.push (capture_time);

    if (capture_time_queue.size () >= 30)
    {
      double removed_time = capture_time_queue.front ();
      capture_time_queue.pop ();

      total_time -= removed_time;
    }

    fps_mutex_.lock ();
    fps_ = static_cast<float> (total_time / capture_time_queue.size ());
    fps_mutex_.unlock ();
  }
}

#endif
