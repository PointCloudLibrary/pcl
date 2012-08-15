/*
 * Software License Agreement (BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2009-2012, Willow Garage, Inc.
 * Copyright (c) 2012-, Open Perception, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of the copyright holder(s) nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

#include <pcl/apps/in_hand_scanner/in_hand_scanner.h>

#include <pcl/io/openni_grabber.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>

////////////////////////////////////////////////////////////////////////////////

pcl::InHandScanner::InHandScanner ()
  : mutex_               (),

    p_grabber_           (new Grabber ("#1")),
    p_visualizer_        (),
    p_normal_estimation_ (new NormalEstimation ()),
    p_pass_through_      (new PassThrough ()),

    p_drawn_cloud_       (new Cloud ())
{
  // Normal estimation
  p_normal_estimation_->setNormalEstimationMethod (NormalEstimation::AVERAGE_3D_GRADIENT);
  p_normal_estimation_->setMaxDepthChangeFactor (0.02f);
  p_normal_estimation_->setNormalSmoothingSize (10.0f);

  // Pass through
}

////////////////////////////////////////////////////////////////////////////////

pcl::InHandScanner::~InHandScanner ()
{
  if (p_grabber_->isRunning ())
  {
    p_grabber_->stop ();
  }
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::InHandScanner::setVisualizer (const PCLVisualizerPtr& p_visualizer)
{
  p_visualizer_ = p_visualizer;
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::InHandScanner::start ()
{
  boost::function<void (const CloudConstPtr&)> f = boost::bind (&pcl::InHandScanner::grabbedDataCallback, this, _1);
  /*boost::signals2::connection c = */
  p_grabber_->registerCallback (f);
  p_grabber_->start ();
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::InHandScanner::draw ()
{
  if (!p_drawn_cloud_)
  {
    boost::this_thread::sleep (boost::posix_time::milliseconds (1));
    return;
  }

  CloudPtr p_temp_cloud;

  {
    boost::mutex::scoped_lock locker (mutex_);
    p_temp_cloud.swap (p_drawn_cloud_);
  }

  p_visualizer_->setBackgroundColor (1.0, 1.0, 1.0);
  if (!p_visualizer_->updatePointCloud (p_temp_cloud, "cloud"))
  {
    p_visualizer_->addPointCloud (p_temp_cloud, "cloud");
    p_visualizer_->resetCameraViewpoint ("cloud");
  }

  this->showFPS ("visualization");
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::InHandScanner::grabbedDataCallback (const CloudConstPtr& p_cloud_in)
{
  boost::mutex::scoped_lock locker (mutex_);

  // Calculate the normals
  CloudWithNormalsPtr p_cloud_with_normals (new CloudWithNormals ());
  //  p_normal_estimation_->setInputCloud (p_cloud_in);
  //  p_normal_estimation_->compute (*p_cloud_with_normals);
  pcl::copyPointCloud (*p_cloud_in, *p_cloud_with_normals); // Adding '<Point, PointWithNormal>' does not help

  //  // Pass through
  //  PointCloudWithNormalsPtr p_cloud_pass_through (new PointCloudWithNormals ());
  //  p_pass_through_->setInputCloud (p_cloud_with_normals);
  //  p_pass_through_->filter (*p_cloud_pass_through);

  // Set the cloud for visualization
  p_drawn_cloud_.reset (new Cloud ());
  pcl::copyPointCloud (*p_cloud_with_normals, *p_drawn_cloud_); // Adding '<PointWithNormal, Point>' does not help
  //pcl::copyPointCloud (*p_cloud_in, *p_drawn_cloud_); // works
  this->showFPS ("computation");
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::InHandScanner::showFPS (const std::string& what) const
{
  static unsigned int count = 0;
  static double       last  = pcl::getTime ();
  double              now   = pcl::getTime ();

  ++count;
  if ((now-last) >= 1.)
  {
    const double fps = static_cast<double> (count) / (now - last);
    std::cerr << "Average framerate (" << what << ") = " << fps << "Hz\n";

    count = 0;
    last  = now;
  }
}

////////////////////////////////////////////////////////////////////////////////
