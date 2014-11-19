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
 *  Author: Victor Lamoine (victor.lamoine@gmail.com)
 */

#include <pcl/io/ensenso_grabber.h>
#include <pcl/visualization/cloud_viewer.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::visualization::CloudViewer CloudViewer;

boost::shared_ptr<CloudViewer> viewer;
pcl::EnsensoGrabber::Ptr ensenso_grabber;

void
grabberCallback (const PointCloudT::ConstPtr& cloud)
{
  if (!viewer->wasStopped ())
    viewer->showCloud (cloud);
}

int
main (void)
{
  ensenso_grabber.reset (new pcl::EnsensoGrabber ());

  if (ensenso_grabber == 0)
    return (-1);

  ensenso_grabber->enumDevices ();
  ensenso_grabber->openTcpPort (); // default port = 24000
  ensenso_grabber->openDevice ();
  ensenso_grabber->configureCapture ();
  ensenso_grabber->setExtrinsicCalibration (); // Temporary reset calibration if it has be written in EEPROM

  boost::function<void (const PointCloudT::ConstPtr&)> f = boost::bind (&grabberCallback, _1);
  ensenso_grabber->registerCallback (f);

  viewer.reset (new CloudViewer ("3D Viewer"));
  ensenso_grabber->start ();
  std::cout << std::endl;

  while (!viewer->wasStopped ()) {
    boost::this_thread::sleep (boost::posix_time::milliseconds (1000));
    std::cout << "FPS: " << ensenso_grabber->getFramesPerSecond () << std::endl;
  }

  ensenso_grabber->stop ();
  ensenso_grabber->closeDevice ();
  return (0);
}

