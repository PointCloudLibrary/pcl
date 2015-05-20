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

#include <iostream>
#include <pcl/io/davidsdk_grabber.h>
#include <pcl/visualization/cloud_viewer.h>

/** @brief Convenience typedef */
typedef pcl::visualization::CloudViewer CloudViewer;

/** @brief Convenience typedef for XYZ point clouds */
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;

/** @brief CloudViewer pointer */
boost::shared_ptr<CloudViewer> viewer_ptr;

/** @brief PCL DavidSDK object pointer */
pcl::DavidSDKGrabber::Ptr davidsdk_ptr;

/** @brief Process and/or display DavidSDK grabber clouds
 * @param[in] cloud DavidSDK cloud */
void
grabberCallback (const PointCloudXYZ::Ptr& cloud)
{
  if (!viewer_ptr->wasStopped ())
    viewer_ptr->showCloud (cloud);
}

/** @brief Main function
 * @param[in] argc
 * @param[in] argv
 * @return Exit status */
int
main (int argc,
      char *argv[])
{
  if (argc != 2)
  {
    PCL_ERROR ("Usage:\n%s 192.168.100.65\n", argv[0]);
    return (-1);
  }

  viewer_ptr.reset (new CloudViewer ("davidSDK 3D cloud viewer"));
  davidsdk_ptr.reset (new pcl::DavidSDKGrabber);
  davidsdk_ptr->connect (argv[1]);

  if (!davidsdk_ptr->isConnected ())
    return (-1);
  PCL_WARN ("davidSDK connected\n");

#ifndef _WIN32// || _WIN64
  PCL_WARN ("Linux / Mac OSX detected, setting local_path_ to /var/tmp/davidsdk/ and remote_path_ to \\\\m6700\\davidsdk\\\n");
  davidsdk_ptr->setLocalAndRemotePaths ("/var/tmp/davidsdk/", "\\\\m6700\\davidsdk\\");
#endif

  //davidsdk_ptr->setFileFormatToPLY();
  std::cout << "Using " << davidsdk_ptr->getFileFormat () << " file format" << std::endl;

  boost::function<void
  (const PointCloudXYZ::Ptr&)> f = boost::bind (&grabberCallback, _1);
  davidsdk_ptr->registerCallback (f);
  davidsdk_ptr->start ();

  while (!viewer_ptr->wasStopped ())
  {
    boost::this_thread::sleep (boost::posix_time::seconds (20));
    std::cout << "FPS: " << davidsdk_ptr->getFramesPerSecond () << std::endl;
  }

  davidsdk_ptr->stop ();
  return (0);
}

