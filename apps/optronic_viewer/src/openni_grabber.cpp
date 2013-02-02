/*
 * Software License Agreement  (BSD License)
 *
 *  Point Cloud Library  (PCL) - www.pointclouds.org
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
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES  (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <pcl/apps/optronic_viewer/openni_grabber.h>

#include <pcl/io/openni_grabber.h>


//////////////////////////////////////////////////////////////////////////////////////////////
pcl::apps::optronic_viewer::
OpenNIGrabber::
OpenNIGrabber (pcl::Grabber * grabber)
  : QThread ()
  , grabber_ (grabber)
{
  qRegisterMetaType<pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr> ("pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr");
}

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::apps::optronic_viewer::
OpenNIGrabber::
~OpenNIGrabber ()
{
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::apps::optronic_viewer::
OpenNIGrabber::
run ()
{
  std::cerr << "run grabber thread..." << std::endl;
  boost::function<void(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr & cloud) > f = 
    boost::bind (&pcl::apps::optronic_viewer::OpenNIGrabber::cloudCallback, this, _1);
  boost::signals2::connection c1 = grabber_->registerCallback (f);
  grabber_->start ();
}


//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::apps::optronic_viewer::
OpenNIGrabber::
cloudCallback (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr & cloud)
{
  //std::cerr << "[grabber thread] cloud received.." << std::endl;
  emit cloudReceived (cloud);
}


