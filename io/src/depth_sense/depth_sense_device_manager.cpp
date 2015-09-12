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

#include <pcl/io/io_exception.h>

#include <pcl/io/depth_sense/depth_sense_grabber_impl.h>
#include <pcl/io/depth_sense/depth_sense_device_manager.h>

boost::mutex pcl::io::depth_sense::DepthSenseDeviceManager::mutex_;

pcl::io::depth_sense::DepthSenseDeviceManager::DepthSenseDeviceManager ()
{
  try
  {
    context_ = DepthSense::Context::create ("localhost");
    depth_sense_thread_ = boost::thread (&DepthSense::Context::run, &context_);
  }
  catch (...)
  {
    THROW_IO_EXCEPTION ("failed to initialize DepthSense context");
  }
}

pcl::io::depth_sense::DepthSenseDeviceManager::~DepthSenseDeviceManager ()
{
  try
  {
    context_.quit();
    depth_sense_thread_.join();
  }
  catch (DepthSense::InvalidOperationException& e)
  {
    // do nothing
  }
}

std::string
pcl::io::depth_sense::DepthSenseDeviceManager::captureDevice (DepthSenseGrabberImpl* grabber)
{
  boost::mutex::scoped_lock lock (mutex_);
  std::vector<DepthSense::Device> devices = context_.getDevices ();
  if (devices.size () == 0)
    THROW_IO_EXCEPTION ("no connected devices");
  for (size_t i = 0; i < devices.size (); ++i)
    if (!isCaptured (devices[i].getSerialNumber ()))
      return (captureDevice (grabber, devices[i]));
  THROW_IO_EXCEPTION ("all connected devices are captured by other grabbers");
  return ("");  // never reached, needed just to silence -Wreturn-type warning
}

std::string
pcl::io::depth_sense::DepthSenseDeviceManager::captureDevice (DepthSenseGrabberImpl* grabber, size_t index)
{
  boost::mutex::scoped_lock lock (mutex_);
  if (index >= context_.getDevices ().size ())
    THROW_IO_EXCEPTION ("device with index %i is not connected", index + 1);
  if (isCaptured (context_.getDevices ().at (index).getSerialNumber ()))
    THROW_IO_EXCEPTION ("device with index %i is captured by another grabber", index);
  return (captureDevice (grabber, context_.getDevices ().at (index)));
}

std::string
pcl::io::depth_sense::DepthSenseDeviceManager::captureDevice (DepthSenseGrabberImpl* grabber, const std::string& sn)
{
  boost::mutex::scoped_lock lock (mutex_);
  std::vector<DepthSense::Device> devices = context_.getDevices ();
  for (size_t i = 0; i < devices.size (); ++i)
  {
    if (devices[i].getSerialNumber () == sn)
    {
      if (isCaptured (sn))
        THROW_IO_EXCEPTION ("device with serial number %s is captured by another grabber", sn.c_str ());
      return (captureDevice (grabber, devices[i]));
    }
  }
  THROW_IO_EXCEPTION ("device with serial number %s is not connected", sn.c_str ());
  return ("");  // never reached, needed just to silence -Wreturn-type warning
}

void
pcl::io::depth_sense::DepthSenseDeviceManager::reconfigureDevice (const std::string& sn)
{
  boost::mutex::scoped_lock lock (mutex_);
  const CapturedDevice& dev = captured_devices_[sn];
  context_.requestControl (dev.depth_node, 0);
  dev.grabber->configureDepthNode (dev.depth_node);
  context_.releaseControl (dev.depth_node);
  context_.requestControl (dev.color_node, 0);
  dev.grabber->configureColorNode (dev.color_node);
  context_.releaseControl (dev.color_node);
}

void
pcl::io::depth_sense::DepthSenseDeviceManager::startDevice (const std::string& sn)
{
  const CapturedDevice& dev = captured_devices_[sn];
  try
  {
    context_.registerNode (dev.depth_node);
    context_.registerNode (dev.color_node);
    context_.startNodes ();
  }
  catch (DepthSense::ArgumentException& e)
  {
    THROW_IO_EXCEPTION ("unable to start device %s, possibly disconnected", sn.c_str ());
  }
}

void
pcl::io::depth_sense::DepthSenseDeviceManager::stopDevice (const std::string& sn)
{
  const CapturedDevice& dev = captured_devices_[sn];
  try
  {
    context_.unregisterNode (dev.depth_node);
    context_.unregisterNode (dev.color_node);
    if (context_.getRegisteredNodes ().size () == 0)
      context_.stopNodes ();
  }
  catch (DepthSense::ArgumentException& e)
  {
    THROW_IO_EXCEPTION ("unable to stop device %s, possibly disconnected", sn.c_str ());
  }
}

void
pcl::io::depth_sense::DepthSenseDeviceManager::releaseDevice (const std::string& sn)
{
  boost::mutex::scoped_lock lock (mutex_);
  const CapturedDevice& dev = captured_devices_[sn];
  dev.depth_node.newSampleReceivedEvent ().disconnect (dev.grabber, &DepthSenseGrabberImpl::onDepthDataReceived);
  dev.color_node.newSampleReceivedEvent ().disconnect (dev.grabber, &DepthSenseGrabberImpl::onColorDataReceived);
  captured_devices_.erase (sn);
}

std::string
pcl::io::depth_sense::DepthSenseDeviceManager::captureDevice (DepthSenseGrabberImpl* grabber, DepthSense::Device device)
{
  // This is called from public captureDevice() functions and should already be
  // under scoped lock
  CapturedDevice dev;
  dev.grabber = grabber;
  std::vector<DepthSense::Node> nodes = device.getNodes ();
  for (size_t i = 0; i < nodes.size (); ++i)
  {
    if (nodes[i].is<DepthSense::DepthNode> ())
    {
      dev.depth_node = nodes[i].as<DepthSense::DepthNode> ();
      dev.depth_node.newSampleReceivedEvent ().connect (grabber, &DepthSenseGrabberImpl::onDepthDataReceived);
      grabber->setCameraParameters (device.getStereoCameraParameters ());
    }
    if (nodes[i].is<DepthSense::ColorNode> ())
    {
      dev.color_node = nodes[i].as<DepthSense::ColorNode> ();
      dev.color_node.newSampleReceivedEvent ().connect (grabber, &DepthSenseGrabberImpl::onColorDataReceived);
    }
  }
  captured_devices_.insert (std::make_pair (device.getSerialNumber (), dev));
  return (device.getSerialNumber ());
}

