/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2015-, Open Perception, Inc.
 *  Copyright (c) 2016, Intel Corporation
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
#include <pcl/io/real_sense/librealsense/real_sense_device_manager.h>

boost::mutex pcl::io::real_sense::RealSenseDeviceManager::mutex_;

using namespace pcl::io;

pcl::io::real_sense::RealSenseDeviceManager::RealSenseDeviceManager ()
{
  populateDeviceList ();
}

pcl::io::real_sense::RealSenseDeviceManager::~RealSenseDeviceManager ()
{
}

pcl::io::real_sense::RealSenseDevice::Ptr
pcl::io::real_sense::RealSenseDeviceManager::captureDevice ()
{
  boost::mutex::scoped_lock lock (mutex_);
  if (device_list_.size () == 0)
    THROW_IO_EXCEPTION ("no connected devices");
  for (size_t i = 0; i < device_list_.size (); ++i)
    if (!device_list_[i].isCaptured)
      return (capture (device_list_[i]));
  THROW_IO_EXCEPTION ("all connected devices are captured by other grabbers");
  return (RealSenseDevice::Ptr ());  // never reached, needed just to silence -Wreturn-type warning
}

pcl::io::real_sense::RealSenseDevice::Ptr
pcl::io::real_sense::RealSenseDeviceManager::captureDevice (size_t index)
{
  boost::mutex::scoped_lock lock (mutex_);
  if (index >= device_list_.size ())
    THROW_IO_EXCEPTION ("device with index %i is not connected", index + 1);
  if (device_list_[index].isCaptured)
    THROW_IO_EXCEPTION ("device with index %i is captured by another grabber", index + 1);
  return (capture (device_list_[index]));
}

pcl::io::real_sense::RealSenseDevice::Ptr
pcl::io::real_sense::RealSenseDeviceManager::captureDevice (const std::string& sn)
{
  boost::mutex::scoped_lock lock (mutex_);
  for (size_t i = 0; i < device_list_.size (); ++i)
  {
    if (device_list_[i].serial == sn)
    {
      if (device_list_[i].isCaptured)
        THROW_IO_EXCEPTION ("device with serial number %s is captured by another grabber", sn.c_str ());
      return (capture (device_list_[i]));
    }
  }
  THROW_IO_EXCEPTION ("device with serial number %s is not connected", sn.c_str ());
  return (RealSenseDevice::Ptr ());  // never reached, needed just to silence -Wreturn-type warning
}

void
pcl::io::real_sense::RealSenseDeviceManager::populateDeviceList ()
{
  device_list_.clear ();
  for (int i = 0; i < context_.get_device_count (); i++)
  {
    device_list_.push_back (DeviceInfo ());
    device_list_.back ().serial = context_.get_device (i)->get_serial ();
    device_list_.back ().index = i;
    device_list_.back ().isCaptured = false;
  }
}

pcl::io::real_sense::RealSenseDevice::Ptr
pcl::io::real_sense::RealSenseDeviceManager::capture (DeviceInfo& device_info)
{
  // This is called from public captureDevice() functions and should already be
  // under scoped lock
  RealSenseDevice::Ptr device (new RealSenseDevice (device_info.serial));
  device->device_ = context_.get_device (device_info.index);
  device_info.isCaptured = true;
  return device;
}