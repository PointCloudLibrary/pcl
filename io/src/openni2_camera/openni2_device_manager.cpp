/*
 * Copyright (c) 2013, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *      Author: Julius Kammerl (jkammerl@willowgarage.com)
 */

#include "openni2_camera/openni2_device_manager.h"
#include "openni2_camera/openni2_convert.h"
#include "openni2_camera/openni2_device.h"
#include "openni2_camera/openni2_exception.h"

#include <boost/make_shared.hpp>

#include <ros/ros.h>

#include <set>
#include <string>

#include "OpenNI.h"

namespace openni2_wrapper
{

class OpenNI2DeviceInfoComparator
{
public:
  bool operator()(const OpenNI2DeviceInfo& di1, const OpenNI2DeviceInfo& di2)
  {
    return (di1.uri_.compare(di2.uri_) < 0);
  }
};

typedef std::set<OpenNI2DeviceInfo, OpenNI2DeviceInfoComparator> DeviceSet;

class OpenNI2DeviceListener : public openni::OpenNI::DeviceConnectedListener,
                             public openni::OpenNI::DeviceDisconnectedListener,
                             public openni::OpenNI::DeviceStateChangedListener
{
public:
  OpenNI2DeviceListener() :
      openni::OpenNI::DeviceConnectedListener(),
      openni::OpenNI::DeviceDisconnectedListener(),
      openni::OpenNI::DeviceStateChangedListener()
  {
    openni::OpenNI::addDeviceConnectedListener(this);
    openni::OpenNI::addDeviceDisconnectedListener(this);
    openni::OpenNI::addDeviceStateChangedListener(this);

    // get list of currently connected devices
    openni::Array<openni::DeviceInfo> device_info_list;
    openni::OpenNI::enumerateDevices(&device_info_list);

    for (int i = 0; i < device_info_list.getSize(); ++i)
    {
      onDeviceConnected(&device_info_list[i]);
    }
  }

  ~OpenNI2DeviceListener()
  {
    openni::OpenNI::removeDeviceConnectedListener(this);
    openni::OpenNI::removeDeviceDisconnectedListener(this);
    openni::OpenNI::removeDeviceStateChangedListener(this);
  }

  virtual void onDeviceStateChanged(const openni::DeviceInfo* pInfo, openni::DeviceState state)
  {
    ROS_INFO("Device \"%s\" error state changed to %d\n", pInfo->getUri(), state);

    switch (state)
    {
      case openni::DEVICE_STATE_OK:
        onDeviceConnected(pInfo);
        break;
      case openni::DEVICE_STATE_ERROR:
      case openni::DEVICE_STATE_NOT_READY:
      case openni::DEVICE_STATE_EOF:
      default:
        onDeviceDisconnected(pInfo);
        break;
    }
  }

  virtual void onDeviceConnected(const openni::DeviceInfo* pInfo)
  {
    boost::mutex::scoped_lock l(device_mutex_);

    ROS_INFO("Device \"%s\" connected\n", pInfo->getUri());

    const OpenNI2DeviceInfo device_info_wrapped = openni2_convert(pInfo);

    // make sure it does not exist in set before inserting
    device_set_.erase(device_info_wrapped);
    device_set_.insert(device_info_wrapped);
  }

  virtual void onDeviceDisconnected(const openni::DeviceInfo* pInfo)
  {
    boost::mutex::scoped_lock l(device_mutex_);

    ROS_WARN("Device \"%s\" disconnected\n", pInfo->getUri());

    const OpenNI2DeviceInfo device_info_wrapped = openni2_convert(pInfo);
    device_set_.erase(device_info_wrapped);
  }

  boost::shared_ptr<std::vector<std::string> > getConnectedDeviceURIs()
  {
    boost::mutex::scoped_lock l(device_mutex_);

    boost::shared_ptr<std::vector<std::string> > result = boost::make_shared<std::vector<std::string> >();

    result->reserve(device_set_.size());

    std::set<OpenNI2DeviceInfo, OpenNI2DeviceInfoComparator>::const_iterator it;
    std::set<OpenNI2DeviceInfo, OpenNI2DeviceInfoComparator>::const_iterator it_end = device_set_.end();

    for (it = device_set_.begin(); it != it_end; ++it)
      result->push_back(it->uri_);

    return result;
  }

  boost::shared_ptr<std::vector<OpenNI2DeviceInfo> > getConnectedDeviceInfos()
  {
    boost::mutex::scoped_lock l(device_mutex_);

    boost::shared_ptr<std::vector<OpenNI2DeviceInfo> > result = boost::make_shared<std::vector<OpenNI2DeviceInfo> >();

    result->reserve(device_set_.size());

    DeviceSet::const_iterator it;
    DeviceSet::const_iterator it_end = device_set_.end();

    for (it = device_set_.begin(); it != it_end; ++it)
      result->push_back(*it);

    return result;
  }

  std::size_t getNumOfConnectedDevices()
  {
    boost::mutex::scoped_lock l(device_mutex_);

    return device_set_.size();
  }

  boost::mutex device_mutex_;
  DeviceSet device_set_;
};

//////////////////////////////////////////////////////////////////////////

boost::shared_ptr<OpenNI2DeviceManager> OpenNI2DeviceManager::singelton_;

OpenNI2DeviceManager::OpenNI2DeviceManager()
{
  openni::Status rc = openni::OpenNI::initialize();
  if (rc != openni::STATUS_OK)
      THROW_OPENNI_EXCEPTION("Initialize failed\n%s\n", openni::OpenNI::getExtendedError());

  device_listener_ = boost::make_shared<OpenNI2DeviceListener>();
}

OpenNI2DeviceManager::~OpenNI2DeviceManager()
{
}

boost::shared_ptr<OpenNI2DeviceManager> OpenNI2DeviceManager::getSingelton()
{
  if (singelton_.get()==0)
    singelton_ = boost::make_shared<OpenNI2DeviceManager>();

  return singelton_;
}

boost::shared_ptr<std::vector<OpenNI2DeviceInfo> > OpenNI2DeviceManager::getConnectedDeviceInfos() const
{
return device_listener_->getConnectedDeviceInfos();
}

boost::shared_ptr<std::vector<std::string> > OpenNI2DeviceManager::getConnectedDeviceURIs() const
{
  return device_listener_->getConnectedDeviceURIs();
}

std::size_t OpenNI2DeviceManager::getNumOfConnectedDevices() const
{
  return device_listener_->getNumOfConnectedDevices();
}

boost::shared_ptr<OpenNI2Device> OpenNI2DeviceManager::getAnyDevice()
{
  return boost::make_shared<OpenNI2Device>("");
}
boost::shared_ptr<OpenNI2Device> OpenNI2DeviceManager::getDevice(const std::string& device_URI)
{
  return boost::make_shared<OpenNI2Device>(device_URI);
}


std::ostream& operator << (std::ostream& stream, const OpenNI2DeviceManager& device_manager) {

  boost::shared_ptr<std::vector<OpenNI2DeviceInfo> > device_info = device_manager.getConnectedDeviceInfos();

  std::vector<OpenNI2DeviceInfo>::const_iterator it;
  std::vector<OpenNI2DeviceInfo>::const_iterator it_end = device_info->end();

  for (it = device_info->begin(); it != it_end; ++it)
  {
    stream << "Uri: " << it->uri_ << " (Vendor: " << it->vendor_ <<
                                     ", Name: " << it->name_ <<
                                     ", Vendor ID: " << it->vendor_id_ <<
                                     ", Product ID: " << it->product_id_ <<
                                      ")" << std::endl;
  }
}


} //namespace openni2_wrapper
