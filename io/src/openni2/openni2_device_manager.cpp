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

#include "pcl/io/openni2/openni2_device_manager.h"
#include "pcl/io/openni2/openni2_convert.h"
#include "pcl/io/openni2/openni2_device.h"
#include "pcl/io/io_exception.h"

#include <boost/make_shared.hpp>

#include <set>
#include <string>

#include "OpenNI.h"

namespace pcl
{
  namespace io
  {
    namespace openni2
    {

      class OpenNI2DeviceInfoComparator
      {
      public:
        bool operator ()(const OpenNI2DeviceInfo& di1, const OpenNI2DeviceInfo& di2) const
        {
          return (di1.uri_.compare (di2.uri_) < 0);
        }
      };

      typedef std::set<OpenNI2DeviceInfo, OpenNI2DeviceInfoComparator> DeviceSet;



      class OpenNI2DeviceListener : public openni::OpenNI::DeviceConnectedListener,
        public openni::OpenNI::DeviceDisconnectedListener,
        public openni::OpenNI::DeviceStateChangedListener
      {
        public:
          OpenNI2DeviceListener ()
            : openni::OpenNI::DeviceConnectedListener ()
            , openni::OpenNI::DeviceDisconnectedListener ()
            , openni::OpenNI::DeviceStateChangedListener ()
          {
            openni::OpenNI::addDeviceConnectedListener (this);
            openni::OpenNI::addDeviceDisconnectedListener (this);
            openni::OpenNI::addDeviceStateChangedListener (this);

            // get list of currently connected devices
            openni::Array<openni::DeviceInfo> device_info_list;
            openni::OpenNI::enumerateDevices (&device_info_list);

            for (int i = 0; i < device_info_list.getSize (); ++i)
            {
              onDeviceConnected (&device_info_list[i]);
            }
          }

          ~OpenNI2DeviceListener ()
          {
            openni::OpenNI::removeDeviceConnectedListener (this);
            openni::OpenNI::removeDeviceDisconnectedListener (this);
            openni::OpenNI::removeDeviceStateChangedListener (this);
          }

          virtual void
          onDeviceStateChanged (const openni::DeviceInfo* pInfo, openni::DeviceState state)
          {
            switch (state)
            {
            case openni::DEVICE_STATE_OK:
              onDeviceConnected (pInfo);
              break;
            case openni::DEVICE_STATE_ERROR:
            case openni::DEVICE_STATE_NOT_READY:
            case openni::DEVICE_STATE_EOF:
            default:
              onDeviceDisconnected (pInfo);
              break;
            }
          }

          virtual void
          onDeviceConnected (const openni::DeviceInfo* pInfo)
          {
            boost::mutex::scoped_lock l (device_mutex_);

            const OpenNI2DeviceInfo device_info_wrapped = openni2_convert (pInfo);

            // make sure it does not exist in set before inserting
            device_set_.erase (device_info_wrapped);
            device_set_.insert (device_info_wrapped);
          }

          virtual void
          onDeviceDisconnected (const openni::DeviceInfo* pInfo)
          {
            boost::mutex::scoped_lock l (device_mutex_);

            const OpenNI2DeviceInfo device_info_wrapped = openni2_convert (pInfo);
            device_set_.erase (device_info_wrapped);
          }

          boost::shared_ptr<std::vector<std::string> >
          getConnectedDeviceURIs ()
          {
            boost::mutex::scoped_lock l (device_mutex_);

            boost::shared_ptr<std::vector<std::string> > result = boost::make_shared<std::vector<std::string> >();

            result->reserve (device_set_.size ());

            std::set<OpenNI2DeviceInfo, OpenNI2DeviceInfoComparator>::const_iterator it;
            std::set<OpenNI2DeviceInfo, OpenNI2DeviceInfoComparator>::const_iterator it_end = device_set_.end ();

            for (it = device_set_.begin (); it != it_end; ++it)
              result->push_back (it->uri_);

            return result;
          }

          boost::shared_ptr<std::vector<OpenNI2DeviceInfo> >
          getConnectedDeviceInfos ()
          {
            boost::mutex::scoped_lock l (device_mutex_);

            boost::shared_ptr<std::vector<OpenNI2DeviceInfo> > result = boost::make_shared<std::vector<OpenNI2DeviceInfo> >();

            result->reserve (device_set_.size ());

            DeviceSet::const_iterator it;
            DeviceSet::const_iterator it_end = device_set_.end ();

            for (it = device_set_.begin (); it != it_end; ++it)
              result->push_back (*it);

            return result;
          }

          std::size_t
          getNumOfConnectedDevices ()
          {
            boost::mutex::scoped_lock l (device_mutex_);

            return device_set_.size ();
          }

          boost::mutex device_mutex_;
          DeviceSet device_set_;
      };

    } //namespace
  }
}


//////////////////////////////////////////////////////////////////////////
using pcl::io::openni2::OpenNI2Device;
using pcl::io::openni2::OpenNI2DeviceInfo;
using pcl::io::openni2::OpenNI2DeviceManager;

pcl::io::openni2::OpenNI2DeviceManager::OpenNI2DeviceManager ()
{
  openni::Status rc = openni::OpenNI::initialize ();
  if (rc != openni::STATUS_OK)
    THROW_IO_EXCEPTION ("Initialize failed\n%s\n", openni::OpenNI::getExtendedError ());

  device_listener_ = boost::make_shared<OpenNI2DeviceListener>();
}

pcl::io::openni2::OpenNI2DeviceManager::~OpenNI2DeviceManager ()
{
}

boost::shared_ptr<std::vector<OpenNI2DeviceInfo> >
pcl::io::openni2::OpenNI2DeviceManager::getConnectedDeviceInfos () const
{
  return device_listener_->getConnectedDeviceInfos ();
}

boost::shared_ptr<std::vector<std::string> >
pcl::io::openni2::OpenNI2DeviceManager::getConnectedDeviceURIs () const
{
  return device_listener_->getConnectedDeviceURIs ();
}

std::size_t
pcl::io::openni2::OpenNI2DeviceManager::getNumOfConnectedDevices () const
{
  return device_listener_->getNumOfConnectedDevices ();
}

boost::shared_ptr<OpenNI2Device>
pcl::io::openni2::OpenNI2DeviceManager::getAnyDevice ()
{
  return boost::make_shared<OpenNI2Device>("");
}

boost::shared_ptr<OpenNI2Device>
pcl::io::openni2::OpenNI2DeviceManager::getDevice (const std::string& device_URI)
{
  return boost::make_shared<OpenNI2Device>(device_URI);
}

boost::shared_ptr<OpenNI2Device>
pcl::io::openni2::OpenNI2DeviceManager::getDeviceByIndex (int index)
{
  boost::shared_ptr<std::vector<std::string> > URIs = getConnectedDeviceURIs ();
  return boost::make_shared<OpenNI2Device>( URIs->at (index) );
}

boost::shared_ptr<OpenNI2Device>
pcl::io::openni2::OpenNI2DeviceManager::getFileDevice (const std::string& path)
{
  return boost::make_shared<OpenNI2Device>(path);
}

std::ostream&
operator<< (std::ostream& stream, const OpenNI2DeviceManager& device_manager) 
{

  boost::shared_ptr<std::vector<OpenNI2DeviceInfo> > device_info = device_manager.getConnectedDeviceInfos ();

  std::vector<OpenNI2DeviceInfo>::const_iterator it;
  std::vector<OpenNI2DeviceInfo>::const_iterator it_end = device_info->end ();

  for (it = device_info->begin (); it != it_end; ++it)
  {
    stream << "Uri: " << it->uri_ << " (Vendor: " << it->vendor_ <<
      ", Name: " << it->name_ <<
      ", Vendor ID: " << it->vendor_id_ <<
      ", Product ID: " << it->product_id_ <<
      ")" << std::endl;
  }

  return (stream);
}
