/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2015-, Open Perception, Inc.
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

#pragma once

#include <pxcsession.h>
#include <pxccapture.h>
#include <pxccapturemanager.h>

#include <pcl/memory.h>  // for pcl::shared_ptr, pcl::weak_ptr
#include <pcl/pcl_exports.h>  // for PCL_EXPORTS

#include <boost/core/noncopyable.hpp>  // for boost::noncopyable

#include <cstddef>  // for std::size_t
#include <mutex>  // for std::lock_guard, std::mutex
#include <string>  // for std::string
#include <vector>  // for std::vector

namespace pcl
{

  class RealSenseGrabber;

  namespace io
  {

    namespace real_sense
    {

      class RealSenseDevice;

      class PCL_EXPORTS RealSenseDeviceManager : boost::noncopyable
      {

        public:

          using Ptr = std::shared_ptr<RealSenseDeviceManager>;

          static Ptr&
          getInstance ()
          {
            static Ptr instance;
            if (!instance)
            {
              std::lock_guard<std::mutex> lock (mutex_);
              if (!instance)
                instance.reset (new RealSenseDeviceManager);
            }
            return (instance);
          }

          inline std::size_t
          getNumDevices ()
          {
            return (device_list_.size ());
          }

          std::shared_ptr<RealSenseDevice>
          captureDevice ();

          std::shared_ptr<RealSenseDevice>
          captureDevice (std::size_t index);

          std::shared_ptr<RealSenseDevice>
          captureDevice (const std::string& sn);

          ~RealSenseDeviceManager ();

        private:

          struct DeviceInfo
          {
            pxcUID iuid;
            pxcI32 didx;
            std::string serial;
            weak_ptr<RealSenseDevice> device_ptr;
            inline bool isCaptured () { return (!device_ptr.expired ()); }
          };

          /** If the device is already captured returns a pointer. */
          std::shared_ptr<RealSenseDevice>
          capture (DeviceInfo& device_info);

          /** This function discovers devices that are capable of streaming
            * depth data. */
          void
          populateDeviceList ();

          std::shared_ptr<PXCSession> session_;
          std::shared_ptr<PXCCaptureManager> capture_manager_;

          std::vector<DeviceInfo> device_list_;

          static std::mutex mutex_;

      };

      class PCL_EXPORTS RealSenseDevice : boost::noncopyable
      {

        public:
          using Ptr = pcl::shared_ptr<RealSenseDevice>;

          inline const std::string&
          getSerialNumber () { return (device_id_); }

          inline PXCCapture::Device&
          getPXCDevice () { return (*device_); }

          /** Reset the state of given device by releasing and capturing again. */
          static void
          reset (RealSenseDevice::Ptr& device)
          {
            std::string id = device->getSerialNumber ();
            device.reset ();
            device = RealSenseDeviceManager::getInstance ()->captureDevice (id);
          }

        private:

          friend class RealSenseDeviceManager;

          std::string device_id_;
          std::shared_ptr<PXCCapture> capture_;
          std::shared_ptr<PXCCapture::Device> device_;

          RealSenseDevice (const std::string& id) : device_id_ (id) { };

      };

    } // namespace real_sense

  } // namespace io

} // namespace pcl
