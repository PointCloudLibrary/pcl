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

#ifndef PCL_IO_REAL_SENSE_DEVICE_MANAGER_H
#define PCL_IO_REAL_SENSE_DEVICE_MANAGER_H

#include <boost/utility.hpp>
#include <boost/shared_ptr.hpp> 
#include <boost/weak_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <vector>

#include <pcl/pcl_exports.h>

#include <librealsense/rs.hpp>

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

          typedef boost::shared_ptr<RealSenseDeviceManager> Ptr;

          static Ptr&
          getInstance ()
          {
            static Ptr instance;
            if (!instance)
            {
              boost::mutex::scoped_lock lock (mutex_);
              if (!instance)
                instance.reset (new RealSenseDeviceManager);
            }
            return (instance);
          }

          inline size_t
          getNumDevices ()
          {
            return (device_list_.size ());
          }

          boost::shared_ptr<RealSenseDevice>
          captureDevice ();

          boost::shared_ptr<RealSenseDevice>
          captureDevice (size_t index);

          boost::shared_ptr<RealSenseDevice>
          captureDevice (const std::string& sn);

          ~RealSenseDeviceManager ();

        private:

          struct DeviceInfo
          {
            int index;
            std::string serial;
            bool isCaptured;
          };

          /** If the device is already captured returns a pointer. */
          boost::shared_ptr<RealSenseDevice>
          capture (DeviceInfo& device_info);

          RealSenseDeviceManager ();

          rs::context context_;

          /** This function discovers devices that are capable of streaming
            * depth data. */
          void
          populateDeviceList ();

          std::vector<DeviceInfo> device_list_;

          static boost::mutex mutex_;
      };

      class PCL_EXPORTS RealSenseDevice : boost::noncopyable
      {
        public:

          typedef boost::shared_ptr<RealSenseDevice> Ptr;

          inline const std::string&
          getSerialNumber () { return (device_id_); }

          inline rs::device*
          getDevice () { return (device_); }

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
          rs::device* device_;

          RealSenseDevice (const std::string& id) : device_id_ (id) { };

      };
    } // namespace real_sense
  } // namespace io
} // namespace pcl

#endif /* PCL_IO_REAL_SENSE_DEVICE_MANAGER_H */