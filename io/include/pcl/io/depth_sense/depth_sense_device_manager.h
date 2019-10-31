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

#pragma once

#include <boost/utility.hpp>

#include <pcl/pcl_exports.h>

#include <DepthSense.hxx>

#include <memory>
#include <thread>

namespace pcl
{

  namespace io
  {

    namespace depth_sense
    {

      struct DepthSenseGrabberImpl;

      /** A helper class for enumerating and managing access to DepthSense
        * devices. */
      class PCL_EXPORTS DepthSenseDeviceManager : boost::noncopyable
      {

        public:

          using Ptr = std::shared_ptr<DepthSenseDeviceManager>;

          static Ptr&
          getInstance ()
          {
            static Ptr instance;
            if (!instance)
            {
              std::lock_guard<std::mutex> lock (mutex_);
              if (!instance)
                instance.reset (new DepthSenseDeviceManager);
            }
            return (instance);
          }

          /** Get the number of connected DepthSense devices. */
          inline std::size_t
          getNumDevices ()
          {
            return (context_.getDevices ().size ());
          }

          /** Capture first available device and associate it with a given
            * grabber instance. */
          std::string
          captureDevice (DepthSenseGrabberImpl* grabber);

          /** Capture the device with given index and associate it with a given
            * grabber instance. */
          std::string
          captureDevice (DepthSenseGrabberImpl* grabber, std::size_t index);

          /** Capture the device with given serial number and associate it with
            * a given grabber instance. */
          std::string
          captureDevice (DepthSenseGrabberImpl* grabber, const std::string& sn);

          /** Release DepthSense device with given serial number. */
          void
          releaseDevice (const std::string& sn);

          /** Reconfigure DepthSense device with given serial number. */
          void
          reconfigureDevice (const std::string& sn);

          /** Start data capturing for a given device. */
          void
          startDevice (const std::string& sn);

          /** Stop data capturing for a given device. */
          void
          stopDevice (const std::string& sn);

          ~DepthSenseDeviceManager ();

        private:

          DepthSenseDeviceManager ();

          std::string
          captureDevice (DepthSenseGrabberImpl* grabber, DepthSense::Device device);

          inline bool
          isCaptured (const std::string& sn) const
          {
            return (captured_devices_.count (sn) != 0);
          }

          DepthSense::Context context_;

          static std::mutex mutex_;

          /// Thread where the grabbing takes place.
          std::thread depth_sense_thread_;

          struct CapturedDevice
          {
            DepthSenseGrabberImpl* grabber;
            DepthSense::DepthNode depth_node;
            DepthSense::ColorNode color_node;
          };

          std::map<std::string, CapturedDevice> captured_devices_;

      };

    } // namespace depth_sense

  } // namespace io

} // namespace pcl
