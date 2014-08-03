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

#ifndef PCL_IO_OPENNI2_DEVICE_MANAGER_H_
#define PCL_IO_OPENNI2_DEVICE_MANAGER_H_

#include <pcl/pcl_exports.h>
#include "pcl/io/openni2/openni2_device_info.h"

#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include <vector>
#include <string>
#include <ostream>

namespace pcl
{
  namespace io
  {
    namespace openni2
    {

      class OpenNI2DeviceListener;
      class OpenNI2Device;

      class PCL_EXPORTS OpenNI2DeviceManager
      {
      public:
        OpenNI2DeviceManager ();
        virtual ~OpenNI2DeviceManager ();

        // This may not actually be a sigleton yet. Need to work out cross-dll incerface.
        // Based on http://stackoverflow.com/a/13431981/1789618
        static boost::shared_ptr<OpenNI2DeviceManager> getInstance ()
        {
          static boost::shared_ptr<OpenNI2DeviceManager> instance = boost::make_shared<OpenNI2DeviceManager>();
          return (instance);
        }

        boost::shared_ptr<std::vector<OpenNI2DeviceInfo> >
        getConnectedDeviceInfos () const;

        boost::shared_ptr<std::vector<std::string> >
        getConnectedDeviceURIs () const;

        std::size_t
        getNumOfConnectedDevices () const;

        boost::shared_ptr<OpenNI2Device>
        getAnyDevice ();

        boost::shared_ptr<OpenNI2Device>
        getDevice (const std::string& device_URI);

        boost::shared_ptr<OpenNI2Device>
        getDeviceByIndex (int index);

        boost::shared_ptr<OpenNI2Device>
        getFileDevice (const std::string& path);

      protected:
        boost::shared_ptr<OpenNI2DeviceListener> device_listener_;
      };

      std::ostream&
      operator<< (std::ostream& stream, const OpenNI2DeviceManager& device_manager);

    } // namespace
  }
}

#endif // PCL_IO_OPENNI2_DEVICE_MANAGER_H_
