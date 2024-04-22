/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
 *  Author: Anatoly Baskeheev, Itseez Ltd, (myname.mysurname@mycompany.com)
 */

#pragma once

#include <pcl/pcl_exports.h>

#include <string>

namespace pcl {
namespace gpu {
/** \brief Returns number of Cuda device. */
PCL_EXPORTS int
getCudaEnabledDeviceCount();

/** \brief Sets active device to work with. */
PCL_EXPORTS void
setDevice(int device);

/** \brief Return device name for given device. */
PCL_EXPORTS std::string
getDeviceName(int device);

/** \brief Prints information about given cuda device or about all devices
 *  \param device: if < 0 prints info for all devices, otherwise the function interprets
 * it as device id.
 */
void PCL_EXPORTS
printCudaDeviceInfo(int device = -1);

/** \brief Prints information about given cuda device or about all devices
 *  \param device: if < 0 prints info for all devices, otherwise the function interprets
 * it as device id.
 */
void PCL_EXPORTS
printShortCudaDeviceInfo(int device = -1);

/** \brief Returns true if pre-Fermi generator GPU.
 * \param device: device id to check, if < 0 checks current device.
 */
bool PCL_EXPORTS
checkIfPreFermiGPU(int device = -1);

/** \brief Error handler. All GPU functions call this to report an error. For internal
 * use only */
void PCL_EXPORTS
error(const char* error_string,
      const char* file,
      const int line,
      const char* func = "");
} // namespace gpu
} // namespace pcl
