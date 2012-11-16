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

#include <pcl/gpu/containers/device_array.h>
#include <pcl/gpu/containers/kernel_containers.h>

#include <boost/shared_ptr.hpp>
#include <string>

#include <pcl/gpu/kinfu_large_scale/kinfu.h>

namespace pcl
{
  namespace gpu
  {
    namespace kinfuLS
    {
      class CaptureOpenNI
      {
  public:
      typedef pcl::gpu::kinfuLS::PixelRGB RGB;

      enum { PROP_OPENNI_REGISTRATION_ON  = 104 };


      CaptureOpenNI();
      CaptureOpenNI(int device);
      CaptureOpenNI(const std::string& oni_filename);

      void open(int device);
      void open(const std::string& oni_filename);
      void release();

      ~CaptureOpenNI();

      bool grab (PtrStepSz<const unsigned short>& depth, PtrStepSz<const RGB>& rgb24);

      //parameters taken from camera/oni
      float depth_focal_length_VGA;
      float baseline;         // mm
      int shadow_value;
      int no_sample_value;
      double pixelSize;         //mm

      unsigned short max_depth;         //mm

      bool setRegistration (bool value = false);
  private:
      struct Impl;
      boost::shared_ptr<Impl> impl_;
      void getParams ();

      };
    }
  }
};
