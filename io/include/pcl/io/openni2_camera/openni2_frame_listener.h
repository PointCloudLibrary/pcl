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

#ifndef OPENNI2_FRAME_LISTENER_H_
#define OPENNI2_FRAME_LISTENER_H_

#include "openni2_camera/openni2_device.h"

#include <sensor_msgs/Image.h>

#include <vector>

#include "OpenNI.h"

namespace openni2_wrapper
{

class OpenNI2TimerFilter;

class OpenNI2FrameListener : public openni::VideoStream::NewFrameListener
{
public:
  OpenNI2FrameListener();

  virtual ~OpenNI2FrameListener()
  { };

  void onNewFrame(openni::VideoStream& stream);

  void setCallback(FrameCallbackFunction& callback)
  {
    callback_ = callback;
  }

  bool setUseDeviceTimer(bool enable);

private:
  openni::VideoFrameRef m_frame;

  FrameCallbackFunction callback_;

  bool user_device_timer_;
  boost::shared_ptr<OpenNI2TimerFilter> timer_filter_;

  double prev_time_stamp_;
};

}

#endif
