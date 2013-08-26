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
#include "OpenNI.h"

#include "pcl/io/openni2_camera/openni2_frame_listener.h"
#include "pcl/io/openni2_camera/openni2_timer_filter.h"

#define TIME_FILTER_LENGTH 15

namespace openni2_wrapper
{

	OpenNI2FrameListener::OpenNI2FrameListener() :
		callback_(0),
		user_device_timer_(false),
		timer_filter_(new OpenNI2TimerFilter(TIME_FILTER_LENGTH)),
		prev_time_stamp_(0.0)
	{ }

	void OpenNI2FrameListener::setUseDeviceTimer(bool enable)
	{
		user_device_timer_ = enable;

		if (user_device_timer_)
			timer_filter_->clear();
	}

	void OpenNI2FrameListener::onNewFrame(openni::VideoStream& stream)
	{
		stream.readFrame(&m_frame);

		if (m_frame.isValid() && callback_)
		{
			callback_(m_frame);
		}

	}

}

