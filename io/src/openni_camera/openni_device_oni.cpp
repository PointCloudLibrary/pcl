/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011 2011 Willow Garage, Inc.
 *    Suat Gedikli <gedikli@willowgarage.com>
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
 */
#include <pcl/pcl_config.h>
#ifdef HAVE_OPENNI

#include <pcl/io/openni_camera/openni_device_oni.h>
#include <pcl/io/openni_camera/openni_image_rgb24.h>

using namespace std;
using namespace boost;

namespace openni_wrapper
{

DeviceONI::DeviceONI(xn::Context& context, const std::string& file_name, bool repeat, bool streaming)
  : OpenNIDevice(context)
  , streaming_ (streaming)
  , depth_stream_running_ (false)
  , image_stream_running_ (false)
  , ir_stream_running_ (false)
{
  XnStatus status;
#if (XN_MINOR_VERSION >= 3)
  status = context_.OpenFileRecording(file_name.c_str(), player_);
  if (status != XN_STATUS_OK)
    THROW_OPENNI_EXCEPTION("Could not open ONI file. Reason: %s", xnGetStatusString(status));
#else
  status = context_.OpenFileRecording(file_name.c_str());
  if (status != XN_STATUS_OK)
    THROW_OPENNI_EXCEPTION("Could not open ONI file. Reason: %s", xnGetStatusString(status));

  status = context.FindExistingNode(XN_NODE_TYPE_PLAYER, player_);
  if (status != XN_STATUS_OK)
    THROW_OPENNI_EXCEPTION("Failed to find player node: %s\n", xnGetStatusString(status));
#endif

  status = context.FindExistingNode(XN_NODE_TYPE_DEPTH, depth_generator_);
  if (status != XN_STATUS_OK)
    THROW_OPENNI_EXCEPTION("could not find depth stream in file %s. Reason: %s", file_name.c_str(), xnGetStatusString(status));
  else
  {
    available_depth_modes_.push_back(getDepthOutputMode());
    depth_generator_.RegisterToNewDataAvailable ((xn::StateChangedHandler)NewONIDepthDataAvailable, this, depth_callback_handle_);
  }

  status = context.FindExistingNode(XN_NODE_TYPE_IMAGE, image_generator_);
  if (status == XN_STATUS_OK)
  {
    available_image_modes_.push_back(getImageOutputMode());
    image_generator_.RegisterToNewDataAvailable ((xn::StateChangedHandler)NewONIImageDataAvailable, this, image_callback_handle_);
  }

  status = context.FindExistingNode(XN_NODE_TYPE_IR, ir_generator_);
  if (status == XN_STATUS_OK)
    ir_generator_.RegisterToNewDataAvailable ((xn::StateChangedHandler)NewONIIRDataAvailable, this, ir_callback_handle_);

  device_node_info_ = player_.GetInfo();

  Init ();

  player_.SetRepeat(repeat);
  if (streaming_)
    player_thread_ = boost::thread (&DeviceONI::PlayerThreadFunction, this);
}

DeviceONI::~DeviceONI() throw ()
{
  if (streaming_)
  {
    quit_ = true;
    player_thread_.join();
  }
}

void DeviceONI::startImageStream ()
{
  if (hasImageStream() && !image_stream_running_)
    image_stream_running_ = true;
}

void DeviceONI::stopImageStream ()
{
  if (hasImageStream() && image_stream_running_)
    image_stream_running_ = false;
}

void DeviceONI::startDepthStream ()
{
  if (hasDepthStream() && !depth_stream_running_)
    depth_stream_running_ = true;
}

void DeviceONI::stopDepthStream ()
{
  if (hasDepthStream() && depth_stream_running_)
    depth_stream_running_ = false;
}

void DeviceONI::startIRStream ()
{
  if (hasIRStream() && !ir_stream_running_)
    ir_stream_running_ = true;
}

void DeviceONI::stopIRStream ()
{
  if (hasIRStream() && ir_stream_running_)
    ir_stream_running_ = false;
}

bool DeviceONI::isImageStreamRunning () const throw ()
{
 return image_stream_running_;
}

bool DeviceONI::isDepthStreamRunning () const throw ()
{
  return depth_stream_running_;
}

bool DeviceONI::isIRStreamRunning () const throw ()
{
  return ir_stream_running_;
}

bool DeviceONI::trigger ()
{
  if (player_.IsEOF())
    return false;

  if (streaming_)
    THROW_OPENNI_EXCEPTION ("Virtual device is in streaming mode. Trigger not available.");

  player_.ReadNext();
  return true;
}

bool DeviceONI::isStreaming () const throw ()
{
  return streaming_;
}

void DeviceONI::PlayerThreadFunction()
{
  quit_ = false;
  while (!quit_)
    player_.ReadNext();
}

void __stdcall DeviceONI::NewONIDepthDataAvailable (xn::ProductionNode&, void* cookie) throw ()
{
  DeviceONI* device = reinterpret_cast<DeviceONI*>(cookie);
  if (device->depth_stream_running_)
    device->depth_condition_.notify_all ();
}

void __stdcall DeviceONI::NewONIImageDataAvailable (xn::ProductionNode&, void* cookie) throw ()
{
  DeviceONI* device = reinterpret_cast<DeviceONI*>(cookie);
  if (device->image_stream_running_)
    device->image_condition_.notify_all ();
}

void __stdcall DeviceONI::NewONIIRDataAvailable (xn::ProductionNode&, void* cookie) throw ()
{
  DeviceONI* device = reinterpret_cast<DeviceONI*>(cookie);
  if (device->ir_stream_running_)
    device->ir_condition_.notify_all ();
}

boost::shared_ptr<Image> DeviceONI::getCurrentImage(boost::shared_ptr<xn::ImageMetaData> image_meta_data) const throw ()
{
  return boost::shared_ptr<Image > (new ImageRGB24(image_meta_data));
}

bool DeviceONI::isImageResizeSupported(unsigned input_width, unsigned input_height, unsigned output_width, unsigned output_height) const throw ()
{
  return ImageRGB24::resizingSupported (input_width, input_height, output_width, output_height);
}

}

#endif //HAVE_OPENNI

