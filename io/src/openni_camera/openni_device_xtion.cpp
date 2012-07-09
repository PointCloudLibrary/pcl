/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2011, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
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
#include <pcl/pcl_config.h>
#ifdef HAVE_OPENNI

#ifdef __GNUC__
#pragma GCC diagnostic ignored "-Wold-style-cast"
#endif

#include <pcl/io/openni_camera/openni_device_xtion.h>
#include <sstream>
#include <pcl/io/boost.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
openni_wrapper::DeviceXtionPro::DeviceXtionPro (xn::Context& context, const xn::NodeInfo& device_node, const xn::NodeInfo& depth_node, const xn::NodeInfo& ir_node)
: OpenNIDevice (context, device_node, depth_node, ir_node)
{
  // setup stream modes
  enumAvailableModes ();
  setDepthOutputMode (getDefaultDepthMode ());
  setIROutputMode (getDefaultIRMode ());

  boost::lock_guard<boost::mutex> depth_lock (depth_mutex_);
  XnStatus status = depth_generator_.SetIntProperty ("RegistrationType", 1);
  if (status != XN_STATUS_OK)
    THROW_OPENNI_EXCEPTION ("Error setting the registration type. Reason: %s", xnGetStatusString (status));
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
openni_wrapper::DeviceXtionPro::~DeviceXtionPro () throw ()
{
  depth_mutex_.lock ();
  depth_generator_.UnregisterFromNewDataAvailable (depth_callback_handle_);
  depth_mutex_.unlock ();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool 
openni_wrapper::DeviceXtionPro::isImageResizeSupported (unsigned, unsigned, unsigned, unsigned) const throw ()
{
  return (false);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void 
openni_wrapper::DeviceXtionPro::enumAvailableModes () throw ()
{
  XnMapOutputMode output_mode;
  available_image_modes_.clear();
  available_depth_modes_.clear();

  // Depth Modes
  output_mode.nFPS = 30;
  output_mode.nXRes = XN_VGA_X_RES;
  output_mode.nYRes = XN_VGA_Y_RES;
  available_depth_modes_.push_back (output_mode);

  output_mode.nFPS = 25;
  output_mode.nXRes = XN_VGA_X_RES;
  output_mode.nYRes = XN_VGA_Y_RES;
  available_depth_modes_.push_back (output_mode);

  output_mode.nFPS = 25;
  output_mode.nXRes = XN_QVGA_X_RES;
  output_mode.nYRes = XN_QVGA_Y_RES;
  available_depth_modes_.push_back (output_mode);

  output_mode.nFPS = 30;
  output_mode.nXRes = XN_QVGA_X_RES;
  output_mode.nYRes = XN_QVGA_Y_RES;
  available_depth_modes_.push_back (output_mode);

  output_mode.nFPS = 60;
  output_mode.nXRes = XN_QVGA_X_RES;
  output_mode.nYRes = XN_QVGA_Y_RES;
  available_depth_modes_.push_back (output_mode);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
boost::shared_ptr<openni_wrapper::Image> 
openni_wrapper::DeviceXtionPro::getCurrentImage (boost::shared_ptr<xn::ImageMetaData>) const throw ()
{
  return (boost::shared_ptr<Image> (reinterpret_cast<Image*> (0)));
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void 
openni_wrapper::DeviceXtionPro::startDepthStream ()
{
  if (isDepthRegistered ())
  {
    // Reset the view point
    setDepthRegistration (false);

    // Start the stream
    OpenNIDevice::startDepthStream ();

    // Register the stream
    setDepthRegistration (true);
  }
  else
    // Start the stream
    OpenNIDevice::startDepthStream ();
}

#endif
