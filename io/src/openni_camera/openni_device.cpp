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

#include <pcl/io/openni_camera/openni_driver.h>
#include <pcl/io/openni_camera/openni_device.h>
#include <pcl/io/openni_camera/openni_depth_image.h>
#include <pcl/io/openni_camera/openni_ir_image.h>
#include <iostream>
#include <limits>
#include <sstream>
#include <map>
#include <vector>
#include "XnVersion.h"

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
openni_wrapper::OpenNIDevice::OpenNIDevice (
    xn::Context& context, 
    const xn::NodeInfo& device_node, 
#ifdef __APPLE__
    const xn::NodeInfo&, 
    const xn::NodeInfo&, 
    const xn::NodeInfo&
#else
    const xn::NodeInfo& image_node, 
    const xn::NodeInfo& depth_node, 
    const xn::NodeInfo& ir_node
#endif
  )
  : image_callback_ (),
    depth_callback_ (),
    ir_callback_ (),
    available_image_modes_ (),
    available_depth_modes_ (),
    context_ (context),
    device_node_info_ (device_node),
    depth_generator_ (),
    image_generator_ (),
    ir_generator_ (),
    depth_callback_handle_ (),
    image_callback_handle_ (),
    ir_callback_handle_ (),
    depth_focal_length_SXGA_ (),
    baseline_ (),
    // This magic value is taken from a calibration routine.
    rgb_focal_length_SXGA_ (1050),
    shadow_value_ (),
    no_sample_value_ (),
    image_callback_handle_counter_ (),
    depth_callback_handle_counter_ (),
    ir_callback_handle_counter_ (),
    quit_ (),
    image_mutex_ (), depth_mutex_ (), ir_mutex_ (),
    image_condition_ (), depth_condition_ (), ir_condition_ (), 
    image_thread_ (), depth_thread_ (), ir_thread_ ()
{
// workaround for MAC from Alex Ichim
#ifdef __APPLE__
  XnStatus rc;

  std::string config ("/etc/openni/SamplesConfig.xml");
  xn::EnumerationErrors errors;
  rc = context_.InitFromXmlFile (config.c_str (), &errors);
  if (rc == XN_STATUS_NO_NODE_PRESENT)
  {
    XnChar str_error[1024];
    errors.ToString (str_error, 1024);
    THROW_OPENNI_EXCEPTION ("[openni_wrapper::OpenNIDevice::OpenNIDevice] %s\n", str_error);
  }
  else if (rc != XN_STATUS_OK)
    THROW_OPENNI_EXCEPTION ("[openni_wrapper::OpenNIDevice::OpenNIDevice] Cannot open %s! (OpenNI: %s)\n", config.c_str (), xnGetStatusString (rc));

  XnStatus status = context_.FindExistingNode (XN_NODE_TYPE_DEPTH, depth_generator_);
  //if (status != XN_STATUS_OK)
  //  THROW_OPENNI_EXCEPTION ("[openni_wrapper::OpenNIDevice::OpenNIDevice] Cannot find any depth nodes!\n");
  status = context_.FindExistingNode (XN_NODE_TYPE_IMAGE, image_generator_);
  //if (status != XN_STATUS_OK)
  //  THROW_OPENNI_EXCEPTION ("[openni_wrapper::OpenNIDevice::OpenNIDevice] Cannot find any image nodes!\n");
  status = context_.FindExistingNode (XN_NODE_TYPE_IR, ir_generator_);
  //if (status != XN_STATUS_OK)
  //  THROW_OPENNI_EXCEPTION ("[openni_wrapper::OpenNIDevice::OpenNIDevice] Cannot find any IR nodes!\n");

#else

#if (XN_MINOR_VERSION >= 3)
// create the production nodes
  XnStatus status = context_.CreateProductionTree (const_cast<xn::NodeInfo&>(depth_node), depth_generator_);
  if (status != XN_STATUS_OK)
    THROW_OPENNI_EXCEPTION ("creating depth generator failed. Reason: %s", xnGetStatusString (status));

  status = context_.CreateProductionTree (const_cast<xn::NodeInfo&>(image_node), image_generator_);
  if (status != XN_STATUS_OK)
    THROW_OPENNI_EXCEPTION ("creating image generator failed. Reason: %s", xnGetStatusString (status));

  status = context_.CreateProductionTree (const_cast<xn::NodeInfo&>(ir_node), ir_generator_);
  if (status != XN_STATUS_OK)
    THROW_OPENNI_EXCEPTION ("creating IR generator failed. Reason: %s", xnGetStatusString (status));
#else
  XnStatus status = context_.CreateProductionTree (const_cast<xn::NodeInfo&>(depth_node));
  if (status != XN_STATUS_OK)
    THROW_OPENNI_EXCEPTION ("creating depth generator failed. Reason: %s", xnGetStatusString (status));

  status = context_.CreateProductionTree (const_cast<xn::NodeInfo&>(image_node));
  if (status != XN_STATUS_OK)
    THROW_OPENNI_EXCEPTION ("creating image generator failed. Reason: %s", xnGetStatusString (status));

  status = context_.CreateProductionTree (const_cast<xn::NodeInfo&>(ir_node));
  if (status != XN_STATUS_OK)
    THROW_OPENNI_EXCEPTION ("creating IR generator failed. Reason: %s", xnGetStatusString (status));

  // get production node instances
  status = depth_node.GetInstance (depth_generator_);
  if (status != XN_STATUS_OK)
    THROW_OPENNI_EXCEPTION ("creating depth generator instance failed. Reason: %s", xnGetStatusString (status));

  status = image_node.GetInstance (image_generator_);
  if (status != XN_STATUS_OK)
    THROW_OPENNI_EXCEPTION ("creating image generator instance failed. Reason: %s", xnGetStatusString (status));

  status = ir_node.GetInstance (ir_generator_);
  if (status != XN_STATUS_OK)
    THROW_OPENNI_EXCEPTION ("creating IR generator instance failed. Reason: %s", xnGetStatusString (status));
#endif // (XN_MINOR_VERSION >= 3)
  ir_generator_.RegisterToNewDataAvailable (static_cast<xn::StateChangedHandler> (NewIRDataAvailable), this, ir_callback_handle_);
#endif // __APPLE__

  depth_generator_.RegisterToNewDataAvailable (static_cast<xn::StateChangedHandler> (NewDepthDataAvailable), this, depth_callback_handle_);
  image_generator_.RegisterToNewDataAvailable (static_cast<xn::StateChangedHandler> (NewImageDataAvailable), this, image_callback_handle_);

  Init ();

  // read sensor configuration from device
  InitShiftToDepthConversion();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
openni_wrapper::OpenNIDevice::OpenNIDevice (xn::Context& context, const xn::NodeInfo& device_node, 
#ifdef __APPLE__
    const xn::NodeInfo&, 
    const xn::NodeInfo&
#else
    const xn::NodeInfo& depth_node, 
    const xn::NodeInfo& ir_node
#endif
    )
  : image_callback_ (),
    depth_callback_ (),
    ir_callback_ (),
    available_image_modes_ (),
    available_depth_modes_ (),
    context_ (context),
    device_node_info_ (device_node),
    depth_generator_ (),
    image_generator_ (),
    ir_generator_ (),
    depth_callback_handle_ (),
    image_callback_handle_ (),
    ir_callback_handle_ (),
    depth_focal_length_SXGA_ (),
    baseline_ (),
    // This magic value is taken from a calibration routine.
    rgb_focal_length_SXGA_ (1050),
    shadow_value_ (),
    no_sample_value_ (),
    image_callback_handle_counter_ (),
    depth_callback_handle_counter_ (),
    ir_callback_handle_counter_ (),
    quit_ (),
    image_mutex_ (), depth_mutex_ (), ir_mutex_ (),
    image_condition_ (), depth_condition_ (), ir_condition_ (), 
    image_thread_ (), depth_thread_ (), ir_thread_ ()
{
// workaround for MAC from Alex Ichim
#ifdef __APPLE__
  XnStatus rc;

  std::string config ("/etc/openni/SamplesConfig.xml");
  xn::EnumerationErrors errors;
  rc = context_.InitFromXmlFile (config.c_str (), &errors);
  if (rc == XN_STATUS_NO_NODE_PRESENT)
  {
    XnChar str_error[1024];
    errors.ToString (str_error, 1024);
    THROW_OPENNI_EXCEPTION ("[openni_wrapper::OpenNIDevice::OpenNIDevice] %s\n", str_error);
  }
  else if (rc != XN_STATUS_OK)
    THROW_OPENNI_EXCEPTION ("[openni_wrapper::OpenNIDevice::OpenNIDevice] Cannot open %s! (OpenNI: %s)\n", config.c_str (), xnGetStatusString (rc));

  XnStatus status = context_.FindExistingNode (XN_NODE_TYPE_DEPTH, depth_generator_);
  //if (status != XN_STATUS_OK)
  //  THROW_OPENNI_EXCEPTION ("[openni_wrapper::OpenNIDevice::OpenNIDevice] Cannot find any depth nodes!\n");
  status = context_.FindExistingNode (XN_NODE_TYPE_IR, ir_generator_);
  //if (status != XN_STATUS_OK)
  //  THROW_OPENNI_EXCEPTION ("[openni_wrapper::OpenNIDevice::OpenNIDevice] Cannot find any IR nodes!\n");

#else
  XnStatus status;
#if (XN_MINOR_VERSION >=3)
  // create the production nodes
  status = context_.CreateProductionTree (const_cast<xn::NodeInfo&>(depth_node), depth_generator_);
  if (status != XN_STATUS_OK)
    THROW_OPENNI_EXCEPTION ("creating depth generator failed. Reason: %s", xnGetStatusString (status));

  status = context_.CreateProductionTree (const_cast<xn::NodeInfo&>(ir_node), ir_generator_);
  if (status != XN_STATUS_OK)
    THROW_OPENNI_EXCEPTION ("creating IR generator failed. Reason: %s", xnGetStatusString (status));
#else
  status = context_.CreateProductionTree (const_cast<xn::NodeInfo&>(depth_node));
  if (status != XN_STATUS_OK)
    THROW_OPENNI_EXCEPTION ("creating depth generator failed. Reason: %s", xnGetStatusString (status));

  status = context_.CreateProductionTree (const_cast<xn::NodeInfo&>(ir_node));
  if (status != XN_STATUS_OK)
    THROW_OPENNI_EXCEPTION ("creating IR generator failed. Reason: %s", xnGetStatusString (status));
  // get production node instances
  status = depth_node.GetInstance (depth_generator_);
  if (status != XN_STATUS_OK)
    THROW_OPENNI_EXCEPTION ("creating depth generator instance failed. Reason: %s", xnGetStatusString (status));

  status = ir_node.GetInstance (ir_generator_);
  if (status != XN_STATUS_OK)
    THROW_OPENNI_EXCEPTION ("creating IR generator instance failed. Reason: %s", xnGetStatusString (status));
#endif // (XN_MINOR_VERSION >= 3)
  ir_generator_.RegisterToNewDataAvailable (static_cast<xn::StateChangedHandler> (NewIRDataAvailable), this, ir_callback_handle_);
#endif // __APPLE__

  depth_generator_.RegisterToNewDataAvailable (static_cast <xn::StateChangedHandler> (NewDepthDataAvailable), this, depth_callback_handle_);
  // set up rest
  Init ();

  // read sensor configuration from device
  InitShiftToDepthConversion();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// For ONI Player devices
openni_wrapper::OpenNIDevice::OpenNIDevice (xn::Context& context)
  : image_callback_ (),
    depth_callback_ (),
    ir_callback_ (),
    available_image_modes_ (),
    available_depth_modes_ (),
    context_ (context),
    device_node_info_ (0),
    depth_generator_ (),
    image_generator_ (),
    ir_generator_ (),
    depth_callback_handle_ (),
    image_callback_handle_ (),
    ir_callback_handle_ (),
    depth_focal_length_SXGA_ (),
    baseline_ (),
    // This magic value is taken from a calibration routine.
    rgb_focal_length_SXGA_ (1050),
    shadow_value_ (),
    no_sample_value_ (),
    image_callback_handle_counter_ (),
    depth_callback_handle_counter_ (),
    ir_callback_handle_counter_ (),
    quit_ (),
    image_mutex_ (), depth_mutex_ (), ir_mutex_ (),
    image_condition_ (), depth_condition_ (), ir_condition_ (), 
    image_thread_ (), depth_thread_ (), ir_thread_ ()
{
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
openni_wrapper::OpenNIDevice::~OpenNIDevice () throw ()
{
  // stop streams
  if (image_generator_.IsValid() && image_generator_.IsGenerating ())
    image_generator_.StopGenerating ();

  if (depth_generator_.IsValid () && depth_generator_.IsGenerating ())
    depth_generator_.StopGenerating ();

  if (ir_generator_.IsValid () && ir_generator_.IsGenerating ())
    ir_generator_.StopGenerating ();

  // lock before changing running flag
  image_mutex_.lock ();
  depth_mutex_.lock ();
  ir_mutex_.lock ();
  quit_ = true;

  depth_condition_.notify_all ();
  image_condition_.notify_all ();
  ir_condition_.notify_all ();
  ir_mutex_.unlock ();
  depth_mutex_.unlock ();
  image_mutex_.unlock ();

  xn::Device deviceNode;
  device_node_info_.GetInstance(deviceNode);
  if (deviceNode.IsValid())
    deviceNode.Release ();
  
  if (hasImageStream ())
  {
    image_thread_.join ();
    image_generator_.Release ();
  }
  
  if (hasDepthStream ())
  {
    depth_thread_.join ();
    depth_generator_.Release ();
  }
  
  if (hasIRStream ())
  {
    ir_thread_.join ();
    ir_generator_.Release ();
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void 
openni_wrapper::OpenNIDevice::Init ()
{
  quit_ = false;
  XnDouble pixel_size;

  // set Depth resolution here only once... since no other mode for kinect is available -> deactivating setDepthResolution method!
  if (hasDepthStream ())
  {
    boost::unique_lock<boost::mutex> depth_lock (depth_mutex_);
    XnStatus status = depth_generator_.GetRealProperty ("ZPPS", pixel_size);
    if (status != XN_STATUS_OK)
      THROW_OPENNI_EXCEPTION ("reading the pixel size of IR camera failed. Reason: %s", xnGetStatusString (status));

    XnUInt64 depth_focal_length_SXGA;
    status = depth_generator_.GetIntProperty ("ZPD", depth_focal_length_SXGA);
    if (status != XN_STATUS_OK)
      THROW_OPENNI_EXCEPTION ("reading the focal length of IR camera failed. Reason: %s", xnGetStatusString (status));

    XnDouble baseline;
    status = depth_generator_.GetRealProperty ("LDDIS", baseline);
    if (status != XN_STATUS_OK)
      THROW_OPENNI_EXCEPTION ("reading the baseline failed. Reason: %s", xnGetStatusString (status));

    status = depth_generator_.GetIntProperty ("ShadowValue", shadow_value_);
    if (status != XN_STATUS_OK)
      THROW_OPENNI_EXCEPTION ("reading the value for pixels in shadow regions failed. Reason: %s", xnGetStatusString (status));

    status = depth_generator_.GetIntProperty ("NoSampleValue", no_sample_value_);
    if (status != XN_STATUS_OK)
      THROW_OPENNI_EXCEPTION ("reading the value for pixels with no depth estimation failed. Reason: %s", xnGetStatusString (status));

    // baseline from cm -> meters
    baseline_ = static_cast<float> (baseline * 0.01);

    //focal length from mm -> pixels (valid for 1280x1024)
    depth_focal_length_SXGA_ = static_cast<float> (static_cast<XnDouble> (depth_focal_length_SXGA) / pixel_size);

    depth_thread_ = boost::thread (&OpenNIDevice::DepthDataThreadFunction, this);
  }

  if (hasImageStream ())
  {
    boost::lock_guard<boost::mutex> image_lock (image_mutex_);
    image_thread_ = boost::thread (&OpenNIDevice::ImageDataThreadFunction, this);
  }

  if (hasIRStream ())
  {
    boost::lock_guard<boost::mutex> ir_lock (ir_mutex_);
    ir_thread_ = boost::thread (&OpenNIDevice::IRDataThreadFunction, this);
  }
}

void openni_wrapper::OpenNIDevice::InitShiftToDepthConversion ()
{
  // Read device configuration from OpenNI sensor node
  ReadDeviceParametersFromSensorNode();

  if (shift_conversion_parameters_.init_)
  {
    // Calculate shift conversion table

    pcl::uint32_t nIndex = 0;
    pcl::int32_t nShiftValue = 0;
    double dFixedRefX = 0;
    double dMetric = 0;
    double dDepth = 0;
    double dPlanePixelSize = shift_conversion_parameters_.zero_plane_pixel_size_;
    double dPlaneDsr = shift_conversion_parameters_.zero_plane_distance_;
    double dPlaneDcl = shift_conversion_parameters_.emitter_dcmos_distace_;
    pcl::int32_t nConstShift = shift_conversion_parameters_.param_coeff_ *
        shift_conversion_parameters_.const_shift_;

    dPlanePixelSize *= shift_conversion_parameters_.pixel_size_factor_;
    nConstShift /= shift_conversion_parameters_.pixel_size_factor_;

    shift_to_depth_table_.resize(shift_conversion_parameters_.device_max_shift_+1);

    for (nIndex = 1; nIndex < shift_conversion_parameters_.device_max_shift_; nIndex++)
    {
      nShiftValue = (pcl::int32_t)nIndex;

      dFixedRefX = (double) (nShiftValue - nConstShift) /
                   (double) shift_conversion_parameters_.param_coeff_;
      dFixedRefX -= 0.375;
      dMetric = dFixedRefX * dPlanePixelSize;
      dDepth = shift_conversion_parameters_.shift_scale_ *
               ((dMetric * dPlaneDsr / (dPlaneDcl - dMetric)) + dPlaneDsr);

      // check cut-offs
      if ((dDepth > shift_conversion_parameters_.min_depth_) &&
          (dDepth < shift_conversion_parameters_.max_depth_))
      {
        shift_to_depth_table_[nIndex] = (pcl::uint16_t)(dDepth);
      }
    }

  }
  else
      THROW_OPENNI_EXCEPTION ("Shift-to-depth lookup calculation failed. Reason: Device configuration not initialized.");
}

void
openni_wrapper::OpenNIDevice::ReadDeviceParametersFromSensorNode ()
{
  if (hasDepthStream ())
  {

    XnUInt64 nTemp;
    XnDouble dTemp;

    XnStatus status = depth_generator_.GetIntProperty ("ZPD", nTemp);
    if (status != XN_STATUS_OK)
      THROW_OPENNI_EXCEPTION ("reading the zero plane distance failed. Reason: %s", xnGetStatusString (status));

    shift_conversion_parameters_.zero_plane_distance_ =  (XnUInt16)nTemp;

    status = depth_generator_.GetRealProperty ("ZPPS", dTemp);
    if (status != XN_STATUS_OK)
      THROW_OPENNI_EXCEPTION ("reading the zero plane pixel size failed. Reason: %s", xnGetStatusString (status));

    shift_conversion_parameters_.zero_plane_pixel_size_ =  (XnFloat)dTemp;

    status = depth_generator_.GetRealProperty ("LDDIS", dTemp);
    if (status != XN_STATUS_OK)
      THROW_OPENNI_EXCEPTION ("reading the dcmos distance failed. Reason: %s", xnGetStatusString (status));

    shift_conversion_parameters_.emitter_dcmos_distace_ =  (XnFloat)dTemp;

    status = depth_generator_.GetIntProperty ("MaxShift", nTemp);
    if (status != XN_STATUS_OK)
      THROW_OPENNI_EXCEPTION ("reading the max shift parameter failed. Reason: %s", xnGetStatusString (status));

    shift_conversion_parameters_.max_shift_ =  (XnUInt32)nTemp;

    status = depth_generator_.GetIntProperty ("DeviceMaxDepth", nTemp);
    if (status != XN_STATUS_OK)
      THROW_OPENNI_EXCEPTION ("reading the device max depth parameter failed. Reason: %s", xnGetStatusString (status));

    shift_conversion_parameters_.device_max_shift_ =  (XnUInt32)nTemp;

    status = depth_generator_.GetIntProperty ("ConstShift", nTemp);
    if (status != XN_STATUS_OK)
      THROW_OPENNI_EXCEPTION ("reading the const shift parameter failed. Reason: %s", xnGetStatusString (status));

    shift_conversion_parameters_.const_shift_ =  (XnUInt32)nTemp;

    status = depth_generator_.GetIntProperty ("PixelSizeFactor", nTemp);
    if (status != XN_STATUS_OK)
      THROW_OPENNI_EXCEPTION ("reading the pixel size factor failed. Reason: %s", xnGetStatusString (status));

    shift_conversion_parameters_.pixel_size_factor_ =  (XnUInt32)nTemp;

    status = depth_generator_.GetIntProperty ("ParamCoeff", nTemp);
    if (status != XN_STATUS_OK)
      THROW_OPENNI_EXCEPTION ("reading the param coeff parameter failed. Reason: %s", xnGetStatusString (status));

    shift_conversion_parameters_.param_coeff_ =  (XnUInt32)nTemp;

    status = depth_generator_.GetIntProperty ("ShiftScale", nTemp);
    if (status != XN_STATUS_OK)
      THROW_OPENNI_EXCEPTION ("reading the shift scale parameter failed. Reason: %s", xnGetStatusString (status));

    shift_conversion_parameters_.shift_scale_ =  (XnUInt32)nTemp;

    status = depth_generator_.GetIntProperty ("MinDepthValue", nTemp);
    if (status != XN_STATUS_OK)
      THROW_OPENNI_EXCEPTION ("reading the min depth value parameter failed. Reason: %s", xnGetStatusString (status));

    shift_conversion_parameters_.min_depth_ =  (XnUInt32)nTemp;

    status = depth_generator_.GetIntProperty ("MaxDepthValue", nTemp);
    if (status != XN_STATUS_OK)
      THROW_OPENNI_EXCEPTION ("reading the max depth value parameter failed. Reason: %s", xnGetStatusString (status));

    shift_conversion_parameters_.max_depth_ =  (XnUInt32)nTemp;

    shift_conversion_parameters_.init_ = true;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void 
openni_wrapper::OpenNIDevice::startImageStream ()
{
  if (hasImageStream ())
  {
    boost::lock_guard<boost::mutex> image_lock (image_mutex_);
    if (!image_generator_.IsGenerating ())
    {
      XnStatus status = image_generator_.StartGenerating ();
      if (status != XN_STATUS_OK)
        THROW_OPENNI_EXCEPTION ("starting image stream failed. Reason: %s", xnGetStatusString (status));
    }
  }
  else
    THROW_OPENNI_EXCEPTION ("Device does not provide an image stream");
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void 
openni_wrapper::OpenNIDevice::stopImageStream ()
{
  if (hasImageStream ())
  {
    boost::lock_guard<boost::mutex> image_lock (image_mutex_);
    if (image_generator_.IsGenerating ())
    {
      XnStatus status = image_generator_.StopGenerating ();
      if (status != XN_STATUS_OK)
        THROW_OPENNI_EXCEPTION ("stopping image stream failed. Reason: %s", xnGetStatusString (status));
    }
  }
  else
    THROW_OPENNI_EXCEPTION ("Device does not provide an image stream");
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void 
openni_wrapper::OpenNIDevice::startDepthStream ()
{
  if (hasDepthStream ())
  {
    boost::lock_guard<boost::mutex> depth_lock (depth_mutex_);
    if (!depth_generator_.IsGenerating ())
    {
      XnStatus status = depth_generator_.StartGenerating ();

      if (status != XN_STATUS_OK)
        THROW_OPENNI_EXCEPTION ("starting depth stream failed. Reason: %s", xnGetStatusString (status));
    }
  }
  else
    THROW_OPENNI_EXCEPTION ("Device does not provide a depth stream");
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void 
openni_wrapper::OpenNIDevice::stopDepthStream ()
{
  if (hasDepthStream ())
  {
    boost::lock_guard<boost::mutex> depth_lock (depth_mutex_);
    if (depth_generator_.IsGenerating ())
    {
      XnStatus status = depth_generator_.StopGenerating ();

      if (status != XN_STATUS_OK)
        THROW_OPENNI_EXCEPTION ("stopping depth stream failed. Reason: %s", xnGetStatusString (status));
    }
  }
  else
    THROW_OPENNI_EXCEPTION ("Device does not provide a depth stream");
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void 
openni_wrapper::OpenNIDevice::startIRStream ()
{
  if (hasIRStream ())
  {
    boost::lock_guard<boost::mutex> ir_lock (ir_mutex_);
    if (!ir_generator_.IsGenerating ())
    {
      XnStatus status = ir_generator_.StartGenerating ();

      if (status != XN_STATUS_OK)
        THROW_OPENNI_EXCEPTION ("starting IR stream failed. Reason: %s", xnGetStatusString (status));
    }
  }
#ifndef __APPLE__
  else
    THROW_OPENNI_EXCEPTION ("Device does not provide an IR stream");
#endif
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void 
openni_wrapper::OpenNIDevice::stopIRStream ()
{
  if (hasIRStream ())
  {
    boost::lock_guard<boost::mutex> ir_lock (ir_mutex_);
    if (ir_generator_.IsGenerating ())
    {
      XnStatus status = ir_generator_.StopGenerating ();

      if (status != XN_STATUS_OK)
        THROW_OPENNI_EXCEPTION ("stopping IR stream failed. Reason: %s", xnGetStatusString (status));
    }
  }
  else
    THROW_OPENNI_EXCEPTION ("Device does not provide an IR stream");
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool 
openni_wrapper::OpenNIDevice::isImageStreamRunning () const throw ()
{
  boost::lock_guard<boost::mutex> image_lock (image_mutex_);
  return (image_generator_.IsValid () && image_generator_.IsGenerating ());
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool 
openni_wrapper::OpenNIDevice::isDepthStreamRunning () const throw ()
{
  boost::lock_guard<boost::mutex> depth_lock (depth_mutex_);
  return (depth_generator_.IsValid () && depth_generator_.IsGenerating ());
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool 
openni_wrapper::OpenNIDevice::isIRStreamRunning () const throw ()
{
  boost::lock_guard<boost::mutex> ir_lock (ir_mutex_);
  return (ir_generator_.IsValid () && ir_generator_.IsGenerating ());
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool 
openni_wrapper::OpenNIDevice::hasImageStream () const throw ()
{
  boost::lock_guard<boost::mutex> lock (image_mutex_);
  return (image_generator_.IsValid () != 0);
  //return (available_image_modes_.size() != 0);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool 
openni_wrapper::OpenNIDevice::hasDepthStream () const throw ()
{
  boost::lock_guard<boost::mutex> lock (depth_mutex_);
  return (depth_generator_.IsValid () != 0);
  //return (available_depth_modes_.size() != 0);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool 
openni_wrapper::OpenNIDevice::hasIRStream () const throw ()
{
  boost::lock_guard<boost::mutex> ir_lock (ir_mutex_);
  return (ir_generator_.IsValid () != 0);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void 
openni_wrapper::OpenNIDevice::setDepthRegistration (bool on_off)
{
  if (hasDepthStream () && hasImageStream())
  {
    boost::lock_guard<boost::mutex> image_lock (image_mutex_);
    boost::lock_guard<boost::mutex> depth_lock (depth_mutex_);
    if (on_off && !depth_generator_.GetAlternativeViewPointCap ().IsViewPointAs (image_generator_))
    {
      if (depth_generator_.GetAlternativeViewPointCap ().IsViewPointSupported (image_generator_))
      {
        XnStatus status = depth_generator_.GetAlternativeViewPointCap ().SetViewPoint (image_generator_);
        if (status != XN_STATUS_OK)
          THROW_OPENNI_EXCEPTION ("turning registration on failed. Reason: %s", xnGetStatusString (status));
      }
      else
        THROW_OPENNI_EXCEPTION ("turning registration on failed. Reason: unsopported viewpoint");
    }
    else if (!on_off)
    {
      XnStatus status = depth_generator_.GetAlternativeViewPointCap ().ResetViewPoint ();

      if (status != XN_STATUS_OK)
        THROW_OPENNI_EXCEPTION ("turning registration off failed. Reason: %s", xnGetStatusString (status));
    }
  }
  else
    THROW_OPENNI_EXCEPTION ("Device does not provide image + depth stream");
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool 
openni_wrapper::OpenNIDevice::isDepthRegistered () const throw ()
{
  if (hasDepthStream () && hasImageStream() )
  {
    xn::DepthGenerator& depth_generator = const_cast<xn::DepthGenerator&>(depth_generator_);
    xn::ImageGenerator& image_generator = const_cast<xn::ImageGenerator&>(image_generator_);

    boost::lock_guard<boost::mutex> image_lock (image_mutex_);
    boost::lock_guard<boost::mutex> depth_lock (depth_mutex_);
    return (depth_generator.GetAlternativeViewPointCap ().IsViewPointAs (image_generator) != 0);
  }
  return (false);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool 
openni_wrapper::OpenNIDevice::isDepthRegistrationSupported () const throw ()
{
  boost::lock_guard<boost::mutex> image_lock (image_mutex_);
  boost::lock_guard<boost::mutex> depth_lock (depth_mutex_);
  xn::ImageGenerator& image_generator = const_cast<xn::ImageGenerator&> (image_generator_);
  return (depth_generator_.IsValid() && image_generator_.IsValid() && depth_generator_.GetAlternativeViewPointCap().IsViewPointSupported(image_generator));
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool 
openni_wrapper::OpenNIDevice::isSynchronizationSupported () const throw ()
{
  boost::lock_guard<boost::mutex> image_lock (image_mutex_);
  boost::lock_guard<boost::mutex> depth_lock (depth_mutex_);
  return (depth_generator_.IsValid() && image_generator_.IsValid() && depth_generator_.IsCapabilitySupported (XN_CAPABILITY_FRAME_SYNC));
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void 
openni_wrapper::OpenNIDevice::setSynchronization (bool on_off)
{
  if (hasDepthStream () && hasImageStream())
  {
    boost::lock_guard<boost::mutex> image_lock (image_mutex_);
    boost::lock_guard<boost::mutex> depth_lock (depth_mutex_);
    XnStatus status;

    if (on_off && !depth_generator_.GetFrameSyncCap ().IsFrameSyncedWith (image_generator_))
    {
      status = depth_generator_.GetFrameSyncCap ().FrameSyncWith (image_generator_);
      if (status != XN_STATUS_OK)
        THROW_OPENNI_EXCEPTION ("could not turn on frame synchronization. Reason: %s", xnGetStatusString (status));
    }
    else if (!on_off && depth_generator_.GetFrameSyncCap ().IsFrameSyncedWith (image_generator_))
    {
      status = depth_generator_.GetFrameSyncCap ().StopFrameSyncWith (image_generator_);
      if (status != XN_STATUS_OK)
        THROW_OPENNI_EXCEPTION ("could not turn off frame synchronization. Reason: %s", xnGetStatusString (status));
    }
  }
  else
    THROW_OPENNI_EXCEPTION ("Device does not provide image + depth stream");
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool 
openni_wrapper::OpenNIDevice::isSynchronized () const throw ()
{
  if (hasDepthStream () && hasImageStream())
  {
    boost::lock_guard<boost::mutex> image_lock (image_mutex_);
    boost::lock_guard<boost::mutex> depth_lock (depth_mutex_);
    xn::DepthGenerator& depth_generator = const_cast<xn::DepthGenerator&>(depth_generator_);
    xn::ImageGenerator& image_generator = const_cast<xn::ImageGenerator&>(image_generator_);
    return (depth_generator.GetFrameSyncCap ().CanFrameSyncWith (image_generator) && depth_generator.GetFrameSyncCap ().IsFrameSyncedWith (image_generator));
  }
  else
    return (false);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool 
openni_wrapper::OpenNIDevice::isDepthCroppingSupported () const throw ()
{
  boost::lock_guard<boost::mutex> depth_lock (depth_mutex_);
  return (image_generator_.IsValid() && depth_generator_.IsCapabilitySupported (XN_CAPABILITY_CROPPING) );
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool 
openni_wrapper::OpenNIDevice::isDepthCropped () const
{
  if (hasDepthStream ())
  {
    boost::lock_guard<boost::mutex> depth_lock (depth_mutex_);
    XnCropping cropping;
    xn::DepthGenerator& depth_generator = const_cast<xn::DepthGenerator&>(depth_generator_);
    XnStatus status = depth_generator.GetCroppingCap ().GetCropping (cropping);
    if (status != XN_STATUS_OK)
      THROW_OPENNI_EXCEPTION ("could not read cropping information for depth stream. Reason: %s", xnGetStatusString (status));

    return (cropping.bEnabled != 0);
  }
  return (false);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void 
openni_wrapper::OpenNIDevice::setDepthCropping (unsigned x, unsigned y, unsigned width, unsigned height)
{
  if (hasDepthStream ())
  {
    boost::lock_guard<boost::mutex> depth_lock (depth_mutex_);
    XnCropping cropping;
    cropping.nXOffset = static_cast<XnUInt16> (x);
    cropping.nYOffset = static_cast<XnUInt16> (y);
    cropping.nXSize   = static_cast<XnUInt16> (width);
    cropping.nYSize   = static_cast<XnUInt16> (height);
    
    cropping.bEnabled = (width != 0 && height != 0);
    XnStatus status = depth_generator_.GetCroppingCap ().SetCropping (cropping);
    if (status != XN_STATUS_OK)
      THROW_OPENNI_EXCEPTION ("could not set cropping information for depth stream. Reason: %s", xnGetStatusString (status));
  }
  else
    THROW_OPENNI_EXCEPTION ("Device does not provide depth stream");
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void 
openni_wrapper::OpenNIDevice::ImageDataThreadFunction ()
{
  while (true)
  {
    // lock before checking running flag
    boost::unique_lock<boost::mutex> image_lock (image_mutex_);
    if (quit_)
      return;
    image_condition_.wait (image_lock);
    if (quit_)
      return;

    image_generator_.WaitAndUpdateData ();
    xn::ImageMetaData image_md;
    image_generator_.GetMetaData (image_md);
    boost::shared_ptr<xn::ImageMetaData> image_data (new xn::ImageMetaData);
    image_data->CopyFrom (image_md);
    image_lock.unlock ();
    
    boost::shared_ptr<Image> image = getCurrentImage (image_data);
    for (std::map< OpenNIDevice::CallbackHandle, ActualImageCallbackFunction >::iterator callbackIt = image_callback_.begin (); callbackIt != image_callback_.end (); ++callbackIt)
    {
      callbackIt->second.operator()(image);
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void 
openni_wrapper::OpenNIDevice::DepthDataThreadFunction ()
{
  while (true)
  {
    // lock before checking running flag
    boost::unique_lock<boost::mutex> depth_lock (depth_mutex_);
    if (quit_)
      return;
    depth_condition_.wait (depth_lock);
    if (quit_)
      return;

    depth_generator_.WaitAndUpdateData ();
    xn::DepthMetaData depth_md;
    depth_generator_.GetMetaData (depth_md);    
    boost::shared_ptr<xn::DepthMetaData> depth_data (new xn::DepthMetaData);
    depth_data->CopyFrom (depth_md);
    depth_lock.unlock ();

    boost::shared_ptr<DepthImage> depth_image ( new DepthImage (depth_data, baseline_, getDepthFocalLength (), shadow_value_, no_sample_value_) );

    for (std::map< OpenNIDevice::CallbackHandle, ActualDepthImageCallbackFunction >::iterator callbackIt = depth_callback_.begin ();
         callbackIt != depth_callback_.end (); ++callbackIt)
    {
      callbackIt->second.operator()(depth_image);
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void 
openni_wrapper::OpenNIDevice::IRDataThreadFunction ()
{
  while (true)
  {
    // lock before checking running flag
    boost::unique_lock<boost::mutex> ir_lock (ir_mutex_);
    if (quit_)
      return;
    ir_condition_.wait (ir_lock);
    if (quit_)
      return;

    ir_generator_.WaitAndUpdateData ();
    xn::IRMetaData ir_md;
    ir_generator_.GetMetaData (ir_md);
    boost::shared_ptr<xn::IRMetaData> ir_data (new xn::IRMetaData);
    ir_data->CopyFrom (ir_md);
    ir_lock.unlock ();

    boost::shared_ptr<IRImage> ir_image ( new IRImage (ir_data) );

    for (std::map< OpenNIDevice::CallbackHandle, ActualIRImageCallbackFunction >::iterator callbackIt = ir_callback_.begin ();
         callbackIt != ir_callback_.end (); ++callbackIt)
    {
      callbackIt->second.operator()(ir_image);
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void __stdcall 
openni_wrapper::OpenNIDevice::NewDepthDataAvailable (xn::ProductionNode&, void* cookie) throw ()
{
  OpenNIDevice* device = reinterpret_cast<OpenNIDevice*>(cookie);
  device->depth_condition_.notify_all ();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void __stdcall 
openni_wrapper::OpenNIDevice::NewImageDataAvailable (xn::ProductionNode&, void* cookie) throw ()
{
  OpenNIDevice* device = reinterpret_cast<OpenNIDevice*>(cookie);
  device->image_condition_.notify_all ();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void __stdcall 
openni_wrapper::OpenNIDevice::NewIRDataAvailable (xn::ProductionNode&, void* cookie) throw ()
{
  OpenNIDevice* device = reinterpret_cast<OpenNIDevice*>(cookie);
  device->ir_condition_.notify_all ();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
openni_wrapper::OpenNIDevice::CallbackHandle 
openni_wrapper::OpenNIDevice::registerImageCallback (const ImageCallbackFunction& callback, void* custom_data) throw ()
{
  image_callback_[image_callback_handle_counter_] = boost::bind (callback, _1, custom_data);
  return (image_callback_handle_counter_++);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool 
openni_wrapper::OpenNIDevice::unregisterImageCallback (const OpenNIDevice::CallbackHandle& callbackHandle) throw ()
{
  return (image_callback_.erase (callbackHandle) != 0);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
openni_wrapper::OpenNIDevice::CallbackHandle 
openni_wrapper::OpenNIDevice::registerDepthCallback (const DepthImageCallbackFunction& callback, void* custom_data) throw ()
{
  depth_callback_[depth_callback_handle_counter_] = boost::bind (callback, _1, custom_data);
  return (depth_callback_handle_counter_++);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool 
openni_wrapper::OpenNIDevice::unregisterDepthCallback (const OpenNIDevice::CallbackHandle& callbackHandle) throw ()
{
  return (depth_callback_.erase (callbackHandle) != 0);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
openni_wrapper::OpenNIDevice::CallbackHandle 
openni_wrapper::OpenNIDevice::registerIRCallback (const IRImageCallbackFunction& callback, void* custom_data) throw ()
{
  ir_callback_[ir_callback_handle_counter_] = boost::bind (callback, _1, custom_data);
  return (ir_callback_handle_counter_++);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool 
openni_wrapper::OpenNIDevice::unregisterIRCallback (const OpenNIDevice::CallbackHandle& callbackHandle) throw ()
{
  return (ir_callback_.erase (callbackHandle) != 0);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
const char* 
openni_wrapper::OpenNIDevice::getSerialNumber () const throw ()
{
  return (device_node_info_.GetInstanceName ());
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
const char* 
openni_wrapper::OpenNIDevice::getConnectionString () const throw ()
{
  return (device_node_info_.GetCreationInfo ());
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned short 
openni_wrapper::OpenNIDevice::getVendorID () const throw ()
{
  unsigned short vendor_id;
  unsigned short product_id;

#ifndef _WIN32
  unsigned char bus;
  unsigned char address;

  sscanf (device_node_info_.GetCreationInfo(), "%hx/%hx@%hhu/%hhu", &vendor_id, &product_id, &bus, &address);

#else
  OpenNIDriver::getDeviceType (device_node_info_.GetCreationInfo(), vendor_id, product_id);
#endif
  return (vendor_id);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned short 
openni_wrapper::OpenNIDevice::getProductID () const throw ()
{
  unsigned short vendor_id;
  unsigned short product_id;
#ifndef _WIN32
  unsigned char bus;
  unsigned char address;
  sscanf (device_node_info_.GetCreationInfo(), "%hx/%hx@%hhu/%hhu", &vendor_id, &product_id, &bus, &address);

#else
  OpenNIDriver::getDeviceType (device_node_info_.GetCreationInfo(), vendor_id, product_id);
#endif
  return (product_id);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned char 
openni_wrapper::OpenNIDevice::getBus () const throw ()
{
  unsigned char bus = 0;
#ifndef _WIN32
  unsigned short vendor_id;
  unsigned short product_id;
  unsigned char address;
  sscanf (device_node_info_.GetCreationInfo(), "%hx/%hx@%hhu/%hhu", &vendor_id, &product_id, &bus, &address);
#endif
  return (bus);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned char 
openni_wrapper::OpenNIDevice::getAddress () const throw ()
{
  unsigned char address = 0;
#ifndef _WIN32
  unsigned short vendor_id;
  unsigned short product_id;
  unsigned char bus;
  sscanf (device_node_info_.GetCreationInfo(), "%hx/%hx@%hhu/%hhu", &vendor_id, &product_id, &bus, &address);
#endif
  return (address);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
const char* 
openni_wrapper::OpenNIDevice::getVendorName () const throw ()
{
  XnProductionNodeDescription& description = const_cast<XnProductionNodeDescription&>(device_node_info_.GetDescription ());
  return (description.strVendor);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
const char* 
openni_wrapper::OpenNIDevice::getProductName () const throw ()
{
  XnProductionNodeDescription& description = const_cast<XnProductionNodeDescription&>(device_node_info_.GetDescription ());
  return (description.strName);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool 
openni_wrapper::OpenNIDevice::findCompatibleImageMode (const XnMapOutputMode& output_mode, XnMapOutputMode& mode) const throw ()
{
  if (isImageModeSupported (output_mode))
  {
    mode = output_mode;
    return (true);
  }
  else
  {
    bool found = false;
    for (std::vector<XnMapOutputMode>::const_iterator modeIt = available_image_modes_.begin (); modeIt != available_image_modes_.end (); ++modeIt)
    {
      if (modeIt->nFPS == output_mode.nFPS && isImageResizeSupported (modeIt->nXRes, modeIt->nYRes, output_mode.nXRes, output_mode.nYRes))
      {
        if (found)
        { // check wheter the new mode is better -> smaller than the current one.
          if (mode.nXRes * mode.nYRes > modeIt->nXRes * modeIt->nYRes )
            mode = *modeIt;
        }
        else
        {
          mode = *modeIt;
          found = true;
        }
      }
    }
    return (found);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool 
openni_wrapper::OpenNIDevice::findCompatibleDepthMode (const XnMapOutputMode& output_mode, XnMapOutputMode& mode) const throw ()
{
  if (isDepthModeSupported (output_mode))
  {
    mode = output_mode;
    return (true);
  }
  else
  {
    bool found = false;
    for (std::vector<XnMapOutputMode>::const_iterator modeIt = available_depth_modes_.begin (); modeIt != available_depth_modes_.end (); ++modeIt)
    {
      if (modeIt->nFPS == output_mode.nFPS && isImageResizeSupported (modeIt->nXRes, modeIt->nYRes, output_mode.nXRes, output_mode.nYRes))
      {
        if (found)
        { // check wheter the new mode is better -> smaller than the current one.
          if (mode.nXRes * mode.nYRes > modeIt->nXRes * modeIt->nYRes )
            mode = *modeIt;
        }
        else
        {
          mode = *modeIt;
          found = true;
        }
      }
    }
    return (found);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool 
openni_wrapper::OpenNIDevice::isImageModeSupported (const XnMapOutputMode& output_mode) const throw ()
{
  for (std::vector<XnMapOutputMode>::const_iterator modeIt = available_image_modes_.begin (); modeIt != available_image_modes_.end (); ++modeIt)
  {
    if (modeIt->nFPS == output_mode.nFPS && modeIt->nXRes == output_mode.nXRes && modeIt->nYRes == output_mode.nYRes)
      return (true);
  }
  return (false);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool 
openni_wrapper::OpenNIDevice::isDepthModeSupported (const XnMapOutputMode& output_mode) const throw ()
{
  for (std::vector<XnMapOutputMode>::const_iterator modeIt = available_depth_modes_.begin (); modeIt != available_depth_modes_.end (); ++modeIt)
  {
    if (modeIt->nFPS == output_mode.nFPS && modeIt->nXRes == output_mode.nXRes && modeIt->nYRes == output_mode.nYRes)
      return (true);
  }
  return (false);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
const XnMapOutputMode& 
openni_wrapper::OpenNIDevice::getDefaultImageMode () const throw ()
{
  return (available_image_modes_[0]);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
const XnMapOutputMode& 
openni_wrapper::OpenNIDevice::getDefaultDepthMode () const throw ()
{
  return (available_depth_modes_[0]);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
const XnMapOutputMode& 
openni_wrapper::OpenNIDevice::getDefaultIRMode () const throw ()
{
  /// @todo Something else here?
  return (available_depth_modes_[0]);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void 
openni_wrapper::OpenNIDevice::setImageOutputMode (const XnMapOutputMode& output_mode)
{
  if (hasImageStream ())
  {
    boost::lock_guard<boost::mutex> image_lock (image_mutex_);
    XnStatus status = image_generator_.SetMapOutputMode (output_mode);
    if (status != XN_STATUS_OK)
      THROW_OPENNI_EXCEPTION ("Could not set image stream output mode to %dx%d@%d. Reason: %s", output_mode.nXRes, output_mode.nYRes, output_mode.nFPS, xnGetStatusString (status));
  }
  else
    THROW_OPENNI_EXCEPTION ("Device does not provide an image stream");
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void 
openni_wrapper::OpenNIDevice::setDepthOutputMode (const XnMapOutputMode& output_mode)
{
  if (hasDepthStream ())
  {
    boost::lock_guard<boost::mutex> depth_lock (depth_mutex_);
    XnStatus status = depth_generator_.SetMapOutputMode (output_mode);
    if (status != XN_STATUS_OK)
      THROW_OPENNI_EXCEPTION ("Could not set depth stream output mode to %dx%d@%d. Reason: %s", output_mode.nXRes, output_mode.nYRes, output_mode.nFPS, xnGetStatusString (status));
  }
  else
    THROW_OPENNI_EXCEPTION ("Device does not provide a depth stream");
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void 
openni_wrapper::OpenNIDevice::setDepthOutputFormat (const DepthMode& depth_mode)
{
  if (hasDepthStream ())
  {
    boost::lock_guard<boost::mutex> depth_lock (depth_mutex_);
    XnStatus status = depth_generator_.SetIntProperty ("OutputFormat", depth_mode);
    if (status != 0)
      THROW_OPENNI_EXCEPTION ("Error setting the depth output format. Reason: %s", xnGetStatusString (status));
  }
  else
    THROW_OPENNI_EXCEPTION ("Device does not provide a depth stream");
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
XnUInt64 
openni_wrapper::OpenNIDevice::getDepthOutputFormat () const
{
  if (!hasDepthStream () )
    THROW_OPENNI_EXCEPTION ("Device does not provide a depth stream");

  boost::lock_guard<boost::mutex> depth_lock (depth_mutex_);
  XnUInt64 mode;
  XnStatus status = depth_generator_.GetIntProperty ("OutputFormat", mode);
  if (status != XN_STATUS_OK)
    THROW_OPENNI_EXCEPTION ("Could not get depth output format. Reason: %s", xnGetStatusString (status));
  return (mode);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void 
openni_wrapper::OpenNIDevice::setIROutputMode (const XnMapOutputMode& output_mode)
{
  if (hasIRStream ())
  {
    boost::lock_guard<boost::mutex> ir_lock (ir_mutex_);
    XnStatus status = ir_generator_.SetMapOutputMode (output_mode);
    if (status != XN_STATUS_OK)
      THROW_OPENNI_EXCEPTION ("Could not set IR stream output mode to %dx%d@%d. Reason: %s", output_mode.nXRes, output_mode.nYRes, output_mode.nFPS, xnGetStatusString (status));
  }
#ifndef __APPLE__
  else
    THROW_OPENNI_EXCEPTION ("Device does not provide an IR stream");
#endif
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
XnMapOutputMode 
openni_wrapper::OpenNIDevice::getImageOutputMode () const
{
  if (!hasImageStream ())
    THROW_OPENNI_EXCEPTION ("Device does not provide an image stream");

  XnMapOutputMode output_mode;
  boost::lock_guard<boost::mutex> image_lock (image_mutex_);
  XnStatus status = image_generator_.GetMapOutputMode (output_mode);
  if (status != XN_STATUS_OK)
    THROW_OPENNI_EXCEPTION ("Could not get image stream output mode. Reason: %s", xnGetStatusString (status));
  return (output_mode);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
XnMapOutputMode 
openni_wrapper::OpenNIDevice::getDepthOutputMode () const
{
  if (!hasDepthStream () )
    THROW_OPENNI_EXCEPTION ("Device does not provide a depth stream");

  XnMapOutputMode output_mode;
  boost::lock_guard<boost::mutex> depth_lock (depth_mutex_);
  XnStatus status = depth_generator_.GetMapOutputMode (output_mode);
  if (status != XN_STATUS_OK)
    THROW_OPENNI_EXCEPTION ("Could not get depth stream output mode. Reason: %s", xnGetStatusString (status));
  return (output_mode);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
XnMapOutputMode 
openni_wrapper::OpenNIDevice::getIROutputMode () const
{
  if (!hasIRStream ())
    THROW_OPENNI_EXCEPTION ("Device does not provide an IR stream");

  XnMapOutputMode output_mode;
  boost::lock_guard<boost::mutex> ir_lock (ir_mutex_);
  XnStatus status = ir_generator_.GetMapOutputMode (output_mode);
  if (status != XN_STATUS_OK)
    THROW_OPENNI_EXCEPTION ("Could not get IR stream output mode. Reason: %s", xnGetStatusString (status));
  return (output_mode);
}

#endif //OPENNI
