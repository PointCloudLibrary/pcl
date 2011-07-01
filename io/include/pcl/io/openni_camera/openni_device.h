/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011 Willow Garage, Inc.
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

#ifndef __OPENNI_IDEVICE_H__
#define __OPENNI_IDEVICE_H__
#include <map>
#include <vector>
#include <utility>
#include "openni_exception.h"
#include <XnCppWrapper.h>
#include <boost/noncopyable.hpp>
#include <boost/function.hpp>
#include <boost/thread.hpp>
#include <boost/thread/condition.hpp>

#include <pcl/pcl_macros.h>

/// @todo Get rid of all exception-specifications, these are useless and soon to be deprecated

#ifndef _WIN32
#define __stdcall
#endif

namespace openni_wrapper
{
class Image;
class DepthImage;
class IRImage;
/**
 * @brief Class representing an astract device for Primesense or MS Kinect devices.
 * @author Suat Gedikli
 * @date 02.january 2011
 * @ingroup io
 */
class PCL_EXPORTS OpenNIDevice : public boost::noncopyable
{
public:
  typedef boost::function<void(boost::shared_ptr<Image>, void* cookie) > ImageCallbackFunction;
  typedef boost::function<void(boost::shared_ptr<DepthImage>, void* cookie) > DepthImageCallbackFunction;
  typedef boost::function<void(boost::shared_ptr<IRImage>, void* cookie) > IRImageCallbackFunction;
  typedef unsigned CallbackHandle;

public:
  virtual ~OpenNIDevice () throw ();

  virtual bool findCompatibleImageMode (const XnMapOutputMode& output_mode, XnMapOutputMode& mode ) const throw (OpenNIException);
  virtual bool findCompatibleDepthMode (const XnMapOutputMode& output_mode, XnMapOutputMode& mode ) const throw (OpenNIException);

  virtual bool isImageModeSupported (const XnMapOutputMode& output_mode) const throw (OpenNIException);
  virtual bool isDepthModeSupported (const XnMapOutputMode& output_mode) const throw (OpenNIException);

  virtual const XnMapOutputMode& getDefaultImageMode () const throw ();
  virtual const XnMapOutputMode& getDefaultDepthMode () const throw ();
  virtual const XnMapOutputMode& getDefaultIRMode () const throw ();

  virtual void setImageOutputMode (const XnMapOutputMode& output_mode) throw (OpenNIException);
  virtual void setDepthOutputMode (const XnMapOutputMode& output_mode) throw (OpenNIException);
  virtual void setIROutputMode (const XnMapOutputMode& output_mode) throw (OpenNIException);

  XnMapOutputMode getImageOutputMode () const throw (OpenNIException);
  XnMapOutputMode getDepthOutputMode () const throw (OpenNIException);
  XnMapOutputMode getIROutputMode () const throw (OpenNIException);

  virtual void setDepthRegistration (bool on_off) throw (OpenNIException);
  bool isDepthRegistered () const throw (OpenNIException);
  virtual bool isDepthRegistrationSupported () const throw (OpenNIException);
  
  virtual void setSynchronization (bool on_off) throw (OpenNIException);
  virtual bool isSynchronized () const throw (OpenNIException);
  virtual bool isSynchronizationSupported () const throw ();

  // just supported by primesense -> virtual
  virtual bool isDepthCropped () const throw (OpenNIException);
  virtual void setDepthCropping (unsigned x, unsigned y, unsigned width, unsigned height) throw (OpenNIException);
  virtual bool isDepthCroppingSupported () const throw ();

  /** \brief returns the focal length for the color camera in pixels. The pixels are assumed to be square.
   Result depends on the output resolution of the image.
   */
  inline float getImageFocalLength (int output_x_resolution = 0) const throw ();

  /** \brief returns the focal length for the IR camera in pixels. The pixels are assumed to be square.
   Result depends on the output resolution of the depth image.
   */
  inline float getDepthFocalLength (int output_x_resolution = 0) const throw ();
  inline float getBaseline () const throw ();

  virtual void startImageStream () throw (OpenNIException);
  virtual void stopImageStream () throw (OpenNIException);

  virtual void startDepthStream () throw (OpenNIException);
  virtual void stopDepthStream () throw (OpenNIException);

  virtual void startIRStream () throw (OpenNIException);
  virtual void stopIRStream () throw (OpenNIException);

  bool hasImageStream () const throw ();
  bool hasDepthStream () const throw ();
  bool hasIRStream () const throw ();

  virtual bool isImageStreamRunning () const throw (OpenNIException);
  virtual bool isDepthStreamRunning () const throw (OpenNIException);
  virtual bool isIRStreamRunning () const throw (OpenNIException);

  CallbackHandle registerImageCallback (const ImageCallbackFunction& callback, void* cookie = NULL) throw ();
  template<typename T> CallbackHandle registerImageCallback (void (T::*callback)(boost::shared_ptr<Image>, void* cookie), T& instance, void* cookie = NULL) throw ();
  bool unregisterImageCallback (const CallbackHandle& callbackHandle) throw ();

  CallbackHandle registerDepthCallback (const DepthImageCallbackFunction& callback, void* cookie = NULL) throw ();
  template<typename T> CallbackHandle registerDepthCallback (void (T::*callback)(boost::shared_ptr<DepthImage>, void* cookie), T& instance, void* cookie = NULL) throw ();
  bool unregisterDepthCallback (const CallbackHandle& callbackHandle) throw ();

  CallbackHandle registerIRCallback (const IRImageCallbackFunction& callback, void* cookie = NULL) throw ();
  template<typename T> CallbackHandle registerIRCallback (void (T::*callback)(boost::shared_ptr<IRImage>, void* cookie), T& instance, void* cookie = NULL) throw ();
  bool unregisterIRCallback (const CallbackHandle& callbackHandle) throw ();

  /** \brief returns the serial number for device.
   *  \attention This might be an empty string!!!
   */
  const char* getSerialNumber () const throw ();
  /** \brief returns the connectionstring for current device, which has following format vendorID/productID\@BusID/DeviceID */
  const char* getConnectionString () const throw ();

  const char* getVendorName () const throw ();
  const char* getProductName () const throw ();
  unsigned short getVendorID () const throw ();
  unsigned short getProductID () const throw ();
  unsigned char  getBus () const throw ();
  unsigned char  getAddress () const throw ();
protected:
  typedef boost::function<void(boost::shared_ptr<Image>) > ActualImageCallbackFunction;
  typedef boost::function<void(boost::shared_ptr<DepthImage>) > ActualDepthImageCallbackFunction;
  typedef boost::function<void(boost::shared_ptr<IRImage>) > ActualIRImageCallbackFunction;

  OpenNIDevice (xn::Context& context, const xn::NodeInfo& device_node, const xn::NodeInfo& image_node, const xn::NodeInfo& depth_node, const xn::NodeInfo& ir_node) throw (OpenNIException);
  OpenNIDevice (xn::Context& context, const xn::NodeInfo& device_node, const xn::NodeInfo& depth_node, const xn::NodeInfo& ir_node) throw (OpenNIException);
  OpenNIDevice (xn::Context& context) throw (OpenNIException);
  static void __stdcall NewDepthDataAvailable (xn::ProductionNode& node, void* cookie) throw ();
  static void __stdcall NewImageDataAvailable (xn::ProductionNode& node, void* cookie) throw ();
  static void __stdcall NewIRDataAvailable (xn::ProductionNode& node, void* cookie) throw ();

  // This is a workaround, since in the NewDepthDataAvailable function WaitAndUpdateData leads to a dead-lock behaviour
  // and retrieving image data without WaitAndUpdateData leads to incomplete images!!!
  void ImageDataThreadFunction () throw (OpenNIException);
  void DepthDataThreadFunction () throw (OpenNIException);
  void IRDataThreadFunction () throw (OpenNIException);

  virtual bool isImageResizeSupported (unsigned input_width, unsigned input_height, unsigned output_width, unsigned output_height) const  throw () = 0;

  void setRegistration (bool on_off) throw (OpenNIException);
  virtual boost::shared_ptr<Image> getCurrentImage (boost::shared_ptr<xn::ImageMetaData> image_data) const throw () = 0;

  virtual void enumAvailableModes () throw (OpenNIException);
  void Init () throw (OpenNIException); 
  // holds the callback functions together with custom data
  // since same callback function can be registered multiple times with e.g. different custom data
  // we use a map structure with a handle as the key
  std::map< CallbackHandle, ActualImageCallbackFunction > image_callback_;
  std::map< CallbackHandle, ActualDepthImageCallbackFunction > depth_callback_;
  std::map< CallbackHandle, ActualIRImageCallbackFunction > ir_callback_;

  std::vector<XnMapOutputMode> available_image_modes_;
  std::vector<XnMapOutputMode> available_depth_modes_;

  /** \brief context to OpenNI driver*/
  xn::Context& context_;
  /** \brief node object for current device */
  xn::NodeInfo device_node_info_;
  
  /** \brief Depth generator object. */
  xn::DepthGenerator depth_generator_;
  /** \brief Image generator object. */
  xn::ImageGenerator image_generator_;
  /** \brief IR generator object. */
  xn::IRGenerator ir_generator_;

  XnCallbackHandle depth_callback_handle_;
  XnCallbackHandle image_callback_handle_;
  XnCallbackHandle ir_callback_handle_;

  /** \brief focal length for IR camera producing depth information in native SXGA mode */
  float depth_focal_length_SXGA_;
  /** \brief distance between the projector and the IR camera*/
  float baseline_;
  /** \brief focal length for regular camera producing color images in native SXGA mode*/
  static const float rgb_focal_length_SXGA_;

  /** the value for shadow (occluded pixels) */
  XnUInt64 shadow_value_;
  /** the value for pixels without a valid disparity measurement */
  XnUInt64 no_sample_value_;

  OpenNIDevice::CallbackHandle image_callback_handle_counter_;
  OpenNIDevice::CallbackHandle depth_callback_handle_counter_;
  OpenNIDevice::CallbackHandle ir_callback_handle_counter_;

  bool quit_;
  mutable boost::mutex image_mutex_;
  mutable boost::mutex depth_mutex_;
  mutable boost::mutex ir_mutex_;
  boost::condition_variable image_condition_;
  boost::condition_variable depth_condition_;
  boost::condition_variable ir_condition_;
  boost::thread image_thread_;
  boost::thread depth_thread_;
  boost::thread ir_thread_;
};

float OpenNIDevice::getImageFocalLength (int output_x_resolution) const throw ()
{
  if (output_x_resolution == 0)
    output_x_resolution = getImageOutputMode ().nXRes;

  float scale = output_x_resolution / (float)XN_SXGA_X_RES;
  return rgb_focal_length_SXGA_ * scale;
}

float OpenNIDevice::getDepthFocalLength (int output_x_resolution) const throw ()
{
  if (output_x_resolution == 0)
    output_x_resolution = getDepthOutputMode ().nXRes;

  float scale = output_x_resolution / (float)XN_SXGA_X_RES;
  if (isDepthRegistered ())
    return rgb_focal_length_SXGA_ * scale;
  else
    return depth_focal_length_SXGA_ * scale;
}

float OpenNIDevice::getBaseline () const throw ()
{
  return baseline_;
}

template<typename T> OpenNIDevice::CallbackHandle OpenNIDevice::registerImageCallback (void (T::*callback)(boost::shared_ptr<Image>, void* cookie), T& instance, void* custom_data) throw ()
{
  image_callback_[image_callback_handle_counter_] = boost::bind (callback, boost::ref (instance), _1, custom_data);
  return image_callback_handle_counter_++;
}

template<typename T> OpenNIDevice::CallbackHandle OpenNIDevice::registerDepthCallback (void (T::*callback)(boost::shared_ptr<DepthImage>, void* cookie), T& instance, void* custom_data) throw ()
{
  depth_callback_[depth_callback_handle_counter_] = boost::bind ( callback,  boost::ref (instance), _1, custom_data);
  return depth_callback_handle_counter_++;
}

template<typename T> OpenNIDevice::CallbackHandle OpenNIDevice::registerIRCallback (void (T::*callback)(boost::shared_ptr<IRImage>, void* cookie), T& instance, void* custom_data) throw ()
{
  ir_callback_[ir_callback_handle_counter_] = boost::bind ( callback,  boost::ref (instance), _1, custom_data);
  return ir_callback_handle_counter_++;
}

}
#endif // __OPENNI_IDEVICE_H__
#endif // HAVE_OPENNI
