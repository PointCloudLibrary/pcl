/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009-2011, Willow Garage, Inc.
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

#include <pcl/pcl_config.h>
#include <pcl/memory.h>
#ifdef HAVE_OPENNI

#include "openni_exception.h"
#include "openni.h"

#include <pcl/io/openni_camera/openni_image.h>
#include <pcl/io/openni_camera/openni_depth_image.h>
#include <pcl/io/openni_camera/openni_ir_image.h>
#include <pcl/pcl_macros.h>

#include <condition_variable>
#include <functional>
#include <map>
#include <mutex>
#include <thread>
#include <vector>

/// @todo Get rid of all exception-specifications, these are useless and soon to be deprecated

#ifndef _WIN32
#define __stdcall
#endif

namespace openni_wrapper
{
  /** \brief Class representing an astract device for OpenNI devices: Primesense PSDK, Microsoft Kinect, Asus Xtion Pro/Live.
    * \author Suat Gedikli
    * \ingroup io
    */
  class PCL_EXPORTS OpenNIDevice
  {
    public:
      enum DepthMode
      {
        OpenNI_shift_values = 0, // Shift values (disparity)
        OpenNI_12_bit_depth = 1, // Default mode: regular 12-bit depth
      };

      using Ptr = pcl::shared_ptr<OpenNIDevice>;
      using ConstPtr = pcl::shared_ptr<const OpenNIDevice>;

      using ImageCallbackFunction = std::function<void(Image::Ptr, void* cookie) >;
      using DepthImageCallbackFunction = std::function<void(DepthImage::Ptr, void* cookie) >;
      using IRImageCallbackFunction = std::function<void(IRImage::Ptr, void* cookie) >;
      using CallbackHandle = unsigned;

    public:

      /** \brief virtual destructor. Never throws an exception. */
      virtual ~OpenNIDevice () noexcept;

      /** \brief finds an image output mode that can be used to retrieve images in desired output mode.
        *        e.g If device just supports VGA at 30Hz, then the desired mode QVGA at 30Hz would be possible by down sampling,
        *        but the modes VGA at 25Hz and SXGA at 30Hz would not be compatible.
        * \param[in] output_mode the desired output mode
        * \param[out] mode the compatible mode that the device natively supports.
        * \return true, if a compatible mode could be found, false otherwise.
        */
      bool 
      findCompatibleImageMode (const XnMapOutputMode& output_mode, XnMapOutputMode& mode ) const throw ();

      /** \brief finds a depth output mode that can be used to retrieve depth images in desired output mode.
        *        e.g If device just supports VGA at 30Hz, then a desired mode of QVGA at 30Hz would be possbile by downsampling,
        *        but the modes VGA at 25Hz and SXGA at 30Hz would not be compatible.
        * \param[in] output_mode the desired output mode
        * \param[out] mode the compatible mode that the device natively supports.
        * \return true, if a compatible mode could be found, false otherwise.
        */
      bool 
      findCompatibleDepthMode (const XnMapOutputMode& output_mode, XnMapOutputMode& mode ) const throw ();

      /** \brief returns whether a given mode is natively supported by the device or not
        * \param[in] output_mode mode to be checked
        * \return true if mode natively available, false otherwise
        */
      bool 
      isImageModeSupported (const XnMapOutputMode& output_mode) const throw ();

      /** \brief returns whether a given mode is natively supported by the device or not
        * \param[in] output_mode mode to be checked
        * \return true if mode natively available, false otherwise
        */
      bool 
      isDepthModeSupported (const XnMapOutputMode& output_mode) const throw ();

      /** \brief returns the default image mode, which is simply the first entry in the list of modes
        * \return the default image mode
        */
      const XnMapOutputMode& 
      getDefaultImageMode () const throw ();

      /** \brief  returns the default depth mode, which is simply the first entry in the list of modes
        * \return the default depth mode
        */
      const XnMapOutputMode& 
      getDefaultDepthMode () const throw ();

      /** \brief  returns the default IR mode, which is simply the first entry in the list of modes
        * \return the default IR mode
        */
      const XnMapOutputMode& 
      getDefaultIRMode () const throw ();

      /** \brief sets the output mode of the image stream
        * \param[in] output_mode the desired output mode
        */
      void 
      setImageOutputMode (const XnMapOutputMode& output_mode);

      /** \brief sets the output mode of the depth stream
        * \param[in] output_mode the desired output mode
        */
      void 
      setDepthOutputMode (const XnMapOutputMode& output_mode);

      /** \brief sets the output mode of the IR stream
        * \param[in] output_mode the desired output mode
        */
      void 
      setIROutputMode (const XnMapOutputMode& output_mode);

      /** \return the current output mode of the image stream */
      XnMapOutputMode 
      getImageOutputMode () const;

      /** \return the current output mode of the depth stream */
      XnMapOutputMode 
      getDepthOutputMode () const;

      /** \return the current output mode of the IR stream */
      XnMapOutputMode 
      getIROutputMode () const;

      /** \brief set the depth stream registration on or off
        * \param[in] on_off
        */
      void 
      setDepthRegistration (bool on_off);

      /** \return whether the depth stream is registered to the RGB camera fram or not. */
      bool 
      isDepthRegistered () const throw ();

      /** \return whether a registration of the depth stream to the RGB camera frame is supported or not. */
      bool 
      isDepthRegistrationSupported () const throw ();

      /** \brief set the hardware synchronization between Depth and RGB stream on or off.
        * \param[in] on_off
        */
      void 
      setSynchronization (bool on_off);

      /** \return true if Depth stream is synchronized to RGB stream, false otherwise. */
      bool 
      isSynchronized () const throw ();

      /** \return true if the Device supports hardware synchronization between Depth and RGB streams or not. */ 
      virtual bool 
      isSynchronizationSupported () const throw ();

      /** \return true if depth stream is a cropped version of the native depth stream, false otherwise. */
      bool 
      isDepthCropped () const;

      /** \brief turn on cropping for the depth stream.
        * \param[in] x x-position of the rectangular subregion.
        * \param[in] y y-position of the rectangular subregion.
        * \param[in] width width of the rectangular subregion.
        * \param[in] height height of the rectangular subregion.
        */
      void 
      setDepthCropping (unsigned x, unsigned y, unsigned width, unsigned height);

      /** \return true if cropping of the depth stream is supported, false otherwise. */
      bool 
      isDepthCroppingSupported () const throw ();

      /** \brief returns the focal length for the color camera in pixels. The pixels are assumed to be square.
        *        Result depends on the output resolution of the image.
        */
      inline float 
      getImageFocalLength (int output_x_resolution = 0) const throw ();

      /** \brief returns the focal length for the IR camera in pixels. The pixels are assumed to be square.
        *        Result depends on the output resolution of the depth image.
        */
      inline float 
      getDepthFocalLength (int output_x_resolution = 0) const throw ();

      /** \return Baseline of the "stereo" frame. i.e. for PSDK compatible devices its the distance between the Projector and the IR camera. */
      inline float 
      getBaseline () const throw ();

      /** \brief starts the image stream. */
      virtual void 
      startImageStream ();

      /** \brief stops the image stream. */
      virtual void 
      stopImageStream ();

      /** \brief starts the depth stream. */
      virtual void 
      startDepthStream ();

      /** \brief stops the depth stream. */
      virtual void 
      stopDepthStream ();

      /** \brief starts the IR stream. */
      virtual void 
      startIRStream ();

      /** \brief stops the IR stream. */
      virtual void 
      stopIRStream ();

      /** \return true if the device supports an image stream, false otherwise. */
      bool 
      hasImageStream () const throw ();

      /** \return true if the device supports a depth stream, false otherwise. */
      bool 
      hasDepthStream () const throw ();

      /** \return true if the device supports an IR stream, false otherwise. */
      bool 
      hasIRStream () const throw ();

      /** \return true if the image stream is running / started, false otherwise. */
      virtual bool 
      isImageStreamRunning () const throw ();

      /** \return true if the depth stream is running / started, false otherwise. */
      virtual bool 
      isDepthStreamRunning () const throw ();

      /** \return true if the IR stream is running / started, false otherwise. */
      virtual bool 
      isIRStreamRunning () const throw ();

      /** \brief registers a callback function of std::function type for the image stream with an optional user defined parameter.
        *        The callback will always be called with a new image and the user data "cookie".
        * \param[in] callback the user callback to be called if a new image is available
        * \param[in] cookie the cookie that needs to be passed to the callback together with the new image.
        * \return a callback handler that can be used to remove the user callback from list of image-stream callbacks.
        */
      CallbackHandle 
      registerImageCallback (const ImageCallbackFunction& callback, void* cookie = nullptr) noexcept;

      /** \brief registers a callback function for the image stream with an optional user defined parameter.
        *        This version is used to register a member function of any class.
        *        The callback will always be called with a new image and the user data "cookie".
        * \param[in] callback the user callback to be called if a new image is available
        * \param instance
        * \param[in] cookie the cookie that needs to be passed to the callback together with the new image.
        * \return a callback handler that can be used to remove the user callback from list of image-stream callbacks.
        */
      template<typename T> CallbackHandle 
      registerImageCallback (void (T::*callback)(Image::Ptr, void* cookie), T& instance, void* cookie = nullptr) noexcept;

      /** \brief unregisters a callback function. i.e. removes that function from the list of image stream callbacks.
        * \param[in] callbackHandle the handle of the callback to unregister.
        * \return true, if callback was in list and could be unregistered, false otherwise.
        */
      bool 
      unregisterImageCallback (const CallbackHandle& callbackHandle) noexcept;


      /** \brief registers a callback function of std::function type for the depth stream with an optional user defined parameter.
        *        The callback will always be called with a new depth image and the user data "cookie".
        * \param[in] callback the user callback to be called if a new depth image is available
        * \param[in] cookie the cookie that needs to be passed to the callback together with the new depth image.
        * \return a callback handler that can be used to remove the user callback from list of depth-stream callbacks.
        */
      CallbackHandle 
      registerDepthCallback (const DepthImageCallbackFunction& callback, void* cookie = nullptr) noexcept;

      /** \brief registers a callback function for the depth stream with an optional user defined parameter.
        *        This version is used to register a member function of any class.
        *        The callback will always be called with a new depth image and the user data "cookie".
        * \param[in] callback the user callback to be called if a new depth image is available
        * \param instance
        * \param[in] cookie the cookie that needs to be passed to the callback together with the new depth image.
        * \return a callback handler that can be used to remove the user callback from list of depth-stream callbacks.
        */
      template<typename T> CallbackHandle 
      registerDepthCallback (void (T::*callback)(DepthImage::Ptr, void* cookie), T& instance, void* cookie = nullptr) noexcept;

      /** \brief unregisters a callback function. i.e. removes that function from the list of depth stream callbacks.
        * \param[in] callbackHandle the handle of the callback to unregister.
        * \return true, if callback was in list and could be unregistered, false otherwise.
        */
      bool 
      unregisterDepthCallback (const CallbackHandle& callbackHandle) noexcept;

      /** \brief registers a callback function of std::function type for the IR stream with an optional user defined parameter.
        *        The callback will always be called with a new IR image and the user data "cookie".
        * \param[in] callback the user callback to be called if a new IR image is available
        * \param[in] cookie the cookie that needs to be passed to the callback together with the new IR image.
        * \return a callback handler that can be used to remove the user callback from list of IR-stream callbacks.
        */
      CallbackHandle 
      registerIRCallback (const IRImageCallbackFunction& callback, void* cookie = nullptr) noexcept;

      /** \brief registers a callback function for the IR stream with an optional user defined parameter.
        *        This version is used to register a member function of any class.
        *        The callback will always be called with a new IR image and the user data "cookie".
        * \param[in] callback the user callback to be called if a new IR image is available
        * \param instance
        * \param[in] cookie the cookie that needs to be passed to the callback together with the new IR image.
        * \return a callback handler that can be used to remove the user callback from list of IR-stream callbacks.
        */
      template<typename T> CallbackHandle 
      registerIRCallback (void (T::*callback)(IRImage::Ptr, void* cookie), T& instance, void* cookie = nullptr) noexcept;

      /** \brief unregisters a callback function. i.e. removes that function from the list of IR stream callbacks.
        * \param[in] callbackHandle the handle of the callback to unregister.
        * \return true, if callback was in list and could be unregistered, false otherwise.
        */
      bool 
      unregisterIRCallback (const CallbackHandle& callbackHandle) noexcept;

      /** \brief returns the serial number for device.
        * \attention This might be an empty string!!!
        */
      const char* 
      getSerialNumber () const throw ();

      /** \brief returns the connection string for current device, which has following format vendorID/productID\@BusID/DeviceID. */
      const char* 
      getConnectionString () const throw ();

      /** \return the Vendor name of the USB device. */
      const char* 
      getVendorName () const throw ();

      /** \return the product name of the USB device. */
      const char* 
      getProductName () const throw ();

      /** \return the vendor ID of the USB device. */
      unsigned short 
      getVendorID () const throw ();

      /** \return the product ID of the USB device. */
      unsigned short 
      getProductID () const throw ();

      /** \return the USB bus on which the device is connected. */
      unsigned char  
      getBus () const throw ();

      /** \return the USB Address of the device. */
      unsigned char  
      getAddress () const throw ();

      /** \brief Set the RGB image focal length.
        * \param[in] focal_length the RGB image focal length
        */
      inline void
      setRGBFocalLength (float focal_length)
      {
        rgb_focal_length_SXGA_ = focal_length;
      }

      /** \brief Set the depth image focal length.
        * \param[in] focal_length the depth image focal length
        */
      inline void
      setDepthFocalLength (float focal_length)
      {
        depth_focal_length_SXGA_ = focal_length;
      }

      /** \brief Set the depth output format. Use 12bit depth values or shift values.
        * \param[in] depth_mode the depth output format
        */
      void
      setDepthOutputFormat (const DepthMode& depth_mode = OpenNI_12_bit_depth);

      /** \brief Get the depth output format as set by the user. */
      XnUInt64 
      getDepthOutputFormat () const;


      /** \brief Convert shift to depth value. */
      std::uint16_t
      shiftToDepth (std::uint16_t shift_value) const
      {
        assert (shift_conversion_parameters_.init_);

        std::uint16_t ret = 0;

        // lookup depth value in shift lookup table
        if (shift_value<shift_to_depth_table_.size())
          ret = shift_to_depth_table_[shift_value];

        return ret;
      }

    private:
      // make OpenNIDevice non copyable
      OpenNIDevice (OpenNIDevice const &);
      OpenNIDevice& operator=(OpenNIDevice const &);
    protected:
      using ActualImageCallbackFunction = std::function<void(Image::Ptr) >;
      using ActualDepthImageCallbackFunction = std::function<void(DepthImage::Ptr) >;
      using ActualIRImageCallbackFunction = std::function<void(IRImage::Ptr) >;

      OpenNIDevice (xn::Context& context, const xn::NodeInfo& device_node, const xn::NodeInfo& image_node, const xn::NodeInfo& depth_node, const xn::NodeInfo& ir_node);
      OpenNIDevice (xn::Context& context, const xn::NodeInfo& device_node, const xn::NodeInfo& depth_node, const xn::NodeInfo& ir_node);
      OpenNIDevice (xn::Context& context);
      static void __stdcall NewDepthDataAvailable (xn::ProductionNode& node, void* cookie) noexcept;
      static void __stdcall NewImageDataAvailable (xn::ProductionNode& node, void* cookie) noexcept;
      static void __stdcall NewIRDataAvailable (xn::ProductionNode& node, void* cookie) noexcept;

      // This is a workaround, since in the NewDepthDataAvailable function WaitAndUpdateData leads to a dead-lock behaviour
      // and retrieving image data without WaitAndUpdateData leads to incomplete images!!!
      void 
      ImageDataThreadFunction ();

      void 
      DepthDataThreadFunction ();

      void 
      IRDataThreadFunction ();

      virtual bool 
      isImageResizeSupported (unsigned input_width, unsigned input_height, unsigned output_width, unsigned output_height) const  throw () = 0;

      void 
      setRegistration (bool on_off);

      virtual Image::Ptr
      getCurrentImage (pcl::shared_ptr<xn::ImageMetaData> image_data) const throw () = 0;

      void 
      Init ();

      void InitShiftToDepthConversion();
      void ReadDeviceParametersFromSensorNode();

      struct ShiftConversion
      {
        ShiftConversion() : init_(false) {}

        XnUInt16 zero_plane_distance_;
        XnFloat zero_plane_pixel_size_;
        XnFloat emitter_dcmos_distace_;
        XnUInt32 max_shift_;
        XnUInt32 device_max_shift_;
        XnUInt32 const_shift_;
        XnUInt32 pixel_size_factor_;
        XnUInt32 param_coeff_;
        XnUInt32 shift_scale_;
        XnUInt32 min_depth_;
        XnUInt32 max_depth_;
        bool init_;

      } shift_conversion_parameters_;

      std::vector<std::uint16_t> shift_to_depth_table_;

      // holds the callback functions together with custom data
      // since same callback function can be registered multiple times with e.g. different custom data
      // we use a map structure with a handle as the key
      std::map<CallbackHandle, ActualImageCallbackFunction> image_callback_;
      std::map<CallbackHandle, ActualDepthImageCallbackFunction> depth_callback_;
      std::map<CallbackHandle, ActualIRImageCallbackFunction> ir_callback_;

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
      /** \brief focal length for regular camera producing color images in native SXGA mode */
      float rgb_focal_length_SXGA_;

      /** the value for shadow (occluded pixels) */
      XnUInt64 shadow_value_;
      /** the value for pixels without a valid disparity measurement */
      XnUInt64 no_sample_value_;

      OpenNIDevice::CallbackHandle image_callback_handle_counter_;
      OpenNIDevice::CallbackHandle depth_callback_handle_counter_;
      OpenNIDevice::CallbackHandle ir_callback_handle_counter_;

      bool quit_;
      mutable std::mutex image_mutex_;
      mutable std::mutex depth_mutex_;
      mutable std::mutex ir_mutex_;
      std::condition_variable image_condition_;
      std::condition_variable depth_condition_;
      std::condition_variable ir_condition_;
      std::thread image_thread_;
      std::thread depth_thread_;
      std::thread ir_thread_;
  };

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  float
  OpenNIDevice::getImageFocalLength (int output_x_resolution) const throw ()
  {
    if (output_x_resolution == 0)
      output_x_resolution = getImageOutputMode ().nXRes;

    float scale = static_cast<float> (output_x_resolution) / static_cast<float> (XN_SXGA_X_RES);
    return (rgb_focal_length_SXGA_ * scale);
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  float
  OpenNIDevice::getDepthFocalLength (int output_x_resolution) const throw ()
  {
    if (output_x_resolution == 0)
      output_x_resolution = getDepthOutputMode ().nXRes;

    float scale = static_cast<float> (output_x_resolution) / static_cast<float> (XN_SXGA_X_RES);
    if (isDepthRegistered ())
      return (rgb_focal_length_SXGA_ * scale);
    return (depth_focal_length_SXGA_ * scale);
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  float
  OpenNIDevice::getBaseline () const throw ()
  {
    return (baseline_);
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  template<typename T> OpenNIDevice::CallbackHandle
  OpenNIDevice::registerImageCallback (void (T::*callback)(Image::Ptr, void* cookie), T& instance, void* custom_data) noexcept
  {
    image_callback_[image_callback_handle_counter_] = [=, &instance] (Image::Ptr img) { (instance.*callback) (img, custom_data); };
    return (image_callback_handle_counter_++);
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  template<typename T> OpenNIDevice::CallbackHandle
  OpenNIDevice::registerDepthCallback (void (T::*callback)(DepthImage::Ptr, void* cookie), T& instance, void* custom_data) noexcept
  {
    depth_callback_[depth_callback_handle_counter_] = [=, &instance] (DepthImage::Ptr img) { (instance.*callback) (img, custom_data); };
    return (depth_callback_handle_counter_++);
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  template<typename T> OpenNIDevice::CallbackHandle
  OpenNIDevice::registerIRCallback (void (T::*callback)(IRImage::Ptr, void* cookie), T& instance, void* custom_data) noexcept
  {
    ir_callback_[ir_callback_handle_counter_] = [=, &instance] (IRImage::Ptr img) { (instance.*callback) (img, custom_data); };
    return (ir_callback_handle_counter_++);
  }

}
#endif // HAVE_OPENNI
