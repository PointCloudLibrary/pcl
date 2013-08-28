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

#ifndef OPENNI2_DEVICE_H
#define OPENNI2_DEVICE_H

#include <pcl/pcl_exports.h>
#include "openni.h"
#include "pcl/io/openni2_camera/openni2_video_mode.h"
#include "pcl/io/openni2_camera/openni2_exception.h"

#include <boost/shared_ptr.hpp>
#include <boost/cstdint.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>

// Frame conversion classes from openni 1.x wrapper
#include <pcl/io/openni2_camera/openni_image.h>
#include <pcl/io/openni2_camera/openni_depth_image.h>
#include <pcl/io/openni2_camera/openni_ir_image.h>
#include <pcl/io/openni2_camera/openni_image_yuv_422.h>
#include <pcl/io/openni2_camera/openni_image_rgb24.h>

#include <string>
#include <vector>

using openni_wrapper::Image;
using openni_wrapper::DepthImage;
using openni_wrapper::IRImage;

namespace openni
{
	class Device;
	class DeviceInfo;
	class VideoStream;
	class SensorInfo;
}

namespace openni2_wrapper
{


	class OpenNI2FrameListener;

	class PCL_EXPORTS OpenNI2Device
	{
	public:

		// OpenNI 1.x interface
		typedef boost::function<void(boost::shared_ptr<Image>, void* cookie) > ImageCallbackFunction;
		typedef boost::function<void(boost::shared_ptr<DepthImage>, void* cookie) > DepthImageCallbackFunction;
		typedef boost::function<void(boost::shared_ptr<IRImage>, void* cookie) > IRImageCallbackFunction;
		typedef unsigned CallbackHandle;

		OpenNI2Device(const std::string& device_URI) throw (OpenNI2Exception);
		virtual ~OpenNI2Device();

		const std::string getUri() const;
		const std::string getVendor() const;
		const std::string getName() const;
		uint16_t getUsbVendorId() const;
		uint16_t getUsbProductId() const;

		const std::string getStringID() const;

		bool isValid() const;

		bool hasIRSensor() const;
		bool hasColorSensor() const;
		bool hasDepthSensor() const;

		void startIRStream();
		void startColorStream();
		void startDepthStream();

		void stopAllStreams();

		void stopIRStream();
		void stopColorStream();
		void stopDepthStream();

		bool isIRStreamStarted();
		bool isColorStreamStarted();
		bool isDepthStreamStarted();

		bool isImageRegistrationModeSupported() const;
		void setImageRegistrationMode(bool enabled) throw (OpenNI2Exception);
		void setDepthColorSync(bool enabled) throw (OpenNI2Exception);

		const OpenNI2VideoMode getIRVideoMode() throw (OpenNI2Exception);
		const OpenNI2VideoMode getColorVideoMode() throw (OpenNI2Exception);
		const OpenNI2VideoMode getDepthVideoMode() throw (OpenNI2Exception);

		const std::vector<OpenNI2VideoMode>& getSupportedIRVideoModes() const;
		const std::vector<OpenNI2VideoMode>& getSupportedColorVideoModes() const;
		const std::vector<OpenNI2VideoMode>& getSupportedDepthVideoModes() const;

		bool isIRVideoModeSupported(const OpenNI2VideoMode& video_mode) const;
		bool isColorVideoModeSupported(const OpenNI2VideoMode& video_mode) const;
		bool isDepthVideoModeSupported(const OpenNI2VideoMode& video_mode) const;

		bool findCompatibleIRMode(const OpenNI2VideoMode& check_mode, OpenNI2VideoMode& mode) const throw (OpenNI2Exception);
		bool findCompatibleColorMode(const OpenNI2VideoMode& check_mode, OpenNI2VideoMode& mode) const throw (OpenNI2Exception);
		bool findCompatibleDepthMode(const OpenNI2VideoMode& check_mode, OpenNI2VideoMode& mode) const throw (OpenNI2Exception);

		void setIRVideoMode(const OpenNI2VideoMode& video_mode) throw (OpenNI2Exception);
		void setColorVideoMode(const OpenNI2VideoMode& video_mode) throw (OpenNI2Exception);
		void setDepthVideoMode(const OpenNI2VideoMode& video_mode) throw (OpenNI2Exception);

		OpenNI2VideoMode getDefaultIRMode() const;
		OpenNI2VideoMode getDefaultColorMode() const;
		OpenNI2VideoMode getDefaultDepthMode() const;

		//void setIRFrameCallback(FrameCallbackFunction callback);
		//void setColorFrameCallback(FrameCallbackFunction callback);
		//void setDepthFrameCallback(FrameCallbackFunction callback);

		float getIRFocalLength (int output_y_resolution) const;
		float getColorFocalLength (int output_y_resolution) const;
		float getDepthFocalLength (int output_y_resolution) const;

		void setAutoExposure(bool enable) throw (OpenNI2Exception);
		void setAutoWhiteBalance(bool enable) throw (OpenNI2Exception);

		bool isSynchronized() { return false; };
		bool isSynchronizationSupported(){ return false; }
		void setSynchronization(bool sync) { setDepthColorSync(true); }

		bool getAutoExposure() const;
		bool getAutoWhiteBalance() const;

		void setUseDeviceTimer(bool enable);

		/************************************************************************************/
		// ***** PCL callbacks, for compatibility with the OpenNI 1.x grabber interface *****
	
		/** \brief registers a callback function of boost::function type for the image stream with an optional user defined parameter.
		*        The callback will always be called with a new image and the user data "cookie".
		* \param[in] callback the user callback to be called if a new image is available
		* \param[in] cookie the cookie that needs to be passed to the callback together with the new image.
		* \return a callback handler that can be used to remove the user callback from list of image-stream callbacks.
		*/
		CallbackHandle 
		registerImageCallback (const ImageCallbackFunction& callback, void* cookie = NULL) throw ();

		/** \brief registers a callback function for the image stream with an optional user defined parameter.
		*        This version is used to register a member function of any class.
		*        The callback will always be called with a new image and the user data "cookie".
		* \param[in] callback the user callback to be called if a new image is available
		* \param[in] cookie the cookie that needs to be passed to the callback together with the new image.
		* \return a callback handler that can be used to remove the user callback from list of image-stream callbacks.
		*/
		template<typename T> CallbackHandle 
		registerImageCallback (void (T::*callback)(boost::shared_ptr<Image>, void* cookie), T& instance, void* cookie = NULL) throw ();

		/** \brief unregisters a callback function. i.e. removes that function from the list of image stream callbacks.
		* \param[in] callbackHandle the handle of the callback to unregister.
		* \return true, if callback was in list and could be unregistered, false otherwise.
		*/
		bool 
		unregisterImageCallback (const CallbackHandle& callbackHandle) throw ();


		/** \brief registers a callback function of boost::function type for the depth stream with an optional user defined parameter.
		*        The callback will always be called with a new depth image and the user data "cookie".
		* \param[in] callback the user callback to be called if a new depth image is available
		* \param[in] cookie the cookie that needs to be passed to the callback together with the new depth image.
		* \return a callback handler that can be used to remove the user callback from list of depth-stream callbacks.
		*/
		CallbackHandle 
		registerDepthCallback (const DepthImageCallbackFunction& callback, void* cookie = NULL) throw ();

		/** \brief registers a callback function for the depth stream with an optional user defined parameter.
		*        This version is used to register a member function of any class.
		*        The callback will always be called with a new depth image and the user data "cookie".
		* \param[in] callback the user callback to be called if a new depth image is available
		* \param[in] cookie the cookie that needs to be passed to the callback together with the new depth image.
		* \return a callback handler that can be used to remove the user callback from list of depth-stream callbacks.
		*/
		template<typename T> CallbackHandle 
		registerDepthCallback (void (T::*callback)(boost::shared_ptr<DepthImage>, void* cookie), T& instance, void* cookie = NULL) throw ();

		/** \brief unregisters a callback function. i.e. removes that function from the list of depth stream callbacks.
		* \param[in] callbackHandle the handle of the callback to unregister.
		* \return true, if callback was in list and could be unregistered, false otherwise.
		*/
		bool 
		unregisterDepthCallback (const CallbackHandle& callbackHandle) throw ();

		/** \brief registers a callback function of boost::function type for the IR stream with an optional user defined parameter.
		*        The callback will always be called with a new IR image and the user data "cookie".
		* \param[in] callback the user callback to be called if a new IR image is available
		* \param[in] cookie the cookie that needs to be passed to the callback together with the new IR image.
		* \return a callback handler that can be used to remove the user callback from list of IR-stream callbacks.
		*/
		CallbackHandle 
		registerIRCallback (const IRImageCallbackFunction& callback, void* cookie = NULL) throw ();

		/** \brief registers a callback function for the IR stream with an optional user defined parameter.
		*        This version is used to register a member function of any class.
		*        The callback will always be called with a new IR image and the user data "cookie".
		* \param[in] callback the user callback to be called if a new IR image is available
		* \param[in] cookie the cookie that needs to be passed to the callback together with the new IR image.
		* \return a callback handler that can be used to remove the user callback from list of IR-stream callbacks.
		*/
		template<typename T> CallbackHandle 
		registerIRCallback (void (T::*callback)(boost::shared_ptr<IRImage>, void* cookie), T& instance, void* cookie = NULL) throw ();

		/** \brief unregisters a callback function. i.e. removes that function from the list of IR stream callbacks.
		* \param[in] callbackHandle the handle of the callback to unregister.
		* \return true, if callback was in list and could be unregistered, false otherwise.
		*/
		bool 
		unregisterIRCallback (const CallbackHandle& callbackHandle) throw ();

	protected:
		void shutdown();

		boost::shared_ptr<openni::VideoStream> getIRVideoStream() const throw (OpenNI2Exception);
		boost::shared_ptr<openni::VideoStream> getColorVideoStream() const throw (OpenNI2Exception);
		boost::shared_ptr<openni::VideoStream> getDepthVideoStream() const throw (OpenNI2Exception);

		void processColorFrame(openni::VideoFrameRef& image);
		void processDepthFrame(openni::VideoFrameRef& image);
		void processIRFrame(openni::VideoFrameRef& image);

		bool findCompatibleVideoMode (const std::vector<OpenNI2VideoMode> supportedModes, 
			const OpenNI2VideoMode& output_mode, OpenNI2VideoMode& mode) const;
		bool resizingSupported (size_t input_width, size_t input_height, size_t output_width, size_t output_height) const;

		// Members

		boost::shared_ptr<openni::Device> openni_device_;
		boost::shared_ptr<openni::DeviceInfo> device_info_;

		boost::shared_ptr<OpenNI2FrameListener> ir_frame_listener;
		boost::shared_ptr<OpenNI2FrameListener> color_frame_listener;
		boost::shared_ptr<OpenNI2FrameListener> depth_frame_listener;

		mutable boost::shared_ptr<openni::VideoStream> ir_video_stream_;
		mutable boost::shared_ptr<openni::VideoStream> color_video_stream_;
		mutable boost::shared_ptr<openni::VideoStream> depth_video_stream_;

		mutable std::vector<OpenNI2VideoMode> ir_video_modes_;
		mutable std::vector<OpenNI2VideoMode> color_video_modes_;
		mutable std::vector<OpenNI2VideoMode> depth_video_modes_;

		bool ir_video_started_;
		bool color_video_started_;
		bool depth_video_started_;

		bool image_registration_activated_;

		bool use_device_time_;

		// For depth calculations
		/** \brief distance between the projector and the IR camera*/
		float baseline_;
		/** the value for shadow (occluded pixels) */
		uint64_t shadow_value_;
		/** the value for pixels without a valid disparity measurement */
		uint64_t no_sample_value_;


		// OpenNI 1.x wrapper interface
		typedef boost::function<void(boost::shared_ptr<Image>) > ActualImageCallbackFunction;
		typedef boost::function<void(boost::shared_ptr<DepthImage>) > ActualDepthImageCallbackFunction;
		typedef boost::function<void(boost::shared_ptr<IRImage>) > ActualIRImageCallbackFunction;

		// holds the callback functions together with custom data
		// since same callback function can be registered multiple times with e.g. different custom data
		// we use a map structure with a handle as the key
		std::map<CallbackHandle, ActualImageCallbackFunction> image_callback_;
		std::map<CallbackHandle, ActualDepthImageCallbackFunction> depth_callback_;
		std::map<CallbackHandle, ActualIRImageCallbackFunction> ir_callback_;

		OpenNI2Device::CallbackHandle image_callback_handle_counter_;
		OpenNI2Device::CallbackHandle depth_callback_handle_counter_;
		OpenNI2Device::CallbackHandle ir_callback_handle_counter_;
	};

	// Name compatibility with OpenNI 1.x wrapper
	typedef OpenNI2Device OpenNIDevice;

	PCL_EXPORTS std::ostream& operator<< (std::ostream& stream, const OpenNI2Device& device);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	template<typename T> OpenNI2Device::CallbackHandle
	OpenNI2Device::registerImageCallback (void (T::*callback)(boost::shared_ptr<Image>, void* cookie), T& instance, void* custom_data) throw ()
	{
		image_callback_[image_callback_handle_counter_] = boost::bind (callback, boost::ref (instance), _1, custom_data);
		return (image_callback_handle_counter_++);
	}

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	template<typename T> OpenNI2Device::CallbackHandle
	OpenNI2Device::registerDepthCallback (void (T::*callback)(boost::shared_ptr<DepthImage>, void* cookie), T& instance, void* custom_data) throw ()
	{
		depth_callback_[depth_callback_handle_counter_] = boost::bind ( callback,  boost::ref (instance), _1, custom_data);
		return (depth_callback_handle_counter_++);
	}

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	template<typename T> OpenNI2Device::CallbackHandle
	OpenNI2Device::registerIRCallback (void (T::*callback)(boost::shared_ptr<IRImage>, void* cookie), T& instance, void* custom_data) throw ()
	{
		ir_callback_[ir_callback_handle_counter_] = boost::bind ( callback,  boost::ref (instance), _1, custom_data);
		return (ir_callback_handle_counter_++);
	}

}

#endif /* OPENNI_DEVICE_H */
