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

#ifndef __PCL_IO_DINAST_GRABBER__
#define __PCL_IO_DINAST_GRABBER__

#include <libusb-1.0/libusb.h>
#include <pcl/io/grabber.h>
#include <boost/circular_buffer.hpp>
#include <string>

#define IMAGE_WIDTH			320
#define IMAGE_HEIGHT		240
#define IMAGE_SIZE      IMAGE_WIDTH * IMAGE_HEIGHT

#define SYNC_PACKET			512

#define RAW8_LENGTH			IMAGE_WIDTH*IMAGE_HEIGHT
#define RGB16_LENGTH		IMAGE_WIDTH*2*IMAGE_HEIGHT
#define RGB24_LENGTH		IMAGE_WIDTH*3*IMAGE_HEIGHT
#define RGB32_LENGTH		IMAGE_WIDTH*4*IMAGE_HEIGHT

#define IMAGE_PAGE_MAX		(1)

#define	MSG_USB_EVENT		WM_USER+3

namespace pcl
{
	/** \brief Grabber for DINAST devices (i.e., IPA-1002, IPA-1110, IPA-2001)
	* \author Marco A. Gutierrez <marcog@unex.es>
	* \ingroup io
	*/
	class PCL_EXPORTS DinastGrabber: public Grabber
	{
		
		public:
			
			/** \brief Constructor. */
			DinastGrabber ();
			
			/** \brief Destructor */
			~DinastGrabber () throw ();
			
			/** \brief returns the name of the concrete subclass, DinastGrabber.
      * \return DinastGrabber.
      */
			std::string
			getName() const;
			
			/** \brief Check if the grabber is running
      * \return true if grabber is running / streaming. False otherwise.
      */
      bool 
      isRunning () const;
			
			/** \brief Obtain the number of frames per second (FPS). */
      float 
      getFramesPerSecond () const;
			
			/** \brief Sets the device. */
			void
			setDevice (struct libusb_device_handle* device_handle);
					//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			/** \brief Find a Dinast 3D camera device
				* \param[out] ctx the libusb context
				* \param[in] id_vendor the ID of the camera vendor (should be 0x18d1)
				* \param[in] id_product the ID of the product (should be 0x1402)
				* \return NULL if no USB device found with the given vendor/product ID, or the usb_device object otherwise
				*/
			struct libusb_device_handle*
			findDevice (libusb_context *ctx = NULL, 
									const int id_vendor = 0x18d1, 
									const int id_product = 0x1402);

			//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			/** \brief Open a given USB device
				* \param[in] ctx the libusb context
				* \param[in] idx the USB device index to open
				* \return NULL if the input device cannot be open, the device handle otherwise
				*/
			struct libusb_device_handle*
			openDevice (struct libusb_context *ctx, int idx);

			//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			/** \brief Close the given USB device
				* \param[in] device_handle the USB device handle to close
				* \param[in] index the index of the device to close
				*/
			void
			closeDevice (struct libusb_device_handle* device_handle, int index = 0);

			//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			/** \brief Send a RX data packet request
				* \param[in] device_handle the USB device handle to close
				* \param[in] req_code the request to send (the request field for the setup packet)
				* \param[in] length the length field for the setup packet. The data buffer should be at least this size.
				*/
			bool
			USBRxControlData (struct libusb_device_handle* device_handle, 
												const unsigned char req_code,
												unsigned char *buffer,
												int length);

			//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			/** \brief Send a TX data packet request
				* \param[in] device_handle the USB device handle to close
				* \param[in] req_code the request to send (the request field for the setup packet)
				* \param[in] length the length field for the setup packet. The data buffer should be at least this size.
				*/
			bool
			USBTxControlData (struct libusb_device_handle* device_handle, 
												const unsigned char req_code,
												unsigned char *buffer,
												int length);

			//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			/** \brief Get the version number of the currently opened device
				* \param[in] device_handle the device to get the version number from
				*/
			std::string
			getDeviceVersion (struct libusb_device_handle* device_handle);

			//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			/** \brief Start the data acquisition process.
				* \param[in] device_handle the USB device to start acquisiton from
				*/
			void
			start ();

			//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			/** \brief Stop the data acquisition process.
				* \param[in] device_handle the USB device to stop acquisiton from
				*/
			void
			stop ();

			//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			/** \brief Check if we have a header in the global buffer, and return the position of the next valid image.
				* \note If the image in the buffer is partial, return -1, as we have to wait until we add more data to it.
				* \return the position of the next valid image (i.e., right after a valid header) or -1 in case the buffer 
				* either doesn't have an image or has a partial image
				*/
			int
			checkHeader ();

			//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			/** \brief Read image data
				* \param[in] device_handle the USB device handle to close
				* \param[out] the image data in unsigned short format
				*/
			int
			readImage (struct libusb_device_handle* device_handle,
								unsigned char *image);

			//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			/** \brief Read image data
				* \param[in] device_handle the USB device handle to close
				* \param[out] the image data in unsigned short format
				*/
			int
			readImage (struct libusb_device_handle* device_handle,
								unsigned char *image1, unsigned char *image2);
			
			// GLOBAL VARIABLES
			/// @todo Should eventually allow opening more than 1 device at once

			// Temporary USB read buffer
			unsigned char raw_buffer[(RGB16_LENGTH + SYNC_PACKET)*2];

			// Global buffer
			boost::circular_buffer<unsigned char> g_buffer;

			// Bulk endpoint address value
			unsigned char bulk_ep;

			// Since there is no header after the first image, we need to save the state
			bool second_image;
			bool running_;
			
			struct libusb_device_handle* device_handle;
	};
} //namespace pcl

#endif //__PCL_IO_DINAST_GRABBER__
