/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
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

#ifndef __PCL_IO_DINAST_GRABBER__
#define __PCL_IO_DINAST_GRABBER__

#include <pcl/pcl_config.h>
#include <pcl/point_types.h>
#include <pcl/io/grabber.h>
#include <pcl/common/time.h>
#include <libusb-1.0/libusb.h>
#include <boost/circular_buffer.hpp>
#include <string>

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

      /** \brief Check if the grabber is running
        * \return true if grabber is running / streaming. False otherwise.
        */
      bool 
      isRunning () const;

      /** \brief Obtain the number of frames per second (FPS). */
      float 
      getFramesPerSecond () const;


      /** \brief Find the specified Dinast 3D camera device
        * \param[in] device_position Number corresponding to the device to grab
        * \param[in] id_vendor the ID of the camera vendor (should be 0x18d1)
        * \param[in] id_product the ID of the product (should be 0x1402)
        */
      void
      findDevice (int device_position,
                  const int id_vendor = 0x18d1, 
                  const int id_product = 0x1402);

      /** \brief Claims the interface for an already founded device
        */
      void
      openDevice ();

      /** \brief Close the DINAST camera device
        */
      void
      closeDevice ();

      /** \brief Get the version number of the currently opened device
        */
      std::string
      getDeviceVersion ();

      /** \brief Start the data acquisition process.
        */
      void
      start ();

      /** \brief Stop the data acquisition process.
        */
      void
      stop ();

      /** \brief Read image data
        * \param[out] the image data in unsigned short format
        */
      int
      readImage (unsigned char *image);
      
      /** \brief Obtains the image and the corresponding XYZI Point Cloud
	*  \param[out] the image data in unsigned short format and the corresponding point cloud from the camera
	*/
      void
      getData (unsigned char *image, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
      
    protected:      
      /** \brief Send a RX data packet request
        * \param[in] req_code the request to send (the request field for the setup packet)
        * \param[in] length the length field for the setup packet. The data buffer should be at least this size.
        */
      bool
      USBRxControlData (const unsigned char req_code,
                        unsigned char *buffer,
                        int length);

      /** \brief Send a TX data packet request
        * \param[in] req_code the request to send (the request field for the setup packet)
        * \param[in] length the length field for the setup packet. The data buffer should be at least this size.
        */
      bool
      USBTxControlData (const unsigned char req_code,
                        unsigned char *buffer,
                        int length);
      
      /** \brief Check if we have a header in the global buffer, and return the position of the next valid image.
        * \note If the image in the buffer is partial, return -1, as we have to wait until we add more data to it.
        * \return the position of the next valid image (i.e., right after a valid header) or -1 in case the buffer 
        * either doesn't have an image or has a partial image
        */
      int
      checkHeader ();
      
      /** \brief Returns the name of the concrete subclass, DinastGrabber.
        * \return DinastGrabber.
        */
      std::string
      getName() const;
      
      /** \brief Width of image */
      const static int IMAGE_WIDTH = 320;
      
      /** \brief Height of image */
      const static int IMAGE_HEIGHT  = 240;
      
      /** \brief Total size of image */
      const static int IMAGE_SIZE = IMAGE_HEIGHT * IMAGE_WIDTH;
      
      /** \brief Length of a sync packet */
      const static int SYNC_PACKET = 512;
      
      /** \brief Size of pixel */
      enum pixel_syze { RAW8=1, RGB16=2, RGB24=3, RGB32=4 };
      
      /** \brief The libusb context*/
      libusb_context *context;
      
      /** \brief the actual device_handle for the camera */
      struct libusb_device_handle* device_handle;
      
      /** \brief Temporary USB read buffer, since we read two RGB16 images at a time size is the double of two images plus a sync packet */
      unsigned char raw_buffer[(RGB16 * (IMAGE_SIZE) + SYNC_PACKET)*2];

      /** \brief Global buffer */
      boost::circular_buffer<unsigned char> g_buffer;

      /**  \brief Bulk endpoint address value */
      unsigned char bulk_ep;

      // Since there is no header after the first image, we need to save the state
      bool second_image;
      bool running;

      
  };
} //namespace pcl

#endif // __PCL_IO_DINAST_GRABBER__
