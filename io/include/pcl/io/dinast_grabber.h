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
 * $Id$
 *
 */

#ifndef PCL_IO_DINAST_GRABBER_
#define PCL_IO_DINAST_GRABBER_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/grabber.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <libusb-1.0/libusb.h>
#include <boost/circular_buffer.hpp>

namespace pcl
{
  /** \brief Grabber for DINAST devices (i.e., IPA-1002, IPA-1110, IPA-2001)
    * \author Marco A. Gutierrez <marcog@unex.es>
    * \ingroup io
    */
  class PCL_EXPORTS DinastGrabber: public Grabber
  {
    // Define callback signature typedefs
    typedef void (sig_cb_dinast_point_cloud) (const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZI> >&);
    
    public:
      /** \brief Constructor that sets up the grabber constants.
        * \param[in] device_position Number corresponding the device to grab
        */
      DinastGrabber (const int device_position=1);

      /** \brief Destructor. It never throws. */
      virtual ~DinastGrabber () throw ();

      /** \brief Check if the grabber is running
        * \return true if grabber is running / streaming. False otherwise.
        */
      virtual bool 
      isRunning () const;
      
      /** \brief Returns the name of the concrete subclass, DinastGrabber.
        * \return DinastGrabber.
        */
      virtual std::string
      getName () const
      { return (std::string ("DinastGrabber")); }
      
      /** \brief Start the data acquisition process.
        */
      virtual void
      start ();

      /** \brief Stop the data acquisition process.
        */
      virtual void
      stop ();
      
      /** \brief Obtain the number of frames per second (FPS). */
      virtual float 
      getFramesPerSecond () const;

      /** \brief Get the version number of the currently opened device
        */
      std::string
      getDeviceVersion ();
      
    protected:  
      
      /** \brief On initialization processing. */
      void
      onInit (const int device_id);
      
      /** \brief Setup a Dinast 3D camera device
        * \param[in] device_position Number corresponding the device to grab
        * \param[in] id_vendor The ID of the camera vendor (should be 0x18d1)
        * \param[in] id_product The ID of the product (should be 0x1402)
        */
      void
      setupDevice (int device_position,
                  const int id_vendor = 0x18d1, 
                  const int id_product = 0x1402);
      
      /** \brief Send a RX data packet request
        * \param[in] req_code the request to send (the request field for the setup packet)
        * \param buffer
        * \param[in] length the length field for the setup packet. The data buffer should be at least this size.
        */
      bool
      USBRxControlData (const unsigned char req_code,
                        unsigned char *buffer,
                        int length);

      /** \brief Send a TX data packet request
        * \param[in] req_code the request to send (the request field for the setup packet)
        * \param buffer
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
      
      /** \brief Read image data and leaves it on image_
        */
      void
      readImage ();
      
      /** \brief Obtains XYZI Point Cloud from the image of the camera
        * \return the point cloud from the image data
        */
      pcl::PointCloud<pcl::PointXYZI>::Ptr
      getXYZIPointCloud ();
      
       /** \brief The function in charge of getting the data from the camera
        */     
      void 
      captureThreadFunction ();
      
      /** \brief Width of image */
      int image_width_;
      
      /** \brief Height of image */
      int image_height_;
      
      /** \brief Total size of image */
      int image_size_;
      
      /** \brief Length of a sync packet */
      int sync_packet_size_;
      
      double dist_max_2d_;
      
      /** \brief diagonal Field of View*/
      double fov_;
      
      /** \brief Size of pixel */
      enum pixel_size { RAW8=1, RGB16=2, RGB24=3, RGB32=4 };
      
      /** \brief The libusb context*/
      libusb_context *context_;
      
      /** \brief the actual device_handle for the camera */
      struct libusb_device_handle *device_handle_;
      
      /** \brief Temporary USB read buffer, since we read two RGB16 images at a time size is the double of two images
        * plus a sync packet.
        */
      unsigned char *raw_buffer_ ;

      /** \brief Global circular buffer */
      boost::circular_buffer<unsigned char> g_buffer_;

      /** \brief Bulk endpoint address value */
      unsigned char bulk_ep_;
      
      /** \brief Device command values */
      enum { CMD_READ_START=0xC7, CMD_READ_STOP=0xC8, CMD_GET_VERSION=0xDC, CMD_SEND_DATA=0xDE };

      unsigned char *image_;
      
      /** \brief Since there is no header after the first image, we need to save the state */
      bool second_image_;
      
      bool running_;
      
      boost::thread capture_thread_;
      
      mutable boost::mutex capture_mutex_;
      boost::signals2::signal<sig_cb_dinast_point_cloud>* point_cloud_signal_;
  };
} //namespace pcl

#endif // PCL_IO_DINAST_GRABBER_
