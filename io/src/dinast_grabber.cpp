/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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

#include <pcl/io/dinast_grabber.h>
#include <iostream>

// From http://libusb.sourceforge.net/api-1.0/structlibusb__control__setup.html#a39b148c231d675492ccd2383196926bf
// In: device-to-host.
#define CTRL_TO_HOST   (LIBUSB_RECIPIENT_DEVICE | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_IN)
// Out: host-to-device
#define CTRL_TO_DEVICE  (LIBUSB_RECIPIENT_DEVICE | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_OUT)

#define CMD_READ_START  0xC7
#define CMD_READ_STOP   0xC8
#define CMD_GET_VERSION 0xDC
#define CMD_SEND_DATA   0xDE

#define SYNC_PACKET     512
#define RGB16_LENGTH    IMAGE_WIDTH*2*IMAGE_HEIGHT

pcl::DinastGrabber::DinastGrabber ()
  : second_image (false)
  , running_ (false)
{
  bulk_ep = -1;
  ctx = NULL;
}

pcl::DinastGrabber::~DinastGrabber () throw ()
{
  try
  {
    stop ();
    libusb_exit (ctx);
    // unregister callbacks

    // release the pointer to the device object

    
    // disconnect all listeners

  }
  catch (...)
  {
    // destructor never throws
  }
}

std::string
pcl::DinastGrabber::getName () const
{
    return (std::string ("DinastGrabber"));
}

bool
pcl::DinastGrabber::isRunning () const
{
  return (running_);
}

float
pcl::DinastGrabber::getFramesPerSecond () const
{
  //figure this out!!
  return (0);
}

void
pcl::DinastGrabber::findDevice ( int device_position, const int id_vendor, const int id_product)
{
  int device_position_counter=0;
  if (libusb_init (&ctx) != 0)
    return;
  
  libusb_set_debug (ctx, 3);

  libusb_device **devs = NULL;
  // Get the list of USB devices
  ssize_t cnt = libusb_get_device_list (ctx, &devs);

  if (cnt < 0)
    //throw PCLIOException ();
    return;
  
  // Iterate over all devices
  for (ssize_t i = 0; i < cnt; ++i)
  {
    libusb_device_descriptor desc;

    if (libusb_get_device_descriptor (devs[i], &desc) < 0)
    {
      // Free the list
      libusb_free_device_list (devs, 1);
      // Return a NULL device
      return;
    }

    if (desc.idVendor != id_vendor || desc.idProduct != id_product)
      continue;
    libusb_config_descriptor *config;
    // Get the configuration descriptor
    libusb_get_config_descriptor (devs[i], 0, &config);

    // Iterate over all interfaces available
    for (int f = 0; f < int (config->bNumInterfaces); ++f)
    {
      // Iterate over the number of alternate settings
      for (int j = 0; j < config->interface[f].num_altsetting; ++j)
      {
        // Iterate over the number of end points present
        for (int k = 0; k < int (config->interface[f].altsetting[j].bNumEndpoints); ++k) 
        {
          if (config->interface[f].altsetting[j].endpoint[k].bmAttributes == LIBUSB_TRANSFER_TYPE_BULK)
          {
            // Initialize the bulk end point
            bulk_ep = config->interface[f].altsetting[j].endpoint[k].bEndpointAddress;
            break;
          }
        }
      }
    }
    device_position_counter++;
    if (device_position_counter==device_position)
      libusb_open(devs[i], &device_handle);
    // Free the configuration descriptor
    libusb_free_config_descriptor (config);
  }
  
  // Free the list
  libusb_free_device_list (devs, 1);
  
  //Check if device founded if not notify
  if (device_handle==NULL)
    cout<<"Failed to find any DINAST device"<<endl;
  
  // No bulk endpoint was found
  // bulk_ep is unsigned char! Can't compare with -1 !!!
  //if (bulk_ep == -1)
    //throw PCLIOException ();
  //  return;

}

void
pcl::DinastGrabber::openDevice ()
{

  if (device_handle == NULL)
  {
    cout<<"device not found"<<endl; //expection
    return;
  }

  if(libusb_claim_interface(device_handle, 0) < 0)
     cout<<"Failed to Claim interface on device"<<endl;  //Throw PCLIOException
  else
    cout<<"Interface Claimed properly"<<endl;
  
}

void
pcl::DinastGrabber::closeDevice ()
{
  // Release the interface
  libusb_release_interface (device_handle, 0);
  // Close it
  libusb_close (device_handle);
}

bool
pcl::DinastGrabber::USBRxControlData (const unsigned char req_code,
                                      unsigned char *buffer,
                                      int length)
{
  // the direction of the transfer is inferred from the requesttype field of the setup packet.
  unsigned char requesttype = CTRL_TO_HOST;
  // the value field for the setup packet
  unsigned int value = 0x02;
  // the index field for the setup packet
  unsigned char index = 0x08;
  // timeout (in ms) that this function should wait before giving up due to no response being received
  // for an unlimited timeout, use value 0.
  unsigned int timeout = 1000;
  
  int nr_read = libusb_control_transfer (device_handle, requesttype, 
                                         req_code, value, index, buffer, length, timeout);
  if (nr_read != (int)length)
  {
    //throw PCLIOException ();
    return (false);
  }

  return (true);
}

bool
pcl::DinastGrabber::USBTxControlData (const unsigned char req_code,
                                      unsigned char *buffer,
                                      int length)
{
  // the direction of the transfer is inferred from the requesttype field of the setup packet.
  unsigned char requesttype = CTRL_TO_DEVICE;
  // the value field for the setup packet
  unsigned int value = 0x01;
  // the index field for the setup packet
  unsigned char index = 0x00;
  // timeout (in ms) that this function should wait before giving up due to no response being received
  // for an unlimited timeout, use value 0.
  unsigned int timeout = 1000;
  
  int nr_read = libusb_control_transfer (device_handle, requesttype, 
                                         req_code, value, index, buffer, length, timeout);
  if (nr_read != (int)length)
  {
    //throw PCLIOException ();
    return (false);
  }

  return (true);
}

std::string
pcl::DinastGrabber::getDeviceVersion ()
{
  unsigned char data[30];
  if (!USBRxControlData (CMD_GET_VERSION, data, 21))
  {
    //throw PCLIOException ();
    return ("");
  }
 
  data[21] = 0; // NULL

  return (std::string (reinterpret_cast<const char*> (data)));
}

void
pcl::DinastGrabber::start ()
{
  unsigned char ctrl_buf[3];
  if (!USBTxControlData (CMD_READ_START, ctrl_buf, 1))
    return;
	running_ = true;
}

void
pcl::DinastGrabber::stop ()
{
  //unsigned char ctrl_buf[3];
//  if (!USBTxControlData (device_handle, CMD_READ_STOP, ctrl_buf, 1))
//    return;
	running_=false;
}

int
pcl::DinastGrabber::checkHeader ()
{
  // We need at least 2 full sync packets, in case the header starts at the end of the first sync packet to 
  // guarantee that the index returned actually exists in g_buffer (we perform no checking in the rest of the code)
  if (g_buffer.size () < 2 * SYNC_PACKET)
    return (-1);

  int data_ptr = -1;

  for (size_t i = 0; i < g_buffer.size (); ++i)
  {
    if ((g_buffer[i+0] == 0xAA) && (g_buffer[i+1] == 0xAA) && 
        (g_buffer[i+2] == 0x44) && (g_buffer[i+3] == 0x44) &&
        (g_buffer[i+4] == 0xBB) && (g_buffer[i+5] == 0xBB) &&
        (g_buffer[i+6] == 0x77) && (g_buffer[i+7] == 0x77))
    {
      data_ptr = int (i) + SYNC_PACKET;
      break;
    }
  }

  return (data_ptr);
}

int
pcl::DinastGrabber::readImage (unsigned char *image)
{
  // Do we have enough data in the buffer for the second image?
  if (second_image)
  {

    for (int i = 0; i < IMAGE_SIZE; ++i)
      image[i] = g_buffer[i];

    g_buffer.rerase (g_buffer.begin(), g_buffer.begin() + IMAGE_SIZE);

    second_image = false;
    
    return (IMAGE_SIZE);
  }

  // Read data in two image blocks until we get a header
  bool first_image = false;
  int data_adr = -1;
  while (!second_image)
  {
    // Read at least two images in synchronous mode
    int actual_length;
    int res = libusb_bulk_transfer (device_handle, bulk_ep, raw_buffer, 
                                    RGB16_LENGTH + SYNC_PACKET, &actual_length, 1000);
    if (res != 0 || actual_length == 0)
    {
      std::cerr << "USB read error!" << std::endl;
      //throw PCLIOException ();
      memset (&image[0], 0x00, IMAGE_SIZE);
      return (0);
    }

    // Copy data from the USB port if we actually read anything
    //std::cerr << "read " << actual_length << ", buf size: " << g_buffer.size () <<  std::endl;
    // Copy data into the buffer
    int back = int (g_buffer.size ());
    g_buffer.resize (back + actual_length);
    //memcpy (&g_buffer[back], &raw_buffer[0], actual_length);
    for (int i = 0; i < actual_length; ++i)
      g_buffer[back++] = raw_buffer[i];
    //std::cerr << "buf size: " << g_buffer.size () << std::endl;

    // Check if the header is set already
    if (!first_image && data_adr == -1)
    {
      data_adr = checkHeader ();
      //std::cerr << "search for header: " << data_adr << std::endl;
    }

    // Is there enough data in the buffer to return one image, and did we find the header?
    // If not, go back and read some more data off the USB port
    // Else, read the data, clean the g_buffer, return to the user
    if (!first_image && (g_buffer.size () >= IMAGE_SIZE + data_adr) && data_adr != -1)
    {
      // An image found. Copy it from the buffer into the user given buffer

      for (int i = 0; i < IMAGE_SIZE; ++i)
        image[i] = g_buffer[data_adr + i];
      // Pop the data from the global buffer. 
      g_buffer.rerase (g_buffer.begin(), g_buffer.begin() + data_adr + IMAGE_SIZE);
      first_image = true;
    }

    if (first_image && g_buffer.size () >= IMAGE_SIZE)
    {
      break;
     }
  }

  return (IMAGE_SIZE);
}

int
pcl::DinastGrabber::readImage (unsigned char *image1, unsigned char *image2)
{
  // Read data in two image blocks until we get a header
  int data_adr = -1;
  // Read at least two images in synchronous mode
  int actual_length;
  int res = libusb_bulk_transfer (device_handle, bulk_ep, raw_buffer, 
                                  RGB16_LENGTH * 2 + SYNC_PACKET, &actual_length, 0);
  if (res != 0 || actual_length == 0)
  {
    std::cerr << "USB read error!" << std::endl;
    //throw PCLIOException ();
    memset (&image1[0], 0x00, IMAGE_SIZE);
    return (0);
  }

  for (size_t i = 0; i < actual_length; ++i)
  {
    if ((raw_buffer[i+0] == 0xAA) && (raw_buffer[i+1] == 0xAA) && 
        (raw_buffer[i+2] == 0x44) && (raw_buffer[i+3] == 0x44) &&
        (raw_buffer[i+4] == 0xBB) && (raw_buffer[i+5] == 0xBB) &&
        (raw_buffer[i+6] == 0x77) && (raw_buffer[i+7] == 0x77))
    {
      data_adr = int (i) + SYNC_PACKET;
      break;
    }
  }
  // An image found. Copy it from the buffer into the user given buffer
  for (int i = 0; i < IMAGE_SIZE; ++i)
    image1[i] = raw_buffer[data_adr + i];
  for (int i = 0; i < IMAGE_SIZE; ++i)
    image2[i] = raw_buffer[data_adr + IMAGE_SIZE + i];

  return (IMAGE_SIZE);
}

