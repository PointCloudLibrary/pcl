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

#include <pcl/pcl_config.h>
#include <pcl/io/dinast_grabber.h>

using namespace std;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
pcl::DinastGrabber::DinastGrabber (const int device_position)
  : image_width_ (320)
  , image_height_ (240)
  , sync_packet_size_ (512)
  , fov_ (64. * M_PI / 180.)
  , context_ (NULL)
  , bulk_ep_ (std::numeric_limits<unsigned char>::max ())
  , second_image_ (false)
  , running_ (false)
{
  image_size_ = image_width_ * image_height_;
  dist_max_2d_ = 1. / (image_width_ / 2.);
  onInit(device_position);
  
  point_cloud_signal_ = createSignal<sig_cb_dinast_point_cloud> ();
  raw_buffer_ = new unsigned char[(RGB16 * (image_size_) + sync_packet_size_) * 2];
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
pcl::DinastGrabber::~DinastGrabber () throw ()
{
  try
  {
    stop ();

    libusb_exit (context_);
    
    // Release the interface
    libusb_release_interface (device_handle_, 0);
    // Close it
    libusb_close (device_handle_); 
    delete[] raw_buffer_;
    delete[] image_;
  }
  catch (...)
  {
    // Destructor never throws
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::DinastGrabber::isRunning () const
{
  return (running_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float
pcl::DinastGrabber::getFramesPerSecond () const
{ 
  static double last = pcl::getTime ();
  double now = pcl::getTime (); 
  float rate = 1 / float(now - last);
  last = now; 

  return (rate);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::DinastGrabber::onInit (const int device_position)
{
  setupDevice (device_position);
  capture_thread_ = boost::thread (&DinastGrabber::captureThreadFunction, this);
  image_ = new unsigned char [image_size_];
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::DinastGrabber::setupDevice (int device_position, const int id_vendor, const int id_product)
{
  int device_position_counter = 0;
  
  // Initialize libusb
  int ret=libusb_init (&context_);
  std::stringstream sstream;
  if (ret != 0)
  {
    sstream << "[pcl::DinastGrabber::setupDevice] libusb initialization failure, LIBUSB_ERROR: "<< ret;
    PCL_THROW_EXCEPTION (pcl::IOException, sstream.str ());
  }
  
  libusb_set_debug (context_, 3);
  libusb_device **devs = NULL;
  
  // Get the list of USB devices
  ssize_t number_devices = libusb_get_device_list (context_, &devs);

  if (number_devices < 0)
    PCL_THROW_EXCEPTION (pcl::IOException, "[pcl::DinastGrabber::setupDevice] No USB devices found!");
  
  // Iterate over all devices
  for (ssize_t i = 0; i < number_devices; ++i)
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
            bulk_ep_ = config->interface[f].altsetting[j].endpoint[k].bEndpointAddress;
            break;
          }
        }
      }
    }
    device_position_counter++;
    if (device_position_counter == device_position)
      libusb_open(devs[i], &device_handle_);
    // Free the configuration descriptor
    libusb_free_config_descriptor (config);
  }
  
  // Free the list
  libusb_free_device_list (devs, 1);
  
  // Check if device founded if not notify
  if (device_handle_ == NULL)
    PCL_THROW_EXCEPTION (pcl::IOException, "[pcl::DinastGrabber::setupDevice] Failed to find any DINAST devices attached");
  
  // Claim device interface
  if (libusb_claim_interface(device_handle_, 0) < 0)
     PCL_THROW_EXCEPTION (pcl::IOException, "[pcl::DinastGrabber::setupDevice] Failed to open DINAST device");

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::string
pcl::DinastGrabber::getDeviceVersion ()
{
  unsigned char data[21];
  if (!USBRxControlData (CMD_GET_VERSION, data, 21))
     PCL_THROW_EXCEPTION (pcl::IOException, "[pcl::DinastGrabber::getDeviceVersion] Error trying to get device version");
 
  //data[21] = 0;
  return (std::string (reinterpret_cast<const char*> (data)));
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::DinastGrabber::start ()
{
  unsigned char ctrl_buf[1];
  if (!USBTxControlData (CMD_READ_START, ctrl_buf, 1))
    PCL_THROW_EXCEPTION (pcl::IOException, "[pcl::DinastGrabber::start] Could not start the USB data reading");
  running_ = true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::DinastGrabber::stop ()
{
  unsigned char ctrl_buf[1];
  if (!USBTxControlData (CMD_READ_STOP, ctrl_buf, 1))
    PCL_THROW_EXCEPTION (pcl::IOException, "[pcl::DinastGrabber::stop] Could not stop the USB data reading");
  running_ = false;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::DinastGrabber::readImage ()
{
  // Do we have enough data in the buffer for the second image?
  if (second_image_)
  {
    for (int i = 0; i < image_size_; ++i)
      image_[i] = g_buffer_[i];

    g_buffer_.rerase (g_buffer_.begin (), g_buffer_.begin () + image_size_);
    second_image_ = false;
    
    return;
  }

  // Read data in two image blocks until we get a header
  bool first_image = false;
  int data_adr = -1;
  while (!second_image_)
  {
    // Read at least two images in synchronous mode
    int actual_length;
    int res = libusb_bulk_transfer (device_handle_, bulk_ep_, raw_buffer_,
                                    RGB16 * (image_size_) + sync_packet_size_, &actual_length, 1000);
    if (res != 0 || actual_length == 0)
    {
      memset (&image_[0], 0x00, image_size_);
      PCL_THROW_EXCEPTION (pcl::IOException, "[pcl::DinastGrabber::readImage] USB read error!");
    }

    // Copy data from the USB port if we actually read anything
    PCL_DEBUG ("[pcl::DinastGrabber::readImage] Read: %d, size of the buffer: %d\n" ,actual_length, g_buffer_.size ());
    
    // Copy data into the buffer
    int back = int (g_buffer_.size ());
    g_buffer_.resize (back + actual_length);

    for (int i = 0; i < actual_length; ++i)
      g_buffer_[back++] = raw_buffer_[i];

    PCL_DEBUG ("[pcl::DinastGrabber::readImage] Buffer size: %d\n", g_buffer_.size());

    // Check if the header is set already
    if (!first_image && data_adr == -1)
    {
      data_adr = checkHeader ();
      PCL_DEBUG ("[pcl::DinastGrabber::readImage] Searching for header at: %d", data_adr);
    }

    // Is there enough data in the buffer to return one image, and did we find the header?
    // If not, go back and read some more data off the USB port
    // Else, read the data, clean the g_buffer_, return to the user
    if (!first_image && (g_buffer_.size () >= static_cast<boost::circular_buffer<unsigned char>::size_type> (image_size_ + data_adr)) && data_adr != -1)
    {
      // An image found. Copy it from the buffer into the user given buffer

      for (int i = 0; i < image_size_; ++i)
        image_[i] = g_buffer_[data_adr + i];

      // Pop the data from the global buffer. 
      g_buffer_.rerase (g_buffer_.begin(), g_buffer_.begin() + data_adr + image_size_);
      first_image = true;
    }

    if (first_image && g_buffer_.size () >= static_cast<boost::circular_buffer<unsigned char>::size_type> (image_size_))
      break;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
pcl::PointCloud<pcl::PointXYZI>::Ptr
pcl::DinastGrabber::getXYZIPointCloud ()
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
  
  cloud->points.resize (image_size_);
  cloud->width = image_width_;
  cloud->height = image_height_;
  cloud->is_dense = false;
  
  int depth_idx = 0;

  for (int x = 0; x < static_cast<int> (cloud->width); ++x)
  {
    for (int y = 0; y < static_cast<int> (cloud->height); ++y, ++depth_idx)
    {
      double pixel = image_[x + image_width_ * y];

      // Correcting distortion, data emprically got in a calibration test
      double xc = static_cast<double> (x - image_width_ / 2);
      double yc = static_cast<double> (y - image_height_ / 2);
      double r1 = sqrt (xc * xc + yc * yc);
      double r2 = r1 * r1;
      double r3 = r1 * r2;
      double A = -2e-5 * r3 + 0.004 * r2 + 0.1719 * r1 + 350.03;
      double B = -2e-9 * r3 + 3e-7 * r2 - 1e-5 * r1 - 0.01;

      // Compute distance
      if (pixel > A)
        pixel = A;
      double dy = y*0.1;
      double dist = (log (static_cast<double> (pixel / A)) / B - dy) * (7E-07*r3 - 0.0001*r2 + 0.004*r1 + 0.9985) * 1.5;
      double theta_colati = fov_ * r1 * dist_max_2d_;
      double c_theta = cos (theta_colati);
      double s_theta = sin (theta_colati);
      double c_ksai = static_cast<double> (x - 160) / r1;
      double s_ksai = static_cast<double> (y - 120) / r1;
      cloud->points[depth_idx].x = static_cast<float> ((dist * s_theta * c_ksai) / 500.0 + 0.5);
      cloud->points[depth_idx].y = static_cast<float> ((dist * s_theta * s_ksai) / 500.0 + 0.5);
      cloud->points[depth_idx].z = static_cast<float> (dist * c_theta);
      if (cloud->points[depth_idx].z < 0.01f)
        cloud->points[depth_idx].z = 0.01f;
      cloud->points[depth_idx].z /= 500.0f;
      cloud->points[depth_idx].intensity = static_cast<float> (pixel);

      
      // Get rid of the noise
      if(cloud->points[depth_idx].z > 0.8f or cloud->points[depth_idx].z < 0.02f)
      {
        cloud->points[depth_idx].x = std::numeric_limits<float>::quiet_NaN ();
      	cloud->points[depth_idx].y = std::numeric_limits<float>::quiet_NaN ();
      	cloud->points[depth_idx].z = std::numeric_limits<float>::quiet_NaN ();
        cloud->points[depth_idx].intensity = static_cast<float> (pixel);
      }
    }
  }
  return cloud;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::DinastGrabber::USBRxControlData (const unsigned char req_code,
                                      unsigned char *buffer,
                                      int length)
{
  // The direction of the transfer is inferred from the requesttype field of the setup packet.
  unsigned char requesttype = (LIBUSB_RECIPIENT_DEVICE | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_IN);
  // The value field for the setup packet
  uint16_t value = 0x02;
  // The index field for the setup packet
  uint16_t index = 0x08;
  // timeout (in ms) that this function should wait before giving up due to no response being received
  // For an unlimited timeout, use value 0.
  uint16_t timeout = 1000;
  
  int nr_read = libusb_control_transfer (device_handle_, requesttype,
                                         req_code, value, index, buffer, static_cast<uint16_t> (length), timeout);
  if (nr_read != int(length))
    PCL_THROW_EXCEPTION (pcl::IOException, "[pcl::DinastGrabber::USBRxControlData] Control data error");

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::DinastGrabber::USBTxControlData (const unsigned char req_code,
                                      unsigned char *buffer,
                                      int length)
{
  // The direction of the transfer is inferred from the requesttype field of the setup packet.
  unsigned char requesttype = (LIBUSB_RECIPIENT_DEVICE | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_OUT);
  // The value field for the setup packet
  uint16_t value = 0x01;
  // The index field for the setup packet
  uint16_t index = 0x00;
  // timeout (in ms) that this function should wait before giving up due to no response being received
  // For an unlimited timeout, use value 0.
  uint16_t timeout = 1000;
  
  int nr_read = libusb_control_transfer (device_handle_, requesttype,
                                         req_code, value, index, buffer, static_cast<uint16_t> (length), timeout);
  if (nr_read != int(length))
  {
    std::stringstream sstream;
    sstream << "[pcl::DinastGrabber::USBTxControlData] USB control data error, LIBUSB_ERROR: " << nr_read;
    PCL_THROW_EXCEPTION (pcl::IOException, sstream.str ());
  }

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int
pcl::DinastGrabber::checkHeader ()
{
  // We need at least 2 full sync packets, in case the header starts at the end of the first sync packet to 
  // guarantee that the index returned actually exists in g_buffer_ (we perform no checking in the rest of the code)
  if (g_buffer_.size () < static_cast<boost::circular_buffer<unsigned char>::size_type> (2 * sync_packet_size_))
    return (-1);

  int data_ptr = -1;

  for (size_t i = 0; i < g_buffer_.size (); ++i)
  {
    if ((g_buffer_[i + 0] == 0xAA) && (g_buffer_[i + 1] == 0xAA) &&
        (g_buffer_[i + 2] == 0x44) && (g_buffer_[i + 3] == 0x44) &&
        (g_buffer_[i + 4] == 0xBB) && (g_buffer_[i + 5] == 0xBB) &&
        (g_buffer_[i + 6] == 0x77) && (g_buffer_[i + 7] == 0x77))
    {
      data_ptr = int (i) + sync_packet_size_;
      break;
    }
  }

  return (data_ptr);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::DinastGrabber::captureThreadFunction ()
{
  while (true)
  {
    // Lock before checking running flag
    boost::unique_lock<boost::mutex> capture_lock (capture_mutex_);
    if(running_)
    {
      readImage ();
    
      // Check for point clouds slots
      if (num_slots<sig_cb_dinast_point_cloud> () > 0 )
        point_cloud_signal_->operator() (getXYZIPointCloud ());
    } 
    capture_lock.unlock ();
  }
}
