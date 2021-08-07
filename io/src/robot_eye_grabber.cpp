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

#include <pcl/io/robot_eye_grabber.h>
#include <pcl/common/point_tests.h> // for pcl::isFinite
#include <pcl/console/print.h>

/////////////////////////////////////////////////////////////////////////////
pcl::RobotEyeGrabber::RobotEyeGrabber ()
  : terminate_thread_ (false)
  , signal_point_cloud_size_ (1000)
  , data_port_ (443)
  , sensor_address_ (boost::asio::ip::address_v4::any ())
{
  point_cloud_signal_ = createSignal<sig_cb_robot_eye_point_cloud_xyzi> ();
  resetPointCloud ();
}

/////////////////////////////////////////////////////////////////////////////
pcl::RobotEyeGrabber::RobotEyeGrabber (const boost::asio::ip::address& ipAddress, unsigned short port)
  : terminate_thread_ (false)
  , signal_point_cloud_size_ (1000)
  , data_port_ (port)
  , sensor_address_ (ipAddress)
{
  point_cloud_signal_ = createSignal<sig_cb_robot_eye_point_cloud_xyzi> ();
  resetPointCloud ();
}

/////////////////////////////////////////////////////////////////////////////
pcl::RobotEyeGrabber::~RobotEyeGrabber () noexcept
{
  stop ();
  disconnect_all_slots<sig_cb_robot_eye_point_cloud_xyzi> ();
}

/////////////////////////////////////////////////////////////////////////////
std::string
pcl::RobotEyeGrabber::getName () const
{
  return (std::string ("Ocular Robotics RobotEye Grabber"));
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float
pcl::RobotEyeGrabber::getFramesPerSecond () const
{
  return (0.0f);
}

/////////////////////////////////////////////////////////////////////////////
bool
pcl::RobotEyeGrabber::isRunning () const
{
  return (socket_thread_ != nullptr);
}

/////////////////////////////////////////////////////////////////////////////
unsigned short
pcl::RobotEyeGrabber::getDataPort () const
{
  return (data_port_);
}

/////////////////////////////////////////////////////////////////////////////
void
pcl::RobotEyeGrabber::setDataPort (const unsigned short port)
{
  data_port_ = port;
}

/////////////////////////////////////////////////////////////////////////////
const boost::asio::ip::address&
pcl::RobotEyeGrabber::getSensorAddress () const
{
  return (sensor_address_);
}

/////////////////////////////////////////////////////////////////////////////
void
pcl::RobotEyeGrabber::setSensorAddress (const boost::asio::ip::address& ipAddress)
{
  sensor_address_ = ipAddress;
}

/////////////////////////////////////////////////////////////////////////////
std::size_t
pcl::RobotEyeGrabber::getSignalPointCloudSize () const
{
  return (signal_point_cloud_size_);
}

/////////////////////////////////////////////////////////////////////////////
void
pcl::RobotEyeGrabber::setSignalPointCloudSize (std::size_t numberOfPoints)
{
  signal_point_cloud_size_ = numberOfPoints;
}

/////////////////////////////////////////////////////////////////////////////
pcl::PointCloud<pcl::PointXYZI>::Ptr
pcl::RobotEyeGrabber::getPointCloud () const
{
  return point_cloud_xyzi_;
}

/////////////////////////////////////////////////////////////////////////////
void
pcl::RobotEyeGrabber::resetPointCloud ()
{
  point_cloud_xyzi_.reset (new pcl::PointCloud<pcl::PointXYZI>);
  point_cloud_xyzi_->is_dense = true;
}

/////////////////////////////////////////////////////////////////////////////
void
pcl::RobotEyeGrabber::consumerThreadLoop ()
{
  while (true)
  {
    boost::shared_array<unsigned char> data;
    if (!packet_queue_.dequeue (data))
      return;
    convertPacketData (data.get(), data_size_);
  }
}

/////////////////////////////////////////////////////////////////////////////
void
pcl::RobotEyeGrabber::convertPacketData (unsigned char *data_packet, std::size_t length)
{
  //Check for the presence of the header
  std::size_t offset = 0;
  //The old packet data format does not start with an E since it just starts with laser data
  if(data_packet[0] != 'E')
  {
    //old packet data format (just laser data)
    offset = 0;
  }
  else
  {
    //The new packet data format contains this as a header
    //char[6]  "EBRBEP"
    //std::uint32_t Timestamp // counts of a 66 MHz clock since power-on of eye.
    std::size_t response_size = 6; //"EBRBEP"
    if( !strncmp((char*)(data_packet), "EBRBEP", response_size) )
    {
      std::uint32_t timestamp; // counts of a 66 MHz clock since power-on of eye.
      computeTimestamp(timestamp, data_packet + response_size);
      //std::cout << "Timestamp: " << timestamp << std::endl;
      offset = (response_size + sizeof(timestamp));
    }
    else
    {
      //Invalid packet received, ignore it.
      return;
    }
  }

  const std::size_t bytes_per_point = 8;
  const std::size_t total_points = (length - offset)/ bytes_per_point;

  for (std::size_t i = 0; i < total_points; ++i)
  {
    PointXYZI xyzi;
    computeXYZI (xyzi, data_packet + i*bytes_per_point + offset);

    if (pcl::isFinite(xyzi))
    {
      point_cloud_xyzi_->push_back (xyzi);
    }
  }

  if (point_cloud_xyzi_->size () > signal_point_cloud_size_)
  {
    if (point_cloud_signal_->num_slots () > 0)
      point_cloud_signal_->operator() (point_cloud_xyzi_);

    resetPointCloud ();
  }
}

/////////////////////////////////////////////////////////////////////////////
void
pcl::RobotEyeGrabber::computeXYZI (pcl::PointXYZI& point, unsigned char* point_data)
{
  std::uint16_t buffer = 0;
  double az = 0.0;
  double el = 0.0;
  double range = 0.0;
  std::uint16_t intensity = 0;

  buffer = point_data[0] << 8;
  buffer |= point_data[1]; // First 2-byte read will be Azimuth
  az = (buffer / 100.0);

  buffer =  point_data[2] << 8;
  buffer |= point_data[3]; // Second 2-byte read will be Elevation
  el = (signed short int)buffer / 100.0;

  buffer =  point_data[4] << 8;
  buffer |= point_data[5]; // Third 2-byte read will be Range
  range = (signed short int)buffer / 100.0;

  buffer =  point_data[6] << 8;
  buffer |= point_data[7]; // Fourth 2-byte read will be Intensity
  intensity = buffer;

  point.x = range * std::cos (el * M_PI/180) * std::sin (az * M_PI/180);
  point.y = range * std::cos (el * M_PI/180) * std::cos (az * M_PI/180);
  point.z = range * std::sin (el * M_PI/180);
  point.intensity = intensity;
}

/////////////////////////////////////////////////////////////////////////////
void
pcl::RobotEyeGrabber::computeTimestamp (std::uint32_t& timestamp, unsigned char* point_data)
{
  std::uint32_t buffer;
  buffer = point_data[0] << 24;
  buffer |= point_data[1] << 16;
  buffer |= point_data[2] << 8;
  buffer |= point_data[3];
  timestamp = buffer;
}



/////////////////////////////////////////////////////////////////////////////
void
pcl::RobotEyeGrabber::socketThreadLoop ()
{
  asyncSocketReceive();
  io_service_.reset();
  io_service_.run();
}

/////////////////////////////////////////////////////////////////////////////
void
pcl::RobotEyeGrabber::asyncSocketReceive ()
{
  // expecting at most max_length bytes (UDP packet).
  socket_->async_receive_from(boost::asio::buffer(receive_buffer_, MAX_LENGTH), sender_endpoint_,
    [this] (const boost::system::error_code& error, std::size_t number_of_bytes) {
      socketCallback (error, number_of_bytes);
    });
}

/////////////////////////////////////////////////////////////////////////////
void
pcl::RobotEyeGrabber::socketCallback (const boost::system::error_code&, std::size_t number_of_bytes)
{
  if (terminate_thread_)
    return;

  if (sensor_address_ == boost::asio::ip::address_v4::any ()
    || sensor_address_ == sender_endpoint_.address ())
  {
    data_size_ = number_of_bytes;
    unsigned char *dup = new unsigned char[number_of_bytes];
    memcpy (dup, receive_buffer_, number_of_bytes);
    packet_queue_.enqueue (boost::shared_array<unsigned char>(dup));
  }

  asyncSocketReceive ();
}

/////////////////////////////////////////////////////////////////////////////
void
pcl::RobotEyeGrabber::start ()
{
  resetPointCloud ();

  if (isRunning ())
    return;

  boost::asio::ip::udp::endpoint destinationEndpoint (boost::asio::ip::address_v4::any (), data_port_);

  try
  {
    socket_.reset (new boost::asio::ip::udp::socket (io_service_, destinationEndpoint));
  }
  catch (std::exception &e)
  {
    PCL_ERROR ("[pcl::RobotEyeGrabber::start] Unable to bind to socket! %s\n", e.what ());
    return;
  }

  terminate_thread_ = false;
  resetPointCloud ();
  consumer_thread_.reset(new std::thread (&RobotEyeGrabber::consumerThreadLoop, this));
  socket_thread_.reset(new std::thread (&RobotEyeGrabber::socketThreadLoop, this));
}

/////////////////////////////////////////////////////////////////////////////
void
pcl::RobotEyeGrabber::stop ()
{
  if (!isRunning ())
    return;

  terminate_thread_ = true;

  socket_->close ();
  io_service_.stop ();
  socket_thread_->join ();
  socket_thread_.reset ();
  socket_.reset();

  packet_queue_.stopQueue ();
  consumer_thread_->join ();
  consumer_thread_.reset ();
}
