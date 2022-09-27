/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2020-, Open Perception
 *  Copyright (c) 2020, ysuzuki19
 *
 *  All rights reserved
 */

#include <pcl/io/tim_grabber.h>

///////////////////////////////////////////////////////////////////////////////////////////////////
pcl::TimGrabber::TimGrabber () :
    tim_socket_ (tim_io_service_)
{
  initialize ();
}

///////////////////////////////////////////////////////////////////////////////////////////////////
pcl::TimGrabber::TimGrabber (const boost::asio::ip::address& ipAddress,
                             const std::uint16_t port) :
    tcp_endpoint_ (ipAddress, port),
    tim_socket_ (tim_io_service_)
{
  initialize ();
}

///////////////////////////////////////////////////////////////////////////////////////////////////
pcl::TimGrabber::~TimGrabber () noexcept
{
  stop ();

  disconnect_all_slots<sig_cb_sick_tim_scan_point_cloud_xyz> ();
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::TimGrabber::initialize ()
{
  buildLookupTables ();

  point_cloud_xyz_signal_ = createSignal<sig_cb_sick_tim_scan_point_cloud_xyz> ();

  point_cloud_xyz_ptr_.reset (new pcl::PointCloud<pcl::PointXYZ>);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
float
pcl::TimGrabber::getFramesPerSecond () const
{
  std::lock_guard<std::mutex> lock (frequency_mutex_);
  return (frequency_.getFrequency ());
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::TimGrabber::buildLookupTables () {
  const float angle_step = angle_range_ / static_cast<float> (amount_of_data_);
  float angle = angle_start_;

  cos_dynamic_lookup_table_.reserve (amount_of_data_);
  cos_dynamic_lookup_table_.clear ();
  sin_dynamic_lookup_table_.reserve (amount_of_data_);
  sin_dynamic_lookup_table_.clear ();

  for (std::size_t i = 0; i < amount_of_data_; i++, angle += angle_step)
  {
    cos_dynamic_lookup_table_.push_back (std::cos (angle));
    sin_dynamic_lookup_table_.push_back (std::sin (angle));
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::TimGrabber::updateLookupTables () {
  //// note: cos, sin are the same size
  if (cos_dynamic_lookup_table_.size () != amount_of_data_)
    buildLookupTables ();
}


///////////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::TimGrabber::isValidPacket () const {
  return received_packet_.data ()[length_-1] == '\03';
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::TimGrabber::receiveTimPacket ()
{
  if (!tim_socket_.is_open ())
    return;

  tim_socket_.send (boost::asio::buffer ("\x02sRN LMDscandata 1\x03\0"));

  std::this_thread::sleep_for (std::chrono::milliseconds (wait_time_milliseconds_));

  length_ = tim_socket_.receive (boost::asio::buffer (received_packet_));

  if (!isValidPacket ()) {
    //// Countup wait_time
    //// Because fit wait_time to network latency
    wait_time_milliseconds_++;

    //// Clear invalid packet in socket
    while (!isValidPacket ())
      length_ = tim_socket_.receive (boost::asio::buffer (received_packet_));

    //// If received packet is invalid, recurse
    receiveTimPacket ();
  }
}

void
pcl::TimGrabber::parsePacketHeader (std::string const& header) {
  //// note:  Please following part, if you want to use other information in the header.
  //std::size_t pos = header.find (' ');
  //for (int i=0; i<24; ++i)
    //pos = header.find (' ', pos+1);

  std::size_t pos = header.rfind (' ');
  amount_of_data_ = std::stoi (header.substr (pos+1), nullptr, 16);
}

void
pcl::TimGrabber::parsePacketBody (std::string const& body) {
  iss_.clear ();
  iss_.str (body);
  iss_.setf (std::ios::hex, std::ios::basefield);
  int d;
  for (auto& distance : distances_) {
    iss_ >> d;
    distance = d / 1000.0;
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::TimGrabber::processTimPacket (std::string const& packet)
{
  std::size_t header_body_splitter = packet.find (' ');
  for (int i=0; i<25; ++i)
    header_body_splitter = packet.find (' ', header_body_splitter + 1);

  //// packet contains header and body.
  //// header has 26 spaces, the rest is body
  std::string header (packet.substr (0, header_body_splitter));
  std::string body   (packet.substr (header_body_splitter + 1));

  parsePacketHeader (header);

  distances_.resize (amount_of_data_);

  parsePacketBody (body);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::TimGrabber::toPointClouds () {
  point_cloud_xyz_ptr_->resize (distances_.size ());
  for (std::size_t i=0; i<distances_.size (); ++i) {
    point_cloud_xyz_ptr_->points[i].x = distances_[i] * cos_dynamic_lookup_table_[i];
    point_cloud_xyz_ptr_->points[i].y = distances_[i] * sin_dynamic_lookup_table_[i];
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::TimGrabber::publishSignal ()
{
  point_cloud_xyz_signal_->operator () (point_cloud_xyz_ptr_);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::TimGrabber::start ()
{
  if (isRunning ())
    return;

  is_running_ = true;

  try {
    boost::asio::ip::tcp::resolver resolver (tim_io_service_);
    tcp_endpoint_ = *resolver.resolve (tcp_endpoint_);
    tim_socket_.connect (tcp_endpoint_);
  }
  catch (std::exception& e)
  {
    PCL_ERROR ("[pcl::TimGrabber::start] Unable to bind to socket! %s\n", e.what ());
    return;
  }
  grabber_thread_ = std::thread (&TimGrabber::processGrabbing, this);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::TimGrabber::stop ()
{
  if (is_running_) {
    is_running_ = false;
    grabber_thread_.join ();
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::TimGrabber::isRunning () const
{
  return is_running_;
}

std::string
pcl::TimGrabber::getName () const
{
  return (std::string ("Sick Tim Grabber"));
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::TimGrabber::processGrabbing ()
{
  while (is_running_)
  {
    frequency_mutex_.lock ();
    frequency_.event ();
    frequency_mutex_.unlock ();

    receiveTimPacket ();
    processTimPacket (received_packet_.data ());

    updateLookupTables ();
    toPointClouds ();

    publishSignal ();
  }
}
