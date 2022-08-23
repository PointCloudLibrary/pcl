/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012-, Open Perception, Inc.
 *  Copyright (c) 2012,2015 The MITRE Corporation
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

#include <thread>

#include <pcl/console/print.h>
#include <pcl/io/hdl_grabber.h>
#include <boost/version.hpp>
#include <boost/foreach.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/array.hpp>
#include <boost/math/special_functions.hpp>
#ifdef HAVE_PCAP
#include <pcap.h>
#endif // #ifdef HAVE_PCAP

double *pcl::HDLGrabber::cos_lookup_table_ = nullptr;
double *pcl::HDLGrabber::sin_lookup_table_ = nullptr;

using boost::asio::ip::udp;

/////////////////////////////////////////////////////////////////////////////
pcl::HDLGrabber::HDLGrabber (const std::string& correctionsFile,
                             const std::string& pcapFile) :
    last_azimuth_ (65000),
    current_scan_xyz_ (new pcl::PointCloud<pcl::PointXYZ> ()),
    current_sweep_xyz_ (new pcl::PointCloud<pcl::PointXYZ> ()),
    current_scan_xyzi_ (new pcl::PointCloud<pcl::PointXYZI> ()),
    current_sweep_xyzi_ (new pcl::PointCloud<pcl::PointXYZI> ()),
    current_scan_xyzrgba_ (new pcl::PointCloud<pcl::PointXYZRGBA> ()),
    current_sweep_xyzrgba_ (new pcl::PointCloud<pcl::PointXYZRGBA> ()),
    sweep_xyz_signal_ (),
    sweep_xyzrgba_signal_ (),
    sweep_xyzi_signal_ (),
    scan_xyz_signal_ (),
    scan_xyzrgba_signal_ (),
    scan_xyzi_signal_ (),
    source_address_filter_ (),
    source_port_filter_ (443),
    hdl_read_socket_service_ (),
    hdl_read_socket_ (nullptr),
    pcap_file_name_ (pcapFile),
    queue_consumer_thread_ (nullptr),
    hdl_read_packet_thread_ (nullptr),
    min_distance_threshold_ (0.0),
    max_distance_threshold_ (10000.0)
{
  initialize (correctionsFile);
}

/////////////////////////////////////////////////////////////////////////////
pcl::HDLGrabber::HDLGrabber (const boost::asio::ip::address& ipAddress,
                             const std::uint16_t port,
                             const std::string& correctionsFile) :
    last_azimuth_ (65000),
    current_scan_xyz_ (new pcl::PointCloud<pcl::PointXYZ> ()),
    current_sweep_xyz_ (new pcl::PointCloud<pcl::PointXYZ> ()),
    current_scan_xyzi_ (new pcl::PointCloud<pcl::PointXYZI> ()),
    current_sweep_xyzi_ (new pcl::PointCloud<pcl::PointXYZI> ()),
    current_scan_xyzrgba_ (new pcl::PointCloud<pcl::PointXYZRGBA> ()),
    current_sweep_xyzrgba_ (new pcl::PointCloud<pcl::PointXYZRGBA> ()),
    sweep_xyz_signal_ (),
    sweep_xyzrgba_signal_ (),
    sweep_xyzi_signal_ (),
    scan_xyz_signal_ (),
    scan_xyzrgba_signal_ (),
    scan_xyzi_signal_ (),
    udp_listener_endpoint_ (ipAddress, port),
    source_address_filter_ (),
    source_port_filter_ (443),
    hdl_read_socket_service_ (),
    hdl_read_socket_ (nullptr),
    queue_consumer_thread_ (nullptr),
    hdl_read_packet_thread_ (nullptr),
    min_distance_threshold_ (0.0),
    max_distance_threshold_ (10000.0)
{
  initialize (correctionsFile);
}

/////////////////////////////////////////////////////////////////////////////
pcl::HDLGrabber::~HDLGrabber () noexcept
{
  stop ();

  disconnect_all_slots<sig_cb_velodyne_hdl_sweep_point_cloud_xyz> ();
  disconnect_all_slots<sig_cb_velodyne_hdl_sweep_point_cloud_xyzrgba> ();
  disconnect_all_slots<sig_cb_velodyne_hdl_sweep_point_cloud_xyzi> ();
  disconnect_all_slots<sig_cb_velodyne_hdl_scan_point_cloud_xyz> ();
  disconnect_all_slots<sig_cb_velodyne_hdl_scan_point_cloud_xyzrgba> ();
  disconnect_all_slots<sig_cb_velodyne_hdl_scan_point_cloud_xyzi> ();
}

/////////////////////////////////////////////////////////////////////////////
void
pcl::HDLGrabber::initialize (const std::string& correctionsFile)
{
  if (cos_lookup_table_ == nullptr && sin_lookup_table_ == nullptr)
  {
    cos_lookup_table_ = static_cast<double *> (malloc (HDL_NUM_ROT_ANGLES * sizeof (*cos_lookup_table_)));
    sin_lookup_table_ = static_cast<double *> (malloc (HDL_NUM_ROT_ANGLES * sizeof (*sin_lookup_table_)));
    for (std::uint16_t i = 0; i < HDL_NUM_ROT_ANGLES; i++)
    {
      double rad = (M_PI / 180.0) * (static_cast<double> (i) / 100.0);
      cos_lookup_table_[i] = std::cos (rad);
      sin_lookup_table_[i] = std::sin (rad);
    }
  }

  loadCorrectionsFile (correctionsFile);

  for (auto &laser_correction : laser_corrections_)
  {
    HDLLaserCorrection correction = laser_correction;
    laser_correction.sinVertOffsetCorrection = correction.verticalOffsetCorrection * correction.sinVertCorrection;
    laser_correction.cosVertOffsetCorrection = correction.verticalOffsetCorrection * correction.cosVertCorrection;
  }
  sweep_xyz_signal_ = createSignal<sig_cb_velodyne_hdl_sweep_point_cloud_xyz> ();
  sweep_xyzrgba_signal_ = createSignal<sig_cb_velodyne_hdl_sweep_point_cloud_xyzrgba> ();
  sweep_xyzi_signal_ = createSignal<sig_cb_velodyne_hdl_sweep_point_cloud_xyzi> ();
  scan_xyz_signal_ = createSignal<sig_cb_velodyne_hdl_scan_point_cloud_xyz> ();
  scan_xyzrgba_signal_ = createSignal<sig_cb_velodyne_hdl_scan_point_cloud_xyzrgba> ();
  scan_xyzi_signal_ = createSignal<sig_cb_velodyne_hdl_scan_point_cloud_xyzi> ();

  current_scan_xyz_.reset (new pcl::PointCloud<pcl::PointXYZ>);
  current_scan_xyzi_.reset (new pcl::PointCloud<pcl::PointXYZI>);
  current_sweep_xyz_.reset (new pcl::PointCloud<pcl::PointXYZ>);
  current_sweep_xyzi_.reset (new pcl::PointCloud<pcl::PointXYZI>);

  for (auto &rgb : laser_rgb_mapping_)
    rgb.r = rgb.g = rgb.b = 0;

  if (laser_corrections_[32].distanceCorrection == 0.0)
  {
    for (std::uint8_t i = 0; i < 16; i++)
    {
      laser_rgb_mapping_[i * 2].b = static_cast<std::uint8_t> (i * 6 + 64);
      laser_rgb_mapping_[i * 2 + 1].b = static_cast<std::uint8_t> ( (i + 16) * 6 + 64);
    }
  }
  else
  {
    for (std::uint8_t i = 0; i < 16; i++)
    {
      laser_rgb_mapping_[i * 2].b = static_cast<std::uint8_t> (i * 3 + 64);
      laser_rgb_mapping_[i * 2 + 1].b = static_cast<std::uint8_t> ( (i + 16) * 3 + 64);
    }
    for (std::uint8_t i = 0; i < 16; i++)
    {
      laser_rgb_mapping_[i * 2 + 32].b = static_cast<std::uint8_t> (i * 3 + 160);
      laser_rgb_mapping_[i * 2 + 33].b = static_cast<std::uint8_t> ( (i + 16) * 3 + 160);
    }
  }
}

/////////////////////////////////////////////////////////////////////////////
void
pcl::HDLGrabber::loadCorrectionsFile (const std::string& correctionsFile)
{
  if (correctionsFile.empty ())
  {
    loadHDL32Corrections ();
    return;
  }

  boost::property_tree::ptree pt;
  try
  {
    read_xml (correctionsFile, pt, boost::property_tree::xml_parser::trim_whitespace);
  }
  catch (boost::exception const&)
  {
    PCL_ERROR ("[pcl::HDLGrabber::loadCorrectionsFile] Error reading calibration file %s!\n", correctionsFile.c_str ());
    return;
  }

  for (const auto& v : pt.get_child ("boost_serialization.DB.points_"))
  {
    if (v.first == "item")
    {
      const auto& points = v.second;
      for (const auto& px : points)
      {
        if (px.first == "px")
        {
          const auto& calibration_data = px.second;
          std::int32_t index = -1;
          double azimuth = 0, vert_correction = 0, dist_correction = 0, vert_offset_correction = 0, horiz_offset_correction = 0;

          for (const auto& item : calibration_data)
          {
            if (item.first == "id_")
              index = atoi (item.second.data ().c_str ());
            if (item.first == "rotCorrection_")
              azimuth = atof (item.second.data ().c_str ());
            if (item.first == "vertCorrection_")
              vert_correction = atof (item.second.data ().c_str ());
            if (item.first == "distCorrection_")
              dist_correction = atof (item.second.data ().c_str ());
            if (item.first == "vertOffsetCorrection_")
              vert_offset_correction = atof (item.second.data ().c_str ());
            if (item.first == "horizOffsetCorrection_")
              horiz_offset_correction = atof (item.second.data ().c_str ());
          }
          if (index != -1)
          {
            laser_corrections_[index].azimuthCorrection = azimuth;
            laser_corrections_[index].verticalCorrection = vert_correction;
            laser_corrections_[index].distanceCorrection = dist_correction / 100.0;
            laser_corrections_[index].verticalOffsetCorrection = vert_offset_correction / 100.0;
            laser_corrections_[index].horizontalOffsetCorrection = horiz_offset_correction / 100.0;

            laser_corrections_[index].cosVertCorrection = std::cos (HDL_Grabber_toRadians(laser_corrections_[index].verticalCorrection));
            laser_corrections_[index].sinVertCorrection = std::sin (HDL_Grabber_toRadians(laser_corrections_[index].verticalCorrection));
          }
        }
      }
    }
  }
}

/////////////////////////////////////////////////////////////////////////////
void
pcl::HDLGrabber::loadHDL32Corrections ()
{
  double hdl32_vertical_corrections[] = { -30.67, -9.3299999, -29.33, -8, -28, -6.6700001, -26.67, -5.3299999, -25.33, -4, -24, -2.6700001, -22.67, -1.33, -21.33,
      0, -20, 1.33, -18.67, 2.6700001, -17.33, 4, -16, 5.3299999, -14.67, 6.6700001, -13.33, 8, -12, 9.3299999, -10.67, 10.67 };
  for (std::uint8_t i = 0; i < HDL_LASER_PER_FIRING; i++)
  {
    laser_corrections_[i].azimuthCorrection = 0.0;
    laser_corrections_[i].distanceCorrection = 0.0;
    laser_corrections_[i].horizontalOffsetCorrection = 0.0;
    laser_corrections_[i].verticalOffsetCorrection = 0.0;
    laser_corrections_[i].verticalCorrection = hdl32_vertical_corrections[i];
    laser_corrections_[i].sinVertCorrection = std::sin (HDL_Grabber_toRadians(hdl32_vertical_corrections[i]));
    laser_corrections_[i].cosVertCorrection = std::cos (HDL_Grabber_toRadians(hdl32_vertical_corrections[i]));
  }
  for (std::uint8_t i = HDL_LASER_PER_FIRING; i < HDL_MAX_NUM_LASERS; i++)
  {
    laser_corrections_[i].azimuthCorrection = 0.0;
    laser_corrections_[i].distanceCorrection = 0.0;
    laser_corrections_[i].horizontalOffsetCorrection = 0.0;
    laser_corrections_[i].verticalOffsetCorrection = 0.0;
    laser_corrections_[i].verticalCorrection = 0.0;
    laser_corrections_[i].sinVertCorrection = 0.0;
    laser_corrections_[i].cosVertCorrection = 1.0;
  }
}

/////////////////////////////////////////////////////////////////////////////
boost::asio::ip::address
pcl::HDLGrabber::getDefaultNetworkAddress ()
{
  return (boost::asio::ip::address::from_string ("192.168.3.255"));
}

/////////////////////////////////////////////////////////////////////////////
void
pcl::HDLGrabber::processVelodynePackets ()
{
  while (true)
  {
    std::uint8_t *data;
    if (!hdl_data_.dequeue (data))
      return;

    toPointClouds (reinterpret_cast<HDLDataPacket *> (data));

    free (data);
  }
}

/////////////////////////////////////////////////////////////////////////////
void
pcl::HDLGrabber::toPointClouds (HDLDataPacket *dataPacket)
{
  static std::uint32_t scan_counter = 0;
  static std::uint32_t sweep_counter = 0;
  if (sizeof(HDLLaserReturn) != 3)
    return;

  current_scan_xyz_.reset (new pcl::PointCloud<pcl::PointXYZ> ());
  current_scan_xyzrgba_.reset (new pcl::PointCloud<pcl::PointXYZRGBA> ());
  current_scan_xyzi_.reset (new pcl::PointCloud<pcl::PointXYZI> ());

  time_t system_time;
  time (&system_time);
  time_t velodyne_time = (system_time & 0x00000000ffffffffl) << 32 | dataPacket->gpsTimestamp;

  current_scan_xyz_->header.stamp = velodyne_time;
  current_scan_xyzrgba_->header.stamp = velodyne_time;
  current_scan_xyzi_->header.stamp = velodyne_time;
  current_scan_xyz_->header.seq = scan_counter;
  current_scan_xyzrgba_->header.seq = scan_counter;
  current_scan_xyzi_->header.seq = scan_counter;
  scan_counter++;

  for (const auto &firing_data : dataPacket->firingData)
  {
    std::uint8_t offset = (firing_data.blockIdentifier == BLOCK_0_TO_31) ? 0 : 32;

    for (std::uint8_t j = 0; j < HDL_LASER_PER_FIRING; j++)
    {
      if (firing_data.rotationalPosition < last_azimuth_)
      {
        if (!current_sweep_xyzrgba_->empty ())
        {
          current_sweep_xyz_->is_dense = current_sweep_xyzrgba_->is_dense = current_sweep_xyzi_->is_dense = false;
          current_sweep_xyz_->header.stamp = velodyne_time;
          current_sweep_xyzrgba_->header.stamp = velodyne_time;
          current_sweep_xyzi_->header.stamp = velodyne_time;
          current_sweep_xyz_->header.seq = sweep_counter;
          current_sweep_xyzrgba_->header.seq = sweep_counter;
          current_sweep_xyzi_->header.seq = sweep_counter;

          sweep_counter++;

          fireCurrentSweep ();
        }
        current_sweep_xyz_.reset (new pcl::PointCloud<pcl::PointXYZ> ());
        current_sweep_xyzrgba_.reset (new pcl::PointCloud<pcl::PointXYZRGBA> ());
        current_sweep_xyzi_.reset (new pcl::PointCloud<pcl::PointXYZI> ());
      }

      PointXYZ xyz;
      PointXYZI xyzi;
      PointXYZRGBA xyzrgba;

      computeXYZI (xyzi, firing_data.rotationalPosition, firing_data.laserReturns[j], laser_corrections_[j + offset]);

      xyz.x = xyzrgba.x = xyzi.x;
      xyz.y = xyzrgba.y = xyzi.y;
      xyz.z = xyzrgba.z = xyzi.z;

      xyzrgba.rgba = laser_rgb_mapping_[j + offset].rgba;
      if (std::isnan (xyz.x) || std::isnan (xyz.y) || std::isnan (xyz.z))
      {
        continue;
      }

      current_scan_xyz_->push_back (xyz);
      current_scan_xyzi_->push_back (xyzi);
      current_scan_xyzrgba_->push_back (xyzrgba);

      current_sweep_xyz_->push_back (xyz);
      current_sweep_xyzi_->push_back (xyzi);
      current_sweep_xyzrgba_->push_back (xyzrgba);

      last_azimuth_ = firing_data.rotationalPosition;
    }
  }

  current_scan_xyz_->is_dense = current_scan_xyzrgba_->is_dense = current_scan_xyzi_->is_dense = true;
  fireCurrentScan (dataPacket->firingData[0].rotationalPosition, dataPacket->firingData[11].rotationalPosition);
}

/////////////////////////////////////////////////////////////////////////////
void
pcl::HDLGrabber::computeXYZI (pcl::PointXYZI& point,
                              std::uint16_t azimuth,
                              HDLLaserReturn laserReturn,
                              HDLLaserCorrection correction) const
{
  double cos_azimuth, sin_azimuth;

  double distanceM = laserReturn.distance * 0.002;

  point.intensity = static_cast<float> (laserReturn.intensity);
  if (distanceM < min_distance_threshold_ || distanceM > max_distance_threshold_)
  {
    point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN ();
    return;
  }

  if (correction.azimuthCorrection == 0)
  {
    cos_azimuth = cos_lookup_table_[azimuth];
    sin_azimuth = sin_lookup_table_[azimuth];
  }
  else
  {
    double azimuthInRadians = HDL_Grabber_toRadians( (static_cast<double> (azimuth) / 100.0) - correction.azimuthCorrection);
    cos_azimuth = std::cos (azimuthInRadians);
    sin_azimuth = std::sin (azimuthInRadians);
  }

  distanceM += correction.distanceCorrection;

  double xyDistance = distanceM * correction.cosVertCorrection;

  point.x = static_cast<float> (xyDistance * sin_azimuth - correction.horizontalOffsetCorrection * cos_azimuth);
  point.y = static_cast<float> (xyDistance * cos_azimuth + correction.horizontalOffsetCorrection * sin_azimuth);
  point.z = static_cast<float> (distanceM * correction.sinVertCorrection + correction.verticalOffsetCorrection);
  if (point.x == 0 && point.y == 0 && point.z == 0)
  {
    point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN ();
  }
}

/////////////////////////////////////////////////////////////////////////////
void
pcl::HDLGrabber::fireCurrentSweep ()
{
  if (sweep_xyz_signal_ != nullptr && sweep_xyz_signal_->num_slots () > 0)
    sweep_xyz_signal_->operator() (current_sweep_xyz_);

  if (sweep_xyzrgba_signal_ != nullptr && sweep_xyzrgba_signal_->num_slots () > 0)
    sweep_xyzrgba_signal_->operator() (current_sweep_xyzrgba_);

  if (sweep_xyzi_signal_ != nullptr && sweep_xyzi_signal_->num_slots () > 0)
    sweep_xyzi_signal_->operator() (current_sweep_xyzi_);
}

/////////////////////////////////////////////////////////////////////////////
void
pcl::HDLGrabber::fireCurrentScan (const std::uint16_t startAngle,
                                  const std::uint16_t endAngle)
{
  const float start = static_cast<float> (startAngle) / 100.0f;
  const float end = static_cast<float> (endAngle) / 100.0f;

  if (scan_xyz_signal_->num_slots () > 0)
    scan_xyz_signal_->operator () (current_scan_xyz_, start, end);

  if (scan_xyzrgba_signal_->num_slots () > 0)
    scan_xyzrgba_signal_->operator () (current_scan_xyzrgba_, start, end);

  if (scan_xyzi_signal_->num_slots () > 0)
    scan_xyzi_signal_->operator() (current_scan_xyzi_, start, end);
}

/////////////////////////////////////////////////////////////////////////////
void
pcl::HDLGrabber::enqueueHDLPacket (const std::uint8_t *data,
                                   std::size_t bytesReceived)
{
  if (bytesReceived == 1206)
  {
    auto *dup = static_cast<std::uint8_t *> (malloc (bytesReceived * sizeof(std::uint8_t)));
    std::copy(data, data + bytesReceived, dup);

    hdl_data_.enqueue (dup);
  }
}

/////////////////////////////////////////////////////////////////////////////
void
pcl::HDLGrabber::start ()
{
  terminate_read_packet_thread_ = false;

  if (isRunning ())
    return;

  queue_consumer_thread_ = new std::thread (&HDLGrabber::processVelodynePackets, this);

  if (pcap_file_name_.empty ())
  {
    try
    {
      try
      {
        if (isAddressUnspecified (udp_listener_endpoint_.address ()))
        {
          udp_listener_endpoint_.address (getDefaultNetworkAddress ());
        }
        if (udp_listener_endpoint_.port () == 0)
        {
          udp_listener_endpoint_.port (HDL_DATA_PORT);
        }
        hdl_read_socket_ = new udp::socket (hdl_read_socket_service_, udp_listener_endpoint_);
      }
      catch (const std::exception&)
      {
        delete hdl_read_socket_;
        hdl_read_socket_ = new udp::socket (hdl_read_socket_service_, udp::endpoint (boost::asio::ip::address_v4::any (), udp_listener_endpoint_.port ()));
      }
      hdl_read_socket_service_.run ();
    }
    catch (std::exception &e)
    {
      PCL_ERROR("[pcl::HDLGrabber::start] Unable to bind to socket! %s\n", e.what ());
      return;
    }
    hdl_read_packet_thread_ = new std::thread (&HDLGrabber::readPacketsFromSocket, this);
  }
  else
  {
#ifdef HAVE_PCAP
    hdl_read_packet_thread_ = new std::thread (&HDLGrabber::readPacketsFromPcap, this);
#endif // #ifdef HAVE_PCAP
  }
}

/////////////////////////////////////////////////////////////////////////////
void
pcl::HDLGrabber::stop ()
{
  // triggers the exit condition
  terminate_read_packet_thread_ = true;
  hdl_data_.stopQueue ();

  if (hdl_read_packet_thread_ != nullptr)
  {
    hdl_read_packet_thread_->join ();
    delete hdl_read_packet_thread_;
    hdl_read_packet_thread_ = nullptr;
  }
  if (queue_consumer_thread_ != nullptr)
  {
    queue_consumer_thread_->join ();
    delete queue_consumer_thread_;
    queue_consumer_thread_ = nullptr;
  }

  delete hdl_read_socket_;
  hdl_read_socket_ = nullptr;
}

/////////////////////////////////////////////////////////////////////////////
bool
pcl::HDLGrabber::isRunning () const
{
  return (!hdl_data_.isEmpty () || hdl_read_packet_thread_);
}

/////////////////////////////////////////////////////////////////////////////
std::string
pcl::HDLGrabber::getName () const
{
  return (std::string ("Velodyne High Definition Laser (HDL) Grabber"));
}

/////////////////////////////////////////////////////////////////////////////
float
pcl::HDLGrabber::getFramesPerSecond () const
{
  return (0.0f);
}

/////////////////////////////////////////////////////////////////////////////
void
pcl::HDLGrabber::filterPackets (const boost::asio::ip::address& ipAddress,
                                const std::uint16_t port)
{
  source_address_filter_ = ipAddress;
  source_port_filter_ = port;
}

/////////////////////////////////////////////////////////////////////////////
void
pcl::HDLGrabber::setLaserColorRGB (const pcl::RGB& color,
                                   const std::uint8_t laserNumber)
{
  if (laserNumber >= HDL_MAX_NUM_LASERS)
    return;

  laser_rgb_mapping_[laserNumber] = color;
}

/////////////////////////////////////////////////////////////////////////////
bool
pcl::HDLGrabber::isAddressUnspecified (const boost::asio::ip::address& ipAddress)
{
  return (ipAddress.is_unspecified ());
}

/////////////////////////////////////////////////////////////////////////////
void
pcl::HDLGrabber::setMaximumDistanceThreshold (float &maxThreshold)
{
  max_distance_threshold_ = maxThreshold;
}

/////////////////////////////////////////////////////////////////////////////
void
pcl::HDLGrabber::setMinimumDistanceThreshold (float &minThreshold)
{
  min_distance_threshold_ = minThreshold;
}

/////////////////////////////////////////////////////////////////////////////
float
pcl::HDLGrabber::getMaximumDistanceThreshold ()
{
  return (max_distance_threshold_);
}

/////////////////////////////////////////////////////////////////////////////
float
pcl::HDLGrabber::getMinimumDistanceThreshold ()
{
  return (min_distance_threshold_);
}

/////////////////////////////////////////////////////////////////////////////
std::uint8_t
pcl::HDLGrabber::getMaximumNumberOfLasers () const
{
    return (HDL_MAX_NUM_LASERS);
}

/////////////////////////////////////////////////////////////////////////////
void
pcl::HDLGrabber::readPacketsFromSocket ()
{
  std::uint8_t data[1500];
  udp::endpoint sender_endpoint;

  while (!terminate_read_packet_thread_ && hdl_read_socket_->is_open ())
  {
    std::size_t length = hdl_read_socket_->receive_from (boost::asio::buffer (data, 1500), sender_endpoint);

    if (isAddressUnspecified (source_address_filter_)
        || (source_address_filter_ == sender_endpoint.address () && source_port_filter_ == sender_endpoint.port ()))
    {
      enqueueHDLPacket (data, length);
    }
  }
}

/////////////////////////////////////////////////////////////////////////////
#ifdef HAVE_PCAP
void
pcl::HDLGrabber::readPacketsFromPcap ()
{
  struct pcap_pkthdr *header;
  const std::uint8_t *data;
  std::int8_t errbuff[PCAP_ERRBUF_SIZE];

  pcap_t *pcap = pcap_open_offline (pcap_file_name_.c_str (), reinterpret_cast<char *> (errbuff));

  struct bpf_program filter;
  std::ostringstream string_stream;

  string_stream << "udp ";
  if (!isAddressUnspecified (source_address_filter_))
  {
    string_stream << " and src port " << source_port_filter_ << " and src host " << source_address_filter_.to_string ();
  }

  // PCAP_NETMASK_UNKNOWN should be 0xffffffff, but it's undefined in older PCAP versions
  if (pcap_compile (pcap, &filter, string_stream.str ().c_str (), 0, 0xffffffff) == -1)
  {
    PCL_WARN ("[pcl::HDLGrabber::readPacketsFromPcap] Issue compiling filter: %s.\n", pcap_geterr (pcap));
  }
  else if (pcap_setfilter(pcap, &filter) == -1)
  {
    PCL_WARN ("[pcl::HDLGrabber::readPacketsFromPcap] Issue setting filter: %s.\n", pcap_geterr (pcap));
  }

  struct timeval lasttime;

  lasttime.tv_sec = lasttime.tv_usec = 0;

  std::int32_t returnValue = pcap_next_ex (pcap, &header, &data);

  while (returnValue >= 0 && !terminate_read_packet_thread_)
  {
    if (lasttime.tv_sec == 0)
    {
      lasttime.tv_sec = header->ts.tv_sec;
      lasttime.tv_usec = header->ts.tv_usec;
    }
    if (lasttime.tv_usec > header->ts.tv_usec)
    {
      lasttime.tv_usec -= 1000000;
      lasttime.tv_sec++;
    }
    std::uint64_t usec_delay = ((header->ts.tv_sec - lasttime.tv_sec) * 1000000) +
    (header->ts.tv_usec - lasttime.tv_usec);

    std::this_thread::sleep_for(std::chrono::microseconds(usec_delay));

    lasttime.tv_sec = header->ts.tv_sec;
    lasttime.tv_usec = header->ts.tv_usec;

    // The ETHERNET header is 42 bytes long; unnecessary
    enqueueHDLPacket (data + 42, header->len - 42);

    returnValue = pcap_next_ex (pcap, &header, &data);
  }
}
#endif //#ifdef HAVE_PCAP

