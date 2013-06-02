/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012 The MITRE Corporation
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

#include <pcl/console/print.h>
#include <pcl/io/boost.h>
#include <pcl/io/hdl_grabber.h>
#include <boost/version.hpp>
#include <boost/foreach.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/math/special_functions.hpp>
#ifdef HAVE_PCAP
#include <pcap.h>
#endif // #ifdef HAVE_PCAP

const boost::asio::ip::address pcl::HDLGrabber::HDL_DEFAULT_NETWORK_ADDRESS = boost::asio::ip::address::from_string ("192.168.3.255");
double *pcl::HDLGrabber::cos_lookup_table_ = NULL;
double *pcl::HDLGrabber::sin_lookup_table_ = NULL;

using boost::asio::ip::udp;

/////////////////////////////////////////////////////////////////////////////
pcl::HDLGrabber::HDLGrabber (const std::string& correctionsFile,
                             const std::string& pcapFile) 
  : hdl_data_ ()
  , udp_listener_endpoint_ (HDL_DEFAULT_NETWORK_ADDRESS, HDL_DATA_PORT)
  , source_address_filter_ ()
  , source_port_filter_ (443)
  , hdl_read_socket_service_ ()
  , hdl_read_socket_ (NULL)
  , pcap_file_name_ (pcapFile)
  , queue_consumer_thread_ (NULL)
  , hdl_read_packet_thread_ (NULL)
  , current_scan_xyz_ (new pcl::PointCloud<pcl::PointXYZ> ())
  , current_sweep_xyz_ (new pcl::PointCloud<pcl::PointXYZ> ())
  , current_scan_xyzi_ (new pcl::PointCloud<pcl::PointXYZI> ())
  , current_sweep_xyzi_ (new pcl::PointCloud<pcl::PointXYZI> ())
  , current_scan_xyzrgb_ (new pcl::PointCloud<pcl::PointXYZRGBA> ())
  , current_sweep_xyzrgb_ (new pcl::PointCloud<pcl::PointXYZRGBA> ())
  , last_azimuth_ (65000)
  , sweep_xyz_signal_ ()
  , sweep_xyzrgb_signal_ ()
  , sweep_xyzi_signal_ ()
  , scan_xyz_signal_ ()
  , scan_xyzrgb_signal_ ()
  , scan_xyzi_signal_ ()
  , min_distance_threshold_(0.0)
  , max_distance_threshold_(10000.0)
{
  initialize (correctionsFile);
}

/////////////////////////////////////////////////////////////////////////////
pcl::HDLGrabber::HDLGrabber (const boost::asio::ip::address& ipAddress,
                             const unsigned short int port, 
                             const std::string& correctionsFile) 
  : hdl_data_ ()
  , udp_listener_endpoint_ (ipAddress, port)
  , source_address_filter_ ()
  , source_port_filter_ (443)
  , hdl_read_socket_service_ ()
  , hdl_read_socket_ (NULL)
  , pcap_file_name_ ()
  , queue_consumer_thread_ (NULL)
  , hdl_read_packet_thread_ (NULL)
  , current_scan_xyz_ (new pcl::PointCloud<pcl::PointXYZ> ())
  , current_sweep_xyz_ (new pcl::PointCloud<pcl::PointXYZ> ())
  , current_scan_xyzi_ (new pcl::PointCloud<pcl::PointXYZI> ())
  , current_sweep_xyzi_ (new pcl::PointCloud<pcl::PointXYZI> ())
  , current_scan_xyzrgb_ (new pcl::PointCloud<pcl::PointXYZRGBA> ())
  , current_sweep_xyzrgb_ (new pcl::PointCloud<pcl::PointXYZRGBA> ())
  , last_azimuth_ (65000)
  , sweep_xyz_signal_ ()
  , sweep_xyzrgb_signal_ ()
  , sweep_xyzi_signal_ ()
  , scan_xyz_signal_ ()
  , scan_xyzrgb_signal_ ()
  , scan_xyzi_signal_ ()
  , min_distance_threshold_(0.0)
  , max_distance_threshold_(10000.0)
{
  initialize (correctionsFile);
}

/////////////////////////////////////////////////////////////////////////////
pcl::HDLGrabber::~HDLGrabber () throw ()
{
  stop ();

  disconnect_all_slots<sig_cb_velodyne_hdl_sweep_point_cloud_xyz> ();
  disconnect_all_slots<sig_cb_velodyne_hdl_sweep_point_cloud_xyzrgb> ();
  disconnect_all_slots<sig_cb_velodyne_hdl_sweep_point_cloud_xyzi> ();
  disconnect_all_slots<sig_cb_velodyne_hdl_scan_point_cloud_xyz> ();
  disconnect_all_slots<sig_cb_velodyne_hdl_scan_point_cloud_xyzrgb> ();
  disconnect_all_slots<sig_cb_velodyne_hdl_scan_point_cloud_xyzi> ();
}

/////////////////////////////////////////////////////////////////////////////
void
pcl::HDLGrabber::initialize (const std::string& correctionsFile)
{
  if (cos_lookup_table_ == NULL && sin_lookup_table_ == NULL)
  {
    cos_lookup_table_ = static_cast<double *> (malloc (HDL_NUM_ROT_ANGLES * sizeof (*cos_lookup_table_)));
    sin_lookup_table_ = static_cast<double *> (malloc (HDL_NUM_ROT_ANGLES * sizeof (*sin_lookup_table_)));
    for (unsigned int i = 0; i < HDL_NUM_ROT_ANGLES; i++)
    {
      double rad = (M_PI / 180.0) * (static_cast<double> (i) / 100.0);
      cos_lookup_table_[i] = std::cos (rad);
      sin_lookup_table_[i] = std::sin (rad);
    }
  }

  loadCorrectionsFile (correctionsFile);

  for (int i = 0; i < HDL_MAX_NUM_LASERS; i++)
  {
    HDLLaserCorrection correction = laser_corrections_[i];
    laser_corrections_[i].sinVertOffsetCorrection = correction.verticalOffsetCorrection
                                       * correction.sinVertCorrection;
    laser_corrections_[i].cosVertOffsetCorrection = correction.verticalOffsetCorrection
                                       * correction.cosVertCorrection;
  }
  sweep_xyz_signal_ = createSignal<sig_cb_velodyne_hdl_sweep_point_cloud_xyz> ();
  sweep_xyzrgb_signal_ = createSignal<sig_cb_velodyne_hdl_sweep_point_cloud_xyzrgb> ();
  sweep_xyzi_signal_ =createSignal<sig_cb_velodyne_hdl_sweep_point_cloud_xyzi> ();
  scan_xyz_signal_ = createSignal<sig_cb_velodyne_hdl_scan_point_cloud_xyz> ();
  scan_xyzrgb_signal_ = createSignal<sig_cb_velodyne_hdl_scan_point_cloud_xyzrgb> ();
  scan_xyzi_signal_ = createSignal<sig_cb_velodyne_hdl_scan_point_cloud_xyzi> ();

  current_scan_xyz_.reset (new pcl::PointCloud<pcl::PointXYZ>);
  current_scan_xyzi_.reset (new pcl::PointCloud<pcl::PointXYZI>);
  current_sweep_xyz_.reset (new pcl::PointCloud<pcl::PointXYZ>);
  current_sweep_xyzi_.reset (new pcl::PointCloud<pcl::PointXYZI>);

  for (int i = 0; i < HDL_MAX_NUM_LASERS; i++)
    laser_rgb_mapping_[i].r = laser_rgb_mapping_[i].g = laser_rgb_mapping_[i].b = 0;

  if (laser_corrections_[32].distanceCorrection == 0.0)
  {
    for (int i = 0; i < 16; i++)
    {
      laser_rgb_mapping_[i * 2].b = static_cast<uint8_t> (i * 6 + 64);
      laser_rgb_mapping_[i * 2 + 1].b = static_cast<uint8_t> ( (i + 16) * 6 + 64);
    }
  }
  else
  {
    for (int i = 0; i < 16; i++)
    {
      laser_rgb_mapping_[i * 2].b = static_cast<uint8_t> (i * 3 + 64);
      laser_rgb_mapping_[i * 2 + 1].b = static_cast<uint8_t> ( (i + 16) * 3 + 64);
    }
    for (int i = 0; i < 16; i++)
    {
      laser_rgb_mapping_[i * 2 + 32].b = static_cast<uint8_t> (i * 3 + 160);
      laser_rgb_mapping_[i * 2 + 33].b = static_cast<uint8_t> ( (i + 16) * 3 + 160);
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

  BOOST_FOREACH (boost::property_tree::ptree::value_type &v, pt.get_child ("boost_serialization.DB.points_"))
  {
    if (v.first == "item")
    {
      boost::property_tree::ptree points = v.second;
      BOOST_FOREACH(boost::property_tree::ptree::value_type &px, points)
      {
        if (px.first == "px")
        {
          boost::property_tree::ptree calibrationData = px.second;
          int index = -1;
          double azimuth = 0, vertCorrection = 0, distCorrection = 0,
                 vertOffsetCorrection = 0, horizOffsetCorrection = 0;

          BOOST_FOREACH (boost::property_tree::ptree::value_type &item, calibrationData)
          {
            if (item.first == "id_")
              index = atoi (item.second.data ().c_str ());
            if (item.first == "rotCorrection_")
              azimuth = atof (item.second.data ().c_str ());
            if (item.first == "vertCorrection_")
              vertCorrection = atof (item.second.data ().c_str ());
            if (item.first == "distCorrection_")
              distCorrection = atof (item.second.data ().c_str ());
            if (item.first == "vertOffsetCorrection_")
              vertOffsetCorrection = atof (item.second.data ().c_str ());
            if (item.first == "horizOffsetCorrection_")
              horizOffsetCorrection = atof (item.second.data ().c_str ());
          }
          if (index != -1)
          {
            laser_corrections_[index].azimuthCorrection = azimuth;
            laser_corrections_[index].verticalCorrection = vertCorrection;
            laser_corrections_[index].distanceCorrection = distCorrection / 100.0;
            laser_corrections_[index].verticalOffsetCorrection = vertOffsetCorrection / 100.0;
            laser_corrections_[index].horizontalOffsetCorrection = horizOffsetCorrection / 100.0;

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
  double hdl32VerticalCorrections[] = { 
    -30.67, -9.3299999, -29.33, -8, -28,
    -6.6700001, -26.67, -5.3299999, -25.33, -4, -24, -2.6700001, -22.67,
    -1.33, -21.33, 0, -20, 1.33, -18.67, 2.6700001, -17.33, 4, -16, 5.3299999,
    -14.67, 6.6700001, -13.33, 8, -12, 9.3299999, -10.67, 10.67 };
  for (int i = 0; i < HDL_LASER_PER_FIRING; i++)
  {
    laser_corrections_[i].azimuthCorrection = 0.0;
    laser_corrections_[i].distanceCorrection = 0.0;
    laser_corrections_[i].horizontalOffsetCorrection = 0.0;
    laser_corrections_[i].verticalOffsetCorrection = 0.0;
    laser_corrections_[i].verticalCorrection = hdl32VerticalCorrections[i];
    laser_corrections_[i].sinVertCorrection = std::sin (HDL_Grabber_toRadians(hdl32VerticalCorrections[i]));
    laser_corrections_[i].cosVertCorrection = std::cos (HDL_Grabber_toRadians(hdl32VerticalCorrections[i]));
  }
  for (int i = HDL_LASER_PER_FIRING; i < HDL_MAX_NUM_LASERS; i++)
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
void
pcl::HDLGrabber::processVelodynePackets ()
{
  while (true)
  {
    unsigned char *data;
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
  static uint32_t scanCounter = 0;
  static uint32_t sweepCounter = 0;
  if (sizeof (HDLLaserReturn) != 3)
    return;

  current_scan_xyz_.reset (new pcl::PointCloud<pcl::PointXYZ> ());
  current_scan_xyzrgb_.reset (new pcl::PointCloud<pcl::PointXYZRGBA> ());
  current_scan_xyzi_.reset (new pcl::PointCloud<pcl::PointXYZI> ());

  time_t  time_;
  time(&time_);
  time_t velodyneTime = (time_ & 0x00000000ffffffffl) << 32 | dataPacket->gpsTimestamp;

  current_scan_xyz_->header.stamp = velodyneTime;
  current_scan_xyzrgb_->header.stamp = velodyneTime;
  current_scan_xyzi_->header.stamp = velodyneTime;
  current_scan_xyz_->header.seq = scanCounter;
  current_scan_xyzrgb_->header.seq = scanCounter;
  current_scan_xyzi_->header.seq = scanCounter;
  scanCounter++;

  for (int i = 0; i < HDL_FIRING_PER_PKT; ++i)
  {
    HDLFiringData firingData = dataPacket->firingData[i];
    int offset = (firingData.blockIdentifier == BLOCK_0_TO_31) ? 0 : 32;

    for (int j = 0; j < HDL_LASER_PER_FIRING; j++)
    {
      if (firingData.rotationalPosition < last_azimuth_)
      {
        if (current_sweep_xyzrgb_->size () > 0)
        {
          current_sweep_xyz_->is_dense = current_sweep_xyzrgb_->is_dense = current_sweep_xyzi_->is_dense = false;
          current_sweep_xyz_->header.stamp = velodyneTime;
          current_sweep_xyzrgb_->header.stamp = velodyneTime;
          current_sweep_xyzi_->header.stamp = velodyneTime;
          current_sweep_xyz_->header.seq = sweepCounter;
          current_sweep_xyzrgb_->header.seq = sweepCounter;
          current_sweep_xyzi_->header.seq = sweepCounter;

          sweepCounter++;

          fireCurrentSweep ();
        }
        current_sweep_xyz_.reset (new pcl::PointCloud<pcl::PointXYZ> ());
        current_sweep_xyzrgb_.reset (new pcl::PointCloud<pcl::PointXYZRGBA> ());
        current_sweep_xyzi_.reset (new pcl::PointCloud<pcl::PointXYZI> ());
      }

      PointXYZ xyz;
      PointXYZI xyzi;
      PointXYZRGBA xyzrgb;

      computeXYZI (xyzi, firingData.rotationalPosition, firingData.laserReturns[j], laser_corrections_[j + offset]);

      xyz.x = xyzrgb.x = xyzi.x;
      xyz.y = xyzrgb.y = xyzi.y;
      xyz.z = xyzrgb.z = xyzi.z;

      xyzrgb.rgba = laser_rgb_mapping_[j + offset].rgba;
      if ((boost::math::isnan)(xyz.x) ||
          (boost::math::isnan)(xyz.y) ||
          (boost::math::isnan)(xyz.z)) {
        continue;
      }

      current_scan_xyz_->push_back (xyz);
      current_scan_xyzi_->push_back (xyzi);
      current_scan_xyzrgb_->push_back (xyzrgb);

      current_sweep_xyz_->push_back (xyz);
      current_sweep_xyzi_->push_back (xyzi);
      current_sweep_xyzrgb_->push_back (xyzrgb);

      last_azimuth_ = firingData.rotationalPosition;
    }
  }

  current_scan_xyz_->is_dense = current_scan_xyzrgb_->is_dense = current_scan_xyzi_->is_dense = true;
  fireCurrentScan (dataPacket->firingData[0].rotationalPosition, 
                   dataPacket->firingData[11].rotationalPosition);
}

/////////////////////////////////////////////////////////////////////////////
void
pcl::HDLGrabber::computeXYZI (pcl::PointXYZI& point, int azimuth, 
                              HDLLaserReturn laserReturn, HDLLaserCorrection correction)
{
  double cosAzimuth, sinAzimuth;

  double distanceM = laserReturn.distance * 0.002;

  if (distanceM < min_distance_threshold_ || distanceM > max_distance_threshold_) {
    point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN();
    point.intensity = static_cast<float> (laserReturn.intensity);
    return;
  }

  if (correction.azimuthCorrection == 0)
  {
    cosAzimuth = cos_lookup_table_[azimuth];
    sinAzimuth = sin_lookup_table_[azimuth];
  }
  else
  {
    double azimuthInRadians = HDL_Grabber_toRadians ((static_cast<double> (azimuth) / 100.0) - correction.azimuthCorrection);
    cosAzimuth = std::cos (azimuthInRadians);
    sinAzimuth = std::sin (azimuthInRadians);
  }

  distanceM += correction.distanceCorrection;

  double xyDistance = distanceM * correction.cosVertCorrection - correction.sinVertOffsetCorrection;

  point.x = static_cast<float> (xyDistance * sinAzimuth - correction.horizontalOffsetCorrection * cosAzimuth);
  point.y = static_cast<float> (xyDistance * cosAzimuth + correction.horizontalOffsetCorrection * sinAzimuth);
  point.z = static_cast<float> (distanceM * correction.sinVertCorrection + correction.cosVertOffsetCorrection);
  point.intensity = static_cast<float> (laserReturn.intensity);
}

/////////////////////////////////////////////////////////////////////////////
void
pcl::HDLGrabber::fireCurrentSweep ()
{
  if (sweep_xyz_signal_->num_slots () > 0)
    sweep_xyz_signal_->operator() (current_sweep_xyz_);
  
  if (sweep_xyzrgb_signal_->num_slots () > 0)
    sweep_xyzrgb_signal_->operator() (current_sweep_xyzrgb_);

  if (sweep_xyzi_signal_->num_slots () > 0)
    sweep_xyzi_signal_->operator() (current_sweep_xyzi_);
}

/////////////////////////////////////////////////////////////////////////////
void
pcl::HDLGrabber::fireCurrentScan (const unsigned short startAngle,
    const unsigned short endAngle)
{
  const float start = static_cast<float> (startAngle) / 100.0f;
  const float end = static_cast<float> (endAngle) / 100.0f;

  if (scan_xyz_signal_->num_slots () > 0)
    scan_xyz_signal_->operator () (current_scan_xyz_, start, end);

  if (scan_xyzrgb_signal_->num_slots () > 0)
    scan_xyzrgb_signal_->operator () (current_scan_xyzrgb_, start, end);

  if (scan_xyzi_signal_->num_slots () > 0)
    scan_xyzi_signal_->operator() (current_scan_xyzi_, start, end);
}

/////////////////////////////////////////////////////////////////////////////
void
pcl::HDLGrabber::enqueueHDLPacket (const unsigned char *data,
    std::size_t bytesReceived)
{
  if (bytesReceived == 1206)
  {
    unsigned char *dup = static_cast<unsigned char *> (malloc (bytesReceived * sizeof(unsigned char)));
    memcpy (dup, data, bytesReceived * sizeof(unsigned char));

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

  queue_consumer_thread_ = new boost::thread (boost::bind (&HDLGrabber::processVelodynePackets, this));

  if (pcap_file_name_.empty ())
  {
    try
    {
      try {
		  hdl_read_socket_ = new udp::socket (hdl_read_socket_service_, udp_listener_endpoint_);
	  }
	  catch (std::exception bind) {
		  delete hdl_read_socket_;
		  hdl_read_socket_ = new udp::socket (hdl_read_socket_service_, udp::endpoint(boost::asio::ip::address_v4::any(), udp_listener_endpoint_.port()));
	  }
      hdl_read_socket_service_.run ();
    }
    catch (std::exception &e)
    {
		PCL_ERROR ("[pcl::HDLGrabber::start] Unable to bind to socket! %s\n", e.what());
        return;
    }
    hdl_read_packet_thread_ = new boost::thread (boost::bind (&HDLGrabber::readPacketsFromSocket, this));
  }
  else
  {
#ifdef HAVE_PCAP
    hdl_read_packet_thread_ = new boost::thread(boost::bind(&HDLGrabber::readPacketsFromPcap, this));
#endif // #ifdef HAVE_PCAP
  }
}

/////////////////////////////////////////////////////////////////////////////
void
pcl::HDLGrabber::stop ()
{
  terminate_read_packet_thread_ = true;
  hdl_data_.stopQueue ();

  if (hdl_read_packet_thread_ != NULL)
  {
    hdl_read_packet_thread_->interrupt ();
    hdl_read_packet_thread_->join ();
    delete hdl_read_packet_thread_;
    hdl_read_packet_thread_ = NULL;
  }
  if (queue_consumer_thread_ != NULL)
  {
    queue_consumer_thread_->join ();
    delete queue_consumer_thread_;
    queue_consumer_thread_ = NULL;
  }

  if (hdl_read_socket_ != NULL)
  {
    delete hdl_read_socket_;
    hdl_read_socket_ = NULL;
  }
}

/////////////////////////////////////////////////////////////////////////////
bool
pcl::HDLGrabber::isRunning () const
{
	return (!hdl_data_.isEmpty() || (hdl_read_packet_thread_ != NULL && 
         !hdl_read_packet_thread_->timed_join (boost::posix_time::milliseconds (10))));
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
                                const unsigned short port)
{
  source_address_filter_ = ipAddress;
  source_port_filter_ = port;
}

/////////////////////////////////////////////////////////////////////////////
void
pcl::HDLGrabber::setLaserColorRGB (const pcl::RGB& color,
                                   unsigned int laserNumber)
{
  if (laserNumber >= HDL_MAX_NUM_LASERS)
    return;

  laser_rgb_mapping_[laserNumber] = color;
}

/////////////////////////////////////////////////////////////////////////////
bool
pcl::HDLGrabber::isAddressUnspecified (const boost::asio::ip::address& ipAddress)
{
#if BOOST_VERSION>=104700
  return (ipAddress.is_unspecified ());
#else
  if (ipAddress.is_v4 ())
    return (ipAddress.to_v4 ().to_ulong() == 0);

  return (false);
#endif
}

/////////////////////////////////////////////////////////////////////////////
void
pcl::HDLGrabber::setMaximumDistanceThreshold(float &maxThreshold) {
  max_distance_threshold_ = maxThreshold;
}

/////////////////////////////////////////////////////////////////////////////
void
pcl::HDLGrabber::setMinimumDistanceThreshold(float &minThreshold) {
  min_distance_threshold_ = minThreshold;
}

/////////////////////////////////////////////////////////////////////////////
float
pcl::HDLGrabber::getMaximumDistanceThreshold() {
  return(max_distance_threshold_);
}

/////////////////////////////////////////////////////////////////////////////
float
pcl::HDLGrabber::getMinimumDistanceThreshold() {
  return(min_distance_threshold_);
}

/////////////////////////////////////////////////////////////////////////////
void
pcl::HDLGrabber::readPacketsFromSocket ()
{
  unsigned char data[1500];
  udp::endpoint sender_endpoint;

  while (!terminate_read_packet_thread_ && hdl_read_socket_->is_open())
  {
    size_t length = hdl_read_socket_->receive_from (boost::asio::buffer (data, 1500), sender_endpoint);

    if (isAddressUnspecified (source_address_filter_) || 
        (source_address_filter_ == sender_endpoint.address () && source_port_filter_ == sender_endpoint.port ()))
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
  const unsigned char *data;
  char errbuff[PCAP_ERRBUF_SIZE];

  pcap_t *pcap = pcap_open_offline (pcap_file_name_.c_str (), errbuff);

  struct bpf_program filter;
  std::ostringstream stringStream;

  stringStream << "udp ";
  if (!isAddressUnspecified(source_address_filter_))
  {
    stringStream << " and src port " << source_port_filter_ << " and src host " << source_address_filter_.to_string();
  }

  // PCAP_NETMASK_UNKNOWN should be 0xffffffff, but it's undefined in older PCAP versions
  if (pcap_compile (pcap, &filter, stringStream.str ().c_str(), 0, 0xffffffff) == -1)
  {
    PCL_WARN ("[pcl::HDLGrabber::readPacketsFromPcap] Issue compiling filter: %s.\n", pcap_geterr (pcap));
  }
  else if (pcap_setfilter(pcap, &filter) == -1)
  {
    PCL_WARN ("[pcl::HDLGrabber::readPacketsFromPcap] Issue setting filter: %s.\n", pcap_geterr (pcap));
  }

  struct timeval lasttime;
  unsigned long long uSecDelay;

  lasttime.tv_sec = 0;

  int returnValue = pcap_next_ex(pcap, &header, &data);

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
    uSecDelay = ((header->ts.tv_sec - lasttime.tv_sec) * 1000000) +
                (header->ts.tv_usec - lasttime.tv_usec);

    boost::this_thread::sleep(boost::posix_time::microseconds(uSecDelay));

    lasttime.tv_sec = header->ts.tv_sec;
    lasttime.tv_usec = header->ts.tv_usec;

    // The ETHERNET header is 42 bytes long; unnecessary
    enqueueHDLPacket(data + 42, header->len - 42);

    returnValue = pcap_next_ex(pcap, &header, &data);
  }
}
#endif //#ifdef HAVE_PCAP

