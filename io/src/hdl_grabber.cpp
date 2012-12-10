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

#include <pcl/io/boost.h>
#include <pcl/io/hdl_grabber.h>
#include <boost/version.hpp>
#include <boost/foreach.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#ifdef USE_PCAP
#include <pcap.h>
#endif // #ifdef USE_PCAP
const boost::asio::ip::address pcl::HDL_Grabber::DEFAULT_NETWORK_ADDRESS =
    boost::asio::ip::address::from_string ("192.168.3.255");
double *pcl::HDL_Grabber::rotCosTable = NULL;
double *pcl::HDL_Grabber::rotSinTable = NULL;

using boost::asio::ip::udp;

pcl::HDL_Grabber::HDL_Grabber (const std::string& correctionsFile,
    const std::string& pcapFile) :
    hdlData (), udpEndpoint (DEFAULT_NETWORK_ADDRESS, DATA_PORT), sourceAddress (), sourcePort (
        443), hdlReadSocket (NULL), pcapFileName (pcapFile), consumeFromQueue (
        NULL), readerThread (NULL), currentScanXYZ (
        new pcl::PointCloud<pcl::PointXYZ> ()), currentSweepXYZ (
        new pcl::PointCloud<pcl::PointXYZ> ()), currentScanXYZI (
        new pcl::PointCloud<pcl::PointXYZI> ()), currentSweepXYZI (
        new pcl::PointCloud<pcl::PointXYZI> ()), currentScanXYZRGB (
        new pcl::PointCloud<pcl::PointXYZRGB> ()), currentSweepXYZRGB (
        new pcl::PointCloud<pcl::PointXYZRGB> ()), lastAzimuth (65000), sweep_xyz_signal (), sweep_xyzi_signal (), scan_xyz_signal (), scan_xyzi_signal ()
{
  initialize (correctionsFile);
}

pcl::HDL_Grabber::HDL_Grabber (const boost::asio::ip::address& ipAddress,
    const unsigned short int port, const std::string& correctionsFile) :
    hdlData (), udpEndpoint (ipAddress, port), sourceAddress (), sourcePort (
        443), hdlReadSocket (NULL), pcapFileName (), consumeFromQueue (NULL), readerThread (
        NULL), currentScanXYZ (new pcl::PointCloud<pcl::PointXYZ> ()), currentSweepXYZ (
        new pcl::PointCloud<pcl::PointXYZ> ()), currentScanXYZI (
        new pcl::PointCloud<pcl::PointXYZI> ()), currentSweepXYZI (
        new pcl::PointCloud<pcl::PointXYZI> ()), currentScanXYZRGB (
        new pcl::PointCloud<pcl::PointXYZRGB> ()), currentSweepXYZRGB (
        new pcl::PointCloud<pcl::PointXYZRGB> ()), lastAzimuth (65000), sweep_xyz_signal (), sweep_xyzrgb_signal (), sweep_xyzi_signal (), scan_xyz_signal (), scan_xyzrgb_signal (), scan_xyzi_signal ()
{
  initialize (correctionsFile);
}

pcl::HDL_Grabber::~HDL_Grabber () throw ()
{
  stop ();

  disconnect_all_slots<sig_cb_velodyne_hdl_sweep_point_cloud_xyz> ();
  disconnect_all_slots<sig_cb_velodyne_hdl_sweep_point_cloud_xyzrgb> ();
  disconnect_all_slots<sig_cb_velodyne_hdl_sweep_point_cloud_xyzi> ();
  disconnect_all_slots<sig_cb_velodyne_hdl_scan_point_cloud_xyz> ();
  disconnect_all_slots<sig_cb_velodyne_hdl_scan_point_cloud_xyzrgb> ();
  disconnect_all_slots<sig_cb_velodyne_hdl_scan_point_cloud_xyzi> ();
}

void pcl::HDL_Grabber::initialize (const std::string& correctionsFile)
{
  if (rotCosTable == NULL && rotSinTable == NULL)
  {
    rotCosTable = static_cast<double *> (malloc (
        HDL_NUM_ROT_ANGLES * sizeof (*rotCosTable)));
    rotSinTable = static_cast<double *> (malloc (
        HDL_NUM_ROT_ANGLES * sizeof (*rotSinTable)));
    for (unsigned int i = 0; i < HDL_NUM_ROT_ANGLES; i++)
    {
      double rad = (M_PI / 180.0) * (static_cast<double> (i) / 100.0);
      rotCosTable[i] = std::cos (rad);
      rotSinTable[i] = std::sin (rad);
    }
  }

  loadCorrectionsFile (correctionsFile);

  for (int i = 0; i < HDL_MAX_NUM_LASERS; i++)
  {
    HDLLaserCorrection correction = laserCorrections[i];
    correction.sinVertOffsetCorrection = correction.verticalOffsetCorrection
        * correction.sinVertCorrection;
    correction.cosVertOffsetCorrection = correction.verticalOffsetCorrection
        * correction.cosVertCorrection;
  }
  sweep_xyz_signal = createSignal<sig_cb_velodyne_hdl_sweep_point_cloud_xyz> ();
  sweep_xyzrgb_signal = createSignal<
      sig_cb_velodyne_hdl_sweep_point_cloud_xyzrgb> ();
  sweep_xyzi_signal =
      createSignal<sig_cb_velodyne_hdl_sweep_point_cloud_xyzi> ();
  scan_xyz_signal = createSignal<sig_cb_velodyne_hdl_scan_point_cloud_xyz> ();
  scan_xyzrgb_signal =
      createSignal<sig_cb_velodyne_hdl_scan_point_cloud_xyzrgb> ();
  scan_xyzi_signal = createSignal<sig_cb_velodyne_hdl_scan_point_cloud_xyzi> ();

  currentScanXYZ.reset (new pcl::PointCloud<pcl::PointXYZ>);
  currentScanXYZI.reset (new pcl::PointCloud<pcl::PointXYZI>);
  currentSweepXYZ.reset (new pcl::PointCloud<pcl::PointXYZ>);
  currentSweepXYZI.reset (new pcl::PointCloud<pcl::PointXYZI>);

  for (int i = 0; i < HDL_MAX_NUM_LASERS; i++)
  {
    laserColors[i].r = laserColors[i].g = laserColors[i].b = 0;
  }
  if (laserCorrections[32].distanceCorrection == 0.0)
  {
    for (int i = 0; i < 16; i++)
    {
      laserColors[i * 2].b = static_cast<uint8_t> (i * 6 + 64);
      laserColors[i * 2 + 1].b = static_cast<uint8_t> ( (i + 16) * 6 + 64);
    }
  }
  else
  {
    for (int i = 0; i < 16; i++)
    {
      laserColors[i * 2].b = static_cast<uint8_t> (i * 3 + 64);
      laserColors[i * 2 + 1].b = static_cast<uint8_t> ( (i + 16) * 3 + 64);
    }
    for (int i = 0; i < 16; i++)
    {
      laserColors[i * 2 + 32].b = static_cast<uint8_t> (i * 3 + 160);
      laserColors[i * 2 + 33].b = static_cast<uint8_t> ( (i + 16) * 3 + 160);
    }
  }
}

void pcl::HDL_Grabber::loadCorrectionsFile (const std::string& correctionsFile)
{
  if (correctionsFile.empty ())
  {
    loadHDL32Corrections ();
    return;
  }

  boost::property_tree::ptree pt;
  try
  {
    read_xml (correctionsFile, pt,
        boost::property_tree::xml_parser::trim_whitespace);
  }
  catch (boost::exception const& ex)
  {
    std::cerr << "Error reading calibration file " << correctionsFile << "."
        << std::endl;
    return;
  }

  BOOST_FOREACH(boost::property_tree::ptree::value_type &v, pt.get_child("boost_serialization.DB.points_"))
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

          BOOST_FOREACH(boost::property_tree::ptree::value_type &item, calibrationData)
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
            laserCorrections[index].azimuthCorrection = azimuth;
            laserCorrections[index].verticalCorrection = vertCorrection;
            laserCorrections[index].distanceCorrection = distCorrection / 100.0;
            laserCorrections[index].verticalOffsetCorrection =
                vertOffsetCorrection / 100.0;
            laserCorrections[index].horizontalOffsetCorrection =
                horizOffsetCorrection / 100.0;

            laserCorrections[index].cosVertCorrection =
                std::cos (
                    HDL_Grabber_toRadians(laserCorrections[index].verticalCorrection));
            laserCorrections[index].sinVertCorrection =
                std::sin (
                    HDL_Grabber_toRadians(laserCorrections[index].verticalCorrection));
          }
        }
      }
    }
  }
}

void pcl::HDL_Grabber::loadHDL32Corrections ()
{
  double hdl32VerticalCorrections[] = { -30.67, -9.3299999, -29.33, -8, -28,
      -6.6700001, -26.67, -5.3299999, -25.33, -4, -24, -2.6700001, -22.67,
      -1.33, -21.33, 0, -20, 1.33, -18.67, 2.6700001, -17.33, 4, -16, 5.3299999,
      -14.67, 6.6700001, -13.33, 8, -12, 9.3299999, -10.67, 10.67 };
  for (int i = 0; i < HDL_LASER_PER_FIRING; i++)
  {
    laserCorrections[i].azimuthCorrection = 0.0;
    laserCorrections[i].distanceCorrection = 0.0;
    laserCorrections[i].horizontalOffsetCorrection = 0.0;
    laserCorrections[i].verticalOffsetCorrection = 0.0;
    laserCorrections[i].verticalCorrection = hdl32VerticalCorrections[i];
    laserCorrections[i].sinVertCorrection = std::sin (
        HDL_Grabber_toRadians(hdl32VerticalCorrections[i]));
    laserCorrections[i].cosVertCorrection = std::cos (
        HDL_Grabber_toRadians(hdl32VerticalCorrections[i]));
  }
  for (int i = HDL_LASER_PER_FIRING; i < HDL_MAX_NUM_LASERS; i++)
  {
    laserCorrections[i].azimuthCorrection = 0.0;
    laserCorrections[i].distanceCorrection = 0.0;
    laserCorrections[i].horizontalOffsetCorrection = 0.0;
    laserCorrections[i].verticalOffsetCorrection = 0.0;
    laserCorrections[i].verticalCorrection = 0.0;
    laserCorrections[i].sinVertCorrection = 0.0;
    laserCorrections[i].cosVertCorrection = 1.0;
  }
}

void pcl::HDL_Grabber::processVelodynePackets ()
{
  while (true)
  {
    unsigned char *data;
    if (!hdlData.Dequeue (data))
    {
      return;
    }

    toPointClouds (reinterpret_cast<HDLDataPacket *> (data));

    free (data);
  }
}

void pcl::HDL_Grabber::toPointClouds (HDLDataPacket *dataPacket)
{
  if (sizeof(HDLLaserReturn) != 3)
  {
    return;
  }
  currentScanXYZ.reset (new pcl::PointCloud<pcl::PointXYZ> ());
  currentScanXYZRGB.reset (new pcl::PointCloud<pcl::PointXYZRGB> ());
  currentScanXYZI.reset (new pcl::PointCloud<pcl::PointXYZI> ());

  for (int i = 0; i < HDL_FIRING_PER_PKT; i++)
  {
    HDLFiringData firingData = dataPacket->firingData[i];
    int offset = (firingData.blockIdentifier == BLOCK_0_TO_31) ? 0 : 32;

    for (int j = 0; j < HDL_LASER_PER_FIRING; j++)
    {
      if (firingData.rotationalPosition < lastAzimuth)
      {
        if (currentSweepXYZRGB->size () > 0)
        {
          currentSweepXYZ->is_dense = currentSweepXYZRGB->is_dense =
              currentSweepXYZI->is_dense = false;

          fireCurrentSweep ();
        }
        currentSweepXYZ.reset (new pcl::PointCloud<pcl::PointXYZ> ());
        currentSweepXYZRGB.reset (new pcl::PointCloud<pcl::PointXYZRGB> ());
        currentSweepXYZI.reset (new pcl::PointCloud<pcl::PointXYZI> ());
      }

      PointXYZ xyz;
      PointXYZI xyzi;
      PointXYZRGB xyzrgb;
      computeXYZI (xyzi, firingData.rotationalPosition,
          firingData.laserReturns[j], laserCorrections[j + offset]);

      xyz.x = xyzrgb.x = xyzi.x;
      xyz.y = xyzrgb.y = xyzi.y;
      xyz.z = xyzrgb.z = xyzi.z;

      xyzrgb.rgba = laserColors[j + offset].rgba;

      currentScanXYZ->push_back (xyz);
      currentScanXYZI->push_back (xyzi);
      currentScanXYZRGB->push_back (xyzrgb);

      currentSweepXYZ->push_back (xyz);
      currentSweepXYZI->push_back (xyzi);
      currentSweepXYZRGB->push_back (xyzrgb);

      lastAzimuth = firingData.rotationalPosition;
    }
  }

  currentScanXYZ->is_dense = currentScanXYZRGB->is_dense =
      currentScanXYZI->is_dense = false;

  fireCurrentScan (dataPacket->firingData[0].rotationalPosition,
      dataPacket->firingData[11].rotationalPosition);
}

void pcl::HDL_Grabber::computeXYZI (pcl::PointXYZI& pointXYZI, int azimuth,
    HDLLaserReturn laserReturn, HDLLaserCorrection correction)
{
  double cosAzimuth, sinAzimuth;
  if (correction.azimuthCorrection == 0)
  {
    cosAzimuth = rotCosTable[azimuth];
    sinAzimuth = rotSinTable[azimuth];
  }
  else
  {
    double azimuthInRadians =
        HDL_Grabber_toRadians((static_cast<double>(azimuth) / 100.0) - correction.azimuthCorrection);
    cosAzimuth = std::cos (azimuthInRadians);
    sinAzimuth = std::sin (azimuthInRadians);
  }
  double distanceCM = laserReturn.distance * 0.2
      + correction.distanceCorrection;

  double xyDistance = distanceCM * correction.cosVertCorrection
      - correction.sinVertOffsetCorrection;

  pointXYZI.x = static_cast<float> (xyDistance * sinAzimuth
      - correction.horizontalOffsetCorrection * cosAzimuth);
  pointXYZI.y = static_cast<float> (xyDistance * cosAzimuth
      + correction.horizontalOffsetCorrection * sinAzimuth);
  pointXYZI.z = static_cast<float> (distanceCM * correction.sinVertCorrection
      + correction.cosVertOffsetCorrection);
  pointXYZI.intensity = static_cast<float> (laserReturn.intensity);
}

void pcl::HDL_Grabber::fireCurrentSweep ()
{
  if (sweep_xyz_signal->num_slots () > 0)
  {
    sweep_xyz_signal->operator() (currentSweepXYZ);
  }
  if (sweep_xyzrgb_signal->num_slots () > 0)
  {
    sweep_xyzrgb_signal->operator() (currentSweepXYZRGB);
  }
  if (sweep_xyzi_signal->num_slots () > 0)
  {
    sweep_xyzi_signal->operator() (currentSweepXYZI);
  }
}

void pcl::HDL_Grabber::fireCurrentScan (const unsigned short startAngle,
    const unsigned short endAngle)
{
  const float start = static_cast<float> (startAngle) / 100.0f;
  const float end = static_cast<float> (endAngle) / 100.0f;

  if (scan_xyz_signal->num_slots () > 0)
  {
    scan_xyz_signal->operator () (currentScanXYZ, start, end);
  }
  if (scan_xyzrgb_signal->num_slots () > 0)
  {
    scan_xyzrgb_signal->operator () (currentScanXYZRGB, start, end);
  }
  if (scan_xyzi_signal->num_slots () > 0)
  {
    scan_xyzi_signal->operator() (currentScanXYZI, start, end);
  }
}

void pcl::HDL_Grabber::enqueueHDLPacket (const unsigned char *data,
    std::size_t bytesReceived)
{
  if (bytesReceived == 1206)
  {
    unsigned char *dup = static_cast<unsigned char *> (malloc (
        bytesReceived * sizeof(unsigned char)));
    memcpy (dup, data, bytesReceived * sizeof(unsigned char));

    hdlData.Enqueue (dup);
  }
}

void pcl::HDL_Grabber::start ()
{
  terminateReaderThread = false;

  if (isRunning ())
  {
    return;
  }

  consumeFromQueue = new boost::thread (
      boost::bind (&HDL_Grabber::processVelodynePackets, this));

  if (pcapFileName.empty ())
  {
    try
    {
      boost::asio::io_service io_service;
      hdlReadSocket = new udp::socket (io_service, udpEndpoint);
      io_service.run ();
    }
    catch (std::exception &e)
    {
      std::cerr << "Unable to bind to " << udpEndpoint << std::endl;
      return;
    }
    readerThread = new boost::thread (
        boost::bind (&HDL_Grabber::readPacketsFromSocket, this));
  }
  else
  {
#ifdef USE_PCAP
    readerThread = new boost::thread(boost::bind(&HDL_Grabber::readPacketsFromPcap, this));
#endif // #ifdef USE_PCAP
  }
}
void pcl::HDL_Grabber::stop ()
{
  terminateReaderThread = true;
  hdlData.StopQueue ();

  if (readerThread != NULL)
  {
    readerThread->interrupt ();
    readerThread->join ();
    delete readerThread;
    readerThread = NULL;
  }
  if (consumeFromQueue != NULL)
  {
    consumeFromQueue->join ();
    delete consumeFromQueue;
    consumeFromQueue = NULL;
  }

  if (hdlReadSocket != NULL)
  {
    delete hdlReadSocket;
    hdlReadSocket = NULL;
  }
}

bool pcl::HDL_Grabber::isRunning () const
{
  return (readerThread != NULL
      && !readerThread->timed_join (boost::posix_time::milliseconds (10)));
}

std::string pcl::HDL_Grabber::getName () const
{
  return (std::string ("Velodyne High Definition Laser (HDL) Grabber"));
}

float pcl::HDL_Grabber::getFramesPerSecond () const
{
  return (0.0f);
}

void pcl::HDL_Grabber::filterPackets (const boost::asio::ip::address& ipAddress,
    const unsigned short port)
{
  sourceAddress = ipAddress;
  sourcePort = port;
}

void pcl::HDL_Grabber::setLaserColorRGB (const pcl::RGB& color,
    unsigned int laserNumber)
{
  if (laserNumber >= HDL_MAX_NUM_LASERS)
  {
    return;
  }
  laserColors[laserNumber] = color;
}

bool pcl::HDL_Grabber::isAddressUnspecified (const boost::asio::ip::address& ip_address)
{
#if BOOST_VERSION>=104700
  return(ip_address.is_unspecified());
#else
  if (ip_address.is_v4()) {
    return(ip_address.to_v4().to_ulong() == 0);
  }
  return(false);
#endif
}

void pcl::HDL_Grabber::readPacketsFromSocket ()
{
  unsigned char data[1500];
  udp::endpoint sender_endpoint;

  while (!terminateReaderThread)
  {
    size_t length = hdlReadSocket->receive_from (
        boost::asio::buffer (data, 1500), sender_endpoint);

    if (isAddressUnspecified (sourceAddress)
        || (sourceAddress == sender_endpoint.address ()
            && sourcePort == sender_endpoint.port ()))
    {
      enqueueHDLPacket (data, length);
    }
    else
    {
      std::cout << "-";
      std::flush (std::cout);
    }
  }
}

#ifdef USE_PCAP
void pcl::HDL_Grabber::readPacketsFromPcap()
{
  struct pcap_pkthdr *header;
  const unsigned char *data;
  char errbuff[PCAP_ERRBUF_SIZE];

  pcap_t *pcap = pcap_open_offline(pcapFileName.c_str(), errbuff);

  struct bpf_program filter;
  std::ostringstream stringStream;

  stringStream << "udp and src port " << sourcePort;
  if (!isAddressUnspecified(sourceAddress))
  {
    stringStream << " and src host " << sourceAddress.to_string();
  }

  if (pcap_compile(pcap, &filter, stringStream.str().c_str(), 0, PCAP_NETMASK_UNKNOWN) == -1)
  {
    std::cout << "Issue compiling filter " << pcap_geterr(pcap) << std::endl;
  }
  else if (pcap_setfilter(pcap, &filter) == -1)
  {
    std::cout << "Issue setting filter" << pcap_geterr(pcap) << std::endl;
  }

  struct timeval lasttime;
  struct timespec delay;

  lasttime.tv_sec = 0;

  int returnValue = pcap_next_ex(pcap, &header, &data);

  while (returnValue >= 0 && !terminateReaderThread)
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
    delay.tv_sec = header->ts.tv_sec - lasttime.tv_sec;
    delay.tv_nsec = (header->ts.tv_usec - lasttime.tv_usec) * 1000;
    nanosleep(&delay, NULL);

    lasttime.tv_sec = header->ts.tv_sec;
    lasttime.tv_usec = header->ts.tv_usec;

    // The ETHERNET header is 42 bytes long; unnecessary
    enqueueHDLPacket(data + 42, header->len - 42);

    returnValue = pcap_next_ex(pcap, &header, &data);
  }
}
#endif //#ifdef USE_PCAP
