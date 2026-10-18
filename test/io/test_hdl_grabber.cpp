/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2026-, Open Perception
 *
 *  All rights reserved
 */

#include <pcl/test/gtest.h>
#include <pcl/common/angles.h>
#include <pcl/io/hdl_grabber.h>
#include <pcl/io/vlp_grabber.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <functional>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

struct TestableHDLGrabber : public pcl::HDLGrabber
{
  using HDLGrabber::HDLGrabber;
  using HDLGrabber::HDLLaserReturn;
  using HDLGrabber::computeXYZI;
  using HDLGrabber::laser_corrections_;

  // Join the reading threads before the members they use are destroyed
  ~TestableHDLGrabber () noexcept override
  {
    stop ();
  }

  static constexpr std::uint8_t complete_packet_marker = 0x5a;

  std::atomic<int> packets_received{0};
  std::atomic<bool> complete_packet_received{false};

  void
  toPointClouds (HDLDataPacket *dataPacket) override
  {
    ++packets_received;
    if (reinterpret_cast<const std::uint8_t *> (dataPacket)[0] == complete_packet_marker)
      complete_packet_received = true;
  }
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::string
laserCorrection (const std::string& id, double rot_correction, double vert_correction)
{
  std::ostringstream ss;
  ss << "<item class_id=\"2\" tracking_level=\"0\" version=\"1\">"
     << "<px class_id=\"3\" tracking_level=\"0\" version=\"1\">"
     << "<id_>" << id << "</id_>"
     << "<rotCorrection_>" << rot_correction << "</rotCorrection_>"
     << "<vertCorrection_>" << vert_correction << "</vertCorrection_>"
     << "<distCorrection_>0</distCorrection_>"
     << "<vertOffsetCorrection_>0</vertOffsetCorrection_>"
     << "<horizOffsetCorrection_>0</horizOffsetCorrection_>"
     << "</px></item>";
  return ss.str ();
}

void
writeCorrectionsFile (const std::string& file_name, const std::string& db)
{
  std::ofstream fs (file_name);
  fs << "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"yes\" ?>\n"
     << "<!DOCTYPE boost_serialization>\n"
     << "<boost_serialization signature=\"serialization::archive\" version=\"4\">\n"
     << "<DB class_id=\"0\" tracking_level=\"0\" version=\"0\">" << db << "</DB>\n"
     << "</boost_serialization>\n";
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (HDLGrabber, LoadCorrectionsFileLaserIdOutOfRange)
{
  // The laser ids come from the calibration file. Ids outside of the range of
  // lasers the grabber knows about must be ignored, not used as an index.
  const std::string file_name = "test_hdl_corrections_out_of_range.xml";
  writeCorrectionsFile (file_name,
                        "<points_ class_id=\"1\" tracking_level=\"0\" version=\"0\">"
                        "<count>5</count><item_version>1</item_version>"
                        + laserCorrection ("5", 2.5, 1.5)
                        + laserCorrection ("64", 3.0, 3.0)
                        + laserCorrection ("-2", 3.0, 3.0)
                        + laserCorrection ("1000000", 3.0, 3.0)
                        + laserCorrection ("-1000000", 3.0, 3.0)
                        + "</points_>");

  TestableHDLGrabber grabber (file_name, "");
  EXPECT_DOUBLE_EQ (grabber.laser_corrections_[5].azimuthCorrection, 2.5);
  EXPECT_DOUBLE_EQ (grabber.laser_corrections_[5].verticalCorrection, 1.5);

  remove (file_name.c_str ());
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (HDLGrabber, LoadCorrectionsFileWithoutPoints)
{
  // A calibration file without the list of corrections is reported, not thrown
  // out of the constructor.
  const std::string file_name = "test_hdl_corrections_without_points.xml";
  writeCorrectionsFile (file_name, "<count>0</count>");

  EXPECT_NO_THROW (TestableHDLGrabber grabber (file_name, ""));

  remove (file_name.c_str ());
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (HDLGrabber, LoadCorrectionsFileKeepsDefaultCorrections)
{
  // Lasers a calibration file has no correction for, and all of them when the
  // file can't be read, keep the default HDL-32 corrections.
  const std::string file_name = "test_hdl_corrections_one_laser.xml";
  writeCorrectionsFile (file_name,
                        "<points_ class_id=\"1\" tracking_level=\"0\" version=\"0\">"
                        "<count>1</count><item_version>1</item_version>"
                        + laserCorrection ("5", 2.5, 1.5)
                        + "</points_>");

  TestableHDLGrabber grabber (file_name, "");
  EXPECT_DOUBLE_EQ (grabber.laser_corrections_[5].verticalCorrection, 1.5);
  EXPECT_DOUBLE_EQ (grabber.laser_corrections_[0].verticalCorrection, -30.67);
  EXPECT_DOUBLE_EQ (grabber.laser_corrections_[0].azimuthCorrection, 0.0);
  EXPECT_DOUBLE_EQ (grabber.laser_corrections_[40].cosVertCorrection, 1.0);

  remove (file_name.c_str ());

  TestableHDLGrabber unreadable ("test_hdl_corrections_missing.xml", "");
  EXPECT_DOUBLE_EQ (unreadable.laser_corrections_[0].verticalCorrection, -30.67);
  EXPECT_DOUBLE_EQ (unreadable.laser_corrections_[40].cosVertCorrection, 1.0);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (HDLGrabber, ComputeXYZIAzimuthAboveLookupTable)
{
  // The azimuth of a firing comes from the packet and can be any 16-bit value.
  // Without an azimuth correction it is looked up in tables of 0 to 360 degrees:
  // a larger one must be wrapped, as when it is computed with a correction.
  TestableHDLGrabber grabber ("", "");
  const auto& correction = grabber.laser_corrections_[0];
  ASSERT_EQ (correction.azimuthCorrection, 0.0);
  const TestableHDLGrabber::HDLLaserReturn laser_return{5000, 100};  // 10 m

  const std::uint16_t azimuths[] = {36001, 45000, 65535};
  for (const auto azimuth : azimuths)
  {
    pcl::PointXYZI point, wrapped_point;
    grabber.computeXYZI (point, azimuth, laser_return, correction);
    grabber.computeXYZI (wrapped_point, static_cast<std::uint16_t> (azimuth - 36000), laser_return, correction);
    EXPECT_EQ (point.x, wrapped_point.x) << "azimuth " << azimuth;
    EXPECT_EQ (point.y, wrapped_point.y) << "azimuth " << azimuth;
    EXPECT_EQ (point.z, wrapped_point.z) << "azimuth " << azimuth;
  }
}

#ifdef HAVE_PCAP
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
appendUInt16 (std::vector<std::uint8_t>& data, std::uint16_t value)
{
  data.push_back (static_cast<std::uint8_t> (value & 0xff));
  data.push_back (static_cast<std::uint8_t> (value >> 8));
}

void
appendUInt32 (std::vector<std::uint8_t>& data, std::uint32_t value)
{
  appendUInt16 (data, static_cast<std::uint16_t> (value & 0xffff));
  appendUInt16 (data, static_cast<std::uint16_t> (value >> 16));
}

// An Ethernet frame with a UDP datagram from the sensor to port 2368 carrying the payload
std::vector<std::uint8_t>
velodyneFrame (const std::vector<std::uint8_t>& payload)
{
  constexpr std::size_t udp_header_size = 8;
  constexpr std::size_t ip_header_size = 20;

  std::vector<std::uint8_t> frame = {
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x08, 0x00};  // Ethernet, IPv4
  const auto ip_length = static_cast<std::uint16_t> (ip_header_size + udp_header_size + payload.size ());
  const std::uint8_t ip_header[] = {
    0x45, 0x00, static_cast<std::uint8_t> (ip_length >> 8), static_cast<std::uint8_t> (ip_length & 0xff),
    0x00, 0x00, 0x00, 0x00, 64, 17, 0x00, 0x00,                                           // UDP
    192, 168, 1, 201, 255, 255, 255, 255};
  frame.insert (frame.end (), ip_header, ip_header + sizeof (ip_header));
  const auto udp_length = static_cast<std::uint16_t> (udp_header_size + payload.size ());
  const std::uint8_t udp_header[] = {
    0x09, 0x40, 0x09, 0x40,                                                               // port 2368
    static_cast<std::uint8_t> (udp_length >> 8), static_cast<std::uint8_t> (udp_length & 0xff), 0x00, 0x00};
  frame.insert (frame.end (), udp_header, udp_header + sizeof (udp_header));
  frame.insert (frame.end (), payload.begin (), payload.end ());
  return frame;
}

struct PcapRecord
{
  std::vector<std::uint8_t> captured;
  std::uint32_t length;  // on the wire
};

void
writePcapFile (const std::string& file_name, const std::vector<PcapRecord>& records)
{
  std::vector<std::uint8_t> data;
  appendUInt32 (data, 0xa1b2c3d4);  // magic
  appendUInt16 (data, 2);           // version 2.4
  appendUInt16 (data, 4);
  appendUInt32 (data, 0);           // time zone
  appendUInt32 (data, 0);           // timestamp accuracy
  appendUInt32 (data, 65535);       // snapshot length
  appendUInt32 (data, 1);           // Ethernet
  for (const auto& record : records)
  {
    appendUInt32 (data, 0);         // timestamp
    appendUInt32 (data, 0);
    appendUInt32 (data, static_cast<std::uint32_t> (record.captured.size ()));  // caplen
    appendUInt32 (data, record.length);                                        // len
    data.insert (data.end (), record.captured.begin (), record.captured.end ());
  }

  std::ofstream fs (file_name, std::ios::binary);
  fs.write (reinterpret_cast<const char *> (data.data ()), data.size ());
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (HDLGrabber, ReadPacketsFromPcapTruncatedPacket)
{
  // A pcap record holds caplen bytes of a packet that was len bytes long on the
  // wire. Only the captured bytes may be handed on: a packet captured with a
  // snapshot length shorter than an HDL data packet must be dropped.
  constexpr std::size_t hdl_packet_size = 1206;
  constexpr std::size_t truncated_size = 90;

  const auto frame = velodyneFrame (std::vector<std::uint8_t> (hdl_packet_size, TestableHDLGrabber::complete_packet_marker));
  // The captured part of the truncated packet ends in zeros, never in the marker
  auto truncated_frame = velodyneFrame (std::vector<std::uint8_t> (hdl_packet_size, 0));
  truncated_frame.resize (truncated_size);

  const std::string file_name = "test_hdl_truncated_packet.pcap";
  const auto frame_size = static_cast<std::uint32_t> (frame.size ());
  // The truncated packet first, then a complete one
  writePcapFile (file_name, {{truncated_frame, frame_size}, {frame, frame_size}});

  TestableHDLGrabber grabber ("", file_name);
  grabber.start ();
  const auto deadline = std::chrono::steady_clock::now () + std::chrono::seconds (10);
  while (!grabber.complete_packet_received && std::chrono::steady_clock::now () < deadline)
    std::this_thread::sleep_for (std::chrono::milliseconds (10));
  grabber.stop ();

  EXPECT_TRUE (grabber.complete_packet_received.load ());
  EXPECT_EQ (grabber.packets_received.load (), 1);

  remove (file_name.c_str ());
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (VLPGrabber, ToPointCloudsNegativeInterpolatedAzimuth)
{
  // Lasers 16 to 31 of a VLP-16 firing fire after lasers 0 to 15, at the azimuth
  // of the firing plus half the step from firing 0 to firing 1. A firing 0
  // azimuth above 36000 makes that step negative: the azimuth must still be
  // wrapped to 0 to 360 degrees. Firing 0 is at 500 degrees, the others at 0, so
  // laser 16 of firing 1 is at (0 + 360 - 500) / 2 = -70 degrees, or 290 degrees.
  std::vector<std::uint8_t> packet;
  for (int firing = 0; firing < 12; ++firing)
  {
    appendUInt16 (packet, 0xeeff);                                           // block identifier
    appendUInt16 (packet, firing == 0 ? 50000 : 0);                          // azimuth
    for (int laser = 0; laser < 32; ++laser)
    {
      appendUInt16 (packet, (firing == 1 && laser == 16) ? 5000 : 0);       // the only return, at 10 m
      packet.push_back (100);                                                // intensity
    }
  }
  appendUInt32 (packet, 0);  // GPS timestamp
  packet.push_back (0x37);   // strongest return
  packet.push_back (0x22);   // VLP-16

  const std::string file_name = "test_vlp_negative_azimuth_step.pcap";
  const auto frame = velodyneFrame (packet);
  writePcapFile (file_name, {{frame, static_cast<std::uint32_t> (frame.size ())}});

  // The sweep holding the point ends at firing 2, whose azimuth is lower
  std::mutex sweep_mutex;
  pcl::PointCloud<pcl::PointXYZI>::ConstPtr sweep;
  std::function<pcl::VLPGrabber::sig_cb_velodyne_hdl_sweep_point_cloud_xyzi> sweep_callback =
    [&] (const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud)
    {
      std::lock_guard<std::mutex> lock (sweep_mutex);
      if (!sweep)
        sweep = cloud;
    };
  const auto sweep_received = [&]
  {
    std::lock_guard<std::mutex> lock (sweep_mutex);
    return static_cast<bool> (sweep);
  };

  pcl::VLPGrabber grabber (file_name);
  grabber.registerCallback (sweep_callback);
  grabber.start ();
  const auto deadline = std::chrono::steady_clock::now () + std::chrono::seconds (10);
  while (!sweep_received () && std::chrono::steady_clock::now () < deadline)
    std::this_thread::sleep_for (std::chrono::milliseconds (10));
  grabber.stop ();

  std::lock_guard<std::mutex> lock (sweep_mutex);
  ASSERT_TRUE (sweep);
  ASSERT_EQ (sweep->size (), 1u);
  const auto& point = (*sweep)[0];
  EXPECT_NEAR (pcl::rad2deg (std::atan2 (point.x, point.y)) + 360.0, 290.0, 0.01);

  remove (file_name.c_str ());
}
#endif // HAVE_PCAP

/* ---[ */
int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
