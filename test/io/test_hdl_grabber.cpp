/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2026-, Open Perception
 *
 *  All rights reserved
 */

#include <pcl/test/gtest.h>
#include <pcl/io/hdl_grabber.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

struct TestableHDLGrabber : public pcl::HDLGrabber
{
  using HDLGrabber::HDLGrabber;
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

TEST (HDLGrabber, ReadPacketsFromPcapTruncatedPacket)
{
  // A pcap record holds caplen bytes of a packet that was len bytes long on the
  // wire. Only the captured bytes may be handed on: a packet captured with a
  // snapshot length shorter than an HDL data packet must be dropped.
  constexpr std::uint32_t hdl_packet_size = 1206;
  constexpr std::uint32_t udp_header_size = 8;
  constexpr std::uint32_t ip_header_size = 20;
  constexpr std::uint32_t frame_size = 14 + ip_header_size + udp_header_size + hdl_packet_size;
  constexpr std::uint32_t truncated_size = 90;

  std::vector<std::uint8_t> frame = {
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x08, 0x00};  // Ethernet, IPv4
  const std::uint16_t ip_length = ip_header_size + udp_header_size + hdl_packet_size;
  const std::uint8_t ip_header[] = {
    0x45, 0x00, static_cast<std::uint8_t> (ip_length >> 8), static_cast<std::uint8_t> (ip_length & 0xff),
    0x00, 0x00, 0x00, 0x00, 64, 17, 0x00, 0x00,                                           // UDP
    192, 168, 1, 201, 255, 255, 255, 255};
  frame.insert (frame.end (), ip_header, ip_header + sizeof (ip_header));
  const std::uint16_t udp_length = udp_header_size + hdl_packet_size;
  const std::uint8_t udp_header[] = {
    0x09, 0x40, 0x09, 0x40,                                                               // port 2368
    static_cast<std::uint8_t> (udp_length >> 8), static_cast<std::uint8_t> (udp_length & 0xff), 0x00, 0x00};
  frame.insert (frame.end (), udp_header, udp_header + sizeof (udp_header));
  frame.resize (frame_size, TestableHDLGrabber::complete_packet_marker);
  // The captured part of the truncated packet ends in zeros, never in the marker
  std::vector<std::uint8_t> truncated_frame (frame.begin (), frame.begin () + truncated_size);
  std::fill (truncated_frame.begin () + 14 + ip_header_size + udp_header_size, truncated_frame.end (), 0);

  std::vector<std::uint8_t> data;
  appendUInt32 (data, 0xa1b2c3d4);  // magic
  appendUInt16 (data, 2);           // version 2.4
  appendUInt16 (data, 4);
  appendUInt32 (data, 0);           // time zone
  appendUInt32 (data, 0);           // timestamp accuracy
  appendUInt32 (data, 65535);       // snapshot length
  appendUInt32 (data, 1);           // Ethernet
  // The truncated packet first, then a complete one
  for (const auto& captured : {truncated_frame, frame})
  {
    appendUInt32 (data, 0);         // timestamp
    appendUInt32 (data, 0);
    appendUInt32 (data, static_cast<std::uint32_t> (captured.size ()));  // caplen
    appendUInt32 (data, frame_size);                                    // len
    data.insert (data.end (), captured.begin (), captured.end ());
  }

  const std::string file_name = "test_hdl_truncated_packet.pcap";
  std::ofstream fs (file_name, std::ios::binary);
  fs.write (reinterpret_cast<const char *> (data.data ()), data.size ());
  fs.close ();

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
#endif // HAVE_PCAP

/* ---[ */
int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
