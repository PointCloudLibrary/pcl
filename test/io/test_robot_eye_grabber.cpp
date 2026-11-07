/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2026-, Open Perception
 *
 *  All rights reserved
 */

#include <pcl/test/gtest.h>
#include <pcl/point_types.h>
#include <pcl/io/robot_eye_grabber.h>

#include <boost/asio.hpp>

#include <chrono>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

using CloudT = pcl::PointCloud<pcl::PointXYZI>;

// Packets shorter than the "EBRBEP" header plus its timestamp must be dropped
// without reading past the received data, and must not disturb later packets.
TEST (RobotEyeGrabber, ShortPackets)
{
  boost::asio::io_context io;
  boost::asio::ip::udp::socket sender (io, boost::asio::ip::udp::endpoint (boost::asio::ip::udp::v4 (), 0));

  // Grab a free port for the grabber.
  unsigned short port;
  {
    boost::asio::ip::udp::socket probe (io, boost::asio::ip::udp::endpoint (boost::asio::ip::make_address ("127.0.0.1"), 0));
    port = probe.local_endpoint ().port ();
  }

  pcl::RobotEyeGrabber grabber (boost::asio::ip::address_v4::any (), port);
  grabber.setSignalPointCloudSize (0);

  std::mutex mutex;
  std::condition_variable cv;
  std::vector<CloudT> clouds;
  std::function<pcl::RobotEyeGrabber::sig_cb_robot_eye_point_cloud_xyzi> callback =
    [&] (const CloudT::ConstPtr& cloud)
    {
      std::lock_guard<std::mutex> lock (mutex);
      clouds.push_back (*cloud);
      cv.notify_one ();
    };
  grabber.registerCallback (callback);
  grabber.start ();
  ASSERT_TRUE (grabber.isRunning ());

  const boost::asio::ip::udp::endpoint destination (boost::asio::ip::make_address ("127.0.0.1"), port);
  auto send = [&] (const std::string& packet)
  {
    sender.send_to (boost::asio::buffer (packet.data (), packet.size ()), destination);
  };

  send ("");
  send ("E");
  send ("EBRBEP");
  send (std::string ("EBRBEP\x00\x00\x00", 9));
  // Header, timestamp and one point: azimuth 0, elevation 0, range 1.00, intensity 7.
  send (std::string ("EBRBEP\x00\x00\x00\x01" "\x00\x00\x00\x00\x00\x64\x00\x07", 18));

  {
    std::unique_lock<std::mutex> lock (mutex);
    cv.wait_for (lock, std::chrono::seconds (5), [&] { return !clouds.empty (); });
  }
  grabber.stop ();

  ASSERT_EQ (1u, clouds.size ());
  ASSERT_EQ (1u, clouds[0].size ());
  EXPECT_NEAR (0.0f, clouds[0][0].x, 1e-4);
  EXPECT_NEAR (1.0f, clouds[0][0].y, 1e-4);
  EXPECT_NEAR (0.0f, clouds[0][0].z, 1e-4);
  EXPECT_EQ (7.0f, clouds[0][0].intensity);
}

/* ---[ */
int
main (int argc, char** argv)
{
  ::testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
