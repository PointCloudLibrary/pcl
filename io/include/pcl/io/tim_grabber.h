/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2020-, Open Perception
 *  Copyright (c) 2020, ysuzuki19
 *
 *  All rights reserved
 */

#pragma once

#include <pcl/pcl_exports.h>
#include <pcl/console/print.h>
#include <pcl/common/time.h>
#include <pcl/io/grabber.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <vector>
#include <string>
#include <thread>

namespace pcl
{

//// note: Protcol named CoLaA (used by SICK) has some information.
////       In this Grabber, only the amount_of_data is used, so other information is truncated.
////       Details of the protocol can be found at the following URL.

//// pp.87~89 (table)

//// https://cdn.sickcn.com/media/docs/7/27/927/technical_information_telegram_listing_ranging_sensors_lms1xx_lms5xx_tim2xx_tim5xx_tim7xx_lms1000_mrs1000_mrs6000_nav310_ld_oem15xx_ld_lrs36xx_lms4000_en_im0045927.pdf


//// By this PDF, the header contains the following information in order

/////////////////////////////////////////////////////
//// command type
//// command
//// version number
//// device number
//// serial number (2 value)
//// device status
//// Telegram counter
//// Scan counter
//// Time since start up
//// Time of transmission
//// Status of digital inputs (2 value)
//// Status of digital outputs (2 value)
//// Reserved
//// scan frequency
//// measurement frequency
//// Amount of encoder
//// Amount of 16 bit channels
//// Content
//// Scale factor according to IEEE754
//// Scale factor offset according to IEEE754
//// Start angle
//// Size of single angular step
//// Amount of data
//// distance_1
//// distance_2
//// distance_3
//// ...
//// distance_n
/////////////////////////////////////////////////////


class PCL_EXPORTS TimGrabber : public Grabber
{
  public:
    using sig_cb_sick_tim_scan_point_cloud_xyz = void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&);

    TimGrabber ();
    TimGrabber (const boost::asio::ip::address& ipAddress, const std::uint16_t port);
    ~TimGrabber () noexcept;

    void
    start () override;

    void
    stop () override;

    std::string
    getName () const override;

    bool
    isRunning () const override;

  protected:
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_xyz_ptr_;
    boost::signals2::signal<sig_cb_sick_tim_scan_point_cloud_xyz>* point_cloud_xyz_signal_;

    void
    publishSignal ();

    //// parse received packet
    //// used by GTEST
    void
    processTimPacket (std::string const& packet);

    //// check size of lookup tables
    //// rebuild if lookup tables have different size
    void
    updateLookupTables ();

    //// convert std::vector (distance) to pcl::PointCloud
    //// used by GTEST
    void
    toPointClouds ();

  private:
    constexpr static float angle_start_ = - 1.0 * M_PI / 4.0;
    constexpr static float angle_range_ = 2.0 * M_PI * 3.0 / 4.0;

    //// lookup tables for calculaing 2d-coordinate
    //// reset lookup tables if amount of received data is different
    std::vector<float> cos_dynamic_lookup_table_;
    std::vector<float> sin_dynamic_lookup_table_;

    std::array<char, 4000> received_packet_;
    std::size_t length_;
    std::istringstream iss_;

    std::size_t amount_of_data_ = 811;
    std::vector<float> distances_;

    boost::asio::ip::tcp::endpoint tcp_endpoint_;
    boost::asio::io_service tim_io_service_;
    boost::asio::ip::tcp::socket tim_socket_;
    //// wait time for receiving data (on the order of milliseconds)
    unsigned int wait_time_milliseconds_ = 0;

    pcl::EventFrequency frequency_;
    mutable boost::mutex frequency_mutex_;

    std::thread grabber_thread_;
    bool is_running_ = false;

    void
    initialize ();

    float
    getFramesPerSecond () const override;

    void
    buildLookupTables ();

    //// check received packet is valid
    bool
    isValidPacket () const;

    //// receive packet (named CoLaA; SICK sensors Communication Language)
    void
    receiveTimPacket ();

    void
    parsePacketHeader (std::string const& header);
    void
    parsePacketBody (std::string const& body);

    void
    processGrabbing ();
};
}
