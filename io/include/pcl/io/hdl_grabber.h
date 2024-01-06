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

#pragma once

#include "pcl/pcl_config.h"
#include <pcl/pcl_macros.h>

#include <pcl/io/grabber.h>
#include <pcl/io/impl/synchronized_queue.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <boost/asio.hpp>
#include <string>
#include <thread>

#define HDL_Grabber_toRadians(x) ((x) * M_PI / 180.0)

namespace pcl
{

  /** \brief Grabber for the Velodyne High-Definition-Laser (HDL)
   * \author Keven Ring <keven@mitre.org>
   * \ingroup io
   */
  class PCL_EXPORTS HDLGrabber : public Grabber
  {
    public:
      /** \brief Signal used for a single sector
       *         Represents 1 corrected packet from the HDL Velodyne
       */
      using sig_cb_velodyne_hdl_scan_point_cloud_xyz = void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &, float, float);

      /** \brief Signal used for a single sector
       *         Represents 1 corrected packet from the HDL Velodyne.  Each laser has a different RGB
       */
      using sig_cb_velodyne_hdl_scan_point_cloud_xyzrgba = void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &, float, float);

      /** \brief Signal used for a single sector
       *         Represents 1 corrected packet from the HDL Velodyne with the returned intensity.
       */
      using sig_cb_velodyne_hdl_scan_point_cloud_xyzi = void (const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &, float, float);

      /** \brief Signal used for a 360 degree sweep
       *         Represents multiple corrected packets from the HDL Velodyne
       *         This signal is sent when the Velodyne passes angle "0"
       */
      using sig_cb_velodyne_hdl_sweep_point_cloud_xyz = void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &);

      /** \brief Signal used for a 360 degree sweep
       *         Represents multiple corrected packets from the HDL Velodyne with the returned intensity
       *         This signal is sent when the Velodyne passes angle "0"
       */
      using sig_cb_velodyne_hdl_sweep_point_cloud_xyzi = void (const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &);

      /** \brief Signal used for a 360 degree sweep
       *         Represents multiple corrected packets from the HDL Velodyne
       *         This signal is sent when the Velodyne passes angle "0".  Each laser has a different RGB
       */
      using sig_cb_velodyne_hdl_sweep_point_cloud_xyzrgba = void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &);

      /** \brief Constructor taking an optional path to an HDL corrections file.  The Grabber will listen on the default IP/port for data packets [192.168.3.255/2368]
       * \param[in] correctionsFile Path to a file which contains the correction parameters for the HDL.  This parameter is mandatory for the HDL-64, optional for the HDL-32
       * \param[in] pcapFile Path to a file which contains previously captured data packets.  This parameter is optional
       */
      HDLGrabber (const std::string& correctionsFile = "",
                  const std::string& pcapFile = "");

      /** \brief Constructor taking a specified IP/port and an optional path to an HDL corrections file.
       * \param[in] ipAddress IP Address that should be used to listen for HDL packets
       * \param[in] port UDP Port that should be used to listen for HDL packets
       * \param[in] correctionsFile Path to a file which contains the correction parameters for the HDL.  This field is mandatory for the HDL-64, optional for the HDL-32
       */
      HDLGrabber (const boost::asio::ip::address& ipAddress,
                  const std::uint16_t port,
                  const std::string& correctionsFile = "");

      /** \brief virtual Destructor inherited from the Grabber interface. It never throws. */
      
      ~HDLGrabber () noexcept override;

      /** \brief Starts processing the Velodyne packets, either from the network or PCAP file. */
      void
      start () override;

      /** \brief Stops processing the Velodyne packets, either from the network or PCAP file */
      void
      stop () override;

      /** \brief Obtains the name of this I/O Grabber
       *  \return The name of the grabber
       */
      std::string
      getName () const override;

      /** \brief Check if the grabber is still running.
       *  \return TRUE if the grabber is running, FALSE otherwise
       */
      bool
      isRunning () const override;

      /** \brief Returns the number of frames per second.
       */
      float
      getFramesPerSecond () const override;

      /** \brief Allows one to filter packets based on the SOURCE IP address and PORT
       *         This can be used, for instance, if multiple HDL LIDARs are on the same network
       */
      void
      filterPackets (const boost::asio::ip::address& ipAddress,
                     const std::uint16_t port = 443);

      /** \brief Allows one to customize the colors used by each laser.
       * \param[in] color RGB color to set
       * \param[in] laserNumber Number of laser to set color
       */
      void
      setLaserColorRGB (const pcl::RGB& color,
                        const std::uint8_t laserNumber);

      /** \brief Allows one to customize the colors used for each of the lasers.
       * \param[in] begin begin iterator of RGB color array
       * \param[in] end end iterator of RGB color array
       */
      template<typename IterT> void
      setLaserColorRGB (const IterT& begin, const IterT& end)
      {
          std::copy (begin, end, laser_rgb_mapping_);
      }

      /** \brief Any returns from the HDL with a distance less than this are discarded.
       *         This value is in meters
       *         Default: 0.0
       */
      void
      setMinimumDistanceThreshold (float & minThreshold);

      /** \brief Any returns from the HDL with a distance greater than this are discarded.
       *         This value is in meters
       *         Default: 10000.0
       */
      void
      setMaximumDistanceThreshold (float & maxThreshold);

      /** \brief Returns the current minimum distance threshold, in meters
       */

      float
      getMinimumDistanceThreshold ();

      /** \brief Returns the current maximum distance threshold, in meters
       */
      float
      getMaximumDistanceThreshold ();

      /** \brief Returns the maximum number of lasers
      */
      virtual std::uint8_t
      getMaximumNumberOfLasers () const;

    protected:
      static const std::uint16_t HDL_DATA_PORT = 2368;
      static const std::uint16_t HDL_NUM_ROT_ANGLES = 36001;
      static const std::uint8_t HDL_LASER_PER_FIRING = 32;
      static const std::uint8_t HDL_MAX_NUM_LASERS = 64;
      static const std::uint8_t HDL_FIRING_PER_PKT = 12;

      enum HDLBlock
      {
        BLOCK_0_TO_31 = 0xeeff, BLOCK_32_TO_63 = 0xddff
      };

#pragma pack(push, 1)
      struct HDLLaserReturn
      {
          std::uint16_t distance;
          std::uint8_t intensity;
      };
#pragma pack(pop)

      struct HDLFiringData
      {
          std::uint16_t blockIdentifier;
          std::uint16_t rotationalPosition;
          HDLLaserReturn laserReturns[HDL_LASER_PER_FIRING];
      };

      struct HDLDataPacket
      {
          HDLFiringData firingData[HDL_FIRING_PER_PKT];
          std::uint32_t gpsTimestamp;
          std::uint8_t mode;
          std::uint8_t sensorType;
      };

      struct HDLLaserCorrection
      {
          double azimuthCorrection;
          double verticalCorrection;
          double distanceCorrection;
          double verticalOffsetCorrection;
          double horizontalOffsetCorrection;
          double sinVertCorrection;
          double cosVertCorrection;
          double sinVertOffsetCorrection;
          double cosVertOffsetCorrection;
      };

      HDLLaserCorrection laser_corrections_[HDL_MAX_NUM_LASERS];
      std::uint16_t last_azimuth_;
      pcl::PointCloud<pcl::PointXYZ>::Ptr current_scan_xyz_, current_sweep_xyz_;
      pcl::PointCloud<pcl::PointXYZI>::Ptr current_scan_xyzi_, current_sweep_xyzi_;
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr current_scan_xyzrgba_, current_sweep_xyzrgba_;
      boost::signals2::signal<sig_cb_velodyne_hdl_sweep_point_cloud_xyz>* sweep_xyz_signal_;
      boost::signals2::signal<sig_cb_velodyne_hdl_sweep_point_cloud_xyzrgba>* sweep_xyzrgba_signal_;
      boost::signals2::signal<sig_cb_velodyne_hdl_sweep_point_cloud_xyzi>* sweep_xyzi_signal_;
      boost::signals2::signal<sig_cb_velodyne_hdl_scan_point_cloud_xyz>* scan_xyz_signal_;
      boost::signals2::signal<sig_cb_velodyne_hdl_scan_point_cloud_xyzrgba>* scan_xyzrgba_signal_;
      boost::signals2::signal<sig_cb_velodyne_hdl_scan_point_cloud_xyzi>* scan_xyzi_signal_;

      void
      fireCurrentSweep ();

      void
      fireCurrentScan (const std::uint16_t startAngle,
                       const std::uint16_t endAngle);
      void
      computeXYZI (pcl::PointXYZI& pointXYZI,
                   std::uint16_t azimuth,
                   HDLLaserReturn laserReturn,
                   HDLLaserCorrection correction) const;


    private:
      static double *cos_lookup_table_;
      static double *sin_lookup_table_;
      pcl::SynchronizedQueue<std::uint8_t *> hdl_data_;
      boost::asio::ip::udp::endpoint udp_listener_endpoint_;
      boost::asio::ip::address source_address_filter_;
      std::uint16_t source_port_filter_;
      boost::asio::io_service hdl_read_socket_service_;
      boost::asio::ip::udp::socket *hdl_read_socket_;
      std::string pcap_file_name_;
      std::thread *queue_consumer_thread_;
      std::thread *hdl_read_packet_thread_;
      bool terminate_read_packet_thread_;
      pcl::RGB laser_rgb_mapping_[HDL_MAX_NUM_LASERS];
      float min_distance_threshold_;
      float max_distance_threshold_;

      virtual void
      toPointClouds (HDLDataPacket *dataPacket);

      virtual boost::asio::ip::address
      getDefaultNetworkAddress ();

      void
      initialize (const std::string& correctionsFile = "");

      void
      processVelodynePackets ();

      void
      enqueueHDLPacket (const std::uint8_t *data,
                        std::size_t bytesReceived);

      void
      loadCorrectionsFile (const std::string& correctionsFile);

      void
      loadHDL32Corrections ();

      void
      readPacketsFromSocket ();

#ifdef HAVE_PCAP
      void
      readPacketsFromPcap();

#endif //#ifdef HAVE_PCAP

      bool
      isAddressUnspecified (const boost::asio::ip::address& ip_address);

  };
}
