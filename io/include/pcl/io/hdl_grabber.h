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

#include "pcl/pcl_config.h"

#ifndef HDL_GRABBER_H_
#define HDL_GRABBER_H_

#include <pcl/io/grabber.h>
#include <pcl/io/synchronized_queue.h>
#include <pcl/point_types.h>
#include <boost/asio.hpp>
#include <string>

#define HDL_Grabber_toRadians(x) ((x) * M_PI / 180.0)

namespace pcl
{

  /** \brief Grabber for the Velodyne High-Definition-Laser (HDL)
   * \author Keven Ring <keven@mitre.org>
   * \ingroup io
   */
  class PCL_EXPORTS HDL_Grabber : public Grabber
  {
    public:
      /** \brief Signal used for a single sector
       *         Represents 1 corrected packet from the HDL Velodyne
       */
      typedef void (sig_cb_velodyne_hdl_scan_point_cloud_xyz) (
          const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ> >&,
          float, float);
      /** \brief Signal used for a single sector
       *         Represents 1 corrected packet from the HDL Velodyne.  Each laser has a different RGB
       */
      typedef void (sig_cb_velodyne_hdl_scan_point_cloud_xyzrgb) (
          const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGB> >&,
          float, float);
      /** \brief Signal used for a single sector
       *         Represents 1 corrected packet from the HDL Velodyne with the returned intensity.
       */
      typedef void (sig_cb_velodyne_hdl_scan_point_cloud_xyzi) (
          const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZI> >&,
          float startAngle, float);
      /** \brief Signal used for a 360 degree sweep
       *         Represents multiple corrected packets from the HDL Velodyne
       *         This signal is sent when the Velodyne passes angle "0"
       */
      typedef void (sig_cb_velodyne_hdl_sweep_point_cloud_xyz) (
          const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ> >&);
      /** \brief Signal used for a 360 degree sweep
       *         Represents multiple corrected packets from the HDL Velodyne with the returned intensity
       *         This signal is sent when the Velodyne passes angle "0"
       */
      typedef void (sig_cb_velodyne_hdl_sweep_point_cloud_xyzi) (
          const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZI> >&);
      /** \brief Signal used for a 360 degree sweep
       *         Represents multiple corrected packets from the HDL Velodyne
       *         This signal is sent when the Velodyne passes angle "0".  Each laser has a different RGB
       */
      typedef void (sig_cb_velodyne_hdl_sweep_point_cloud_xyzrgb) (
          const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGB> >&);

      /** \brief Constructor taking an optional path to an HDL corrections file.  The Grabber will listen on the default IP/port for data packets [192.168.3.255/2368]
       * \param[in] correctionsFile Path to a file which contains the correction parameters for the HDL.  This parameter is mandatory for the HDL-64, optional for the HDL-32
       * \param[in] pcapFile Path to a file which contains previously captured data packets.  This parameter is optional
       */
      HDL_Grabber (const std::string& correctionsFile = "",
          const std::string& pcapFile = "");

      /** \brief Constructor taking a pecified IP/port and an optional path to an HDL corrections file.
       * \param[in] ipAddress IP Address that should be used to listen for HDL packets
       * \param[in] port UDP Port that should be used to listen for HDL packets
       * \param[in] correctionsFile Path to a file which contains the correction parameters for the HDL.  This field is mandatory for the HDL-64, optional for the HDL-32
       */
      HDL_Grabber (const boost::asio::ip::address& ipAddress,
          const unsigned short port, const std::string& correctionsFile = "");

      /** \brief virtual Destructor inherited from the Grabber interface. It never throws. */
      virtual ~HDL_Grabber () throw ();

      /** \brief Starts processing the Velodyne packets, either from the network or PCAP file. */
      virtual void start ();

      /** \brief Stops processing the Velodyne packets, either from the network or PCAP file */
      virtual void stop ();

      /** \brief Obtains the name of this I/O Grabber
       *  \return The name of the grabber
       */
      virtual std::string getName () const;

      /** \brief Check if the grabber is still running.
       *  \return TRUE if the grabber is running, FALSE otherwise
       */
      virtual bool isRunning () const;

      /** \brief Returns the number of frames per second.
       */
      virtual float getFramesPerSecond () const;

      /** \brief Allows one to filter packets based on the SOURCE IP address and PORT
       *         This can be used, for instance, if multiple HDL LIDARs are on the same network
       */
      void filterPackets (const boost::asio::ip::address& ipAddress,
          const unsigned short port = 443);

      /** \brief Allows one to customize the colors used for each of the lasers.
       */
      void setLaserColorRGB (const pcl::RGB& color, unsigned int laserNumber);

    protected:
      static const int DATA_PORT = 2368;
      static const int HDL_NUM_ROT_ANGLES = 36000;
      static const int HDL_LASER_PER_FIRING = 32;
      static const int HDL_MAX_NUM_LASERS = 64;
      static const int HDL_FIRING_PER_PKT = 12;
      static const boost::asio::ip::address DEFAULT_NETWORK_ADDRESS;

      enum HDLBlock
      {
        BLOCK_0_TO_31 = 0xeeff, BLOCK_32_TO_63 = 0xddff
      };

#pragma pack(push, 1)
      typedef struct HDLLaserReturn
      {
          unsigned short distance;
          unsigned char intensity;
      } HDLLaserReturn;
#pragma pack(pop)

      struct HDLFiringData
      {
          unsigned short blockIdentifier;
          unsigned short rotationalPosition;
          HDLLaserReturn laserReturns[HDL_LASER_PER_FIRING];
      };

      struct HDLDataPacket
      {
          HDLFiringData firingData[HDL_FIRING_PER_PKT];
          unsigned int gpsTimestamp;
          unsigned char blank1;
          unsigned char blank2;
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

    private:
      static double *rotCosTable;
      static double *rotSinTable;
      pcl::SynchronizedQueue<unsigned char *> hdlData;
      boost::asio::ip::udp::endpoint udpEndpoint;
      boost::asio::ip::address sourceAddress;
      unsigned short sourcePort;
      boost::asio::ip::udp::socket *hdlReadSocket;
      std::string pcapFileName;
      boost::thread *consumeFromQueue;
      boost::thread *readerThread;
      HDLLaserCorrection laserCorrections[HDL_MAX_NUM_LASERS];
      bool terminateReaderThread;
      boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > currentScanXYZ,
          currentSweepXYZ;
      boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI> > currentScanXYZI,
          currentSweepXYZI;
      boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > currentScanXYZRGB,
          currentSweepXYZRGB;
      unsigned int lastAzimuth;
      boost::signals2::signal<sig_cb_velodyne_hdl_sweep_point_cloud_xyz>* sweep_xyz_signal;
      boost::signals2::signal<sig_cb_velodyne_hdl_sweep_point_cloud_xyzrgb>* sweep_xyzrgb_signal;
      boost::signals2::signal<sig_cb_velodyne_hdl_sweep_point_cloud_xyzi>* sweep_xyzi_signal;
      boost::signals2::signal<sig_cb_velodyne_hdl_scan_point_cloud_xyz>* scan_xyz_signal;
      boost::signals2::signal<sig_cb_velodyne_hdl_scan_point_cloud_xyzrgb>* scan_xyzrgb_signal;
      boost::signals2::signal<sig_cb_velodyne_hdl_scan_point_cloud_xyzi>* scan_xyzi_signal;
      pcl::RGB laserColors[HDL_MAX_NUM_LASERS];

      void processVelodynePackets ();
      void enqueueHDLPacket (const unsigned char *data,
          std::size_t bytesReceived);
      void initialize (const std::string& correctionsFile);
      void loadCorrectionsFile (const std::string& correctionsFile);
      void loadHDL32Corrections ();
      void readPacketsFromSocket ();
#ifdef USE_PCAP
      void readPacketsFromPcap();
#endif //#ifdef USE_PCAP
      void toPointClouds (HDLDataPacket *dataPacket);
      void fireCurrentSweep ();
      void fireCurrentScan (const unsigned short startAngle,
          const unsigned short endAngle);
      void computeXYZI (pcl::PointXYZI& pointXYZI, int azimuth,
          HDLLaserReturn laserReturn, HDLLaserCorrection correction);
      bool isAddressUnspecified (const boost::asio::ip::address& ip_address);
  };
}

#endif /* HDL_GRABBER_H_ */
