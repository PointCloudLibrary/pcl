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

#include "pcl/pcl_config.h"

#ifndef PCL_IO_HDL_GRABBER_H_
#define PCL_IO_HDL_GRABBER_H_

#include <pcl/io/grabber.h>
#include <pcl/io/impl/synchronized_queue.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <boost/asio.hpp>
#include <string>

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
      typedef void
      (sig_cb_velodyne_hdl_scan_point_cloud_xyz) (const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ> >&,
                                                  float,
                                                  float);
      /** \brief Signal used for a single sector
       *         Represents 1 corrected packet from the HDL Velodyne.  Each laser has a different RGB
       */
      typedef void
      (sig_cb_velodyne_hdl_scan_point_cloud_xyzrgb) (const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGBA> >&,
                                                     float,
                                                     float);
      /** \brief Signal used for a single sector
       *         Represents 1 corrected packet from the HDL Velodyne with the returned intensity.
       */
      typedef void
      (sig_cb_velodyne_hdl_scan_point_cloud_xyzi) (const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZI> >&,
                                                   float startAngle,
                                                   float);
      /** \brief Signal used for a 360 degree sweep
       *         Represents multiple corrected packets from the HDL Velodyne
       *         This signal is sent when the Velodyne passes angle "0"
       */
      typedef void
      (sig_cb_velodyne_hdl_sweep_point_cloud_xyz) (const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ> >&);
      /** \brief Signal used for a 360 degree sweep
       *         Represents multiple corrected packets from the HDL Velodyne with the returned intensity
       *         This signal is sent when the Velodyne passes angle "0"
       */
      typedef void
      (sig_cb_velodyne_hdl_sweep_point_cloud_xyzi) (const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZI> >&);
      /** \brief Signal used for a 360 degree sweep
       *         Represents multiple corrected packets from the HDL Velodyne
       *         This signal is sent when the Velodyne passes angle "0".  Each laser has a different RGB
       */
      typedef void
      (sig_cb_velodyne_hdl_sweep_point_cloud_xyzrgb) (const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGBA> >&);

      /** \brief Constructor taking an optional path to an HDL corrections file.  The Grabber will listen on the default IP/port for data packets [192.168.3.255/2368]
       * \param[in] correctionsFile Path to a file which contains the correction parameters for the HDL.  This parameter is mandatory for the HDL-64, optional for the HDL-32
       * \param[in] pcapFile Path to a file which contains previously captured data packets.  This parameter is optional
       */
      HDLGrabber (const std::string& correctionsFile = "",
                  const std::string& pcapFile = "");

      /** \brief Constructor taking a pecified IP/port and an optional path to an HDL corrections file.
       * \param[in] ipAddress IP Address that should be used to listen for HDL packets
       * \param[in] port UDP Port that should be used to listen for HDL packets
       * \param[in] correctionsFile Path to a file which contains the correction parameters for the HDL.  This field is mandatory for the HDL-64, optional for the HDL-32
       */
      HDLGrabber (const boost::asio::ip::address& ipAddress,
                  const unsigned short port,
                  const std::string& correctionsFile = "");

      /** \brief virtual Destructor inherited from the Grabber interface. It never throws. */
      virtual
      ~HDLGrabber () throw ();

      /** \brief Starts processing the Velodyne packets, either from the network or PCAP file. */
      virtual void
      start ();

      /** \brief Stops processing the Velodyne packets, either from the network or PCAP file */
      virtual void
      stop ();

      /** \brief Obtains the name of this I/O Grabber
       *  \return The name of the grabber
       */
      virtual std::string
      getName () const;

      /** \brief Check if the grabber is still running.
       *  \return TRUE if the grabber is running, FALSE otherwise
       */
      virtual bool
      isRunning () const;

      /** \brief Returns the number of frames per second.
       */
      virtual float
      getFramesPerSecond () const;

      /** \brief Allows one to filter packets based on the SOURCE IP address and PORT
       *         This can be used, for instance, if multiple HDL LIDARs are on the same network
       */
      void
      filterPackets (const boost::asio::ip::address& ipAddress,
                     const unsigned short port = 443);

      /** \brief Allows one to customize the colors used for each of the lasers.
       */
      void
      setLaserColorRGB (const pcl::RGB& color,
                        unsigned int laserNumber);

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

    protected:
      static const int HDL_DATA_PORT = 2368;
      static const int HDL_NUM_ROT_ANGLES = 36001;
      static const int HDL_LASER_PER_FIRING = 32;
      static const int HDL_MAX_NUM_LASERS = 64;
      static const int HDL_FIRING_PER_PKT = 12;

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
          unsigned char mode;
          unsigned char sensorType;
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

      // Generated by script at Perception/lib/HAL/resources/hdl32-timestamp-offset-generator.py
      int16_t hdl_timing_offsets_[HDL_FIRING_PER_PKT][HDL_LASER_PER_FIRING] = {
          { -543, -541, -540, -539, -538, -537, -536, -535, -533, -532, -531, -530, -529, -528, -526, -525, -524, -523, -522, -521, -520, -518, -517, -516, -515, -514, -513, -511, -510, -509, -508, -507, },
          { -497, -495, -494, -493, -492, -491, -490, -488, -487, -486, -485, -484, -483, -482, -480, -479, -478, -477, -476, -475, -473, -472, -471, -470, -469, -468, -467, -465, -464, -463, -462, -461, },
          { -450, -449, -448, -447, -446, -445, -444, -442, -441, -440, -439, -438, -437, -435, -434, -433, -432, -431, -430, -429, -427, -426, -425, -424, -423, -422, -420, -419, -418, -417, -416, -415, },
          { -404, -403, -402, -401, -400, -399, -397, -396, -395, -394, -393, -392, -391, -389, -388, -387, -386, -385, -384, -382, -381, -380, -379, -378, -377, -376, -374, -373, -372, -371, -370, -369, },
          { -358, -357, -356, -355, -354, -353, -351, -350, -349, -348, -347, -346, -344, -343, -342, -341, -340, -339, -338, -336, -335, -334, -333, -332, -331, -329, -328, -327, -326, -325, -324, -323, },
          { -312, -311, -310, -309, -308, -306, -305, -304, -303, -302, -301, -300, -298, -297, -296, -295, -294, -293, -291, -290, -289, -288, -287, -286, -285, -283, -282, -281, -280, -279, -278, -276, },
          { -266, -265, -264, -263, -262, -260, -259, -258, -257, -256, -255, -253, -252, -251, -250, -249, -248, -247, -245, -244, -243, -242, -241, -240, -238, -237, -236, -235, -234, -233, -232, -230, },
          { -220, -219, -218, -217, -215, -214, -213, -212, -211, -210, -209, -207, -206, -205, -204, -203, -202, -200, -199, -198, -197, -196, -195, -194, -192, -191, -190, -189, -188, -187, -185, -184, },
          { -174, -173, -172, -170, -169, -168, -167, -166, -165, -164, -162, -161, -160, -159, -158, -157, -156, -154, -153, -152, -151, -150, -149, -147, -146, -145, -144, -143, -142, -141, -139, -138, },
          { -128, -127, -126, -124, -123, -122, -121, -120, -119, -118, -116, -115, -114, -113, -112, -111, -109, -108, -107, -106, -105, -104, -103, -101, -100, -99, -98, -97, -96, -94, -93, -92, },
          { -82, -81, -79, -78, -77, -76, -75, -74, -73, -71, -70, -69, -68, -67, -66, -65, -63, -62, -61, -60, -59, -58, -56, -55, -54, -53, -52, -51, -50, -48, -47, -46, },
          { -36, -35, -33, -32, -31, -30, -29, -28, -26, -25, -24, -23, -22, -21, -20, -18, -17, -16, -15, -14, -13, -12, -10, -9, -8, -7, -6, -5, -3, -2, -1, 0 },
      };

      HDLLaserCorrection laser_corrections_[HDL_MAX_NUM_LASERS];
      unsigned int last_azimuth_;
      boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > current_scan_xyz_, current_sweep_xyz_;
      boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI> > current_scan_xyzi_, current_sweep_xyzi_;
      boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA> > current_scan_xyzrgb_, current_sweep_xyzrgb_;
      boost::signals2::signal<sig_cb_velodyne_hdl_sweep_point_cloud_xyz>* sweep_xyz_signal_;
      boost::signals2::signal<sig_cb_velodyne_hdl_sweep_point_cloud_xyzrgb>* sweep_xyzrgb_signal_;
      boost::signals2::signal<sig_cb_velodyne_hdl_sweep_point_cloud_xyzi>* sweep_xyzi_signal_;
      boost::signals2::signal<sig_cb_velodyne_hdl_scan_point_cloud_xyz>* scan_xyz_signal_;
      boost::signals2::signal<sig_cb_velodyne_hdl_scan_point_cloud_xyzrgb>* scan_xyzrgb_signal_;
      boost::signals2::signal<sig_cb_velodyne_hdl_scan_point_cloud_xyzi>* scan_xyzi_signal_;

      void
      fireCurrentSweep ();

      void
      fireCurrentScan (const unsigned short startAngle,
                       const unsigned short endAngle);
      void
      computeXYZI (pcl::PointXYZI& pointXYZI,
                   int azimuth,
                   HDLLaserReturn laserReturn,
                   HDLLaserCorrection correction);


    private:
      static double *cos_lookup_table_;
      static double *sin_lookup_table_;
      pcl::SynchronizedQueue<unsigned char *> hdl_data_;
      boost::asio::ip::udp::endpoint udp_listener_endpoint_;
      boost::asio::ip::address source_address_filter_;
      unsigned short source_port_filter_;
      boost::asio::io_service hdl_read_socket_service_;
      boost::asio::ip::udp::socket *hdl_read_socket_;
      std::string pcap_file_name_;
      boost::thread *queue_consumer_thread_;
      boost::thread *hdl_read_packet_thread_;
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
      enqueueHDLPacket (const unsigned char *data,
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

#endif /* PCL_IO_HDL_GRABBER_H_ */
