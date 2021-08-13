/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012-, Open Perception, Inc.
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

#include <pcl/io/grabber.h>
#include <pcl/io/impl/synchronized_queue.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/memory.h>
#include <boost/asio.hpp>
#include <boost/shared_array.hpp> // for shared_array

#include <memory>
#include <thread>

namespace pcl
{

  /** \brief Grabber for the Ocular Robotics RobotEye sensor.
   * \ingroup io
   */
  class PCL_EXPORTS RobotEyeGrabber : public Grabber
  {
    public:

      /** \brief Signal used for the point cloud callback.
       * This signal is sent when the accumulated number of points reaches
       * the limit specified by setSignalPointCloudSize().
       */
      using sig_cb_robot_eye_point_cloud_xyzi = void (const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &);

      /** \brief RobotEyeGrabber default constructor. */
      RobotEyeGrabber ();

      /** \brief RobotEyeGrabber constructor taking a specified IP address and data port. */
      RobotEyeGrabber (const boost::asio::ip::address& ipAddress, unsigned short port=443);

      /** \brief virtual Destructor inherited from the Grabber interface. It never throws. */
      ~RobotEyeGrabber () noexcept;

      /** \brief Starts the RobotEye grabber.
       * The grabber runs on a separate thread, this call will return without blocking. */
      void start () override;

      /** \brief Stops the RobotEye grabber. */
      void stop () override;

      /** \brief Obtains the name of this I/O Grabber
       *  \return The name of the grabber
       */
      std::string getName () const override;

      /** \brief Check if the grabber is still running.
       *  \return TRUE if the grabber is running, FALSE otherwise
       */
      bool isRunning () const override;

      /** \brief Returns the number of frames per second.
       */
      float getFramesPerSecond () const override;

      /** \brief Set/get ip address of the sensor that sends the data.
       * The default is address_v4::any ().
       */
      void setSensorAddress (const boost::asio::ip::address& ipAddress);
      const boost::asio::ip::address& getSensorAddress () const;

      /** \brief Set/get the port number which receives data from the sensor.
       * The default is 443.
       */
      void setDataPort (unsigned short port);
      unsigned short getDataPort () const;

      /** \brief Set/get the number of points to accumulate before the grabber
       * callback is signaled.  The default is 1000.
       */
      void setSignalPointCloudSize (std::size_t numerOfPoints);
      std::size_t getSignalPointCloudSize () const;

      /** \brief Returns the point cloud with point accumulated by the grabber.
       * It is not safe to access this point cloud except if the grabber is
       * stopped or during the grabber callback.
       */
      pcl::PointCloud<pcl::PointXYZI>::Ptr getPointCloud() const;

    private:

      bool terminate_thread_;
      std::size_t signal_point_cloud_size_;
      unsigned short data_port_;
      enum { MAX_LENGTH = 65535 };
      unsigned char receive_buffer_[MAX_LENGTH];
      unsigned int data_size_;

      boost::asio::ip::address sensor_address_;
      boost::asio::ip::udp::endpoint sender_endpoint_;
      boost::asio::io_service io_service_;
      std::shared_ptr<boost::asio::ip::udp::socket> socket_;
      std::shared_ptr<std::thread> socket_thread_;
      std::shared_ptr<std::thread> consumer_thread_;

      pcl::SynchronizedQueue<boost::shared_array<unsigned char> > packet_queue_;
      pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_xyzi_;
      boost::signals2::signal<sig_cb_robot_eye_point_cloud_xyzi>* point_cloud_signal_;

      void consumerThreadLoop ();
      void socketThreadLoop ();
      void asyncSocketReceive ();
      void resetPointCloud ();
      void socketCallback (const boost::system::error_code& error, std::size_t number_of_bytes);
      void convertPacketData (unsigned char *data_packet, std::size_t length);
      void computeXYZI (pcl::PointXYZI& point_XYZI, unsigned char* point_data);
      void computeTimestamp (std::uint32_t& timestamp, unsigned char* point_data);
  };
}
