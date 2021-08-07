/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2015-, Open Perception, Inc.
 *  Copyright (c) 2015 The MITRE Corporation
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

#include <pcl/io/hdl_grabber.h>
#include <pcl/io/grabber.h>
#include <pcl/point_types.h>
#include <boost/asio.hpp>
#include <string>

namespace pcl
{

  /** \brief Grabber for the Velodyne LiDAR (VLP), based on the Velodyne High Definition Laser (HDL)
   * \author Keven Ring <keven@mitre.org>
   * \ingroup io
   */
  class PCL_EXPORTS VLPGrabber : public HDLGrabber
  {
    public:
      /** \brief Constructor taking an optional path to an vlp corrections file.  The Grabber will listen on the default IP/port for data packets [192.168.3.255/2368]
       * \param[in] pcapFile Path to a file which contains previously captured data packets.  This parameter is optional
       */
      VLPGrabber (const std::string& pcapFile = "");

      /** \brief Constructor taking a specified IP/port
       * \param[in] ipAddress IP Address that should be used to listen for VLP packets
       * \param[in] port UDP Port that should be used to listen for VLP packets
       */
      VLPGrabber (const boost::asio::ip::address& ipAddress,
                  const std::uint16_t port);

      /** \brief virtual Destructor inherited from the Grabber interface. It never throws. */
      
      ~VLPGrabber () noexcept;

      /** \brief Obtains the name of this I/O Grabber
       *  \return The name of the grabber
       */
      std::string
      getName () const override;

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

      /** \brief Returns the maximum number of lasers
      */
      std::uint8_t
      getMaximumNumberOfLasers () const override;

    protected:
      static const std::uint8_t VLP_MAX_NUM_LASERS = 16;
      static const std::uint8_t VLP_DUAL_MODE = 0x39;

    private:
      pcl::RGB laser_rgb_mapping_[VLP_MAX_NUM_LASERS];

      void
      toPointClouds (HDLDataPacket *dataPacket) override;

      boost::asio::ip::address
      getDefaultNetworkAddress () override;

      void
      initializeLaserMapping ();

      void
      loadVLP16Corrections ();

  };
}
