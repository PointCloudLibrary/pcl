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

#include <pcl/io/vlp_grabber.h>

using boost::asio::ip::udp;

/////////////////////////////////////////////////////////////////////////////
pcl::VLPGrabber::VLPGrabber (const std::string& pcapFile) :
    HDLGrabber ("", pcapFile)
{
  initializeLaserMapping ();
  loadVLP16Corrections ();
}

/////////////////////////////////////////////////////////////////////////////
pcl::VLPGrabber::VLPGrabber (const boost::asio::ip::address& ipAddress,
                             const std::uint16_t port) :
    HDLGrabber (ipAddress, port)
{
  initializeLaserMapping ();
  loadVLP16Corrections ();
}

/////////////////////////////////////////////////////////////////////////////
pcl::VLPGrabber::~VLPGrabber () noexcept = default;

/////////////////////////////////////////////////////////////////////////////
void
pcl::VLPGrabber::initializeLaserMapping ()
{
  for (std::uint8_t i = 0; i < VLP_MAX_NUM_LASERS / 2u; i++)
  {
     laser_rgb_mapping_[i * 2].b = static_cast<std::uint8_t> (i * 6 + 64);
     laser_rgb_mapping_[i * 2 + 1].b = static_cast<std::uint8_t> ((i + 16) * 6 + 64);
  }
}

/////////////////////////////////////////////////////////////////////////////
void
pcl::VLPGrabber::loadVLP16Corrections ()
{
  double vlp16_vertical_corrections[] = { -15, 1, -13, 3, -11, 5, -9, 7, -7, 9, -5, 11, -3, 13, -1, 15 };
  for (std::uint8_t i = 0; i < VLP_MAX_NUM_LASERS; i++)
  {
    HDLGrabber::laser_corrections_[i].azimuthCorrection = 0.0;
    HDLGrabber::laser_corrections_[i].distanceCorrection = 0.0;
    HDLGrabber::laser_corrections_[i].horizontalOffsetCorrection = 0.0;
    HDLGrabber::laser_corrections_[i].verticalOffsetCorrection = 0.0;
    HDLGrabber::laser_corrections_[i].verticalCorrection = vlp16_vertical_corrections[i];
    HDLGrabber::laser_corrections_[i].sinVertCorrection = std::sin (HDL_Grabber_toRadians(vlp16_vertical_corrections[i]));
    HDLGrabber::laser_corrections_[i].cosVertCorrection = std::cos (HDL_Grabber_toRadians(vlp16_vertical_corrections[i]));
  }
}

/////////////////////////////////////////////////////////////////////////////
boost::asio::ip::address
pcl::VLPGrabber::getDefaultNetworkAddress ()
{
  return (boost::asio::ip::address::from_string ("255.255.255.255"));
}

/////////////////////////////////////////////////////////////////////////////
void
pcl::VLPGrabber::toPointClouds (HDLDataPacket *dataPacket)
{
  static std::uint32_t sweep_counter = 0;
  if (sizeof(HDLLaserReturn) != 3)
    return;

  time_t system_time;
  time (&system_time);
  time_t velodyne_time = (system_time & 0x00000000ffffffffl) << 32 | dataPacket->gpsTimestamp;

  double interpolated_azimuth_delta;

  std::uint8_t index = 1;
  if (dataPacket->mode == VLP_DUAL_MODE)
  {
    index = 2;
  }
  if (dataPacket->firingData[index].rotationalPosition < dataPacket->firingData[0].rotationalPosition)
  {
    interpolated_azimuth_delta = ((dataPacket->firingData[index].rotationalPosition + 36000) - dataPacket->firingData[0].rotationalPosition) / 2.0;
  }
  else
  {
    interpolated_azimuth_delta = (dataPacket->firingData[index].rotationalPosition - dataPacket->firingData[0].rotationalPosition) / 2.0;
  }

  for (std::uint8_t i = 0; i < HDL_FIRING_PER_PKT; ++i)
  {
    HDLFiringData firing_data = dataPacket->firingData[i];

    for (std::uint8_t j = 0; j < HDL_LASER_PER_FIRING; j++)
    {
      double current_azimuth = firing_data.rotationalPosition;
      if (j >= VLP_MAX_NUM_LASERS)
      {
        current_azimuth += interpolated_azimuth_delta;
      }
      if (current_azimuth > 36000)
      {
        current_azimuth -= 36000;
      }
      if (current_azimuth < HDLGrabber::last_azimuth_)
      {
        if (!current_sweep_xyz_->empty ())
        {
          current_sweep_xyz_->is_dense = current_sweep_xyzrgba_->is_dense = current_sweep_xyzi_->is_dense = false;
          current_sweep_xyz_->header.stamp = velodyne_time;
          current_sweep_xyzrgba_->header.stamp = velodyne_time;
          current_sweep_xyzi_->header.stamp = velodyne_time;
          current_sweep_xyz_->header.seq = sweep_counter;
          current_sweep_xyzrgba_->header.seq = sweep_counter;
          current_sweep_xyzi_->header.seq = sweep_counter;

          sweep_counter++;

          HDLGrabber::fireCurrentSweep ();
        }
        current_sweep_xyz_.reset (new pcl::PointCloud<pcl::PointXYZ> ());
        current_sweep_xyzrgba_.reset (new pcl::PointCloud<pcl::PointXYZRGBA> ());
        current_sweep_xyzi_.reset (new pcl::PointCloud<pcl::PointXYZI> ());
      }

      PointXYZ xyz;
      PointXYZI xyzi;
      PointXYZRGBA xyzrgba;
      PointXYZ dual_xyz;
      PointXYZI dual_xyzi;
      PointXYZRGBA dual_xyzrgba;

      HDLGrabber::computeXYZI (xyzi, current_azimuth, firing_data.laserReturns[j], laser_corrections_[j % VLP_MAX_NUM_LASERS]);

      xyz.x = xyzrgba.x = xyzi.x;
      xyz.y = xyzrgba.y = xyzi.y;
      xyz.z = xyzrgba.z = xyzi.z;

      xyzrgba.rgba = laser_rgb_mapping_[j % VLP_MAX_NUM_LASERS].rgba;

      if (dataPacket->mode == VLP_DUAL_MODE)
      {
        HDLGrabber::computeXYZI (dual_xyzi, current_azimuth, dataPacket->firingData[i + 1].laserReturns[j], laser_corrections_[j % VLP_MAX_NUM_LASERS]);

        dual_xyz.x = dual_xyzrgba.x = dual_xyzi.x;
        dual_xyz.y = dual_xyzrgba.y = dual_xyzi.y;
        dual_xyz.z = dual_xyzrgba.z = dual_xyzi.z;

        dual_xyzrgba.rgba = laser_rgb_mapping_[j % VLP_MAX_NUM_LASERS].rgba;
      }

      if (! (std::isnan (xyz.x) || std::isnan (xyz.y) || std::isnan (xyz.z)))
      {
        current_sweep_xyz_->push_back (xyz);
        current_sweep_xyzrgba_->push_back (xyzrgba);
        current_sweep_xyzi_->push_back (xyzi);

        last_azimuth_ = current_azimuth;
      }
      if (dataPacket->mode == VLP_DUAL_MODE)
      {
        if ((dual_xyz.x != xyz.x || dual_xyz.y != xyz.y || dual_xyz.z != xyz.z)
            && ! (std::isnan (dual_xyz.x) || std::isnan (dual_xyz.y) || std::isnan (dual_xyz.z)))
        {
          current_sweep_xyz_->push_back (dual_xyz);
          current_sweep_xyzrgba_->push_back (dual_xyzrgba);
          current_sweep_xyzi_->push_back (dual_xyzi);
        }
      }
    }
    if (dataPacket->mode == VLP_DUAL_MODE)
    {
      i++;
    }
  }
}

/////////////////////////////////////////////////////////////////////////////
std::string
pcl::VLPGrabber::getName () const
{
  return (std::string ("Velodyne LiDAR (VLP) Grabber"));
}

/////////////////////////////////////////////////////////////////////////////
void
pcl::VLPGrabber::setLaserColorRGB (const pcl::RGB& color,
                                   const std::uint8_t laserNumber)
{
  if (laserNumber >= VLP_MAX_NUM_LASERS)
    return;

  laser_rgb_mapping_[laserNumber] = color;
}

/////////////////////////////////////////////////////////////////////////////
std::uint8_t
pcl::VLPGrabber::getMaximumNumberOfLasers () const
{
    return (VLP_MAX_NUM_LASERS);
}
