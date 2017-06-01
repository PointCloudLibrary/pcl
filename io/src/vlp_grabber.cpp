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

#define VLP_MAX_NUM_LASERS 16
#define VLP_DUAL_MODE 0x39

/////////////////////////////////////////////////////////////////////////////
pcl::VLPGrabber::VLPGrabber (const std::string& pcapFile) :
    HDLGrabber ("", pcapFile)
{
  loadVLP16Corrections ();
}

/////////////////////////////////////////////////////////////////////////////
pcl::VLPGrabber::VLPGrabber (const boost::asio::ip::address& ipAddress,
                             const unsigned short int port) :
    HDLGrabber (ipAddress, port)
{
  loadVLP16Corrections ();
}

/////////////////////////////////////////////////////////////////////////////
pcl::VLPGrabber::~VLPGrabber () throw ()
{
}

/////////////////////////////////////////////////////////////////////////////
void
pcl::VLPGrabber::loadVLP16Corrections ()
{
  double vlp16_vertical_corrections[] = { -15, 1, -13, 3, -11, 5, -9, 7, -7, 9, -5, 11, -3, 13, -1, 15

  };
  for (int i = 0; i < VLP_MAX_NUM_LASERS; i++)
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
  static uint32_t scan_counter = 0;
  static uint32_t sweep_counter = 0;
  if (sizeof(HDLLaserReturn) != 3)
    return;

  time_t system_time;
  time (&system_time);
  time_t velodyne_time = (system_time & 0x00000000ffffffffl) << 32 | dataPacket->gpsTimestamp;

  scan_counter++;

  double interpolated_azimuth_delta;

  int index = 1;
  if (dataPacket->mode == VLP_DUAL_MODE)
  {
    index = 2;
  }
  if (dataPacket->firingData[index].rotationalPosition < dataPacket->firingData[0].rotationalPosition)
  {
    interpolated_azimuth_delta = ( (dataPacket->firingData[index].rotationalPosition + 36000) - dataPacket->firingData[0].rotationalPosition) / 2.0;
  }
  else
  {
    interpolated_azimuth_delta = (dataPacket->firingData[index].rotationalPosition - dataPacket->firingData[0].rotationalPosition) / 2.0;
  }

  for (int i = 0; i < HDL_FIRING_PER_PKT; ++i)
  {
    HDLFiringData firing_data = dataPacket->firingData[i];

    for (int j = 0; j < HDL_LASER_PER_FIRING; j++)
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
        if (current_sweep_xyz_->size () > 0)
        {
          current_sweep_xyz_->is_dense = current_sweep_xyzi_->is_dense = false;
          current_sweep_xyz_->header.stamp = velodyne_time;
          current_sweep_xyzi_->header.stamp = velodyne_time;
          current_sweep_xyz_->header.seq = sweep_counter;
          current_sweep_xyzi_->header.seq = sweep_counter;

          sweep_counter++;

          HDLGrabber::fireCurrentSweep ();
        }
        current_sweep_xyz_.reset (new pcl::PointCloud<pcl::PointXYZ> ());
        current_sweep_xyzi_.reset (new pcl::PointCloud<pcl::PointXYZI> ());
      }

      PointXYZ xyz;
      PointXYZI xyzi;
      PointXYZ dual_xyz;
      PointXYZI dual_xyzi;

      HDLGrabber::computeXYZI (xyzi, current_azimuth, firing_data.laserReturns[j], laser_corrections_[j % VLP_MAX_NUM_LASERS]);

      xyz.x = xyzi.x;
      xyz.y = xyzi.y;
      xyz.z = xyzi.z;

      if (dataPacket->mode == VLP_DUAL_MODE)
      {
        HDLGrabber::computeXYZI (dual_xyzi, current_azimuth, dataPacket->firingData[i + 1].laserReturns[j], laser_corrections_[j % VLP_MAX_NUM_LASERS]);

        dual_xyz.x = dual_xyzi.x;
        dual_xyz.y = dual_xyzi.y;
        dual_xyz.z = dual_xyzi.z;

      }

      if (! (pcl_isnan (xyz.x) || pcl_isnan (xyz.y) || pcl_isnan (xyz.z)))
      {
        current_sweep_xyz_->push_back (xyz);
        current_sweep_xyzi_->push_back (xyzi);

        last_azimuth_ = current_azimuth;
      }
      if (dataPacket->mode == VLP_DUAL_MODE)
      {
        if ( (dual_xyz.x != xyz.x || dual_xyz.y != xyz.y || dual_xyz.z != xyz.z)
            && ! (pcl_isnan (dual_xyz.x) || pcl_isnan (dual_xyz.y) || pcl_isnan (dual_xyz.z)))
        {
          current_sweep_xyz_->push_back (dual_xyz);
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

