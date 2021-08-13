/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Intelligent Robotics Lab, DLUT.
 *  Author: Qinghua Li, Yan Zhuang, Fei Yan
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
 *   * Neither the name of Intelligent Robotics Lab, DLUT. nor the names
 *     of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
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
 */

/**
  * \file bearing_angle_image.h
  * Created on: July 07, 2012
  */

#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace pcl
{
  /** \brief class BearingAngleImage is used as an interface to generate Bearing Angle(BA) image.
    * \author: Qinghua Li (qinghua__li@163.com)
    */
  class PCL_EXPORTS BearingAngleImage : public pcl::PointCloud<PointXYZRGBA>
  {
    public:
      // ===== TYPEDEFS =====
      using BaseClass = pcl::PointCloud<PointXYZRGBA>;

      // =====CONSTRUCTOR & DESTRUCTOR=====
      /** Constructor */
      BearingAngleImage ();

    public:
      /** \brief Reset all values to an empty Bearing Angle image */
      void
      reset ();

      /** \brief Calculate the angle between the laser beam and the segment joining two consecutive
        * measurement points.
        * \param point1
        * \param point2
        */
      double
      getAngle (const PointXYZ &point1, const PointXYZ &point2);

      /** \brief Transform 3D point cloud into a 2D Bearing Angle(BA) image */
      void
      generateBAImage (PointCloud<PointXYZ>& point_cloud);

    protected:
      /**< This point is used to be able to return a reference to a unknown gray point */
      PointXYZRGBA unobserved_point_;
  };
}
