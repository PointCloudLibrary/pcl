/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
 * $Id: cloud_properties.h 5295 2012-03-25 19:03:32Z rusu $
 *
 */

#ifndef PCL_CLOUD_PROPERTIES_H_
#define PCL_CLOUD_PROPERTIES_H_

#include <pcl/common/eigen.h>

namespace pcl
{
  /** \brief CloudProperties stores a list of \b optional point cloud properties such as:
    *
    *   - acquisition time
    *   - the origin and orientation of the sensor when the data was acquired
    *   - optional user parameters
    *
    * <b>This part of the API is for advanced users only, and constitutes a transition to the 2.0 API!</b>
    *
    * \author Radu B. Rusu
    */
  class CloudProperties
  {
    public:

      /** \brief Default constructor. Sets:
        *
        *   - \ref acquisition_time to 0
        *   - \ref sensor_origin to {0, 0, 0}
        *   - \ref sensor_orientation to {1, 0, 0, 0}
        */
      CloudProperties () :
        acquisition_time (0),
        sensor_origin (Eigen::Vector4f::Zero ()), 
        sensor_orientation (Eigen::Quaternionf::Identity ())
      {
      }

      /** \brief Data acquisition time. */
      uint64_t acquisition_time;

      /** \brief Sensor acquisition pose (origin/translation in the cloud data coordinate system). */
      Eigen::Vector4f    sensor_origin;

      /** \brief Sensor acquisition pose (rotation in the cloud data coordinate system). 
        * \note the data is stored in (w, x, y, z) format.
        */
      Eigen::Quaternionf sensor_orientation;
  };

}

#endif  // PCL_CLOUD_PROPERTIES_H_


