/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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
 * $Id$
 *
 */

#ifndef PCL_ROS_CONVERSIONS_H_ 
#define PCL_ROS_CONVERSIONS_H_

#ifdef __DEPRECATED
#warning The <pcl/ros/conversions.h> header is deprecated. please use \
<pcl/conversions.h> instead.
#endif

#include <pcl/conversions.h>

namespace pcl
{
  /** \brief Convert a PCLPointCloud2 binary data blob into a pcl::PointCloud<T> object using a field_map.
    * \param[in] msg the PCLPointCloud2 binary blob
    * \param[out] cloud the resultant pcl::PointCloud<T>
    * \param[in] field_map a MsgFieldMap object
    *
    * \note Use fromROSMsg (PCLPointCloud2, PointCloud<T>) directly or create you
    * own MsgFieldMap using:
    *
    * \code
    * MsgFieldMap field_map;
    * createMapping<PointT> (msg.fields, field_map);
    * \endcode
    */
  template <typename PointT>
  PCL_DEPRECATED (void fromROSMsg (
        const pcl::PCLPointCloud2& msg, pcl::PointCloud<PointT>& cloud,
        const MsgFieldMap& field_map),
      "pcl::fromROSMsg is deprecated, please use fromPCLPointCloud2 instead.");

  template <typename PointT> void 
  fromROSMsg (const pcl::PCLPointCloud2& msg, pcl::PointCloud<PointT>& cloud,
              const MsgFieldMap& field_map)
  {
    fromPCLPointCloud2 (msg, cloud, field_map);
  }

  /** \brief Convert a PCLPointCloud2 binary data blob into a pcl::PointCloud<T> object.
    * \param[in] msg the PCLPointCloud2 binary blob
    * \param[out] cloud the resultant pcl::PointCloud<T>
    */
  template<typename PointT>
  PCL_DEPRECATED (void fromROSMsg (
        const pcl::PCLPointCloud2& msg, pcl::PointCloud<PointT>& cloud),
      "pcl::fromROSMsg is deprecated, please use fromPCLPointCloud2 instead.");
  template<typename PointT> void 
  fromROSMsg (const pcl::PCLPointCloud2& msg, pcl::PointCloud<PointT>& cloud)
  {
    fromPCLPointCloud2 (msg, cloud);
  }

  /** \brief Convert a pcl::PointCloud<T> object to a PCLPointCloud2 binary data blob.
    * \param[in] cloud the input pcl::PointCloud<T>
    * \param[out] msg the resultant PCLPointCloud2 binary blob
    */
  template<typename PointT>
  PCL_DEPRECATED (void toROSMsg (
        const pcl::PointCloud<PointT>& cloud, pcl::PCLPointCloud2& msg),
      "pcl::fromROSMsg is deprecated, please use fromPCLPointCloud2 instead.");
  template<typename PointT> void 
  toROSMsg (const pcl::PointCloud<PointT>& cloud, pcl::PCLPointCloud2& msg)
  {
    toPCLPointCloud2 (cloud, msg);
  }

   /** \brief Copy the RGB fields of a PointCloud into pcl::PCLImage format
     * \param[in] cloud the point cloud message
     * \param[out] msg the resultant pcl::PCLImage
     * CloudT cloud type, CloudT should be akin to pcl::PointCloud<pcl::PointXYZRGBA>
     * \note will throw std::runtime_error if there is a problem
     */
  template<typename CloudT> 
  PCL_DEPRECATED (void toROSMsg (
        const CloudT& cloud, pcl::PCLImage& msg),
      "pcl::fromROSMsg is deprecated, please use fromPCLPointCloud2 instead.");
  template<typename CloudT> void
  toROSMsg (const CloudT& cloud, pcl::PCLImage& msg)
  {
    toPCLPointCloud2 (cloud, msg);
  }

  /** \brief Copy the RGB fields of a PCLPointCloud2 msg into pcl::PCLImage format
    * \param cloud the point cloud message
    * \param msg the resultant pcl::PCLImage
    * will throw std::runtime_error if there is a problem
    */
  PCL_DEPRECATED (inline void toROSMsg (
        const pcl::PCLPointCloud2& cloud, pcl::PCLImage& msg),
      "pcl::fromROSMsg is deprecated, please use fromPCLPointCloud2 instead.");
  inline void
  toROSMsg (const pcl::PCLPointCloud2& cloud, pcl::PCLImage& msg)
  {
    toPCLPointCloud2 (cloud, msg);
  }
}

#endif  //#ifndef PCL_ROS_CONVERSIONS_H_
