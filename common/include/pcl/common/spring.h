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

#ifndef PCL_POINT_CLOUD_SPRING_H_
#define PCL_POINT_CLOUD_SPRING_H_

#include <pcl/point_cloud.h>

namespace pcl
{
  namespace common
  {
    /** expand point cloud inserting \a amount rows at the 
     * top and the bottom of a point cloud and filling them with 
     * custom values.
     * \param[in] input the input point cloud
     * \param[out] output the output point cloud
     * \param[in] val the point value to be insterted
     * \param[in] amount the amount of rows to be added
     */
    template <typename PointT> void
    expandRows (const PointCloud<PointT>& input, PointCloud<PointT>& output, 
                const PointT& val, const size_t& amount);

    /** expand point cloud inserting \a amount columns at 
      * the right and the left of a point cloud and filling them with 
      * custom values.
      * \param[in] input the input point cloud
      * \param[out] output the output point cloud
      * \param[in] val the point value to be insterted
      * \param[in] amount the amount of columns to be added
      */
    template <typename PointT> void
    expandColumns (const PointCloud<PointT>& input, PointCloud<PointT>& output, 
                   const PointT& val, const size_t& amount);

    /** expand point cloud duplicating the \a amount top and bottom rows times.
      * \param[in] input the input point cloud
      * \param[out] output the output point cloud
      * \param[in] amount the amount of rows to be added
      */
    template <typename PointT> void
    duplicateRows (const PointCloud<PointT>& input, PointCloud<PointT>& output, 
                   const size_t& amount);

    /** expand point cloud duplicating the \a amount right and left columns
      * times.
      * \param[in] input the input point cloud
      * \param[out] output the output point cloud
      * \param[in] amount the amount of cilumns to be added
      */
    template <typename PointT> void
    duplicateColumns (const PointCloud<PointT>& input, PointCloud<PointT>& output, 
                      const size_t& amount);

    /** expand point cloud mirroring \a amount top and bottom rows. 
      * \param[in] input the input point cloud
      * \param[out] output the output point cloud
      * \param[in] amount the amount of rows to be added
      */
    template <typename PointT> void
    mirrorRows (const PointCloud<PointT>& input, PointCloud<PointT>& output, 
                const size_t& amount);

    /** expand point cloud mirroring \a amount right and left columns.
      * \param[in] input the input point cloud
      * \param[out] output the output point cloud
      * \param[in] amount the amount of rows to be added
      */
    template <typename PointT> void
    mirrorColumns (const PointCloud<PointT>& input, PointCloud<PointT>& output, 
                   const size_t& amount);

    /** delete \a amount rows in top and bottom of point cloud 
      * \param[in] input the input point cloud
      * \param[out] output the output point cloud
      * \param[in] amount the amount of rows to be added
      */
    template <typename PointT> void
    deleteRows (const PointCloud<PointT>& input, PointCloud<PointT>& output, 
                const size_t& amount);

    /** delete \a amount columns in top and bottom of point cloud
      * \param[in] input the input point cloud
      * \param[out] output the output point cloud
      * \param[in] amount the amount of rows to be added
      */
    template <typename PointT> void
    deleteCols (const PointCloud<PointT>& input, PointCloud<PointT>& output, 
                const size_t& amount);
  };
}

#include <pcl/common/impl/spring.hpp>

#endif
