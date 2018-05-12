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
 * $Id: io.h 2413 2011-09-07 07:01:00Z rusu $
 *
 */

#ifndef PCL_KDTREE_IO_H_
#define PCL_KDTREE_IO_H_

#include <pcl/point_cloud.h>

namespace pcl
{
  /** \brief Get a set of approximate indices for a given point cloud into a reference point cloud. 
    * The coordinates of the two point clouds can differ. The method uses an internal KdTree for 
    * finding the closest neighbors from \a cloud_in in \a cloud_ref. 
    *
    * \param[in] cloud_in the input point cloud dataset
    * \param[in] cloud_ref the reference point cloud dataset
    * \param[out] indices the resultant set of nearest neighbor indices of \a cloud_in in \a cloud_ref
    * \ingroup kdtree
    */
  template <typename PointT> void
  getApproximateIndices (const typename pcl::PointCloud<PointT>::ConstPtr &cloud_in,
                         const typename pcl::PointCloud<PointT>::ConstPtr &cloud_ref,
                         std::vector<int> &indices);

  /** \brief Get a set of approximate indices for a given point cloud into a reference point cloud. 
    * The coordinates of the two point clouds can differ. The method uses an internal KdTree for 
    * finding the closest neighbors from \a cloud_in in \a cloud_ref. 
    *
    * \param[in] cloud_in the input point cloud dataset
    * \param[in] cloud_ref the reference point cloud dataset
    * \param[out] indices the resultant set of nearest neighbor indices of \a cloud_in in \a cloud_ref
    * \ingroup kdtree
    */
  template <typename Point1T, typename Point2T> void
  getApproximateIndices (const typename pcl::PointCloud<Point1T>::ConstPtr &cloud_in,
                         const typename pcl::PointCloud<Point2T>::ConstPtr &cloud_ref,
                         std::vector<int> &indices);
}

#include <pcl/kdtree/impl/io.hpp>

#endif  //#ifndef PCL_KDTREE_IO_H_

