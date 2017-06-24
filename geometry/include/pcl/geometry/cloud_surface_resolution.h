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

#ifndef PCL_GEOMETRY_CLOUD_SURFACE_RESOLUTION_H
#define PCL_GEOMETRY_CLOUD_SURFACE_RESOLUTION_H

#include <vector>

namespace pcl 
{
  namespace geometry 
  {
    /** \brief compute the mean resolution of a point cloud, as if it
      * were a surface or mesh, by way of the mean of the median
      * distances to the nine nearest neighbors of each point in the cloud
      * \param[in] input point cloud
      * \return mean resolution of the cloud
      */
    template <typename PointT>
    float
    computeCloudSurfaceResolution (typename const pcl::PointCloud<PointT>::ConstPtr & input);

    /** \brief compute the mean resolution of a point cloud, as if it
      * were a surface or mesh, using an array of indices; by way of
      * the mean of the median distances to the nine nearest neighbors
      * of each point in the cloud
      * \param[in] input point cloud
      * \param[in] indices indices from input to use
      * \return mean resolution of the cloud
      */
    template <typename PointT>
    float
    computeCloudSurfaceResolution (typename const pcl::PointCloud<PointT>::ConstPtr & input, const boost::shared_ptr<std::vector<int> > & indices)
  }
}

#include <pcl/geometry/impl/mesh_resolution.hpp>

#endif // PCL_GEOMETRY_CLOUD_SURFACE_RESOLUTION_H
