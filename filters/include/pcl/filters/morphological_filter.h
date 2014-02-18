/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009-2012, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
 *  Copyright (c) 2014, RadiantBlue Technologies, Inc.
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

#ifndef PCL_FILTERS_MORPHOLOGICAL_FILTER_H_
#define PCL_FILTERS_MORPHOLOGICAL_FILTER_H_

#include <string>
#include <pcl/pcl_base.h>
#include <pcl/PointIndices.h>
#include <pcl/conversions.h>
#include <locale>

namespace pcl
{
  /** \brief Morphological dilate of the input point cloud
    * \param[in] cloud_in the input point cloud dataset
    * \param[in] resolution the window size to be used for morphological dilate
    * \param[out] cloud_out the resultant output point cloud dataset
    * \note Assumes unique indices.
    * \ingroup filters
    */
  template <typename PointT> PCL_EXPORTS void
  morphologicalDilate (const pcl::PointCloud<PointT> &cloud_in,
                       float resolution,
                       pcl::PointCloud<PointT> &cloud_out);

  /** \brief Morphological erode of the input point cloud
    * \param[in] cloud_in the input point cloud dataset
    * \param[in] resolution the window size to be used for morphological erode
    * \param[out] cloud_out the resultant output point cloud dataset
    * \note Assumes unique indices.
    * \ingroup filters
    */
  template <typename PointT> PCL_EXPORTS void
  morphologicalErode (const pcl::PointCloud<PointT> &cloud_in,
                      float resolution,
                      pcl::PointCloud<PointT> &cloud_out);

  /** \brief Morphological open of the input point cloud
    * \param[in] cloud_in the input point cloud dataset
    * \param[in] resolution the window size to be used for morphological open
    * \param[out] cloud_out the resultant output point cloud dataset
    * \note Assumes unique indices.
    * \ingroup filters
    */
  template <typename PointT> PCL_EXPORTS void
  morphologicalOpen (const pcl::PointCloud<PointT> &cloud_in,
                     float resolution,
                     pcl::PointCloud<PointT> &cloud_out);

  /** \brief Morphological close of the input point cloud
    * \param[in] cloud_in the input point cloud dataset
    * \param[in] resolution the window size to be used for morphological close
    * \param[out] cloud_out the resultant output point cloud dataset
    * \note Assumes unique indices.
    * \ingroup filters
    */
  template <typename PointT> PCL_EXPORTS void
  morphologicalClose (const pcl::PointCloud<PointT> &cloud_in,
                      float resolution,
                      pcl::PointCloud<PointT> &cloud_out);
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/filters/impl/morphological_filter.hpp>
#endif

#endif  //#ifndef PCL_FILTERS_MORPHOLOGICAL_FILTER_H_

