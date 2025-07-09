/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
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

#include <pcl/point_cloud.h>
#include <pcl/search/search.h>

namespace pcl {
  namespace search {
    enum class Purpose : std::uint32_t {
      undefined = 0, ///< Default value, for general-purpose search method
      radius_search = 1, ///< The search method will mainly be used for radiusSearch
      one_knn_search = 2, ///< The search method will mainly be used for nearestKSearch where k is 1
      many_knn_search = 4 ///< The search method will mainly be used for nearestKSearch where k is larger than 1
    };

    /**
      * Automatically select the fastest search method for the given point cloud. Make sure to delete the returned object after use! Distance function (euclidean?)
      * \param[in] sorted_results Whether the search method should always return results sorted by distance (may be slower than unsorted)
      * \param[in] purpose Optional, can be used to give more information about what this search method will be used for, to achieve optimal performance
      */
    template<typename PointT>
    pcl::search::Search<PointT> * autoSelectMethod(const typename pcl::PointCloud<PointT>::ConstPtr& cloud, bool sorted_results, Purpose purpose = Purpose::undefined);
  } // namespace search
} // namespace pcl
