/*
* SPDX-License-Identifier: BSD-3-Clause
*
*  Point Cloud Library (PCL) - www.pointclouds.org
*  Copyright (c) 2025-, Open Perception Inc.
*
*  All rights reserved
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
      * Automatically select the fastest search method for the given point cloud. Make sure to delete the returned object after use!
      * \param[in] cloud Point cloud, this function will pass it to the search method via setInputCloud
      * \param[in] sorted_results Whether the search method should always return results sorted by distance (may be slower than unsorted)
      * \param[in] purpose Optional, can be used to give more information about what this search method will be used for, to achieve optimal performance
      */
    template<typename PointT>
    pcl::search::Search<PointT> * autoSelectMethod(const typename pcl::PointCloud<PointT>::ConstPtr& cloud, bool sorted_results, Purpose purpose = Purpose::undefined)
    {
      return autoSelectMethod<PointT>(cloud, pcl::IndicesConstPtr(), sorted_results, purpose);
    }

    /**
      * Automatically select the fastest search method for the given point cloud. Make sure to delete the returned object after use!
      * \param[in] cloud Point cloud, this function will pass it to the search method via setInputCloud
      * \param[in] indices Will be passed to the search method via setInputCloud, together with the point cloud
      * \param[in] sorted_results Whether the search method should always return results sorted by distance (may be slower than unsorted)
      * \param[in] purpose Optional, can be used to give more information about what this search method will be used for, to achieve optimal performance
      */
    template<typename PointT>
    pcl::search::Search<PointT> * autoSelectMethod(const typename pcl::PointCloud<PointT>::ConstPtr& cloud, const pcl::IndicesConstPtr& indices, bool sorted_results, Purpose purpose = Purpose::undefined);
  } // namespace search
} // namespace pcl

#ifdef PCL_NO_PRECOMPILE
#include <pcl/search/impl/auto.hpp>
#endif
