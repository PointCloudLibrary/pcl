/*
* SPDX-License-Identifier: BSD-3-Clause
*
*  Point Cloud Library (PCL) - www.pointclouds.org
*  Copyright (c) 2025-, Open Perception Inc.
*
*  All rights reserved
*/

#ifndef PCL_SEARCH_AUTO_IMPL_HPP_
#define PCL_SEARCH_AUTO_IMPL_HPP_

#include <pcl/common/utils.h> // for ignore
#include <pcl/search/auto.h>
#include <pcl/search/brute_force.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/kdtree_nanoflann.h>
#include <pcl/search/organized.h>

template<typename PointT>
pcl::search::Search<PointT> * pcl::search::autoSelectMethod(const typename pcl::PointCloud<PointT>::ConstPtr& cloud, const pcl::IndicesConstPtr& indices, bool sorted_results, pcl::search::Purpose purpose) {
  pcl::search::Search<PointT> * searcher = nullptr;
  if constexpr (pcl::traits::has_xyz_v<PointT>) {
    if (cloud->isOrganized ()) {
      searcher = new pcl::search::OrganizedNeighbor<PointT> (sorted_results);
      if(searcher->setInputCloud (cloud, indices)) { // may return false if OrganizedNeighbor cannot work with the cloud, then use another search method instead
        return searcher;
      }
      delete searcher;
    }
  }
#if PCL_HAS_NANOFLANN
  // we get the number of search dimensions as a compile-time-constant via NR_DIMS. NR_DIMS may be -1 if it is not possible to determine the dimensions at compile-time (only at run-time), however then searching may be slower. If NR_DIMS is not -1, it must be the same as the return value of getNumberOfDimensions().
  searcher = new pcl::search::KdTreeNanoflann<PointT, pcl::DefaultPointRepresentation<PointT>::NR_DIMS> (sorted_results, (purpose == pcl::search::Purpose::one_knn_search ? 10 : 20));
  if(searcher->setInputCloud (cloud, indices)) {
    return searcher;
  }
  delete searcher;
#else
  pcl::utils::ignore(purpose);
#endif
#if PCL_HAS_FLANN
  searcher = new pcl::search::KdTree<PointT> (sorted_results);
  if(searcher->setInputCloud (cloud, indices)) {
    return searcher;
  }
  delete searcher;
#endif
  // If nothing else works, and the point type has xyz coordinates, use brute force method
  if constexpr (pcl::traits::has_xyz_v<PointT>) {
    searcher = new pcl::search::BruteForce<PointT> (sorted_results);
    searcher->setInputCloud (cloud, indices);
    return searcher;
  }
  PCL_ERROR("[pcl::search::autoSelectMethod] No suitable method found. Make sure you have nanoflann and/or FLANN installed.\n");
  return nullptr;
}

#define PCL_INSTANTIATE_AutoSelectMethod(T) template PCL_EXPORTS pcl::search::Search<T> * pcl::search::autoSelectMethod<T>(const typename pcl::PointCloud<T>::ConstPtr& cloud, const pcl::IndicesConstPtr& indices, bool sorted_results, pcl::search::Purpose purpose);

#endif  //#ifndef PCL_SEARCH_AUTO_IMPL_HPP_
