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
pcl::search::Search<PointT> * pcl::search::autoSelectMethod(const typename pcl::PointCloud<PointT>::ConstPtr& cloud, bool sorted_results, pcl::search::Purpose purpose) {
  pcl::search::Search<PointT> * searcher = nullptr;
  if (cloud->isOrganized ()) {
    searcher = new pcl::search::OrganizedNeighbor<PointT> (sorted_results);
    if(searcher->setInputCloud (cloud)) { // may return false if OrganizedNeighbor cannot work with the cloud, then use another search method instead
      return searcher;
    }
  }
#if PCL_HAS_NANOFLANN
  searcher = new pcl::search::KdTreeNanoflann<PointT> (sorted_results, (purpose == pcl::search::Purpose::one_knn_search ? 10 : 20));
  if(searcher->setInputCloud (cloud)) {
    return searcher;
  }
#else
  pcl::utils::ignore(purpose);
#endif
#if PCL_HAS_FLANN
  searcher = new pcl::search::KdTree<PointT> (sorted_results);
  if(searcher->setInputCloud (cloud)) {
    return searcher;
  }
#endif
  // If nothing else works, use brute force method
  searcher = new pcl::search::BruteForce<PointT> (sorted_results);
  searcher->setInputCloud (cloud);
  return searcher;
}

#define PCL_INSTANTIATE_AutoSelectMethod(T) template PCL_EXPORTS pcl::search::Search<T> * pcl::search::autoSelectMethod<T>(const typename pcl::PointCloud<T>::ConstPtr& cloud, bool sorted_results, pcl::search::Purpose purpose);

#endif  //#ifndef PCL_SEARCH_AUTO_IMPL_HPP_
