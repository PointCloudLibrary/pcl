/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
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

#ifndef PCL_NO_PRECOMPILE
#include <pcl/impl/instantiate.hpp>
#include <pcl/point_types.h>
#include <pcl/common/utils.h> // for ignore

#include <pcl/search/auto.h>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/kdtree_nanoflann.h>

template<typename PointT>
pcl::search::Search<PointT> * pcl::search::autoSelectMethod(const typename pcl::PointCloud<PointT>::ConstPtr& cloud, bool sorted_results, pcl::search::Purpose purpose) { // TODO knn/radius/1knn search?
  pcl::search::Search<PointT> * searcher = nullptr;
  if (cloud->isOrganized ()) {
    searcher = new pcl::search::OrganizedNeighbor<PointT> (sorted_results);
    if(searcher->setInputCloud (cloud)) { // may return false if OrganizedNeighbor cannot work with the cloud, then use KdTree instead
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
  return searcher; // TODO should this guarantee to call setInputCloud on searcher?
}

#define PCL_INSTANTIATE_AutoSelectMethod(T) template PCL_EXPORTS pcl::search::Search<T> * pcl::search::autoSelectMethod<T>(const typename pcl::PointCloud<T>::ConstPtr& cloud, bool sorted_results, pcl::search::Purpose purpose);
PCL_INSTANTIATE(AutoSelectMethod, PCL_XYZ_POINT_TYPES)
#endif    // PCL_NO_PRECOMPILE

