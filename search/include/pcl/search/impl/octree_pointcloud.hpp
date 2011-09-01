/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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
 * Author: Julius Kammerl (julius@kammerl.de)
 */

#ifndef PCL_OCTREE_SEARCH_POINTCLOUD_HPP_
#define PCL_OCTREE_SEARCH_POINTCLOUD_HPP_

#include <vector>
#include <assert.h>

#include "pcl/common/common.h"

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafTWrap, typename OctreeT> int
pcl::octree::OctreeWrapper<PointT, LeafTWrap, OctreeT>::nearestKSearch (
    const PointCloudConstPtr &cloud_arg, int index_arg, int k_arg,
    std::vector<int> &k_indices_arg,
    std::vector<float> &k_sqr_distances_arg)
{
  _searchptr->setInputCloud (cloud_arg);
//   this->addPointsFromInputCloud ();

  return (_searchptr->nearestKSearch (index_arg, k_arg, k_indices_arg, k_sqr_distances_arg));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafTWrap, typename OctreeT> int
pcl::octree::OctreeWrapper<PointT, LeafTWrap, OctreeT>::nearestKSearch (const PointT &p_q_arg, int k_arg,
                                                            std::vector<int> &k_indices_arg,
                                                            std::vector<float> &k_sqr_distances_arg)
{
  return (_searchptr->nearestKSearch(p_q_arg,k_arg,k_indices_arg,k_sqr_distances_arg));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafTWrap, typename OctreeT> int
pcl::octree::OctreeWrapper<PointT, LeafTWrap, OctreeT>::nearestKSearch (
    int index_arg, int k_arg,
    std::vector<int> &k_indices_arg,
    std::vector<float> &k_sqr_distances_arg)
{
  return _searchptr->nearestKSearch(index_arg,k_arg,k_indices_arg,k_sqr_distances_arg);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafTWrap, typename OctreeT> void
pcl::octree::OctreeWrapper<PointT, LeafTWrap, OctreeT>::approxNearestSearch (
    const PointCloudConstPtr &cloud_arg,
    int query_index_arg, int &result_index_arg,
    float &sqr_distance_arg)
{
  _searchptr->setInputCloud (cloud_arg);
 // this->addPointsFromInputCloud ();

  _searchptr->approxNearestSearch (query_index_arg, result_index_arg, sqr_distance_arg);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafTWrap, typename OctreeT> void
pcl::octree::OctreeWrapper<PointT, LeafTWrap, OctreeT>::approxNearestSearch (
    const PointT &p_q_arg, int &result_index_arg,
    float &sqr_distance_arg)
{
  _searchptr->approxNearestSearch(p_q_arg,result_index_arg,sqr_distance_arg);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafTWrap, typename OctreeT> void
pcl::octree::OctreeWrapper<PointT, LeafTWrap, OctreeT>::approxNearestSearch (
    int query_index_arg, int &result_index_arg,
    float &sqr_distance_arg)
{
  _searchptr->approxNearestSearch(query_index_arg,result_index_arg,sqr_distance_arg);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafTWrap, typename OctreeT> int
pcl::octree::OctreeWrapper<PointT, LeafTWrap, OctreeT>::radiusSearch (
    const PointCloudConstPtr &cloud_arg, int index_arg,
    double radius_arg, std::vector<int> &k_indices_arg,
    std::vector<float> &k_sqr_distances_arg, int max_nn_arg)
{
  _searchptr->setInputCloud (cloud_arg);
 //   this->addPointsFromInputCloud ();

  return (_searchptr->radiusSearch (index_arg, radius_arg, k_indices_arg, k_sqr_distances_arg, max_nn_arg));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafTWrap, typename OctreeT> int
pcl::octree::OctreeWrapper<PointT, LeafTWrap, OctreeT>::radiusSearch (
    const PointT &p_q_arg, const double radius_arg,
    std::vector<int> &k_indices_arg,
    std::vector<float> &k_sqr_distances_arg, int max_nn_arg) const
{
  return (_searchptr->radiusSearch(p_q_arg,radius_arg,k_indices_arg,k_sqr_distances_arg,max_nn_arg));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafTWrap, typename OctreeT> int
pcl::octree::OctreeWrapper<PointT, LeafTWrap, OctreeT>::radiusSearch (
    int index_arg, const double radius_arg,
    std::vector<int> &k_indices_arg,
    std::vector<float> &k_sqr_distances_arg, int max_nn_arg) const
{
  return (_searchptr->radiusSearch (index_arg, radius_arg, k_indices_arg, k_sqr_distances_arg, max_nn_arg));
}

#define PCL_INSTANTIATE_OctreeSearch(T) template class PCL_EXPORTS pcl::octree::OctreeWrapper<T, pcl::octree::OctreeLeafDataTVector<int> , pcl::octree::OctreeBase<int, pcl::octree::OctreeLeafDataTVector<int> > >;

#endif // PCL_OCTREE_POINTCLOUD_HPP_
