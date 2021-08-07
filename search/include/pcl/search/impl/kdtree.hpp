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

#ifndef PCL_SEARCH_KDTREE_IMPL_HPP_
#define PCL_SEARCH_KDTREE_IMPL_HPP_

#include <pcl/search/kdtree.h>

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, class Tree>
pcl::search::KdTree<PointT,Tree>::KdTree (bool sorted)
  : pcl::search::Search<PointT> ("KdTree", sorted)
  , tree_ (new Tree (sorted))
{
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, class Tree> void
pcl::search::KdTree<PointT,Tree>::setPointRepresentation (
    const PointRepresentationConstPtr &point_representation)
{
  tree_->setPointRepresentation (point_representation);
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, class Tree> void
pcl::search::KdTree<PointT,Tree>::setSortedResults (bool sorted_results)
{
  sorted_results_ = sorted_results;
  tree_->setSortedResults (sorted_results);
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, class Tree> void
pcl::search::KdTree<PointT,Tree>::setEpsilon (float eps)
{
  tree_->setEpsilon (eps);
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, class Tree> void
pcl::search::KdTree<PointT,Tree>::setInputCloud (
    const PointCloudConstPtr& cloud, 
    const IndicesConstPtr& indices)
{
  tree_->setInputCloud (cloud, indices);
  input_ = cloud;
  indices_ = indices;
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, class Tree> int
pcl::search::KdTree<PointT,Tree>::nearestKSearch (
    const PointT &point, int k, Indices &k_indices,
    std::vector<float> &k_sqr_distances) const
{
  return (tree_->nearestKSearch (point, k, k_indices, k_sqr_distances));
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, class Tree> int
pcl::search::KdTree<PointT,Tree>::radiusSearch (
    const PointT& point, double radius, 
    Indices &k_indices, std::vector<float> &k_sqr_distances,
    unsigned int max_nn) const
{
  return (tree_->radiusSearch (point, radius, k_indices, k_sqr_distances, max_nn));
}

#define PCL_INSTANTIATE_KdTree(T) template class PCL_EXPORTS pcl::search::KdTree<T>;

#endif  //#ifndef _PCL_SEARCH_KDTREE_IMPL_HPP_


