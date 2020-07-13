/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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

#ifndef PCL_SEGMENTATION_IMPL_EXTRACT_CLUSTERS_H_
#define PCL_SEGMENTATION_IMPL_EXTRACT_CLUSTERS_H_

#include <pcl/segmentation/extract_clusters.h>

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::extractEuclideanClusters (const PointCloud<PointT> &cloud,
                               const typename search::Search<PointT>::Ptr &tree,
                               float tolerance, std::vector<PointIndices> &clusters,
                               unsigned int min_pts_per_cluster,
                               unsigned int max_pts_per_cluster)
{
  auto extract_all = [&](int i, int j, const Indices& nn_indices) {
    utils::ignore(i, j, nn_indices);
    return true;
  };
  pcl::extractEuclideanClusters<PointT>(cloud, extract_all, tree, tolerance, clusters, min_pts_per_cluster, max_pts_per_cluster);
}

//////////////////////////////////////////////////////////////////////////////////////////////
/** @todo: fix the return value, make sure the exit is not needed anymore*/
template <typename PointT> void
pcl::extractEuclideanClusters (const PointCloud<PointT> &cloud,
                               const Indices &indices,
                               const typename search::Search<PointT>::Ptr &tree,
                               float tolerance, std::vector<PointIndices> &clusters,
                               unsigned int min_pts_per_cluster,
                               unsigned int max_pts_per_cluster)
{
  if (indices.empty())
    return;

  auto extract_all = [&](index_t i, index_t j, const Indices& nn_indices) {
    utils::ignore(i, j, nn_indices);
    return true;
  };
  pcl::extractEuclideanClusters(cloud, indices, extract_all, tree, tolerance, clusters, min_pts_per_cluster, max_pts_per_cluster);
}

//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////

template <typename PointT> void 
pcl::EuclideanClusterExtraction<PointT>::extract (std::vector<PointIndices> &clusters)
{
  if (!initCompute () || 
      (input_   && input_->points.empty ()) ||
      (indices_ && indices_->empty ()))
  {
    clusters.clear ();
    return;
  }

  // Initialize the spatial locator
  if (!tree_)
  {
    if (input_->isOrganized ())
      tree_.reset (new pcl::search::OrganizedNeighbor<PointT> ());
    else
      tree_.reset (new pcl::search::KdTree<PointT> (false));
  }

  // Send the input dataset to the spatial locator
  tree_->setInputCloud (input_, indices_);
  extractEuclideanClusters (*input_, *indices_, tree_, static_cast<float> (cluster_tolerance_), clusters, min_pts_per_cluster_, max_pts_per_cluster_);

  //tree_->setInputCloud (input_);
  //extractEuclideanClusters (*input_, tree_, cluster_tolerance_, clusters, min_pts_per_cluster_, max_pts_per_cluster_);

  // Sort the clusters based on their size (largest one first)
  std::sort (clusters.rbegin (), clusters.rend (), comparePointClusters);

  deinitCompute ();
}

#define PCL_INSTANTIATE_EuclideanClusterExtraction(T) template class PCL_EXPORTS pcl::EuclideanClusterExtraction<T>;
#define PCL_INSTANTIATE_extractEuclideanClusters(T) template void PCL_EXPORTS pcl::extractEuclideanClusters<T>(const pcl::PointCloud<T> &, const typename pcl::search::Search<T>::Ptr &, float , std::vector<pcl::PointIndices> &, unsigned int, unsigned int);
#define PCL_INSTANTIATE_extractEuclideanClusters_indices(T) template void PCL_EXPORTS pcl::extractEuclideanClusters<T>(const pcl::PointCloud<T> &, const Indices &, const typename pcl::search::Search<T>::Ptr &, float , std::vector<pcl::PointIndices> &, unsigned int, unsigned int);

#endif        // PCL_EXTRACT_CLUSTERS_IMPL_H_
