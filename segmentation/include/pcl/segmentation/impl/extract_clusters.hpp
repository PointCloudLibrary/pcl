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

template <typename PointT, typename FunctorT> void
pcl::extractEuclideanClusters (
    const PointCloud<PointT> &cloud,
    const Indices &indices,
    FunctorT filter,
    const typename search::Search<PointT>::Ptr &tree,
    float tolerance, std::vector<PointIndices> &clusters,
    unsigned int min_pts_per_cluster,
    unsigned int max_pts_per_cluster) {
  if (tree->getInputCloud ()->points.size () != cloud.points.size ())
  {
    PCL_ERROR ("[pcl::extractEuclideanClusters] Tree built for a different point cloud dataset (%lu) than the input cloud (%lu)!\n", tree->getInputCloud ()->points.size (), cloud.points.size ());
    return;
  }
  // Check if the tree is sorted -- if it is we don't need to check the first element
  index_t nn_start_idx = tree->getSortedResults () ? 1 : 0;
  // Create a bool vector of processed point indices, and initialize it to false
  std::vector<bool> processed (cloud.points.size (), false);

  Indices nn_indices;
  std::vector<float> nn_distances;
  // Process all points in the indices vector


  auto it = indices.empty() ? ConstCloudIterator<PointT>(cloud) : ConstCloudIterator<PointT>(cloud, indices);

  for (; it.isValid(); ++it) {
    if (processed[it.getCurrentIndex()])
      continue;

    index_t sq_idx = 0;
    clusters.emplace_back();
    auto& seed_queue = clusters.back();
    seed_queue.indices.push_back (it.getCurrentIndex());

    processed[it.getCurrentIndex()] = true;

    while (sq_idx < static_cast<index_t> (seed_queue.indices.size()))
    {
      // Search for sq_idx
      if (!tree->radiusSearch (seed_queue.indices[sq_idx], tolerance, nn_indices, nn_distances))
      {
        sq_idx++;
        continue;
      }

      for (std::size_t j = nn_start_idx; j < nn_indices.size (); ++j)             // can't assume sorted (default isn't!)
      {
        if (processed[nn_indices[j]])        // Has this point been processed before ?
          continue;

        if (filter(it.getCurrentIndex(), j, nn_indices)) {
          seed_queue.indices.push_back(nn_indices[j]);
          processed[nn_indices[j]] = true;
        }
      }

      sq_idx++;
    }

    // If this queue is satisfactory, add to the clusters
    if (seed_queue.indices.size () >= min_pts_per_cluster && seed_queue.indices.size () <= max_pts_per_cluster)
      seed_queue.header = cloud.header;
    else
      clusters.pop_back();
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::extractEuclideanClusters (const PointCloud<PointT> &cloud,
                               const typename search::Search<PointT>::Ptr &tree,
                               float tolerance, std::vector<PointIndices> &clusters,
                               unsigned int min_pts_per_cluster,
                               unsigned int max_pts_per_cluster)
{
  auto noop = [&](__attribute__((unused)) int i, __attribute__((unused)) int j, __attribute__((unused)) const Indices& nn_indices) {
    return true;
  };
  Indices indices;
  pcl::extractEuclideanClusters<PointT, decltype(noop)>(cloud, indices, noop, tree, clusters, min_pts_per_cluster, max_pts_per_cluster);
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
  // \note If the tree was created over <cloud, indices>, we guarantee a 1-1 mapping between what the tree returns
  //and indices[i]
  if (tree->getIndices ()->size () != indices.size ())
  {
    PCL_ERROR ("[pcl::extractEuclideanClusters] Tree built for a different set of indices (%lu) than the input set (%lu)!\n", tree->getIndices ()->size (), indices.size ());
    return;
  }

  if (indices.empty())
    return;

  auto noop = [&](__attribute__((unused)) int i, __attribute__((unused)) int j, __attribute__((unused)) const Indices& nn_indices) {
    return true;
  };
  pcl::extractEuclideanClusters(cloud, indices, noop, tree, tolerance, clusters, min_pts_per_cluster, max_pts_per_cluster);
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
