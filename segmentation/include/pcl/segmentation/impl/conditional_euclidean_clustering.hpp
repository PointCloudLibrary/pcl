/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
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
 */

#ifndef PCL_SEGMENTATION_IMPL_CONDITIONAL_EUCLIDEAN_CLUSTERING_HPP_
#define PCL_SEGMENTATION_IMPL_CONDITIONAL_EUCLIDEAN_CLUSTERING_HPP_

#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/search/organized.h> // for OrganizedNeighbor
#include <pcl/search/kdtree.h> // for KdTree

template<typename PointT> void
pcl::ConditionalEuclideanClustering<PointT>::segment (pcl::IndicesClusters &clusters)
{
  // Prepare output (going to use push_back)
  clusters.clear ();
  if (extract_removed_clusters_)
  {
    small_clusters_->clear ();
    large_clusters_->clear ();
  }

  // Validity checks
  if (!initCompute () || input_->points.empty () || indices_->empty () || !condition_function_)
    return;

  // Initialize the search class
  if (!searcher_)
  {
    if (input_->isOrganized ())
      searcher_.reset (new pcl::search::OrganizedNeighbor<PointT> ());
    else
      searcher_.reset (new pcl::search::KdTree<PointT> ());
  }
  searcher_->setInputCloud (input_, indices_);

  // Temp variables used by search class
  Indices nn_indices;
  std::vector<float> nn_distances;

  // Create a bool vector of processed point indices, and initialize it to false
  // Need to have it contain all possible points because radius search can not return indices into indices
  std::vector<bool> processed (input_->size (), false);

  // Process all points indexed by indices_
  for (const auto& iindex : (*indices_)) // iindex = input index
  {
    // Has this point been processed before?
    if (iindex == UNAVAILABLE || processed[iindex])
      continue;

    // Set up a new growing cluster
    Indices current_cluster;
    int cii = 0;  // cii = cluster indices iterator

    // Add the point to the cluster
    current_cluster.push_back (iindex);
    processed[iindex] = true;

    // Process the current cluster (it can be growing in size as it is being processed)
    while (cii < static_cast<int> (current_cluster.size ()))
    {
      // Search for neighbors around the current seed point of the current cluster
      if (searcher_->radiusSearch ((*input_)[current_cluster[cii]], cluster_tolerance_, nn_indices, nn_distances) < 1)
      {
        cii++;
        continue;
      }

      // Process the neighbors
      for (int nii = 1; nii < static_cast<int> (nn_indices.size ()); ++nii)  // nii = neighbor indices iterator
      {
        // Has this point been processed before?
        if (nn_indices[nii] == UNAVAILABLE || processed[nn_indices[nii]])
          continue;

        // Validate if condition holds
        if (condition_function_ ((*input_)[current_cluster[cii]], (*input_)[nn_indices[nii]], nn_distances[nii]))
        {
          // Add the point to the cluster
          current_cluster.push_back (nn_indices[nii]);
          processed[nn_indices[nii]] = true;
        }
      }
      cii++;
    }

    // If extracting removed clusters, all clusters need to be saved, otherwise only the ones within the given cluster size range
    if (extract_removed_clusters_ ||
        (static_cast<int> (current_cluster.size ()) >= min_cluster_size_ &&
         static_cast<int> (current_cluster.size ()) <= max_cluster_size_))
    {
      pcl::PointIndices pi;
      pi.header = input_->header;
      pi.indices.resize (current_cluster.size ());
      for (int ii = 0; ii < static_cast<int> (current_cluster.size ()); ++ii)  // ii = indices iterator
        pi.indices[ii] = current_cluster[ii];

      if (extract_removed_clusters_ && static_cast<int> (current_cluster.size ()) < min_cluster_size_)
        small_clusters_->push_back (pi);
      else if (extract_removed_clusters_ && static_cast<int> (current_cluster.size ()) > max_cluster_size_)
        large_clusters_->push_back (pi);
      else
        clusters.push_back (pi);
    }
  }

  deinitCompute ();
}

#define PCL_INSTANTIATE_ConditionalEuclideanClustering(T) template class PCL_EXPORTS pcl::ConditionalEuclideanClustering<T>;

#endif  // PCL_SEGMENTATION_IMPL_CONDITIONAL_EUCLIDEAN_CLUSTERING_HPP_

