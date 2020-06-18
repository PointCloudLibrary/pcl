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
 * $id $
 */

#ifndef PCL_SEGMENTATION_IMPL_EXTRACT_LABELED_CLUSTERS_H_
#define PCL_SEGMENTATION_IMPL_EXTRACT_LABELED_CLUSTERS_H_

#include <pcl/segmentation/extract_labeled_clusters.h>

namespace impl
{
  // convert labeled_cluster_map_t
  void
  labeled_cluster_map_to_labeled_cluster_vector( const pcl::PCLHeader& header, pcl::labeled_cluster_map_t labeled_map, std::vector<std::vector<pcl::PointIndices>>& vec )
  {
    if ( labeled_map.empty() )
      return;

    // get max label
    const auto extracted_max_label = std::max_element( labeled_map.begin(), labeled_map.end(), 
      []( const pcl::labeled_cluster_map_t::value_type& lhs, const pcl::labeled_cluster_map_t::value_type& rhs  ) { return lhs.first < rhs.first; }  
      )
      ->first;

    // if the user didn't provide a preallocated vector, create it for them based on max label
    if ( vec.empty() )
      vec.resize(extracted_max_label+1);
    else if ( vec.size() <= extracted_max_label )
    {
      PCL_ERROR ("[pcl::extractLabeledEuclideanClusters] labeled_clusters size (%lu) less than extracted max_label (%lu)!\n", vec.size (), (unsigned int)extracted_max_label );
      return;
    }
    
    // move clusters to output vector
    //  convert pcl::Indices to pcl:PointIndices
    for ( auto& label_group : labeled_map )
    {
      for ( auto& cluster : label_group.second )  // cluster is pcl::Indices
      {
        pcl::PointIndices pi = {};
        pi.header = header;
        pi.indices = std::move(cluster);
        vec[label_group.first].emplace_back(std::move(pi));
      }
    }

  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::extractLabeledEuclideanClusters (const PointCloud<PointT> &cloud,
                                      const typename search::Search<PointT>::Ptr &tree,
                                      float tolerance,
                                      labeled_cluster_map_t &labeled_clusters,
                                      unsigned int min_pts_per_cluster,
                                      unsigned int max_pts_per_cluster,
                                      unsigned int max_labels )
{
  assert(tree); // ensure we have a tree

  if (tree->getInputCloud ()->size () != cloud.size ())
  {
    PCL_ERROR ("[pcl::extractLabeledEuclideanClusters] Tree built for a different point cloud dataset (%lu) than the input cloud (%lu)!\n", tree->getInputCloud ()->size (), cloud.size ());
    return;
  }

  // Create a bool vector of processed point indices, and initialize it to false
  std::vector<bool> processed (cloud.size (), false);

  // Process all points in the cloud
  for ( index_t i = 0; i < static_cast<index_t>(cloud.size()); ++i)
  {
    if (processed[i])
      continue;

    processed[i] = true;

    // label for the current point
    const auto current_label = cloud[i].label;

    // iterator in labeled_clusters for the current label
    const auto current_label_cluster_it = labeled_clusters.find( current_label );

    // max labels check
    //  if we would have to add a new label and we're already at max, no need to proceed
    if ( current_label_cluster_it == labeled_clusters.end() && labeled_clusters.size() >= max_labels )
      continue;

    pcl::Indices seed_queue = {i};

    index_t sq_idx = 0;
    while (sq_idx < static_cast<index_t> (seed_queue.size ()))
    {
      // Search for sq_idx
      pcl::Indices nn_indices = {};
      std::vector<float> nn_distances = {};

      const int ret = tree->radiusSearch (seed_queue[sq_idx], tolerance, nn_indices, nn_distances, max_pts_per_cluster );
      if(ret == -1)
        PCL_ERROR("radiusSearch on tree came back with error -1");
        
      if (!ret)
      {
        sq_idx++;
        continue;
      }

      for (std::size_t j = 1; j < nn_indices.size (); ++j)             // nn_indices[0] should be sq_idx
      {
        // labels match, and nn point not processed before
        if ( !processed[nn_indices[j]] && current_label == cloud[nn_indices[j]].label )
        {
          seed_queue.push_back ( nn_indices[j] ) ;
          processed[nn_indices[j]] = true;
        }
      }

      sq_idx++;
    }

    // If this queue is satisfactory, add to the clusters
    if (seed_queue.size () >= min_pts_per_cluster && seed_queue.size () <= max_pts_per_cluster)
    { 
      if ( current_label_cluster_it == labeled_clusters.end() )  // label not found in map, add
      {
        std::vector<pcl::Indices> v = {};
        v.emplace_back(std::move(seed_queue));
        labeled_clusters.emplace(current_label, std::move(v));
      }
      else  // label already exists in map; append seed_queue to label vector
        current_label_cluster_it->second.emplace_back(std::move(seed_queue));

    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::extractLabeledEuclideanClusters (const PointCloud<PointT> &cloud,
                                      const typename search::Search<PointT>::Ptr &tree,
                                      float tolerance,
                                      std::vector<std::vector<PointIndices> > &labeled_clusters,
                                      unsigned int min_pts_per_cluster,
                                      unsigned int max_pts_per_cluster,
                                      unsigned int max_label )
{
  labeled_cluster_map_t label_map = {};
  extractLabeledEuclideanClusters( cloud, tree, tolerance, label_map, min_pts_per_cluster, max_pts_per_cluster, max_label );

  impl::labeled_cluster_map_to_labeled_cluster_vector( cloud.header, std::move(label_map), labeled_clusters );
}

//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////

template <typename PointT> void 
pcl::LabeledEuclideanClusterExtraction<PointT>::extract (std::vector<std::vector<PointIndices> > &labeled_clusters)
{
  labeled_cluster_map_t label_map = {};
  extract( label_map );
  impl::labeled_cluster_map_to_labeled_cluster_vector( input_->header, std::move(label_map), labeled_clusters );

  // Sort the clusters based on their size (largest one first)
  for (auto &labeled_cluster : labeled_clusters)
    std::sort (labeled_cluster.rbegin (), labeled_cluster.rend (), comparePointClusters);
}

template <typename PointT> void 
pcl::LabeledEuclideanClusterExtraction<PointT>::extract ( pcl::labeled_cluster_map_t &labeled_clusters)
{
  if (!initCompute () || 
      (input_   && input_->points.empty ()) ||
      (indices_ && indices_->empty ()))
  {
    labeled_clusters.clear ();
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
  tree_->setInputCloud (input_);
  extractLabeledEuclideanClusters (*input_, tree_, static_cast<float> (cluster_tolerance_), labeled_clusters, min_pts_per_cluster_, max_pts_per_cluster_, max_label_);

  deinitCompute ();
}

#define PCL_INSTANTIATE_LabeledEuclideanClusterExtraction(T) template class PCL_EXPORTS pcl::LabeledEuclideanClusterExtraction<T>;
#define PCL_INSTANTIATE_extractLabeledEuclideanClusters(T) template void PCL_EXPORTS pcl::extractLabeledEuclideanClusters<T>(const pcl::PointCloud<T> &, const typename pcl::search::Search<T>::Ptr &, float , std::vector<std::vector<pcl::PointIndices> > &, unsigned int, unsigned int, unsigned int);
#define PCL_INSTANTIATE_extractLabeledEuclideanClustersMap(T) template void PCL_EXPORTS pcl::extractLabeledEuclideanClusters<T>(const pcl::PointCloud<T> &, const typename pcl::search::Search<T>::Ptr &, float , pcl::labeled_cluster_map_t &, unsigned int, unsigned int, unsigned int);

#endif        // PCL_EXTRACT_CLUSTERS_IMPL_H_
