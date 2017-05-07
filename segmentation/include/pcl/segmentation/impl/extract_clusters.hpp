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
                               const boost::shared_ptr<search::Search<PointT> > &tree,
                               float tolerance, std::vector<PointIndices> &clusters,
                               unsigned int min_pts_per_cluster, 
                               unsigned int max_pts_per_cluster)
{
  if (tree->getInputCloud ()->points.size () != cloud.points.size ())
  {
    PCL_ERROR ("[pcl::extractEuclideanClusters] Tree built for a different point cloud dataset (%lu) than the input cloud (%lu)!\n", tree->getInputCloud ()->points.size (), cloud.points.size ());
    return;
  }
  // Check if the tree is sorted -- if it is we don't need to check the first element
  int nn_start_idx = tree->getSortedResults () ? 1 : 0;
  // Create a bool vector of processed point indices, and initialize it to false
  std::vector<bool> processed (cloud.points.size (), false);

  std::vector<int> nn_indices;
  std::vector<float> nn_distances;
  // Process all points in the indices vector
  for (int i = 0; i < static_cast<int> (cloud.points.size ()); ++i)
  {
    if (processed[i])
      continue;

    std::vector<int> seed_queue;
    int sq_idx = 0;
    seed_queue.push_back (i);

    processed[i] = true;

    while (sq_idx < static_cast<int> (seed_queue.size ()))
    {
      // Search for sq_idx
      if (!tree->radiusSearch (seed_queue[sq_idx], tolerance, nn_indices, nn_distances))
      {
        sq_idx++;
        continue;
      }

      for (size_t j = nn_start_idx; j < nn_indices.size (); ++j)             // can't assume sorted (default isn't!)
      {
        if (nn_indices[j] == -1 || processed[nn_indices[j]])        // Has this point been processed before ?
          continue;

        // Perform a simple Euclidean clustering
        seed_queue.push_back (nn_indices[j]);
        processed[nn_indices[j]] = true;
      }

      sq_idx++;
    }

    // If this queue is satisfactory, add to the clusters
    if (seed_queue.size () >= min_pts_per_cluster && seed_queue.size () <= max_pts_per_cluster)
    {
      pcl::PointIndices r;
      r.indices.resize (seed_queue.size ());
      for (size_t j = 0; j < seed_queue.size (); ++j)
        r.indices[j] = seed_queue[j];

      // These two lines should not be needed: (can anyone confirm?) -FF
      std::sort (r.indices.begin (), r.indices.end ());
      r.indices.erase (std::unique (r.indices.begin (), r.indices.end ()), r.indices.end ());

      r.header = cloud.header;
      clusters.push_back (r);   // We could avoid a copy by working directly in the vector
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
/** @todo: fix the return value, make sure the exit is not needed anymore*/
template <typename PointT> void
pcl::extractEuclideanClusters (const PointCloud<PointT> &cloud, 
                               const std::vector<int> &indices,
                               const boost::shared_ptr<search::Search<PointT> > &tree,
                               float tolerance, std::vector<PointIndices> &clusters,
                               unsigned int min_pts_per_cluster, 
                               unsigned int max_pts_per_cluster)
{
  // \note If the tree was created over <cloud, indices>, we guarantee a 1-1 mapping between what the tree returns
  //and indices[i]
  if (tree->getInputCloud ()->points.size () != cloud.points.size ())
  {
    PCL_ERROR ("[pcl::extractEuclideanClusters] Tree built for a different point cloud dataset (%lu) than the input cloud (%lu)!\n", tree->getInputCloud ()->points.size (), cloud.points.size ());
    return;
  }
  if (tree->getIndices ()->size () != indices.size ())
  {
    PCL_ERROR ("[pcl::extractEuclideanClusters] Tree built for a different set of indices (%lu) than the input set (%lu)!\n", tree->getIndices ()->size (), indices.size ());
    return;
  }
  // Check if the tree is sorted -- if it is we don't need to check the first element
  int nn_start_idx = tree->getSortedResults () ? 1 : 0;

  // Create a bool vector of processed point indices, and initialize it to false
  std::vector<bool> processed (cloud.points.size (), false);

  std::vector<int> nn_indices;
  std::vector<float> nn_distances;
  // Process all points in the indices vector
  for (int i = 0; i < static_cast<int> (indices.size ()); ++i)
  {
    if (processed[indices[i]])
      continue;

    std::vector<int> seed_queue;
    int sq_idx = 0;
    seed_queue.push_back (indices[i]);

    processed[indices[i]] = true;

    while (sq_idx < static_cast<int> (seed_queue.size ()))
    {
      // Search for sq_idx
      int ret = tree->radiusSearch (cloud.points[seed_queue[sq_idx]], tolerance, nn_indices, nn_distances);
      if( ret == -1)
      {
        PCL_ERROR("[pcl::extractEuclideanClusters] Received error code -1 from radiusSearch\n");
        exit(0);
      }
      if (!ret)
      {
        sq_idx++;
        continue;
      }

      for (size_t j = nn_start_idx; j < nn_indices.size (); ++j)             // can't assume sorted (default isn't!)
      {
        if (nn_indices[j] == -1 || processed[nn_indices[j]])        // Has this point been processed before ?
          continue;

        // Perform a simple Euclidean clustering
        seed_queue.push_back (nn_indices[j]);
        processed[nn_indices[j]] = true;
      }

      sq_idx++;
    }

    // If this queue is satisfactory, add to the clusters
    if (seed_queue.size () >= min_pts_per_cluster && seed_queue.size () <= max_pts_per_cluster)
    {
      pcl::PointIndices r;
      r.indices.resize (seed_queue.size ());
      for (size_t j = 0; j < seed_queue.size (); ++j)
        // This is the only place where indices come into play
        r.indices[j] = seed_queue[j];

      // These two lines should not be needed: (can anyone confirm?) -FF
      //r.indices.assign(seed_queue.begin(), seed_queue.end());
      std::sort (r.indices.begin (), r.indices.end ());
      r.indices.erase (std::unique (r.indices.begin (), r.indices.end ()), r.indices.end ());

      r.header = cloud.header;
      clusters.push_back (r);   // We could avoid a copy by working directly in the vector
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////

template <typename PointT> void 
pcl::EuclideanClusterExtraction<PointT>::extract (std::vector<PointIndices> &clusters)
{
  if (!initCompute () || 
      (input_ != 0   && input_->points.empty ()) ||
      (indices_ != 0 && indices_->empty ()))
  {
		clusters_.clear ();
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

	clusters_.resize (clusters.size ());
  std::vector<pcl::PointIndices>::iterator cluster_iter_input = clusters_.begin ();
  for (std::vector<pcl::PointIndices>::const_iterator cluster_iter = clusters.begin (); cluster_iter != clusters.end (); cluster_iter++)
  {
    if ((static_cast<int> (cluster_iter->indices.size ()) >= min_pts_per_cluster_) &&
        (static_cast<int> (cluster_iter->indices.size ()) <= max_pts_per_cluster_))
    {
      *cluster_iter_input = *cluster_iter;
      cluster_iter_input++;
    }
  }

  clusters = std::vector<pcl::PointIndices> (clusters_.begin (), cluster_iter_input);
  clusters_.resize(clusters.size());

  deinitCompute ();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> pcl::PointCloud<pcl::PointXYZRGB>::Ptr
pcl::EuclideanClusterExtraction<PointT>::getColoredCloud ()
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud;

  if (!clusters_.empty ())
  {
    colored_cloud = (new pcl::PointCloud<pcl::PointXYZRGB>)->makeShared ();

    srand (static_cast<unsigned int> (time (0)));
    std::vector<unsigned char> colors;
    for (size_t i_segment = 0; i_segment < clusters_.size (); i_segment++)
    {
      colors.push_back (static_cast<unsigned char> (rand () % 256));
      colors.push_back (static_cast<unsigned char> (rand () % 256));
      colors.push_back (static_cast<unsigned char> (rand () % 256));
    }

    colored_cloud->width = input_->width;
    colored_cloud->height = input_->height;
    colored_cloud->is_dense = input_->is_dense;
    for (size_t i_point = 0; i_point < input_->points.size (); i_point++)
    {
      pcl::PointXYZRGB point;
      point.x = *(input_->points[i_point].data);
      point.y = *(input_->points[i_point].data + 1);
      point.z = *(input_->points[i_point].data + 2);
      point.r = 255;
      point.g = 0;
      point.b = 0;
      colored_cloud->points.push_back (point);
    }

    std::vector< pcl::PointIndices >::iterator i_segment;
    int next_color = 0;
    for (i_segment = clusters_.begin (); i_segment != clusters_.end (); i_segment++)
    {
      std::vector<int>::iterator i_point;
      for (i_point = i_segment->indices.begin (); i_point != i_segment->indices.end (); i_point++)
      {
        int index;
        index = *i_point;
        colored_cloud->points[index].r = colors[3 * next_color];
        colored_cloud->points[index].g = colors[3 * next_color + 1];
        colored_cloud->points[index].b = colors[3 * next_color + 2];
      }
      next_color++;
    }
  }

  return (colored_cloud);
}

#define PCL_INSTANTIATE_EuclideanClusterExtraction(T) template class PCL_EXPORTS pcl::EuclideanClusterExtraction<T>;
#define PCL_INSTANTIATE_extractEuclideanClusters(T) template void PCL_EXPORTS pcl::extractEuclideanClusters<T>(const pcl::PointCloud<T> &, const boost::shared_ptr<pcl::search::Search<T> > &, float , std::vector<pcl::PointIndices> &, unsigned int, unsigned int);
#define PCL_INSTANTIATE_extractEuclideanClusters_indices(T) template void PCL_EXPORTS pcl::extractEuclideanClusters<T>(const pcl::PointCloud<T> &, const std::vector<int> &, const boost::shared_ptr<pcl::search::Search<T> > &, float , std::vector<pcl::PointIndices> &, unsigned int, unsigned int);

#endif        // PCL_EXTRACT_CLUSTERS_IMPL_H_
