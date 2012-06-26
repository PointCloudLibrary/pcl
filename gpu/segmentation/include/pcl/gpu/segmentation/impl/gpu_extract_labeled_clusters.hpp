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
 * $Id:$
 *
 */

#ifndef PCL_GPU_SEGMENTATION_IMPL_EXTRACT_LABELED_CLUSTERS_H_
#define PCL_GPU_SEGMENTATION_IMPL_EXTRACT_LABELED_CLUSTERS_H_

#include <pcl/gpu/segmentation/gpu_extract_labeled_clusters.h>

template <typename PointT> void
pcl::gpu::extractLabeledEuclideanClusters (const boost::shared_ptr<pcl::PointCloud<PointT> >  &host_cloud_,
                                           const pcl::gpu::Octree::Ptr                                   &tree,
                                           float                                                         tolerance,
                                           std::vector<PointIndices>                                     &clusters,
                                           unsigned int                                                  min_pts_per_cluster,
                                           unsigned int                                                  max_pts_per_cluster)
{

  // Create a bool vector of processed point indices, and initialize it to false
  // cloud is a DeviceArray<PointType>
  std::vector<bool> processed (host_cloud_->points.size (), false);

  int max_answers;

  if(max_pts_per_cluster > host_cloud_->points.size ())
    max_answers = static_cast<int> (host_cloud_->points.size ());
  else
    max_answers = max_pts_per_cluster;

  // to store the current cluster
  pcl::PointIndices r;

  // Process all points in the cloud
  for (size_t i = 0; i < host_cloud_->points.size (); ++i)
  {
    // if we already processed this point continue with the next one
    if (processed[i])
      continue;
    // now we will process this point
    processed[i] = true;

    // Create the query queue on the device, point based not indices
    pcl::gpu::Octree::Queries queries_device;
    // Create the query queue on the host
    pcl::PointCloud<pcl::PointXYZ>::VectorType queries_host;

    // Buffer in a new PointXYZ type
    PointT t = host_cloud_->points[i];
    PointXYZ p;
    p.x = t.x; p.y = t.y; p.z = t.z;

    // Push the starting point in the vector
    queries_host.push_back (p);
    // Clear vector
    r.indices.clear ();
    // Push the starting point in
    r.indices.push_back (static_cast<int> (i));

    unsigned int found_points = static_cast<unsigned int> (queries_host.size ());
    unsigned int previous_found_points = 0;

    pcl::gpu::NeighborIndices result_device;

    // once the area stop growing, stop also iterating.
    while (previous_found_points < found_points)
    {
      // Move queries to GPU
      queries_device.upload(queries_host);
      // Execute search
      tree->radiusSearch(queries_device, tolerance, max_answers, result_device);

      // Store the previously found number of points
      previous_found_points = found_points;

      // Host buffer for results
      std::vector<int> sizes, data;

      // Copy results from GPU to Host
      result_device.sizes.download (sizes);
      result_device.data.download (data);

      for(size_t qp = 0; qp < sizes.size (); qp++)
      {
        for(int qp_r = 0; qp_r < sizes[qp]; qp_r++)
        {
          if(processed[data[qp_r + qp * max_answers]])
            continue;
          // Only add if label matches the original label
          if(host_cloud_->points[i].label == host_cloud_->points[data[qp_r + qp * max_answers]].label)
          {
            processed[data[qp_r + qp * max_answers]] = true;
            PointT t_l = host_cloud_->points[data[qp_r + qp * max_answers]];
            PointXYZ p_l;
            p_l.x = t_l.x; p_l.y = t_l.y; p_l.z = t_l.z;
            queries_host.push_back (p_l);
            found_points++;
            r.indices.push_back(data[qp_r + qp * max_answers]);
          }
        }
      }
    }
    // If this queue is satisfactory, add to the clusters
    if (found_points >= min_pts_per_cluster && found_points <= max_pts_per_cluster)
    {
      std::sort (r.indices.begin (), r.indices.end ());
      // @todo: check if the following is actually still needed
      //r.indices.erase (std::unique (r.indices.begin (), r.indices.end ()), r.indices.end ());

      r.header = host_cloud_->header;
      clusters.push_back (r);   // We could avoid a copy by working directly in the vector
    }
  }
}

template <typename PointT> void 
pcl::gpu::EuclideanLabeledClusterExtraction<PointT>::extract (std::vector<PointIndices> &clusters)
{
  // Initialize the GPU search tree
  if (!tree_)
  {
    tree_.reset (new pcl::gpu::Octree());
    ///@todo what do we do if input isn't a PointXYZ cloud?
    tree_->setCloud(input_);
  }
  if (!tree_->isBuilt())
  {
    tree_->build();
  }
/*
  if(tree_->cloud_.size() != host_cloud.points.size ())
  {
    PCL_ERROR("[pcl::gpu::EuclideanClusterExtraction] size of host cloud and device cloud don't match!\n");
    return;
  }
*/
  // Extract the actual clusters
  extractLabeledEuclideanClusters (host_cloud_, tree_, cluster_tolerance_, clusters, min_pts_per_cluster_, max_pts_per_cluster_);

  // Sort the clusters based on their size (largest one first)
  std::sort (clusters.rbegin (), clusters.rend (), compareLabeledPointClusters);
}

#define PCL_INSTANTIATE_extractLabeledEuclideanClusters(T) template void PCL_EXPORTS pcl::gpu::extractLabeledEuclideanClusters (const boost::shared_ptr<pcl::PointCloud<T> >  &, const pcl::gpu::Octree::Ptr &,float, std::vector<PointIndices> &, unsigned int, unsigned int);
#define PCL_INSTANTIATE_EuclideanLabeledClusterExtraction(T) template class PCL_EXPORTS pcl::gpu::EuclideanLabeledClusterExtraction<T>;
/*
#define PCL_INSTANTIATE_extractLabeledEuclideanClusters(T) template void PCL_EXPORTS pcl::gpu::extractLabeledEuclideanClusters (const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBL> >  &, const pcl::gpu::Octree::Ptr &,float, std::vector<PointIndices> &, unsigned int, unsigned int);
#define PCL_INSTANTIATE_EuclideanLabeledClusterExtraction(T) template class PCL_EXPORTS pcl::gpu::EuclideanLabeledClusterExtraction<T>;
*/
#endif //PCL_GPU_SEGMENTATION_IMPL_EXTRACT_LABELED_CLUSTERS_H_
