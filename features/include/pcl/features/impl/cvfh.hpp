/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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
 * $Id$
 *
 */

#ifndef PCL_FEATURES_IMPL_CVFH_H_
#define PCL_FEATURES_IMPL_CVFH_H_

#include <pcl/features/cvfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/pfh_tools.h>
#include <pcl/common/centroid.h>

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT, typename PointNT, typename PointOutT> void
pcl::CVFHEstimation<PointInT, PointNT, PointOutT>::compute (PointCloudOut &output)
{
  if (!Feature<PointInT, PointOutT>::initCompute ())
  {
    output.width = output.height = 0;
    output.points.clear ();
    return;
  }
  // Resize the output dataset
  // Important! We should only allocate precisely how many elements we will need, otherwise
  // we risk at pre-allocating too much memory which could lead to bad_alloc 
  // (see http://dev.pointclouds.org/issues/657)
  output.width = output.height = 1;
  output.points.resize (1);

  // Perform the actual feature computation
  computeFeature (output);

  Feature<PointInT, PointOutT>::deinitCompute ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT, typename PointNT, typename PointOutT> void
pcl::CVFHEstimation<PointInT, PointNT, PointOutT>::extractEuclideanClustersSmooth (
    const pcl::PointCloud<pcl::PointNormal> &cloud,
    const pcl::PointCloud<pcl::PointNormal> &normals,
    float tolerance,
    const pcl::search::Search<pcl::PointNormal>::Ptr &tree,
    std::vector<pcl::PointIndices> &clusters,
    double eps_angle,
    unsigned int min_pts_per_cluster,
    unsigned int max_pts_per_cluster)
{
  if (tree->getInputCloud ()->points.size () != cloud.points.size ())
  {
    PCL_ERROR ("[pcl::extractEuclideanClusters] Tree built for a different point cloud dataset (%lu) than the input cloud (%lu)!\n", tree->getInputCloud ()->points.size (), cloud.points.size ());
    return;
  }
  if (cloud.points.size () != normals.points.size ())
  {
    PCL_ERROR ("[pcl::extractEuclideanClusters] Number of points in the input point cloud (%lu) different than normals (%lu)!\n", cloud.points.size (), normals.points.size ());
    return;
  }

  // Create a bool vector of processed point indices, and initialize it to false
  std::vector<bool> processed (cloud.points.size (), false);

  std::vector<int> nn_indices;
  std::vector<float> nn_distances;
  // Process all points in the indices vector
  for (int i = 0; i < static_cast<int> (cloud.points.size ()); ++i)
  {
    if (processed[i])
      continue;

    std::vector<unsigned int> seed_queue;
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

      for (size_t j = 1; j < nn_indices.size (); ++j) // nn_indices[0] should be sq_idx
      {
        if (processed[nn_indices[j]]) // Has this point been processed before ?
          continue;

        //processed[nn_indices[j]] = true;
        // [-1;1]

        double dot_p = normals.points[seed_queue[sq_idx]].normal[0] * normals.points[nn_indices[j]].normal[0]
                     + normals.points[seed_queue[sq_idx]].normal[1] * normals.points[nn_indices[j]].normal[1]
                     + normals.points[seed_queue[sq_idx]].normal[2] * normals.points[nn_indices[j]].normal[2];

        if (fabs (acos (dot_p)) < eps_angle)
        {
          processed[nn_indices[j]] = true;
          seed_queue.push_back (nn_indices[j]);
        }
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

      std::sort (r.indices.begin (), r.indices.end ());
      r.indices.erase (std::unique (r.indices.begin (), r.indices.end ()), r.indices.end ());

      r.header = cloud.header;
      clusters.push_back (r); // We could avoid a copy by working directly in the vector
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT, typename PointNT, typename PointOutT> void
pcl::CVFHEstimation<PointInT, PointNT, PointOutT>::filterNormalsWithHighCurvature (
    const pcl::PointCloud<PointNT> & cloud,
    std::vector<int> &indices_to_use,
    std::vector<int> &indices_out,
    std::vector<int> &indices_in,
    float threshold)
{
  indices_out.resize (cloud.points.size ());
  indices_in.resize (cloud.points.size ());

  size_t in, out;
  in = out = 0;

  for (int i = 0; i < static_cast<int> (indices_to_use.size ()); i++)
  {
    if (cloud.points[indices_to_use[i]].curvature > threshold)
    {
      indices_out[out] = indices_to_use[i];
      out++;
    }
    else
    {
      indices_in[in] = indices_to_use[i];
      in++;
    }
  }

  indices_out.resize (out);
  indices_in.resize (in);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT, typename PointNT, typename PointOutT> void
pcl::CVFHEstimation<PointInT, PointNT, PointOutT>::computeFeature (PointCloudOut &output)
{
  // Check if input was set
  if (!normals_)
  {
    PCL_ERROR ("[pcl::%s::computeFeature] No input dataset containing normals was given!\n", getClassName ().c_str ());
    output.width = output.height = 0;
    output.points.clear ();
    return;
  }
  if (normals_->points.size () != surface_->points.size ())
  {
    PCL_ERROR ("[pcl::%s::computeFeature] The number of points in the input dataset differs from the number of points in the dataset containing the normals!\n", getClassName ().c_str ());
    output.width = output.height = 0;
    output.points.clear ();
    return;
  }

  centroids_dominant_orientations_.clear ();

  // ---[ Step 0: remove normals with high curvature
  std::vector<int> indices_out;
  std::vector<int> indices_in;
  filterNormalsWithHighCurvature (*normals_, *indices_, indices_out, indices_in, curv_threshold_);

  pcl::PointCloud<pcl::PointNormal>::Ptr normals_filtered_cloud (new pcl::PointCloud<pcl::PointNormal> ());
  normals_filtered_cloud->width = static_cast<uint32_t> (indices_in.size ());
  normals_filtered_cloud->height = 1;
  normals_filtered_cloud->points.resize (normals_filtered_cloud->width);

  for (size_t i = 0; i < indices_in.size (); ++i)
  {
    normals_filtered_cloud->points[i].x = surface_->points[indices_in[i]].x;
    normals_filtered_cloud->points[i].y = surface_->points[indices_in[i]].y;
    normals_filtered_cloud->points[i].z = surface_->points[indices_in[i]].z;
  }

  std::vector<pcl::PointIndices> clusters;

  if(normals_filtered_cloud->points.size() >= min_points_)
  {
    //recompute normals and use them for clustering
    KdTreePtr normals_tree_filtered (new pcl::search::KdTree<pcl::PointNormal> (false));
    normals_tree_filtered->setInputCloud (normals_filtered_cloud);


    pcl::NormalEstimation<PointNormal, PointNormal> n3d;
    n3d.setRadiusSearch (radius_normals_);
    n3d.setSearchMethod (normals_tree_filtered);
    n3d.setInputCloud (normals_filtered_cloud);
    n3d.compute (*normals_filtered_cloud);

    KdTreePtr normals_tree (new pcl::search::KdTree<pcl::PointNormal> (false));
    normals_tree->setInputCloud (normals_filtered_cloud);

    extractEuclideanClustersSmooth (*normals_filtered_cloud,
                                    *normals_filtered_cloud,
                                    cluster_tolerance_,
                                    normals_tree,
                                    clusters,
                                    eps_angle_threshold_,
                                    static_cast<unsigned int> (min_points_));

  }

  VFHEstimator vfh;
  vfh.setInputCloud (surface_);
  vfh.setInputNormals (normals_);
  vfh.setIndices(indices_);
  vfh.setSearchMethod (this->tree_);
  vfh.setUseGivenNormal (true);
  vfh.setUseGivenCentroid (true);
  vfh.setNormalizeBins (normalize_bins_);
  vfh.setNormalizeDistance (true);
  vfh.setFillSizeComponent (true);
  output.height = 1;

  // ---[ Step 1b : check if any dominant cluster was found
  if (clusters.size () > 0)
  { // ---[ Step 1b.1 : If yes, compute CVFH using the cluster information

    for (size_t i = 0; i < clusters.size (); ++i) //for each cluster
    {
      Eigen::Vector4f avg_normal = Eigen::Vector4f::Zero ();
      Eigen::Vector4f avg_centroid = Eigen::Vector4f::Zero ();

      for (size_t j = 0; j < clusters[i].indices.size (); j++)
      {
        avg_normal += normals_filtered_cloud->points[clusters[i].indices[j]].getNormalVector4fMap ();
        avg_centroid += normals_filtered_cloud->points[clusters[i].indices[j]].getVector4fMap ();
      }

      avg_normal /= static_cast<float> (clusters[i].indices.size ());
      avg_centroid /= static_cast<float> (clusters[i].indices.size ());

      Eigen::Vector4f centroid_test;
      pcl::compute3DCentroid (*normals_filtered_cloud, centroid_test);
      avg_normal.normalize ();

      Eigen::Vector3f avg_norm (avg_normal[0], avg_normal[1], avg_normal[2]);
      Eigen::Vector3f avg_dominant_centroid (avg_centroid[0], avg_centroid[1], avg_centroid[2]);

      //append normal and centroid for the clusters
      dominant_normals_.push_back (avg_norm);
      centroids_dominant_orientations_.push_back (avg_dominant_centroid);
    }

    //compute modified VFH for all dominant clusters and add them to the list!
    output.points.resize (dominant_normals_.size ());
    output.width = static_cast<uint32_t> (dominant_normals_.size ());

    for (size_t i = 0; i < dominant_normals_.size (); ++i)
    {
      //configure VFH computation for CVFH
      vfh.setNormalToUse (dominant_normals_[i]);
      vfh.setCentroidToUse (centroids_dominant_orientations_[i]);
      pcl::PointCloud<pcl::VFHSignature308> vfh_signature;
      vfh.compute (vfh_signature);
      output.points[i] = vfh_signature.points[0];
    }
  }
  else
  { // ---[ Step 1b.1 : If no, compute CVFH using all the object points
    Eigen::Vector4f avg_centroid;
    pcl::compute3DCentroid (*surface_, avg_centroid);
    Eigen::Vector3f cloud_centroid (avg_centroid[0], avg_centroid[1], avg_centroid[2]);
    centroids_dominant_orientations_.push_back (cloud_centroid);

    //configure VFH computation for CVFH using all object points
    vfh.setCentroidToUse (cloud_centroid);
    vfh.setUseGivenNormal (false);

    pcl::PointCloud<pcl::VFHSignature308> vfh_signature;
    vfh.compute (vfh_signature);

    output.points.resize (1);
    output.width = 1;

    output.points[0] = vfh_signature.points[0];
  }
}

#define PCL_INSTANTIATE_CVFHEstimation(T,NT,OutT) template class PCL_EXPORTS pcl::CVFHEstimation<T,NT,OutT>;

#endif    // PCL_FEATURES_IMPL_VFH_H_ 
