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
#include <pcl/common/centroid.h>

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT, typename PointNT, typename PointOutT> void
pcl::CVFHEstimation<PointInT, PointNT, PointOutT>::compute (PointCloudOut &output)
{
  if (!Feature<PointInT, PointOutT>::initCompute ())
  {
    output.width = output.height = 0;
    output.clear ();
    return;
  }
  // Resize the output dataset
  // Important! We should only allocate precisely how many elements we will need, otherwise
  // we risk at pre-allocating too much memory which could lead to bad_alloc 
  // (see http://dev.pointclouds.org/issues/657)
  output.width = output.height = 1;
  output.resize (1);

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
  if (tree->getInputCloud ()->size () != cloud.size ())
  {
    PCL_ERROR("[pcl::extractEuclideanClusters] Tree built for a different point cloud "
              "dataset (%zu) than the input cloud (%zu)!\n",
              static_cast<std::size_t>(tree->getInputCloud()->size()),
              static_cast<std::size_t>(cloud.size()));
    return;
  }
  if (cloud.size () != normals.size ())
  {
    PCL_ERROR("[pcl::extractEuclideanClusters] Number of points in the input point "
              "cloud (%zu) different than normals (%zu)!\n",
              static_cast<std::size_t>(cloud.size()),
              static_cast<std::size_t>(normals.size()));
    return;
  }

  // Create a bool vector of processed point indices, and initialize it to false
  std::vector<bool> processed (cloud.size (), false);

  pcl::Indices nn_indices;
  std::vector<float> nn_distances;
  // Process all points in the indices vector
  for (std::size_t i = 0; i < cloud.size (); ++i)
  {
    if (processed[i])
      continue;
    processed[i] = true;

    pcl::PointIndices r;
    r.header = cloud.header;
    auto& seed_queue = r.indices;

    seed_queue.push_back (i);

    // loop has an emplace_back, making it difficult to use modern loops
    for (std::size_t idx = 0; idx != seed_queue.size (); ++idx)
    {
      // Search for seed_queue[index]
      if (!tree->radiusSearch (seed_queue[idx], tolerance, nn_indices, nn_distances))
      {
        continue;
      }

      // skip index 0, since nn_indices[0] == idx, worth it?
      for (std::size_t j = 1; j < nn_indices.size (); ++j)
      {
        if (processed[nn_indices[j]]) // Has this point been processed before ?
          continue;

        //processed[nn_indices[j]] = true;
        // [-1;1]
        const double dot_p = normals[seed_queue[idx]].getNormalVector3fMap().dot(
                        normals[nn_indices[j]].getNormalVector3fMap());

        if (std::acos (dot_p) < eps_angle)
        {
          processed[nn_indices[j]] = true;
          seed_queue.emplace_back (nn_indices[j]);
        }
      }
    }

    // If this queue is satisfactory, add to the clusters
    if (seed_queue.size () >= min_pts_per_cluster && seed_queue.size () <= max_pts_per_cluster)
    {
      std::sort (r.indices.begin (), r.indices.end ());
      r.indices.erase (std::unique (r.indices.begin (), r.indices.end ()), r.indices.end ());

      // Might be better to work directly in the cluster somehow
      clusters.emplace_back (std::move(r)); // Trying to avoid a copy by moving
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT, typename PointNT, typename PointOutT> void
pcl::CVFHEstimation<PointInT, PointNT, PointOutT>::filterNormalsWithHighCurvature (
    const pcl::PointCloud<PointNT> & cloud,
    pcl::Indices &indices_to_use,
    pcl::Indices &indices_out,
    pcl::Indices &indices_in,
    float threshold)
{
  indices_out.resize (cloud.size ());
  indices_in.resize (cloud.size ());

  std::size_t in, out;
  in = out = 0;

  for (const auto &index : indices_to_use)
  {
    if (cloud[index].curvature > threshold)
    {
      indices_out[out] = index;
      out++;
    }
    else
    {
      indices_in[in] = index;
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
    output.clear ();
    return;
  }
  if (normals_->size () != surface_->size ())
  {
    PCL_ERROR ("[pcl::%s::computeFeature] The number of points in the input dataset differs from the number of points in the dataset containing the normals!\n", getClassName ().c_str ());
    output.width = output.height = 0;
    output.clear ();
    return;
  }

  centroids_dominant_orientations_.clear ();

  // ---[ Step 0: remove normals with high curvature
  pcl::Indices indices_out;
  pcl::Indices indices_in;
  filterNormalsWithHighCurvature (*normals_, *indices_, indices_out, indices_in, curv_threshold_);

  pcl::PointCloud<pcl::PointNormal>::Ptr normals_filtered_cloud (new pcl::PointCloud<pcl::PointNormal> ());
  normals_filtered_cloud->width = indices_in.size ();
  normals_filtered_cloud->height = 1;
  normals_filtered_cloud->points.resize (normals_filtered_cloud->width);

  for (std::size_t i = 0; i < indices_in.size (); ++i)
  {
    (*normals_filtered_cloud)[i].x = (*surface_)[indices_in[i]].x;
    (*normals_filtered_cloud)[i].y = (*surface_)[indices_in[i]].y;
    (*normals_filtered_cloud)[i].z = (*surface_)[indices_in[i]].z;
  }

  std::vector<pcl::PointIndices> clusters;

  if(normals_filtered_cloud->size() >= min_points_)
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
  if (!clusters.empty ())
  { // ---[ Step 1b.1 : If yes, compute CVFH using the cluster information

    for (const auto &cluster : clusters) //for each cluster
    {
      Eigen::Vector4f avg_normal = Eigen::Vector4f::Zero ();
      Eigen::Vector4f avg_centroid = Eigen::Vector4f::Zero ();

      for (const auto &index : cluster.indices)
      {
        avg_normal += (*normals_filtered_cloud)[index].getNormalVector4fMap ();
        avg_centroid += (*normals_filtered_cloud)[index].getVector4fMap ();
      }

      avg_normal /= static_cast<float> (cluster.indices.size ());
      avg_centroid /= static_cast<float> (cluster.indices.size ());

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
    output.resize (dominant_normals_.size ());
    output.width = dominant_normals_.size ();

    for (std::size_t i = 0; i < dominant_normals_.size (); ++i)
    {
      //configure VFH computation for CVFH
      vfh.setNormalToUse (dominant_normals_[i]);
      vfh.setCentroidToUse (centroids_dominant_orientations_[i]);
      pcl::PointCloud<pcl::VFHSignature308> vfh_signature;
      vfh.compute (vfh_signature);
      output[i] = vfh_signature[0];
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

    output.resize (1);
    output.width = 1;

    output[0] = vfh_signature[0];
  }
}

#define PCL_INSTANTIATE_CVFHEstimation(T,NT,OutT) template class PCL_EXPORTS pcl::CVFHEstimation<T,NT,OutT>;

#endif    // PCL_FEATURES_IMPL_VFH_H_ 
