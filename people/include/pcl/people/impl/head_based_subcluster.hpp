/*
 * Software License Agreement (BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2013-, Open Perception, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 * with the distribution.
 * * Neither the name of the copyright holder(s) nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * head_based_subcluster.hpp
 * Created on: Nov 30, 2012
 * Author: Matteo Munaro
 */

#ifndef PCL_PEOPLE_HEAD_BASED_SUBCLUSTER_HPP_
#define PCL_PEOPLE_HEAD_BASED_SUBCLUSTER_HPP_

#include <pcl/people/head_based_subcluster.h>

template <typename PointT>
pcl::people::HeadBasedSubclustering<PointT>::HeadBasedSubclustering ()
{
  // set default values for optional parameters:
  vertical_ = false;
  head_centroid_ = true;
  min_height_ = 1.3;
  max_height_ = 2.3;
  min_points_ = 30;
  max_points_ = 5000;
  heads_minimum_distance_ = 0.3;

  // set flag values for mandatory parameters:
  sqrt_ground_coeffs_ = std::numeric_limits<float>::quiet_NaN();
}

template <typename PointT> void
pcl::people::HeadBasedSubclustering<PointT>::setInputCloud (PointCloudPtr& cloud)
{
  cloud_ = cloud;
}

template <typename PointT> void
pcl::people::HeadBasedSubclustering<PointT>::setGround (Eigen::VectorXf& ground_coeffs)
{
  ground_coeffs_ = ground_coeffs;
  sqrt_ground_coeffs_ = (ground_coeffs - Eigen::Vector4f(0.0f, 0.0f, 0.0f, ground_coeffs(3))).norm();
}

template <typename PointT> void
pcl::people::HeadBasedSubclustering<PointT>::setInitialClusters (std::vector<pcl::PointIndices>& cluster_indices)
{
  cluster_indices_ = cluster_indices;
}

template <typename PointT> void
pcl::people::HeadBasedSubclustering<PointT>::setSensorPortraitOrientation (bool vertical)
{
  vertical_ = vertical;
}

template <typename PointT> void
pcl::people::HeadBasedSubclustering<PointT>::setHeightLimits (float min_height, float max_height)
{
  min_height_ = min_height;
  max_height_ = max_height;
}

template <typename PointT> void
pcl::people::HeadBasedSubclustering<PointT>::setDimensionLimits (int min_points, int max_points)
{
  min_points_ = min_points;
  max_points_ = max_points;
}

template <typename PointT> void
pcl::people::HeadBasedSubclustering<PointT>::setMinimumDistanceBetweenHeads (float heads_minimum_distance)
{
  heads_minimum_distance_= heads_minimum_distance;
}

template <typename PointT> void
pcl::people::HeadBasedSubclustering<PointT>::setHeadCentroid (bool head_centroid)
{
  head_centroid_ = head_centroid;
}

template <typename PointT> void
pcl::people::HeadBasedSubclustering<PointT>::getHeightLimits (float& min_height, float& max_height)
{
  min_height = min_height_;
  max_height = max_height_;
}

template <typename PointT> void
pcl::people::HeadBasedSubclustering<PointT>::getDimensionLimits (int& min_points, int& max_points)
{
  min_points = min_points_;
  max_points = max_points_;
}

template <typename PointT> float
pcl::people::HeadBasedSubclustering<PointT>::getMinimumDistanceBetweenHeads ()
{
  return (heads_minimum_distance_);
}

template <typename PointT> void
pcl::people::HeadBasedSubclustering<PointT>::mergeClustersCloseInFloorCoordinates (std::vector<pcl::people::PersonCluster<PointT> >& input_clusters,
    std::vector<pcl::people::PersonCluster<PointT> >& output_clusters)
{
  float min_distance_between_cluster_centers = 0.4;                   // meters
  float normalize_factor = std::pow(sqrt_ground_coeffs_, 2);          // sqrt_ground_coeffs ^ 2 (precomputed for speed)
  Eigen::Vector3f head_ground_coeffs = ground_coeffs_.head(3);        // ground plane normal (precomputed for speed)
  std::vector <std::vector<int> > connected_clusters;
  connected_clusters.resize(input_clusters.size());
  std::vector<bool> used_clusters;          // 0 in correspondence of clusters remained to process, 1 for already used clusters
  used_clusters.resize(input_clusters.size());
  for(std::size_t i = 0; i < input_clusters.size(); i++)             // for every cluster
  {
    Eigen::Vector3f theoretical_center = input_clusters[i].getTCenter();
    float t = theoretical_center.dot(head_ground_coeffs) / normalize_factor;    // height from the ground
    Eigen::Vector3f current_cluster_center_projection = theoretical_center - head_ground_coeffs * t;    // projection of the point on the groundplane
    for(std::size_t j = i+1; j < input_clusters.size(); j++)         // for every remaining cluster
    {
      theoretical_center = input_clusters[j].getTCenter();
      float t = theoretical_center.dot(head_ground_coeffs) / normalize_factor;    // height from the ground
      Eigen::Vector3f new_cluster_center_projection = theoretical_center - head_ground_coeffs * t;      // projection of the point on the groundplane
      if (((new_cluster_center_projection - current_cluster_center_projection).norm()) < min_distance_between_cluster_centers)
      {
        connected_clusters[i].push_back(j);
      }
    }
  }

  for(std::size_t i = 0; i < connected_clusters.size(); i++)   // for every cluster
  {
    if (!used_clusters[i])                                      // if this cluster has not been used yet
    {
      used_clusters[i] = true;
      if (connected_clusters[i].empty())                        // no other clusters to merge
      {
        output_clusters.push_back(input_clusters[i]);
      }
      else
      {
        // Copy cluster points into new cluster:
        pcl::PointIndices point_indices;
        point_indices = input_clusters[i].getIndices();
        for(const int &cluster : connected_clusters[i])
        {
          if (!used_clusters[cluster])         // if this cluster has not been used yet
          {
            used_clusters[cluster] = true;
            for(const auto& cluster_idx : input_clusters[cluster].getIndices().indices)
            {
              point_indices.indices.push_back(cluster_idx);
            }
          }
        }
        pcl::people::PersonCluster<PointT> cluster(cloud_, point_indices, ground_coeffs_, sqrt_ground_coeffs_, head_centroid_, vertical_);
        output_clusters.push_back(cluster);
      }
    }
  }
    }

template <typename PointT> void
pcl::people::HeadBasedSubclustering<PointT>::createSubClusters (pcl::people::PersonCluster<PointT>& cluster, int maxima_number,
    std::vector<int>& maxima_cloud_indices, std::vector<pcl::people::PersonCluster<PointT> >& subclusters)
{
  // create new clusters from the current cluster and put corresponding indices into sub_clusters_indices:
  float normalize_factor = std::pow(sqrt_ground_coeffs_, 2);          // sqrt_ground_coeffs ^ 2 (precomputed for speed)
  Eigen::Vector3f head_ground_coeffs = ground_coeffs_.head(3);        // ground plane normal (precomputed for speed)
  Eigen::Matrix3Xf maxima_projected(3,maxima_number);                 // matrix containing the projection of maxima onto the ground plane
  Eigen::VectorXi subclusters_number_of_points(maxima_number);        // subclusters number of points
  std::vector <pcl::Indices> sub_clusters_indices;                    // vector of vectors with the cluster indices for every maximum
  sub_clusters_indices.resize(maxima_number);                         // resize to number of maxima

  // Project maxima on the ground plane:
  for(int i = 0; i < maxima_number; i++)                              // for every maximum
  {
    PointT* current_point = &(*cloud_)[maxima_cloud_indices[i]]; // current maximum point cloud point
    Eigen::Vector3f p_current_eigen(current_point->x, current_point->y, current_point->z);  // conversion to eigen
    float t = p_current_eigen.dot(head_ground_coeffs) / normalize_factor;       // height from the ground
    maxima_projected.col(i).matrix () = p_current_eigen - head_ground_coeffs * t;         // projection of the point on the groundplane
    subclusters_number_of_points(i) = 0;                              // initialize number of points
  }

  // Associate cluster points to one of the maximum:
  for(const auto& cluster_idx : cluster.getIndices().indices)
  {
    Eigen::Vector3f p_current_eigen((*cloud_)[cluster_idx].x, (*cloud_)[cluster_idx].y, (*cloud_)[cluster_idx].z);  // conversion to eigen
    float t = p_current_eigen.dot(head_ground_coeffs) / normalize_factor;       // height from the ground
    p_current_eigen -= head_ground_coeffs * t;       // projection of the point on the groundplane

    int i = 0;
    bool correspondence_detected = false;
    while ((!correspondence_detected) && (i < maxima_number))
    {
      if (((p_current_eigen - maxima_projected.col(i)).norm()) < heads_minimum_distance_)
      {
        correspondence_detected = true;
        sub_clusters_indices[i].push_back(cluster_idx);
        subclusters_number_of_points(i)++;
      }
      else
        i++;
    }
  }

  // Create a subcluster if the number of points associated to a maximum is over a threshold:
  for(int i = 0; i < maxima_number; i++)                              // for every maximum
  {
    if (subclusters_number_of_points(i) > min_points_)
    {
      pcl::PointIndices point_indices;
      point_indices.indices = sub_clusters_indices[i];                // indices associated to the i-th maximum

      pcl::people::PersonCluster<PointT> cluster(cloud_, point_indices, ground_coeffs_, sqrt_ground_coeffs_, head_centroid_, vertical_);
      subclusters.push_back(cluster);
      //std::cout << "Cluster number of points: " << subclusters_number_of_points(i) << std::endl;
    }
  }
}

template <typename PointT> void
pcl::people::HeadBasedSubclustering<PointT>::subcluster (std::vector<pcl::people::PersonCluster<PointT> >& clusters)
{
  // Check if all mandatory variables have been set:
  if (std::isnan(sqrt_ground_coeffs_))
  {
    PCL_ERROR ("[pcl::people::pcl::people::HeadBasedSubclustering::subcluster] Floor parameters have not been set or they are not valid!\n");
    return;
  }
  if (cluster_indices_.empty ())
  {
    PCL_ERROR ("[pcl::people::pcl::people::HeadBasedSubclustering::subcluster] Cluster indices have not been set!\n");
    return;
  }
  if (cloud_ == nullptr)
  {
    PCL_ERROR ("[pcl::people::pcl::people::HeadBasedSubclustering::subcluster] Input cloud has not been set!\n");
    return;
  }

  // Person clusters creation from clusters indices:
  for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices_.begin(); it != cluster_indices_.end(); ++it)
  {
    pcl::people::PersonCluster<PointT> cluster(cloud_, *it, ground_coeffs_, sqrt_ground_coeffs_, head_centroid_, vertical_);  // PersonCluster creation
    clusters.push_back(cluster);
  }

  // Remove clusters with too high height from the ground plane:
  std::vector<pcl::people::PersonCluster<PointT> > new_clusters;
  for(std::size_t i = 0; i < clusters.size(); i++)   // for every cluster
  {
    if (clusters[i].getHeight() <= max_height_)
      new_clusters.push_back(clusters[i]);
  }
  clusters = new_clusters;
  new_clusters.clear();

  // Merge clusters close in floor coordinates:
  mergeClustersCloseInFloorCoordinates(clusters, new_clusters);
  clusters = new_clusters;

  std::vector<pcl::people::PersonCluster<PointT> > subclusters;
  int cluster_min_points_sub = int(float(min_points_) * 1.5);
  //  int cluster_max_points_sub = max_points_;

  // create HeightMap2D object:
  pcl::people::HeightMap2D<PointT> height_map_obj;
  height_map_obj.setGround(ground_coeffs_);
  height_map_obj.setInputCloud(cloud_);
  height_map_obj.setSensorPortraitOrientation(vertical_);
  height_map_obj.setMinimumDistanceBetweenMaxima(heads_minimum_distance_);
  for(typename std::vector<pcl::people::PersonCluster<PointT> >::iterator it = clusters.begin(); it != clusters.end(); ++it)   // for every cluster
  {
    float height = it->getHeight();
    int number_of_points = it->getNumberPoints();
    if(height > min_height_ && height < max_height_)
    {
      if (number_of_points > cluster_min_points_sub) //  && number_of_points < cluster_max_points_sub)
      {
        // Compute height map associated to the current cluster and its local maxima (heads):
        height_map_obj.compute(*it);
        if (height_map_obj.getMaximaNumberAfterFiltering() > 1)        // if more than one maximum
        {
          // create new clusters from the current cluster and put corresponding indices into sub_clusters_indices:
          createSubClusters(*it, height_map_obj.getMaximaNumberAfterFiltering(), height_map_obj.getMaximaCloudIndicesFiltered(), subclusters);
        }
        else
        {  // Only one maximum --> copy original cluster:
          subclusters.push_back(*it);
        }
      }
      else
      {
        // Cluster properties not good for sub-clustering --> copy original cluster:
        subclusters.push_back(*it);
      }
    }
  }
  clusters = subclusters;    // substitute clusters with subclusters
}

template <typename PointT>
pcl::people::HeadBasedSubclustering<PointT>::~HeadBasedSubclustering ()
{
  // TODO Auto-generated destructor stub
}
#endif /* PCL_PEOPLE_HEAD_BASED_SUBCLUSTER_HPP_ */
