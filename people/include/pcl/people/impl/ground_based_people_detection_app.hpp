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
 * ground_based_people_detection_app.hpp
 * Created on: Nov 30, 2012
 * Author: Matteo Munaro
 */

#ifndef PCL_PEOPLE_GROUND_BASED_PEOPLE_DETECTION_APP_HPP_
#define PCL_PEOPLE_GROUND_BASED_PEOPLE_DETECTION_APP_HPP_

#include <pcl/people/ground_based_people_detection_app.h>
#include <pcl/filters/extract_indices.h> // for ExtractIndices
#include <pcl/segmentation/extract_clusters.h> // for EuclideanClusterExtraction
#include <pcl/filters/voxel_grid.h> // for VoxelGrid

template <typename PointT>
pcl::people::GroundBasedPeopleDetectionApp<PointT>::GroundBasedPeopleDetectionApp ()
{
  rgb_image_ = pcl::PointCloud<pcl::RGB>::Ptr(new pcl::PointCloud<pcl::RGB>);

  // set default values for optional parameters:
  sampling_factor_ = 1;
  voxel_size_ = 0.06;
  vertical_ = false;
  head_centroid_ = true;
  min_fov_ = 0;
  max_fov_ = 50;
  min_height_ = 1.3;
  max_height_ = 2.3;
  min_width_ = 0.1;
  max_width_ = 8.0;
  updateMinMaxPoints ();
  heads_minimum_distance_ = 0.3;

  // set flag values for mandatory parameters:
  sqrt_ground_coeffs_ = std::numeric_limits<float>::quiet_NaN();
  ground_coeffs_set_ = false;
  intrinsics_matrix_set_ = false;
  person_classifier_set_flag_ = false;

  // set other flags
  transformation_set_ = false;
}

template <typename PointT> void
pcl::people::GroundBasedPeopleDetectionApp<PointT>::setInputCloud (PointCloudPtr& cloud)
{
  cloud_ = cloud;
}

template <typename PointT> void
pcl::people::GroundBasedPeopleDetectionApp<PointT>::setTransformation (const Eigen::Matrix3f& transformation)
{
  if (!transformation.isUnitary())
  {
	PCL_ERROR ("[pcl::people::GroundBasedPeopleDetectionApp::setCloudTransform] The cloud transformation matrix must be an orthogonal matrix!\n");
  }

  transformation_ = transformation;
  transformation_set_ = true;
  applyTransformationGround();
  applyTransformationIntrinsics();
}

template <typename PointT> void
pcl::people::GroundBasedPeopleDetectionApp<PointT>::setGround (Eigen::VectorXf& ground_coeffs)
{
  ground_coeffs_ = ground_coeffs;
  ground_coeffs_set_ = true;
  sqrt_ground_coeffs_ = (ground_coeffs - Eigen::Vector4f(0.0f, 0.0f, 0.0f, ground_coeffs(3))).norm();
  applyTransformationGround();
}

template <typename PointT> void
pcl::people::GroundBasedPeopleDetectionApp<PointT>::setSamplingFactor (int sampling_factor)
{
  sampling_factor_ = sampling_factor;
}

template <typename PointT> void
pcl::people::GroundBasedPeopleDetectionApp<PointT>::setVoxelSize (float voxel_size)
{
  voxel_size_ = voxel_size;
  updateMinMaxPoints ();
}

template <typename PointT> void
pcl::people::GroundBasedPeopleDetectionApp<PointT>::setIntrinsics (Eigen::Matrix3f intrinsics_matrix)
{
  intrinsics_matrix_ = intrinsics_matrix;
  intrinsics_matrix_set_ = true;
  applyTransformationIntrinsics();
}

template <typename PointT> void
pcl::people::GroundBasedPeopleDetectionApp<PointT>::setClassifier (pcl::people::PersonClassifier<pcl::RGB> person_classifier)
{
  person_classifier_ = person_classifier;
  person_classifier_set_flag_ = true;
}

template <typename PointT> void
pcl::people::GroundBasedPeopleDetectionApp<PointT>::setFOV (float min_fov, float max_fov)
{
  min_fov_ = min_fov;
  max_fov_ = max_fov;
}

template <typename PointT> void
pcl::people::GroundBasedPeopleDetectionApp<PointT>::setSensorPortraitOrientation (bool vertical)
{
  vertical_ = vertical;
}

template<typename PointT>
void pcl::people::GroundBasedPeopleDetectionApp<PointT>::updateMinMaxPoints ()
{
  min_points_ = (int) (min_height_ * min_width_ / voxel_size_ / voxel_size_);
  max_points_ = (int) (max_height_ * max_width_ / voxel_size_ / voxel_size_);
}

template <typename PointT> void
pcl::people::GroundBasedPeopleDetectionApp<PointT>::setPersonClusterLimits (float min_height, float max_height, float min_width, float max_width)
{
  min_height_ = min_height;
  max_height_ = max_height;
  min_width_ = min_width;
  max_width_ = max_width;
  updateMinMaxPoints ();
}

template <typename PointT> void
pcl::people::GroundBasedPeopleDetectionApp<PointT>::setMinimumDistanceBetweenHeads (float heads_minimum_distance)
{
  heads_minimum_distance_= heads_minimum_distance;
}

template <typename PointT> void
pcl::people::GroundBasedPeopleDetectionApp<PointT>::setHeadCentroid (bool head_centroid)
{
  head_centroid_ = head_centroid;
}

template <typename PointT> void
pcl::people::GroundBasedPeopleDetectionApp<PointT>::getPersonClusterLimits (float& min_height, float& max_height, float& min_width, float& max_width)
{
  min_height = min_height_;
  max_height = max_height_;
  min_width = min_width_;
  max_width = max_width_;
}

template <typename PointT> void
pcl::people::GroundBasedPeopleDetectionApp<PointT>::getDimensionLimits (int& min_points, int& max_points)
{
  min_points = min_points_;
  max_points = max_points_;
}

template <typename PointT> float
pcl::people::GroundBasedPeopleDetectionApp<PointT>::getMinimumDistanceBetweenHeads ()
{
  return (heads_minimum_distance_);
}

template <typename PointT> Eigen::VectorXf
pcl::people::GroundBasedPeopleDetectionApp<PointT>::getGround ()
{
  if (!ground_coeffs_set_)
  {
    PCL_ERROR ("[pcl::people::GroundBasedPeopleDetectionApp::getGround] Floor parameters have not been set or they are not valid!\n");
  }
  return (ground_coeffs_);
}

template <typename PointT> typename pcl::people::GroundBasedPeopleDetectionApp<PointT>::PointCloudPtr
pcl::people::GroundBasedPeopleDetectionApp<PointT>::getFilteredCloud ()
{
  return (cloud_filtered_);
}

template <typename PointT> typename pcl::people::GroundBasedPeopleDetectionApp<PointT>::PointCloudPtr
pcl::people::GroundBasedPeopleDetectionApp<PointT>::getNoGroundCloud ()
{
  return (no_ground_cloud_);
}

template <typename PointT> void
pcl::people::GroundBasedPeopleDetectionApp<PointT>::extractRGBFromPointCloud (PointCloudPtr input_cloud, pcl::PointCloud<pcl::RGB>::Ptr& output_cloud)
{
  // Extract RGB information from a point cloud and output the corresponding RGB point cloud  
  output_cloud->points.resize(input_cloud->height*input_cloud->width);
  output_cloud->width = input_cloud->width;
  output_cloud->height = input_cloud->height;

  pcl::RGB rgb_point;
  for (std::uint32_t j = 0; j < input_cloud->width; j++)
  {
    for (std::uint32_t i = 0; i < input_cloud->height; i++)
    { 
      rgb_point.r = (*input_cloud)(j,i).r;
      rgb_point.g = (*input_cloud)(j,i).g;
      rgb_point.b = (*input_cloud)(j,i).b;    
      (*output_cloud)(j,i) = rgb_point; 
    }
  }
}

template <typename PointT> void
pcl::people::GroundBasedPeopleDetectionApp<PointT>::swapDimensions (pcl::PointCloud<pcl::RGB>::Ptr& cloud)
{
  pcl::PointCloud<pcl::RGB>::Ptr output_cloud(new pcl::PointCloud<pcl::RGB>);
  output_cloud->points.resize(cloud->height*cloud->width);
  output_cloud->width = cloud->height;
  output_cloud->height = cloud->width;
  for (std::uint32_t i = 0; i < cloud->width; i++)
  {
    for (std::uint32_t j = 0; j < cloud->height; j++)
    {
      (*output_cloud)(j,i) = (*cloud)(cloud->width - i - 1, j);
    }
  }
  cloud = output_cloud;
}

template <typename PointT> void
pcl::people::GroundBasedPeopleDetectionApp<PointT>::applyTransformationPointCloud ()
{
  if (transformation_set_)
  {
    Eigen::Transform<float, 3, Eigen::Affine> transform;
    transform = transformation_;
    pcl::transformPointCloud(*cloud_, *cloud_, transform);
  }
}

template <typename PointT> void
pcl::people::GroundBasedPeopleDetectionApp<PointT>::applyTransformationGround ()
{
  if (transformation_set_ && ground_coeffs_set_)
  {
    Eigen::Transform<float, 3, Eigen::Affine> transform;
    transform = transformation_;
    ground_coeffs_transformed_ = transform.matrix() * ground_coeffs_;
  }
  else
  {
    ground_coeffs_transformed_ = ground_coeffs_;
  }
}

template <typename PointT> void
pcl::people::GroundBasedPeopleDetectionApp<PointT>::applyTransformationIntrinsics ()
{
  if (transformation_set_ && intrinsics_matrix_set_)
  {
    intrinsics_matrix_transformed_ = intrinsics_matrix_ * transformation_.transpose();
  }
  else
  {
    intrinsics_matrix_transformed_ = intrinsics_matrix_;
  }
}

template <typename PointT> void
pcl::people::GroundBasedPeopleDetectionApp<PointT>::filter ()
{
  cloud_filtered_ = PointCloudPtr (new PointCloud);
  pcl::VoxelGrid<PointT> grid;
  grid.setInputCloud(cloud_);
  grid.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
  grid.setFilterFieldName("z");
  grid.setFilterLimits(min_fov_, max_fov_);
  grid.filter(*cloud_filtered_);
}

template <typename PointT> bool
pcl::people::GroundBasedPeopleDetectionApp<PointT>::compute (std::vector<pcl::people::PersonCluster<PointT> >& clusters)
{
  // Check if all mandatory variables have been set:
  if (!ground_coeffs_set_)
  {
    PCL_ERROR ("[pcl::people::GroundBasedPeopleDetectionApp::compute] Floor parameters have not been set or they are not valid!\n");
    return (false);
  }
  if (cloud_ == nullptr)
  {
    PCL_ERROR ("[pcl::people::GroundBasedPeopleDetectionApp::compute] Input cloud has not been set!\n");
    return (false);
  }
  if (!intrinsics_matrix_set_)
  {
    PCL_ERROR ("[pcl::people::GroundBasedPeopleDetectionApp::compute] Camera intrinsic parameters have not been set!\n");
    return (false);
  }
  if (!person_classifier_set_flag_)
  {
    PCL_ERROR ("[pcl::people::GroundBasedPeopleDetectionApp::compute] Person classifier has not been set!\n");
    return (false);
  }

  // Fill rgb image:
  rgb_image_->points.clear();                            // clear RGB pointcloud
  extractRGBFromPointCloud(cloud_, rgb_image_);          // fill RGB pointcloud

  // Downsample of sampling_factor in every dimension:
  if (sampling_factor_ != 1)
  {
    PointCloudPtr cloud_downsampled(new PointCloud);
    cloud_downsampled->width = (cloud_->width)/sampling_factor_;
    cloud_downsampled->height = (cloud_->height)/sampling_factor_;
    cloud_downsampled->points.resize(cloud_downsampled->height*cloud_downsampled->width);
    cloud_downsampled->is_dense = cloud_->is_dense;
    for (std::uint32_t j = 0; j < cloud_downsampled->width; j++)
    {
      for (std::uint32_t i = 0; i < cloud_downsampled->height; i++)
      {
        (*cloud_downsampled)(j,i) = (*cloud_)(sampling_factor_*j,sampling_factor_*i);
      }
    }
    (*cloud_) = (*cloud_downsampled);
  }

  applyTransformationPointCloud();

  filter();

  // Ground removal and update:
  pcl::IndicesPtr inliers(new pcl::Indices);
  typename pcl::SampleConsensusModelPlane<PointT>::Ptr ground_model (new pcl::SampleConsensusModelPlane<PointT> (cloud_filtered_));
  ground_model->selectWithinDistance(ground_coeffs_transformed_, 2 * voxel_size_, *inliers);
  no_ground_cloud_ = PointCloudPtr (new PointCloud);
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloud_filtered_);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*no_ground_cloud_);
  if (inliers->size () >= (300 * 0.06 / voxel_size_ / std::pow (static_cast<double> (sampling_factor_), 2)))
    ground_model->optimizeModelCoefficients (*inliers, ground_coeffs_transformed_, ground_coeffs_transformed_);
  else
    PCL_INFO ("No groundplane update!\n");

  // Euclidean Clustering:
  std::vector<pcl::PointIndices> cluster_indices;
  typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
  tree->setInputCloud(no_ground_cloud_);
  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance(2 * voxel_size_);
  ec.setMinClusterSize(min_points_);
  ec.setMaxClusterSize(max_points_);
  ec.setSearchMethod(tree);
  ec.setInputCloud(no_ground_cloud_);
  ec.extract(cluster_indices);

  // Head based sub-clustering //
  pcl::people::HeadBasedSubclustering<PointT> subclustering;
  subclustering.setInputCloud(no_ground_cloud_);
  subclustering.setGround(ground_coeffs_transformed_);
  subclustering.setInitialClusters(cluster_indices);
  subclustering.setHeightLimits(min_height_, max_height_);
  subclustering.setMinimumDistanceBetweenHeads(heads_minimum_distance_);
  subclustering.setSensorPortraitOrientation(vertical_);
  subclustering.subcluster(clusters);

  // Person confidence evaluation with HOG+SVM:
  if (vertical_)  // Rotate the image if the camera is vertical
  {
    swapDimensions(rgb_image_);
  }
  for(auto it = clusters.begin(); it != clusters.end(); ++it)
  {
    //Evaluate confidence for the current PersonCluster:
    Eigen::Vector3f centroid = intrinsics_matrix_transformed_ * (it->getTCenter());
    centroid /= centroid(2);
    Eigen::Vector3f top = intrinsics_matrix_transformed_ * (it->getTTop());
    top /= top(2);
    Eigen::Vector3f bottom = intrinsics_matrix_transformed_ * (it->getTBottom());
    bottom /= bottom(2);
    it->setPersonConfidence(person_classifier_.evaluate(rgb_image_, bottom, top, centroid, vertical_));
  }
 
  return (true);
}

template <typename PointT>
pcl::people::GroundBasedPeopleDetectionApp<PointT>::~GroundBasedPeopleDetectionApp () = default;
#endif /* PCL_PEOPLE_GROUND_BASED_PEOPLE_DETECTION_APP_HPP_ */
