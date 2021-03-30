/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009-2012, Willow Garage, Inc.
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
 */

#include <pcl/features/fpfh.h>
#include <pcl/features/multiscale_feature_persistence.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/memory.h>

using namespace pcl;

const Eigen::Vector4f subsampling_leaf_size(0.01f, 0.01f, 0.01f, 0.0f);
const float normal_estimation_search_radius = 0.05f;

void
subsampleAndCalculateNormals(PointCloud<PointXYZ>::Ptr& cloud,
                             PointCloud<PointXYZ>::Ptr& cloud_subsampled,
                             PointCloud<Normal>::Ptr& cloud_subsampled_normals)
{
  cloud_subsampled = PointCloud<PointXYZ>::Ptr(new PointCloud<PointXYZ>());
  VoxelGrid<PointXYZ> subsampling_filter;
  subsampling_filter.setInputCloud(cloud);
  subsampling_filter.setLeafSize(subsampling_leaf_size);
  subsampling_filter.filter(*cloud_subsampled);

  cloud_subsampled_normals = PointCloud<Normal>::Ptr(new PointCloud<Normal>());
  NormalEstimation<PointXYZ, Normal> normal_estimation_filter;
  normal_estimation_filter.setInputCloud(cloud_subsampled);
  pcl::search::KdTree<PointXYZ>::Ptr search_tree(new pcl::search::KdTree<PointXYZ>);
  normal_estimation_filter.setSearchMethod(search_tree);
  normal_estimation_filter.setRadiusSearch(normal_estimation_search_radius);
  normal_estimation_filter.compute(*cloud_subsampled_normals);
}

int
main(int argc, char** argv)
{
  if (argc != 2) {
    PCL_ERROR("Syntax: ./multiscale_feature_persistence_example [path_to_cloud.pcl]\n");
    return -1;
  }

  PointCloud<PointXYZ>::Ptr cloud_scene(new PointCloud<PointXYZ>());
  PCDReader reader;
  reader.read(argv[1], *cloud_scene);

  PointCloud<PointXYZ>::Ptr cloud_subsampled;
  PointCloud<Normal>::Ptr cloud_subsampled_normals;
  subsampleAndCalculateNormals(cloud_scene, cloud_subsampled, cloud_subsampled_normals);

  PCL_INFO("STATS:\ninitial point cloud size: %zu\nsubsampled point cloud size: %zu\n",
           static_cast<std::size_t>(cloud_scene->size()),
           static_cast<std::size_t>(cloud_subsampled->size()));
  visualization::CloudViewer viewer(
      "Multiscale Feature Persistence Example Visualization");
  viewer.showCloud(cloud_scene, "scene");

  MultiscaleFeaturePersistence<PointXYZ, FPFHSignature33> feature_persistence;
  std::vector<float> scale_values;
  for (float x = 2.0f; x < 3.6f; x += 0.35f)
    scale_values.push_back(x / 100.0f);
  feature_persistence.setScalesVector(scale_values);
  feature_persistence.setAlpha(1.3f);
  FPFHEstimation<PointXYZ, Normal, FPFHSignature33>::Ptr fpfh_estimation(
      new FPFHEstimation<PointXYZ, Normal, FPFHSignature33>());
  fpfh_estimation->setInputCloud(cloud_subsampled);
  fpfh_estimation->setInputNormals(cloud_subsampled_normals);
  pcl::search::KdTree<PointXYZ>::Ptr tree(new pcl::search::KdTree<PointXYZ>());
  fpfh_estimation->setSearchMethod(tree);
  feature_persistence.setFeatureEstimator(fpfh_estimation);
  feature_persistence.setDistanceMetric(pcl::CS);

  PointCloud<FPFHSignature33>::Ptr output_features(new PointCloud<FPFHSignature33>());
  auto output_indices = pcl::make_shared<pcl::Indices>();
  feature_persistence.determinePersistentFeatures(*output_features, output_indices);

  PCL_INFO("persistent features cloud size: %zu\n",
           static_cast<std::size_t>(output_features->size()));

  ExtractIndices<PointXYZ> extract_indices_filter;
  extract_indices_filter.setInputCloud(cloud_subsampled);
  extract_indices_filter.setIndices(output_indices);
  PointCloud<PointXYZ>::Ptr persistent_features_locations(new PointCloud<PointXYZ>());
  extract_indices_filter.filter(*persistent_features_locations);

  viewer.showCloud(persistent_features_locations, "persistent features");
  PCL_INFO("Persistent features have been computed. Waiting for the user to quit the "
           "visualization window.\n");

  while (!viewer.wasStopped(50)) {
  }

  return 0;
}
