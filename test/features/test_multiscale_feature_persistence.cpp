/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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
 * $Id$
 */

#include <pcl/features/fpfh.h>
#include <pcl/features/multiscale_feature_persistence.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/test/gtest.h>
#include <pcl/point_cloud.h>

using PointT = pcl::PointXYZ;
using PointCloudT = pcl::PointCloud<PointT>;
using PointCloudNormal = pcl::PointCloud<pcl::Normal>;
using FPFHSignature = pcl::FPFHSignature33;
using PointCloudFeature = pcl::PointCloud<FPFHSignature>;

PointCloudT::Ptr cloud;
const Eigen::Vector4f subsampling_leaf_size(0.01f, 0.01f, 0.01f, 0.0f);
const float normal_estimation_search_radius = 0.05f;

void
subsampleAndCalculateNormals(PointCloudT::Ptr& cloud,
                             PointCloudT::Ptr& cloud_subsampled,
                             PointCloudNormal::Ptr& cloud_subsampled_normals)
{
  cloud_subsampled = PointCloudT::Ptr(new PointCloudT());
  pcl::VoxelGrid<PointT> subsampling_filter;
  subsampling_filter.setInputCloud(cloud);
  subsampling_filter.setLeafSize(subsampling_leaf_size);
  subsampling_filter.filter(*cloud_subsampled);

  cloud_subsampled_normals = PointCloudNormal::Ptr(new PointCloudNormal());
  pcl::NormalEstimation<PointT, pcl::Normal> normal_estimation_filter;
  normal_estimation_filter.setInputCloud(cloud_subsampled);
  pcl::search::KdTree<PointT>::Ptr search_tree(new pcl::search::KdTree<PointT>);
  normal_estimation_filter.setSearchMethod(search_tree);
  normal_estimation_filter.setRadiusSearch(normal_estimation_search_radius);
  normal_estimation_filter.compute(*cloud_subsampled_normals);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

TEST(PCL, DetermineConsistentFeatures)
{
  pcl::MultiscaleFeaturePersistence<PointT, FPFHSignature> feature_persistence;
  PointCloudFeature::Ptr output_features(new PointCloudFeature());
  auto output_indices = pcl::make_shared<pcl::Indices>();

  const float gt_output_features[27][33] = {
      {0, 0, 0,       0,       0,        100,     0,       0,       0, 0, 0,
       0, 0, 0,       0,       0.408169, 85.7398, 10.3308, 3.52122, 0, 0, 0,
       0, 0, 23.9626, 29.6761, 27.0757,  3.11305, 5.43367, 10.7389, 0, 0, 0},
      {0, 0, 0,       0,       0,      100,     0,       0,       0, 0, 0,
       0, 0, 0,       0,       0,      70.8214, 5.89744, 23.2811, 0, 0, 0,
       0, 0, 20.8214, 23.9332, 2.4597, 23.2811, 23.6071, 5.89744, 0, 0, 0},
      {0, 0, 0,       0,       0,       100,     0,       0,       0, 0, 0,
       0, 0, 0,       0,       3.60031, 66.6484, 8.67291, 21.0784, 0, 0, 0,
       0, 0, 13.8488, 13.8488, 17.968,  20.4249, 24.5831, 9.32638, 0, 0, 0},
      {0,       0,        0,       0,       4.70533, 88.1828, 7.11186, 0,       0,
       0,       0,        0,       0,       0,       0,       1.95028, 53.6036, 1.6717,
       4.37546, 0.817529, 37.5814, 0,       0,       1.02352, 1.35795, 6.66472, 18.6587,
       11.647,  26.8912,  27.3757, 1.47872, 3.5287,  1.3739},
      {0,       0,       0,        0,       0.605387, 99.3946, 0,
       0,       0,       0,        0,       0,        0,       0,
       4.77742, 39.5865, 35.3368,  19.0977, 1.20156,  0,       0,
       0,       0,       0.658105, 4.99191, 22.3883,  18.0404, 8.44501,
       3.07399, 4.32509, 15.2844,  16.3155, 6.47731},
      {0,       0,       0,       0,       14.837,  78.5993, 6.56364, 0,       0,
       0,       0,       0,       0,       0,       2.19842, 15.8933, 18.4357, 41.8781,
       11.9318, 1.70603, 7.95676, 0,       0,       1.65455, 2.752,   3.91164, 24.7348,
       7.42696, 13.4733, 16.0626, 12.5205, 14.1836, 3.28003},
      {0,       0,       0,        0,       4.22143,  95.3754,  0.403222,
       0,       0,       0,        0,       0,        0,        0,
       1.176,   37.0574, 35.8172,  17.7436, 0.826528, 0.239069, 7.14017,
       0,       0,       0.403222, 5.45272, 15.526,   15.2772,  8.60978,
       3.89588, 5.48677, 9.90773,  21.8822, 13.5585},
      {0,        0,       0,       0,       9.69826,  88.4992,  1.80255,
       0,        0,       0,       0,       0,        0,        0,
       0.686038, 19.5698, 35.0578, 11.1379, 0.936046, 0.391773, 32.2206,
       0,        0,       1.69964, 3.60651, 3.55828,  21.7155,  3.76102,
       4.34089,  17.9839, 9.71725, 19.8585, 13.7586},
      {0,        0,       0,       0,       8.13041,  89.8249,  2.04466,
       0,        0,       0,       0,       0,        0,        0,
       0.570385, 21.9487, 35.0296, 9.5727,  0.717108, 0.337256, 31.8243,
       0,        0,       1.84835, 3.03275, 7.60577,  15.6637,  13.7929,
       10.2215,  13.4343, 2.75174, 18.9798, 12.6693},
      {0,       0,       0,       0,       2.23767, 97.6162,  0.146125,
       0,       0,       0,       0,       0,       0,        0,
       2.03409, 38.1643, 36.0216, 20.0808, 1.79313, 0.111934, 1.79412,
       0,       0,       1.07664, 1.84207, 18.1883, 13.8026,  12.8604,
       2.06537, 3.72738, 9.75049, 23.6166, 13.0701},
      {0,       0,       0,       0,       1.94179, 98.0582,   0,
       0,       0,       0,       0,       0,       0,         0,
       2.67209, 40.5256, 27.9669, 24.9425, 2.21885, 0.0951404, 1.57891,
       0,       0,       1.72444, 14.6981, 7.70923, 22.4565,   5.35895,
       5.86537, 7.73914, 8.17847, 14.2164, 12.0534},
      {0, 0,      0,        0,       60.6515, 39.3485, 0,     0, 0, 0, 0,        0, 0,
       0, 11.346, 21.5938,  43.9709, 23.0893, 0,       0,     0, 0, 0, 0.721846, 0, 0,
       0, 1.2147, 0.887928, 20.1842, 18.1468, 39.9526, 18.892},
      {0, 0, 0, 0,       59.2965, 40.7035, 0,       0,       0,      0,       0,
       0, 0, 0, 7.21215, 19.2253, 52.7256, 20.8369, 0,       0,      0,       0,
       0, 0, 0, 0,       0,       5.80513, 1.24504, 15.8787, 25.505, 34.3453, 17.2208},
      {0, 0, 0, 0,       56.421, 43.579,  0,       0,       0,       0,       0,
       0, 0, 0, 7.43699, 24.655, 47.9592, 19.9489, 0,       0,       0,       0,
       0, 0, 0, 0,       0,      2.20281, 2.46741, 24.4398, 17.8244, 39.8236, 13.242},
      {0, 0, 0,       0,      56.7789, 43.2211, 0,       0,       0,      0, 0, 0,
       0, 0, 11.6001, 22.861, 47.6905, 17.8484, 0,       0,       0,      0, 0, 1.00471,
       0, 0, 0,       1.7485, 1.95555, 13.8623, 11.9821, 42.1631, 27.2837},
      {0, 0, 0, 0,       56.8728, 43.1272, 0,       0,        0,       0,      0,
       0, 0, 0, 6.01596, 26.1097, 49.8964, 17.7679, 0.210063, 0,       0,      0,
       0, 0, 0, 0,       0,       2.53109, 5.06131, 19.2648,  25.4152, 36.075, 11.6525},
      {0, 0, 0,       0,       51.8106, 48.1894, 0,        0,       0,      0, 0, 0,
       0, 0, 5.59972, 19.2943, 54.1986, 20.6518, 0.255575, 0,       0,      0, 0, 0,
       0, 0, 0,       6.78113, 7.8062,  15.5474, 31.9517,  25.3035, 12.6101},
      {0, 0, 0, 0, 0.858694, 99.1413, 0,       0,       0,       0, 0,
       0, 0, 0, 0, 0.188111, 99.8119, 0,       0,       0,       0, 0,
       0, 0, 0, 0, 0,        6.26467, 82.2678, 10.3244, 1.14313, 0, 0},
      {0, 0, 0, 0, 4.95132, 95.0487, 0, 0, 0, 0, 0,       0,      0,       0, 0, 0, 100,
       0, 0, 0, 0, 0,       0,       0, 0, 0, 0, 5.61338, 81.416, 12.9706, 0, 0, 0},
      {0, 0, 0, 0, 6.45045, 93.5496, 0,       0,       0, 0, 0,
       0, 0, 0, 0, 0,       100,     0,       0,       0, 0, 0,
       0, 0, 0, 0, 0,       5.79338, 80.1302, 14.0764, 0, 0, 0},
      {0, 0, 0, 0, 3.19265, 96.8073, 0,       0,       0,        0, 0,
       0, 0, 0, 0, 0,       100,     0,       0,       0,        0, 0,
       0, 0, 0, 0, 0,       7.14492, 77.4834, 14.8963, 0.475413, 0, 0},
      {0, 0, 0, 0, 2.91857, 97.0814, 0,       0,       0,        0, 0,
       0, 0, 0, 0, 0,       100,     0,       0,       0,        0, 0,
       0, 0, 0, 0, 0,       6.15287, 83.1738, 10.5024, 0.170916, 0, 0},
      {0, 0, 0, 0, 2.7342, 97.2658, 0,       0,       0,       0, 0,
       0, 0, 0, 0, 0,      100,     0,       0,       0,       0, 0,
       0, 0, 0, 0, 0,      5.28806, 83.1876, 10.1264, 1.39793, 0, 0},
      {0, 0, 0, 0, 8.13745, 91.8625, 0,       0,       0, 0, 0,
       0, 0, 0, 0, 0,       100,     0,       0,       0, 0, 0,
       0, 0, 0, 0, 0,       2.63515, 88.9699, 8.39498, 0, 0, 0},
      {0, 0, 0, 0, 9.74746, 90.2525, 0,       0,       0,        0, 0,
       0, 0, 0, 0, 0,       100,     0,       0,       0,        0, 0,
       0, 0, 0, 0, 0,       6.45207, 80.3423, 12.6033, 0.602394, 0, 0},
      {0, 0, 0, 0, 10.3765, 89.6235, 0, 0, 0, 0, 0,       0,       0,      0, 0, 0, 100,
       0, 0, 0, 0, 0,       0,       0, 0, 0, 0, 5.73981, 82.4882, 11.772, 0, 0, 0},
      {0, 0, 0, 0, 9.47275, 90.5273, 0,       0,       0,        0, 0,
       0, 0, 0, 0, 0,       100,     0,       0,       0,        0, 0,
       0, 0, 0, 0, 0,       5.58698, 79.8493, 14.2933, 0.270383, 0, 0}};

  const size_t gt_output_indices[27] = {0,   1,   3,   11,  25,  28,  29,  30,  31,
                                        43,  44,  74,  77,  79,  112, 115, 117, 154,
                                        222, 231, 232, 243, 244, 278, 282, 283, 287};

  std::vector<float> scale_values;
  for (float x = 2.0f; x < 3.6f; x += 0.35f)
    scale_values.push_back(x / 100.0f);
  feature_persistence.setScalesVector(scale_values);
  feature_persistence.setAlpha(1.3f);
  PointCloudT ::Ptr cloud_subsampled;
  PointCloudNormal ::Ptr cloud_subsampled_normals;
  subsampleAndCalculateNormals(cloud, cloud_subsampled, cloud_subsampled_normals);
  pcl::FPFHEstimation<PointT, pcl::Normal, FPFHSignature>::Ptr fpfh_estimation(
      new pcl::FPFHEstimation<PointT, pcl::Normal, FPFHSignature>());
  fpfh_estimation->setInputCloud(cloud_subsampled);
  fpfh_estimation->setInputNormals(cloud_subsampled_normals);
  pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
  fpfh_estimation->setSearchMethod(tree);
  feature_persistence.setFeatureEstimator(fpfh_estimation);
  feature_persistence.setDistanceMetric(pcl::CS);
  feature_persistence.determinePersistentFeatures(*output_features, output_indices);
  EXPECT_EQ(output_indices->size(), 27);

  for (size_t point_idx = 0; point_idx < output_features->points.size(); ++point_idx) {
    EXPECT_EQ(output_indices->at(point_idx), gt_output_indices[point_idx]);
    const auto& point = output_features->points[point_idx];
    const auto& gt_point = gt_output_features[point_idx];
    for (int feature_idx = 0; feature_idx < FPFHSignature::descriptorSize();
         ++feature_idx) {
      EXPECT_NEAR(point.histogram[feature_idx], gt_point[feature_idx], 1e-4);
    }
  }
}

/* ---[ */
int
main(int argc, char** argv)
{
  if (argc < 2) {
    std::cerr << "No test file given. Please download `bun0.pcd` and pass its path to "
                 "the test."
              << std::endl;
    return (-1);
  }

  cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) < 0) {
    std::cerr << "Failed to read test file. Please download `bun0.pcd` and pass its "
                 "path to the test."
              << std::endl;
    return (-1);
  }
  testing::InitGoogleTest(&argc, argv);
  return (RUN_ALL_TESTS());
}
/* ]--- */
