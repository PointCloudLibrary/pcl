/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2021-, Open Perception
 *
 *  All rights reserved
 */

#include <pcl/common/norms.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/multiscale_feature_persistence.h>
#include <pcl/test/gtest.h>
#include <pcl/point_cloud.h>

using PointT = pcl::PointXYZ;
using PointCloudT = pcl::PointCloud<PointT>;
using FPFHSignature = pcl::FPFHSignature33;
PointCloudT::Ptr cloud;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

TEST(PCL, DistanceBetweenFeatures)
{
  pcl::detail::MultiscaleFeaturePersistenceTest<PointT, FPFHSignature> test_distance;
  std::vector<float> a{101, -11, 24, 3, 18, 27, 65};
  std::vector<float> b{89, 29, 24, 1008, -57, 106, 85};

  // L1

  test_distance.distance_metric_ = pcl::L1;
  EXPECT_EQ(test_distance.distanceBetweenFeatures(a, b), pcl::L1_Norm(a, b, a.size()));

  // L2_SQR

  test_distance.distance_metric_ = pcl::L2_SQR;
  EXPECT_EQ(test_distance.distanceBetweenFeatures(a, b),
            pcl::L2_Norm_SQR(a, b, a.size()));

  // L2
  test_distance.distance_metric_ = pcl::L2;
  EXPECT_EQ(test_distance.distanceBetweenFeatures(a, b), pcl::L2_Norm(a, b, a.size()));

  // LINF
  test_distance.distance_metric_ = pcl::LINF;
  EXPECT_EQ(test_distance.distanceBetweenFeatures(a, b),
            pcl::Linf_Norm(a, b, a.size()));

  // JM
  // test_distance.distance_metric_ = pcl::JM;
  // EXPECT_EQ(test_distance.distanceBetweenFeatures(a, b), pcl::JM_Norm(a, b,
  // a.size()));

  // B
  test_distance.distance_metric_ = pcl::B;
  EXPECT_EQ(test_distance.distanceBetweenFeatures(a, b), pcl::B_Norm(a, b, a.size()));

  // SUBLINEAR
  test_distance.distance_metric_ = pcl::SUBLINEAR;
  EXPECT_EQ(test_distance.distanceBetweenFeatures(a, b),
            pcl::Sublinear_Norm(a, b, a.size()));

  // CS
  test_distance.distance_metric_ = pcl::CS;
  EXPECT_EQ(test_distance.distanceBetweenFeatures(a, b), pcl::CS_Norm(a, b, a.size()));

  // DIV
  test_distance.distance_metric_ = pcl::DIV;
  EXPECT_EQ(test_distance.distanceBetweenFeatures(a, b), pcl::Div_Norm(a, b, a.size()));

  // KL
  test_distance.distance_metric_ = pcl::KL;
  EXPECT_EQ(test_distance.distanceBetweenFeatures(a, b), pcl::KL_Norm(a, b, a.size()));

  // HIK
  test_distance.distance_metric_ = pcl::HIK;
  EXPECT_EQ(test_distance.distanceBetweenFeatures(a, b), pcl::HIK_Norm(a, b, a.size()));
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST(PCL, CalculateMeanFeature)
{
  pcl::detail::MultiscaleFeaturePersistenceTest<PointT, FPFHSignature> test_mean;
  test_mean.features_at_scale_vectorized_ = {{{1, 2, 3, 4, 5}, {6, 2, 9, 4, 21}},
                                             {{19, 22, 93, 4, -57}, {6, 2, 4, 7, 8}},
                                             {{11, 2, 78, 35, 89}, {2, 3, 7, 7, 14}}};
  test_mean.mean_feature_.resize(5);
  test_mean.calculateMeanFeature();

  std::vector<float> gt_mean{7.5f, 5.5f, 32.3333f, 10.1666f, 13.3333f};
  for (auto gt_it = gt_mean.begin(), result_it = gt_mean.begin();
       gt_it != gt_mean.end();
       ++gt_it, ++result_it) {
    EXPECT_FLOAT_EQ(*gt_it, *result_it);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST(PCL, ExtractUniqueFeatures)
{
  pcl::detail::MultiscaleFeaturePersistenceTest<PointT, FPFHSignature>
      test_unique_feature_extraction;
  test_unique_feature_extraction.features_at_scale_vectorized_ = {
      {{1, 2, 3, 4, 5}, {6, 2, 9, 4, 21}},
      {{19, 22, 93, 4, -57}, {6, 2, 4, 7, 8}},
      {{11, 2, 78, 35, 89}, {2, 3, 7, 7, 14}}};
  test_unique_feature_extraction.mean_feature_.resize(5);
  test_unique_feature_extraction.calculateMeanFeature();
  test_unique_feature_extraction.distance_metric_ = pcl::L1;
  test_unique_feature_extraction.scale_values_ = {1, 2, 3};

  std::vector<std::list<std::size_t>> gt_unique_features_indices_;
  std::vector<std::vector<bool>> gt_unique_features_table_;
  // alpha is 0.5
  gt_unique_features_indices_ = {{0, 1}, {0}, {0}};
  gt_unique_features_table_ = {{true, true}, {true, false}, {true, false}};
  test_unique_feature_extraction.alpha_ = 0.5f;
  test_unique_feature_extraction.extractUniqueFeatures();
  for (size_t scale_i = 0; scale_i < gt_unique_features_table_.size(); ++scale_i) {
    for (auto gt_it = gt_unique_features_indices_[scale_i].begin(),
              res_it = test_unique_feature_extraction.unique_features_indices_[scale_i]
                           .begin();
         gt_it != gt_unique_features_indices_[scale_i].end();
         ++gt_it, ++res_it) {
      EXPECT_EQ(*gt_it, *res_it);
    }
    for (auto
             gt_it = gt_unique_features_table_[scale_i].begin(),
             res_it =
                 test_unique_feature_extraction.unique_features_table_[scale_i].begin();
         gt_it != gt_unique_features_table_[scale_i].end();
         ++gt_it, ++res_it) {
      EXPECT_EQ(*gt_it, *res_it);
    }
  }

  // alpha is 1
  gt_unique_features_indices_ = {{0}, {0}, {0}};
  gt_unique_features_table_ = {{true, false}, {true, false}, {true, false}};
  test_unique_feature_extraction.alpha_ = 1;
  test_unique_feature_extraction.extractUniqueFeatures();
  for (size_t scale_i = 0; scale_i < gt_unique_features_table_.size(); ++scale_i) {
    for (auto gt_it = gt_unique_features_indices_[scale_i].begin(),
              res_it = test_unique_feature_extraction.unique_features_indices_[scale_i]
                           .begin();
         gt_it != gt_unique_features_indices_[scale_i].end();
         ++gt_it, ++res_it) {
      EXPECT_EQ(*gt_it, *res_it);
    }
    for (auto
             gt_it = gt_unique_features_table_[scale_i].begin(),
             res_it =
                 test_unique_feature_extraction.unique_features_table_[scale_i].begin();
         gt_it != gt_unique_features_table_[scale_i].end();
         ++gt_it, ++res_it) {
      EXPECT_EQ(*gt_it, *res_it);
    }
  }

  // alpha is 2
  gt_unique_features_indices_ = {{}, {}, {}};
  gt_unique_features_table_ = {{false, false}, {false, false}, {false, false}};
  test_unique_feature_extraction.alpha_ = 2;
  test_unique_feature_extraction.extractUniqueFeatures();
  for (size_t scale_i = 0; scale_i < gt_unique_features_table_.size(); ++scale_i) {
    for (auto gt_it = gt_unique_features_indices_[scale_i].begin(),
              res_it = test_unique_feature_extraction.unique_features_indices_[scale_i]
                           .begin();
         gt_it != gt_unique_features_indices_[scale_i].end();
         ++gt_it, ++res_it) {
      EXPECT_EQ(*gt_it, *res_it);
    }
    for (auto
             gt_it = gt_unique_features_table_[scale_i].begin(),
             res_it =
                 test_unique_feature_extraction.unique_features_table_[scale_i].begin();
         gt_it != gt_unique_features_table_[scale_i].end();
         ++gt_it, ++res_it) {
      EXPECT_EQ(*gt_it, *res_it);
    }
  }
}

/* ---[ */
int
main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return (RUN_ALL_TESTS());
}
/* ]--- */
