/*
 * shot_local_estimator.h
 *
 *  Created on: Mar 24, 2012
 *      Author: aitor
 */

#pragma once

#include <pcl/apps/3d_rec_framework/feature_wrapper/local/local_estimator.h>
#include <pcl/apps/3d_rec_framework/feature_wrapper/normal_estimator.h>
#include <pcl/features/shot_omp.h>
#include <pcl/io/pcd_io.h>

namespace pcl {
namespace rec_3d_framework {

template <typename PointInT, typename FeatureT>
class SHOTLocalEstimationOMP : public LocalEstimator<PointInT, FeatureT> {

  using PointInTPtr = typename pcl::PointCloud<PointInT>::Ptr;
  using FeatureTPtr = typename pcl::PointCloud<FeatureT>::Ptr;
  using KeypointCloud = pcl::PointCloud<pcl::PointXYZ>;

  using LocalEstimator<PointInT, FeatureT>::support_radius_;
  using LocalEstimator<PointInT, FeatureT>::normal_estimator_;
  using LocalEstimator<PointInT, FeatureT>::keypoint_extractor_;
  using LocalEstimator<PointInT, FeatureT>::neighborhood_indices_;
  using LocalEstimator<PointInT, FeatureT>::neighborhood_dist_;
  using LocalEstimator<PointInT, FeatureT>::adaptative_MLS_;

public:
  bool
  estimate(PointInTPtr& in,
           PointInTPtr& processed,
           PointInTPtr& keypoints,
           FeatureTPtr& signatures) override
  {
    if (!normal_estimator_) {
      PCL_ERROR("SHOTLocalEstimationOMP :: This feature needs normals... please "
                "provide a normal estimator\n");
      return false;
    }

    if (keypoint_extractor_.empty()) {
      PCL_ERROR("SHOTLocalEstimationOMP :: This feature needs a keypoint extractor... "
                "please provide one\n");
      return false;
    }

    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::MovingLeastSquares<PointInT, PointInT> mls;
    if (adaptative_MLS_) {
      typename search::KdTree<PointInT>::Ptr tree;
      Eigen::Vector4f centroid_cluster;
      pcl::compute3DCentroid(*in, centroid_cluster);
      float dist_to_sensor = centroid_cluster.norm();
      float sigma = dist_to_sensor * 0.01f;
      mls.setSearchMethod(tree);
      mls.setSearchRadius(sigma);
      mls.setUpsamplingMethod(mls.SAMPLE_LOCAL_PLANE);
      mls.setUpsamplingRadius(0.002);
      mls.setUpsamplingStepSize(0.001);
    }

    normals.reset(new pcl::PointCloud<pcl::Normal>);
    {
      pcl::ScopeTime t("Compute normals");
      normal_estimator_->estimate(in, processed, normals);
    }

    if (adaptative_MLS_) {
      mls.setInputCloud(processed);

      PointInTPtr filtered(new pcl::PointCloud<PointInT>);
      mls.process(*filtered);

      processed.reset(new pcl::PointCloud<PointInT>);
      normals.reset(new pcl::PointCloud<pcl::Normal>);
      {
        pcl::ScopeTime t("Compute normals after MLS");
        filtered->is_dense = false;
        normal_estimator_->estimate(filtered, processed, normals);
      }
    }

    this->computeKeypoints(processed, keypoints, normals);
    std::cout << " " << normals->points.size() << " " << processed->points.size()
              << std::endl;

    if (keypoints->points.empty()) {
      PCL_WARN("SHOTLocalEstimationOMP :: No keypoints were found\n");
      return false;
    }

    // compute signatures
    using SHOTEstimator = pcl::SHOTEstimationOMP<PointInT, pcl::Normal, pcl::SHOT352>;
    typename pcl::search::KdTree<PointInT>::Ptr tree(new pcl::search::KdTree<PointInT>);
    tree->setInputCloud(processed);

    pcl::PointCloud<pcl::SHOT352>::Ptr shots(new pcl::PointCloud<pcl::SHOT352>);
    SHOTEstimator shot_estimate;
    shot_estimate.setNumberOfThreads(8);
    shot_estimate.setSearchMethod(tree);
    shot_estimate.setInputCloud(keypoints);
    shot_estimate.setSearchSurface(processed);
    shot_estimate.setInputNormals(normals);
    shot_estimate.setRadiusSearch(support_radius_);

    {
      pcl::ScopeTime t("Compute SHOT");
      shot_estimate.compute(*shots);
    }

    signatures->resize(shots->points.size());
    signatures->width = static_cast<int>(shots->points.size());
    signatures->height = 1;

    int size_feat = sizeof((*signatures)[0].histogram) / sizeof(float);

    int good = 0;
    for (const auto& point : shots->points) {

      int NaNs = 0;
      for (int i = 0; i < size_feat; i++) {
        if (!std::isfinite(point.descriptor[i]))
          NaNs++;
      }

      if (NaNs == 0) {
        for (int i = 0; i < size_feat; i++) {
          (*signatures)[good].histogram[i] = point.descriptor[i];
        }

        good++;
      }
    }

    signatures->resize(good);
    signatures->width = good;

    return true;
  }
};

} // namespace rec_3d_framework
} // namespace pcl
