/*
 * vfh_estimator.h
 *
 *  Created on: Mar 22, 2012
 *      Author: aitor
 */

#pragma once

#include <pcl/apps/3d_rec_framework/feature_wrapper/global/global_estimator.h>
#include <pcl/apps/3d_rec_framework/feature_wrapper/normal_estimator.h>
#include <pcl/features/crh.h>

#include <memory>

namespace pcl {
namespace rec_3d_framework {

template <typename PointInT, typename FeatureT>
class CRHEstimation : public GlobalEstimator<PointInT, FeatureT> {

  using PointInTPtr = typename pcl::PointCloud<PointInT>::Ptr;
  using GlobalEstimator<PointInT, FeatureT>::normal_estimator_;
  using GlobalEstimator<PointInT, FeatureT>::normals_;

  std::shared_ptr<GlobalEstimator<PointInT, FeatureT>> feature_estimator_;
  using CRHPointCloud = pcl::PointCloud<pcl::Histogram<90>>;
  std::vector<CRHPointCloud::Ptr> crh_histograms_;

public:
  CRHEstimation() = default;

  void
  setFeatureEstimator(
      std::shared_ptr<GlobalEstimator<PointInT, FeatureT>>& feature_estimator)
  {
    feature_estimator_ = feature_estimator;
  }

  void
  estimate(PointInTPtr& in,
           PointInTPtr& processed,
           std::vector<pcl::PointCloud<FeatureT>,
                       Eigen::aligned_allocator<pcl::PointCloud<FeatureT>>>& signatures,
           std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>&
               centroids) override
  {

    if (!feature_estimator_) {
      PCL_ERROR("CRHEstimation needs a global estimator... please provide one\n");
      return;
    }

    feature_estimator_->estimate(in, processed, signatures, centroids);

    if (!feature_estimator_->computedNormals()) {
      normals_.reset(new pcl::PointCloud<pcl::Normal>);
      normal_estimator_->estimate(in, processed, normals_);
    }
    else {
      feature_estimator_->getNormals(normals_);
    }

    crh_histograms_.resize(signatures.size());

    using CRHEstimation = pcl::CRHEstimation<PointInT, pcl::Normal, pcl::Histogram<90>>;
    CRHEstimation crh;
    crh.setInputCloud(processed);
    crh.setInputNormals(normals_);

    for (std::size_t idx = 0; idx < signatures.size(); idx++) {
      Eigen::Vector4f centroid4f(
          centroids[idx][0], centroids[idx][1], centroids[idx][2], 0);
      crh.setCentroid(centroid4f);
      crh_histograms_[idx].reset(new CRHPointCloud());
      crh.compute(*crh_histograms_[idx]);
    }
  }

  void
  getCRHHistograms(std::vector<CRHPointCloud::Ptr>& crh_histograms)
  {
    crh_histograms = crh_histograms_;
  }

  bool
  computedNormals() override
  {
    return true;
  }
};

} // namespace rec_3d_framework
} // namespace pcl
