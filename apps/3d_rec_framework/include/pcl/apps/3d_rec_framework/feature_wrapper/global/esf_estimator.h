/*
 * vfh_estimator.h
 *
 *  Created on: Mar 22, 2012
 *      Author: aitor
 */

#pragma once

#include <pcl/apps/3d_rec_framework/feature_wrapper/global/global_estimator.h>
#include <pcl/features/esf.h>

namespace pcl {
namespace rec_3d_framework {

template <typename PointInT, typename FeatureT>
class ESFEstimation : public GlobalEstimator<PointInT, FeatureT> {

  using PointInTPtr = typename pcl::PointCloud<PointInT>::Ptr;

public:
  void
  estimate(PointInTPtr& in,
           PointInTPtr& processed,
           typename pcl::PointCloud<FeatureT>::CloudVectorType& signatures,
           std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>&
               centroids) override
  {

    using ESFEstimation = pcl::ESFEstimation<PointInT, FeatureT>;
    pcl::PointCloud<FeatureT> ESF_signature;

    ESFEstimation esf;
    esf.setInputCloud(in);
    esf.compute(ESF_signature);

    signatures.resize(1);
    centroids.resize(1);

    signatures[0] = ESF_signature;

    Eigen::Vector4f centroid4f;
    pcl::compute3DCentroid(*in, centroid4f);
    centroids[0] = Eigen::Vector3f(centroid4f[0], centroid4f[1], centroid4f[2]);

    pcl::copyPointCloud(*in, *processed);
  }

  bool
  computedNormals() override
  {
    return false;
  }
};

} // namespace rec_3d_framework
} // namespace pcl
