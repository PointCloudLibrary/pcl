/*
 * vfh_estimator.h
 *
 *  Created on: Mar 22, 2012
 *      Author: aitor
 */

#pragma once

#include <pcl/apps/3d_rec_framework/feature_wrapper/global/global_estimator.h>
#include <pcl/apps/3d_rec_framework/feature_wrapper/normal_estimator.h>
#include <pcl/features/vfh.h>

namespace pcl {
namespace rec_3d_framework {

template <typename PointInT, typename FeatureT>
class VFHEstimation : public GlobalEstimator<PointInT, FeatureT> {

  using PointInTPtr = typename pcl::PointCloud<PointInT>::Ptr;
  using GlobalEstimator<PointInT, FeatureT>::normal_estimator_;
  using GlobalEstimator<PointInT, FeatureT>::normals_;

public:
  void
  estimate(PointInTPtr& in,
           PointInTPtr& processed,
           typename pcl::PointCloud<FeatureT>::CloudVectorType& signatures,
           std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>&
               centroids) override
  {

    if (!normal_estimator_) {
      PCL_ERROR("This feature needs normals... please provide a normal estimator\n");
      return;
    }

    normals_.reset(new pcl::PointCloud<pcl::Normal>);
    normal_estimator_->estimate(in, processed, normals_);

    using VFHEstimation = pcl::VFHEstimation<PointInT, pcl::Normal, FeatureT>;
    pcl::PointCloud<FeatureT> vfh_signature;

    VFHEstimation vfh;
    typename pcl::search::KdTree<PointInT>::Ptr vfh_tree(
        new pcl::search::KdTree<PointInT>);
    vfh.setSearchMethod(vfh_tree);
    vfh.setInputCloud(processed);
    vfh.setInputNormals(normals_);
    vfh.setNormalizeBins(true);
    vfh.setNormalizeDistance(true);
    vfh.compute(vfh_signature);

    signatures.resize(1);
    centroids.resize(1);

    signatures[0] = vfh_signature;

    Eigen::Vector4f centroid4f;
    pcl::compute3DCentroid(*in, centroid4f);
    centroids[0] = Eigen::Vector3f(centroid4f[0], centroid4f[1], centroid4f[2]);
  }

  bool
  computedNormals() override
  {
    return true;
  }
};

} // namespace rec_3d_framework
} // namespace pcl
