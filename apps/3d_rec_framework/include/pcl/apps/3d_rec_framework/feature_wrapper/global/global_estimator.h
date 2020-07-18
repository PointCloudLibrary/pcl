/*
 * estimator.h
 *
 *  Created on: Mar 22, 2012
 *      Author: aitor
 */

#pragma once

#include <pcl/apps/3d_rec_framework/feature_wrapper/normal_estimator.h>

#include <memory>

namespace pcl {
namespace rec_3d_framework {

template <typename PointInT, typename FeatureT>
class GlobalEstimator {
protected:
  bool computed_normals_;
  using PointInTPtr = typename pcl::PointCloud<PointInT>::Ptr;
  using FeatureTPtr = typename pcl::PointCloud<FeatureT>::Ptr;

  std::shared_ptr<PreProcessorAndNormalEstimator<PointInT, pcl::Normal>>
      normal_estimator_;

  pcl::PointCloud<pcl::Normal>::Ptr normals_;

public:
  virtual ~GlobalEstimator() = default;

  virtual void
  estimate(PointInTPtr& in,
           PointInTPtr& processed,
           std::vector<pcl::PointCloud<FeatureT>,
                       Eigen::aligned_allocator<pcl::PointCloud<FeatureT>>>& signatures,
           std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>&
               centroids) = 0;

  virtual bool
  computedNormals() = 0;

  void
  setNormalEstimator(
      std::shared_ptr<PreProcessorAndNormalEstimator<PointInT, pcl::Normal>>& ne)
  {
    normal_estimator_ = ne;
  }

  void
  getNormals(pcl::PointCloud<pcl::Normal>::Ptr& normals)
  {
    normals = normals_;
  }
};

} // namespace rec_3d_framework
} // namespace pcl
