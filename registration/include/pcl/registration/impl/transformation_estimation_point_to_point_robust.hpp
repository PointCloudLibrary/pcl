/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
 *
 *  All rights reserved.
 */

#ifndef PCL_REGISTRATION_TRANSFORMATION_ESTIMATION_POINT_TO_POINT_ROBUST_HPP_
#define PCL_REGISTRATION_TRANSFORMATION_ESTIMATION_POINT_TO_POINT_ROBUST_HPP_

#include <pcl/common/eigen.h>

#include <limits>

namespace pcl {

namespace registration {

template <typename PointSource, typename PointTarget, typename Scalar>
inline void
TransformationEstimationPointToPointRobust<PointSource, PointTarget, Scalar>::
    estimateRigidTransformation(const pcl::PointCloud<PointSource>& cloud_src,
                                const pcl::PointCloud<PointTarget>& cloud_tgt,
                                Matrix4& transformation_matrix) const
{
  const auto nr_points = cloud_src.size();
  if (cloud_tgt.size() != nr_points) {
    PCL_ERROR("[pcl::TransformationEstimationPointToPointRobust::"
              "estimateRigidTransformation] Number "
              "of points in source (%zu) differs than target (%zu)!\n",
              static_cast<std::size_t>(nr_points),
              static_cast<std::size_t>(cloud_tgt.size()));
    return;
  }

  ConstCloudIterator<PointSource> source_it(cloud_src);
  ConstCloudIterator<PointTarget> target_it(cloud_tgt);
  estimateRigidTransformation(source_it, target_it, transformation_matrix);
}

template <typename PointSource, typename PointTarget, typename Scalar>
void
TransformationEstimationPointToPointRobust<PointSource, PointTarget, Scalar>::
    estimateRigidTransformation(const pcl::PointCloud<PointSource>& cloud_src,
                                const pcl::Indices& indices_src,
                                const pcl::PointCloud<PointTarget>& cloud_tgt,
                                Matrix4& transformation_matrix) const
{
  if (indices_src.size() != cloud_tgt.size()) {
    PCL_ERROR("[pcl::TransformationEstimationPointToPointRobust::"
              "estimateRigidTransformation] Number of points "
              "in source (%zu) differs than target (%zu)!\n",
              indices_src.size(),
              static_cast<std::size_t>(cloud_tgt.size()));
    return;
  }

  ConstCloudIterator<PointSource> source_it(cloud_src, indices_src);
  ConstCloudIterator<PointTarget> target_it(cloud_tgt);
  estimateRigidTransformation(source_it, target_it, transformation_matrix);
}

template <typename PointSource, typename PointTarget, typename Scalar>
inline void
TransformationEstimationPointToPointRobust<PointSource, PointTarget, Scalar>::
    estimateRigidTransformation(const pcl::PointCloud<PointSource>& cloud_src,
                                const pcl::Indices& indices_src,
                                const pcl::PointCloud<PointTarget>& cloud_tgt,
                                const pcl::Indices& indices_tgt,
                                Matrix4& transformation_matrix) const
{
  if (indices_src.size() != indices_tgt.size()) {
    PCL_ERROR("[pcl::TransformationEstimationPointToPointRobust::"
              "estimateRigidTransformation] Number of points "
              "in source (%zu) differs than target (%zu)!\n",
              indices_src.size(),
              indices_tgt.size());
    return;
  }

  ConstCloudIterator<PointSource> source_it(cloud_src, indices_src);
  ConstCloudIterator<PointTarget> target_it(cloud_tgt, indices_tgt);
  estimateRigidTransformation(source_it, target_it, transformation_matrix);
}

template <typename PointSource, typename PointTarget, typename Scalar>
void
TransformationEstimationPointToPointRobust<PointSource, PointTarget, Scalar>::
    estimateRigidTransformation(const pcl::PointCloud<PointSource>& cloud_src,
                                const pcl::PointCloud<PointTarget>& cloud_tgt,
                                const pcl::Correspondences& correspondences,
                                Matrix4& transformation_matrix) const
{
  ConstCloudIterator<PointSource> source_it(cloud_src, correspondences, true);
  ConstCloudIterator<PointTarget> target_it(cloud_tgt, correspondences, false);
  estimateRigidTransformation(source_it, target_it, transformation_matrix);
}

template <typename PointSource, typename PointTarget, typename Scalar>
inline void
TransformationEstimationPointToPointRobust<PointSource, PointTarget, Scalar>::
    estimateRigidTransformation(ConstCloudIterator<PointSource>& source_it,
                                ConstCloudIterator<PointTarget>& target_it,
                                Matrix4& transformation_matrix) const
{
  // Convert to Eigen format
  const int npts = static_cast<int>(source_it.size());
  if (npts == 0) {
    PCL_ERROR("[pcl::TransformationEstimationPointToPointRobust::"
              "estimateRigidTransformation] Empty correspondence set.\n");
    return;
  }
  source_it.reset();
  target_it.reset();
  Eigen::Matrix<Scalar, Eigen::Dynamic, 1> weights(npts);
  Eigen::Matrix<Scalar, Eigen::Dynamic, 1> square_distances(npts);
  for (int i = 0; i < npts; i++) {
    Scalar dx = source_it->x - target_it->x;
    Scalar dy = source_it->y - target_it->y;
    Scalar dz = source_it->z - target_it->z;
    Scalar dist2 = dx * dx + dy * dy + dz * dz;
    square_distances[i] = dist2;

    source_it++;
    target_it++;
  }

  const Scalar epsilon = std::numeric_limits<Scalar>::epsilon();
  Scalar sigma2;
  if (sigma_ < 0)
    sigma2 = std::max(square_distances.maxCoeff() / Scalar(9.0), epsilon);
  else
    sigma2 = std::max(sigma_ * sigma_, epsilon);

  for (int i = 0; i < npts; i++) {
    weights[i] = std::exp(-square_distances[i] / (Scalar(2.0) * sigma2));
  }
  const Scalar weights_sum = weights.sum();
  if (weights_sum > epsilon)
    weights /= weights_sum;
  else
    weights.setConstant(Scalar(1.0) / static_cast<Scalar>(npts));

  source_it.reset();
  target_it.reset();
  // <cloud_src,cloud_src> is the source dataset
  transformation_matrix.setIdentity();

  Eigen::Matrix<Scalar, 4, 1> centroid_src, centroid_tgt;
  // Estimate the centroids of source, target
  computeWeighted3DCentroid(source_it, weights, centroid_src);
  computeWeighted3DCentroid(target_it, weights, centroid_tgt);
  source_it.reset();
  target_it.reset();

  // Subtract the centroids from source, target
  Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> cloud_src_demean,
      cloud_tgt_demean;
  demeanPointCloud(source_it, centroid_src, cloud_src_demean);
  demeanPointCloud(target_it, centroid_tgt, cloud_tgt_demean);

  getTransformationFromCorrelation(cloud_src_demean,
                                   centroid_src,
                                   cloud_tgt_demean,
                                   centroid_tgt,
                                   weights,
                                   transformation_matrix);
}

template <typename PointSource, typename PointTarget, typename Scalar>
void
TransformationEstimationPointToPointRobust<PointSource, PointTarget, Scalar>::
    getTransformationFromCorrelation(
        const Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>& cloud_src_demean,
        const Eigen::Matrix<Scalar, 4, 1>& centroid_src,
        const Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>& cloud_tgt_demean,
        const Eigen::Matrix<Scalar, 4, 1>& centroid_tgt,
        const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& weights,
        Matrix4& transformation_matrix) const
{
  transformation_matrix.setIdentity();

  // Assemble the correlation matrix H = source * weights * target'
  Eigen::Matrix<Scalar, 3, 3> H =
      (cloud_src_demean * weights.asDiagonal() * cloud_tgt_demean.transpose())
          .template topLeftCorner<3, 3>();

  // Compute the Singular Value Decomposition
  Eigen::JacobiSVD<Eigen::Matrix<Scalar, 3, 3>> svd(
      H, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix<Scalar, 3, 3> u = svd.matrixU();
  Eigen::Matrix<Scalar, 3, 3> v = svd.matrixV();

  // Compute R = V * U'
  if (u.determinant() * v.determinant() < 0) {
    for (int x = 0; x < 3; ++x)
      v(x, 2) *= -1;
  }

  Eigen::Matrix<Scalar, 3, 3> R = v * u.transpose();

  // Return the correct transformation
  transformation_matrix.template topLeftCorner<3, 3>() = R;
  const Eigen::Matrix<Scalar, 3, 1> Rc(R * centroid_src.template head<3>());
  transformation_matrix.template block<3, 1>(0, 3) =
      centroid_tgt.template head<3>() - Rc;

  if (pcl::console::isVerbosityLevelEnabled(pcl::console::L_DEBUG)) {
    const auto N = cloud_src_demean.cols();
    if (N > 0) {
      PCL_DEBUG("[pcl::registration::TransformationEstimationPointToPointRobust::"
                "getTransformationFromCorrelation] Loss: %.10e\n",
                (cloud_tgt_demean.template topRows<3>() -
                 R * cloud_src_demean.template topRows<3>())
                        .squaredNorm() /
                    static_cast<double>(N));
    }
  }
}

template <typename PointSource, typename PointTarget, typename Scalar>
template <typename PointT>
inline unsigned int
TransformationEstimationPointToPointRobust<PointSource, PointTarget, Scalar>::
    computeWeighted3DCentroid(ConstCloudIterator<PointT>& cloud_iterator,
                              const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& weights,
                              Eigen::Matrix<Scalar, 4, 1>& centroid) const
{
  Eigen::Matrix<Scalar, 4, 1> accumulator{0, 0, 0, 0};

  unsigned int cp = 0;
  Scalar sum_weights = Scalar(0);
  Eigen::Index weight_idx = 0;

  // For each point in the cloud
  // If the data is dense, we don't need to check for NaN
  while (cloud_iterator.isValid()) {
    // Check if the point is invalid
    if (pcl::isFinite(*cloud_iterator)) {
      const Scalar w = weights[weight_idx];
      accumulator[0] += w * cloud_iterator->x;
      accumulator[1] += w * cloud_iterator->y;
      accumulator[2] += w * cloud_iterator->z;
      sum_weights += w;
      ++cp;
    }
    ++weight_idx;
    ++cloud_iterator;
  }

  if (cp > 0 && sum_weights > std::numeric_limits<Scalar>::epsilon()) {
    centroid = accumulator / sum_weights;
    centroid[3] = 1;
  }
  return (cp);
}

} // namespace registration
} // namespace pcl

#endif /* PCL_REGISTRATION_TRANSFORMATION_ESTIMATION_POINT_TO_POINT_ROBUST_HPP_ */
