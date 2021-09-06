/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2021-, Open Perception
 *
 *  All rights reserved
 */

#pragma once

#include <pcl/common/eigen.h>       // for getTransformation
#include <pcl/common/point_tests.h> // for isFinite
#include <pcl/filters/experimental/functor_filter.h>
#include <pcl/filters/filter_indices.h>

#include <functional>
namespace pcl {
namespace experimental {

/** \brief CropBox is a filter that allows the user to filter all the data
 * inside of a given box.
 *
 * \author Justin Rosen
 * \ingroup filters
 */
template <typename PointT>
class CropBox : public FilterIndices<PointT> {
  using PointCloud = typename FilterIndices<PointT>::PointCloud;
  using PointCloudPtr = typename PointCloud::Ptr;
  using PointCloudConstPtr = typename PointCloud::ConstPtr;

public:
  using Ptr = shared_ptr<CropBox<PointT>>;
  using ConstPtr = shared_ptr<const CropBox<PointT>>;

  /** \brief Constructor.
   * \param[in] extract_removed_indices Set to true if you want to be able to extract
   * the indices of points being removed (default = false).
   */
  CropBox(bool extract_removed_indices = false)
  : FilterIndices<PointT>(extract_removed_indices)
  {
    filter_name_ = "CropBox";
  }

  /** \brief Set the minimum point of the box
   * \param[in] min_pt the minimum point of the box
   */
  inline void
  setMin(const Eigen::Vector4f& min_pt)
  {
    min_pt_ = min_pt;
  }

  /** \brief Get the value of the minimum point of the box, as set by the user
   * \return the value of the internal \a min_pt parameter.
   */
  inline Eigen::Vector4f
  getMin() const
  {
    return min_pt_;
  }

  /** \brief Set the maximum point of the box
   * \param[in] max_pt the maximum point of the box
   */
  inline void
  setMax(const Eigen::Vector4f& max_pt)
  {
    max_pt_ = max_pt;
  }

  /** \brief Get the value of the maximum point of the box, as set by the user
   * \return the value of the internal \a max_pt parameter.
   */
  inline Eigen::Vector4f
  getMax() const
  {
    return max_pt_;
  }

  /** \brief Set a translation value for the box
   * \param[in] translation the (tx,ty,tz) values that the box should be translated by
   */
  inline void
  setTranslation(const Eigen::Vector3f& translation)
  {
    translation_ = translation;
  }

  /** \brief Get the value of the box translation parameter as set by the user. */
  Eigen::Vector3f
  getTranslation() const
  {
    return translation_;
  }

  /** \brief Set a rotation value for the box
   * \param[in] rotation the (rx,ry,rz) values that the box should be rotated by
   */
  inline void
  setRotation(const Eigen::Vector3f& rotation)
  {
    rotation_ = rotation;
  }

  /** \brief Get the value of the box rotatation parameter, as set by the user. */
  inline Eigen::Vector3f
  getRotation() const
  {
    return rotation_;
  }

  /** \brief Set a transformation that should be applied to the cloud before filtering
   * \param[in] transform an affine transformation that needs to be applied to the
   * cloud before filtering
   */
  inline void
  setTransform(const Eigen::Affine3f& transform)
  {
    transform_ = transform;
  }

  /** \brief Get the value of the transformation parameter, as set by the user. */
  inline Eigen::Affine3f
  getTransform() const
  {
    return transform_;
  }

  /** \brief Sample of point indices
   * \param[out] indices the resultant point cloud indices
   */
  void
  applyFilter(Indices& indices) override
  {
    Eigen::Affine3f box_transform;
    pcl::getTransformation(translation_(0),
                           translation_(1),
                           translation_(2),
                           rotation_(0),
                           rotation_(1),
                           rotation_(2),
                           box_transform);
    const Eigen::Matrix4f pt_transform =
        (box_transform.inverse() * transform_).matrix();

    std::function<bool(const PointCloud&, index_t)> lambda;
    if (pt_transform.isIdentity())
      lambda = [&](const PointCloud& cloud, index_t idx) {
        const auto& pt = cloud.at(idx).getVector4fMap();
        return (pt.array() >= min_pt_.array()).template head<3>().all() &&
               (pt.array() <= max_pt_.array()).template head<3>().all();
      };
    else
      lambda = [&](const PointCloud& cloud, index_t idx) {
        const Eigen::Vector4f pt =
            pt_transform * cloud.at(idx).getVector3fMap().homogeneous();
        return (pt.array() >= min_pt_.array()).template head<3>().all() &&
               (pt.array() <= max_pt_.array()).template head<3>().all();
      };

    auto filter = advanced::FunctorFilter<PointT, decltype(lambda)>(
        lambda, this->extract_removed_indices_);
    filter.setNegative(this->getNegative());
    filter.setKeepOrganized(this->getKeepOrganized());
    filter.setIndices(this->getIndices());
    filter.setInputCloud(this->getInputCloud());
    filter.applyFilter(indices);
    if (this->extract_removed_indices_)
      *removed_indices_ = *filter.getRemovedIndices(); // copy
  }

protected:
  using PCLBase<PointT>::input_;
  using PCLBase<PointT>::indices_;
  using Filter<PointT>::filter_name_;
  using FilterIndices<PointT>::negative_;
  using FilterIndices<PointT>::keep_organized_;
  using FilterIndices<PointT>::user_filter_value_;
  using FilterIndices<PointT>::extract_removed_indices_;
  using FilterIndices<PointT>::removed_indices_;

private:
  /** \brief The minimum point of the box. */
  Eigen::Vector4f min_pt_ = Eigen::Vector4f(-1, -1, -1, 1);

  /** \brief The maximum point of the box. */
  Eigen::Vector4f max_pt_ = Eigen::Vector4f(1, 1, 1, 1);

  /** \brief The 3D rotation for the box. */
  Eigen::Vector3f rotation_ = Eigen::Vector3f::Zero();

  /** \brief The 3D translation for the box. */
  Eigen::Vector3f translation_ = Eigen::Vector3f::Zero();

  /** \brief The affine transform applied to the cloud. */
  Eigen::Affine3f transform_ = Eigen::Affine3f::Identity();
};
} // namespace experimental
} // namespace pcl