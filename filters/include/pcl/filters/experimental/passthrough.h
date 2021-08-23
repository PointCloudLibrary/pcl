/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2021-, Open Perception
 *
 *  All rights reserved
 */

#pragma once

#include <pcl/filters/experimental/functor_filter.h>

#include <limits> // for numeric_limits

namespace pcl {
namespace experimental {

template <typename PointT>
struct PassThroughFunctor {
  PassThroughFunctor() = default;
  PassThroughFunctor(const uindex_t* field_offset,
                     const float* filter_limit_min,
                     const float* filter_limit_max,
                     const bool* has_filter_field,
                     const bool* negative)
  : field_offset_(field_offset)
  , filter_limit_min_(filter_limit_min)
  , filter_limit_max_(filter_limit_max)
  , has_filter_field_(has_filter_field)
  , negative_(negative)
  {}

  bool
  operator()(const PointCloud<PointT>& cloud, index_t idx) const
  {
    // always filter out points with NaN in XYZ fields, regardless of the user's choice
    // of what points should be filtered (normal/inverted)
    if (!*has_filter_field_)
      return !*negative_; // always keep the point in FunctorFilter

    float field_value;
    const std::uint8_t* pt_data = reinterpret_cast<const std::uint8_t*>(&cloud.at(idx));
    memcpy(&field_value, pt_data + *field_offset_, sizeof(float));

    if (!std::isfinite(field_value))
      return *negative_; // always discard the point in FunctorFilter
    else
      return field_value >= *filter_limit_min_ && field_value <= *filter_limit_max_;
  }

private:
  const uindex_t* field_offset_ = nullptr;
  const float* filter_limit_min_ = nullptr;
  const float* filter_limit_max_ = nullptr;
  const bool* has_filter_field_ = nullptr;
  const bool* negative_ = nullptr;
};

template <typename PointT>
using PassThroughFilter = advanced::FunctorFilter<PointT, PassThroughFunctor<PointT>>;

/** \brief @b PassThrough passes points in a cloud based on constraints for one
 * particular field of the point type.
 * \details Iterates through the entire input once, automatically filtering non-finite
 * points and the points outside the interval specified by setFilterLimits(), which
 * applies only to the field specified by setFilterFieldName(). The specified field
 * should have float type.
 * <br><br>
 * Usage example:
 * \code
 * pcl::PassThrough<PointType> ptfilter (true); // Initializing with true will allow us
 * to extract the removed indices
 * ptfilter.setInputCloud (cloud_in);
 * ptfilter.setFilterFieldName ("x");
 * ptfilter.setFilterLimits (0.0, 1000.0);
 * ptfilter.filter (*indices_x);
 * // The indices_x array indexes all points of cloud_in that have x between 0.0 and
 * 1000.0 indices_rem = ptfilter.getRemovedIndices ();
 * // The indices_rem array indexes all points of cloud_in that have x smaller than 0.0
 * or larger than 1000.0
 * // and also indexes all non-finite points of cloud_in
 * ptfilter.setIndices (indices_x);
 * ptfilter.setFilterFieldName ("z");
 * ptfilter.setFilterLimits (-10.0, 10.0);
 * ptfilter.setNegative (true);
 * ptfilter.filter (*indices_xz);
 * // The indices_xz array indexes all points of cloud_in that have x between 0.0 and
 * 1000.0 and z larger than 10.0 or smaller than -10.0 ptfilter.setIndices (indices_xz);
 * ptfilter.setFilterFieldName ("intensity");
 * ptfilter.setFilterLimits (FLT_MIN, 0.5);
 * ptfilter.setNegative (false);
 * ptfilter.filter (*cloud_out);
 * // The resulting cloud_out contains all points of cloud_in that are finite and have:
 * // x between 0.0 and 1000.0, z larger than 10.0 or smaller than -10.0 and intensity
 * smaller than 0.5.
 * \endcode
 * \author Radu Bogdan Rusu
 * \ingroup filters
 */
template <typename PointT>
class PassThrough : public PassThroughFilter<PointT> {
protected:
  using PointCloud = typename FilterIndices<PointT>::PointCloud;
  using PointCloudPtr = typename PointCloud::Ptr;
  using PointCloudConstPtr = typename PointCloud::ConstPtr;
  using FieldList = typename pcl::traits::fieldList<PointT>::type;

public:
  using Ptr = shared_ptr<PassThrough<PointT>>;
  using ConstPtr = shared_ptr<const PassThrough<PointT>>;

  /** \brief Constructor.
   * \param[in] extract_removed_indices Set to true if you want to be able to extract
   * the indices of points being removed (default = false).
   */
  PassThrough(bool extract_removed_indices = false)
  : PassThroughFilter<PointT>(extract_removed_indices)
  {
    filter_name_ = "PassThrough";
    PassThroughFilter<PointT>::setFunctionObject(
        PassThroughFunctor<PointT>(&field_offset_,
                                   &filter_limit_min_,
                                   &filter_limit_max_,
                                   &has_filter_field_,
                                   &negative_));
  }

  /** \brief Provide the name of the field to be used for filtering data.
   * \details In conjunction with setFilterLimits(), points having values outside this
   * interval for this field will be discarded.
   * \param[in] field_name The name of the field that will be used for filtering.
   */
  inline void
  setFilterFieldName(const std::string& field_name)
  {
    filter_field_name_ = field_name;
  }

  /** \brief Retrieve the name of the field to be used for filtering data.
   * \return The name of the field that will be used for filtering.
   */
  inline std::string const
  getFilterFieldName() const
  {
    return filter_field_name_;
  }

  /** \brief Set the numerical limits for the field for filtering data.
   * \details In conjunction with setFilterFieldName(), points having values outside
   * this interval for this field will be discarded.
   * \param[in] limit_min The minimum allowed field value (default = FLT_MIN).
   * \param[in] limit_max The maximum allowed field value (default = FLT_MAX).
   */
  inline void
  setFilterLimits(const float& limit_min, const float& limit_max)
  {
    filter_limit_min_ = limit_min;
    filter_limit_max_ = limit_max;
  }

  /** \brief Get the numerical limits for the field for filtering data.
   * \param[out] limit_min The minimum allowed field value (default = FLT_MIN).
   * \param[out] limit_max The maximum allowed field value (default = FLT_MAX).
   */
  inline void
  getFilterLimits(float& limit_min, float& limit_max) const
  {
    limit_min = filter_limit_min_;
    limit_max = filter_limit_max_;
  }

protected:
  using PCLBase<PointT>::input_;
  using PCLBase<PointT>::indices_;
  using Filter<PointT>::filter_name_;
  using Filter<PointT>::getClassName;
  using FilterIndices<PointT>::negative_;
  using FilterIndices<PointT>::keep_organized_;
  using FilterIndices<PointT>::user_filter_value_;
  using FilterIndices<PointT>::extract_removed_indices_;
  using FilterIndices<PointT>::removed_indices_;

  /** \brief Filtered results are indexed by an indices array.
   * \param[out] indices The resultant indices.
   */
  void
  applyFilter(Indices& indices) override
  {
    has_filter_field_ = !filter_field_name_.empty();
    if (has_filter_field_) {
      // Attempt to get the field name's index
      std::vector<PCLPointField> fields;
      const int distance_idx_ = getFieldIndex<PointT>(filter_field_name_, fields);
      if (distance_idx_ == -1) {
        PCL_WARN("[pcl::%s::applyFilter] Unable to find field name in point type.\n",
                 getClassName().c_str());
        indices.clear();
        removed_indices_->clear();
        return;
      }
      field_offset_ = fields.at(distance_idx_).offset;
    }

    PassThroughFilter<PointT>::applyFilter(indices);
  }

private:
  /** \brief The name of the field that will be used for filtering. */
  std::string filter_field_name_;

  /** \brief The minimum allowed field value (default = FLT_MIN). */
  float filter_limit_min_ = std::numeric_limits<float>::min();

  /** \brief The maximum allowed field value (default = FLT_MIN). */
  float filter_limit_max_ = std::numeric_limits<float>::min();

  /** \brief Whether a filtering field is set. */
  bool has_filter_field_;

  /** \brief The offset of the filtering field.  */
  uindex_t field_offset_;

  using PassThroughFilter<PointT>::getFunctionObject; // hide method
};
} // namespace experimental
} // namespace pcl
