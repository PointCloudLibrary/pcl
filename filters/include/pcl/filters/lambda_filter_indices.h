/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2020-, Open Perception
 *
 *  All rights reserved
 */

#pragma once

#include <pcl/filters/filter_indices.h>
#include <pcl/type_traits.h> // for is_invocable

namespace pcl {
template <typename PointT, typename Function>
constexpr static bool is_lambda_filter_functor_v =
    pcl::is_invocable_r_v<bool,
                          Function,
                          const pcl::remove_cvref_t<pcl::PointCloud<PointT>>&,
                          pcl::index_t>;

/**
 * \brief Filter point clouds and indices based on a functor passed in the ctor
 * \ingroup filters
 */
template <typename PointT, typename Functor>
class LambdaFilter : public FilterIndices<PointT> {
  using Base = FilterIndices<PointT>;
  using PCLBase = pcl::PCLBase<PointT>;

public:
  using FunctorT = Functor;
  // using in type would complicate signature
  static_assert(is_lambda_filter_functor_v<PointT, FunctorT>,
                "Functor signature must be similar to `bool(const PointCloud<PointT>&, "
                "index_t)`");

protected:
  using Base::extract_removed_indices_;
  using Base::filter_name_;
  using Base::negative_;
  using Base::removed_indices_;
  using PCLBase::indices_;
  using PCLBase::input_;

  // need to hold a value because lambdas can only be copy or move constructed
  FunctorT lambda_;

public:
  /** \brief Constructor.
   * \param[in] extract_removed_indices Set to true if you want to be able to
   * extract the indices of points being removed (default = false).
   */
  LambdaFilter(FunctorT lambda, bool extract_removed_indices = false)
  : Base(extract_removed_indices), lambda_(std::move(lambda))
  {
    filter_name_ = "lambda_filter_indices";
  }

  const FunctorT&
  getLambda() const noexcept
  {
    return lambda_;
  }

  /**
   * \brief Filtered results are indexed by an indices array.
   * \param[out] indices The resultant indices.
   */
  void
  applyFilter(Indices& indices) override
  {
    indices.clear();
    indices.reserve(input_->size());
    if (extract_removed_indices_) {
      removed_indices_->clear();
      removed_indices_->reserve(input_->size());
    }

    for (const auto index : *indices_) {
      // lambda returns true for points that should be selected
      if (negative_ != lambda_(*input_, index)) {
        indices.push_back(index);
      }
      else if (extract_removed_indices_) {
        removed_indices_->push_back(index);
      }
    }
  }
};
} // namespace pcl
