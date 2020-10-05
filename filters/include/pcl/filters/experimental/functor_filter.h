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
namespace experimental {
template <typename PointT, typename Function>
constexpr static bool is_functor_for_filter_v =
    pcl::is_invocable_r_v<bool,
                          Function,
                          const pcl::remove_cvref_t<pcl::PointCloud<PointT>>&,
                          pcl::index_t>;

/**
 * \brief Filter point clouds and indices based on a functor passed in the ctor
 * \ingroup filters
 */
template <typename PointT, typename Functor>
class FunctorFilter : public FilterIndices<PointT> {
  using Base = FilterIndices<PointT>;
  using PCLBase = pcl::PCLBase<PointT>;

public:
  using FunctorT = Functor;
  // using in type would complicate signature
  static_assert(is_functor_for_filter_v<PointT, FunctorT>,
                "Functor signature must be similar to `bool(const PointCloud<PointT>&, "
                "index_t)`");

protected:
  using Base::extract_removed_indices_;
  using Base::filter_name_;
  using Base::negative_;
  using Base::removed_indices_;
  using PCLBase::indices_;
  using PCLBase::input_;

  // need to hold a value because functors can only be copy or move constructed
  FunctorT functor_;

public:
  /** \brief Constructor.
   * \param[in] extract_removed_indices Set to true if you want to be able to
   * extract the indices of points being removed (default = false).
   */
  FunctorFilter(FunctorT functor, bool extract_removed_indices = false)
  : Base(extract_removed_indices), functor_(std::move(functor))
  {
    filter_name_ = "functor_filter";
  }

  const FunctorT&
  getFunctor() const noexcept
  {
    return functor_;
  }

  FunctorT&
  getFunctor() noexcept
  {
    return functor_;
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
      // functor returns true for points that should be selected
      if (negative_ != functor_(*input_, index)) {
        indices.push_back(index);
      }
      else if (extract_removed_indices_) {
        removed_indices_->push_back(index);
      }
    }
  }
};
} // namespace experimental
} // namespace pcl
