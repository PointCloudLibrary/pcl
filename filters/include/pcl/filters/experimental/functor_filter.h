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
/**
 * \brief Checks if the function object meets the usage in `FunctorFilter` class
 * \details `Function` needs to be callable with a const reference to a PointCloud
 * and an index value. The return type should be implicitly convertible to a boolean
 */
template <typename PointT, typename Function>
constexpr static bool is_function_object_for_filter_v =
    is_invocable_r_v<bool, Function, const PointCloud<PointT>&, index_t>;

namespace advanced {
/**
 * \brief Filter point clouds and indices based on a function object passed in the ctor
 * \details The function object can be anything (lambda, std::function, invocable class,
 * etc.) that can be moved into the class. Additionally, it must satisfy the condition
 * `is_function_object_for_filter_v`
 * \ingroup filters
 */
template <typename PointT, typename FunctionObject>
class FunctorFilter : public FilterIndices<PointT> {
  using Base = FilterIndices<PointT>;
  using PCL_Base = PCLBase<PointT>;

public:
  using FunctionObjectT = FunctionObject;
  // using in type would complicate signature
  static_assert(is_function_object_for_filter_v<PointT, FunctionObjectT>,
                "Function object signature must be similar to `bool(const "
                "PointCloud<PointT>&, index_t)`");

protected:
  using Base::extract_removed_indices_;
  using Base::filter_name_;
  using Base::negative_;
  using Base::removed_indices_;
  using PCL_Base::indices_;
  using PCL_Base::input_;

private:
  // need to hold a value because lambdas can only be copy or move constructed in C++14
  FunctionObjectT functionObject_;

public:
  /** \brief Constructor.
   * \param[in] function_object Object of effective type `FilterFunction` in order to
   * filter out the indices for which it returns false
   * \param[in] extract_removed_indices Set to true if you want to be able to
   * extract the indices of points being removed (default = false).
   */
  FunctorFilter(FunctionObjectT function_object, bool extract_removed_indices = false)
  : Base(extract_removed_indices), functionObject_(std::move(function_object))
  {
    filter_name_ = "functor_filter";
  }

  const FunctionObjectT&
  getFunctionObject() const noexcept
  {
    return functionObject_;
  }

  FunctionObjectT&
  getFunctionObject() noexcept
  {
    return functionObject_;
  }

  /**
   * \brief Filtered results are indexed by an indices array.
   * \param[out] indices The resultant indices.
   */
  void
  applyFilter(Indices& indices) override
  {
    indices.clear();
    indices.reserve(indices_->size());
    if (extract_removed_indices_) {
      removed_indices_->clear();
      removed_indices_->reserve(indices_->size());
    }

    for (const auto index : *indices_) {
      // function object returns true for points that should be selected
      if (negative_ != functionObject_(*input_, index)) {
        indices.push_back(index);
      }
      else if (extract_removed_indices_) {
        removed_indices_->push_back(index);
      }
    }
  }

protected:
  /**
   * \brief ctor to be used by derived classes with member function as FilterFunction
   * \param[in] extract_removed_indices Set to true if you want to be able to
   * extract the indices of points being removed (default = false).
   * \note The class would be ill-defined until `setFunctionObject` has been called
   * Do not call any filter routine until then
   */
  FunctorFilter(bool extract_removed_indices = false) : Base(extract_removed_indices)
  {
    filter_name_ = "functor_filter";
  }

  /**
   * \brief utility function for derived class
   * \param[in] function_object Object of effective type `FilterFunction` in order to
   * filter out the indices for which it returns false
   */
  void
  setFunctionObject(FunctionObjectT function_object) const noexcept
  {
    functionObject_ = std::move(function_object);
  }
};
} // namespace advanced

template <class PointT>
using FilterFunction = std::function<bool(const PointCloud<PointT>&, index_t)>;

template <class PointT>
using FunctionFilter = advanced::FunctorFilter<PointT, FilterFunction<PointT>>;
} // namespace experimental
} // namespace pcl
