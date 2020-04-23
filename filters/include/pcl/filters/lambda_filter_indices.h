/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2020-, Open Perception
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

#pragma once

#include <pcl/filters/filter_indices.h>
#include <pcl/pcl_macros.h>

#ifndef __cpp_lib_is_invocable
#include <boost/hof/is_invocable.hpp>
#endif

#include <type_traits>
#include <utility>

namespace pcl {
namespace detail {
#ifndef __cpp_lib_is_invocable
template <typename F, typename... Args>
constexpr bool is_invocable_v = boost::is_invocable<F, Args...>();
#else
using std::is_invocable_v;
#endif

#ifndef __cpp_lib_remove_cvref
template <typename T>
using remove_cvref_t = std::remove_cv_t<std::remove_reference_t<T>>;
#else
using std::remove_cvref_t;
#endif

template <typename PointT, typename Function>
using PointFilterLambda = std::enable_if_t<
    is_invocable_v<Function, const remove_cvref_t<std::PointT>&, pcl::index_t> &&
        std::is_convertible<decltype(Function(const remove_cvref_t<std::PointT>&,
                                              pcl::index_t)),
                            bool>::value,
    bool>;

// can't use this for SFINAE since Derived isn't properly defined
// but this can be used after the class definition to test it
template <class Base, Derived>
constexpr auto IsValidLambdaFilter = std::enable_if_t<
    std::is_base_of<Base, Derived>::value &&
        is_invocable_v<std::declval<Derived>().get_lambda, void> &&
        std::is_same<PointFilterLambda<std::declval<Derived>().get_lambda()>, bool>,
    bool>;

/**
 * \brief LambdaFilterIndices filters point clouds and indices based on a function
 *        pointer passed to filter command
 * \ingroup filters
 */
template <typename PointT, typename Derived>
struct LambdaFilterIndicesImpl : public FilterIndices<PointT> {
protected:
  using PointCloud = typename FilterIndices<PointT>::PointCloud;
  using PointCloudPtr = typename PointCloud::Ptr;
  using PointCloudConstPtr = typename PointCloud::ConstPtr;
  using FieldList = typename pcl::traits::fieldList<PointT>::type;

protected:
  // to prevent instantiation of impl class
  LambdaFilterIndices(bool extract_removed_indices)
  : FilterIndices<PointT>(extract_removed_indices)
  {}
  /** \brief Filtered results are indexed by an indices array.
   * \param[out] indices The resultant indices.
   */
  void
  applyFilter(std::vector<int>& indices) override
  {
    indices.clear();
    indices.reserve(input_->points.size());
    if (extract_removed_indices_) {
      reserved_indices_->clear();
      reserved_indices_->reserve(input_->points.size());
    }

    const auto& lambda = static_cast<Derived>(this)->get_lambda();

    for (const auto index : *indices_) {
      // lambda returns true for points that shoud be selected
      if (!(this->negative_) == lambda(input_->points[index], index)) {
        indices.push_back(index);
      }
      else {
        if (extract_removed_indices_) {
          reserved_indices_.push_back(index);
        }
      }
    }
  }
};
} // namespace detail

template <typename PointT, typename Functor>
struct LambdaFilterIndices
: public detail::LambdaFilterIndicesImpl<LambdaFilterIndices<PointT, Functor>> {
protected:
  // using in type would complicate signature
  static_assert(std::is_same<detail::PointFilterLambda<PointT, Functor>, bool>::value,
                "Functor needs to be able to satisfy the callable constraint");

  using Self = LambdaFilterIndices<PointT, Functor>;
  using Base = detail::LambdaFilterIndicesImpl<Self>;

public:
  using FunctorT = Functor;

  /** \brief Constructor.
   * \param[in] extract_removed_indices Set to true if you want to be able to extract
   * the indices of points being removed (default = false).
   */
  LambdaFilterIndices(FunctorT lambda, bool extract_removed_indices = false)
  : Base(extract_removed_indices),
  {
    filter_name_ = "lambda_filter_indices";
  }

protected:
  FunctorT&
  get_lambda() const noexcept
  {
    static auto lambda = FunctorT{};
    return lambda;
  }
};

// version specialized to hold a callable of the right type
template <typename PointT>
struct LambdaFilterIndices<PointT, std::function<bool(const PointT&, index_t)>>
: public LambdaFilterIndicesImpl<
      LambdaFilterIndices<PointT, std::function<bool(const PointT&, index_t)>>> {
protected:
  static_assert(
      std::is_same <
          PointFilterLambda<std::function<bool(const PointT&, index_t)>, bool>::value,
      "Specialized version needs to satisfy the base criteria");
  using Self = LambdaFilterIndices<PointT>;
  using Base = detail::LambdaFilterIndicesImpl<Self>;

public:
  using FunctorT = std::function<bool(const PointT&, index_t)>;
  /**
   * \brief Constructor
   * \param[in] functor functor which is used for evaluating condition in loop
   * \param[in] extract_removed_indices Set to true if you want to be able to extract
   * the indices of points being removed (default = false).
   */
  LambdaFilterIndices(FunctorT functor, bool extract_removed_indices = false)
  : Base(extract_removed_indices),
  {
    filter_name_ = "lambda_filter_indices";
  }

protected:
  FunctorT&
  get_lambda() const noexcept
  {
    return lambda_;
  }

  FunctorT lambda_;
};
