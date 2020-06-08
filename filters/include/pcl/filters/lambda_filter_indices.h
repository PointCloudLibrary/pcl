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
#include <pcl/type_traits.h> // for is_invocable

#include <utility>

namespace pcl {
namespace detail {
template <typename PointT, typename Function>
constexpr static bool is_lambda_point_filter_v = pcl::
    is_invocable_r_v<bool, Function, const pcl::remove_cvref_t<PointT>&, pcl::index_t>;

// can't use this for SFINAE since Derived isn't properly defined
// but this can be used after the class definition to test it
/*
template <class Base, class Derived>
constexpr auto IsValidLambdaFilter = std::enable_if_t<
    std::is_base_of<Base, Derived>::value &&
        pcl::is_invocable_v<std::declval<Derived>().get_lambda, void> &&
        is_lambda_point_filter_v<std::declval<Derived>().get_lambda()>,
    bool>, bool>;
*/

/**
 * \brief LambdaFilterIndices filters point clouds and indices based on a
 * function pointer passed to filter command \ingroup filters
 */
template <typename PointT, typename Derived>
class LambdaFilterIndicesImpl : public FilterIndices<PointT> {
private:
  using Base = FilterIndices<PointT>;
  using PCLBase = pcl::PCLBase<PointT>;

protected:
  using FilterIndices<PointT>::PointCloud;
  using PointCloudPtr = typename PointCloud::Ptr;
  using PointCloudConstPtr = typename PointCloud::ConstPtr;
  using FieldList = typename pcl::traits::fieldList<PointT>::type;

  using Base::extract_removed_indices_;
  using Base::negative_;
  using Base::removed_indices_;
  using PCLBase::indices_;
  using PCLBase::input_;

  // to prevent instantiation of impl class
  LambdaFilterIndicesImpl(bool extract_removed_indices)
  : FilterIndices<PointT>(extract_removed_indices)
  {}

  /** \brief Filtered results are indexed by an indices array.
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

    const auto& lambda = static_cast<Derived*>(this)->get_lambda();

    for (const auto index : *indices_) {
      // lambda returns true for points that should be selected
      if (negative_ != lambda((*input_)[index], index)) {
        indices.push_back(index);
      }
      else if (extract_removed_indices_) {
          removed_indices_->push_back(index);
      }
    }
  }
};
} // namespace detail

template <typename PointT, typename Functor>
class LambdaFilterIndices
: public detail::LambdaFilterIndicesImpl<PointT, LambdaFilterIndices<PointT, Functor>> {
private:
  using Self = LambdaFilterIndices<PointT, Functor>;
  using Base = detail::LambdaFilterIndicesImpl<PointT, Self>;

public:
  using FunctorT = Functor;
  // using in type would complicate signature
  static_assert(is_lambda_point_filter_v<PointT, FunctorT>,
                "Functor signature must be similar to `bool(const PointT&, index_t)`");

protected:
  using Base::filter_name_;
  // need to hold a value because lambdas can only be copy or move constructed
  const FunctorT lambda_;

public:
  /** \brief Constructor.
   * \param[in] extract_removed_indices Set to true if you want to be able to
   * extract the indices of points being removed (default = false).
   */
  LambdaFilterIndices(FunctorT lambda, bool extract_removed_indices = false)
  : Base(extract_removed_indices), lambda_(std::move(lambda))
  {
    filter_name_ = "lambda_filter_indices";
  }

  const FunctorT&
  get_lambda() const noexcept
  {
    return lambda_;
  }
};
} // namespace pcl
