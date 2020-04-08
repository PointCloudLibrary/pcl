/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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

namespace pcl {
template <typename T>
inline bool
conditional_reserve(T& container, std::size_t size)
{
  if (container.capacity() < size) {
    container.reserve(size);
    return true;
  }
  return false;
}
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
    conditional_reserve(indices, input_->points.size());
    if (extract_removed_indices_) {
      reserved_indices_->clear();
      conditional_reserve(*reserved_indices_, input_->points.size());
    }

    const auto& lambda = static_cast<Derived>(this)->get_lambda();

    for (const auto index : *indices_) {
      if (!(this->negative_) && lambda(input_->points[index], index)) {
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

template <typename PointT, typename FunctorT>
struct LambdaFilterIndices
: public LambdaFilterIndicesImpl<LambdaFilterIndices<PointT, FunctorT>> {
protected:
  using Self = LambdaFilterIndices<PointT, FunctorT>;
  using Base = LambdaFilterIndicesImpl<Self>;

public:
  /** \brief Constructor.
   * \param[in] extract_removed_indices Set to true if you want to be able to extract
   * the indices of points being removed (default = false).
   */
  LambdaFilterIndices(FunctorT lambda, bool extract_removed_indices = false)
  : LambdaFilterIndicesImpl<PointT>(extract_removed_indices),
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

template <typename PointT>
struct LambdaFilterIndices<PointT, std::function<bool(const PointT&, index_t)>>
: public LambdaFilterIndicesImpl<
      LambdaFilterIndices<PointT, std::function<bool(const PointT&, index_t)>>> {
protected:
  using Self = LambdaFilterIndices<PointT>;
  using Base = LambdaFilterIndicesImpl<Self>;

public:
  using FunctorT = std::function<bool(const PointT&, index_t)>;
  /**
   * \brief Constructor
   * \param[in] functor functor which is used for evaluating condition in loop
   * \param[in] extract_removed_indices Set to true if you want to be able to extract
   * the indices of points being removed (default = false).
   */
  LambdaFilterIndices(FunctorT functor, bool extract_removed_indices = false)
  : LambdaFilterIndicesImpl<PointT>(extract_removed_indices),
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
