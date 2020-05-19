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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
 * $Id: point_cloud.h 4696 2012-02-23 06:12:55Z rusu $
 *
 */

#pragma once

#include <pcl/search/search.h>
#include <pcl/tracking/tracking.h>
#include <pcl/memory.h>
#include <pcl/pcl_base.h>
#include <pcl/pcl_macros.h>

namespace pcl {
namespace tracking {
/** \brief @b Tracker represents the base tracker class.
 * \author Ryohei Ueda
 * \ingroup tracking
 */
template <typename PointInT, typename StateT>
class Tracker : public PCLBase<PointInT> {
protected:
  using PCLBase<PointInT>::deinitCompute;

public:
  using PCLBase<PointInT>::indices_;
  using PCLBase<PointInT>::input_;

  using BaseClass = PCLBase<PointInT>;
  using Ptr = shared_ptr<Tracker<PointInT, StateT>>;
  using ConstPtr = shared_ptr<const Tracker<PointInT, StateT>>;

  using SearchPtr = typename pcl::search::Search<PointInT>::Ptr;
  using SearchConstPtr = typename pcl::search::Search<PointInT>::ConstPtr;

  using PointCloudIn = pcl::PointCloud<PointInT>;
  using PointCloudInPtr = typename PointCloudIn::Ptr;
  using PointCloudInConstPtr = typename PointCloudIn::ConstPtr;

  using PointCloudState = pcl::PointCloud<StateT>;
  using PointCloudStatePtr = typename PointCloudState::Ptr;
  using PointCloudStateConstPtr = typename PointCloudState::ConstPtr;

public:
  /** \brief Empty constructor. */
  Tracker() : search_() {}

  /** \brief Base method for tracking for all points given in
   * <setInputCloud (), setIndices ()> using the indices in setIndices ()
   */
  void
  compute();

protected:
  /** \brief The tracker name. */
  std::string tracker_name_;

  /** \brief A pointer to the spatial search object. */
  SearchPtr search_;

  /** \brief Get a string representation of the name of this class. */
  inline const std::string&
  getClassName() const
  {
    return (tracker_name_);
  }

  /** \brief This method should get called before starting the actual
   * computation. */
  virtual bool
  initCompute();

  /** \brief Provide a pointer to a dataset to add additional information
   * to estimate the features for every point in the input dataset.  This
   * is optional, if this is not set, it will only use the data in the
   * input cloud to estimate the features.  This is useful when you only
   * need to compute the features for a downsampled cloud.
   * \param search a pointer to a PointCloud message
   */
  inline void
  setSearchMethod(const SearchPtr& search)
  {
    search_ = search;
  }

  /** \brief Get a pointer to the point cloud dataset. */
  inline SearchPtr
  getSearchMethod()
  {
    return (search_);
  }

  /** \brief Get an instance of the result of tracking. */
  virtual StateT
  getResult() const = 0;

private:
  /** \brief Abstract tracking method. */
  virtual void
  computeTracking() = 0;

public:
  PCL_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace tracking
} // namespace pcl

#include <pcl/tracking/impl/tracker.hpp>
