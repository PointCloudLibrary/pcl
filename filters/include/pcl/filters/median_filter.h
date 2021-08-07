/*
 * Software License Agreement (BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2012-, Open Perception, Inc.

 * All rights reserved.

 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 * * Neither the name of the copyright holder(s) nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

#pragma once

#include <pcl/filters/filter.h>

namespace pcl
{
  /** \brief Implementation of the median filter.
    * The median filter is one of the simplest and wide-spread image processing filters. It is known to perform well
    * with "shot"/impulse noise (some individual pixels having extreme values), it does not reduce contrast across steps
    * in the function (as compared to filters based on averaging), and it is robust to outliers. Furthermore, it is
    * simple to implement and efficient, as it requires a single pass over the image. It consists of a moving window of
    * fixed size that replaces the pixel in the center with the median inside the window.
    *
    * \note This algorithm filters only the depth (z-component) of _organized_ and untransformed (i.e., in camera coordinates)
    * point clouds. An error will be outputted if an unorganized cloud is given to the class instance.
    *
    * \author Alexandru E. Ichim
    * \ingroup filters
    */
  template <typename PointT>
  class MedianFilter : public pcl::Filter<PointT>
  {
      using pcl::Filter<PointT>::input_;
      using PointCloud = typename pcl::Filter<PointT>::PointCloud;

    public:
      /** \brief Empty constructor. */
      MedianFilter ()
        : window_size_ (5)
        , max_allowed_movement_ (std::numeric_limits<float>::max ())
      { }

      /** \brief Set the window size of the filter.
        * \param[in] window_size the new window size
        */
      inline void
      setWindowSize (int window_size)
      { window_size_ = window_size; }

      /** \brief Get the window size of the filter.
        * \returns the window size of the filter
        */
      inline int
      getWindowSize () const
      { return window_size_; }

      /** \brief Set the largest value one dexel is allowed to move
        * \param[in] max_allowed_movement maximum value a dexel is allowed to move during filtering
        */
      inline void
      setMaxAllowedMovement (float max_allowed_movement)
      { max_allowed_movement_ = max_allowed_movement; }

      /** \brief Get the maximum distance one point is allowed to move along the z-axis.
        * \returns the maximum distance a dexel is allowed to move
        */
      inline float
      getMaxAllowedMovement () const
      { return max_allowed_movement_; }

      /** \brief Filter the input data and store the results into output.
        * \param[out] output the result point cloud
        */
      void
      applyFilter (PointCloud &output) override;

    protected:
      int window_size_;
      float max_allowed_movement_;
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/filters/impl/median_filter.hpp>
#else
#define PCL_INSTANTIATE_MedianFilter(T) template class PCL_EXPORTS pcl::MedianFilter<T>;
#endif
