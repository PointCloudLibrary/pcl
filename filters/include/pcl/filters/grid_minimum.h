/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009-2012, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
 *  Copyright (c) 2014, RadiantBlue Technologies, Inc.
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

#include <pcl/filters/filter.h>
#include <pcl/filters/filter_indices.h>

namespace pcl
{
  /** \brief GridMinimum assembles a local 2D grid over a given PointCloud, and downsamples the data.
    *
    * The GridMinimum class creates a *2D grid* over the input point cloud
    * data. Then, in each *cell* (i.e., 2D grid element), all the points
    * present will be *downsampled* with the minimum z value. This grid minimum
    * can be useful in a number of topographic processing tasks such as crudely
    * estimating ground returns, especially under foliage.
    *
    * \author Bradley J Chambers
    * \ingroup filters
    */
  template <typename PointT>
  class GridMinimum: public FilterIndices<PointT>
  {
    protected:
      using Filter<PointT>::filter_name_;
      using Filter<PointT>::getClassName;
      using Filter<PointT>::input_;
      using Filter<PointT>::indices_;

      using PointCloud = typename FilterIndices<PointT>::PointCloud;

    public:
      /** \brief Empty constructor. */
      GridMinimum (const float resolution)
      {
        setResolution (resolution);
        filter_name_ = "GridMinimum";
      }

      /** \brief Destructor. */
      ~GridMinimum ()
      {
      }

      /** \brief Set the grid resolution.
        * \param[in] resolution the grid resolution
        */
      inline void
      setResolution (const float resolution)
      {
        resolution_ = resolution;
        // Use multiplications instead of divisions
        inverse_resolution_ = 1.0f / resolution_;
      }

      /** \brief Get the grid resolution. */
      inline float
      getResolution () { return (resolution_); }

    protected:
      /** \brief The resolution. */
      float resolution_;

      /** \brief Internal resolution stored as 1/resolution_ for efficiency reasons. */
      float inverse_resolution_;

      /** \brief Downsample a Point Cloud using a 2D grid approach
        * \param[out] output the resultant point cloud message
        */
      void
      applyFilter (PointCloud &output) override;

      /** \brief Filtered results are indexed by an indices array.
        * \param[out] indices The resultant indices.
        */
      void
      applyFilter (Indices &indices) override
      {
        applyFilterIndices (indices);
      }

      /** \brief Filtered results are indexed by an indices array.
        * \param[out] indices The resultant indices.
        */
      void
      applyFilterIndices (Indices &indices);

  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/filters/impl/grid_minimum.hpp>
#endif
