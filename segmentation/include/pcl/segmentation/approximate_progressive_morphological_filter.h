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
 */

#pragma once

#include <pcl/pcl_base.h>
#include <pcl/search/search.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace pcl
{
  /** \brief
    * Implements the Progressive Morphological Filter for segmentation of ground points.
    * Description can be found in the article
    * "A Progressive Morphological Filter for Removing Nonground Measurements from
    * Airborne LIDAR Data"
    * by K. Zhang, S. Chen, D. Whitman, M. Shyu, J. Yan, and C. Zhang.
    */
  template <typename PointT>
  class PCL_EXPORTS ApproximateProgressiveMorphologicalFilter : public pcl::PCLBase<PointT>
  {
    public:

      using PointCloud = pcl::PointCloud<PointT>;

      using PCLBase <PointT>::input_;
      using PCLBase <PointT>::indices_;
      using PCLBase <PointT>::initCompute;
      using PCLBase <PointT>::deinitCompute;

    public:

      /** \brief Constructor that sets default values for member variables. */
      ApproximateProgressiveMorphologicalFilter ();

      
      ~ApproximateProgressiveMorphologicalFilter () override;

      /** \brief Get the maximum window size to be used in filtering ground returns. */
      inline int
      getMaxWindowSize () const { return (max_window_size_); }

      /** \brief Set the maximum window size to be used in filtering ground returns. */
      inline void
      setMaxWindowSize (int max_window_size) { max_window_size_ = max_window_size; }

      /** \brief Get the slope value to be used in computing the height threshold. */
      inline float
      getSlope () const { return (slope_); }

      /** \brief Set the slope value to be used in computing the height threshold. */
      inline void
      setSlope (float slope) { slope_ = slope; }

      /** \brief Get the maximum height above the parameterized ground surface to be considered a ground return. */
      inline float
      getMaxDistance () const { return (max_distance_); }
      
      /** \brief Set the maximum height above the parameterized ground surface to be considered a ground return. */
      inline void
      setMaxDistance (float max_distance) { max_distance_ = max_distance; }

      /** \brief Get the initial height above the parameterized ground surface to be considered a ground return. */
      inline float
      getInitialDistance () const { return (initial_distance_); }

      /** \brief Set the initial height above the parameterized ground surface to be considered a ground return. */
      inline void
      setInitialDistance (float initial_distance) { initial_distance_ = initial_distance; }

      /** \brief Get the cell size. */
      inline float
      getCellSize () const { return (cell_size_); }
      
      /** \brief Set the cell size. */
      inline void
      setCellSize (float cell_size) { cell_size_ = cell_size; }

      /** \brief Get the base to be used in computing progressive window sizes. */
      inline float
      getBase () const { return (base_); }

      /** \brief Set the base to be used in computing progressive window sizes. */
      inline void
      setBase (float base) { base_ = base; }

      /** \brief Get flag indicating whether or not to exponentially grow window sizes? */
      inline bool
      getExponential () const { return (exponential_); }

      /** \brief Set flag indicating whether or not to exponentially grow window sizes? */
      inline void
      setExponential (bool exponential) { exponential_ = exponential; }

      /** \brief Initialize the scheduler and set the number of threads to use.
        * \param nr_threads the number of hardware threads to use (0 sets the value back to automatic)
        */
      inline void
      setNumberOfThreads (unsigned int nr_threads = 0) { threads_ = nr_threads; }

      /** \brief This method launches the segmentation algorithm and returns indices of
        * points determined to be ground returns.
        * \param[out] ground indices of points determined to be ground returns.
        */
      virtual void
      extract (Indices& ground);

    protected:

      /** \brief Maximum window size to be used in filtering ground returns. */
      int max_window_size_;

      /** \brief Slope value to be used in computing the height threshold. */
      float slope_;

      /** \brief Maximum height above the parameterized ground surface to be considered a ground return. */
      float max_distance_;

      /** \brief Initial height above the parameterized ground surface to be considered a ground return. */
      float initial_distance_;

      /** \brief Cell size. */
      float cell_size_;

      /** \brief Base to be used in computing progressive window sizes. */
      float base_;

      /** \brief Exponentially grow window sizes? */
      bool exponential_;

      /** \brief Number of threads to be used. */
      unsigned int threads_;      
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/segmentation/impl/approximate_progressive_morphological_filter.hpp>
#endif
