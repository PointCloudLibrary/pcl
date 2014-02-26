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

#ifndef PCL_FILTERS_VOXEL_GRID_MINIMUM_H_
#define PCL_FILTERS_VOXEL_GRID_MINIMUM_H_

#include <pcl/filters/boost.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <map>

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

      typedef typename FilterIndices<PointT>::PointCloud PointCloud;

    public:
      /** \brief Empty constructor. */
      GridMinimum () :
        leaf_size_ (Eigen::Vector4f::Zero ()),
        inverse_leaf_size_ (Eigen::Array4f::Zero ()),
        min_b_ (Eigen::Vector4i::Zero ()),
        max_b_ (Eigen::Vector4i::Zero ()),
        div_b_ (Eigen::Vector4i::Zero ()),
        divb_mul_ (Eigen::Vector4i::Zero ()),
        min_points_per_grid_ (0)
      {
        filter_name_ = "GridMinimum";
      }

      /** \brief Destructor. */
      virtual ~GridMinimum ()
      {
      }

      /** \brief Set the grid leaf size.
        * \param[in] leaf_size the grid leaf size
        */
      inline void
      setLeafSize (const Eigen::Vector4f &leaf_size)
      {
        leaf_size_ = leaf_size;
        // Avoid division errors
        if (leaf_size_[2] == 0)
          leaf_size_[2] = 1;
        if (leaf_size_[3] == 0)
          leaf_size_[3] = 1;
        // Use multiplications instead of divisions
        inverse_leaf_size_ = Eigen::Array4f::Ones () / leaf_size_.array ();
      }

      /** \brief Set the grid leaf size.
        * \param[in] lx the leaf size for X
        * \param[in] ly the leaf size for Y
        */
      inline void
      setLeafSize (float lx, float ly)
      {
        leaf_size_[0] = lx; leaf_size_[1] = ly;
        // Avoid division errors
        if (leaf_size_[2] == 0)
          leaf_size_[2] = 1;
        if (leaf_size_[3] == 0)
          leaf_size_[3] = 1;
        // Use multiplications instead of divisions
        inverse_leaf_size_ = Eigen::Array4f::Ones () / leaf_size_.array ();
      }

      /** \brief Get the grid leaf size. */
      inline Eigen::Vector3f
      getLeafSize () { return (leaf_size_.head<3> ()); }

      /** \brief Set the minimum number of points required for a grid cell to be used.
        * \param[in] min_points_per_voxel the minimum number of points for required for a cell to be used
        */
      inline void
      setMinimumPointsNumberPerGrid (unsigned int min_points_per_grid) { min_points_per_grid_ = min_points_per_grid; }

      /** \brief Return the minimum number of points required for a grid cell to be used.
       */
      inline unsigned int
      getMinimumPointsNumberPerGrid () { return min_points_per_grid_; }

      /** \brief Get the minimum coordinates of the bounding box (after
        * filtering is performed).
        */
      inline Eigen::Vector3i
      getMinBoxCoordinates () { return (min_b_.head<3> ()); }

      /** \brief Get the maximum coordinates of the bounding box (after
        * filtering is performed).
        */
      inline Eigen::Vector3i
      getMaxBoxCoordinates () { return (max_b_.head<3> ()); }

      /** \brief Get the number of divisions along all 3 axes (after filtering
        * is performed).
        */
      inline Eigen::Vector3i
      getNrDivisions () { return (div_b_.head<3> ()); }

      /** \brief Get the multipliers to be applied to the grid coordinates in
        * order to find the centroid index (after filtering is performed).
        */
      inline Eigen::Vector3i
      getDivisionMultiplier () { return (divb_mul_.head<3> ()); }

    protected:
      /** \brief The size of a leaf. */
      Eigen::Vector4f leaf_size_;

      /** \brief Internal leaf sizes stored as 1/leaf_size_ for efficiency reasons. */
      Eigen::Array4f inverse_leaf_size_;

      /** \brief Minimum number of points per grid cell for the minimum to be computed */
      unsigned int min_points_per_grid_;

      /** \brief The minimum and maximum bin coordinates, the number of divisions, and the division multiplier. */
      Eigen::Vector4i min_b_, max_b_, div_b_, divb_mul_;

      /** \brief Downsample a Point Cloud using a 2D grid approach
        * \param[out] output the resultant point cloud message
        */
      void
      applyFilter (PointCloud &output);

      /** \brief Filtered results are indexed by an indices array.
        * \param[out] indices The resultant indices.
        */
      void
      applyFilter (std::vector<int> &indices)
      {
        applyFilterIndices (indices);
      }

      /** \brief Filtered results are indexed by an indices array.
        * \param[out] indices The resultant indices.
        */
      void
      applyFilterIndices (std::vector<int> &indices);

  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/filters/impl/grid_minimum.hpp>
#endif

#endif  //#ifndef PCL_FILTERS_VOXEL_GRID_MINIMUM_H_

