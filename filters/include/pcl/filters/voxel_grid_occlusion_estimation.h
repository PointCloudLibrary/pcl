/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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
 * Author : Christian Potthast
 * Email  : potthast@usc.edu
 *
 */

#ifndef PCL_FILTERS_VOXEL_GRID_OCCLUSION_ESTIMATION_H_
#define PCL_FILTERS_VOXEL_GRID_OCCLUSION_ESTIMATION_H_

#include <pcl/filters/voxel_grid.h>

namespace pcl
{
  /** \brief VoxelGrid to estimate occluded space in the scene.
    * The ray traversal algorithm is implemented by the work of 
    * 'John Amanatides and Andrew Woo, A Fast Voxel Traversal Algorithm for Ray Tracing'
    *
    * \author Christian Potthast
    * \ingroup filters
    */
  template <typename PointT>
  class VoxelGridOcclusionEstimation: public VoxelGrid<PointT>
  {
    protected:
      using VoxelGrid<PointT>::min_b_;
      using VoxelGrid<PointT>::max_b_;
      using VoxelGrid<PointT>::div_b_;
      using VoxelGrid<PointT>::leaf_size_;
      using VoxelGrid<PointT>::inverse_leaf_size_;

      typedef typename Filter<PointT>::PointCloud PointCloud;
      typedef typename PointCloud::Ptr PointCloudPtr;
      typedef typename PointCloud::ConstPtr PointCloudConstPtr;

    public:
      /** \brief Empty constructor. */
      VoxelGridOcclusionEstimation ()
      {
        initialized_ = false;
        this->setSaveLeafLayout (true);
      }

      /** \brief Destructor. */
      virtual ~VoxelGridOcclusionEstimation ()
      {
      }

      /** \brief Initialize the voxel grid, needs to be called first
        * Builts the voxel grid and computes additional values for
        * the ray traversal algorithm.
        */
      void
      initializeVoxelGrid ();

      /** \brief Computes the state (free = 0, occluded = 1) of the voxel
        * after utilizing a ray traversal algorithm to a target voxel
        * in (i, j, k) coordinates.
        * \param[out] out_state The state of the voxel.
        * \param[in] in_target_voxel The target voxel coordinate (i, j, k) of the voxel.
        * \return 0 upon success and -1 if an error occurs
        */
      int
      occlusionEstimation (int& out_state,
                           const Eigen::Vector3i& in_target_voxel);

      /** \brief Computes the state (free = 0, occluded = 1) of the voxel
        * after utilizing a ray traversal algorithm to a target voxel
        * in (i, j, k) coordinates. Additionally, this function returns
        * the voxels penetrated of the ray-traversal algorithm till reaching
        * the target voxel.
        * \param[out] out_state The state of the voxel.
        * \param[out] out_ray The voxels penetrated of the ray-traversal algorithm.
        * \param[in] in_target_voxel The target voxel coordinate (i, j, k) of the voxel.
        * \return 0 upon success and -1 if an error occurs
        */
      int
      occlusionEstimation (int& out_state,
                           std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i> >& out_ray,
                           const Eigen::Vector3i& in_target_voxel);

      /** \brief Computes the voxel coordinates (i, j, k) of all occluded
        * voxels in the voxel gird.
        * \param[out] occluded_voxels the coordinates (i, j, k) of all occluded voxels
        * \return 0 upon success and -1 if an error occurs
        */
      int
      occlusionEstimationAll (std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i> >& occluded_voxels);

      /** \brief Returns the voxel grid filtered point cloud
        * \return The voxel grid filtered point cloud
        */
      inline PointCloud
      getFilteredPointCloud () { return filtered_cloud_; }

      
      /** \brief Returns the minimum bounding of coordinates of the voxel grid (x,y,z).
        * \return the minimum coordinates (x,y,z)
        */
      inline Eigen::Vector3f
      getMinBoundCoordinates () { return (b_min_.head<3> ()); }

      /** \brief Returns the maximum bounding of coordinates of the voxel grid (x,y,z).
        * \return the maximum coordinates (x,y,z)
        */
      inline Eigen::Vector3f
      getMaxBoundCoordinates () { return (b_max_.head<3> ()); }

      /** \brief Returns the corresponding centroid (x,y,z) coordinates
        * in the grid of voxel (i,j,k).
        * \param[in] ijk the coordinate (i, j, k) of the voxel
        * \return the (x,y,z) coordinate of the voxel centroid
        */
      inline Eigen::Vector4f
      getCentroidCoordinate (const Eigen::Vector3i& ijk)
      {
        int i,j,k;
        i = ((b_min_[0] < 0) ? (abs (min_b_[0]) + ijk[0]) : (ijk[0] - min_b_[0]));
        j = ((b_min_[1] < 0) ? (abs (min_b_[1]) + ijk[1]) : (ijk[1] - min_b_[1]));
        k = ((b_min_[2] < 0) ? (abs (min_b_[2]) + ijk[2]) : (ijk[2] - min_b_[2]));

        Eigen::Vector4f xyz;
        xyz[0] = b_min_[0] + (leaf_size_[0] * 0.5f) + (static_cast<float> (i) * leaf_size_[0]);
        xyz[1] = b_min_[1] + (leaf_size_[1] * 0.5f) + (static_cast<float> (j) * leaf_size_[1]);
        xyz[2] = b_min_[2] + (leaf_size_[2] * 0.5f) + (static_cast<float> (k) * leaf_size_[2]);
        xyz[3] = 0;
        return xyz;
      }

      // inline void
      // setSensorOrigin (const Eigen::Vector4f origin) { sensor_origin_ = origin; }

      // inline void
      // setSensorOrientation (const Eigen::Quaternionf orientation) { sensor_orientation_ = orientation; }

    protected:

      /** \brief Returns the scaling value (tmin) were the ray intersects with the
        * voxel grid bounding box. (p_entry = origin + tmin * orientation)
        * \param[in] origin The sensor origin
        * \param[in] direction The sensor orientation
        * \return the scaling value
        */
      float
      rayBoxIntersection (const Eigen::Vector4f& origin, 
                          const Eigen::Vector4f& direction);

      /** \brief Returns the state of the target voxel (0 = visible, 1 = occupied)
        * unsing a ray traversal algorithm.
        * \param[in] target_voxel The target voxel in the voxel grid with coordinate (i, j, k).
        * \param[in] origin The sensor origin.
        * \param[in] direction The sensor orientation
        * \param[in] t_min The scaling value (tmin).
        * \return The estimated voxel state.
        */
      int
      rayTraversal (const Eigen::Vector3i& target_voxel,
                    const Eigen::Vector4f& origin, 
                    const Eigen::Vector4f& direction,
                    const float t_min);

      /** \brief Returns the state of the target voxel (0 = visible, 1 = occupied) and
        * the voxels penetrated by the ray unsing a ray traversal algorithm.
        * \param[out] out_ray The voxels penetrated by the ray in (i, j, k) coordinates
        * \param[in] target_voxel The target voxel in the voxel grid with coordinate (i, j, k).
        * \param[in] origin The sensor origin.
        * \param[in] direction The sensor orientation
        * \param[in] t_min The scaling value (tmin).
        * \return The estimated voxel state.
        */
      int
      rayTraversal (std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i> >& out_ray,
                    const Eigen::Vector3i& target_voxel,
                    const Eigen::Vector4f& origin, 
                    const Eigen::Vector4f& direction,
                    const float t_min);

      /** \brief Returns a rounded value. 
        * \param[in] d
        * \return rounded value
        */
      inline float
      round (float d)
      {
        return static_cast<float> (floor (d + 0.5f));
      }

      // We use round here instead of floor due to some numerical issues.
      /** \brief Returns the corresponding (i,j,k) coordinates in the grid of point (x,y,z). 
        * \param[in] x the X point coordinate to get the (i, j, k) index at
        * \param[in] y the Y point coordinate to get the (i, j, k) index at
        * \param[in] z the Z point coordinate to get the (i, j, k) index at
        */
      inline Eigen::Vector3i
      getGridCoordinatesRound (float x, float y, float z) 
      {
        return Eigen::Vector3i (static_cast<int> (round (x * inverse_leaf_size_[0])), 
                                static_cast<int> (round (y * inverse_leaf_size_[1])), 
                                static_cast<int> (round (z * inverse_leaf_size_[2])));
      }

      // initialization flag
      bool initialized_;

      Eigen::Vector4f sensor_origin_;
      Eigen::Quaternionf sensor_orientation_;

      // minimum and maximum bounding box coordinates
      Eigen::Vector4f b_min_, b_max_;

      // voxel grid filtered cloud
      PointCloud filtered_cloud_;
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/filters/impl/voxel_grid_occlusion_estimation.hpp>
#endif

#endif  //#ifndef PCL_FILTERS_VOXEL_GRID_OCCLUSION_ESTIMATION_H_
