/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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
 */

#pragma once

#include <pcl/memory.h>
#include <pcl/pcl_macros.h>
#include <pcl/surface/marching_cubes.h>

namespace pcl
{
   /** \brief The marching cubes surface reconstruction algorithm, using a signed distance function based on the distance
     * from tangent planes, proposed by Hoppe et. al. in:
     * Hoppe H., DeRose T., Duchamp T., MC-Donald J., Stuetzle W., "Surface reconstruction from unorganized points",
     * SIGGRAPH '92
     * \author Alexandru E. Ichim
     * \ingroup surface
     */
  template <typename PointNT>
  class MarchingCubesHoppe : public MarchingCubes<PointNT>
  {
    public:
      using Ptr = shared_ptr<MarchingCubesHoppe<PointNT> >;
      using ConstPtr = shared_ptr<const MarchingCubesHoppe<PointNT> >;

      using SurfaceReconstruction<PointNT>::input_;
      using SurfaceReconstruction<PointNT>::tree_;
      using MarchingCubes<PointNT>::grid_;
      using MarchingCubes<PointNT>::res_x_;
      using MarchingCubes<PointNT>::res_y_;
      using MarchingCubes<PointNT>::res_z_;
      using MarchingCubes<PointNT>::size_voxel_;
      using MarchingCubes<PointNT>::upper_boundary_;
      using MarchingCubes<PointNT>::lower_boundary_;

      using PointCloudPtr = typename pcl::PointCloud<PointNT>::Ptr;

      using KdTree = pcl::KdTree<PointNT>;
      using KdTreePtr = typename KdTree::Ptr;


      /** \brief Constructor. */
      MarchingCubesHoppe (const float dist_ignore = -1.0f,
                          const float percentage_extend_grid = 0.0f,
                          const float iso_level = 0.0f) :
        MarchingCubes<PointNT> (percentage_extend_grid, iso_level),
        dist_ignore_ (dist_ignore)
      {
      }

      /** \brief Destructor. */
      ~MarchingCubesHoppe () override;

      /** \brief Convert the point cloud into voxel data.
        */
      void
      voxelizeData () override;

      /** \brief Method that sets the distance for ignoring voxels which are far from point cloud.
        * If the distance is negative, then the distance functions would be calculated in all voxels;
        * otherwise, only voxels with distance lower than dist_ignore would be involved in marching cube.
        * \param[in] dist_ignore threshold of distance. Default value is -1.0. Set to negative if all voxels are
        * to be involved.
        */
      inline void
      setDistanceIgnore (const float dist_ignore)
      { dist_ignore_ = dist_ignore; }

      /** \brief get the distance for ignoring voxels which are far from point cloud.
       * */
      inline float
      getDistanceIgnore () const
      { return dist_ignore_; }

    protected:
      /** \brief ignore the distance function
       * if it is negative
       * or distance between voxel centroid and point are larger that it. */
      float dist_ignore_;

    public:
      PCL_MAKE_ALIGNED_OPERATOR_NEW
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/surface/impl/marching_cubes_hoppe.hpp>
#endif
