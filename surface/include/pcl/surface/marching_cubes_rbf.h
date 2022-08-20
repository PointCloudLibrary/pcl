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
  /** \brief The marching cubes surface reconstruction algorithm, using a signed distance function based on radial
    * basis functions. Partially based on:
    * Carr J.C., Beatson R.K., Cherrie J.B., Mitchell T.J., Fright W.R., McCallum B.C. and Evans T.R.,
    * "Reconstruction and representation of 3D objects with radial basis functions"
    * SIGGRAPH '01
    *
    * \note This algorithm in its current implementation may not be suitable for very
    * large point clouds, due to high memory requirements.
    * \author Alexandru E. Ichim
    * \ingroup surface
    */
  template <typename PointNT>
  class MarchingCubesRBF : public MarchingCubes<PointNT>
  {
    public:
      using Ptr = shared_ptr<MarchingCubesRBF<PointNT> >;
      using ConstPtr = shared_ptr<const MarchingCubesRBF<PointNT> >;

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
      MarchingCubesRBF (const float off_surface_epsilon = 0.1f,
                        const float percentage_extend_grid = 0.0f,
                        const float iso_level = 0.0f) :
        MarchingCubes<PointNT> (percentage_extend_grid, iso_level),
        off_surface_epsilon_ (off_surface_epsilon)
      {
      }

      /** \brief Destructor. */
      ~MarchingCubesRBF () override;

      /** \brief Convert the point cloud into voxel data.
        */
      void
      voxelizeData () override;


      /** \brief Set the off-surface points displacement value.
        * \param[in] epsilon the value
        */
      inline void
      setOffSurfaceDisplacement (float epsilon)
      { off_surface_epsilon_ = epsilon; }

      /** \brief Get the off-surface points displacement value. */
      inline float
      getOffSurfaceDisplacement ()
      { return off_surface_epsilon_; }


    protected:
      /** \brief the Radial Basis Function kernel. */
      double
      kernel (Eigen::Vector3d c, Eigen::Vector3d x);

      /** \brief The off-surface displacement value. */
      float off_surface_epsilon_;

    public:
      PCL_MAKE_ALIGNED_OPERATOR_NEW
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/surface/impl/marching_cubes_rbf.hpp>
#endif
