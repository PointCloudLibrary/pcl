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

#ifndef PCL_SURFACE_IMPL_MARCHING_CUBES_HOPPE_H_
#define PCL_SURFACE_IMPL_MARCHING_CUBES_HOPPE_H_

#include <pcl/surface/marching_cubes_hoppe.h>
#include <pcl/common/common.h>
#include <pcl/common/vector_average.h>
#include <pcl/Vertices.h>
#include <pcl/kdtree/kdtree_flann.h>

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointNT>
pcl::MarchingCubesHoppe<PointNT>::MarchingCubesHoppe ()
  : MarchingCubes<PointNT> ()
{
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointNT>
pcl::MarchingCubesHoppe<PointNT>::~MarchingCubesHoppe ()
{
}


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointNT> void
pcl::MarchingCubesHoppe<PointNT>::voxelizeData ()
{
  for (int x = 0; x < res_x_; ++x)
    for (int y = 0; y < res_y_; ++y)
      for (int z = 0; z < res_z_; ++z)
      {
        std::vector<int> nn_indices;
        std::vector<float> nn_sqr_dists;

        Eigen::Vector3f point;
        point[0] = min_p_[0] + (max_p_[0] - min_p_[0]) * float (x) / float (res_x_);
        point[1] = min_p_[1] + (max_p_[1] - min_p_[1]) * float (y) / float (res_y_);
        point[2] = min_p_[2] + (max_p_[2] - min_p_[2]) * float (z) / float (res_z_);

        PointNT p;
        p.getVector3fMap () = point;

        tree_->nearestKSearch (p, 1, nn_indices, nn_sqr_dists);

        grid_[x * res_y_*res_z_ + y * res_z_ + z] = input_->points[nn_indices[0]].getNormalVector3fMap ().dot (
            point - input_->points[nn_indices[0]].getVector3fMap ());
      }
}



#define PCL_INSTANTIATE_MarchingCubesHoppe(T) template class PCL_EXPORTS pcl::MarchingCubesHoppe<T>;

#endif    // PCL_SURFACE_IMPL_MARCHING_CUBES_HOPPE_H_

