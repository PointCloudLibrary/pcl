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

#ifndef PCL_SURFACE_IMPL_MARCHING_CUBES_GREEDY_H_
#define PCL_SURFACE_IMPL_MARCHING_CUBES_GREEDY_H_

#include <pcl/surface/marching_cubes_greedy.h>
//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointNT>
pcl::MarchingCubesGreedy<PointNT>::MarchingCubesGreedy ()
{
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointNT>
pcl::MarchingCubesGreedy<PointNT>::~MarchingCubesGreedy ()
{
}

template <typename PointNT> void
pcl::MarchingCubesGreedy<PointNT>::voxelizeData()
{
  for (size_t cp = 0; cp < input_->points.size(); ++cp)
  {
    // Check if the point is invalid
    if (!pcl_isfinite (input_->points[cp].x) ||
        !pcl_isfinite (input_->points[cp].y) ||
        !pcl_isfinite (input_->points[cp].z))
      continue;

    Eigen::Vector3i index_3d;
    MarchingCubes<PointNT>::getCellIndex (input_->points[cp].getVector4fMap (), index_3d);
    uint64_t index_1d = MarchingCubes<PointNT>::getIndexIn1D (index_3d);
    Leaf cell_data;
    for (int i = 0; i < 8; ++i)
      cell_data.vertex[i] = 1;

    cell_hash_map_[index_1d] = cell_data;

    // the vertices are shared by 8 voxels, so we need to update all 8 of them
    HashMap neighbor_list;
    this->getNeighborList1D (cell_data, index_3d, neighbor_list);
    BOOST_FOREACH (typename HashMap::value_type entry, neighbor_list)
    {
      Eigen::Vector3i i3d;
      this->getIndexIn3D(entry.first, i3d);
      // if the neighbor doesn't exist, add it, otherwise we need to do an OR operation on the vertices
      if (cell_hash_map_.find (entry.first) == cell_hash_map_.end ())
      {
        cell_hash_map_[entry.first] = entry.second;
      }
      else
      {
        for (int i = 0; i < 8; ++i)
        {
          if (entry.second.vertex[i] > 0)
            cell_hash_map_[entry.first].vertex[i] = entry.second.vertex[i];
        }
      }
    }
  }
}

#define PCL_INSTANTIATE_MarchingCubesGreedy(T) template class PCL_EXPORTS pcl::MarchingCubesGreedy<T>;

#endif    // PCL_SURFACE_IMPL_MARCHING_CUBES_GREEDY_H_

