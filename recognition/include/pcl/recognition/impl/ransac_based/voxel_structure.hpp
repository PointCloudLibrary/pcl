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
 *
 */


#ifndef PCL_RECOGNITION_VOXEL_STRUCTURE_HPP_
#define PCL_RECOGNITION_VOXEL_STRUCTURE_HPP_


namespace pcl
{

namespace recognition
{

template<class T, typename REAL> inline void
VoxelStructure<T,REAL>::build (const REAL bounds[6], int num_of_voxels[3])
{
  this->clear();

  // Copy the bounds
  bounds_[0] = bounds[0];
  bounds_[1] = bounds[1];
  bounds_[2] = bounds[2];
  bounds_[3] = bounds[3];
  bounds_[4] = bounds[4];
  bounds_[5] = bounds[5];

  num_of_voxels_[0] = num_of_voxels[0];
  num_of_voxels_[1] = num_of_voxels[1];
  num_of_voxels_[2] = num_of_voxels[2];
  num_of_voxels_xy_plane_ = num_of_voxels[0]*num_of_voxels[1];
  total_num_of_voxels_ = num_of_voxels_xy_plane_*num_of_voxels[2];

  // Allocate memory for the voxels
  voxels_ = new T[total_num_of_voxels_];

  // Compute the spacing between the voxels in x, y and z direction
  spacing_[0] = (bounds[1]-bounds[0])/static_cast<REAL> (num_of_voxels[0]);
  spacing_[1] = (bounds[3]-bounds[2])/static_cast<REAL> (num_of_voxels[1]);
  spacing_[2] = (bounds[5]-bounds[4])/static_cast<REAL> (num_of_voxels[2]);

  // Compute the center of the voxel with integer coordinates (0, 0, 0)
  min_center_[0] = bounds_[0] + static_cast<REAL> (0.5)*spacing_[0];
  min_center_[1] = bounds_[2] + static_cast<REAL> (0.5)*spacing_[1];
  min_center_[2] = bounds_[4] + static_cast<REAL> (0.5)*spacing_[2];
}


template<class T, typename REAL> inline T*
VoxelStructure<T,REAL>::getVoxel (const REAL p[3])
{
  if ( p[0] < bounds_[0] || p[0] >= bounds_[1] || p[1] < bounds_[2] || p[1] >= bounds_[3] || p[2] < bounds_[4] || p[2] >= bounds_[5] )
    return nullptr;

  int x = static_cast<int> ((p[0] - bounds_[0])/spacing_[0]);
  int y = static_cast<int> ((p[1] - bounds_[2])/spacing_[1]);
  int z = static_cast<int> ((p[2] - bounds_[4])/spacing_[2]);

  return &voxels_[z*num_of_voxels_xy_plane_ + y*num_of_voxels_[0] + x];
}


template<class T, typename REAL> inline T*
VoxelStructure<T,REAL>::getVoxel (int x, int y, int z) const
{
  if ( x < 0 || x >= num_of_voxels_[0] ) return nullptr;
  if ( y < 0 || y >= num_of_voxels_[1] ) return nullptr;
  if ( z < 0 || z >= num_of_voxels_[2] ) return nullptr;

  return &voxels_[z*num_of_voxels_xy_plane_ + y*num_of_voxels_[0] + x];
}


template<class T, typename REAL> inline int
VoxelStructure<T,REAL>::getNeighbors (const REAL* p, T **neighs) const
{
  if ( p[0] < bounds_[0] || p[0] >= bounds_[1] || p[1] < bounds_[2] || p[1] >= bounds_[3] || p[2] < bounds_[4] || p[2] >= bounds_[5] )
    return 0;

  const int x = static_cast<int> ((p[0] - bounds_[0])/spacing_[0]);
  const int y = static_cast<int> ((p[1] - bounds_[2])/spacing_[1]);
  const int z = static_cast<int> ((p[2] - bounds_[4])/spacing_[2]);

  const int x_m1 = x-1, x_p1 = x+1;
  const int y_m1 = y-1, y_p1 = y+1;
  const int z_m1 = z-1, z_p1 = z+1;

  T* voxel;
  int num_neighs = 0;

  voxel = this->getVoxel (x_p1, y_p1, z_p1); if ( voxel ) neighs[num_neighs++] = voxel;
  voxel = this->getVoxel (x_p1, y_p1, z   ); if ( voxel ) neighs[num_neighs++] = voxel;
  voxel = this->getVoxel (x_p1, y_p1, z_m1); if ( voxel ) neighs[num_neighs++] = voxel;
  voxel = this->getVoxel (x_p1, y   , z_p1); if ( voxel ) neighs[num_neighs++] = voxel;
  voxel = this->getVoxel (x_p1, y   , z   ); if ( voxel ) neighs[num_neighs++] = voxel;
  voxel = this->getVoxel (x_p1, y   , z_m1); if ( voxel ) neighs[num_neighs++] = voxel;
  voxel = this->getVoxel (x_p1, y_m1, z_p1); if ( voxel ) neighs[num_neighs++] = voxel;
  voxel = this->getVoxel (x_p1, y_m1, z   ); if ( voxel ) neighs[num_neighs++] = voxel;
  voxel = this->getVoxel (x_p1, y_m1, z_m1); if ( voxel ) neighs[num_neighs++] = voxel;

  voxel = this->getVoxel (x   , y_p1, z_p1); if ( voxel ) neighs[num_neighs++] = voxel;
  voxel = this->getVoxel (x   , y_p1, z   ); if ( voxel ) neighs[num_neighs++] = voxel;
  voxel = this->getVoxel (x   , y_p1, z_m1); if ( voxel ) neighs[num_neighs++] = voxel;
  voxel = this->getVoxel (x   , y   , z_p1); if ( voxel ) neighs[num_neighs++] = voxel;
  voxel = this->getVoxel (x   , y   , z   ); if ( voxel ) neighs[num_neighs++] = voxel;
  voxel = this->getVoxel (x   , y   , z_m1); if ( voxel ) neighs[num_neighs++] = voxel;
  voxel = this->getVoxel (x   , y_m1, z_p1); if ( voxel ) neighs[num_neighs++] = voxel;
  voxel = this->getVoxel (x   , y_m1, z   ); if ( voxel ) neighs[num_neighs++] = voxel;
  voxel = this->getVoxel (x   , y_m1, z_m1); if ( voxel ) neighs[num_neighs++] = voxel;

  voxel = this->getVoxel (x_m1, y_p1, z_p1); if ( voxel ) neighs[num_neighs++] = voxel;
  voxel = this->getVoxel (x_m1, y_p1, z   ); if ( voxel ) neighs[num_neighs++] = voxel;
  voxel = this->getVoxel (x_m1, y_p1, z_m1); if ( voxel ) neighs[num_neighs++] = voxel;
  voxel = this->getVoxel (x_m1, y   , z_p1); if ( voxel ) neighs[num_neighs++] = voxel;
  voxel = this->getVoxel (x_m1, y   , z   ); if ( voxel ) neighs[num_neighs++] = voxel;
  voxel = this->getVoxel (x_m1, y   , z_m1); if ( voxel ) neighs[num_neighs++] = voxel;
  voxel = this->getVoxel (x_m1, y_m1, z_p1); if ( voxel ) neighs[num_neighs++] = voxel;
  voxel = this->getVoxel (x_m1, y_m1, z   ); if ( voxel ) neighs[num_neighs++] = voxel;
  voxel = this->getVoxel (x_m1, y_m1, z_m1); if ( voxel ) neighs[num_neighs++] = voxel;

  return num_neighs;
}

} // namespace recognition
} // namespace pcl

#endif // PCL_RECOGNITION_VOXEL_STRUCTURE_HPP_

