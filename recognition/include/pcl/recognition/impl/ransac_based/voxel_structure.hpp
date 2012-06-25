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

template<class T> void
pcl::recognition::VoxelStructure<T>::build (const double bounds[6], int num_of_voxels[3])
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
  spacing_[0] = (bounds[1]-bounds[0])/static_cast<double>(num_of_voxels[0]);
  spacing_[1] = (bounds[3]-bounds[2])/static_cast<double>(num_of_voxels[1]);
  spacing_[2] = (bounds[5]-bounds[4])/static_cast<double>(num_of_voxels[2]);

  // Compute the center of the voxel with integer coordinates (0, 0, 0)
  min_center_[0] = bounds_[0] + 0.5*spacing_[0];
  min_center_[1] = bounds_[2] + 0.5*spacing_[1];
  min_center_[2] = bounds_[4] + 0.5*spacing_[2];
}

#endif // PCL_RECOGNITION_VOXEL_STRUCTURE_HPP_
