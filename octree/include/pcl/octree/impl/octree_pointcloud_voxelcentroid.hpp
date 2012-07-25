/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
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
 * $Id: octree_pointcloud_voxelcentroid.hpp 6459 2012-07-18 07:50:37Z dpb $
 */

#ifndef PCL_OCTREE_VOXELCENTROID_HPP
#define PCL_OCTREE_VOXELCENTROID_HPP

#include <pcl/octree/octree_pointcloud_voxelcentroid.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafT, typename BranchT> unsigned int
pcl::octree::OctreePointCloudVoxelCentroid<PointT, LeafT, BranchT>::getVoxelCentroids (
    typename pcl::octree::OctreePointCloud<PointT, LeafT, BranchT>::AlignedPointTVector &voxel_centroid_list_arg)
{
  size_t i;
  unsigned int point_counter;
  OctreeKey key_c, key_p;
  PointT mean_point;
  PointT idx_point;

  std::vector<int> indices_vector;

  voxel_centroid_list_arg.clear ();
  voxel_centroid_list_arg.reserve (this->leafCount_);

  // serialize leafs - this returns a list of point indices. Points indices from the same voxel are locates next to each other within this vector.
  this->serializeLeafs (indices_vector);

  // initializing
  key_p.x = key_p.y = key_p.z = std::numeric_limits<unsigned int>::max ();
  mean_point.x = mean_point.y = mean_point.z = 0.0;
  point_counter = 0;

  // iterate over all point indices
  for (i = 0; i < indices_vector.size (); i++)
  {
    idx_point = this->input_->points[indices_vector[i]];

    // get octree key for point (key specifies octree voxel)
    this->genOctreeKeyforPoint (idx_point, key_c);

    if (key_c == key_p)
    {
      // key addresses same voxel - add point
      mean_point.x += idx_point.x;
      mean_point.y += idx_point.y;
      mean_point.z += idx_point.z;

      point_counter++;
    }
    else
    {
      // voxel key did change - calculate centroid and push it to result vector
      if (point_counter > 0)
      {
        mean_point.x /= float (point_counter);
        mean_point.y /= float (point_counter);
        mean_point.z /= float (point_counter);

        voxel_centroid_list_arg.push_back (mean_point);
      }

      // reset centroid to current input point
      mean_point.x = idx_point.x;
      mean_point.y = idx_point.y;
      mean_point.z = idx_point.z;
      point_counter = 1;

      key_p = key_c;
    }
  }

  // push last centroid to result vector if necessary
  if (point_counter > 0)
  {
    mean_point.x /= float (point_counter);
    mean_point.y /= float (point_counter);
    mean_point.z /= float (point_counter);

    voxel_centroid_list_arg.push_back (mean_point);
  }

  // return size of centroid vector
  return (static_cast<unsigned int> (voxel_centroid_list_arg.size ()));
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafT, typename BranchT> bool
pcl::octree::OctreePointCloudVoxelCentroid<PointT, LeafT, BranchT>::getVoxelCentroidAtPoint (
    const PointT& point_arg, PointT& voxel_centroid_arg)
{
  size_t i;
  unsigned int point_counter;
  std::vector<int> indices_vector;
  PointT mean_point;
  PointT idx_point;

  bool b_result;

  // get all point indixes from voxel at point point_arg
  b_result = this->voxelSearch (point_arg, indices_vector);

  if (b_result)
  {
    mean_point.x = mean_point.y = mean_point.z = 0.0;
    point_counter = 0;

    // iterate over all point indices
    for (i = 0; i < indices_vector.size (); i++)
    {
      idx_point = this->input_->points[indices_vector[i]];

      mean_point.x += idx_point.x;
      mean_point.y += idx_point.y;
      mean_point.z += idx_point.z;

      point_counter++;
    }

    // calculate centroid
    voxel_centroid_arg.x = mean_point.x / float (point_counter);
    voxel_centroid_arg.y = mean_point.y / float (point_counter);
    voxel_centroid_arg.z = mean_point.z / float (point_counter);
  }

  return (b_result);
}

#define PCL_INSTANTIATE_OctreePointCloudVoxelCentroid(T) template class PCL_EXPORTS pcl::octree::OctreePointCloudVoxelCentroid<T>;

#endif

