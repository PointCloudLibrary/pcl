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

#ifndef PCL_FILTERS_IMPL_VOXEL_GRID_MINIMUM_H_
#define PCL_FILTERS_IMPL_VOXEL_GRID_MINIMUM_H_

#include <pcl/common/common.h>
#include <pcl/common/io.h>
#include <pcl/filters/grid_minimum.h>

struct point_index_idx
{
  unsigned int idx;
  unsigned int cloud_point_index;

  point_index_idx (unsigned int idx_, unsigned int cloud_point_index_) : idx (idx_), cloud_point_index (cloud_point_index_) {}
  bool operator < (const point_index_idx &p) const { return (idx < p.idx); }
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::GridMinimum<PointT>::applyFilter (PointCloud &output)
{
  // Has the input dataset been set already?
  if (!input_)
  {
    PCL_WARN ("[pcl::%s::applyFilter] No input dataset given!\n", getClassName ().c_str ());
    output.width = output.height = 0;
    output.points.clear ();
    return;
  }

  std::vector<int> indices;

  output.is_dense = true;
  applyFilterIndices (indices);
  pcl::copyPointCloud<PointT> (*input_, indices, output);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::GridMinimum<PointT>::applyFilterIndices (std::vector<int> &indices)
{
  indices.resize (indices_->size ());
  int oii = 0;

  // Get the minimum and maximum dimensions
  Eigen::Vector4f min_p, max_p;
  getMinMax3D<PointT> (*input_, *indices_, min_p, max_p);

  // Check that the resolution is not too small, given the size of the data
  int64_t dx = static_cast<int64_t> ((max_p[0] - min_p[0]) * inverse_resolution_)+1;
  int64_t dy = static_cast<int64_t> ((max_p[1] - min_p[1]) * inverse_resolution_)+1;

  if ((dx*dy) > static_cast<int64_t> (std::numeric_limits<int32_t>::max ()))
  {
    PCL_WARN ("[pcl::%s::applyFilter] Leaf size is too small for the input dataset. Integer indices would overflow.", getClassName ().c_str ());
    return;
  }

  Eigen::Vector4i min_b, max_b, div_b, divb_mul;

  // Compute the minimum and maximum bounding box values
  min_b[0] = static_cast<int> (floor (min_p[0] * inverse_resolution_));
  max_b[0] = static_cast<int> (floor (max_p[0] * inverse_resolution_));
  min_b[1] = static_cast<int> (floor (min_p[1] * inverse_resolution_));
  max_b[1] = static_cast<int> (floor (max_p[1] * inverse_resolution_));

  // Compute the number of divisions needed along all axis
  div_b = max_b - min_b + Eigen::Vector4i::Ones ();
  div_b[3] = 0;

  // Set up the division multiplier
  divb_mul = Eigen::Vector4i (1, div_b[0], 0, 0);

  std::vector<point_index_idx> index_vector;
  index_vector.reserve (indices_->size ());

  // First pass: go over all points and insert them into the index_vector vector
  // with calculated idx. Points with the same idx value will contribute to the
  // same point of resulting CloudPoint
  for (std::vector<int>::const_iterator it = indices_->begin (); it != indices_->end (); ++it)
  {
    if (!input_->is_dense)
      // Check if the point is invalid
      if (!pcl_isfinite (input_->points[*it].x) ||
          !pcl_isfinite (input_->points[*it].y) ||
          !pcl_isfinite (input_->points[*it].z))
        continue;

    int ijk0 = static_cast<int> (floor (input_->points[*it].x * inverse_resolution_) - static_cast<float> (min_b[0]));
    int ijk1 = static_cast<int> (floor (input_->points[*it].y * inverse_resolution_) - static_cast<float> (min_b[1]));

    // Compute the grid cell index
    int idx = ijk0 * divb_mul[0] + ijk1 * divb_mul[1];
    index_vector.push_back (point_index_idx (static_cast<unsigned int> (idx), *it));
  }
  
  // Second pass: sort the index_vector vector using value representing target cell as index
  // in effect all points belonging to the same output cell will be next to each other
  std::sort (index_vector.begin (), index_vector.end (), std::less<point_index_idx> ());

  // Third pass: count output cells
  // we need to skip all the same, adjacenent idx values
  unsigned int total = 0;
  unsigned int index = 0;

  // first_and_last_indices_vector[i] represents the index in index_vector of the first point in
  // index_vector belonging to the voxel which corresponds to the i-th output point,
  // and of the first point not belonging to.
  std::vector<std::pair<unsigned int, unsigned int> > first_and_last_indices_vector;
  
  // Worst case size
  first_and_last_indices_vector.reserve (index_vector.size ());
  while (index < index_vector.size ())
  {
    unsigned int i = index + 1;
    while (i < index_vector.size () && index_vector[i].idx == index_vector[index].idx)
      ++i;
    ++total;
    first_and_last_indices_vector.push_back (std::pair<unsigned int, unsigned int> (index, i));
    index = i;
  }

  // Fourth pass: locate grid minimums
  indices.resize (total);

  index = 0;

  for (unsigned int cp = 0; cp < first_and_last_indices_vector.size (); ++cp)
  {
    unsigned int first_index = first_and_last_indices_vector[cp].first;
    unsigned int last_index = first_and_last_indices_vector[cp].second;
    unsigned int min_index = index_vector[first_index].cloud_point_index;
    float min_z = input_->points[index_vector[first_index].cloud_point_index].z;

    for (unsigned int i = first_index + 1; i < last_index; ++i)
    {
      if (input_->points[index_vector[i].cloud_point_index].z < min_z)
      {
        min_z = input_->points[index_vector[i].cloud_point_index].z;
        min_index = index_vector[i].cloud_point_index;
      }
    }

    indices[index] = min_index;
    
    ++index;
  }

  oii = indices.size ();

  // Resize the output arrays
  indices.resize (oii);
}

#define PCL_INSTANTIATE_GridMinimum(T) template class PCL_EXPORTS pcl::GridMinimum<T>;

#endif    // PCL_FILTERS_IMPL_VOXEL_GRID_MINIMUM_H_

