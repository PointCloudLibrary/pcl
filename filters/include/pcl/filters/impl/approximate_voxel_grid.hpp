/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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
 * $Id: voxel_grid.hpp 1600 2011-07-07 16:55:51Z shapovalov $
 *
 */

#ifndef PCL_FILTERS_IMPL_FAST_VOXEL_GRID_H_
#define PCL_FILTERS_IMPL_FAST_VOXEL_GRID_H_

#include <pcl/common/common.h>
#include <pcl/common/io.h>
#include <pcl/filters/approximate_voxel_grid.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::ApproximateVoxelGrid<PointT>::flush (PointCloud &output, size_t op, he *hhe, int rgba_index, int centroid_size)
{
  hhe->centroid /= static_cast<float> (hhe->count);
  pcl::for_each_type <FieldList> (pcl::xNdCopyEigenPointFunctor <PointT> (hhe->centroid, output.points[op]));
  // ---[ RGB special case
  if (rgba_index >= 0)
  {
    // pack r/g/b into rgb
    float r = hhe->centroid[centroid_size-3], 
          g = hhe->centroid[centroid_size-2], 
          b = hhe->centroid[centroid_size-1];
    int rgb = (static_cast<int> (r)) << 16 | (static_cast<int> (g)) << 8 | (static_cast<int> (b));
    memcpy (reinterpret_cast<char*> (&output.points[op]) + rgba_index, &rgb, sizeof (float));
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::ApproximateVoxelGrid<PointT>::applyFilter (PointCloud &output)
{
  int centroid_size = 4;
  if (downsample_all_data_)
    centroid_size = boost::mpl::size<FieldList>::value;

  // ---[ RGB special case
  std::vector<pcl::PCLPointField> fields;
  int rgba_index = -1;
  rgba_index = pcl::getFieldIndex (*input_, "rgb", fields);
  if (rgba_index == -1)
    rgba_index = pcl::getFieldIndex (*input_, "rgba", fields);
  if (rgba_index >= 0)
  {
    rgba_index = fields[rgba_index].offset;
    centroid_size += 3;
  }

  for (size_t i = 0; i < histsize_; i++) 
  {
    history_[i].count = 0;
    history_[i].centroid = Eigen::VectorXf::Zero (centroid_size);
  }
  Eigen::VectorXf scratch = Eigen::VectorXf::Zero (centroid_size);

  output.points.resize (input_->points.size ());   // size output for worst case
  size_t op = 0;    // output pointer
  for (size_t cp = 0; cp < input_->points.size (); ++cp) 
  {
    int ix = static_cast<int> (floor (input_->points[cp].x * inverse_leaf_size_[0]));
    int iy = static_cast<int> (floor (input_->points[cp].y * inverse_leaf_size_[1]));
    int iz = static_cast<int> (floor (input_->points[cp].z * inverse_leaf_size_[2]));
    unsigned int hash = static_cast<unsigned int> ((ix * 7171 + iy * 3079 + iz * 4231) & (histsize_ - 1));
    he *hhe = &history_[hash];
    if (hhe->count && ((ix != hhe->ix) || (iy != hhe->iy) || (iz != hhe->iz))) 
    {
      flush (output, op++, hhe, rgba_index, centroid_size);
      hhe->count = 0;
      hhe->centroid.setZero ();// = Eigen::VectorXf::Zero (centroid_size);
    }
    hhe->ix = ix;
    hhe->iy = iy;
    hhe->iz = iz;
    hhe->count++;

    // Unpack the point into scratch, then accumulate
    // ---[ RGB special case
    if (rgba_index >= 0)
    {
      // fill r/g/b data
      pcl::RGB rgb;
      memcpy (&rgb, (reinterpret_cast<const char *> (&input_->points[cp])) + rgba_index, sizeof (RGB));
      scratch[centroid_size-3] = rgb.r;
      scratch[centroid_size-2] = rgb.g;
      scratch[centroid_size-1] = rgb.b;
    }
    pcl::for_each_type <FieldList> (xNdCopyPointEigenFunctor <PointT> (input_->points[cp], scratch));
    hhe->centroid += scratch;
  }
  for (size_t i = 0; i < histsize_; i++) 
  {
    he *hhe = &history_[i];
    if (hhe->count)
      flush (output, op++, hhe, rgba_index, centroid_size);
  }
  output.points.resize (op);
  output.width = static_cast<uint32_t> (output.points.size ());
  output.height       = 1;                    // downsampling breaks the organized structure
  output.is_dense     = false;                 // we filter out invalid points
}

#define PCL_INSTANTIATE_ApproximateVoxelGrid(T) template class PCL_EXPORTS pcl::ApproximateVoxelGrid<T>;

#endif    // PCL_FILTERS_IMPL_FAST_VOXEL_GRID_H_
