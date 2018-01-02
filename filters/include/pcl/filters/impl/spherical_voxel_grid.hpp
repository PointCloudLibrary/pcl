/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009, Willow Garage, Inc.
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
 * $Id$
 *
 */

#ifndef PCL_FILTERS_SPHERICAL_VOXEL_GRID_IMPL_H_
#define PCL_FILTERS_SPHERICAL_VOXEL_GRID_IMPL_H_

#include <pcl/filters/spherical_voxel_grid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>

template<typename PointT> inline void
pcl::getMaxDistance (const pcl::PointCloud<PointT> &cloud,
                     const std::string &distance_field_name, float min_distance, float max_distance,
                     const Eigen::Vector4f &pivot_pt, Eigen::Vector4f &max_pt, bool limit_negative)
{
  float max_dist = -FLT_MAX;
  int max_idx = -1;
  float dist;
  const Eigen::Vector3f pivot_pt3 = pivot_pt.head<3> ();

  // Get the fields list and the distance field index
  std::vector<pcl::PCLPointField> fields;
  int distance_idx = pcl::getFieldIndex (cloud, distance_field_name, fields);

  float distance_value;

  // If the data is dense, we don't need to check for NaN
  if (cloud.is_dense)
  {
    for (size_t i = 0; i < cloud.points.size (); ++i)
    {
      // Get the distance value
      const uint8_t* pt_data = reinterpret_cast<const uint8_t*> (&cloud.points[i]);
      memcpy (&distance_value, pt_data + fields[distance_idx].offset, sizeof (float));

      if (limit_negative)
      {
        // Use a threshold for cutting out points which inside the interval
        if ((distance_value < max_distance) && (distance_value > min_distance))
          continue;
      }
      else
      {
        // Use a threshold for cutting out points which are too close/far away
        if ((distance_value > max_distance) || (distance_value < min_distance))
          continue;
      }

      pcl::Vector3fMapConst pt = cloud.points[i].getVector3fMap ();
      dist = (pivot_pt3 - pt).norm ();
      if (dist > max_dist)
      {
        max_idx = int (i);
        max_dist = dist;
      }
    }
  }
    // NaN or Inf values could exist => check for them
  else
  {
    for (size_t i = 0; i < cloud.points.size (); ++i)
    {
      // Check if the point is invalid
      if (!pcl_isfinite (cloud.points[i].x) || !pcl_isfinite (cloud.points[i].y) || !pcl_isfinite (cloud.points[i].z))
        continue;

      // Get the distance value
      const uint8_t* pt_data = reinterpret_cast<const uint8_t*> (&cloud.points[i]);
      memcpy (&distance_value, pt_data + fields[distance_idx].offset, sizeof (float));

      if (limit_negative)
      {
        // Use a threshold for cutting out points which inside the interval
        if ((distance_value < max_distance) && (distance_value > min_distance))
          continue;
      }
      else
      {
        // Use a threshold for cutting out points which are too close/far away
        if ((distance_value > max_distance) || (distance_value < min_distance))
          continue;
      }

      pcl::Vector3fMapConst pt = cloud.points[i].getVector3fMap ();
      dist = (pivot_pt3 - pt).norm ();
      if (dist > max_dist)
      {
        max_idx = int (i);
        max_dist = dist;
      }
    }
  }

  if(max_idx != -1)
    max_pt = cloud.points[max_idx].getVector4fMap ();
  else
    max_pt = Eigen::Vector4f(std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::quiet_NaN());
}

template<typename PointT> inline void
pcl::getMaxDistance (const pcl::PointCloud<PointT> &cloud, const std::vector<int> &indices,
                     const std::string &distance_field_name, float min_distance, float max_distance,
                     const Eigen::Vector4f &pivot_pt, Eigen::Vector4f &max_pt, bool limit_negative)
{
  float max_dist = -FLT_MAX;
  int max_idx = -1;
  float dist;
  const Eigen::Vector3f pivot_pt3 = pivot_pt.head<3> ();

  // Get the fields list and the distance field index
  std::vector<pcl::PCLPointField> fields;
  int distance_idx = pcl::getFieldIndex (cloud, distance_field_name, fields);

  float distance_value;

  // If the data is dense, we don't need to check for NaN
  if (cloud.is_dense)
  {
    for (size_t i = 0; i < indices.size (); ++i)
    {
      // Get the distance value
      const uint8_t* pt_data = reinterpret_cast<const uint8_t*> (&cloud.points[indices[i]]);
      memcpy (&distance_value, pt_data + fields[distance_idx].offset, sizeof (float));

      if (limit_negative)
      {
        // Use a threshold for cutting out points which inside the interval
        if ((distance_value < max_distance) && (distance_value > min_distance))
          continue;
      }
      else
      {
        // Use a threshold for cutting out points which are too close/far away
        if ((distance_value > max_distance) || (distance_value < min_distance))
          continue;
      }

      pcl::Vector3fMapConst pt = cloud.points[indices[i]].getVector3fMap ();
      dist = (pivot_pt3 - pt).norm ();
      if (dist > max_dist)
      {
        max_idx = static_cast<int> (i);
        max_dist = dist;
      }
    }
  }
    // NaN or Inf values could exist => check for them
  else
  {
    for (size_t i = 0; i < indices.size (); ++i)
    {
      // Check if the point is invalid
      if (!pcl_isfinite (cloud.points[indices[i]].x) ||
          !pcl_isfinite (cloud.points[indices[i]].y) ||
          !pcl_isfinite (cloud.points[indices[i]].z))
        continue;

      // Get the distance value
      const uint8_t* pt_data = reinterpret_cast<const uint8_t*> (&cloud.points[indices[i]]);
      memcpy (&distance_value, pt_data + fields[distance_idx].offset, sizeof (float));

      if (limit_negative)
      {
        // Use a threshold for cutting out points which inside the interval
        if ((distance_value < max_distance) && (distance_value > min_distance))
          continue;
      }
      else
      {
        // Use a threshold for cutting out points which are too close/far away
        if ((distance_value > max_distance) || (distance_value < min_distance))
          continue;
      }

      pcl::Vector3fMapConst pt = cloud.points[indices[i]].getVector3fMap ();
      dist = (pivot_pt3 - pt).norm ();
      if (dist > max_dist)
      {
        max_idx = static_cast<int> (i);
        max_dist = dist;
      }
    }
  }

  if(max_idx != -1)
    max_pt = cloud.points[indices[max_idx]].getVector4fMap ();
  else
    max_pt = Eigen::Vector4f(std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::quiet_NaN());
}

struct cloud_point_index_idx
{
  unsigned int idx;
  unsigned int cloud_point_index;

  cloud_point_index_idx (unsigned int idx_, unsigned int cloud_point_index_) : idx (idx_), cloud_point_index (cloud_point_index_) {}
  bool operator < (const cloud_point_index_idx &p) const { return (idx < p.idx); }
};

template <typename PointT> void
pcl::SphericalVoxelGrid<PointT>::applyFilter (PointCloud &output)
{
  // Has the input dataset been set already?
  if (!input_)
  {
    PCL_WARN ("[pcl::%s::applyFilter] No input dataset given!\n", getClassName().c_str());
    output.width = output.height = 0;
    output.points.clear();
    return;
  }

  // Copy the header (and thus the frame_id) + allocate enough space for points
  output.height = 1;                    // downsampling breaks the organized structure
  output.is_dense = true;                 // we filter out invalid points

  if (leaf_size_r_ <= 0 || leaf_size_theta_ <= 0 || leaf_size_phi_ <= 0)
  {
    PCL_WARN("[pcl::%s::applyFilter] Leaf size improperly set.", getClassName().c_str());
    output = *input_;
    return;
  }

  // Find the farthest away point, subject to filter parameters if specified
  Eigen::Vector4f max_p;

  if (!filter_field_name_.empty ()) // If we don't want to process the entire cloud...
    getMaxDistance<PointT> (*input_, *indices_, filter_field_name_, static_cast<float> (filter_limit_min_),
                            static_cast<float> (filter_limit_max_), filter_origin_, max_p, filter_limit_negative_);
  else
    getMaxDistance<PointT> (*input_, *indices_, filter_origin_, max_p);

  // Find the number of radial layers required given the farthest point and resolution
  max_radius_ = (max_p - filter_origin_).norm ();
  std::uint64_t rIdxNum = static_cast<std::uint64_t>(std::floor(max_radius_ / leaf_size_r_)) + 1;
  leaf_r_divisions_ =  rIdxNum;

  if (rIdxNum * static_cast<std::uint64_t> (leaf_theta_divisions_) * static_cast<std::uint64_t> (leaf_phi_divisions_)
      > static_cast<std::uint64_t> (std::numeric_limits<int32_t>::max()))
  {
    PCL_WARN("[pcl::%s::applyFilter] Leaf size is too small for the input dataset. Integer indices would overflow.", getClassName().c_str());
    output = *input_;
    return;
  }

  // Storage for mapping leaf and pointcloud indexes
  std::vector<cloud_point_index_idx> index_vector;
  index_vector.reserve (indices_->size ());

  if (!filter_field_name_.empty ())
  {
    // Get the distance field index
    std::vector<pcl::PCLPointField> fields;
    int distance_idx = pcl::getFieldIndex (*input_, filter_field_name_, fields);
    if (distance_idx == -1)
      PCL_WARN ("[pcl::%s::applyFilter] Invalid filter field name. Index is %d.\n", getClassName ().c_str (), distance_idx);

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

      // Get the distance value
      const uint8_t* pt_data = reinterpret_cast<const uint8_t*> (&input_->points[*it]);
      float distance_value = 0;
      memcpy (&distance_value, pt_data + fields[distance_idx].offset, sizeof (float));

      if (filter_limit_negative_)
      {
        // Use a threshold for cutting out points which inside the interval
        if ((distance_value < filter_limit_max_) && (distance_value > filter_limit_min_))
          continue;
      }
      else
      {
        // Use a threshold for cutting out points which are too close/far away
        if ((distance_value > filter_limit_max_) || (distance_value < filter_limit_min_))
          continue;
      }

      PointT shiftedPoint = input_->points[*it];

      shiftedPoint.x -= filter_origin_[0];
      shiftedPoint.y -= filter_origin_[1];
      shiftedPoint.z -= filter_origin_[2];

      // Convert to spherical coordinates
      float r = std::sqrt (shiftedPoint.x * shiftedPoint.x +
                           shiftedPoint.y * shiftedPoint.y +
                           shiftedPoint.z * shiftedPoint.z);

      float theta = std::acos (shiftedPoint.z / r);

      // Convert range of atan2 to ( 0, 2PI ]
      float phi = std::atan2 (shiftedPoint.y, shiftedPoint.x) + M_PI;

      // Get index
      int rIdx = static_cast<int> (std::floor (r / leaf_size_r_));
      int thetaIdx = static_cast<int> (std::floor (theta / leaf_size_theta_));
      int phiIdx = static_cast<int> (std::floor (phi / leaf_size_phi_));

      int idx = ((thetaIdx * leaf_phi_divisions_) + phiIdx) + (leaf_theta_divisions_ * leaf_phi_divisions_ * rIdx);

      index_vector.push_back (cloud_point_index_idx (idx, *it));
    }
  }
  else
  {
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

      PointT shiftedPoint = input_->points[*it];

      shiftedPoint.x -= filter_origin_[0];
      shiftedPoint.y -= filter_origin_[1];
      shiftedPoint.z -= filter_origin_[2];

      // Convert to spherical coordinates
      float r = std::sqrt (shiftedPoint.x * shiftedPoint.x +
                           shiftedPoint.y * shiftedPoint.y +
                           shiftedPoint.z * shiftedPoint.z);

      float theta = std::acos (shiftedPoint.z / r);

      // Convert range of atan2 to ( 0, 2PI ]
      float phi = std::atan2 (shiftedPoint.y, shiftedPoint.x) + M_PI;

      // Get index
      int rIdx = static_cast<int> (std::floor (r / leaf_size_r_));
      int thetaIdx = static_cast<int> (std::floor (theta / leaf_size_theta_));
      int phiIdx = static_cast<int> (std::floor (phi / leaf_size_phi_));

      int idx = ((thetaIdx * leaf_phi_divisions_) + phiIdx) + (leaf_theta_divisions_ * leaf_phi_divisions_ * rIdx);

      index_vector.push_back (cloud_point_index_idx (idx, *it));
    }
  }

  // Second pass: sort the index_vector vector using value representing target cell as index
  // in effect all points belonging to the same output cell will be next to each other
  std::sort (index_vector.begin (), index_vector.end (), std::less<cloud_point_index_idx> ());

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
    if (i - index >= min_points_per_voxel_)
    {
      ++total;
      first_and_last_indices_vector.push_back (std::pair<unsigned int, unsigned int> (index, i));
    }
    index = i;
  }

  // Fourth pass: compute centroids, insert them into their final position
  output.points.resize (total);

  index = 0;
  for (unsigned int cp = 0; cp < first_and_last_indices_vector.size (); ++cp)
  {
    // calculate centroid - sum values from all input points, that have the same idx value in index_vector array
    unsigned int first_index = first_and_last_indices_vector[cp].first;
    unsigned int last_index = first_and_last_indices_vector[cp].second;

    //Limit downsampling to coords
    if (!downsample_all_data_)
    {
      Eigen::Vector4f centroid (Eigen::Vector4f::Zero ());

      for (unsigned int li = first_index; li < last_index; ++li)
        centroid += input_->points[index_vector[li].cloud_point_index].getVector4fMap ();

      centroid /= static_cast<float> (last_index - first_index);
      output.points[index].getVector4fMap () = centroid;
    }
    else
    {
      CentroidPoint<PointT> centroid;

      // fill in the accumulator with leaf points
      for (unsigned int li = first_index; li < last_index; ++li)
        centroid.add (input_->points[index_vector[li].cloud_point_index]);

      centroid.get (output.points[index]);
    }

    ++index;
  }
  output.width = static_cast<uint32_t> (output.points.size ());
}

#define PCL_INSTANTIATE_SphericalVoxelGrid(T) template class PCL_EXPORTS pcl::SphericalVoxelGrid<T>;
#define PCL_INSTANTIATE_getMaxDistance(T) template PCL_EXPORTS void pcl::getMaxDistance<T> (const pcl::PointCloud<T> &, const std::string &, float, float, const Eigen::Vector4f &, Eigen::Vector4f &, bool);


#endif // PCL_FILTERS_SPHERICAL_VOXEL_GRID_IMPL_H_