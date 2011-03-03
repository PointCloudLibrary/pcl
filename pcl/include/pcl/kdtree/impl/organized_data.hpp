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
 * $Id: organized_data.hpp 35810 2011-02-08 00:03:46Z rusu $
 *
 */

#ifndef PCL_KDTREE_KDTREE_IMPL_ORGANIZED_DATA_INDEX_H_
#define PCL_KDTREE_KDTREE_IMPL_ORGANIZED_DATA_INDEX_H_

#include "pcl/point_cloud.h"
#include "pcl/kdtree/organized_data.h"

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> int 
pcl::OrganizedDataIndex<PointT>::radiusSearch (
    const PointCloud &cloud, int index, double radius, std::vector<int> &k_indices, 
    std::vector<float> &k_distances, int max_nn) const
{
  k_indices.clear ();
  k_distances.clear ();

  if (cloud.height == 1)
  {
    ROS_ERROR ("[pcl::%s::nearestKSearch] Input dataset is not organized!", getName ().c_str ());
    return 0;
  }
  int data_size = cloud.points.size ();
  if (index >= data_size)
    return 0;

  // Get the cloud's dimensions width
  int width = cloud.width,
      height = cloud.height;

  int y=index/width, x=index-y*width;
  
  const std::vector<PointT, Eigen::aligned_allocator<PointT> >& points = cloud.points;
  const PointT& point = points[index];
  if (!pcl_isfinite(point.x))
    return 0;
  
  // Put point itself into results
  k_indices.push_back(index);
  k_distances.push_back(0.0f);
  
  float max_dist_squared = radius*radius;
  bool still_in_range = true,
       done = false;
  for (int radius=1;  !done;  ++radius) 
  {
    int x2=x-radius-1, y2=y-radius;  // Top left - 1
    still_in_range = false;
    for (int i=0; i<8*radius; ++i)
    {
      if (i<=2*radius) ++x2; else if (i<=4*radius) ++y2; else if (i<=6*radius) --x2; else --y2;
      if (x2<0 || x2>=width || y2<0 || y2>=height)
        continue;
      int neighbor_index = y2*width + x2;
      const PointT& neighbor = points[neighbor_index];
      if (!pcl_isfinite(neighbor.x))
        continue;
      float distance_squared = squaredEuclideanDistance(point, neighbor);
      if (distance_squared > max_dist_squared)
        continue;
      //cout << "Radius "<<radius<<": found "<<neighbor_index << " with distance "<<sqrtf(distance_squared)<<"\n";
      still_in_range = true;
      k_indices.push_back(neighbor_index);
      k_distances.push_back(distance_squared);
      if ((int)k_indices.size() >= max_nn)
      {
        done = true;
        break;
      }
    }
    if (!still_in_range)
      done = true;
  }
  
  return (int(k_indices.size ()));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> int 
pcl::OrganizedDataIndex<PointT>::nearestKSearch (const PointCloud &cloud, int index, int k, 
    std::vector<int> &k_indices, std::vector<float> &k_distances)
{
  k_indices.resize (k);
  if (cloud.height == 1)
  {
    ROS_ERROR ("[pcl::%s::nearestKSearch] Input dataset is not dense!", getName ().c_str ());
    return 0;
  }
  int data_size = cloud.points.size ();
  if (index >= data_size)
    return 0;

  // Get the cloud width
  int width = cloud.width;

  // Obtain the <u,v> pixel values
  int u = index / width;
  int v = index % width;

  int l = -1, idx, uwv = u * width + v, uwvx;

  // Save the query point as the first neighbor (*ANN compatibility)
  k_indices[++l] = index;

  if (horizontal_window_==0 || vertical_window_)
    setSearchWindowAsK (k);

  // Get all point neighbors in a H x V window
  for (int x = -horizontal_window_; x != horizontal_window_; ++x)
  {
    uwvx = uwv + x * width;     // Get the correct index

    for (int y = -vertical_window_; y != vertical_window_; ++y)
    {
      // idx = (u+x) * cloud.width + (v+y);
      idx = uwvx + y;

      // If the index is not in the point cloud, continue
      if (idx == index || idx < 0 || idx >= data_size)
        continue;

      if (max_distance_ != 0)
      {
        if (fabs (cloud.points[index].z - cloud.points[idx].z) < max_distance_)
          k_indices[++l] = idx;
      }
      else
        k_indices[++l] = idx;
    }
  }
  // We need at least min_pts_ nearest neighbors to do something useful with them
  if (l < min_pts_)
    return 0;
  return k;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void 
pcl::OrganizedDataIndex<PointT>::setSearchWindowAsK (int k)
{
  int hw = 0, vw = 0;
  while ( (2 * hw + 1 ) * (2 * vw + 1) < k)
  {
    ++hw; ++vw;
  }
  horizontal_window_ = hw - 1;
  vertical_window_ = vw - 1;
}

#define PCL_INSTANTIATE_OrganizedDataIndex(T) template class pcl::OrganizedDataIndex<T>;

#endif  //#ifndef _PCL_KDTREE_KDTREE_IMPL_ORGANIZED_DATA_INDEX_H_

