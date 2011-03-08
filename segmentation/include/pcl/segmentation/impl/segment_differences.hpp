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
 * $Id: segment_differences.hpp 34694 2010-12-12 06:52:05Z rusu $
 *
 */

#ifndef PCL_SEGMENTATION_IMPL_SEGMENT_DIFFERENCES_H_
#define PCL_SEGMENTATION_IMPL_SEGMENT_DIFFERENCES_H_

#include "pcl/segmentation/segment_differences.h"
#include "pcl/io/io.h"

//////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::getPointCloudDifference (
    const pcl::PointCloud<PointT> &src, 
    const pcl::PointCloud<PointT> &tgt, 
    double threshold, const boost::shared_ptr<pcl::KdTree<PointT> > &tree,
    pcl::PointCloud<PointT> &output)
{
  // We're interested in a single nearest neighbor only
  std::vector<int> nn_indices (1);
  std::vector<float> nn_distances (1);

  // The src indices that do not have a neighbor in tgt
  std::vector<int> src_indices;

  // Iterate through the source data set
  for (size_t i = 0; i < src.points.size (); ++i)
  {
    // Search for the closest point in the target data set (number of neighbors to find = 1)
    if (!tree->nearestKSearch (src.points[i], 1, nn_indices, nn_distances))
    {
      ROS_WARN ("No neighbor found for point %zu (%f %f %f)!", i, src.points[i].x, src.points[i].y, src.points[i].z);
      continue;
    }

    if (nn_distances[0] > threshold)
      src_indices.push_back (i);
  }
 
  copyPointCloud (src, src_indices, output);
}

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
template <typename PointT> void 
pcl::SegmentDifferences<PointT>::segment (PointCloud &output)
{
  output.header = input_->header;

  if (!initCompute ()) 
  {
    output.width = output.height = 0;
    output.points.clear ();
    return;
  }

  // If target is empty, input - target = input
  if (target_->points.empty ())
  {
    output = *input_;
    return;
  }

  // Initialize the spatial locator
  initTree (spatial_locator_, tree_);
  // Send the input dataset to the spatial locator
  tree_->setInputCloud (target_);

  getPointCloudDifference (*input_, *target_, distance_threshold_, tree_, output);

  deinitCompute ();
}

#define PCL_INSTANTIATE_SegmentDifferences(T) template class pcl::SegmentDifferences<T>;
#define PCL_INSTANTIATE_getPointCloudDifference(T) template void pcl::getPointCloudDifference<T>(const pcl::PointCloud<T> &, const pcl::PointCloud<T> &, double, const boost::shared_ptr<pcl::KdTree<T> > &, pcl::PointCloud<T> &);

#endif        // PCL_SEGMENTATION_IMPL_SEGMENT_DIFFERENCES_H_

