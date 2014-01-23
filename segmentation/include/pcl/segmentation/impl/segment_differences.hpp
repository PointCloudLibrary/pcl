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

#ifndef PCL_SEGMENTATION_IMPL_SEGMENT_DIFFERENCES_H_
#define PCL_SEGMENTATION_IMPL_SEGMENT_DIFFERENCES_H_

#include <pcl/segmentation/segment_differences.h>
#include <pcl/common/io.h>

//////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::getPointCloudDifference (
    const pcl::PointCloud<PointT> &src, 
    const pcl::PointCloud<PointT> &, 
    double threshold, const boost::shared_ptr<pcl::search::Search<PointT> > &tree,
    pcl::PointCloud<PointT> &output)
{
  // We're interested in a single nearest neighbor only
  std::vector<int> nn_indices (1);
  std::vector<float> nn_distances (1);

  // The src indices that do not have a neighbor in tgt
  std::vector<int> src_indices;

  // Iterate through the source data set
  for (int i = 0; i < static_cast<int> (src.points.size ()); ++i)
  {
    if (!isFinite (src.points[i]))
      continue;
    // Search for the closest point in the target data set (number of neighbors to find = 1)
    if (!tree->nearestKSearch (src.points[i], 1, nn_indices, nn_distances))
    {
      PCL_WARN ("No neighbor found for point %zu (%f %f %f)!\n", i, src.points[i].x, src.points[i].y, src.points[i].z);
      continue;
    }

    if (nn_distances[0] > threshold)
      src_indices.push_back (i);
  }
 
  // Allocate enough space and copy the basics
  output.points.resize (src_indices.size ());
  output.header   = src.header;
  output.width    = static_cast<uint32_t> (src_indices.size ());
  output.height   = 1;
  //if (src.is_dense)
    output.is_dense = true;
  //else
    // It's not necessarily true that is_dense is false if cloud_in.is_dense is false
    // To verify this, we would need to iterate over all points and check for NaNs
    //output.is_dense = false;

  // Copy all the data fields from the input cloud to the output one
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
  if (!tree_)
  {
    if (target_->isOrganized ())
      tree_.reset (new pcl::search::OrganizedNeighbor<PointT> ());
    else
      tree_.reset (new pcl::search::KdTree<PointT> (false));
  }
  // Send the input dataset to the spatial locator
  tree_->setInputCloud (target_);

  getPointCloudDifference (*input_, *target_, distance_threshold_, tree_, output);

  deinitCompute ();
}

#define PCL_INSTANTIATE_SegmentDifferences(T) template class PCL_EXPORTS pcl::SegmentDifferences<T>;
#define PCL_INSTANTIATE_getPointCloudDifference(T) template PCL_EXPORTS void pcl::getPointCloudDifference<T>(const pcl::PointCloud<T> &, const pcl::PointCloud<T> &, double, const boost::shared_ptr<pcl::search::Search<T> > &, pcl::PointCloud<T> &);

#endif        // PCL_SEGMENTATION_IMPL_SEGMENT_DIFFERENCES_H_

