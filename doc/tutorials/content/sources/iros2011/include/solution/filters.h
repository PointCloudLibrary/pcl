/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception, Inc.
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
 */


#ifndef FILTERS_H
#define FILTERS_H

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>

#include "typedefs.h"

/* Use a PassThrough filter to remove points with depth values that are too large or too small */
PointCloudPtr
thresholdDepth (const PointCloudPtr & input, float min_depth, float max_depth)
{
  pcl::PassThrough<PointT> pass_through;
  pass_through.setInputCloud (input);
  pass_through.setFilterFieldName ("z");
  pass_through.setFilterLimits (min_depth, max_depth);
  PointCloudPtr thresholded (new PointCloud);
  pass_through.filter (*thresholded);

  return (thresholded);
}

/* Use a VoxelGrid filter to reduce the number of points */
PointCloudPtr
downsample (const PointCloudPtr & input, float leaf_size)
{
  pcl::VoxelGrid<PointT> voxel_grid;
  voxel_grid.setInputCloud (input);
  voxel_grid.setLeafSize (leaf_size, leaf_size, leaf_size);
  PointCloudPtr downsampled (new PointCloud);
  voxel_grid.filter (*downsampled);

  return (downsampled);
}

/* Use a RadiusOutlierRemoval filter to remove all points with too few local neighbors */
PointCloudPtr
removeOutliers (const PointCloudPtr & input, float radius, int min_neighbors)
{
  pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> radius_outlier_removal;
  radius_outlier_removal.setInputCloud (input);
  radius_outlier_removal.setRadiusSearch (radius);
  radius_outlier_removal.setMinNeighborsInRadius (min_neighbors);
  PointCloudPtr inliers (new PointCloud);
  radius_outlier_removal.filter (*inliers);

  return (inliers);
}

/* Apply a series of filters (threshold depth, downsample, and remove outliers) */
PointCloudPtr
applyFilters (const PointCloudPtr & input, float min_depth, float max_depth, float leaf_size, float radius, 
              float min_neighbors)
{
  PointCloudPtr filtered;
  filtered = thresholdDepth (input, min_depth, max_depth);
  filtered = downsample (filtered, leaf_size);
  filtered = removeOutliers (filtered, radius, min_neighbors);

  return (filtered);
}

#endif
