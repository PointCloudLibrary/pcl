///
/// Copyright (c) 2012, Texas A&M University
/// All rights reserved.
///
/// Redistribution and use in source and binary forms, with or without
/// modification, are permitted provided that the following conditions
/// are met:
///
///  * Redistributions of source code must retain the above copyright
///    notice, this list of conditions and the following disclaimer.
///  * Redistributions in binary form must reproduce the above
///    copyright notice, this list of conditions and the following
///    disclaimer in the documentation and/or other materials provided
///    with the distribution.
///  * Neither the name of Texas A&M University nor the names of its
///    contributors may be used to endorse or promote products derived
///    from this software without specific prior written permission.
///
/// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
/// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
/// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
/// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
/// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
/// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
/// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
/// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
/// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
/// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
/// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
/// POSSIBILITY OF SUCH DAMAGE.
///
/// The following software was written as part of a collaboration with the
/// University of South Carolina, Interdisciplinary Mathematics Institute.
///

/// @file denoiseCommand.cpp
/// @details the implementation of the class DenoiseCommand
/// @author Yue Li and Matthew Hielsberg

#include <pcl/PointIndices.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/apps/point_cloud_editor/denoiseCommand.h>
#include <pcl/apps/point_cloud_editor/selection.h>
#include <pcl/apps/point_cloud_editor/cloud.h>

void
DenoiseCommand::execute ()
{
  Cloud3D temp_cloud;
  // denoise
  // uses point neighborhood statistics to filter outlier data.
  // For a more detailed explanation, see PCL's tutorial on denoising:
  // http://pointclouds.org/documentation/tutorials/statistical_outlier.php
  pcl::StatisticalOutlierRemoval<Point3D> filter(true);
  filter.setInputCloud(cloud_ptr_->getInternalCloud().makeShared());
  filter.setMeanK(mean_);
  filter.setStddevMulThresh (threshold_);
  // filtering and back up
  filter.setNegative(false);
  filter.filter(temp_cloud);
  // back up the removed indices.
  pcl::IndicesConstPtr indices_ptr = filter.getRemovedIndices();
  std::vector<int>::const_iterator it;
  for(it = indices_ptr->begin(); it != indices_ptr->end(); ++it)
    removed_indices_.addIndex(static_cast<unsigned int>(*it));
  // back up the removed points.
  removed_points_.set(cloud_ptr_, removed_indices_);
  // remove the noisy points.
  cloud_ptr_->remove(removed_indices_);
  selection_ptr_->clear();
}

void
DenoiseCommand::undo ()
{
  cloud_ptr_->restore(removed_points_, removed_indices_);
}
