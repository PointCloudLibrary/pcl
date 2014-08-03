/*
 *  Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009-2012, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
 *  Copyright (c) Alexandru-Eugen Ichim
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of the copyright holder(s) nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
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
 */

#include <pcl/registration/correspondence_rejection_organized_boundary.h>

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::registration::CorrespondenceRejectionOrganizedBoundary::getRemainingCorrespondences (const pcl::Correspondences& original_correspondences,
                                                                                          pcl::Correspondences& remaining_correspondences)
{
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud = boost::static_pointer_cast<pcl::registration::DataContainer<pcl::PointXYZ, pcl::PointNormal> >(data_container_)->getInputTarget ();

  if (!cloud->isOrganized ())
  {
    PCL_ERROR ("[pcl::registration::CorrespondenceRejectionOrganizedBoundary::getRemainingCorrespondences] The target cloud is not organized.\n");
    remaining_correspondences.clear ();
    return;
  }

  remaining_correspondences.reserve (original_correspondences.size ());
  for (size_t c_i = 0; c_i < original_correspondences.size (); ++c_i)
  {
    /// Count how many NaNs bound the target point
    int x = original_correspondences[c_i].index_match % cloud->width;
    int y = original_correspondences[c_i].index_match / cloud->width;

    int nan_count_tgt = 0;
    for (int x_d = -window_size_/2; x_d <= window_size_/2; ++x_d)
      for (int y_d = -window_size_/2; y_d <= window_size_/2; ++y_d)
        if (x + x_d >= 0 && x + x_d < static_cast<int> (cloud->width) &&
            y + y_d >= 0 && y + y_d < static_cast<int> (cloud->height))
        {
          if (!pcl_isfinite ((*cloud)(x + x_d, y + y_d).z) ||
              fabs ((*cloud)(x, y).z - (*cloud)(x + x_d, y + y_d).z) > depth_step_threshold_)
            nan_count_tgt ++;
        }

    if (nan_count_tgt >= boundary_nans_threshold_)
      continue;


    /// The correspondence passes both tests, add it to the filtered set of correspondences
    remaining_correspondences.push_back (original_correspondences[c_i]);
  }
}
