/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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
 *
 *
 */

#ifndef PCL_FEATURES_IMPL_ORGANIZED_EDGE_DETECTION_H_
#define PCL_FEATURES_IMPL_ORGANIZED_EDGE_DETECTION_H_

#include <pcl/features/organized_edge_detection.h>

/**
 *  Directions: 1 2 3
 *              0 x 4
 *              7 6 5
 * e.g. direction y means we came from pixel with label y to the center pixel x
 */
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename PointLT> void
pcl::OrganizedEdgeDetection<PointT, PointLT>::compute (pcl::PointCloud<PointLT>& labels, std::vector<pcl::PointIndices>& label_indices) const
{
  const unsigned invalid_label = std::numeric_limits<unsigned>::max ();
  pcl::Label invalid_pt;
  invalid_pt.label = std::numeric_limits<unsigned>::max ();
  labels.points.resize (input_->points.size (), invalid_pt);
  labels.width = input_->width;
  labels.height = input_->height;
  unsigned int clust_id = 0;

  std::cout << "width: " << labels.width << std::endl;
  std::cout << "height: " << labels.height << std::endl;
  std::cout << "[done] OrganizedEdgeDetection::compute ()" << std::endl;

  // fill lookup table for next points to visit
  const int num_of_ngbr = 8;
  Neighbor directions [num_of_ngbr] = {Neighbor(-1,  0,                -1),
                                       Neighbor(-1, -1, -labels.width - 1), 
                                       Neighbor( 0, -1, -labels.width    ),
                                       Neighbor( 1, -1, -labels.width + 1),
                                       Neighbor( 1,  0,                 1),
                                       Neighbor( 1,  1,  labels.width + 1),
                                       Neighbor( 0,  1,  labels.width    ),
                                       Neighbor(-1,  1,  labels.width - 1)};
  
  for (int row = 1; row < int(input_->height) - 1; row++)
  {
    for (int col = 1; col < int(input_->width) - 1; col++)
    {
      int curr_idx = row*int(input_->width) + col;
      if (!pcl_isfinite (input_->points[curr_idx].z))
        continue;

      float curr_depth = fabs (input_->points[curr_idx].z);

      // Calculate depth distances between current point and neighboring points
      std::vector<float> nghr_dist;
      nghr_dist.resize (8);
      bool found_invalid_neighbor = false;
      for (int d_idx = 0; d_idx < num_of_ngbr; d_idx++)
      {
        int nghr_idx = curr_idx + directions[d_idx].d_index;
        assert (nghr_idx >= 0 && nghr_idx < input_->points.size ());
        if (!pcl_isfinite (input_->points[nghr_idx].z))
        {
          found_invalid_neighbor = true;
          break;
        }
        nghr_dist[d_idx] = curr_depth - fabs (input_->points[nghr_idx].z);
      }

      if (!found_invalid_neighbor)
      {
        // Every neighboring points are valid
        std::vector<float>::iterator min_itr = std::min_element (nghr_dist.begin (), nghr_dist.end ());
        std::vector<float>::iterator max_itr = std::max_element (nghr_dist.begin (), nghr_dist.end ());
        float nghr_dist_min = *min_itr;
        float nghr_dist_max = *max_itr;
        float dist_dominant = fabs (nghr_dist_min) > fabs (nghr_dist_max) ? nghr_dist_min : nghr_dist_max;
        if (fabs (dist_dominant) > th_depth_discon_*fabs (curr_depth))
        {
          // Found a depth discontinuity
          if (dist_dominant > 0.f)
            labels[curr_idx].label = EDGELABEL_OCCLUDED;
          else
            labels[curr_idx].label = EDGELABEL_OCCLUDING;
        }
      }
      else
      {
        // Some neighboring points are not valid (nan points)
        // Search for corresponding point across invalid points
        // Search direction is determined by nan point locations with respect to current point
        int dx = 0;
        int dy = 0;
        int num_of_invalid_pt = 0;
        for (int d_idx = 0; d_idx < num_of_ngbr; d_idx++)
        {
          int nghr_idx = curr_idx + directions[d_idx].d_index;
          assert (nghr_idx >= 0 && nghr_idx < input_->points.size ());
          if (!pcl_isfinite (input_->points[nghr_idx].z))
          {
            dx += directions[d_idx].d_x;
            dy += directions[d_idx].d_y;
            num_of_invalid_pt++;
          }
        }

        // Search directions
        assert (num_of_invalid_pt > 0);
        float f_dx = static_cast<float> (dx) / static_cast<float> (num_of_invalid_pt);
        float f_dy = static_cast<float> (dy) / static_cast<float> (num_of_invalid_pt);

        // Search for corresponding point across invalid points
        float corr_depth = std::numeric_limits<float>::quiet_NaN ();
        for (int s_idx = 1; s_idx < max_search_neighbors_; s_idx++)
        {
          int s_row = row + static_cast<int> (std::floor (f_dy*static_cast<float> (s_idx)));
          int s_col = col + static_cast<int> (std::floor (f_dx*static_cast<float> (s_idx)));

          if (s_row < 0 || s_row >= int(input_->height) || s_col < 0 || s_col >= int(input_->width))
            break;

          if (pcl_isfinite (input_->points[s_row*int(input_->width)+s_col].z))
          {
            corr_depth = fabs (input_->points[s_row*int(input_->width)+s_col].z);
            break;
          }
        }

        if (!pcl_isnan (corr_depth))
        {
          // Found a corresponding point
          float dist = curr_depth - corr_depth;
          if (fabs (dist) > th_depth_discon_*fabs (curr_depth))
          {
            // Found a depth discontinuity
            if (dist > 0.f)
              labels[curr_idx].label = EDGELABEL_OCCLUDED;
            else
              labels[curr_idx].label = EDGELABEL_OCCLUDING;
          }
        } 
        else
        {
          // Not found a corresponding point, just nan boundary edge
          labels[curr_idx].label = EDGELABEL_NAN_BOUNDARY;
        }
      }
    }
  }

  // Assign label indices
  label_indices.resize (3);
  for (unsigned idx = 0; idx < input_->points.size (); idx++)
  {
    if (labels[idx].label != invalid_label)
    {
      label_indices[labels[idx].label].indices.push_back (idx);
    }
  }
}

#define PCL_INSTANTIATE_OrganizedEdgeDetection(T,LT) template class PCL_EXPORTS pcl::OrganizedEdgeDetection<T,LT>;

#endif //#ifndef PCL_FEATURES_IMPL_ORGANIZED_EDGE_DETECTION_H_
