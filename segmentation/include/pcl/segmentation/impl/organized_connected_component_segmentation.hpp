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
 *
 *
 */

#ifndef PCL_SEGMENTATION_IMPL_ORGANIZED_CONNECTED_COMPONENT_SEGMENTATION_H_
#define PCL_SEGMENTATION_IMPL_ORGANIZED_CONNECTED_COMPONENT_SEGMENTATION_H_

#include <pcl/segmentation/organized_connected_component_segmentation.h>

/**
 *  Directions: 1 2 3
 *              0 x 4
 *              7 6 5
 * e.g. direction y means we came from pixel with label y to the center pixel x
 */
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename PointLT> void
pcl::OrganizedConnectedComponentSegmentation<PointT, PointLT>::findLabeledRegionBoundary (int start_idx, PointCloudLPtr labels, pcl::PointIndices& boundary_indices)
{
  boundary_indices.indices.clear ();
  int curr_idx = start_idx;
  int curr_x   = start_idx % labels->width;
  int curr_y   = start_idx / labels->width;
  unsigned label = labels->points[start_idx].label;
  
  // fill lookup table for next points to visit
  Neighbor directions [8] = {Neighbor(-1,  0,                 -1),
                             Neighbor(-1, -1, -labels->width - 1), 
                             Neighbor( 0, -1, -labels->width    ),
                             Neighbor( 1, -1, -labels->width + 1),
                             Neighbor( 1,  0,                  1),
                             Neighbor( 1,  1,  labels->width + 1),
                             Neighbor( 0,  1,  labels->width    ),
                             Neighbor(-1,  1,  labels->width - 1)};
  
  // find one pixel with other label in the neighborhood -> assume thats the one we came from
  int direction = -1;
  int x;
  int y;
  int index;
  for (unsigned dIdx = 0; dIdx < 8; ++dIdx)
  {
    x = curr_x + directions [dIdx].d_x;
    y = curr_y + directions [dIdx].d_y;
    index = curr_idx + directions [dIdx].d_index;
    if (x >= 0 && x < int(labels->width) && y >= 0 && y < int(labels->height) && labels->points[index].label != label)
    {
      direction = dIdx;
      break;
    }
  }

  // no connection to outer regions => start_idx is not on the border
  if (direction == -1)
    return;
  
  boundary_indices.indices.push_back (start_idx);
  
  do {
    unsigned nIdx;
    for (unsigned dIdx = 1; dIdx <= 8; ++dIdx)
    {
      nIdx = (direction + dIdx) & 7;
      
      x = curr_x + directions [nIdx].d_x;
      y = curr_y + directions [nIdx].d_y;
      index = curr_idx + directions [nIdx].d_index;
      if (x >= 0 && y < int(labels->width) && y >= 0 && y < int(labels->height) && labels->points[index].label == label)
        break;
    }
    
    // update the direction
    direction = (nIdx + 4) & 7;
    curr_idx += directions [nIdx].d_index;
    curr_x   += directions [nIdx].d_x;
    curr_y   += directions [nIdx].d_y;
    boundary_indices.indices.push_back(curr_idx);
  } while ( curr_idx != start_idx);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename PointLT> void
pcl::OrganizedConnectedComponentSegmentation<PointT, PointLT>::segment (pcl::PointCloud<PointLT>& labels, std::vector<pcl::PointIndices>& label_indices) const
{
  std::vector<unsigned> run_ids;

  unsigned invalid_label = std::numeric_limits<unsigned>::max ();
  PointLT invalid_pt;
  invalid_pt.label = std::numeric_limits<unsigned>::max ();
  labels.points.resize (input_->points.size (), invalid_pt);
  labels.width = input_->width;
  labels.height = input_->height;
  unsigned int clust_id = 0;
  
  //First pixel
  if (pcl_isfinite (input_->points[0].x))
  {
    labels[0].label = clust_id++;
    run_ids.push_back (labels[0].label );
  }   

  // First row
  for (int colIdx = 1; colIdx < static_cast<int> (input_->width); ++colIdx)
  {
    if (!pcl_isfinite (input_->points[colIdx].x))
      continue;
    else if (compare_->compare (colIdx, colIdx - 1 ))
    {
      labels[colIdx].label = labels[colIdx - 1].label;
    }
    else
    {
      labels[colIdx].label = clust_id++;
      run_ids.push_back (labels[colIdx].label );
    }
  }
  
  // Everything else
  unsigned int current_row = input_->width;
  unsigned int previous_row = 0;
  for (size_t rowIdx = 1; rowIdx < input_->height; ++rowIdx, previous_row = current_row, current_row += input_->width)
  {
    // First pixel
    if (pcl_isfinite (input_->points[current_row].x))
    {
      if (compare_->compare (current_row, previous_row))
      {
        labels[current_row].label = labels[previous_row].label;
      }
      else
      {
        labels[current_row].label = clust_id++;
        run_ids.push_back (labels[current_row].label);
      }
    }
    
    // Rest of row
    for (int colIdx = 1; colIdx < static_cast<int> (input_->width); ++colIdx)
    {
      if (pcl_isfinite (input_->points[current_row + colIdx].x))
      {
        if (compare_->compare (current_row + colIdx, current_row + colIdx - 1))
        {
          labels[current_row + colIdx].label = labels[current_row + colIdx - 1].label;
        }
        if (compare_->compare (current_row + colIdx, previous_row + colIdx) )
        {
          if (labels[current_row + colIdx].label == invalid_label)
            labels[current_row + colIdx].label = labels[previous_row + colIdx].label;
          else if (labels[previous_row + colIdx].label != invalid_label)
          {
            unsigned root1 = findRoot (run_ids, labels[current_row + colIdx].label);
            unsigned root2 = findRoot (run_ids, labels[previous_row + colIdx].label);
            
            if (root1 < root2)
              run_ids[root2] = root1;
            else
              run_ids[root1] = root2;
          }
        }
        
        if (labels[current_row + colIdx].label == invalid_label)
        {
          labels[current_row + colIdx].label = clust_id++;
          run_ids.push_back (labels[current_row + colIdx].label);
        }
        
      }
    }
  }
  
  std::vector<unsigned> map (clust_id);
  unsigned max_id = 0;
  for (unsigned runIdx = 0; runIdx < run_ids.size (); ++runIdx)
  {
    // if it is its own root -> new region
    if (run_ids[runIdx] == runIdx)
      map[runIdx] = max_id++;
    else // assign this sub-segment to the region (root) it belongs
      map [runIdx] = map [findRoot (run_ids, runIdx)];
  }

  label_indices.resize (max_id + 1);
  for (unsigned idx = 0; idx < input_->points.size (); idx++)
  {
    if (labels[idx].label != invalid_label)
    {
      labels[idx].label = map[labels[idx].label];
      label_indices[labels[idx].label].indices.push_back (idx);
    }
  }
}

#define PCL_INSTANTIATE_OrganizedConnectedComponentSegmentation(T,LT) template class PCL_EXPORTS pcl::OrganizedConnectedComponentSegmentation<T,LT>;

#endif //#ifndef PCL_SEGMENTATION_IMPL_ORGANIZED_CONNECTED_COMPONENT_SEGMENTATION_H_
