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

#ifndef PCL_SEGMENTATION_IMPL_ORGANIZED_CONNECTED_COMPONENT_SEGMENTATION_H_
#define PCL_SEGMENTATION_IMPL_ORGANIZED_CONNECTED_COMPONENT_SEGMENTATION_H_

#include "pcl/segmentation/organized_connected_component_segmentation.h"

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename PointLT> unsigned
pcl::OrganizedConnectedComponentSegmentation<PointT, PointLT>::findRoot (const std::vector<unsigned>& runs, unsigned index)
{
  unsigned idx = index;
  while (runs[idx] != idx)
    idx = runs[idx];

  return (idx);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename PointLT> void
pcl::OrganizedConnectedComponentSegmentation<PointT, PointLT>::segment (pcl::PointCloud<PointLT>& labels, std::vector<pcl::PointIndices>& label_indices)
{
  std::vector<unsigned> run_ids;

  std::vector<int> clusts (input_->points.size (), -1);
  unsigned int clust_id = 0;
  
  // First row
  for (size_t colIdx = 1; colIdx < input_->width; ++colIdx)
  {
    if (!pcl_isfinite (input_->points[colIdx].x))
      continue;
    else if (compare_->compare (colIdx, colIdx - 1 ))
    {
      clusts[colIdx] = clusts[colIdx - 1];
    }
    else
    {
      clusts[colIdx] = clust_id++;
      run_ids.push_back (clusts[colIdx]);
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
        clusts[current_row] = clusts[previous_row];
      }
      else
      {
        clusts[current_row] = clust_id++;
        run_ids.push_back (clusts[current_row]);
      }
    }
    
    // Rest of row
    for (size_t colIdx = 1; colIdx < input_->width; ++colIdx)
    {
      if (pcl_isfinite (input_->points[current_row + colIdx].x))
      {
        if (compare_->compare (current_row + colIdx, current_row + colIdx - 1))
        {
          clusts[current_row + colIdx] = clusts[current_row + colIdx - 1];
        }
        
        if (compare_->compare (current_row + colIdx, previous_row + colIdx) )
        {
          if (clusts[current_row + colIdx] == -1)
            clusts[current_row + colIdx] = clusts[previous_row + colIdx];
          else
          {
            // find the root nodes of both sets
            unsigned root1 = findRoot (run_ids, clusts[current_row + colIdx]);
            unsigned root2 = findRoot (run_ids, clusts[previous_row + colIdx]);
            
            if (root1 < root2)
              run_ids[root2] = root1;
            else
              run_ids[root1] = root2;
          }
        }
        
        if (clusts[current_row + colIdx] == -1)
        {
          clusts[current_row + colIdx] = clust_id++;
          run_ids.push_back (clusts[current_row + colIdx]);
        }
        
      }
    }
  }
  
  std::vector<unsigned> map (clust_id);
  unsigned max_id = 0;
  for (unsigned runIdx = 0; runIdx < run_ids.size (); ++runIdx)
  {
    if (run_ids[runIdx] == runIdx)
      map[runIdx] = max_id++;
  }
  label_indices.resize (max_id + 1);
  for (unsigned idx = 0; idx < input_->points.size (); idx++)
  {
    if (clusts[idx] != -1)
    {
      label_indices[map[findRoot (run_ids, clusts[idx])]].indices.push_back (idx);
    }
  }
}

#define PCL_INSTANTIATE_OrganizedConnectedComponentSegmentation(T,LT) template class PCL_EXPORTS pcl::OrganizedConnectedComponentSegmentation<T,LT>;

#endif //#ifndef PCL_SEGMENTATION_IMPL_ORGANIZED_CONNECTED_COMPONENT_SEGMENTATION_H_
