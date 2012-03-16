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

#include <pcl/segmentation/organized_connected_component_segmentation.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename PointLT> void
pcl::OrganizedConnectedComponentSegmentation<PointT, PointLT>::getNextIdxs(int& p1, int& p2, int& p3, int curr_idx, int dir, int width){
  if(dir == 0){
    p2 = (curr_idx - width);
    p1 = p2 - 1;
    p3 = p2 + 1;
  } else if(dir == 1){
    p1 = (curr_idx - width) + 1;
    p2 = curr_idx + 1;
    p3 = (curr_idx + width) + 1;
  } else if(dir == 2){
    p2 = (curr_idx + width);
    p1 = p2 + 1;
    p3 = p2 - 1;
  } else if(dir == 3){
    p1 = (curr_idx + width) - 1;
    p2 = curr_idx - 1;
    p3 = (curr_idx - width) - 1;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename PointLT> bool
pcl::OrganizedConnectedComponentSegmentation<PointT, PointLT>::isIndexValid (int curr_idx, int test_idx, int width, int max)
{
  if (test_idx < 0)
    return (false);
  else if (test_idx >= max)
    return (false);
  // else if ((abs(curr_idx - test_idx) == 1) && (curr_idx / width) != (test_idx % width))
  //  return (false);
  
  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename PointLT> void
pcl::OrganizedConnectedComponentSegmentation<PointT, PointLT>::findLabeledRegionBoundary (int start_idx, PointCloudLPtr labels, pcl::PointIndices& boundary_indices)
{
  bool first_done = false;
  boundary_indices.indices.push_back (start_idx);
  int curr_idx = start_idx;
  int p1, p2, p3 = 0;
  int dir = 0;
  unsigned label = labels->points[start_idx].label;
  int max_idx = labels->points.size ();

  while (!first_done)
  {
    getNextIdxs(p1,p2,p3,curr_idx,dir,labels->width);
    if( isIndexValid (curr_idx, p1, labels->width, max_idx) && labels->points[p1].label == label){
      boundary_indices.indices.push_back(p1);
      curr_idx = p1;
      first_done = true;
      dir = (dir-1);
      if(dir == -1)
        dir = 3;
    } else if( isIndexValid (curr_idx, p2, labels->width, max_idx) && labels->points[p2].label == label){
      boundary_indices.indices.push_back(p2);
      curr_idx = p2;
      first_done = true;
    } else if( isIndexValid (curr_idx, p3, labels->width, max_idx) && labels->points[p3].label == label){
      boundary_indices.indices.push_back(p3);
      curr_idx = p3;
      first_done = true;
    } else {
      dir = (dir+1) % 4;
    } 
  }

  while ((curr_idx != start_idx)){
    getNextIdxs(p1, p2, p3, curr_idx, dir, labels->width);   
    if (isIndexValid (curr_idx, p1, labels->width, max_idx) && labels->points[p1].label == label){
      boundary_indices.indices.push_back(p1);
      curr_idx = p1;
      dir = (dir-1);// % 4;
      if(dir == -1)
        dir=3;
    } else if (isIndexValid (curr_idx, p2, labels->width, max_idx) && labels->points[p2].label == label){
      boundary_indices.indices.push_back(p2);
      curr_idx = p2;
    } else if (isIndexValid (curr_idx, p3, labels->width, max_idx) && labels->points[p3].label == label){
      boundary_indices.indices.push_back(p3);
      curr_idx = p3;
    } else {
      dir = (dir+1) % 4;
    } 
  }
}

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

  unsigned invalid_label = std::numeric_limits<unsigned>::max ();
  pcl::Label invalid_pt;
  invalid_pt.label = std::numeric_limits<unsigned>::max ();
  labels.points.resize (input_->points.size (), invalid_pt);
  labels.width = input_->width;
  labels.height = input_->height;
  unsigned int clust_id = 0;
  
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
          else
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
    if (run_ids[runIdx] == runIdx)
      map[runIdx] = max_id++;
  }
  label_indices.resize (max_id + 1);
  for (unsigned idx = 0; idx < input_->points.size (); idx++)
  {
    if (labels[idx].label != invalid_label)
    {
      labels[idx].label = map[findRoot (run_ids, labels[idx].label)];
      label_indices[labels[idx].label].indices.push_back (idx);
    }
  }
}

#define PCL_INSTANTIATE_OrganizedConnectedComponentSegmentation(T,LT) template class PCL_EXPORTS pcl::OrganizedConnectedComponentSegmentation<T,LT>;

#endif //#ifndef PCL_SEGMENTATION_IMPL_ORGANIZED_CONNECTED_COMPONENT_SEGMENTATION_H_
