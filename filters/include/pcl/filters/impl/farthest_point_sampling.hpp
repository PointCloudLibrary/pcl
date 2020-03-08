/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
 *  Copyright (c) 2015, Google, Inc.
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

#ifndef PCL_FILTERS_FARTHEST_POINT_SAMPLING_IMPL_H_
#define PCL_FILTERS_FARTHEST_POINT_SAMPLING_IMPL_H_

#include <limits>
#include <algorithm>
#include <pcl/filters/farthest_point_sampling.h>
#include <pcl/common/geometry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


template<typename PointT> void
pcl::FarthestPointSampling<PointT>::applyFilter (PointCloud &output)
{
    int initial_size = input_->points.size();
    int sample_size = sample_;
    int max_index;
    std::vector<int> selected_indices;
    std::vector<float>distances_to_selected_points;
    
    //set random seed
    std::srand (seed_);

    
    //check if number of points in cloud is less than requested number of points
    if (initial_size <= sample_size)
    {
        PCL_THROW_EXCEPTION (BadArgumentException,
                           "Requested number of points is greater than point cloud size!");
        return;
    }

    //set initial distances as infinite
    for (int m = 0; m < initial_size; m++)
      distances_to_selected_points.push_back(std::numeric_limits<float>::infinity());
    

    for (int j = 0; j < sample_size; j++)
    {
      //pick the first point at random
      if (j ==0)
        max_index = rand() % (initial_size -1);
      else
      {
        //select farthest point based on previously calculated distances
        //since distance is set to -1 for all selected elements,previously selected 
        //elements are guaranteed to not be selected
        max_index = std::max_element(distances_to_selected_points.begin(), distances_to_selected_points.end()) - distances_to_selected_points.begin(); 
      }
      //set distance to -1 to ignore during max element search
      distances_to_selected_points[max_index] = -1.0;
      selected_indices.push_back(max_index);
      
      //recompute distances
      for (int i = 0; i < distances_to_selected_points.size(); i++)
      {
        if (distances_to_selected_points[i] == -1.0)
          continue;
        distances_to_selected_points[i] = std::min(distances_to_selected_points[i], geometry::distance(input_->points[i], input_->points[max_index]));
      }
    }
    copyPointCloud(*input_, selected_indices, output);

}

 #define PCL_INSTANTIATE_FarthestPointSampling(T) template class PCL_EXPORTS pcl::FarthestPointSampling<T>;

 #endif // PCL_FILTERS_FARTHEST_POINT_SAMPLING_IMPL_H_