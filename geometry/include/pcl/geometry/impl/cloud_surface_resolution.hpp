/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
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
 * $Id$
 *
 */

#ifndef PCL_GEOMETRY_CLOUD_SURFACE_RESOLUTION_IMPL_HPP
#define PCL_GEOMETRY_CLOUD_SURFACE_RESOLUTION_IMPL_HPP

#include <limits>
#include <pcl/common/common.h>

namespace pcl
{
  namespace geometry
  { 
    template <typename PointT>
    float
    computeCloudSurfaceResolution (typename pcl::PointCloud<PointT>::Ptr & input)
    {
      typedef typename pcl::PointCloud<PointT>::ConstPtr PointCloudInTPtr;
      typedef typename pcl::KdTree<PointT>::Ptr KdTreeInPtr;
    
      if (input->points.size () == 0)
      { 
        PCL_ERROR ("input cloud is empty\n");
        return std::numeric_limits<float>::infinity;
      }
    
      KdTreeInPtr tree = boost::make_shared<pcl::KdTreeFLANN<PointT> > (false);
      tree->setInputCloud (input);
    
      std::vector<int> nn_indices (9);
      std::vector<float> nn_distances (9);
    
      float sum_distances = 0.0;
      std::vector<float> avg_distances (input->points.size ());
    
      // Iterate through the source data set
      for (size_t i = 0; i < input->points.size (); ++i)
      {
        tree->nearestKSearch (input->points[i], 9, nn_indices, nn_distances);
      
        float avg_dist_neighbours = 0.0;

        for (size_t j = 1; j < nn_indices.size (); j++)
          avg_dist_neighbours += sqrtf (nn_distances[j]);
      
        avg_dist_neighbours /= static_cast<float> (nn_indices.size ());
      
        avg_distances[i] = avg_dist_neighbours;
      
        sum_distances += avg_dist_neighbours;
      }
    
      std::sort (avg_distances.begin (), avg_distances.end ());

      return avg_distances[static_cast<int> (avg_distances.size ()) / 2 + 1];
    }

    template <typename PointT>
    float
    computeCloudSurfaceResolution (typename const pcl::PointCloud<PointT>::ConstPtr & input, const boost::shared_ptr<std::vector<int> > & indices)
    {
      typedef typename pcl::PointCloud<PointT>::ConstPtr PointCloudInTPtr;
      typedef typename pcl::KdTree<PointT>::Ptr KdTreeInPtr;
    
      if (input->points.size() == 0)
      {
        PCL_ERROR ("input cloud is empty\n");
        return std::numeric_limits<float>::infinity;
      }
      else if (indices->size() == 0)
      {
        PCL_ERROR ("input indices is empty\n");
        return std::numeric_limits<float>::infinity;
      }

      KdTreeInPtr tree = boost::make_shared<pcl::KdTreeFLANN<PointT> > (false);
      tree->setInputCloud (input, indices);
    
      std::vector<int> nn_indices (9);
      std::vector<float> nn_distances (9);
    
      float sum_distances = 0.0;
      std::vector<float> avg_distances (indices->size ());
        
      // Iterate through the source data set
      size_t i = 0;
      for (std::vector<int>::const_iterator it = indices->begin(); it != indices->end(); ++it, ++i)
      {
        tree->nearestKSearch (input->points[*it], 9, nn_indices, nn_distances);
      
        float avg_dist_neighbours = 0.0;
      
        for (size_t j = 1; j < nn_indices.size (); j++)
          avg_dist_neighbours += sqrtf (nn_distances[j]);
      
        avg_dist_neighbours /= static_cast<float> (nn_indices.size ());
      
        avg_distances[i] = avg_dist_neighbours;
            
        sum_distances += avg_dist_neighbours;
      }
    
      std::sort (avg_distances.begin (), avg_distances.end ());
      return avg_distances[static_cast<int> (avg_distances.size ()) / 2 + 1];
    }
  }
}

#endif
