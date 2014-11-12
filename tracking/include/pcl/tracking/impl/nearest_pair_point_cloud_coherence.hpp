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


#ifndef PCL_TRACKING_IMPL_NEAREST_PAIR_POINT_CLOUD_COHERENCE_H_
#define PCL_TRACKING_IMPL_NEAREST_PAIR_POINT_CLOUD_COHERENCE_H_

#include <pcl/search/kdtree.h>
#include <pcl/search/organized.h>
#include <pcl/tracking/nearest_pair_point_cloud_coherence.h>

namespace pcl
{
  namespace tracking
  {
    template <typename PointInT> void 
    NearestPairPointCloudCoherence<PointInT>::computeCoherence (
        const PointCloudInConstPtr &cloud, const IndicesConstPtr &, float &w)
    {
      double val = 0.0;
      //for (size_t i = 0; i < indices->size (); i++)
      for (size_t i = 0; i < cloud->points.size (); i++)
      {
        PointInT input_point = cloud->points[i];
        std::vector<int> k_indices(1);
        std::vector<float> k_distances(1);
        search_->nearestKSearch (input_point, 1, k_indices, k_distances);
        int k_index = k_indices[0];
        float k_distance = k_distances[0];
        if (k_distance < maximum_distance_ * maximum_distance_)
        {
          // nearest_targets.push_back (k_index);
          // nearest_inputs.push_back (i);
          PointInT target_point = target_input_->points[k_index];
          double coherence_val = 1.0;
          for (size_t i = 0; i < point_coherences_.size (); i++)
          {
            PointCoherencePtr coherence = point_coherences_[i];  
            double w = coherence->compute (input_point, target_point);
            coherence_val *= w;
          }
          val += coherence_val;
        }
      }
      w = - static_cast<float> (val);
    }
    
    template <typename PointInT> bool
    NearestPairPointCloudCoherence<PointInT>::initCompute ()
    {
      if (!PointCloudCoherence<PointInT>::initCompute ())
      {
        PCL_ERROR ("[pcl::%s::initCompute] PointCloudCoherence::Init failed.\n", getClassName ().c_str ());
        //deinitCompute ();
        return (false);
      }
      
      // initialize tree
      if (!search_)
        search_.reset (new pcl::search::KdTree<PointInT> (false));
      
      if (new_target_ && target_input_)
      {
        search_->setInputCloud (target_input_);
        new_target_ = false;
      }
      
      return true;
    }
  }
}

#define PCL_INSTANTIATE_NearestPairPointCloudCoherence(T) template class PCL_EXPORTS pcl::tracking::NearestPairPointCloudCoherence<T>;

#endif
