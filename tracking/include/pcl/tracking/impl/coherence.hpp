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


#ifndef PCL_TRACKING_IMPL_COHERENCE_H_
#define PCL_TRACKING_IMPL_COHERENCE_H_

#include <pcl/console/print.h>
#include <pcl/tracking/coherence.h>

namespace pcl
{
  namespace tracking
  {
    
    template <typename PointInT> double
    PointCoherence<PointInT>::compute (PointInT &source, PointInT &target)
    {
      return computeCoherence (source, target);
    }

    template <typename PointInT> double
    PointCloudCoherence<PointInT>::calcPointCoherence (PointInT &source, PointInT &target)
    {
      double val = 0.0;
      for (size_t i = 0; i < point_coherences_.size (); i++)
      {
        PointCoherencePtr coherence = point_coherences_[i];
        double d = log(coherence->compute (source, target));
        //double d = coherence->compute (source, target);
        if (! pcl_isnan(d))
          val += d;
        else
          PCL_WARN ("nan!\n");
      }
      return val;
    }
    
    template <typename PointInT> bool
    PointCloudCoherence<PointInT>::initCompute ()
    {
      if (!target_input_ || target_input_->points.empty ())
      {
        PCL_ERROR ("[pcl::%s::compute] target_input_ is empty!\n", getClassName ().c_str ());
        return false;
      }

      return true;
      
    }
    
    template <typename PointInT> void
    PointCloudCoherence<PointInT>::compute (const PointCloudInConstPtr &cloud, const IndicesConstPtr &indices, float &w)
    {
      if (!initCompute ())
      {
        PCL_ERROR ("[pcl::%s::compute] Init failed.\n", getClassName ().c_str ());
        return;
      }
      computeCoherence (cloud, indices, w);
    }
  }
}

#endif
