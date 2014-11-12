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


#ifndef PCL_TRACKING_APPROX_NEAREST_PAIR_POINT_CLOUD_COHERENCE_H_
#define PCL_TRACKING_APPROX_NEAREST_PAIR_POINT_CLOUD_COHERENCE_H_

#include <pcl/search/search.h>
#include <pcl/search/octree.h>
#include <pcl/tracking/nearest_pair_point_cloud_coherence.h>
namespace pcl
{
  namespace tracking
  {
    /** \brief @b ApproxNearestPairPointCloudCoherence computes coherence between two pointclouds using the
         approximate nearest point pairs.
      * \author Ryohei Ueda
      * \ingroup tracking
      */
    template <typename PointInT>
    class ApproxNearestPairPointCloudCoherence: public NearestPairPointCloudCoherence<PointInT>
    {
    public:
      typedef typename NearestPairPointCloudCoherence<PointInT>::PointCoherencePtr PointCoherencePtr;
      typedef typename NearestPairPointCloudCoherence<PointInT>::PointCloudInConstPtr PointCloudInConstPtr;
      //using NearestPairPointCloudCoherence<PointInT>::search_;
      using NearestPairPointCloudCoherence<PointInT>::maximum_distance_;
      using NearestPairPointCloudCoherence<PointInT>::target_input_;
      using NearestPairPointCloudCoherence<PointInT>::point_coherences_;
      using NearestPairPointCloudCoherence<PointInT>::coherence_name_;
      using NearestPairPointCloudCoherence<PointInT>::new_target_;
      using NearestPairPointCloudCoherence<PointInT>::getClassName;
      
      /** \brief empty constructor */
      ApproxNearestPairPointCloudCoherence () : 
        NearestPairPointCloudCoherence<PointInT> (), search_ ()
      {
        coherence_name_ = "ApproxNearestPairPointCloudCoherence";
      }
      
    protected:
      /** \brief This method should get called before starting the actual computation. */
      virtual bool initCompute ();
      
      /** \brief compute the nearest pairs and compute coherence using point_coherences_ */
      virtual void
      computeCoherence (const PointCloudInConstPtr &cloud, const IndicesConstPtr &indices, float &w_j);

      typename boost::shared_ptr<pcl::search::Octree<PointInT> > search_;
    };
  }
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/tracking/impl/approx_nearest_pair_point_cloud_coherence.hpp>
#endif

#endif

