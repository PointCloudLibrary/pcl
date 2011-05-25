/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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
 * $Id$
 *
 */

#ifndef PCL_CUDA_NORMAL_3D_H_
#define PCL_CUDA_NORMAL_3D_H_

#include "pcl/cuda/common/eigen.h"

namespace pcl
{
  namespace cuda
  {
  
    template <template <typename> class Storage>
    struct NormalEstimationKernel
    {
      typedef boost::shared_ptr <const PointCloudAOS <Storage> > CloudConstPtr;
      NormalEstimationKernel (const boost::shared_ptr <const PointCloudAOS <Storage> > &input, float focallength, float sqr_radius, float sqrt_desired_nr_neighbors)
        : points_ (thrust::raw_pointer_cast<const PointXYZRGB> (&input->points[0]))
        , focallength_ (focallength)
        , search_ (input, focallength, sqr_radius)
        , sqr_radius_(sqr_radius)
        , sqrt_desired_nr_neighbors_ (sqrt_desired_nr_neighbors)
     {}
  
      inline __host__ __device__
      float4 operator () (float3 query_pt)
      {
        CovarianceMatrix cov;
        int nnn = 0;
        if (!isnan (query_pt.x))
          nnn = 
          search_.computeCovarianceOnline (query_pt, cov, sqrt_desired_nr_neighbors_);
        else
          return make_float4(query_pt.x);
  
        CovarianceMatrix evecs;
        float3 evals;
        // compute eigenvalues and -vectors
        if (nnn <= 1)
          return make_float4(0);
  
        eigen33 (cov, evecs, evals);
        //float curvature = evals.x / (evals.x + evals.y + evals.z);
        float curvature = evals.x / (query_pt.z * (0.2f / 4.0f) * query_pt.z * (0.2f / 4.0f));
  
        float3 mc = normalize (evecs.data[0]);
        // TODO: this should be an optional step, as it slows down eveything
        // btw, this flips the normals to face the origin (assumed to be the view point)
        if ( dot (query_pt, mc) > 0 )
          mc = -mc;
        return make_float4 (mc.x, mc.y, mc.z, curvature);
      }
  
      const PointXYZRGB *points_;
      float focallength_;
      OrganizedRadiusSearch<CloudConstPtr> search_;
      float sqr_radius_;
      float sqrt_desired_nr_neighbors_;
    };
  
    template <typename InputIteratorT, typename OutputIteratorT, template <typename> class Storage>
      void computePointNormals (InputIteratorT begin, InputIteratorT end, OutputIteratorT output, float focallength, const boost::shared_ptr <const PointCloudAOS <Storage> > &input, float radius, int desired_number_neighbors);
  
    template <template <typename> class Storage, typename InputIteratorT>
      typename Storage<float4>::type computePointNormals (InputIteratorT begin, InputIteratorT end, float focallength, const boost::shared_ptr <const PointCloudAOS <Storage> > &input, float radius, int desired_number_neighbors);

  } // namespace
} // namespace

#endif  


