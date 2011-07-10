/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Alexandru-Eugen Ichim
 *                      Willow Garage, Inc
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
 *  $Id$
 */

#ifndef PCL_NORMAL_BASED_SIGNATURE_H_
#define PCL_NORMAL_BASED_SIGNATURE_H_

#include "pcl/features/feature.h"

namespace pcl
{
  template <typename PointT, typename PointNT, typename PointFeature>
  class NormalBasedSignatureEstimation : public FeatureFromNormals<PointT, PointNT, PointFeature>
  {
    public:
      using Feature<PointT, PointFeature>::input_;
      using Feature<PointT, PointFeature>::tree_;
      using Feature<PointT, PointFeature>::search_radius_;
      using PCLBase<PointT>::indices_;
      using FeatureFromNormals<PointT, PointNT, PointFeature>::normals_;

      typedef pcl::PointCloud<PointFeature> FeatureCloud;


      NormalBasedSignatureEstimation (float a_scale_h,
                            float a_normal_search_radius,
                            size_t a_N = 32,
                            size_t a_M = 8,
                            size_t a_N_prime = 4,
                            size_t a_M_prime = 3)
        : FeatureFromNormals<PointT, PointNT, PointFeature> (),
          scale_h (a_scale_h),
          normal_search_radius (a_normal_search_radius),
          N (a_N),
          M (a_M),
          N_prime (a_N_prime),
          M_prime (a_M_prime)
      {
        search_radius_ = a_normal_search_radius;
      }

    protected:
      void
      computeFeature (FeatureCloud &output);

    private:
      float scale_h, normal_search_radius;
      size_t N, M, N_prime, M_prime;
  };
}

#endif /* PCL_NORMAL_BASED_SIGNATURE_H_ */
