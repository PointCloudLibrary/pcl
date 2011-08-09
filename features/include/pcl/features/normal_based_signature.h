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
  /** \brief Normal-based feature signature estimation class [more info here ...]
   * Please consult the following publication for more details:
   *    Xinju Li and Igor Guskov
   *    Multi-scale features for approximate alignment of point-based surfaces
   *    Proceedings of the third Eurographics symposium on Geometry processing
   *    July 2005, Vienna, Austria
   *
   * \Note These features were meant to be used at keypoints detected by a detector using different smoothing radii
   * (e.g., SmoothedSurfacesKeypoint)
   * \author Alexandru-Eugen Ichim
   */
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


      /** \brief Empty constructor, initializes the internal parameters to the default values
        */
      NormalBasedSignatureEstimation ()
        : FeatureFromNormals<PointT, PointNT, PointFeature> (),
          N_ (32),
          M_ (8),
          N_prime_ (4),
          M_prime_ (3)
      {
      }

      inline void
      setN (size_t N) { N_ = N; }

      inline size_t
      getN () { return N_; }

      inline void
      setM (size_t M) { M_ = M; }

      inline size_t
      getM () { return M_; }

      inline void
      setNPrime (size_t N_prime) { N_prime_ = N_prime; }

      inline size_t
      getNPrime () { return N_prime_; }

      inline void
      setMPrime (size_t M_prime) { M_prime_ = M_prime; }

      inline size_t
      getMPrime () { return M_prime_; }

      inline void
      setScale (float scale) { scale_h_ = scale; }

      inline float
      getScale () { return scale_h_; }


    protected:
      void
      computeFeature (FeatureCloud &output);

    private:
      float scale_h_;
      size_t N_, M_, N_prime_, M_prime_;
  };
}

#endif /* PCL_NORMAL_BASED_SIGNATURE_H_ */
