/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2011, Alexandru-Eugen Ichim
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
 *  $Id$
 */

#pragma once

#include <pcl/features/feature.h>

namespace pcl
{
  template <typename PointInT, typename PointNT, typename PointOutT = pcl::PFHRGBSignature250>
  class PFHRGBEstimation : public FeatureFromNormals<PointInT, PointNT, PointOutT>
  {
    public:
      using Ptr = shared_ptr<PFHRGBEstimation<PointInT, PointNT, PointOutT> >;
      using ConstPtr = shared_ptr<const PFHRGBEstimation<PointInT, PointNT, PointOutT> >;
      using PCLBase<PointInT>::indices_;
      using Feature<PointInT, PointOutT>::feature_name_;
      using Feature<PointInT, PointOutT>::surface_;
      using Feature<PointInT, PointOutT>::k_;
      using Feature<PointInT, PointOutT>::search_parameter_;
      using FeatureFromNormals<PointInT, PointNT, PointOutT>::normals_;
      using PointCloudOut = typename Feature<PointInT, PointOutT>::PointCloudOut;


      PFHRGBEstimation ()
        : nr_subdiv_ (5), d_pi_ (1.0f / (2.0f * static_cast<float> (M_PI)))
      {
        feature_name_ = "PFHRGBEstimation";
      }

      bool
      computeRGBPairFeatures (const pcl::PointCloud<PointInT> &cloud, const pcl::PointCloud<PointNT> &normals,
                              int p_idx, int q_idx,
                              float &f1, float &f2, float &f3, float &f4, float &f5, float &f6, float &f7);

      void
      computePointPFHRGBSignature (const pcl::PointCloud<PointInT> &cloud, const pcl::PointCloud<PointNT> &normals,
                                   const pcl::Indices &indices, int nr_split, Eigen::VectorXf &pfhrgb_histogram);

    protected:
      void
      computeFeature (PointCloudOut &output) override;

    private:
      /** \brief The number of subdivisions for each angular feature interval. */
      int nr_subdiv_;

      /** \brief Placeholder for a point's PFHRGB signature. */
      Eigen::VectorXf pfhrgb_histogram_;

      /** \brief Placeholder for a PFHRGB 7-tuple. */
      Eigen::VectorXf pfhrgb_tuple_;

      /** \brief Placeholder for a histogram index. */
      int f_index_[7];

      /** \brief Float constant = 1.0 / (2.0 * M_PI) */
      float d_pi_;
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/features/impl/pfhrgb.hpp>
#endif
