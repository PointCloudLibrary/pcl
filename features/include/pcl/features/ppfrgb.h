/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012-, Open Perception, Inc
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

#pragma once

#include <pcl/features/feature.h>
#include <pcl/features/boost.h>

namespace pcl
{
  template <typename PointInT, typename PointNT, typename PointOutT>
  class PPFRGBEstimation : public FeatureFromNormals<PointInT, PointNT, PointOutT>
  {
    public:
      using PCLBase<PointInT>::indices_;
      using Feature<PointInT, PointOutT>::input_;
      using Feature<PointInT, PointOutT>::feature_name_;
      using Feature<PointInT, PointOutT>::getClassName;
      using FeatureFromNormals<PointInT, PointNT, PointOutT>::normals_;

      using PointCloudOut = pcl::PointCloud<PointOutT>;

      /**
        * \brief Empty Constructor
        */
      PPFRGBEstimation ();


    private:
      /** \brief The method called for actually doing the computations
        * \param output the resulting point cloud (which should be of type pcl::PPFRGBSignature);
        */
      void
      computeFeature (PointCloudOut &output);
  };

  template <typename PointInT, typename PointNT, typename PointOutT>
  class PPFRGBRegionEstimation : public FeatureFromNormals<PointInT, PointNT, PointOutT>
  {
    public:
      using Ptr = shared_ptr<PPFRGBRegionEstimation<PointInT, PointNT, PointOutT> >;
      using ConstPtr = shared_ptr<const PPFRGBRegionEstimation<PointInT, PointNT, PointOutT> >;
      using PCLBase<PointInT>::indices_;
      using Feature<PointInT, PointOutT>::input_;
      using Feature<PointInT, PointOutT>::feature_name_;
      using Feature<PointInT, PointOutT>::search_radius_;
      using Feature<PointInT, PointOutT>::tree_;
      using Feature<PointInT, PointOutT>::getClassName;
      using FeatureFromNormals<PointInT, PointNT, PointOutT>::normals_;

      using PointCloudOut = pcl::PointCloud<PointOutT>;

      PPFRGBRegionEstimation ();

    private:
      void
      computeFeature (PointCloudOut &output) override;
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/features/impl/ppfrgb.hpp>
#endif
