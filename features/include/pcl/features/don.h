/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012, Yani Ioannou <yani.ioannou@gmail.com>
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
 */

#pragma once

#include <pcl/features/feature.h>

namespace pcl
{
  /** \brief A Difference of Normals (DoN) scale filter implementation for point cloud data.
   *
   * For each point in the point cloud two normals estimated with a differing search radius (sigma_s, sigma_l)
   * are subtracted, the difference of these normals provides a scale-based feature which
   * can be further used to filter the point cloud, somewhat like the Difference of Guassians
   * in image processing, but instead on surfaces. Best results are had when the two search
   * radii are related as sigma_l=10*sigma_s, the octaves between the two search radii
   * can be though of as a filter bandwidth. For appropriate values and thresholds it
   * can be used for surface edge extraction.
   *
   * \attention The input normals given by setInputNormalsSmall and setInputNormalsLarge have
   * to match the input point cloud given by setInputCloud. This behavior is different than
   * feature estimation methods that extend FeatureFromNormals, which match the normals
   * with the search surface.
   *
   * \note For more information please see
   *    <b>Yani Ioannou. Automatic Urban Modelling using Mobile Urban LIDAR Data.
   *    Thesis (Master, Computing), Queen's University, March, 2010.</b>
   *
   * \author Yani Ioannou.
   * \ingroup features
   */
  template <typename PointInT, typename PointNT, typename PointOutT>
  class DifferenceOfNormalsEstimation : public Feature<PointInT, PointOutT>
  {
      using Feature<PointInT, PointOutT>::getClassName;
      using Feature<PointInT, PointOutT>::feature_name_;
      using PCLBase<PointInT>::input_;
      using PointCloudN = pcl::PointCloud<PointNT>;
      using PointCloudNPtr = typename PointCloudN::Ptr;
      using PointCloudNConstPtr = typename PointCloudN::ConstPtr;
      using PointCloudOut = typename Feature<PointInT, PointOutT>::PointCloudOut;
    public:
      using Ptr = shared_ptr<DifferenceOfNormalsEstimation<PointInT, PointNT, PointOutT> >;
      using ConstPtr = shared_ptr<const DifferenceOfNormalsEstimation<PointInT, PointNT, PointOutT> >;

      /**
        * Creates a new Difference of Normals filter.
        */
      DifferenceOfNormalsEstimation ()
      {
        feature_name_ = "DifferenceOfNormalsEstimation";
      }

      ~DifferenceOfNormalsEstimation ()
      {
        //
      }

      /**
       * Set the normals calculated using a smaller search radius (scale) for the DoN operator.
       * @param normals the smaller radius (scale) of the DoN filter.
       */
      inline void
      setNormalScaleSmall (const PointCloudNConstPtr &normals)
      {
        input_normals_small_ = normals;
      }

      /**
       * Set the normals calculated using a larger search radius (scale) for the DoN operator.
       * @param normals the larger radius (scale) of the DoN filter.
       */
      inline void
      setNormalScaleLarge (const PointCloudNConstPtr &normals)
      {
        input_normals_large_ = normals;
      }

      /**
       * Computes the DoN vector for each point in the input point cloud and outputs the vector cloud to the given output.
       * @param output the cloud to output the DoN vector cloud to.
       */
      void
      computeFeature (PointCloudOut &output) override;

      /**
       * Initialize for computation of features.
       * @return true if parameters (input normals, input) are sufficient to perform computation.
       */
      bool
      initCompute () override;
    private:
      /** \brief Make the compute (&PointCloudOut); inaccessible from outside the class
        * \param[out] output the output point cloud
        */
      void
      compute (PointCloudOut &) {}

      ///The smallest radius (scale) used in the DoN filter.
      PointCloudNConstPtr input_normals_small_;
      ///The largest radius (scale) used in the DoN filter.
      PointCloudNConstPtr input_normals_large_;
    };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/features/impl/don.hpp>
#endif
