/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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
 */

/* \author Stefan Holzer */

#ifndef PCL_INTEGRALIMAGE_BASED_NORMAL_ESTIMATOR_H_
#define PCL_INTEGRALIMAGE_BASED_NORMAL_ESTIMATOR_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "pcl/features/feature.h"
#include "pcl/features/integral_image_2d.h"

namespace pcl
{
  /**
    * \brief Surface normal estimation on dense data using integral images.
    * \author Stefan Holzer
    */
  template <typename PointInT, typename PointOutT>
  class IntegralImageNormalEstimation: public Feature<PointInT, PointOutT>
  {
    using Feature<PointInT, PointOutT>::input_;
    using Feature<PointInT, PointOutT>::feature_name_;

    public:

      enum NormalEstimationMethod
      {
        COVARIANCE_MATRIX,
        AVERAGE_3D_GRADIENT,
        AVERAGE_DEPTH_CHANGE
      };

      typedef typename Feature<PointInT, PointOutT>::PointCloudIn  PointCloudIn;
      typedef typename Feature<PointInT, PointOutT>::PointCloudOut PointCloudOut;

      //! Constructor
      IntegralImageNormalEstimation () : integral_image_x_(NULL), integral_image_y_(NULL), 
                                         integral_image_xyz_(NULL), integral_image_(NULL),
                                         diff_x_(NULL), diff_y_(NULL), depth_data_(NULL)
      {
        feature_name_ = "IntegralImagesNormalEstimation";
      }

      //! Destructor
      virtual ~IntegralImageNormalEstimation ();

      /** \brief Set the regions size which is considered for normal estimation.
        * \param width the width of the search rectangle
        * \param height the height of the search rectangle
        */
      void 
      setRectSize (const int width, const int height);

      /** \brief Computes the normal at the specified position. 
        * \param pos_x x position (pixel)
        * \param pos_y y position (pixel)
        * \param normal the output estimated normal 
        */
      void
      computePointNormal (const int pos_x, const int pos_y, PointOutT &normal);

      /** \brief Estimate normals for all points given in <setInputCloud (), setIndices ()> using the surface in
        * setSearchSurface () and the spatial locator in setSearchMethod ()
        * \param output the resultant point cloud model dataset that contains surface normals and curvatures
        */
      void 
      compute (PointCloudOut &output);

    protected:

      void 
      computeFeature (PointCloudOut &output) {}

      /** \brief Computes the normal for the complete cloud. 
        * \param cloud the input point cloud
        */
      void
      computeFeature (PointCloudIn &cloud,
                      PointCloudOut &normals,
                      const float maxDepthChangeFactor = 20.0f*0.001f,
                      const float normalSmoothingSize = 10.0f,
                      const NormalEstimationMethod normal_estimation_method = AVERAGE_3D_GRADIENT);

      /** \brief Computes the normal for the complete cloud. 
        * \param cloud the input point cloud
        */
      void 
      computeFeature (PointCloudIn &cloud,
                      PointCloudOut &normals,
                      const bool useDepthDependentSmoothing,
                      const float maxDepthChangeFactor = 20.0f*0.001f,
                      const float normalSmoothingSize = 10.0f,
                      const NormalEstimationMethod normal_estimation_method = AVERAGE_3D_GRADIENT);

      /**
        * Sets the input data.
        *
        * \param data the dense 2d input data array.
        * \param width the width of the 2d input data array.
        * \param height the height of the 2d input data array. 
        * \param dimensions number of dimensions of each element.
        * \param element_stride number of DataType entries per element (equal or bigger than dimensions).
        * \param row_stride number of DataType entries per row (equal or bigger than element_stride * number of 
        *          elements per row).
        * \param distance_threshold threshold for detecting depth discontinuities
        * \param normal_estimation_method the normal estimation method. Select between: COVARIANCE_MATRIX, AVERAGE_3D_GRADIENT, AVERAGE_DEPTH_CHANGE
        */
      void 
      setInputData (float *data, const int width, const int height, const int dimensions,
                    const int element_stride, const int row_stride, const float distance_threshold,
                    const NormalEstimationMethod normal_estimation_method = AVERAGE_3D_GRADIENT);

    private:
      /** \brief The normal estimation method to use. */
      NormalEstimationMethod normal_estimation_method_;
    
      /** The width of the neighborhood region used for computing the normal. */
      int rect_width_;
      /** The height of the neighborhood region used for computing the normal. */
      int rect_height_;

      /** The input data */
      float * data_;
      /** the width of the 2d input data array */
      int width_;
      /** the height of the 2d input data array */
      int height_;
      /** number of dimensions of each element */
      int dimensions_;
      /** number of DataType entries per element */
      int element_stride_;
      /** number of DataType entries per row */
      int row_stride_;

      /** the threshold used to detect depth discontinuities */
      float distance_threshold_;

      /** integral image in x-direction */
      IntegralImage2D<float, double> *integral_image_x_;
      /** integral image in y-direction */
      IntegralImage2D<float, double> *integral_image_y_;
      /** integral image xyz */
      IntegralImage2D<float, double> *integral_image_xyz_;
      /** integral image */
      IntegralImage2D<float, double> *integral_image_;

      /** derivatives in x-direction */
      float *diff_x_;
      /** derivatives in y-direction */
      float *diff_y_;

      /** depth data */
      float *depth_data_;
  };

}

#include "pcl/features/impl/integral_image_normal.hpp"

#endif 

