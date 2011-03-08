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

#ifndef PCL_INTEGRAL_IMAGE_2D_H_
#define PCL_INTEGRAL_IMAGE_2D_H_

#include <vector>

namespace pcl
{

  /**
   * \brief Generic implementation for creating 2D integral images (including second order integral images).
   */
  template <class DataType, class IIDataType>
  class IntegralImage2D
  {
    public: // functions
    
      /**
       * Constructor. Internally creates the integral images.
       *
       * \param data the dense 2d input data array.
       * \param width the width of the 2d input data array.
       * \param height the height of the 2d input data array. 
       * \param dimensions number of dimensions of each element.
       * \param compute_second_order_integral_images whether to compute second order integral images.
       * \param element_stride number of DataType entries per element (equal or bigger than dimensions).
       * \param row_stride number of DataType entries per row (equal or bigger than element_stride * number of 
       *          elements per row).
       */
      IntegralImage2D (DataType * data,
                       const int width,
                       const int height,
                       const int dimensions,
                       const bool compute_second_order_integral_images, 
                       const int element_stride, const int row_stride);
      //! Destructor
      virtual ~IntegralImage2D ();

      /**
       * Computes the sum of the "dimension_index"-th dimension of the elements within the specified area.
       */
      IIDataType getSum (const int start_x, const int start_y, 
                         const int width,   const int height, 
                         const int dimension_index);

      /**
       * Computes the sum of the "dimension_index1"-th dimension of the elements times the 
       * "dimension_index2"-th dimension of the elements within the specified area.
       */
      IIDataType getSum (const int start_x, const int start_y,
                         const int width,   const int height, 
                         const int dimension_index_1, const int dimension_index_2);

    protected: // functions
    
      /** Computes integral images for multiple dimensions. */
      void computeIntegralImages (DataType * data);
      /** Computes integral image for one dimension. */
      void computeIntegralImagesOneDimensional (DataType * data);

    protected: // data
    
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
      /** indicates whether second order integral images are available **/
      bool are_second_order_ii_available_;

      /** first order integral images */
      std::vector <IIDataType *> first_order_integral_images_;
      /** second order integral images */
      std::vector <std::vector <IIDataType*> > second_order_integral_images_;
  };

}

#include <pcl/features/impl/integral_image_2d.hpp>

#endif


