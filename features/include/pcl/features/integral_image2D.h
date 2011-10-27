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

/* \author Suat Gedikli */

#ifndef PCL_INTEGRAL_IMAGE2D_H_
#define PCL_INTEGRAL_IMAGE2D_H_

#include <vector>

namespace pcl
{
  template <typename DataType>
  struct IntegralImageTypeTraits
  {
    typedef DataType Type;
    typedef DataType IntegralType;
  };
  
  template <>
  struct IntegralImageTypeTraits<float>
  {
    typedef float Type;
    typedef double IntegralType;
  };
  
  template <>
  struct IntegralImageTypeTraits<char>
  {
    typedef char Type;
    typedef int IntegralType;
  };

  template <>
  struct IntegralImageTypeTraits<short>
  {
    typedef short Type;
    typedef long IntegralType;
  };

  template <>
  struct IntegralImageTypeTraits<unsigned short>
  {
    typedef unsigned short Type;
    typedef unsigned long IntegralType;
  };
  
  template <>
  struct IntegralImageTypeTraits<unsigned char>
  {
    typedef unsigned char Type;
    typedef unsigned int IntegralType;
  };
  
  template <>
  struct IntegralImageTypeTraits<int>
  {
    typedef int Type;
    typedef long IntegralType;
  };
  
  template <>
  struct IntegralImageTypeTraits<unsigned int>
  {
    typedef unsigned int Type;
    typedef unsigned long IntegralType;
  };
  
  template <class DataType, unsigned Dimensions>
  class IntegralImage2Dim
  {
    public:
      static const unsigned second_order_size = (Dimensions * (Dimensions + 1)) >> 1;
      typedef Eigen::Matrix<typename IntegralImageTypeTraits<DataType>::IntegralType, Dimensions, 1> ElementType;
      typedef Eigen::Matrix<typename IntegralImageTypeTraits<DataType>::IntegralType, second_order_size, 1> SecondOrderType;
      
      IntegralImage2Dim (unsigned width, unsigned height, bool compute_second_order_integral_images);
      
      virtual ~IntegralImage2Dim ();
      
      void setInput (const DataType * data, unsigned width, unsigned height, unsigned element_stride, unsigned row_stride);
      
      ElementType getFirstOrderSum (unsigned start_x, unsigned start_y, unsigned width, unsigned height) const;
      SecondOrderType getSecondOrderSum (unsigned start_x, unsigned start_y, unsigned width, unsigned height) const;
    
    private:
      typedef Eigen::Matrix<typename IntegralImageTypeTraits<DataType>::Type, Dimensions, 1> InputType;

      void computeIntegralImages (const DataType * data, unsigned row_stride, unsigned element_stride);
      
      std::vector<ElementType, Eigen::aligned_allocator<ElementType> > first_order_integral_image_;
      std::vector<SecondOrderType, Eigen::aligned_allocator<SecondOrderType> > second_order_integral_image_;
      
      /** the width of the 2d input data array */
      unsigned width_;
      /** the height of the 2d input data array */
      unsigned height_;
      
      /** indicates whether second order integral images are available **/
      bool compute_second_order_integral_images_;
  };
  
  template <typename DataType>
  class IntegralImage2Dim<DataType, 1>
  {
    public:
      typedef Eigen::Matrix<typename IntegralImageTypeTraits<DataType>::IntegralType, 2, 1> VectorType;
      
      IntegralImage2Dim (unsigned width, unsigned height, bool compute_second_order_integral_images);
      
      virtual ~IntegralImage2Dim ();
      
      void setInput (const DataType * data, unsigned width, unsigned height, unsigned element_stride, unsigned row_stride);
      
      VectorType getSum (unsigned start_x, unsigned start_y, unsigned width, unsigned height) const;
      
    private:
      void computeIntegralImages (const DataType * data, unsigned row_stride, unsigned element_stride);
      
      std::vector<VectorType, Eigen::aligned_allocator<VectorType> > integral_image_;
      
      /** the width of the 2d input data array */
      unsigned width_;
      /** the height of the 2d input data array */
      unsigned height_;
      
      /** indicates whether second order integral images are available **/
      bool compute_second_order_integral_images_;
  };
}

#include <pcl/features/impl/integral_image2D.hpp>

#endif


