/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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
 * $Id: feature.h 2784 2011-10-15 22:05:38Z aichim $
 */

#pragma once

#include <pcl/memory.h>

#include <vector>


namespace pcl
{
  template <typename DataType>
  struct IntegralImageTypeTraits
  {
    using Type = DataType;
    using IntegralType = DataType;
  };

  template <>
  struct IntegralImageTypeTraits<float>
  {
    using Type = float;
    using IntegralType = double;
  };

  template <>
  struct IntegralImageTypeTraits<char>
  {
    using Type = char;
    using IntegralType = int;
  };

  template <>
  struct IntegralImageTypeTraits<short>
  {
    using Type = short;
    using IntegralType = long;
  };

  template <>
  struct IntegralImageTypeTraits<unsigned short>
  {
    using Type = unsigned short;
    using IntegralType = unsigned long;
  };

  template <>
  struct IntegralImageTypeTraits<unsigned char>
  {
    using Type = unsigned char;
    using IntegralType = unsigned int;
  };

  template <>
  struct IntegralImageTypeTraits<int>
  {
    using Type = int;
    using IntegralType = long;
  };

  template <>
  struct IntegralImageTypeTraits<unsigned int>
  {
    using Type = unsigned int;
    using IntegralType = unsigned long;
  };

  /** \brief Determines an integral image representation for a given organized data array
    * \author Suat Gedikli
    */
  template <class DataType, unsigned Dimension>
  class IntegralImage2D
  {
    public:
      using Ptr = shared_ptr<IntegralImage2D<DataType, Dimension>>;
      using ConstPtr = shared_ptr<const IntegralImage2D<DataType, Dimension>>;
      static const unsigned second_order_size = (Dimension * (Dimension + 1)) >> 1;
      using ElementType = Eigen::Matrix<typename IntegralImageTypeTraits<DataType>::IntegralType, Dimension, 1>;
      using SecondOrderType = Eigen::Matrix<typename IntegralImageTypeTraits<DataType>::IntegralType, second_order_size, 1>;

      /** \brief Constructor for an Integral Image
        * \param[in] compute_second_order_integral_images set to true if we want to compute a second order image
        */
      IntegralImage2D (bool compute_second_order_integral_images) :
        first_order_integral_image_ (),
        second_order_integral_image_ (),
        width_ (1), 
        height_ (1), 
        compute_second_order_integral_images_ (compute_second_order_integral_images)
      {
      }

      /** \brief Destructor */
      virtual
      ~IntegralImage2D () = default;

      /** \brief sets the computation for second order integral images on or off.
        * \param compute_second_order_integral_images
        */
      void 
      setSecondOrderComputation (bool compute_second_order_integral_images);

      /** \brief Set the input data to compute the integral image for
        * \param[in] data the input data
        * \param[in] width the width of the data
        * \param[in] height the height of the data
        * \param[in] element_stride the element stride of the data
        * \param[in] row_stride the row stride of the data
        */
      void
      setInput (const DataType * data,
                unsigned width, unsigned height, unsigned element_stride, unsigned row_stride);

      /** \brief Compute the first order sum within a given rectangle
        * \param[in] start_x x position of rectangle
        * \param[in] start_y y position of rectangle
        * \param[in] width width of rectangle
        * \param[in] height height of rectangle
        */
      inline ElementType
      getFirstOrderSum (unsigned start_x, unsigned start_y, unsigned width, unsigned height) const;

      /** \brief Compute the first order sum within a given rectangle
        * \param[in] start_x x position of the start of the rectangle
        * \param[in] start_y x position of the start of the rectangle
        * \param[in] end_x x position of the end of the rectangle
        * \param[in] end_y x position of the end of the rectangle
        */
      inline ElementType
      getFirstOrderSumSE (unsigned start_x, unsigned start_y, unsigned end_x, unsigned end_y) const;

      /** \brief Compute the second order sum within a given rectangle
        * \param[in] start_x x position of rectangle
        * \param[in] start_y y position of rectangle
        * \param[in] width width of rectangle
        * \param[in] height height of rectangle
        */
      inline SecondOrderType
      getSecondOrderSum (unsigned start_x, unsigned start_y, unsigned width, unsigned height) const;

      /** \brief Compute the second order sum within a given rectangle
        * \param[in] start_x x position of the start of the rectangle
        * \param[in] start_y x position of the start of the rectangle
        * \param[in] end_x x position of the end of the rectangle
        * \param[in] end_y x position of the end of the rectangle
        */
      inline SecondOrderType
      getSecondOrderSumSE (unsigned start_x, unsigned start_y, unsigned end_x, unsigned end_y) const;

      /** \brief Compute the number of finite elements within a given rectangle
        * \param[in] start_x x position of rectangle
        * \param[in] start_y y position of rectangle
        * \param[in] width width of rectangle
        * \param[in] height height of rectangle
        */
      inline unsigned
      getFiniteElementsCount (unsigned start_x, unsigned start_y, unsigned width, unsigned height) const;

      /** \brief Compute the number of finite elements within a given rectangle
        * \param[in] start_x x position of the start of the rectangle
        * \param[in] start_y x position of the start of the rectangle
        * \param[in] end_x x position of the end of the rectangle
        * \param[in] end_y x position of the end of the rectangle
        */
      inline unsigned
      getFiniteElementsCountSE (unsigned start_x, unsigned start_y, unsigned end_x, unsigned end_y) const;

    private:
      using InputType = Eigen::Matrix<typename IntegralImageTypeTraits<DataType>::Type, Dimension, 1>;

      /** \brief Compute the actual integral image data
        * \param[in] data the input data
        * \param[in] element_stride the element stride of the data
        * \param[in] row_stride the row stride of the data
        */
      void
      computeIntegralImages (const DataType * data, unsigned row_stride, unsigned element_stride);

      std::vector<ElementType, Eigen::aligned_allocator<ElementType> > first_order_integral_image_;
      std::vector<SecondOrderType, Eigen::aligned_allocator<SecondOrderType> > second_order_integral_image_;
      std::vector<unsigned> finite_values_integral_image_;

      /** \brief The width of the 2d input data array */
      unsigned width_;
      /** \brief The height of the 2d input data array */
      unsigned height_;

      /** \brief Indicates whether second order integral images are available **/
      bool compute_second_order_integral_images_;
   };

   /**
     * \brief partial template specialization for integral images with just one channel.
     */
  template <class DataType>
  class IntegralImage2D <DataType, 1>
  {
    public:
      using Ptr = shared_ptr<IntegralImage2D<DataType, 1>>;
      using ConstPtr = shared_ptr<const IntegralImage2D<DataType, 1>>;

      static const unsigned second_order_size = 1;
      using ElementType = typename IntegralImageTypeTraits<DataType>::IntegralType;
      using SecondOrderType = typename IntegralImageTypeTraits<DataType>::IntegralType;

      /** \brief Constructor for an Integral Image
        * \param[in] compute_second_order_integral_images set to true if we want to compute a second order image
        */
      IntegralImage2D (bool compute_second_order_integral_images) : 
        first_order_integral_image_ (),
        second_order_integral_image_ (),
        
        width_ (1), height_ (1), 
        compute_second_order_integral_images_ (compute_second_order_integral_images)
      {
      }

      /** \brief Destructor */
      virtual
      ~IntegralImage2D () = default;

      /** \brief Set the input data to compute the integral image for
        * \param[in] data the input data
        * \param[in] width the width of the data
        * \param[in] height the height of the data
        * \param[in] element_stride the element stride of the data
        * \param[in] row_stride the row stride of the data
        */
      void
      setInput (const DataType * data,
                unsigned width, unsigned height, unsigned element_stride, unsigned row_stride);

      /** \brief Compute the first order sum within a given rectangle
        * \param[in] start_x x position of rectangle
        * \param[in] start_y y position of rectangle
        * \param[in] width width of rectangle
        * \param[in] height height of rectangle
        */
      inline ElementType
      getFirstOrderSum (unsigned start_x, unsigned start_y, unsigned width, unsigned height) const;

      /** \brief Compute the first order sum within a given rectangle
        * \param[in] start_x x position of the start of the rectangle
        * \param[in] start_y x position of the start of the rectangle
        * \param[in] end_x x position of the end of the rectangle
        * \param[in] end_y x position of the end of the rectangle
        */
      inline ElementType
      getFirstOrderSumSE (unsigned start_x, unsigned start_y, unsigned end_x, unsigned end_y) const;

      /** \brief Compute the second order sum within a given rectangle
        * \param[in] start_x x position of rectangle
        * \param[in] start_y y position of rectangle
        * \param[in] width width of rectangle
        * \param[in] height height of rectangle
        */
      inline SecondOrderType
      getSecondOrderSum (unsigned start_x, unsigned start_y, unsigned width, unsigned height) const;

      /** \brief Compute the second order sum within a given rectangle
        * \param[in] start_x x position of the start of the rectangle
        * \param[in] start_y x position of the start of the rectangle
        * \param[in] end_x x position of the end of the rectangle
        * \param[in] end_y x position of the end of the rectangle
        */
      inline SecondOrderType
      getSecondOrderSumSE (unsigned start_x, unsigned start_y, unsigned end_x, unsigned end_y) const;

      /** \brief Compute the number of finite elements within a given rectangle
        * \param[in] start_x x position of rectangle
        * \param[in] start_y y position of rectangle
        * \param[in] width width of rectangle
        * \param[in] height height of rectangle
        */
      inline unsigned
      getFiniteElementsCount (unsigned start_x, unsigned start_y, unsigned width, unsigned height) const;

      /** \brief Compute the number of finite elements within a given rectangle
        * \param[in] start_x x position of the start of the rectangle
        * \param[in] start_y x position of the start of the rectangle
        * \param[in] end_x x position of the end of the rectangle
        * \param[in] end_y x position of the end of the rectangle
        */
      inline unsigned
      getFiniteElementsCountSE (unsigned start_x, unsigned start_y, unsigned end_x, unsigned end_y) const;

  private:
    //  using InputType = typename IntegralImageTypeTraits<DataType>::Type;

      /** \brief Compute the actual integral image data
        * \param[in] data the input data
        * \param[in] element_stride the element stride of the data
        * \param[in] row_stride the row stride of the data
        */
      void
      computeIntegralImages (const DataType * data, unsigned row_stride, unsigned element_stride);

      std::vector<ElementType, Eigen::aligned_allocator<ElementType> > first_order_integral_image_;
      std::vector<SecondOrderType, Eigen::aligned_allocator<SecondOrderType> > second_order_integral_image_;
      std::vector<unsigned> finite_values_integral_image_;

      /** \brief The width of the 2d input data array */
      unsigned width_;
      /** \brief The height of the 2d input data array */
      unsigned height_;

      /** \brief Indicates whether second order integral images are available **/
      bool compute_second_order_integral_images_;
   };
 }

#include <pcl/features/impl/integral_image2D.hpp>
