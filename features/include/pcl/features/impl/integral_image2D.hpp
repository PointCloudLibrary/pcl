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


#ifndef PCL_INTEGRAL_IMAGE2D_IMPL_H_
#define PCL_INTEGRAL_IMAGE2D_IMPL_H_


namespace pcl
{

template <typename DataType, unsigned Dimension> void
IntegralImage2D<DataType, Dimension>::setSecondOrderComputation (bool compute_second_order_integral_images)
{
  compute_second_order_integral_images_ = compute_second_order_integral_images;
}


template <typename DataType, unsigned Dimension> void
IntegralImage2D<DataType, Dimension>::setInput (const DataType * data, unsigned width,unsigned height, unsigned element_stride, unsigned row_stride)
{
  if ((width + 1) * (height + 1) > first_order_integral_image_.size () )
  {
    width_  = width;
    height_ = height;
    first_order_integral_image_.resize ( (width_ + 1) * (height_ + 1) );
    finite_values_integral_image_.resize ( (width_ + 1) * (height_ + 1) );
    if (compute_second_order_integral_images_)
      second_order_integral_image_.resize ( (width_ + 1) * (height_ + 1) );
  }
  computeIntegralImages (data, row_stride, element_stride);
}


template <typename DataType, unsigned Dimension> typename pcl::IntegralImage2D<DataType, Dimension>::ElementType
IntegralImage2D<DataType, Dimension>::getFirstOrderSum (
    unsigned start_x, unsigned start_y, unsigned width, unsigned height) const
{
  const unsigned upper_left_idx      = start_y * (width_ + 1) + start_x;
  const unsigned upper_right_idx     = upper_left_idx + width;
  const unsigned lower_left_idx      = (start_y + height) * (width_ + 1) + start_x;
  const unsigned lower_right_idx     = lower_left_idx + width;

  return (first_order_integral_image_[lower_right_idx] + first_order_integral_image_[upper_left_idx]  -
          first_order_integral_image_[upper_right_idx] - first_order_integral_image_[lower_left_idx]  );
}


template <typename DataType, unsigned Dimension> typename pcl::IntegralImage2D<DataType, Dimension>::SecondOrderType
IntegralImage2D<DataType, Dimension>::getSecondOrderSum (
    unsigned start_x, unsigned start_y, unsigned width, unsigned height) const
{
  const unsigned upper_left_idx      = start_y * (width_ + 1) + start_x;
  const unsigned upper_right_idx     = upper_left_idx + width;
  const unsigned lower_left_idx      = (start_y + height) * (width_ + 1) + start_x;
  const unsigned lower_right_idx     = lower_left_idx + width;

  return (second_order_integral_image_[lower_right_idx] + second_order_integral_image_[upper_left_idx]  -
          second_order_integral_image_[upper_right_idx] - second_order_integral_image_[lower_left_idx]  );
}


template <typename DataType, unsigned Dimension> unsigned
IntegralImage2D<DataType, Dimension>::getFiniteElementsCount (
    unsigned start_x, unsigned start_y, unsigned width, unsigned height) const
{
  const unsigned upper_left_idx      = start_y * (width_ + 1) + start_x;
  const unsigned upper_right_idx     = upper_left_idx + width;
  const unsigned lower_left_idx      = (start_y + height) * (width_ + 1) + start_x;
  const unsigned lower_right_idx     = lower_left_idx + width;

  return (finite_values_integral_image_[lower_right_idx] + finite_values_integral_image_[upper_left_idx]  -
          finite_values_integral_image_[upper_right_idx] - finite_values_integral_image_[lower_left_idx]  );
}


template <typename DataType, unsigned Dimension> typename pcl::IntegralImage2D<DataType, Dimension>::ElementType
IntegralImage2D<DataType, Dimension>::getFirstOrderSumSE (
    unsigned start_x, unsigned start_y, unsigned end_x, unsigned end_y) const
{
  const unsigned upper_left_idx      = start_y * (width_ + 1) + start_x;
  const unsigned upper_right_idx     = start_y * (width_ + 1) + end_x;
  const unsigned lower_left_idx      = end_y * (width_ + 1) + start_x;
  const unsigned lower_right_idx     = end_y * (width_ + 1) + end_x;

  return (first_order_integral_image_[lower_right_idx] + first_order_integral_image_[upper_left_idx]  -
          first_order_integral_image_[upper_right_idx] - first_order_integral_image_[lower_left_idx]  );
}


template <typename DataType, unsigned Dimension> typename pcl::IntegralImage2D<DataType, Dimension>::SecondOrderType
IntegralImage2D<DataType, Dimension>::getSecondOrderSumSE (
    unsigned start_x, unsigned start_y, unsigned end_x, unsigned end_y) const
{
  const unsigned upper_left_idx      = start_y * (width_ + 1) + start_x;
  const unsigned upper_right_idx     = start_y * (width_ + 1) + end_x;
  const unsigned lower_left_idx      = end_y * (width_ + 1) + start_x;
  const unsigned lower_right_idx     = end_y * (width_ + 1) + end_x;

  return (second_order_integral_image_[lower_right_idx] + second_order_integral_image_[upper_left_idx]  -
          second_order_integral_image_[upper_right_idx] - second_order_integral_image_[lower_left_idx]  );
}


template <typename DataType, unsigned Dimension> unsigned
IntegralImage2D<DataType, Dimension>::getFiniteElementsCountSE (
    unsigned start_x, unsigned start_y, unsigned end_x, unsigned end_y) const
{
  const unsigned upper_left_idx      = start_y * (width_ + 1) + start_x;
  const unsigned upper_right_idx     = start_y * (width_ + 1) + end_x;
  const unsigned lower_left_idx      = end_y * (width_ + 1) + start_x;
  const unsigned lower_right_idx     = end_y * (width_ + 1) + end_x;

  return (finite_values_integral_image_[lower_right_idx] + finite_values_integral_image_[upper_left_idx]  -
          finite_values_integral_image_[upper_right_idx] - finite_values_integral_image_[lower_left_idx]  );
}


template <typename DataType, unsigned Dimension> void
IntegralImage2D<DataType, Dimension>::computeIntegralImages (
    const DataType *data, unsigned row_stride, unsigned element_stride)
{
  ElementType* previous_row = &first_order_integral_image_[0];
  ElementType* current_row  = previous_row + (width_ + 1);
  for (unsigned int i = 0; i < (width_ + 1); ++i)
    previous_row[i].setZero();

  unsigned* count_previous_row = &finite_values_integral_image_[0];
  unsigned* count_current_row  = count_previous_row + (width_ + 1);
  std::fill_n(count_previous_row, width_ + 1, 0);

  if (!compute_second_order_integral_images_)
  {
    for (unsigned rowIdx = 0; rowIdx < height_; ++rowIdx, data += row_stride,
                                                previous_row = current_row, current_row += (width_ + 1),
                                                count_previous_row = count_current_row, count_current_row += (width_ + 1))
    {
      current_row [0].setZero ();
      count_current_row [0] = 0;
      for (unsigned colIdx = 0, valIdx = 0; colIdx < width_; ++colIdx, valIdx += element_stride)
      {
        current_row [colIdx + 1] = previous_row [colIdx + 1] + current_row [colIdx] - previous_row [colIdx];
        count_current_row [colIdx + 1] = count_previous_row [colIdx + 1] + count_current_row [colIdx] - count_previous_row [colIdx];
        const auto* element = reinterpret_cast <const InputType*> (&data [valIdx]);
        if (std::isfinite (element->sum ()))
        {
          current_row [colIdx + 1] += element->template cast<typename IntegralImageTypeTraits<DataType>::IntegralType>();
          ++(count_current_row [colIdx + 1]);
        }
      }
    }
  }
  else
  {
    SecondOrderType* so_previous_row = &second_order_integral_image_[0];
    SecondOrderType* so_current_row  = so_previous_row + (width_ + 1);
    for (unsigned int i = 0; i < (width_ + 1); ++i)
      so_previous_row[i].setZero();

    SecondOrderType so_element;
    for (unsigned rowIdx = 0; rowIdx < height_; ++rowIdx, data += row_stride,
                                                previous_row = current_row, current_row += (width_ + 1),
                                                count_previous_row = count_current_row, count_current_row += (width_ + 1),
                                                so_previous_row = so_current_row, so_current_row += (width_ + 1))
    {
      current_row [0].setZero ();
      so_current_row [0].setZero ();
      count_current_row [0] = 0;
      for (unsigned colIdx = 0, valIdx = 0; colIdx < width_; ++colIdx, valIdx += element_stride)
      {
        current_row [colIdx + 1] = previous_row [colIdx + 1] + current_row [colIdx] - previous_row [colIdx];
        so_current_row [colIdx + 1] = so_previous_row [colIdx + 1] + so_current_row [colIdx] - so_previous_row [colIdx];
        count_current_row [colIdx + 1] = count_previous_row [colIdx + 1] + count_current_row [colIdx] - count_previous_row [colIdx];

        const auto* element = reinterpret_cast <const InputType*> (&data [valIdx]);
        if (std::isfinite (element->sum ()))
        {
          current_row [colIdx + 1] += element->template cast<typename IntegralImageTypeTraits<DataType>::IntegralType>();
          ++(count_current_row [colIdx + 1]);
          for (unsigned myIdx = 0, elIdx = 0; myIdx < Dimension; ++myIdx)
            for (unsigned mxIdx = myIdx; mxIdx < Dimension; ++mxIdx, ++elIdx)
              so_current_row [colIdx + 1][elIdx] += (*element)[myIdx] * (*element)[mxIdx];
        }
      }
    }
  }
}


template <typename DataType> void
IntegralImage2D<DataType, 1>::setInput (const DataType * data, unsigned width,unsigned height, unsigned element_stride, unsigned row_stride)
{
  if ((width + 1) * (height + 1) > first_order_integral_image_.size () )
  {
    width_  = width;
    height_ = height;
    first_order_integral_image_.resize ( (width_ + 1) * (height_ + 1) );
    finite_values_integral_image_.resize ( (width_ + 1) * (height_ + 1) );
    if (compute_second_order_integral_images_)
      second_order_integral_image_.resize ( (width_ + 1) * (height_ + 1) );
  }
  computeIntegralImages (data, row_stride, element_stride);
}


template <typename DataType> typename pcl::IntegralImage2D<DataType, 1>::ElementType
IntegralImage2D<DataType, 1>::getFirstOrderSum (
    unsigned start_x, unsigned start_y, unsigned width, unsigned height) const
{
  const unsigned upper_left_idx      = start_y * (width_ + 1) + start_x;
  const unsigned upper_right_idx     = upper_left_idx + width;
  const unsigned lower_left_idx      = (start_y + height) * (width_ + 1) + start_x;
  const unsigned lower_right_idx     = lower_left_idx + width;

  return (first_order_integral_image_[lower_right_idx] + first_order_integral_image_[upper_left_idx]  -
          first_order_integral_image_[upper_right_idx] - first_order_integral_image_[lower_left_idx]  );
}


template <typename DataType> typename pcl::IntegralImage2D<DataType, 1>::SecondOrderType
IntegralImage2D<DataType, 1>::getSecondOrderSum (
    unsigned start_x, unsigned start_y, unsigned width, unsigned height) const
{
  const unsigned upper_left_idx      = start_y * (width_ + 1) + start_x;
  const unsigned upper_right_idx     = upper_left_idx + width;
  const unsigned lower_left_idx      = (start_y + height) * (width_ + 1) + start_x;
  const unsigned lower_right_idx     = lower_left_idx + width;

  return (second_order_integral_image_[lower_right_idx] + second_order_integral_image_[upper_left_idx]  -
          second_order_integral_image_[upper_right_idx] - second_order_integral_image_[lower_left_idx]  );
}


template <typename DataType> unsigned
IntegralImage2D<DataType, 1>::getFiniteElementsCount (
    unsigned start_x, unsigned start_y, unsigned width, unsigned height) const
{
  const unsigned upper_left_idx      = start_y * (width_ + 1) + start_x;
  const unsigned upper_right_idx     = upper_left_idx + width;
  const unsigned lower_left_idx      = (start_y + height) * (width_ + 1) + start_x;
  const unsigned lower_right_idx     = lower_left_idx + width;

  return (finite_values_integral_image_[lower_right_idx] + finite_values_integral_image_[upper_left_idx]  -
          finite_values_integral_image_[upper_right_idx] - finite_values_integral_image_[lower_left_idx]  );
}


template <typename DataType> typename pcl::IntegralImage2D<DataType, 1>::ElementType
IntegralImage2D<DataType, 1>::getFirstOrderSumSE (
    unsigned start_x, unsigned start_y, unsigned end_x, unsigned end_y) const
{
  const unsigned upper_left_idx      = start_y * (width_ + 1) + start_x;
  const unsigned upper_right_idx     = start_y * (width_ + 1) + end_x;
  const unsigned lower_left_idx      = end_y * (width_ + 1) + start_x;
  const unsigned lower_right_idx     = end_y * (width_ + 1) + end_x;

  return (first_order_integral_image_[lower_right_idx] + first_order_integral_image_[upper_left_idx]  -
          first_order_integral_image_[upper_right_idx] - first_order_integral_image_[lower_left_idx]  );
}


template <typename DataType> typename pcl::IntegralImage2D<DataType, 1>::SecondOrderType
IntegralImage2D<DataType, 1>::getSecondOrderSumSE (
    unsigned start_x, unsigned start_y, unsigned end_x, unsigned end_y) const
{
  const unsigned upper_left_idx      = start_y * (width_ + 1) + start_x;
  const unsigned upper_right_idx     = start_y * (width_ + 1) + end_x;
  const unsigned lower_left_idx      = end_y * (width_ + 1) + start_x;
  const unsigned lower_right_idx     = end_y * (width_ + 1) + end_x;

  return (second_order_integral_image_[lower_right_idx] + second_order_integral_image_[upper_left_idx]  -
          second_order_integral_image_[upper_right_idx] - second_order_integral_image_[lower_left_idx]  );
}


template <typename DataType> unsigned
IntegralImage2D<DataType, 1>::getFiniteElementsCountSE (
    unsigned start_x, unsigned start_y, unsigned end_x, unsigned end_y) const
{
  const unsigned upper_left_idx      = start_y * (width_ + 1) + start_x;
  const unsigned upper_right_idx     = start_y * (width_ + 1) + end_x;
  const unsigned lower_left_idx      = end_y * (width_ + 1) + start_x;
  const unsigned lower_right_idx     = end_y * (width_ + 1) + end_x;

  return (finite_values_integral_image_[lower_right_idx] + finite_values_integral_image_[upper_left_idx]  -
          finite_values_integral_image_[upper_right_idx] - finite_values_integral_image_[lower_left_idx]  );
}


template <typename DataType> void
IntegralImage2D<DataType, 1>::computeIntegralImages (
    const DataType *data, unsigned row_stride, unsigned element_stride)
{
  ElementType* previous_row = &first_order_integral_image_[0];
  ElementType* current_row  = previous_row + (width_ + 1);
  std::fill_n(previous_row, width_ + 1, 0);

  unsigned* count_previous_row = &finite_values_integral_image_[0];
  unsigned* count_current_row  = count_previous_row + (width_ + 1);
  std::fill_n(count_previous_row, width_ + 1, 0);

  if (!compute_second_order_integral_images_)
  {
    for (unsigned rowIdx = 0; rowIdx < height_; ++rowIdx, data += row_stride,
                                                previous_row = current_row, current_row += (width_ + 1),
                                                count_previous_row = count_current_row, count_current_row += (width_ + 1))
    {
      current_row [0] = 0.0;
      count_current_row [0] = 0;
      for (unsigned colIdx = 0, valIdx = 0; colIdx < width_; ++colIdx, valIdx += element_stride)
      {
        current_row [colIdx + 1] = previous_row [colIdx + 1] + current_row [colIdx] - previous_row [colIdx];
        count_current_row [colIdx + 1] = count_previous_row [colIdx + 1] + count_current_row [colIdx] - count_previous_row [colIdx];
        if (std::isfinite (data [valIdx]))
        {
          current_row [colIdx + 1] += data [valIdx];
          ++(count_current_row [colIdx + 1]);
        }
      }
    }
  }
  else
  {
    SecondOrderType* so_previous_row = &second_order_integral_image_[0];
    SecondOrderType* so_current_row  = so_previous_row + (width_ + 1);
    std::fill_n(so_previous_row, width_ + 1, 0);

    for (unsigned rowIdx = 0; rowIdx < height_; ++rowIdx, data += row_stride,
                                                previous_row = current_row, current_row += (width_ + 1),
                                                count_previous_row = count_current_row, count_current_row += (width_ + 1),
                                                so_previous_row = so_current_row, so_current_row += (width_ + 1))
    {
      current_row [0] = 0.0;
      so_current_row [0] = 0.0;
      count_current_row [0] = 0;
      for (unsigned colIdx = 0, valIdx = 0; colIdx < width_; ++colIdx, valIdx += element_stride)
      {
        current_row [colIdx + 1] = previous_row [colIdx + 1] + current_row [colIdx] - previous_row [colIdx];
        so_current_row [colIdx + 1] = so_previous_row [colIdx + 1] + so_current_row [colIdx] - so_previous_row [colIdx];
        count_current_row [colIdx + 1] = count_previous_row [colIdx + 1] + count_current_row [colIdx] - count_previous_row [colIdx];
        if (std::isfinite (data[valIdx]))
        {
          current_row [colIdx + 1] += data[valIdx];
          so_current_row [colIdx + 1] += data[valIdx] * data[valIdx];
          ++(count_current_row [colIdx + 1]);
        }
      }
    }
  }
}

} // namespace pcl

#endif    // PCL_INTEGRAL_IMAGE2D_IMPL_H_

