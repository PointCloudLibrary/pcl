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

template <class DataType, class IIDataType>
pcl::IntegralImage2D<DataType, IIDataType>::IntegralImage2D (
    DataType * data, const int width, const int height,
    const int dimensions, const bool compute_second_order_integral_images,
    const int element_stride,const int row_stride) : width_(width), height_(height),
  dimensions_(dimensions),
  element_stride_(element_stride),
  row_stride_(row_stride),
  are_second_order_ii_available_(compute_second_order_integral_images)
{
  // allocate memory
  for (int dimension_index = 0; dimension_index < dimensions; ++dimension_index)
  {
    IIDataType * data_array = new IIDataType[width*height]; // reinterpret_cast<IIDataType*>(new unsigned char(width*height*sizeof(IIDataType)));

    first_order_integral_images_.push_back(data_array);
  }

  if (compute_second_order_integral_images)
  {
    for (int dimension_index1 = 0; dimension_index1 < dimensions; ++dimension_index1)
    {
      second_order_integral_images_.push_back(::std::vector< IIDataType* >());
      for (int dimension_index2 = 0; dimension_index2 < dimensions; ++dimension_index2)
      {
        IIDataType * data_array = new IIDataType[width*height];

        second_order_integral_images_[dimension_index1].push_back(data_array);
      }
    }
  }


  // compute integral images
  if (dimensions_ == 1)
  {
    this->computeIntegralImagesOneDimensional (data);
  }
  else
  {
    this->computeIntegralImages (data);
  }
}


// ---------------------------------------------------------------------------- 
template <class DataType, class IIDataType>
pcl::IntegralImage2D<DataType, IIDataType>::~IntegralImage2D()
{
  for (unsigned int index = 0; index < first_order_integral_images_.size(); ++index)
  {
    delete[] (first_order_integral_images_[index]);
  }
  for (unsigned int index1 = 0; index1 < second_order_integral_images_.size(); ++index1)
  {
    for (unsigned int index2 = 0; index2 < second_order_integral_images_[index1].size(); ++index2)
    {
      delete[] (second_order_integral_images_[index1][index2]);
    }
  }
}


// ---------------------------------------------------------------------------- 
template <class DataType, class IIDataType> IIDataType 
pcl::IntegralImage2D<DataType, IIDataType>::getSum (
  const int start_x, const int start_y,
  const int width, const int height,
  const int dimension_index )
{
  const int l_ulX = start_x;
  const int l_ulY = start_y;
  const int l_lrX = start_x + width;
  const int l_lrY = start_y + height;
  
  return 
     (first_order_integral_images_[dimension_index][l_ulY*width_ + l_ulX]
    + first_order_integral_images_[dimension_index][l_lrY*width_ + l_lrX]
    - first_order_integral_images_[dimension_index][l_lrY*width_ + l_ulX]
    - first_order_integral_images_[dimension_index][l_ulY*width_ + l_lrX]);
}


// ---------------------------------------------------------------------------- 
template <class DataType, class IIDataType> IIDataType
pcl::IntegralImage2D<DataType, IIDataType>::getSum (
  const int start_x, const int start_y,
  const int width, const int height,
  const int dimension_index1, const int dimension_index2)
{
  const int l_ulX = start_x;
  const int l_ulY = start_y;
  const int l_lrX = start_x + width;
  const int l_lrY = start_y + height;
  
  return second_order_integral_images_[dimension_index1][dimension_index2][l_ulY*width_ + l_ulX]
    + second_order_integral_images_[dimension_index1][dimension_index2][l_lrY*width_ + l_lrX]
    - second_order_integral_images_[dimension_index1][dimension_index2][l_lrY*width_ + l_ulX]
    - second_order_integral_images_[dimension_index1][dimension_index2][l_ulY*width_ + l_lrX];
}


// ---------------------------------------------------------------------------- 
template <class DataType, class IIDataType> void
pcl::IntegralImage2D<DataType, IIDataType>::computeIntegralImages (DataType *data)
{
  // first element
  {
    const int row_index = 0;
    const int col_index = 0;
    for (int dimension_index = 0; dimension_index < dimensions_; ++dimension_index)
    {
      const IIDataType data_value = static_cast<IIDataType>(data[row_index*row_stride_ + col_index*element_stride_ + dimension_index]);

      first_order_integral_images_[dimension_index][row_index*width_ + col_index] = data_value;

      if (are_second_order_ii_available_)
      {
        for (int dimension_index2 = 0; dimension_index2 < dimensions_; ++dimension_index2)
        {
          const IIDataType data_value2 = static_cast<IIDataType>(data[row_index*row_stride_ + col_index*element_stride_ + dimension_index2]);

          second_order_integral_images_[dimension_index][dimension_index2][row_index*width_ + col_index] = data_value*data_value2;
        }
      }
    }
  }

  // first row
  {
    const int row_index = 0; 
    for (int col_index = 1; col_index < width_; ++col_index)
    {
      for (int dimension_index = 0; dimension_index < dimensions_; ++dimension_index)
      {
        const IIDataType data_value = static_cast<IIDataType>(data[row_index*row_stride_ + col_index*element_stride_ + dimension_index]);

        first_order_integral_images_[dimension_index][row_index*width_ + col_index] = data_value
          + first_order_integral_images_[dimension_index][row_index*width_ + (col_index-1)];

        if (are_second_order_ii_available_)
        {
          for (int dimension_index2 = 0; dimension_index2 < dimensions_; ++dimension_index2)
          {
            const IIDataType data_value2 = static_cast<IIDataType>(data[row_index*row_stride_ + col_index*element_stride_ + dimension_index2]);

            second_order_integral_images_[dimension_index][dimension_index2][row_index*width_ + col_index] = data_value*data_value2
              + second_order_integral_images_[dimension_index][dimension_index2][row_index*width_ + (col_index-1)];
          }
        }
      }
    }
  }

  // first column
  {
    const int col_index = 0;
    for (int row_index = 1; row_index < height_; ++row_index)
    {
      for (int dimension_index = 0; dimension_index < dimensions_; ++dimension_index)
      {
        const IIDataType data_value = static_cast<IIDataType>(data[row_index*row_stride_ + col_index*element_stride_ + dimension_index]);

        first_order_integral_images_[dimension_index][row_index*width_ + col_index] = data_value
          + first_order_integral_images_[dimension_index][(row_index-1)*width_ + col_index];

        if (are_second_order_ii_available_)
        {
          for (int dimension_index2 = 0; dimension_index2 < dimensions_; ++dimension_index2)
          {
            const IIDataType data_value2 = static_cast<IIDataType>(data[row_index*row_stride_ + col_index*element_stride_ + dimension_index2]);

            second_order_integral_images_[dimension_index][dimension_index2][row_index*width_ + col_index] = data_value*data_value2
              + second_order_integral_images_[dimension_index][dimension_index2][(row_index-1)*width_ + col_index];
          }
        }
      }
    }
  }

  // the rest
  for (int row_index = 1; row_index < height_; ++row_index)
  {
    for (int col_index = 1; col_index < width_; ++col_index)
    {
      for (int dimension_index = 0; dimension_index < dimensions_; ++dimension_index)
      {
        const IIDataType data_value = static_cast<IIDataType>(data[row_index*row_stride_ + col_index*element_stride_ + dimension_index]);

        first_order_integral_images_[dimension_index][row_index*width_ + col_index] = data_value
          + first_order_integral_images_[dimension_index][row_index*width_ + (col_index-1)]
          + first_order_integral_images_[dimension_index][(row_index-1)*width_ + col_index]
          - first_order_integral_images_[dimension_index][(row_index-1)*width_ + (col_index-1)];

        if (are_second_order_ii_available_)
        {
          for (int dimension_index2 = 0; dimension_index2 < dimensions_; ++dimension_index2)
          {
            const IIDataType data_value2 = static_cast<IIDataType>(data[row_index*row_stride_ + col_index*element_stride_ + dimension_index2]);

            second_order_integral_images_[dimension_index][dimension_index2][row_index*width_ + col_index] = data_value*data_value2
              + second_order_integral_images_[dimension_index][dimension_index2][row_index*width_ + (col_index-1)]
              + second_order_integral_images_[dimension_index][dimension_index2][(row_index-1)*width_ + col_index]
              - second_order_integral_images_[dimension_index][dimension_index2][(row_index-1)*width_ + (col_index-1)];
          }
        }
      }
    }
  }
}


// ---------------------------------------------------------------------------- 
template <class DataType, class IIDataType> void
pcl::IntegralImage2D<DataType, IIDataType>::computeIntegralImagesOneDimensional (DataType *data)
{
  IIDataType * first_order_integral_image = first_order_integral_images_[0];
  IIDataType * second_order_integral_image = NULL;
  if (are_second_order_ii_available_)
  {
    second_order_integral_image = second_order_integral_images_[0][0];
  }

  // first element
  {
    const int row_index = 0;
    const int col_index = 0;

    const IIDataType data_value = static_cast<IIDataType>(data[row_index*row_stride_ + col_index*element_stride_]);

    first_order_integral_image[row_index*width_ + col_index] = data_value;

    if (are_second_order_ii_available_)
    {
      second_order_integral_image[row_index*width_ + col_index] = data_value*data_value;
    }
  }

  // first row
  {
    const int row_index = 0; 
    for (int col_index = 1; col_index < width_; ++col_index)
    {
      const IIDataType data_value = static_cast<IIDataType>(data[row_index*row_stride_ + col_index*element_stride_]);

      first_order_integral_image[row_index*width_ + col_index] = data_value
        + first_order_integral_image[row_index*width_ + (col_index-1)];

      if (are_second_order_ii_available_)
      {
        second_order_integral_image[row_index*width_ + col_index] = data_value*data_value
          + second_order_integral_image[row_index*width_ + (col_index-1)];
      }
    }
  }

  // first column
  {
    const int col_index = 0;
    for (int row_index = 1; row_index < height_; ++row_index)
    {
      const IIDataType data_value = static_cast<IIDataType>(data[row_index*row_stride_ + col_index*element_stride_]);

      first_order_integral_image[row_index*width_ + col_index] = data_value
        + first_order_integral_image[(row_index-1)*width_ + col_index];

      if (are_second_order_ii_available_)
      {
        second_order_integral_image[row_index*width_ + col_index] = data_value*data_value
          + second_order_integral_image[(row_index-1)*width_ + col_index];
      }
    }
  }

  // the rest
  for (int row_index = 1; row_index < height_; ++row_index)
  {
    for (int col_index = 1; col_index < width_; ++col_index)
    {
      const IIDataType data_value = static_cast<IIDataType>(data[row_index*row_stride_ + col_index*element_stride_]);

      first_order_integral_image[row_index*width_ + col_index] = data_value
        - first_order_integral_image[(row_index-1)*width_ + (col_index-1)]
        + first_order_integral_image[(row_index-1)*width_ + col_index]
        + first_order_integral_image[row_index*width_ + (col_index-1)];

      if (are_second_order_ii_available_)
      {
        second_order_integral_image[row_index*width_ + col_index] = data_value*data_value
          + second_order_integral_image[row_index*width_ + (col_index-1)]
          + second_order_integral_image[(row_index-1)*width_ + col_index]
          - second_order_integral_image[(row_index-1)*width_ + (col_index-1)];
      }
    }
  }
}

