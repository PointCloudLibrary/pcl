/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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
 * $Id$
 *
 */

#ifndef PCL_2D_CONVOLUTION_IMPL_HPP
#define PCL_2D_CONVOLUTION_IMPL_HPP

template<typename PointT>
void
pcl::pcl_2d::convolution<PointT>::convolve (pcl::PointCloud<PointT> &output)
{
  int rows = input_->height;
  int cols = input_->width;
  int k_rows = kernel_.height;
  int k_cols = kernel_.width;

  int input_row = 0;
  int input_col = 0;
  /*default boundary option : zero padding*/
  output = *input_;
  typedef pcl::PointCloud<PointT> CloudT;

  typedef typename pcl::traits::fieldList<typename CloudT::PointType>::type FieldList;

  typename CloudT::PointType test_point = input_->points[0];

  bool has_intensity;
  //bool has_rgb;
  float output_intensity, output_r, output_g, output_b;
  float input_intensity, input_r, input_g, input_b;
  float kernel_intensity, kernel_r, kernel_g, kernel_b;
  pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> (test_point, "intensity", has_intensity, output_intensity));
  pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> (test_point, "r", has_intensity, output_r));
  pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> (test_point, "g", has_intensity, output_g));
  pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> (test_point, "b", has_intensity, output_b));
  if (boundary_options_ == BOUNDARY_OPTION_CLAMP)
  {
    for (int i = 0; i < rows; i++)
    {
      for (int j = 0; j < cols; j++)
      {
        //output (j, i).intensity = 0;
        if(image_channel_ == IMAGE_CHANNEL_INTENSITY){
          output_intensity = 0;
        }
        if(image_channel_ == IMAGE_CHANNEL_RGB){
          output_r = 0;
          output_g = 0;
          output_b = 0;
        }
        for (int k = 0; k < k_rows; k++)
        {
          for (int l = 0; l < k_cols; l++)
          {
            if ((i + k - k_rows / 2) < 0)
              input_row = 0;
            else
              if ((i + k - k_rows / 2) >= rows)
              {
                input_row = rows - 1;
              }
              else
                input_row = i + k - k_rows / 2;
            if ((j + l - k_cols / 2) < 0)
              input_col = 0;
            else
              if ((j + l - k_cols / 2) >= cols)
                input_col = cols - 1;
              else
                input_col = j + l - k_cols / 2;
            if(image_channel_ == IMAGE_CHANNEL_INTENSITY){
              pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> ((*input_)(input_col, input_row), "intensity", has_intensity, input_intensity));
              pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> (kernel_ (l, k), "intensity", has_intensity, kernel_intensity));
              output_intensity += kernel_intensity*input_intensity;
            }
            if(image_channel_ == IMAGE_CHANNEL_RGB){
              pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> ((*input_)(input_col, input_row), "r", has_intensity, input_r));
              pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> (kernel_ (l, k), "r", has_intensity, kernel_r));
              output_r += kernel_r*input_r;
              pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> ((*input_)(input_col, input_row), "g", has_intensity, input_g));
              pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> (kernel_ (l, k), "g", has_intensity, kernel_g));
              output_g += kernel_r*input_g;
              pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> ((*input_)(input_col, input_row), "b", has_intensity, input_b));
              pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> (kernel_ (l, k), "b", has_intensity, kernel_b));
              output_b += kernel_r*input_b;
            }
            //output (j, i).intensity += kernel_ (l, k).intensity * ((*input_)(input_col, input_row).intensity);
          }
        }
        if(image_channel_ == IMAGE_CHANNEL_INTENSITY){
          pcl::for_each_type<FieldList> (pcl::SetIfFieldExists<typename CloudT::PointType, float> (output (j, i), "intensity", output_intensity));
        }
        if(image_channel_ == IMAGE_CHANNEL_RGB){
          pcl::for_each_type<FieldList> (pcl::SetIfFieldExists<typename CloudT::PointType, float> (output (j, i), "r", output_r));
          pcl::for_each_type<FieldList> (pcl::SetIfFieldExists<typename CloudT::PointType, float> (output (j, i), "g", output_g));
          pcl::for_each_type<FieldList> (pcl::SetIfFieldExists<typename CloudT::PointType, float> (output (j, i), "b", output_b));
        }
      }
    }
  }
  else
    if (boundary_options_ == BOUNDARY_OPTION_MIRROR)
    {
      for (int i = 0; i < rows; i++)
      {
        for (int j = 0; j < cols; j++)
        {
          //output (j, i).intensity = 0;
          if(image_channel_ == IMAGE_CHANNEL_INTENSITY){
            output_intensity = 0;
          }
          if(image_channel_ == IMAGE_CHANNEL_RGB){
            output_r = 0;
            output_g = 0;
            output_b = 0;
          }
          for (int k = 0; k < k_rows; k++)
          {
            for (int l = 0; l < k_cols; l++)
            {
              if ((i + k - (k_rows / 2)) < 0)
                input_row = -(i + k - (k_rows / 2)) - 1;
              else
                if ((i + k - (k_rows / 2)) >= rows)
                {
                  input_row = 2 * rows - 1 - (i + k - (k_rows / 2));
                }
                else
                  input_row = i + k - (k_rows / 2);

              if ((j + l - (k_cols / 2)) < 0)
                input_col = -(j + l - (k_cols / 2)) - 1;
              else
                if ((j + l - (k_cols / 2)) >= cols)
                  input_col = 2 * cols - 1 - (j + l - (k_cols / 2));
                else
                  input_col = j + l - (k_cols / 2);

              //output (j, i).intensity += kernel_ (l, k).intensity * ((*input_)(input_col, input_row).intensity);
              if(image_channel_ == IMAGE_CHANNEL_INTENSITY){
                pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> ((*input_)(input_col, input_row), "intensity", has_intensity, input_intensity));
                pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> (kernel_ (l, k), "intensity", has_intensity, kernel_intensity));
                output_intensity += kernel_intensity*input_intensity;
              }
              if(image_channel_ == IMAGE_CHANNEL_RGB){
                pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> ((*input_)(input_col, input_row), "r", has_intensity, input_r));
                pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> (kernel_ (l, k), "r", has_intensity, kernel_r));
                output_r += kernel_r*input_r;
                pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> ((*input_)(input_col, input_row), "g", has_intensity, input_g));
                pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> (kernel_ (l, k), "g", has_intensity, kernel_g));
                output_g += kernel_r*input_g;
                pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> ((*input_)(input_col, input_row), "b", has_intensity, input_b));
                pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> (kernel_ (l, k), "b", has_intensity, kernel_b));
                output_b += kernel_r*input_b;
              }
            }
          }
          if(image_channel_ == IMAGE_CHANNEL_INTENSITY){
            pcl::for_each_type<FieldList> (pcl::SetIfFieldExists<typename CloudT::PointType, float> (output (j, i), "intensity", output_intensity));
          }
          if(image_channel_ == IMAGE_CHANNEL_RGB){
            pcl::for_each_type<FieldList> (pcl::SetIfFieldExists<typename CloudT::PointType, float> (output (j, i), "r", output_r));
            pcl::for_each_type<FieldList> (pcl::SetIfFieldExists<typename CloudT::PointType, float> (output (j, i), "g", output_g));
            pcl::for_each_type<FieldList> (pcl::SetIfFieldExists<typename CloudT::PointType, float> (output (j, i), "b", output_b));
          }
        }
      }
    }
    else
      if (boundary_options_ == BOUNDARY_OPTION_ZERO_PADDING)
      {
        for (int i = 0; i < rows; i++)
        {
          for (int j = 0; j < cols; j++)
          {
            //output (j, i).intensity = 0;
            if(image_channel_ == IMAGE_CHANNEL_INTENSITY){
              output_intensity = 0;
            }
            if(image_channel_ == IMAGE_CHANNEL_RGB){
              output_r = 0;
              output_g = 0;
              output_b = 0;
            }
            for (int k = 0; k < k_rows; k++)
            {
              for (int l = 0; l < k_cols; l++)
              {
                if ((i + k - k_rows / 2) < 0 || (i + k - k_rows / 2) >= rows || (j + l - k_cols / 2) < 0 || (j + l - k_cols / 2) >= cols)
                {
                  continue;
                }
                else
                {
                  //output (j, i).intensity += kernel_ (l, k).intensity * ((*input_)(j + l - k_cols / 2, i + k - k_rows / 2).intensity);
                  if(image_channel_ == IMAGE_CHANNEL_INTENSITY){
                    pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> ((*input_)(j + l - k_cols / 2, i + k - k_rows / 2), "intensity", has_intensity, input_intensity));
                    pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> (kernel_ (l, k), "intensity", has_intensity, kernel_intensity));
                    output_intensity += kernel_intensity*input_intensity;
                  }
                  if(image_channel_ == IMAGE_CHANNEL_RGB){
                    pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> ((*input_)(j + l - k_cols / 2, i + k - k_rows / 2), "r", has_intensity, input_r));
                    pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> (kernel_ (l, k), "r", has_intensity, kernel_r));
                    output_r += kernel_r*input_r;
                    pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> ((*input_)(j + l - k_cols / 2, i + k - k_rows / 2), "g", has_intensity, input_g));
                    pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> (kernel_ (l, k), "g", has_intensity, kernel_g));
                    output_g += kernel_r*input_g;
                    pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> ((*input_)(j + l - k_cols / 2, i + k - k_rows / 2), "b", has_intensity, input_b));
                    pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> (kernel_ (l, k), "b", has_intensity, kernel_b));
                    output_b += kernel_r*input_b;
                  }
                }
              }
            }
            if(image_channel_ == IMAGE_CHANNEL_INTENSITY){
              pcl::for_each_type<FieldList> (pcl::SetIfFieldExists<typename CloudT::PointType, float> (output (j, i), "intensity", output_intensity));
            }
            if(image_channel_ == IMAGE_CHANNEL_RGB){
              pcl::for_each_type<FieldList> (pcl::SetIfFieldExists<typename CloudT::PointType, float> (output (j, i), "r", output_r));
              pcl::for_each_type<FieldList> (pcl::SetIfFieldExists<typename CloudT::PointType, float> (output (j, i), "g", output_g));
              pcl::for_each_type<FieldList> (pcl::SetIfFieldExists<typename CloudT::PointType, float> (output (j, i), "b", output_b));
            }
          }
        }
      }
}

template<typename PointT>
void
pcl::pcl_2d::convolution<PointT>::applyFilter (pcl::PointCloud<PointT> &output)
{
  convolve (output);
}

template<typename PointT>
void
pcl::pcl_2d::convolution<PointT>::setBoundaryOptions (BOUNDARY_OPTIONS_ENUM boundary_options)
{
  boundary_options_ = boundary_options;
}

template<typename PointT>
void
pcl::pcl_2d::convolution<PointT>::setKernel (pcl::PointCloud<PointT> &kernel)
{
  kernel_ = kernel;
}

template<typename PointT>
void
pcl::pcl_2d::convolution<PointT>::setImageChannel(IMAGE_CHANNEL image_channel){
  image_channel_ = image_channel;
}
#endif
