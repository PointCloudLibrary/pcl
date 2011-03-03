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

#include "pcl/features/integral_image_normal.h"

#include "pcl/win32_macros.h"

#define SQR(a) ((a)*(a))

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::IntegralImageNormalEstimation::IntegralImageNormalEstimation ()
: integral_image_x_(NULL),
  integral_image_y_(NULL),
  diff_x_(NULL),
  diff_y_(NULL),
  depth_data_(NULL)
{
}

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::IntegralImageNormalEstimation::~IntegralImageNormalEstimation ()
{
  if (integral_image_x_ != NULL) delete integral_image_x_;
  if (integral_image_y_ != NULL) delete integral_image_y_;
  if (diff_x_ != NULL) delete diff_x_;
  if (diff_y_ != NULL) delete diff_y_;
  if (depth_data_ != NULL) delete depth_data_;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::IntegralImageNormalEstimation::setInputData (
  float * data,
  const int width, const int height,
  const int dimensions, const int element_stride,
  const int row_stride, const float distance_threshold)
{
  data_ = data;
  width_ = width;
  height_ = height;
  dimensions_ = dimensions;
  element_stride_ = element_stride;
  row_stride_ = row_stride;

  distance_threshold_ = distance_threshold;

  // compute derivatives
  if (diff_x_ != NULL) delete diff_x_;
  if (diff_y_ != NULL) delete diff_y_;
  if (depth_data_ != NULL) delete depth_data_;

  diff_x_ = new float[4*width_*height_];
  diff_y_ = new float[4*width_*height_];

  depth_data_ = new float[width_*height_];

  memset(diff_x_, 0, sizeof(float)*4*width_*height_);
  memset(diff_y_, 0, sizeof(float)*4*width_*height_);

  //const float sqr_distance_threshold = SQR(distance_threshold);

  for (int row_index = 0; row_index < height_; ++row_index)
  {
    float * data_pointer = data_ + row_index*row_stride_;

    float * diff_x_pointer = diff_x_ + row_index*4*width_;
    float * diff_y_pointer = diff_y_ + row_index*4*width_;

    for (int col_index = 0; col_index < width_; ++col_index)
    {
      depth_data_[row_index*width_ + col_index] = data_pointer[2];

      diff_x_pointer[3] = 1;
      diff_y_pointer[3] = 1;

      data_pointer += element_stride_;
      diff_x_pointer += 4;
      diff_y_pointer += 4;
    }
  }

  for (int row_index = 1; row_index < height_-1; ++row_index)
  {
    float * data_pointer_y_up = data_ + (row_index-1)*row_stride_ + element_stride_;
    float * data_pointer_y_down = data_ + (row_index+1)*row_stride_ + element_stride_;
    float * data_pointer_x_left = data_ + row_index*row_stride_;
    float * data_pointer_x_right = data_ + row_index*row_stride_ + 2*element_stride_;

    float * diff_x_pointer = diff_x_ + row_index*4*width_ + 4;
    float * diff_y_pointer = diff_y_ + row_index*4*width_ + 4;

    for (int col_index = 1; col_index < width_-1; ++col_index)
    {
      if (pcl_isfinite (data_pointer_x_right[0]) && pcl_isfinite (data_pointer_x_left[0]))
      {      
        diff_x_pointer[0] = data_pointer_x_right[0]-data_pointer_x_left[0];
        diff_x_pointer[1] = data_pointer_x_right[1]-data_pointer_x_left[1];
        diff_x_pointer[2] = data_pointer_x_right[2]-data_pointer_x_left[2];      

        //const float l_lengthX = SQR(diff_x_pointer[0]) + SQR(diff_x_pointer[1]) + SQR(diff_x_pointer[2]);
        const float l_lengthX = fabs(diff_x_pointer[2]);
        
        //::std::cerr << diff_x_pointer[0] << " ";

        //if (l_lengthX > sqr_distance_threshold)
        if (l_lengthX < distance_threshold/* && depth_data_[row_index*width_ + col_index] < 5.0f && depth_data_[row_index*width_ + col_index] > 0.0f*/)
        {
          diff_x_pointer[3] = 0;
        }
        //else
        //{
          //::std::cerr << "x: " << l_lengthX << ::std::endl;
          //++tmpCounter;
        //}
      }

      if (pcl_isfinite (data_pointer_y_down[0]) && pcl_isfinite (data_pointer_y_up[0]))
      {      
        diff_y_pointer[0] = data_pointer_y_down[0]-data_pointer_y_up[0];
        diff_y_pointer[1] = data_pointer_y_down[1]-data_pointer_y_up[1];
        diff_y_pointer[2] = data_pointer_y_down[2]-data_pointer_y_up[2];

        //const float l_lengthY = SQR(diff_y_pointer[0]) + SQR(diff_y_pointer[1]) + SQR(diff_y_pointer[2]);
        const float l_lengthY = fabs(diff_y_pointer[2]);

        //if (l_lengthY > sqr_distance_threshold)
        if (l_lengthY < distance_threshold/* && depth_data_[row_index*width_ + col_index] < 5.0f && depth_data_[row_index*width_ + col_index] > 0.0f*/)
        {
          diff_y_pointer[3] = 0;
        }
      }

      diff_x_pointer += 4;
      diff_y_pointer += 4;

      data_pointer_y_up += element_stride_;
      data_pointer_y_down += element_stride_;
      data_pointer_x_left += element_stride_;
      data_pointer_x_right += element_stride_;
    }
  }


  // compute integral images
  integral_image_x_ = new ::pcl::IntegralImage2D<float, float>(
    diff_x_,
    width_,
    height_,
    4,
    false,
    4,
    4*width_ );

  integral_image_y_ = new ::pcl::IntegralImage2D<float, float>(
    diff_y_,
    width_,
    height_,
    4,
    false,
    4,
    4*width_ );
}


//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::IntegralImageNormalEstimation::setRectSize (const int width, const int height)
{
  rect_width_ = width;
  rect_height_ = height;
}

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::Normal
pcl::IntegralImageNormalEstimation::compute (const int pos_x, const int pos_y)
{
  const float threshold_violation_count_x = integral_image_x_->getSum(pos_x-rect_width_/2, pos_y-rect_height_/2, rect_width_, rect_height_, 3);
  const float threshold_violation_count_y = integral_image_y_->getSum(pos_x-rect_width_/2, pos_y-rect_height_/2, rect_width_, rect_height_, 3);

  //::std::cerr << "threshold_violation_count_x: " << threshold_violation_count_x << ::std::endl;
  //::std::cerr << "threshold_violation_count_y: " << threshold_violation_count_y << ::std::endl;

  if (threshold_violation_count_x < 1.0f && threshold_violation_count_y < 1.0f)
  {
    const float mean_x_x = integral_image_x_->getSum(pos_x-rect_width_/2, pos_y-rect_height_/2, rect_width_, rect_height_, 0);
    const float mean_x_y = integral_image_x_->getSum(pos_x-rect_width_/2, pos_y-rect_height_/2, rect_width_, rect_height_, 1);
    const float mean_x_z = integral_image_x_->getSum(pos_x-rect_width_/2, pos_y-rect_height_/2, rect_width_, rect_height_, 2);

    const float mean_y_x = integral_image_y_->getSum(pos_x-rect_width_/2, pos_y-rect_height_/2, rect_width_, rect_height_, 0);
    const float mean_y_y = integral_image_y_->getSum(pos_x-rect_width_/2, pos_y-rect_height_/2, rect_width_, rect_height_, 1);
    const float mean_y_z = integral_image_y_->getSum(pos_x-rect_width_/2, pos_y-rect_height_/2, rect_width_, rect_height_, 2);

    const float normal_x = mean_x_y * mean_y_z - mean_x_z * mean_y_y;
    const float normal_y = mean_x_z * mean_y_x - mean_x_x * mean_y_z;
    const float normal_z = mean_x_x * mean_y_y - mean_x_y * mean_y_x;

    const float normal_length = sqrt(SQR(normal_x) + SQR(normal_y) + SQR(normal_z));
    
    if (normal_length == 0.0f)
    {
      pcl::Normal normal;
      normal.normal_x = 0.0f;
      normal.normal_y = 0.0f;
      normal.normal_z = 0.0f;
      normal.curvature = 0.0f;
      
      return normal;
    }
    
    const float scale = -1.0f/normal_length;
    
    pcl::Normal normal;
    normal.normal_x = normal_x*scale;
    normal.normal_y = normal_y*scale;
    normal.normal_z = normal_z*scale;
    normal.curvature = 0.0f;
    
    return normal;
  }
  else
  {
    //const float depth = depth_data_[pos_y*width_ + pos_x];

    //mean_x = ::cv::Point3f(0.0f, 0.0f, 0.0f);
    //mean_y = ::cv::Point3f(0.0f, 0.0f, 0.0f);
    //for (int row_index = -rect_height/2; row_index <= rect_height/2; ++row_index)
    //{
    //  for (int col_index = -rect_width/2; col_index <= rect_width/2; ++col_index)
    //  {
    //    const float cur_depth = depth_data_[(pos_y+row_index)*width_ + (pos_x+col_index)];
    //    const float invalid_x = diff_x_[4*((pos_y+row_index)*width_ + (pos_x+col_index)) + 3];
    //    const float invalid_y = diff_y_[4*((pos_y+row_index)*width_ + (pos_x+col_index)) + 3];

    //    if ( fabs(depth - cur_depth) < distance_threshold_
    //      && invalid_x != 1.0f
    //      && invalid_y != 1.0f )
    //    {
    //      mean_x.x += diff_x_[4*((pos_y+row_index)*width_ + (pos_x+col_index)) + 0];
    //      mean_x.y += diff_x_[4*((pos_y+row_index)*width_ + (pos_x+col_index)) + 1];
    //      mean_x.z += diff_x_[4*((pos_y+row_index)*width_ + (pos_x+col_index)) + 2];

    //      mean_y.x += diff_y_[4*((pos_y+row_index)*width_ + (pos_x+col_index)) + 0];
    //      mean_y.y += diff_y_[4*((pos_y+row_index)*width_ + (pos_x+col_index)) + 1];
    //      mean_y.z += diff_y_[4*((pos_y+row_index)*width_ + (pos_x+col_index)) + 2];
    //    }
    //  }
    //}

    //::cv::Point3f normal = mean_x.cross(mean_y);
    //const float normal_length = ::cv::sqrt(SQR(normal.x) + SQR(normal.y) + SQR(normal.z));
    //
    //if (normal_length == 0.0f)
    //{
    //  return ::cv::Point3f(0.0f, 0.0f, 0.0f);
    //}
    //
    //normal *= -1.0f/normal_length;

    //return normal;
  }
  
  pcl::Normal normal;
  normal.normal_x = 0.0f;
  normal.normal_y = 0.0f;
  normal.normal_z = 0.0f;
  normal.curvature = 0.0f;
  
  return normal;
}

