/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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

#ifndef PCL_FEATURES_INTEGRALIMAGE_BASED_IMPL_NORMAL_ESTIMATOR_H_
#define PCL_FEATURES_INTEGRALIMAGE_BASED_IMPL_NORMAL_ESTIMATOR_H_

#include "pcl/features/integral_image_normal.h"

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT>
pcl::IntegralImageNormalEstimation<PointInT, PointOutT>::~IntegralImageNormalEstimation ()
{
  if (diff_x_ != NULL) delete diff_x_;
  if (diff_y_ != NULL) delete diff_y_;
  if (depth_data_ != NULL) delete depth_data_;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void
pcl::IntegralImageNormalEstimation<PointInT, PointOutT>::initData ()
{
  // compute derivatives
  if (diff_x_ != NULL) delete diff_x_;
  if (diff_y_ != NULL) delete diff_y_;
  if (depth_data_ != NULL) delete depth_data_;

  if (normal_estimation_method_ == COVARIANCE_MATRIX)
    initCovarianceMatrixMethod ();
  else if (normal_estimation_method_ == AVERAGE_3D_GRADIENT)
    initAverage3DGradientMethod ();
  else if (normal_estimation_method_ == AVERAGE_DEPTH_CHANGE)
    initAverageDepthChangeMethod ();
}


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void
pcl::IntegralImageNormalEstimation<PointInT, PointOutT>::setRectSize (const int width, const int height)
{
  rect_width_ = width;
  rect_height_ = height;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void
pcl::IntegralImageNormalEstimation<PointInT, PointOutT>::initCovarianceMatrixMethod ()
{
  // number of DataType entries per element (equal or bigger than dimensions)
  int element_stride = sizeof (input_->points[0]) / sizeof (float);
  // number of DataType entries per row (equal or bigger than element_stride number of elements per row)
  int row_stride     = element_stride * input_->width;

  float *data_ = reinterpret_cast<float*>((PointInT*)(&(input_->points[0])));

  integral_image_XYZ_.setInput (data_, input_->width, input_->height, element_stride, row_stride);

  init_covariance_matrix_ = true;
  init_average_3d_gradient_ = init_depth_change_ = false;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void
pcl::IntegralImageNormalEstimation<PointInT, PointOutT>::initAverage3DGradientMethod ()
{
  float *data_ = reinterpret_cast<float*>((PointInT*)(&(input_->points[0])));
  size_t nr_points = 4 * input_->points.size ();
  diff_x_ = new float[nr_points];
  diff_y_ = new float[nr_points];

  // number of DataType entries per element (equal or bigger than dimensions)
  int element_stride = sizeof (input_->points[0]) / sizeof (float);
  // number of DataType entries per row (equal or bigger than element_stride number of elements per row)
  int row_stride     = element_stride * input_->width;

  memset (diff_x_, 0, sizeof(float) * nr_points);
  memset (diff_y_, 0, sizeof(float) * nr_points);

  for (size_t ri = 1; ri < input_->height - 1; ++ri)
  {
    float *data_pointer_y_up    = data_ + (ri-1)*row_stride + element_stride;
    float *data_pointer_y_down  = data_ + (ri+1)*row_stride + element_stride;
    float *data_pointer_x_left  = data_ + ri*row_stride;
    float *data_pointer_x_right = data_ + ri*row_stride + 2*element_stride;

    float * diff_x_pointer = diff_x_ + ri * 4 * input_->width + 4;
    float * diff_y_pointer = diff_y_ + ri * 4 * input_->width + 4;

    for (size_t ci = 1; ci < input_->width - 1; ++ci)
    {
      diff_x_pointer[0] = data_pointer_x_right[0] - data_pointer_x_left[0];
      diff_x_pointer[1] = data_pointer_x_right[1] - data_pointer_x_left[1];
      diff_x_pointer[2] = data_pointer_x_right[2] - data_pointer_x_left[2];

      diff_y_pointer[0] = data_pointer_y_down[0] - data_pointer_y_up[0];
      diff_y_pointer[1] = data_pointer_y_down[1] - data_pointer_y_up[1];
      diff_y_pointer[2] = data_pointer_y_down[2] - data_pointer_y_up[2];

      diff_x_pointer += 4;
      diff_y_pointer += 4;

      data_pointer_y_up    += element_stride;
      data_pointer_y_down  += element_stride;
      data_pointer_x_left  += element_stride;
      data_pointer_x_right += element_stride;
    }
  }

  // Compute integral images
  integral_image_X_.setInput (diff_x_, input_->width, input_->height, 4, 4 * input_->width);
  integral_image_Y_.setInput (diff_y_, input_->width, input_->height, 4, 4 * input_->width);
  init_covariance_matrix_ = init_depth_change_ = false;
  init_average_3d_gradient_ = true;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void
pcl::IntegralImageNormalEstimation<PointInT, PointOutT>::initAverageDepthChangeMethod ()
{
  // number of DataType entries per element (equal or bigger than dimensions)
  int element_stride = sizeof (input_->points[0]) / sizeof (float);
  // number of DataType entries per row (equal or bigger than element_stride number of elements per row)
  int row_stride     = element_stride * input_->width;

  float *data_ = reinterpret_cast<float*>((PointInT*)(&(input_->points[0])));
  // compute integral image
  //integral_image_ = new pcl::IntegralImage2D<float, double>(
  //  &(data_[2]), input_->width, input_->height, 1, false, element_stride, row_stride);

  integral_image_depth_.setInput (&(data_[2]), input_->width, input_->height, element_stride, row_stride);
  init_depth_change_ = true;
  init_covariance_matrix_ = init_average_3d_gradient_ = false;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void
pcl::IntegralImageNormalEstimation<PointInT, PointOutT>::computePointNormal (
    const int pos_x, const int pos_y, PointOutT &normal)
{
  float bad_point = std::numeric_limits<float>::quiet_NaN ();

  if (normal_estimation_method_ == COVARIANCE_MATRIX)
  {
    if (!init_covariance_matrix_)
      initCovarianceMatrixMethod ();

    EIGEN_ALIGN16 Eigen::Matrix3f covariance_matrix;
    Eigen::Vector3f center;
    typename IntegralImage2D<float, 3>::SecondOrderType so_elements;

    center = integral_image_XYZ_.getFirstOrderSum(pos_x - (rect_width_ >> 1), pos_y - (rect_height_ >> 1), rect_width_, rect_height_).cast<float> ();
    so_elements = integral_image_XYZ_.getSecondOrderSum(pos_x - (rect_width_ >> 1), pos_y - (rect_height_ >> 1), rect_width_, rect_height_);
    covariance_matrix.coeffRef (0) = so_elements [0];
    covariance_matrix.coeffRef (1) = covariance_matrix.coeffRef (3) = so_elements [1];
    covariance_matrix.coeffRef (2) = covariance_matrix.coeffRef (6) = so_elements [2];
    covariance_matrix.coeffRef (4) = so_elements [3];
    covariance_matrix.coeffRef (5) = covariance_matrix.coeffRef (7) = so_elements [4];
    covariance_matrix.coeffRef (8) = so_elements [5];
    covariance_matrix -= (center * center.transpose ()) / (rect_width_ * rect_height_);
    float eigen_value = -1;
    Eigen::Vector3f eigen_vector;
    pcl::eigen33 (covariance_matrix, eigen_value, eigen_vector);
    if (eigen_vector [2] < 0.0f)
      normal.getNormalVector4fMap () = Eigen::Vector4f (eigen_vector [0], eigen_vector [1], eigen_vector [2], 0);
    else
      normal.getNormalVector4fMap () = Eigen::Vector4f (-eigen_vector [0], -eigen_vector [1], -eigen_vector [2], 0);

    // Compute the curvature surface change
    if (eigen_value > 0.0)
      normal.curvature = fabs ( eigen_value / (covariance_matrix.coeff (0) + covariance_matrix.coeff (4) + covariance_matrix.coeff (8)) );
    else
      normal.curvature = 0;

    return;
  }
  else if (normal_estimation_method_ == AVERAGE_3D_GRADIENT)
  {
    if (!init_average_3d_gradient_)
      initAverage3DGradientMethod ();
    const float mean_x_x = integral_image_X_.getFirstOrderSum (pos_x-rect_width_/2, pos_y-rect_height_/2, rect_width_, rect_height_) [0];
    const float mean_x_y = integral_image_X_.getFirstOrderSum (pos_x-rect_width_/2, pos_y-rect_height_/2, rect_width_, rect_height_) [1];
    const float mean_x_z = integral_image_X_.getFirstOrderSum (pos_x-rect_width_/2, pos_y-rect_height_/2, rect_width_, rect_height_) [2];

    const float mean_y_x = integral_image_Y_.getFirstOrderSum (pos_x-rect_width_/2, pos_y-rect_height_/2, rect_width_, rect_height_) [0];
    const float mean_y_y = integral_image_Y_.getFirstOrderSum (pos_x-rect_width_/2, pos_y-rect_height_/2, rect_width_, rect_height_) [1];
    const float mean_y_z = integral_image_Y_.getFirstOrderSum (pos_x-rect_width_/2, pos_y-rect_height_/2, rect_width_, rect_height_) [2];

    const float normal_x = mean_x_y * mean_y_z - mean_x_z * mean_y_y;
    const float normal_y = mean_x_z * mean_y_x - mean_x_x * mean_y_z;
    const float normal_z = mean_x_x * mean_y_y - mean_x_y * mean_y_x;


    const float normal_length = sqrt (normal_x * normal_x + normal_y * normal_y + normal_z * normal_z);

    if (normal_length == 0.0f)
    {
      normal.getNormalVector4fMap ().setConstant (bad_point);
      normal.curvature = bad_point;
      return;
    }

    const float scale = -1.0f / normal_length;

    normal.normal_x = normal_x * scale;
    normal.normal_y = normal_y * scale;
    normal.normal_z = normal_z * scale;
    normal.curvature = bad_point;
    return;
  }
  else if (normal_estimation_method_ == AVERAGE_DEPTH_CHANGE)
  {
    if (!init_depth_change_)
      initAverageDepthChangeMethod ();

    PointInT pointL = input_->points[pos_y * input_->width + pos_x-rect_width_/2];
    PointInT pointR = input_->points[pos_y * input_->width + pos_x+rect_width_/2];
    PointInT pointU = input_->points[(pos_y-rect_height_/2) * input_->width+pos_x];
    PointInT pointD = input_->points[(pos_y+rect_height_/2) * input_->width+pos_x];

    const float mean_L_z = integral_image_depth_.getFirstOrderSum (pos_x-1-rect_width_/2, pos_y-rect_height_/2, rect_width_-1, rect_height_-1)/((rect_width_-1)*(rect_height_-1));
    const float mean_R_z = integral_image_depth_.getFirstOrderSum (pos_x+1-rect_width_/2, pos_y-rect_height_/2, rect_width_-1, rect_height_-1)/((rect_width_-1)*(rect_height_-1));
    const float mean_U_z = integral_image_depth_.getFirstOrderSum (pos_x-rect_width_/2, pos_y-1-rect_height_/2, rect_width_-1, rect_height_-1)/((rect_width_-1)*(rect_height_-1));
    const float mean_D_z = integral_image_depth_.getFirstOrderSum (pos_x-rect_width_/2, pos_y+1-rect_height_/2, rect_width_-1, rect_height_-1)/((rect_width_-1)*(rect_height_-1));

    const float mean_x_z = (mean_R_z - mean_L_z)/2.0f;
    const float mean_y_z = (mean_D_z - mean_U_z)/2.0f;

    const float mean_x_x = (pointR.x - pointL.x)/(rect_width_);
    const float mean_x_y = (pointR.y - pointL.y)/(rect_height_);
    const float mean_y_x = (pointD.x - pointU.x)/(rect_width_);
    const float mean_y_y = (pointD.y - pointU.y)/(rect_height_);

    const float normal_x = mean_x_y * mean_y_z - mean_x_z * mean_y_y;
    const float normal_y = mean_x_z * mean_y_x - mean_x_x * mean_y_z;
    const float normal_z = mean_x_x * mean_y_y - mean_x_y * mean_y_x;

    const float normal_length = sqrt (normal_x * normal_x + normal_y * normal_y + normal_z * normal_z);

    if (normal_length == 0.0f)
    {
      normal.getNormalVector4fMap ().setConstant (bad_point);
      normal.curvature = bad_point;
      return;
    }

    const float scale = -1.0f / normal_length;

    normal.normal_x = normal_x*scale;
    normal.normal_y = normal_y*scale;
    normal.normal_z = normal_z*scale;
    normal.curvature = bad_point;

    return;
  }
  normal.getNormalVector4fMap ().setConstant (bad_point);
  normal.curvature = bad_point;
  return;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void
pcl::IntegralImageNormalEstimation<PointInT, PointOutT>::computeFeature (PointCloudOut &output)
{
  float bad_point = std::numeric_limits<float>::quiet_NaN ();

  // compute depth-change map
  unsigned char * depthChangeMap = new unsigned char[input_->points.size ()];
  memset (depthChangeMap, 255, input_->points.size ());

  for (unsigned int ri = 0; ri < input_->height-1; ++ri)
  {
    for (unsigned int ci = 0; ci < input_->width-1; ++ci)
    {
      const float depth  = (*input_)(ci,     ri    ).z;
      const float depthR = (*input_)(ci + 1, ri    ).z;
      const float depthD = (*input_)(ci,     ri + 1).z;

      const float depthDependendDepthChange = (max_depth_change_factor_ * (fabs(depth)+1.0f))/(500.0f*0.001f);

      if (abs (depth - depthR) > depthDependendDepthChange
        || !pcl_isfinite (depth) || !pcl_isfinite (depthR))
      {
        depthChangeMap[ri*input_->width + ci] = 0;
        depthChangeMap[ri*input_->width + ci+1] = 0;
      }
      if (abs (depth - depthD) > depthDependendDepthChange
        || !pcl_isfinite (depth) || !pcl_isfinite (depthD))
      {
        depthChangeMap[ri*input_->width + ci] = 0;
        depthChangeMap[(ri+1)*input_->width + ci] = 0;
      }
    }
  }


  // compute distance map
  float *distanceMap = new float[input_->points.size ()];
  for (size_t index = 0; index < input_->points.size (); ++index)
  {
    if (depthChangeMap[index] == 0)
      distanceMap[index] = 0.0f;
    else
      distanceMap[index] = 640.0f;
  }

  // first pass
  for (size_t ri = 1; ri < input_->height; ++ri)
  {
    for (size_t ci = 1; ci < input_->width; ++ci)
    {
      const float upLeft = distanceMap[(ri-1)*input_->width + ci-1] + 1.4f;
      const float up = distanceMap[(ri-1)*input_->width + ci] + 1.0f;
      const float upRight = distanceMap[(ri-1)*input_->width + ci+1] + 1.4f;
      const float left = distanceMap[ri*input_->width + ci-1] + 1.0f;
      const float center = distanceMap[ri*input_->width + ci];

      const float minValue = std::min (std::min (upLeft, up), std::min (left, upRight));

      if (minValue < center)
        distanceMap[ri * input_->width + ci] = minValue;
    }
  }

  // second pass
  for (int ri = input_->height-2; ri >= 0; --ri)
  {
    for (int ci = input_->width-2; ci >= 0; --ci)
    {
      const float lowerLeft = distanceMap[(ri+1)*input_->width + ci-1] + 1.4f;
      const float lower = distanceMap[(ri+1)*input_->width + ci] + 1.0f;
      const float lowerRight = distanceMap[(ri+1)*input_->width + ci+1] + 1.4f;
      const float right = distanceMap[ri*input_->width + ci+1] + 1.0f;
      const float center = distanceMap[ri*input_->width + ci];

      const float minValue = std::min (std::min (lowerLeft, lower), std::min (right, lowerRight));

      if (minValue < center)
        distanceMap[ri*input_->width + ci] = minValue;
    }
  }

  // Set all normals that we do not touch to NaN
  for (size_t ri = 0; ri < normal_smoothing_size_; ++ri)
  {
    for (size_t ci = 0; ci < input_->width; ++ci)
    {
      output (ci, ri).getNormalVector4fMap ().setConstant (bad_point);
      output (ci, ri).curvature = bad_point;
    }
  }
  for (size_t ri = normal_smoothing_size_; ri < input_->height; ++ri)
  {
    for (size_t ci = 0; ci < normal_smoothing_size_; ++ci)
    {
      output (ci, ri).getNormalVector4fMap ().setConstant (bad_point);
      output (ci, ri).curvature = bad_point;
    }
  }

  for (size_t ri = input_->height - normal_smoothing_size_; ri < input_->height; ++ri)
  {
    for (size_t ci = normal_smoothing_size_; ci < input_->width; ++ci)
    {
      output (ci, ri).getNormalVector4fMap ().setConstant (bad_point);
      output (ci, ri).curvature = bad_point;
    }
  }

  for (size_t ri = normal_smoothing_size_; ri < input_->height - normal_smoothing_size_; ++ri)
  {
    for (size_t ci = input_->width - normal_smoothing_size_; ci < input_->width; ++ci)
    {
      output (ci, ri).getNormalVector4fMap ().setConstant (bad_point);
      output (ci, ri).curvature = bad_point;
    }
  }

  if (use_depth_dependent_smoothing_)
  {
    for (int ri = normal_smoothing_size_; ri < input_->height-normal_smoothing_size_; ++ri)
    {
      for (int ci = normal_smoothing_size_; ci < input_->width-normal_smoothing_size_; ++ci)
      {
        const float depth = (*input_)(ci, ri).z;
        if (!pcl_isfinite (depth))
        {
          output (ci, ri).getNormalVector4fMap ().setConstant (bad_point);
          output (ci, ri).curvature = bad_point;
          continue;
        }

        float smoothing = (std::min)(distanceMap[ri*input_->width + ci], normal_smoothing_size_ + static_cast<float>(depth)/10.0f);

        if (smoothing > 2.0f)
        {
          setRectSize (smoothing, smoothing);
          computePointNormal (ci, ri, output (ci, ri));
        }
        else
        {
          output (ci, ri).getNormalVector4fMap ().setConstant (bad_point);
          output (ci, ri).curvature = bad_point;
        }
      }
    }
  }
  else
  {
    float smoothing_constant = normal_smoothing_size_ * static_cast<float>(1.0f) / (500.0f * 0.001f);
    for (int ri = normal_smoothing_size_; ri < input_->height-normal_smoothing_size_; ++ri)
    {
      for (int ci = normal_smoothing_size_; ci < input_->width-normal_smoothing_size_; ++ci)
      {
        if (!pcl_isfinite ((*input_) (ci, ri).z))
        {
          output (ci, ri).getNormalVector4fMap ().setConstant (bad_point);
          output (ci, ri).curvature = bad_point;
          continue;
        }

        float smoothing = (std::min)(distanceMap[ri*input_->width + ci], smoothing_constant);

        if (smoothing > 2.0f)
        {
          setRectSize (smoothing, smoothing);
          computePointNormal (ci, ri, output (ci, ri));
        }
        else
        {
          output (ci, ri).getNormalVector4fMap ().setConstant (bad_point);
          output (ci, ri).curvature = bad_point;
        }
      }
    }
  }

  delete[] depthChangeMap;
  delete[] distanceMap;
}

#define PCL_INSTANTIATE_IntegralImageNormalEstimation(T,NT) template class PCL_EXPORTS pcl::IntegralImageNormalEstimation<T,NT>;

#endif

