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
  if (integral_image_x_ != NULL) delete integral_image_x_;
  if (integral_image_y_ != NULL) delete integral_image_y_;
  if (integral_image_xyz_ != NULL) delete integral_image_xyz_;
  if (integral_image_ != NULL) delete integral_image_;
  if (diff_x_ != NULL) delete diff_x_;
  if (diff_y_ != NULL) delete diff_y_;
  if (depth_data_ != NULL) delete depth_data_;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void
pcl::IntegralImageNormalEstimation<PointInT, PointOutT>::setInputData (
  float * data,
  const int width, const int height,
  const int dimensions, const int element_stride,
  const int row_stride, const float distance_threshold,
  const NormalEstimationMethod normal_estimation_method )
{
  data_ = data;
  width_ = width;
  height_ = height;
  dimensions_ = dimensions;
  element_stride_ = element_stride;
  row_stride_ = row_stride;


  distance_threshold_ = distance_threshold;

  normal_estimation_method_ = normal_estimation_method;

  // compute derivatives
  if (integral_image_x_ != NULL) delete integral_image_x_;
  if (integral_image_y_ != NULL) delete integral_image_y_;
  if (integral_image_xyz_ != NULL) delete integral_image_xyz_;
  if (integral_image_ != NULL) delete integral_image_;
  if (diff_x_ != NULL) delete diff_x_;
  if (diff_y_ != NULL) delete diff_y_;
  if (depth_data_ != NULL) delete depth_data_;

  if (normal_estimation_method == COVARIANCE_MATRIX)
  {
    // compute integral images
    integral_image_xyz_ = new ::pcl::IntegralImage2D<float, double>(
      data,
      width_,
      height_,
      3,
      true,
      element_stride,
      row_stride );
  }
  else if (normal_estimation_method == AVERAGE_3D_GRADIENT)
  {
    diff_x_ = new float[4*width_*height_];
    diff_y_ = new float[4*width_*height_];

    memset(diff_x_, 0, sizeof(float)*4*width_*height_);
    memset(diff_y_, 0, sizeof(float)*4*width_*height_);

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
        {      
          diff_x_pointer[0] = data_pointer_x_right[0]-data_pointer_x_left[0];
          diff_x_pointer[1] = data_pointer_x_right[1]-data_pointer_x_left[1];
          diff_x_pointer[2] = data_pointer_x_right[2]-data_pointer_x_left[2];    
        }

        {      
          diff_y_pointer[0] = data_pointer_y_down[0]-data_pointer_y_up[0];
          diff_y_pointer[1] = data_pointer_y_down[1]-data_pointer_y_up[1];
          diff_y_pointer[2] = data_pointer_y_down[2]-data_pointer_y_up[2];
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
    integral_image_x_ = new ::pcl::IntegralImage2D<float, double>(
      diff_x_,
      width_,
      height_,
      3,
      false,
      4,
      4*width_ );

    integral_image_y_ = new ::pcl::IntegralImage2D<float, double>(
      diff_y_,
      width_,
      height_,
      3,
      false,
      4,
      4*width_ );
  }
  else if (normal_estimation_method == AVERAGE_DEPTH_CHANGE)
  {
    // compute integral image
    integral_image_ = new ::pcl::IntegralImage2D<float, double>(
      &(data_[2]),
      width_,
      height_,
      1,
      false,
      element_stride,
      row_stride );
  }
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
pcl::IntegralImageNormalEstimation<PointInT, PointOutT>::computePointNormal (
    const int pos_x, const int pos_y, PointOutT &normal)
{
  if (normal_estimation_method_ == COVARIANCE_MATRIX)
  {
    const float mean_x = integral_image_xyz_->getSum(pos_x-rect_width_/2, pos_y-rect_height_/2, rect_width_, rect_height_, 0);
    const float mean_y = integral_image_xyz_->getSum(pos_x-rect_width_/2, pos_y-rect_height_/2, rect_width_, rect_height_, 1);
    const float mean_z = integral_image_xyz_->getSum(pos_x-rect_width_/2, pos_y-rect_height_/2, rect_width_, rect_height_, 2);

    const float mean_xx = integral_image_xyz_->getSum(pos_x-rect_width_/2, pos_y-rect_height_/2, rect_width_, rect_height_, 0, 0);
    const float mean_xy = integral_image_xyz_->getSum(pos_x-rect_width_/2, pos_y-rect_height_/2, rect_width_, rect_height_, 0, 1);
    const float mean_xz = integral_image_xyz_->getSum(pos_x-rect_width_/2, pos_y-rect_height_/2, rect_width_, rect_height_, 0, 2);
    const float mean_yx = mean_xy;
    const float mean_yy = integral_image_xyz_->getSum(pos_x-rect_width_/2, pos_y-rect_height_/2, rect_width_, rect_height_, 1, 1);
    const float mean_yz = integral_image_xyz_->getSum(pos_x-rect_width_/2, pos_y-rect_height_/2, rect_width_, rect_height_, 1, 2);
    const float mean_zx = mean_xz;
    const float mean_zy = mean_yz;
    const float mean_zz = integral_image_xyz_->getSum(pos_x-rect_width_/2, pos_y-rect_height_/2, rect_width_, rect_height_, 2, 2);


    EIGEN_ALIGN16 Eigen::Matrix3f covariance_matrix;
    covariance_matrix (0, 0) = mean_xx; covariance_matrix (0, 1) = mean_xy; covariance_matrix (0, 2) = mean_xz;
    covariance_matrix (1, 0) = mean_yx; covariance_matrix (1, 1) = mean_yy; covariance_matrix (1, 2) = mean_yz;
    covariance_matrix (2, 0) = mean_zx; covariance_matrix (2, 1) = mean_zy; covariance_matrix (2, 2) = mean_zz;

    Eigen::Vector3f center (mean_x, mean_y, mean_z);

    covariance_matrix -= (1.0f/(rect_width_*rect_height_)) * (center * center.transpose());
    covariance_matrix *= 1.0f/(rect_width_*rect_height_-1);
    
    EIGEN_ALIGN16 Eigen::Vector3f eigen_values;
    EIGEN_ALIGN16 Eigen::Matrix3f eigen_vectors;
    pcl::eigen33 (covariance_matrix, eigen_vectors, eigen_values);
  
    float normal_x = eigen_vectors(0, 0);
    float normal_y = eigen_vectors(1, 0);
    float normal_z = eigen_vectors(2, 0);

    if (normal_z > 0.0f)
    {
      normal_x *= -1.0f;
      normal_y *= -1.0f;
      normal_z *= -1.0f;
    }

    const float normal_length = sqrt(normal_x * normal_x + normal_y * normal_y + normal_z * normal_z);
    const float scale = 1.0f/normal_length;
    
    normal.normal_x = normal_x*scale;
    normal.normal_y = normal_y*scale;
    normal.normal_z = normal_z*scale;
    normal.curvature = 0.0f;
    return;
  }
  else if (normal_estimation_method_ == AVERAGE_3D_GRADIENT)
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

    const float normal_length = sqrt(normal_x * normal_x + normal_y * normal_y + normal_z * normal_z);
    
    if (normal_length == 0.0f)
    {
      normal.normal_x = 0.0f;
      normal.normal_y = 0.0f;
      normal.normal_z = 0.0f;
      normal.curvature = 0.0f;
      return;
    }
    
    const float scale = -1.0f/normal_length;
    
    normal.normal_x = normal_x*scale;
    normal.normal_y = normal_y*scale;
    normal.normal_z = normal_z*scale;
    normal.curvature = 0.0f;
    return;
  }
  else if (normal_estimation_method_ == AVERAGE_DEPTH_CHANGE)
  {
    pcl::PointXYZ * points = reinterpret_cast<pcl::PointXYZ*>(data_);

    pcl::PointXYZ pointL = points[pos_y*width_+pos_x-rect_width_/2];
    pcl::PointXYZ pointR = points[pos_y*width_+pos_x+rect_width_/2];
    pcl::PointXYZ pointU = points[(pos_y-rect_height_/2)*width_+pos_x];
    pcl::PointXYZ pointD = points[(pos_y+rect_height_/2)*width_+pos_x];

    const float mean_L_z = integral_image_->getSum(pos_x-1-rect_width_/2, pos_y-rect_height_/2, rect_width_-1, rect_height_-1, 0)/((rect_width_-1)*(rect_height_-1));
    const float mean_R_z = integral_image_->getSum(pos_x+1-rect_width_/2, pos_y-rect_height_/2, rect_width_-1, rect_height_-1, 0)/((rect_width_-1)*(rect_height_-1));
    const float mean_U_z = integral_image_->getSum(pos_x-rect_width_/2, pos_y-1-rect_height_/2, rect_width_-1, rect_height_-1, 0)/((rect_width_-1)*(rect_height_-1));
    const float mean_D_z = integral_image_->getSum(pos_x-rect_width_/2, pos_y+1-rect_height_/2, rect_width_-1, rect_height_-1, 0)/((rect_width_-1)*(rect_height_-1));

    const float mean_x_z = (mean_R_z - mean_L_z)/2.0f;
    const float mean_y_z = (mean_D_z - mean_U_z)/2.0f;

    const float mean_x_x = (pointR.x - pointL.x)/(rect_width_);
    const float mean_x_y = (pointR.y - pointL.y)/(rect_height_);
    const float mean_y_x = (pointD.x - pointU.x)/(rect_width_);
    const float mean_y_y = (pointD.y - pointU.y)/(rect_height_);

    const float normal_x = mean_x_y * mean_y_z - mean_x_z * mean_y_y;
    const float normal_y = mean_x_z * mean_y_x - mean_x_x * mean_y_z;
    const float normal_z = mean_x_x * mean_y_y - mean_x_y * mean_y_x;

    const float normal_length = sqrt(normal_x * normal_x + normal_y * normal_y + normal_z * normal_z);
    
    if (normal_length == 0.0f)
    {
      normal.normal_x = 0.0f;
      normal.normal_y = 0.0f;
      normal.normal_z = 0.0f;
      normal.curvature = 0.0f;
      return;
    }
    
    const float scale = -1.0f/normal_length;
    
    normal.normal_x = normal_x*scale;
    normal.normal_y = normal_y*scale;
    normal.normal_z = normal_z*scale;
    normal.curvature = 0.0f;
    
    return;
  }
  normal.normal_x = 0.0f;
  normal.normal_y = 0.0f;
  normal.normal_z = 0.0f;
  normal.curvature = 0.0f;
  return;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void
pcl::IntegralImageNormalEstimation<PointInT, PointOutT>::computeFeature (
    PointCloudIn &cloud,
    PointCloudOut &normals,
    const float maxDepthChangeFactor,
    const float normalSmoothingSize,
    const NormalEstimationMethod normal_estimation_method)
{
  // compute depth-change map
  unsigned char * depthChangeMap = new unsigned char[cloud.height*cloud.width];
  memset(depthChangeMap, 255, cloud.height*cloud.width);

  for (unsigned int row_index = 0; row_index < cloud.height-1; ++row_index)
  {
    for (unsigned int col_index = 0; col_index < cloud.width-1; ++col_index)
    {
      const float depth = cloud.points[row_index*cloud.width + col_index].z;
      const float depthR = cloud.points[row_index*cloud.width + col_index+1].z;
      const float depthD = cloud.points[(row_index+1)*cloud.width + col_index].z;

      const float depthDependendDepthChange = (maxDepthChangeFactor * depth)/(500.0f*0.001f);

      if (abs(depth-depthR) > depthDependendDepthChange)
      {
        depthChangeMap[row_index*cloud.width + col_index] = 0;
        depthChangeMap[row_index*cloud.width + col_index+1] = 0;
      }
      if (abs(depth-depthD) > depthDependendDepthChange)
      {
        depthChangeMap[row_index*cloud.width + col_index] = 0;
        depthChangeMap[(row_index+1)*cloud.width + col_index] = 0;
      }
    }
  }

    
  // compute distance map
  float * distanceMap = new float[cloud.width*cloud.height];
  for (unsigned int index = 0; index < (cloud.width*cloud.height); ++index)
  {
    if (depthChangeMap[index] == 0)
    {
      distanceMap[index] = 0.0f;
    }
    else
    {
      distanceMap[index] = 640.0f;
    }
  }

  // first pass
  for (unsigned int row_index = 1; row_index < cloud.height; ++row_index)
  {
    for (unsigned int col_index = 1; col_index < cloud.width; ++col_index)
    {
      const float upLeft = distanceMap[(row_index-1)*cloud.width + col_index-1] + 1.4f;
      const float up = distanceMap[(row_index-1)*cloud.width + col_index] + 1.0f;
      const float upRight = distanceMap[(row_index-1)*cloud.width + col_index+1] + 1.4f;
      const float left = distanceMap[row_index*cloud.width + col_index-1] + 1.0f;
      const float center = distanceMap[row_index*cloud.width + col_index];

      const float minValue = ::std::min(
        ::std::min(upLeft, up),
        ::std::min(left, upRight) );

      if (minValue < center)
      {
        distanceMap[row_index*cloud.width + col_index] = minValue;
      }
    }
  }

  // second pass
  for (int row_index = cloud.height-2; row_index >= 0; --row_index)
  {
    for (int col_index = cloud.width-2; col_index >= 0; --col_index)
    {
      const float lowerLeft = distanceMap[(row_index+1)*cloud.width + col_index-1] + 1.4f;
      const float lower = distanceMap[(row_index+1)*cloud.width + col_index] + 1.0f;
      const float lowerRight = distanceMap[(row_index+1)*cloud.width + col_index+1] + 1.4f;
      const float right = distanceMap[row_index*cloud.width + col_index+1] + 1.0f;
      const float center = distanceMap[row_index*cloud.width + col_index];

      const float minValue = ::std::min(
        ::std::min(lowerLeft, lower),
        ::std::min(right, lowerRight) );

      if (minValue < center)
      {
        distanceMap[row_index*cloud.width + col_index] = minValue;
      }
    }
  }

  // setup normal estimation
//  pcl::IntegralImageNormalEstimation normalEstimator;
  //normalEstimator.
  setInputData(
    reinterpret_cast<float*>(&(cloud.points[0])),
    cloud.width, cloud.height,
    3, sizeof(cloud.points[0])/sizeof(float), (sizeof(cloud.points[0])/sizeof(float))*cloud.width, 10.0f,
    normal_estimation_method );
  

  // estimate normals
  normals.width = cloud.width;
  normals.height = cloud.height;
  normals.points.resize(cloud.width*cloud.height);

  PointOutT zero_normal;
  zero_normal.normal_x = 0;
  zero_normal.normal_y = 0;
  zero_normal.normal_z = 0;

  for (int row_index = normalSmoothingSize; row_index < cloud.height-normalSmoothingSize; ++row_index)
  {
    for (int col_index = normalSmoothingSize; col_index < cloud.width-normalSmoothingSize; ++col_index)
    {
      const float depth = cloud.points[row_index*cloud.width + col_index].z;

      if (depth != 0)
      {
        float smoothing = normalSmoothingSize*static_cast<float>(depth)/(500.0f*0.001f);
        smoothing = ::std::min (distanceMap[row_index*cloud.width + col_index], smoothing);

        if (smoothing > 2.0f)
        {
          //normalEstimator.setRectSize(smoothing, smoothing);
          setRectSize(smoothing, smoothing);

          //::pcl::Normal normal = normalEstimator.compute(col_index, row_index);
          PointOutT normal;
          computePointNormal (col_index, row_index, normal);

          normals.points[row_index*normals.width + col_index] = normal;
        }
        else
        {
          normals.points[row_index*normals.width + col_index] = zero_normal;
        }
      }
      else
      {
        normals.points[row_index*normals.width + col_index] = zero_normal;
      }
    }
  }

  delete[] depthChangeMap;
  delete[] distanceMap;
}


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void
pcl::IntegralImageNormalEstimation<PointInT, PointOutT>::computeFeature (
    PointCloudIn &cloud,
    PointCloudOut &normals,
    const bool useDepthDependentSmoothing,
    const float maxDepthChangeFactor,
    const float normalSmoothingSize,
    const NormalEstimationMethod normal_estimation_method)
{
  // compute depth-change map
  unsigned char * depthChangeMap = new unsigned char[cloud.height*cloud.width];
  memset (depthChangeMap, 255, cloud.height*cloud.width);

  for (unsigned int row_index = 0; row_index < cloud.height-1; ++row_index)
  {
    for (unsigned int col_index = 0; col_index < cloud.width-1; ++col_index)
    {
      const float depth = cloud.points[row_index*cloud.width + col_index].z;
      const float depthR = cloud.points[row_index*cloud.width + col_index+1].z;
      const float depthD = cloud.points[(row_index+1)*cloud.width + col_index].z;

      const float depthDependendDepthChange = (maxDepthChangeFactor * depth)/(500.0f*0.001f);

      if (abs(depth-depthR) > depthDependendDepthChange)
      {
        depthChangeMap[row_index*cloud.width + col_index] = 0;
        depthChangeMap[row_index*cloud.width + col_index+1] = 0;
      }
      if (abs(depth-depthD) > depthDependendDepthChange)
      {
        depthChangeMap[row_index*cloud.width + col_index] = 0;
        depthChangeMap[(row_index+1)*cloud.width + col_index] = 0;
      }
    }
  }

    
  // compute distance map
  float * distanceMap = new float[cloud.width*cloud.height];
  for (unsigned int index = 0; index < (cloud.width*cloud.height); ++index)
  {
    if (depthChangeMap[index] == 0)
      distanceMap[index] = 0.0f;
    else
      distanceMap[index] = 640.0f;
  }

  // first pass
  for (unsigned int row_index = 1; row_index < cloud.height; ++row_index)
  {
    for (unsigned int col_index = 1; col_index < cloud.width; ++col_index)
    {
      const float upLeft = distanceMap[(row_index-1)*cloud.width + col_index-1] + 1.4f;
      const float up = distanceMap[(row_index-1)*cloud.width + col_index] + 1.0f;
      const float upRight = distanceMap[(row_index-1)*cloud.width + col_index+1] + 1.4f;
      const float left = distanceMap[row_index*cloud.width + col_index-1] + 1.0f;
      const float center = distanceMap[row_index*cloud.width + col_index];

      const float minValue = ::std::min(
        ::std::min(upLeft, up),
        ::std::min(left, upRight) );

      if (minValue < center)
      {
        distanceMap[row_index*cloud.width + col_index] = minValue;
      }
    }
  }

  // second pass
  for (int row_index = cloud.height-2; row_index >= 0; --row_index)
  {
    for (int col_index = cloud.width-2; col_index >= 0; --col_index)
    {
      const float lowerLeft = distanceMap[(row_index+1)*cloud.width + col_index-1] + 1.4f;
      const float lower = distanceMap[(row_index+1)*cloud.width + col_index] + 1.0f;
      const float lowerRight = distanceMap[(row_index+1)*cloud.width + col_index+1] + 1.4f;
      const float right = distanceMap[row_index*cloud.width + col_index+1] + 1.0f;
      const float center = distanceMap[row_index*cloud.width + col_index];

      const float minValue = ::std::min(
        ::std::min(lowerLeft, lower),
        ::std::min(right, lowerRight) );

      if (minValue < center)
      {
        distanceMap[row_index*cloud.width + col_index] = minValue;
      }
    }
  }


  // setup normal estimation
  //IntegralImageNormalEstimation normalEstimator;
  //normalEstimator.setInputData(
  setInputData (reinterpret_cast<float*>(&(cloud.points[0])),
                cloud.width, cloud.height, 3, 
                sizeof(cloud.points[0])/sizeof(float), (sizeof(cloud.points[0])/sizeof(float))*cloud.width, 10.0f,
                normal_estimation_method );

  // estimate normals
  normals.width  = cloud.width;
  normals.height = cloud.height;
  normals.points.resize (cloud.width * cloud.height);

  PointOutT zero_normal;
  zero_normal.normal_x = 0;
  zero_normal.normal_y = 0;
  zero_normal.normal_z = 0;

  if (useDepthDependentSmoothing)
  {
    for (int row_index = normalSmoothingSize; row_index < cloud.height-normalSmoothingSize; ++row_index)
    {
      for (int col_index = normalSmoothingSize; col_index < cloud.width-normalSmoothingSize; ++col_index)
      {
        const float depth = cloud.points[row_index*cloud.width + col_index].z;

        if (depth != 0)
        {
          float smoothing = normalSmoothingSize*static_cast<float>(depth)/(500.0f*0.001f);
          smoothing = ::std::min(distanceMap[row_index*cloud.width + col_index], smoothing);

          if (smoothing > 2.0f)
          {
            //normalEstimator.setRectSize(smoothing, smoothing);
            setRectSize (smoothing, smoothing);

            //pcl::Normal normal = normalEstimator.compute(col_index, row_index);
            PointOutT normal;
            computePointNormal (col_index, row_index, normal);

            normals.points[row_index*normals.width + col_index] = normal;
          }
          else
            normals.points[row_index*normals.width + col_index] = zero_normal;
        }
        else
          normals.points[row_index*normals.width + col_index] = zero_normal;
      }
    }
  }
  else
  {
    for (int row_index = normalSmoothingSize; row_index < cloud.height-normalSmoothingSize; ++row_index)
    {
      for (int col_index = normalSmoothingSize; col_index < cloud.width-normalSmoothingSize; ++col_index)
      {
        const float depth = cloud.points[row_index*cloud.width + col_index].z;

        if (depth != 0)
        {
          float smoothing = normalSmoothingSize*static_cast<float>(1.0f)/(500.0f*0.001f);
          smoothing = ::std::min(distanceMap[row_index*cloud.width + col_index], smoothing);

          if (smoothing > 2.0f)
          {
            //normalEstimator.setRectSize(smoothing, smoothing);
            setRectSize (smoothing, smoothing);

            //::pcl::Normal normal = normalEstimator.compute(col_index, row_index);
            PointOutT normal;
            computePointNormal (col_index, row_index, normal);

            normals.points[row_index*normals.width + col_index] = normal;
          }
          else
            normals.points[row_index*normals.width + col_index] = zero_normal;
        }
        else
          normals.points[row_index*normals.width + col_index] = zero_normal;
      }
    }
  }

  delete[] depthChangeMap;
  delete[] distanceMap;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void
pcl::IntegralImageNormalEstimation<PointInT, PointOutT>::compute (PointCloudOut &output)
{
  PointCloudIn input = *input_;
  computeFeature (input, output, 
                  use_depth_dependent_smoothing_,
                  max_depth_change_factor_,
                  normal_smoothing_size_,
                  normal_estimation_method_);
}


#define PCL_INSTANTIATE_IntegralImageNormalEstimation(T,NT) template class PCL_EXPORTS pcl::IntegralImageNormalEstimation<T,NT>;

#endif

