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

#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT>
pcl::IntegralImageNormalEstimation<PointInT, PointOutT>::~IntegralImageNormalEstimation ()
{
  if (diff_x_ != NULL) delete diff_x_;
  if (diff_y_ != NULL) delete diff_y_;
  if (depth_data_ != NULL) delete depth_data_;
  if (distance_map_ != NULL) delete distance_map_;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void
pcl::IntegralImageNormalEstimation<PointInT, PointOutT>::initData ()
{
  // compute derivatives
  if (diff_x_ != NULL) delete diff_x_;
  if (diff_y_ != NULL) delete diff_y_;
  if (depth_data_ != NULL) delete depth_data_;
  if (distance_map_ != NULL) delete distance_map_;
  diff_x_ = NULL;
  diff_y_ = NULL;
  depth_data_ = NULL;
  distance_map_ = NULL;

  if (normal_estimation_method_ == COVARIANCE_MATRIX)
    initCovarianceMatrixMethod ();
  else if (normal_estimation_method_ == AVERAGE_3D_GRADIENT)
    initAverage3DGradientMethod ();
  else if (normal_estimation_method_ == AVERAGE_DEPTH_CHANGE)
    initAverageDepthChangeMethod ();
  else if (normal_estimation_method_ == SIMPLE_3D_GRADIENT)
    initSimple3DGradientMethod ();
}


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void
pcl::IntegralImageNormalEstimation<PointInT, PointOutT>::setRectSize (const int width, const int height)
{
  rect_width_      = width;
  rect_width_2_    = width >> 1;
  rect_width_4_    = width >> 2;
  rect_height_     = height;
  rect_height_2_   = height >> 1;
  rect_height_4_   = height >> 2;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void
pcl::IntegralImageNormalEstimation<PointInT, PointOutT>::initSimple3DGradientMethod ()
{
  // number of DataType entries per element (equal or bigger than dimensions)
  int element_stride = sizeof (PointInT) / sizeof (float);
  // number of DataType entries per row (equal or bigger than element_stride number of elements per row)
  int row_stride     = element_stride * input_->width;

  const float *data_ = reinterpret_cast<const float*> (&input_->points[0]);

  integral_image_XYZ_.setSecondOrderComputation (false);
  integral_image_XYZ_.setInput (data_, input_->width, input_->height, element_stride, row_stride);

  init_simple_3d_gradient_ = true;
  init_covariance_matrix_ = init_average_3d_gradient_ = init_depth_change_ = false;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void
pcl::IntegralImageNormalEstimation<PointInT, PointOutT>::initCovarianceMatrixMethod ()
{
  // number of DataType entries per element (equal or bigger than dimensions)
  int element_stride = sizeof (PointInT) / sizeof (float);
  // number of DataType entries per row (equal or bigger than element_stride number of elements per row)
  int row_stride     = element_stride * input_->width;

  const float *data_ = reinterpret_cast<const float*> (&input_->points[0]);

  integral_image_XYZ_.setSecondOrderComputation (true);
  integral_image_XYZ_.setInput (data_, input_->width, input_->height, element_stride, row_stride);

  init_covariance_matrix_ = true;
  init_average_3d_gradient_ = init_depth_change_ = init_simple_3d_gradient_ = false;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void
pcl::IntegralImageNormalEstimation<PointInT, PointOutT>::initAverage3DGradientMethod ()
{
  size_t data_size = (input_->points.size () << 2);
  diff_x_ = new float[data_size];
  diff_y_ = new float[data_size];

  memset (diff_x_, 0, sizeof(float) * data_size);
  memset (diff_y_, 0, sizeof(float) * data_size);

  // x u x
  // l x r
  // x d x
  const PointInT* point_up = &(input_->points [1]);
  const PointInT* point_dn = point_up + (input_->width << 1);//&(input_->points [1 + (input_->width << 1)]);
  const PointInT* point_lf = &(input_->points [input_->width]);
  const PointInT* point_rg = point_lf + 2; //&(input_->points [input_->width + 2]);
  float* diff_x_ptr = diff_x_ + ((input_->width + 1) << 2);
  float* diff_y_ptr = diff_y_ + ((input_->width + 1) << 2);
  unsigned diff_skip = 8; // skip last element in row and the first in the next row

  for (size_t ri = 1; ri < input_->height - 1; ++ri
                                             , point_up += input_->width
                                             , point_dn += input_->width
                                             , point_lf += input_->width
                                             , point_rg += input_->width
                                             , diff_x_ptr += diff_skip
                                             , diff_y_ptr += diff_skip)
  {
    for (size_t ci = 0; ci < input_->width - 2; ++ci, diff_x_ptr += 4, diff_y_ptr += 4)
    {
      diff_x_ptr[0] = point_rg[ci].x - point_lf[ci].x;
      diff_x_ptr[1] = point_rg[ci].y - point_lf[ci].y;
      diff_x_ptr[2] = point_rg[ci].z - point_lf[ci].z;

      diff_y_ptr[0] = point_dn[ci].x - point_up[ci].x;
      diff_y_ptr[1] = point_dn[ci].y - point_up[ci].y;
      diff_y_ptr[2] = point_dn[ci].z - point_up[ci].z;
    }
  }

  // Compute integral images
  integral_image_DX_.setInput (diff_x_, input_->width, input_->height, 4, input_->width << 2);
  integral_image_DY_.setInput (diff_y_, input_->width, input_->height, 4, input_->width << 2);
  init_covariance_matrix_ = init_depth_change_ = init_simple_3d_gradient_ = false;
  init_average_3d_gradient_ = true;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void
pcl::IntegralImageNormalEstimation<PointInT, PointOutT>::initAverageDepthChangeMethod ()
{
  // number of DataType entries per element (equal or bigger than dimensions)
  int element_stride = sizeof (PointInT) / sizeof (float);
  // number of DataType entries per row (equal or bigger than element_stride number of elements per row)
  int row_stride     = element_stride * input_->width;

  const float *data_ = reinterpret_cast<const float*> (&input_->points[0]);

  // integral image over the z - value
  integral_image_depth_.setInput (&(data_[2]), input_->width, input_->height, element_stride, row_stride);
  init_depth_change_ = true;
  init_covariance_matrix_ = init_average_3d_gradient_ = init_simple_3d_gradient_ = false;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void
pcl::IntegralImageNormalEstimation<PointInT, PointOutT>::computePointNormal (
    const int pos_x, const int pos_y, const unsigned point_index, PointOutT &normal)
{
  float bad_point = std::numeric_limits<float>::quiet_NaN ();

  if (normal_estimation_method_ == COVARIANCE_MATRIX)
  {
    if (!init_covariance_matrix_)
      initCovarianceMatrixMethod ();

    unsigned count = integral_image_XYZ_.getFiniteElementsCount (pos_x - (rect_width_2_), pos_y - (rect_height_2_), rect_width_, rect_height_);

    // no valid points within the rectangular reagion?
    if (count == 0)
    {
      normal.normal_x = normal.normal_y = normal.normal_z = normal.curvature = std::numeric_limits<float>::quiet_NaN ();
      return;
    }

    EIGEN_ALIGN16 Eigen::Matrix3f covariance_matrix;
    Eigen::Vector3f center;
    typename IntegralImage2D<float, 3>::SecondOrderType so_elements;
    center = integral_image_XYZ_.getFirstOrderSum(pos_x - rect_width_2_, pos_y - rect_height_2_, rect_width_, rect_height_).cast<float> ();
    so_elements = integral_image_XYZ_.getSecondOrderSum(pos_x - rect_width_2_, pos_y - rect_height_2_, rect_width_, rect_height_);

    covariance_matrix.coeffRef (0) = static_cast<float> (so_elements [0]);
    covariance_matrix.coeffRef (1) = covariance_matrix.coeffRef (3) = static_cast<float> (so_elements [1]);
    covariance_matrix.coeffRef (2) = covariance_matrix.coeffRef (6) = static_cast<float> (so_elements [2]);
    covariance_matrix.coeffRef (4) = static_cast<float> (so_elements [3]);
    covariance_matrix.coeffRef (5) = covariance_matrix.coeffRef (7) = static_cast<float> (so_elements [4]);
    covariance_matrix.coeffRef (8) = static_cast<float> (so_elements [5]);
    covariance_matrix -= (center * center.transpose ()) / static_cast<float> (count);
    float eigen_value;
    Eigen::Vector3f eigen_vector;
    pcl::eigen33 (covariance_matrix, eigen_value, eigen_vector);
    pcl::flipNormalTowardsViewpoint (input_->points[point_index], vpx_, vpy_, vpz_, eigen_vector);
    normal.getNormalVector3fMap () = eigen_vector;

    // Compute the curvature surface change
    if (eigen_value > 0.0)
      normal.curvature = fabsf (eigen_value / (covariance_matrix.coeff (0) + covariance_matrix.coeff (4) + covariance_matrix.coeff (8)));
    else
      normal.curvature = 0;

    return;
  }
  else if (normal_estimation_method_ == AVERAGE_3D_GRADIENT)
  {
    if (!init_average_3d_gradient_)
      initAverage3DGradientMethod ();

    unsigned count_x = integral_image_DX_.getFiniteElementsCount (pos_x - rect_width_2_, pos_y - rect_height_2_, rect_width_, rect_height_);
    unsigned count_y = integral_image_DY_.getFiniteElementsCount (pos_x - rect_width_2_, pos_y - rect_height_2_, rect_width_, rect_height_);
    if (count_x == 0 || count_y == 0)
    {
      normal.normal_x = normal.normal_y = normal.normal_z = normal.curvature = std::numeric_limits<float>::quiet_NaN ();
      return;
    }
    Eigen::Vector3d gradient_x = integral_image_DX_.getFirstOrderSum (pos_x - rect_width_2_, pos_y - rect_height_2_, rect_width_, rect_height_);
    Eigen::Vector3d gradient_y = integral_image_DY_.getFirstOrderSum (pos_x - rect_width_2_, pos_y - rect_height_2_, rect_width_, rect_height_);

    Eigen::Vector3d normal_vector = gradient_y.cross (gradient_x);
    double normal_length = normal_vector.squaredNorm ();
    if (normal_length == 0.0f)
    {
      normal.getNormalVector4fMap ().setConstant (bad_point);
      normal.curvature = bad_point;
      return;
    }

    normal_vector /= sqrt (normal_length);
    pcl::flipNormalTowardsViewpoint (input_->points[point_index], vpx_, vpy_, vpz_, normal_vector);

    normal.normal_x = static_cast<float> (normal_vector [0]);
    normal.normal_y = static_cast<float> (normal_vector [1]);
    normal.normal_z = static_cast<float> (normal_vector [2]);
    normal.curvature = bad_point;
    return;
  }
  else if (normal_estimation_method_ == AVERAGE_DEPTH_CHANGE)
  {
    if (!init_depth_change_)
      initAverageDepthChangeMethod ();

//    unsigned count = integral_image_depth_.getFiniteElementsCount (pos_x - rect_width_2_, pos_y - rect_height_2_, rect_width_, rect_height_);
//    if (count == 0)
//    {
//      normal.normal_x = normal.normal_y = normal.normal_z = normal.curvature = std::numeric_limits<float>::quiet_NaN ();
//      return;
//    }
//    const float mean_L_z = integral_image_depth_.getFirstOrderSum (pos_x - rect_width_2_ - 1, pos_y - rect_height_2_    , rect_width_ - 1, rect_height_ - 1) / ((rect_width_-1)*(rect_height_-1));
//    const float mean_R_z = integral_image_depth_.getFirstOrderSum (pos_x - rect_width_2_ + 1, pos_y - rect_height_2_    , rect_width_ - 1, rect_height_ - 1) / ((rect_width_-1)*(rect_height_-1));
//    const float mean_U_z = integral_image_depth_.getFirstOrderSum (pos_x - rect_width_2_    , pos_y - rect_height_2_ - 1, rect_width_ - 1, rect_height_ - 1) / ((rect_width_-1)*(rect_height_-1));
//    const float mean_D_z = integral_image_depth_.getFirstOrderSum (pos_x - rect_width_2_    , pos_y - rect_height_2_ + 1, rect_width_ - 1, rect_height_ - 1) / ((rect_width_-1)*(rect_height_-1));

    // width and height are at least 3 x 3
    unsigned count_L_z = integral_image_depth_.getFiniteElementsCount (pos_x - rect_width_2_, pos_y - rect_height_4_, rect_width_2_, rect_height_2_);
    unsigned count_R_z = integral_image_depth_.getFiniteElementsCount (pos_x + 1            , pos_y - rect_height_4_, rect_width_2_, rect_height_2_);
    unsigned count_U_z = integral_image_depth_.getFiniteElementsCount (pos_x - rect_width_4_, pos_y - rect_height_2_, rect_width_2_, rect_height_2_);
    unsigned count_D_z = integral_image_depth_.getFiniteElementsCount (pos_x - rect_width_4_, pos_y + 1             , rect_width_2_, rect_height_2_);

    if (count_L_z == 0 || count_R_z == 0 || count_U_z == 0 || count_D_z == 0)
    {
      normal.normal_x = normal.normal_y = normal.normal_z = normal.curvature = std::numeric_limits<float>::quiet_NaN ();
      return;
    }

    float mean_L_z = static_cast<float> (integral_image_depth_.getFirstOrderSum (pos_x - rect_width_2_, pos_y - rect_height_4_, rect_width_2_, rect_height_2_) / count_L_z);
    float mean_R_z = static_cast<float> (integral_image_depth_.getFirstOrderSum (pos_x + 1            , pos_y - rect_height_4_, rect_width_2_, rect_height_2_) / count_R_z);
    float mean_U_z = static_cast<float> (integral_image_depth_.getFirstOrderSum (pos_x - rect_width_4_, pos_y - rect_height_2_, rect_width_2_, rect_height_2_) / count_U_z);
    float mean_D_z = static_cast<float> (integral_image_depth_.getFirstOrderSum (pos_x - rect_width_4_, pos_y + 1             , rect_width_2_, rect_height_2_) / count_D_z);

    PointInT pointL = input_->points[point_index - rect_width_4_ - 1];
    PointInT pointR = input_->points[point_index + rect_width_4_ + 1];
    PointInT pointU = input_->points[point_index - rect_height_4_ * input_->width - 1];
    PointInT pointD = input_->points[point_index + rect_height_4_ * input_->width + 1];

    const float mean_x_z = mean_R_z - mean_L_z;
    const float mean_y_z = mean_D_z - mean_U_z;

    const float mean_x_x = pointR.x - pointL.x;
    const float mean_x_y = pointR.y - pointL.y;
    const float mean_y_x = pointD.x - pointU.x;
    const float mean_y_y = pointD.y - pointU.y;

    float normal_x = mean_x_y * mean_y_z - mean_x_z * mean_y_y;
    float normal_y = mean_x_z * mean_y_x - mean_x_x * mean_y_z;
    float normal_z = mean_x_x * mean_y_y - mean_x_y * mean_y_x;

    const float normal_length = (normal_x * normal_x + normal_y * normal_y + normal_z * normal_z);

    if (normal_length == 0.0f)
    {
      normal.getNormalVector4fMap ().setConstant (bad_point);
      normal.curvature = bad_point;
      return;
    }

    pcl::flipNormalTowardsViewpoint (input_->points[point_index], vpx_, vpy_, vpz_, normal_x, normal_y, normal_z);
    
    const float scale = 1.0f / sqrt (normal_length);

    normal.normal_x = normal_x * scale;
    normal.normal_y = normal_y * scale;
    normal.normal_z = normal_z * scale;
    normal.curvature = bad_point;

    return;
  }
  else if (normal_estimation_method_ == SIMPLE_3D_GRADIENT)
  {
    if (!init_simple_3d_gradient_)
      initSimple3DGradientMethod ();

    // this method does not work if lots of NaNs are in the neighborhood of the point
    Eigen::Vector3d gradient_x = integral_image_XYZ_.getFirstOrderSum (pos_x + rect_width_2_, pos_y - rect_height_2_, 1, rect_height_) -
                                 integral_image_XYZ_.getFirstOrderSum (pos_x - rect_width_2_, pos_y - rect_height_2_, 1, rect_height_);

    Eigen::Vector3d gradient_y = integral_image_XYZ_.getFirstOrderSum (pos_x - rect_width_2_, pos_y + rect_height_2_, rect_width_, 1) -
                                 integral_image_XYZ_.getFirstOrderSum (pos_x - rect_width_2_, pos_y - rect_height_2_, rect_width_, 1);
    Eigen::Vector3d normal_vector = gradient_y.cross (gradient_x);
    double normal_length = normal_vector.squaredNorm ();
    if (normal_length == 0.0f)
    {
      normal.getNormalVector4fMap ().setConstant (bad_point);
      normal.curvature = bad_point;
      return;
    }

    normal_vector /= sqrt (normal_length);
    pcl::flipNormalTowardsViewpoint (input_->points[point_index], vpx_, vpy_, vpz_, normal_vector);
    
    normal.normal_x = static_cast<float> (normal_vector [0]);
    normal.normal_y = static_cast<float> (normal_vector [1]);
    normal.normal_z = static_cast<float> (normal_vector [2]);
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

  unsigned index = 0;
  for (unsigned int ri = 0; ri < input_->height-1; ++ri)
  {
    for (unsigned int ci = 0; ci < input_->width-1; ++ci, ++index)
    {
//      const float depth  = (*input_)(ci,     ri    ).z;
//      const float depthR = (*input_)(ci + 1, ri    ).z;
//      const float depthD = (*input_)(ci,     ri + 1).z;

      const float depth  = input_->points [index].z;
      const float depthR = input_->points [index + 1].z;
      const float depthD = input_->points [index + input_->width].z;

      //const float depthDependendDepthChange = (max_depth_change_factor_ * (fabs(depth)+1.0f))/(500.0f*0.001f);
      const float depthDependendDepthChange = (max_depth_change_factor_ * (fabsf (depth) + 1.0f) * 2.0f);

      if (fabs (depth - depthR) > depthDependendDepthChange
        || !pcl_isfinite (depth) || !pcl_isfinite (depthR))
      {
        depthChangeMap[index] = 0;
        depthChangeMap[index+1] = 0;
      }
      if (fabs (depth - depthD) > depthDependendDepthChange
        || !pcl_isfinite (depth) || !pcl_isfinite (depthD))
      {
        depthChangeMap[index] = 0;
        depthChangeMap[index + input_->width] = 0;
      }
    }
  }

  // compute distance map
  //float *distanceMap = new float[input_->points.size ()];
  if (distance_map_ != NULL) delete distance_map_;
  distance_map_ = new float[input_->points.size ()];
  float *distanceMap = distance_map_;
  for (size_t index = 0; index < input_->points.size (); ++index)
  {
    if (depthChangeMap[index] == 0)
      distanceMap[index] = 0.0f;
    else
      distanceMap[index] = static_cast<float> (input_->width + input_->height);
  }

  // first pass
  float* previous_row = distanceMap;
  float* current_row = previous_row + input_->width;
  for (size_t ri = 1; ri < input_->height; ++ri)
  {
    for (size_t ci = 1; ci < input_->width; ++ci)
    {
      const float upLeft  = previous_row [ci - 1] + 1.4f; //distanceMap[(ri-1)*input_->width + ci-1] + 1.4f;
      const float up      = previous_row [ci] + 1.0f;     //distanceMap[(ri-1)*input_->width + ci] + 1.0f;
      const float upRight = previous_row [ci + 1] + 1.4f; //distanceMap[(ri-1)*input_->width + ci+1] + 1.4f;
      const float left    = current_row  [ci - 1] + 1.0f;  //distanceMap[ri*input_->width + ci-1] + 1.0f;
      const float center  = current_row  [ci];             //distanceMap[ri*input_->width + ci];

      const float minValue = std::min (std::min (upLeft, up), std::min (left, upRight));

      if (minValue < center)
        current_row [ci] = minValue; //distanceMap[ri * input_->width + ci] = minValue;
    }
    previous_row = current_row;
    current_row += input_->width;
  }

  float* next_row    = distanceMap + input_->width * (input_->height - 1);
  current_row = next_row - input_->width;
  // second pass
  for (int ri = input_->height-2; ri >= 0; --ri)
  {
    for (int ci = input_->width-2; ci >= 0; --ci)
    {
      const float lowerLeft  = next_row [ci - 1] + 1.4f;    //distanceMap[(ri+1)*input_->width + ci-1] + 1.4f;
      const float lower      = next_row [ci] + 1.0f;        //distanceMap[(ri+1)*input_->width + ci] + 1.0f;
      const float lowerRight = next_row [ci + 1] + 1.4f;    //distanceMap[(ri+1)*input_->width + ci+1] + 1.4f;
      const float right      = current_row [ci + 1] + 1.0f; //distanceMap[ri*input_->width + ci+1] + 1.0f;
      const float center     = current_row [ci];            //distanceMap[ri*input_->width + ci];

      const float minValue = std::min (std::min (lowerLeft, lower), std::min (right, lowerRight));

      if (minValue < center)
        current_row [ci] = minValue; //distanceMap[ri*input_->width + ci] = minValue;
    }
    next_row = current_row;
    current_row -= input_->width;
  }

  // Set all normals that we do not touch to NaN
  // top and bottom borders
  unsigned border = int(normal_smoothing_size_);
  PointOutT* vec1 = &output [0];
  PointOutT* vec2 = vec1 + input_->width * (input_->height - border);

  size_t count = border * input_->width;
  for (size_t idx = 0; idx < count; ++idx)
  {
    vec1 [idx].getNormalVector4fMap ().setConstant (bad_point);
    vec1 [idx].curvature = bad_point;
    vec2 [idx].getNormalVector4fMap ().setConstant (bad_point);
    vec2 [idx].curvature = bad_point;
  }

  // left and right borders actually columns
  vec1 = &output [border * input_->width];
  vec2 = vec1 + input_->width - border;
  for (size_t ri = border; ri < input_->height - border; ++ri, vec1 += input_->width, vec2 += input_->width)
  {
    for (size_t ci = 0; ci < border; ++ci)
    {
      vec1 [ci].getNormalVector4fMap ().setConstant (bad_point);
      vec1 [ci].curvature = bad_point;
      vec2 [ci].getNormalVector4fMap ().setConstant (bad_point);
      vec2 [ci].curvature = bad_point;
    }
  }

  if (use_depth_dependent_smoothing_)
  {
    index = border + input_->width * border;
    unsigned skip = (border << 1);
    for (unsigned ri = border; ri < input_->height - border; ++ri, index += skip)
    {
      for (unsigned ci = border; ci < input_->width - border; ++ci, ++index)
      {
        const float depth = input_->points[index].z;
        if (!pcl_isfinite (depth))
        {
          output[index].getNormalVector4fMap ().setConstant (bad_point);
          output[index].curvature = bad_point;
          continue;
        }

        float smoothing = (std::min)(distanceMap[index], normal_smoothing_size_ + static_cast<float>(depth)/10.0f);

        if (smoothing > 2.0f)
        {
          setRectSize (static_cast<int> (smoothing), static_cast<int> (smoothing));
          computePointNormal (ci, ri, index, output [index]);
        }
        else
        {
          output[index].getNormalVector4fMap ().setConstant (bad_point);
          output[index].curvature = bad_point;
        }
      }
    }
  }
  else
  {
    float smoothing_constant = normal_smoothing_size_;

    index = border + input_->width * border;
    unsigned skip = (border << 1);
    for (unsigned ri = border; ri < input_->height - border; ++ri, index += skip)
    {
      for (unsigned ci = border; ci < input_->width - border; ++ci, ++index)
      {
        if (!pcl_isfinite (input_->points[index].z))
        {
          output [index].getNormalVector4fMap ().setConstant (bad_point);
          output [index].curvature = bad_point;
          continue;
        }

        float smoothing = (std::min)(distanceMap[index], smoothing_constant);

        if (smoothing > 2.0f)
        {
          setRectSize (static_cast<int> (smoothing), static_cast<int> (smoothing));
          computePointNormal (ci, ri, index, output [index]);
        }
        else
        {
          output [index].getNormalVector4fMap ().setConstant (bad_point);
          output [index].curvature = bad_point;
        }
      }
    }
  }

  delete[] depthChangeMap;
  //delete[] distanceMap;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> bool
pcl::IntegralImageNormalEstimation<PointInT, PointOutT>::initCompute ()
{
  if (!input_->isOrganized ())
  {
    PCL_ERROR ("[pcl::IntegralImageNormalEstimation::initCompute] Input dataset is not organized (height = 1).\n");
    return (false);
  }
  return (Feature<PointInT, PointOutT>::initCompute ());
}

#define PCL_INSTANTIATE_IntegralImageNormalEstimation(T,NT) template class PCL_EXPORTS pcl::IntegralImageNormalEstimation<T,NT>;

#endif

