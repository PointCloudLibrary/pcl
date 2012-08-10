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

#include "pcl/2d/convolution.h"
#ifndef PCL_2D_EDGE_IMPL_HPP
#define PCL_2D_EDGE_IMPL_HPP
#include <pcl/common/common_headers.h> // rad2deg()

template<typename PointInT, typename PointOutT>
void
pcl::pcl_2d::edge<PointInT, PointOutT>::detectEdgeSobel (pcl::PointCloud<PointOutT> &output)
{
  convolution_->setInputCloud (input_);
  pcl::PointCloud<PointXYZI>::Ptr kernel_x (new pcl::PointCloud<PointXYZI>);
  pcl::PointCloud<PointXYZI>::Ptr magnitude_x (new pcl::PointCloud<PointXYZI>);
  kernel_->setKernelType (kernel<PointXYZI>::SOBEL_X);
  kernel_->fetchKernel (*kernel_x);
  convolution_->setKernel (*kernel_x);
  convolution_->convolve (*magnitude_x);

  pcl::PointCloud<PointXYZI>::Ptr kernel_y (new pcl::PointCloud<PointXYZI>);
  pcl::PointCloud<PointXYZI>::Ptr magnitude_y (new pcl::PointCloud<PointXYZI>);
  kernel_->setKernelType (kernel<PointXYZI>::SOBEL_Y);
  kernel_->fetchKernel (*kernel_y);
  convolution_->setKernel (*kernel_y);
  convolution_->convolve (*magnitude_y);

  const int height = input_->height;
  const int width = input_->width;

  output.resize (height * width);
  output.height = height;
  output.width = width;

  for (int i = 0; i < height; i++)
  {
    for (int j = 0; j < width; j++)
    {
      output (j, i).magnitude_x = (*magnitude_x)(j, i).intensity;
      output (j, i).magnitude_y = (*magnitude_y)(j, i).intensity;
      output (j, i).magnitude = sqrt  ((*magnitude_x)(j, i).intensity * (*magnitude_x)(j, i).intensity + (*magnitude_y)(j, i).intensity * (*magnitude_y)(j, i).intensity);
      output (j, i).direction = atan2f ((*magnitude_y)(j, i).intensity, (*magnitude_x)(j, i).intensity);
    }
  }
}

template<typename PointInT, typename PointOutT>
void
pcl::pcl_2d::edge<PointInT, PointOutT>::sobelMagnitudeDirection (pcl::PointCloud<PointOutT> &output, pcl::PointCloud<PointInT> &input_x, pcl::PointCloud<PointInT> &input_y)
{
  convolution_->setInputCloud (input_x.makeShared());
  pcl::PointCloud<PointXYZI>::Ptr kernel_x (new pcl::PointCloud<PointXYZI>);
  pcl::PointCloud<PointXYZI>::Ptr magnitude_x (new pcl::PointCloud<PointXYZI>);
  kernel_->setKernelType (kernel<PointXYZI>::SOBEL_X);
  kernel_->fetchKernel (*kernel_x);
  convolution_->setKernel (*kernel_x);
  convolution_->convolve (*magnitude_x);

  convolution_->setInputCloud (input_y.makeShared());
  pcl::PointCloud<PointXYZI>::Ptr kernel_y (new pcl::PointCloud<PointXYZI>);
  pcl::PointCloud<PointXYZI>::Ptr magnitude_y (new pcl::PointCloud<PointXYZI>);
  kernel_->setKernelType (kernel<PointXYZI>::SOBEL_Y);
  kernel_->fetchKernel (*kernel_y);
  convolution_->setKernel (*kernel_y);
  convolution_->convolve (*magnitude_y);

  const int height = input_x.height;
  const int width = input_x.width;

  output.resize (height * width);
  output.height = height;
  output.width = width;

  for (int i = 0; i < height; i++)
  {
    for (int j = 0; j < width; j++)
    {
      output (j, i).magnitude_x = (*magnitude_x)(j, i).intensity;
      output (j, i).magnitude_y = (*magnitude_y)(j, i).intensity;
      output (j, i).magnitude = sqrt  ((*magnitude_x)(j, i).intensity * (*magnitude_x)(j, i).intensity + (*magnitude_y)(j, i).intensity * (*magnitude_y)(j, i).intensity);
      output (j, i).direction = atan2f ((*magnitude_y)(j, i).intensity, (*magnitude_x)(j, i).intensity);
    }
  }
}

template<typename PointInT, typename PointOutT>
void
pcl::pcl_2d::edge<PointInT, PointOutT>::detectEdgePrewitt (pcl::PointCloud<PointOutT> &output)
{
  convolution_->setInputCloud (input_);

  pcl::PointCloud<PointXYZI>::Ptr kernel_x (new pcl::PointCloud<PointXYZI>);
  pcl::PointCloud<PointXYZI>::Ptr magnitude_x (new pcl::PointCloud<PointXYZI>);
  kernel_->setKernelType (kernel<PointXYZI>::PREWITT_X);
  kernel_->fetchKernel (*kernel_x);
  convolution_->setKernel (*kernel_x);
  convolution_->convolve (*magnitude_x);

  pcl::PointCloud<PointXYZI>::Ptr kernel_y (new pcl::PointCloud<PointXYZI>);
  pcl::PointCloud<PointXYZI>::Ptr magnitude_y (new pcl::PointCloud<PointXYZI>);
  kernel_->setKernelType (kernel<PointXYZI>::PREWITT_Y);
  kernel_->fetchKernel (*kernel_y);
  convolution_->setKernel (*kernel_y);
  convolution_->convolve (*magnitude_y);

  const int height = input_->height;
  const int width = input_->width;

  output.resize (height * width);
  output.height = height;
  output.width = width;

  for (int i = 0; i < height; i++)
  {
    for (int j = 0; j < width; j++)
    {
      output (j, i).magnitude_x = (*magnitude_x)(j, i).intensity;
      output (j, i).magnitude_y = (*magnitude_y)(j, i).intensity;
      output (j, i).magnitude = sqrt  ((*magnitude_x)(j, i).intensity * (*magnitude_x)(j, i).intensity + (*magnitude_y)(j, i).intensity * (*magnitude_y)(j, i).intensity);
      output (j, i).direction = atan2f ((*magnitude_y)(j, i).intensity, (*magnitude_x)(j, i).intensity);
    }
  }
}

template<typename PointInT, typename PointOutT>
void
pcl::pcl_2d::edge<PointInT, PointOutT>::detectEdgeRoberts (pcl::PointCloud<PointOutT> &output)
{
  convolution_->setInputCloud (input_);

  pcl::PointCloud<PointXYZI>::Ptr kernel_x (new pcl::PointCloud<PointXYZI>);
  pcl::PointCloud<PointXYZI>::Ptr magnitude_x (new pcl::PointCloud<PointXYZI>);
  kernel_->setKernelType (kernel<PointXYZI>::ROBERTS_X);
  kernel_->fetchKernel (*kernel_x);
  convolution_->setKernel (*kernel_x);
  convolution_->convolve (*magnitude_x);

  pcl::PointCloud<PointXYZI>::Ptr kernel_y (new pcl::PointCloud<PointXYZI>);
  pcl::PointCloud<PointXYZI>::Ptr magnitude_y (new pcl::PointCloud<PointXYZI>);
  kernel_->setKernelType (kernel<PointXYZI>::ROBERTS_Y);
  kernel_->fetchKernel (*kernel_y);
  convolution_->setKernel (*kernel_y);
  convolution_->convolve (*magnitude_y);

  const int height = input_->height;
  const int width = input_->width;

  output.resize (height * width);
  output.height = height;
  output.width = width;

  for (int i = 0; i < height; i++)
  {
    for (int j = 0; j < width; j++)
    {
      output (j, i).magnitude_x = (*magnitude_x)(j, i).intensity;
      output (j, i).magnitude_y = (*magnitude_y)(j, i).intensity;
      output (j, i).magnitude = sqrt  ((*magnitude_x)(j, i).intensity * (*magnitude_x)(j, i).intensity + (*magnitude_y)(j, i).intensity * (*magnitude_y)(j, i).intensity);
      output (j, i).direction = atan2  ((*magnitude_y)(j, i).intensity, (*magnitude_x)(j, i).intensity);
    }
  }
}

template<typename PointInT, typename PointOutT>
void
pcl::pcl_2d::edge<PointInT, PointOutT>::cannyTraceEdge (int rowOffset, int colOffset, int row, int col, pcl::PointCloud<PointXYZI> &maxima)
{
  int newRow = row + rowOffset;
  int newCol = col + colOffset;
  if (newRow > 0 && newRow < maxima.height && newCol > 0 && newCol < maxima.width)
  {
    if (maxima (newCol, newRow).intensity == 0.0f || maxima (newCol, newRow).intensity == std::numeric_limits<float>::max ())
      return;

    maxima (newCol, newRow).intensity = std::numeric_limits<float>::max ();
    cannyTraceEdge ( 1, 0, newRow, newCol, maxima);
    cannyTraceEdge (-1, 0, newRow, newCol, maxima);
    cannyTraceEdge ( 1, 1, newRow, newCol, maxima);
    cannyTraceEdge (-1, -1, newRow, newCol, maxima);
    cannyTraceEdge ( 0, -1, newRow, newCol, maxima);
    cannyTraceEdge ( 0, 1, newRow, newCol, maxima);
    cannyTraceEdge (-1, 1, newRow, newCol, maxima);
    cannyTraceEdge ( 1, -1, newRow, newCol, maxima);
  }
}

template<typename PointInT, typename PointOutT>
void
pcl::pcl_2d::edge<PointInT, PointOutT>::discretizeAngles (pcl::PointCloud<PointOutT> &thet)
{
  const int height = thet.height;
  const int width = thet.width;
  float angle;
  for (int i = 0; i < height; i++)
  {
    for (int j = 0; j < width; j++)
    {
      angle = pcl::rad2deg (thet (j, i).direction);
      if (((angle <= 22.5) && (angle >= -22.5)) || (angle >= 157.5) || (angle <= -157.5))
        thet (j, i).direction = 0;
      else
        if (((angle > 22.5) && (angle < 67.5)) || ((angle < -112.5) && (angle > -157.5)))
          thet (j, i).direction = 45;
        else
          if (((angle >= 67.5) && (angle <= 112.5)) || ((angle <= -67.5) && (angle >= -112.5)))
            thet (j, i).direction = 90;
          else
            if (((angle > 112.5) && (angle < 157.5)) || ((angle < -22.5) && (angle > -67.5)))
              thet (j, i).direction = 135;
    }
  }
}

template<typename PointInT, typename PointOutT>
void
pcl::pcl_2d::edge<PointInT, PointOutT>::suppressNonMaxima (pcl::PointCloud<PointXYZIEdge> &edges, pcl::PointCloud<PointXYZI> &maxima, float tLow)
{
  const int height = edges.height;
  const int width = edges.width;

  maxima.height = height;
  maxima.width = width;
  maxima.resize (height * width);

  for (int i=0; i<height; i++)
  {
    for (int j=0; j<width; j++)
    {
      maxima (j, i).intensity = 0.0f;
    }
  }

  //for (int row=0; row<height; row++)
  //  maxima[row].resize (width, 0.0f);

  /*tHigh and non-maximal supression*/
  for (int i = 1; i < height - 1; i++)
  {
    for (int j = 1; j < width - 1; j++)
    {
      if (edges (j, i).magnitude < tLow)
        continue;
      //maxima (j, i).intensity = 0;
      switch (int (edges (j, i).direction))
      {
        case 0:
          if (edges (j, i).magnitude >= edges (j - 1, i).magnitude && edges (j, i).magnitude >= edges (j + 1, i).magnitude)
            maxima (j, i).intensity = edges (j, i).magnitude;
          break;
        case 45:
          if (edges (j, i).magnitude >= edges (j - 1, i - 1).magnitude && edges (j, i).magnitude >= edges (j + 1, i + 1).magnitude)
            maxima (j, i).intensity = edges (j, i).magnitude;
          break;
        case 90:
          if (edges (j, i).magnitude >= edges (j, i - 1).magnitude && edges (j, i).magnitude >= edges (j, i + 1).magnitude)
            maxima (j, i).intensity = edges (j, i).magnitude;
          break;
        case 135:
          if (edges (j, i).magnitude >= edges (j + 1, i - 1).magnitude && edges (j, i).magnitude >= edges (j - 1, i + 1).magnitude)
            maxima (j, i).intensity = edges (j, i).magnitude;
          break;
      }
    }
  }
}

template<typename PointInT, typename PointOutT>
void
pcl::pcl_2d::edge<PointInT, PointOutT>::detectEdgeCanny (pcl::PointCloud<PointOutT> &output)
{
  float tHigh = hysteresis_threshold_high_;
  float tLow = hysteresis_threshold_low_;
  const int height = input_->height;
  const int width = input_->width;

  output.resize (height * width);
  output.height = height;
  output.width = width;

  /*noise reduction using gaussian blurring*/
  pcl::PointCloud<PointXYZI>::Ptr gaussian_kernel (new pcl::PointCloud<PointXYZI>);
  PointCloudInPtr smoothed_cloud (new PointCloudIn);
  kernel_->setKernelSize (3);
  kernel_->setKernelSigma (1.0);
  kernel_->setKernelType (kernel<PointXYZI>::GAUSSIAN);
  kernel_->fetchKernel (*gaussian_kernel);
  convolution_->setKernel (*gaussian_kernel);
  convolution_->setInputCloud (input_);
  convolution_->convolve (*smoothed_cloud);

  /*edge detection usign Sobel*/
  pcl::PointCloud<PointXYZIEdge>::Ptr edges (new pcl::PointCloud<PointXYZIEdge>);
  setInputCloud (smoothed_cloud);
  detectEdgeSobel (*edges);

  /*edge discretization*/
  discretizeAngles (*edges);

  /*tHigh and non-maximal supression*/
  pcl::PointCloud<PointXYZI>::Ptr maxima (new pcl::PointCloud<PointXYZI>);
  suppressNonMaxima (*edges, *maxima, tLow);

  /*edge tracing*/
  for (int i = 0; i < height; i++)
  {
    for (int j = 0; j < width; j++)
    {
      if ((*maxima)(j, i).intensity < tHigh || (*maxima)(j, i).intensity == std::numeric_limits<float>::max ())
        continue;

      (*maxima)(j, i).intensity = std::numeric_limits<float>::max ();
      cannyTraceEdge ( 1, 0, i, j, *maxima);
      cannyTraceEdge (-1, 0, i, j, *maxima);
      cannyTraceEdge ( 1, 1, i, j, *maxima);
      cannyTraceEdge (-1, -1, i, j, *maxima);
      cannyTraceEdge ( 0, -1, i, j, *maxima);
      cannyTraceEdge ( 0, 1, i, j, *maxima);
      cannyTraceEdge (-1, 1, i, j, *maxima);
      cannyTraceEdge ( 1, -1, i, j, *maxima);
    }
  }

  /*final thresholding*/
  for (int i = 0; i < height; i++)
  {
    for (int j = 0; j < width; j++)
    {
      if ((*maxima)(j, i).intensity == std::numeric_limits<float>::max ())
        output (j, i).magnitude = 255;
      else
        output (j, i).magnitude = 0;
    }
  }
}

template<typename PointInT, typename PointOutT>
void
pcl::pcl_2d::edge<PointInT, PointOutT>::canny (pcl::PointCloud<PointOutT> &output, pcl::PointCloud<PointInT> &input_x, pcl::PointCloud<PointInT> &input_y)
{
  float tHigh = hysteresis_threshold_high_;
  float tLow = hysteresis_threshold_low_;
  const int height = input_x.height;
  const int width = input_x.width;

  output.resize (height * width);
  output.height = height;
  output.width = width;

  /*noise reduction using gaussian blurring*/
  pcl::PointCloud<PointXYZI>::Ptr gaussian_kernel (new pcl::PointCloud<PointXYZI>);
  kernel_->setKernelSize (3);
  kernel_->setKernelSigma (1.0);
  kernel_->setKernelType (kernel<PointXYZI>::GAUSSIAN);
  kernel_->fetchKernel (*gaussian_kernel);
  convolution_->setKernel (*gaussian_kernel);

  PointCloudIn smoothed_cloud_x;
  convolution_->setInputCloud (input_x.makeShared());
  convolution_->convolve (smoothed_cloud_x);

  PointCloudIn smoothed_cloud_y;
  convolution_->setInputCloud (input_y.makeShared());
  convolution_->convolve (smoothed_cloud_y);


  /*edge detection usign Sobel*/
  pcl::PointCloud<PointXYZIEdge>::Ptr edges (new pcl::PointCloud<PointXYZIEdge>);
  sobelMagnitudeDirection (*edges.get(), smoothed_cloud_x, smoothed_cloud_y);

  /*edge discretization*/
  discretizeAngles (*edges);

  pcl::PointCloud<PointXYZI>::Ptr maxima (new pcl::PointCloud<PointXYZI>);
  suppressNonMaxima (*edges, *maxima, tLow);

  /*edge tracing*/
  for (int i = 0; i < height; i++)
  {
    for (int j = 0; j < width; j++)
    {
      if ((*maxima)(j, i).intensity < tHigh || (*maxima)(j, i).intensity == std::numeric_limits<float>::max ())
        continue;

      (*maxima)(j, i).intensity = std::numeric_limits<float>::max ();
      cannyTraceEdge ( 1, 0, i, j, *maxima);
      cannyTraceEdge (-1, 0, i, j, *maxima);
      cannyTraceEdge ( 1, 1, i, j, *maxima);
      cannyTraceEdge (-1, -1, i, j, *maxima);
      cannyTraceEdge ( 0, -1, i, j, *maxima);
      cannyTraceEdge ( 0, 1, i, j, *maxima);
      cannyTraceEdge (-1, 1, i, j, *maxima);
      cannyTraceEdge ( 1, -1, i, j, *maxima);
    }
  }

  /*final thresholding*/
  for (int i = 0; i < height; i++)
  {
    for (int j = 0; j < width; j++)
    {
      if ((*maxima)(j, i).intensity == std::numeric_limits<float>::max ())
        output (j, i).magnitude = 255;
      else
        output (j, i).magnitude = 0;
    }
  }
}

template<typename PointInT, typename PointOutT>
void
pcl::pcl_2d::edge<PointInT, PointOutT>::detectEdgeLoG  (pcl::PointCloud<PointOutT> &output, const float kernel_sigma, const float kernel_size)
{
  convolution_->setInputCloud (input_);

  const int height = input_->height;
  const int width = input_->width;

  pcl::PointCloud<PointXYZI>::Ptr log_kernel (new pcl::PointCloud<PointXYZI>);
  kernel_->setKernelType(kernel<PointXYZI>::LOG);
  kernel_->setKernelSigma (kernel_sigma);
  kernel_->setKernelSize (kernel_size);
  kernel_->fetchKernel (*log_kernel);
  convolution_->setKernel (*log_kernel);
  convolution_->convolve (*output);
}

template<typename PointInT, typename PointOutT>
void
pcl::pcl_2d::edge<PointInT, PointOutT>::setInputCloud (PointCloudInPtr input){
  input_ = input;
}

template<typename PointInT, typename PointOutT>
void
pcl::pcl_2d::edge<PointInT, PointOutT>::setHysteresisThresholdLow(float t_low){
  hysteresis_threshold_low_ = t_low;
}

template<typename PointInT, typename PointOutT>
void
pcl::pcl_2d::edge<PointInT, PointOutT>::setHysteresisThresholdHigh(float t_high){
  hysteresis_threshold_high_ = t_high;
}

#endif
