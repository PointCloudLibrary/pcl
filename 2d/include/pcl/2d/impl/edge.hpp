/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
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
 */

#ifndef PCL_2D_EDGE_IMPL_HPP
#define PCL_2D_EDGE_IMPL_HPP

#include <pcl/2d/convolution.h>
#include <pcl/common/common_headers.h> // rad2deg()
#include <pcl/console/time.h>

//////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void
pcl::Edge<PointInT, PointOutT>::detectEdgeSobel (
    pcl::PointCloud<PointOutT> &output)
{
  //pcl::console::TicToc tt;
  //tt.tic ();
  convolution_.setInputCloud (input_);
  pcl::PointCloud<PointXYZI>::Ptr kernel_x (new pcl::PointCloud<PointXYZI>);
  pcl::PointCloud<PointXYZI>::Ptr magnitude_x (new pcl::PointCloud<PointXYZI>);
  kernel_.setKernelType (kernel<PointXYZI>::SOBEL_X);
  kernel_.fetchKernel (*kernel_x);
  convolution_.setKernel (*kernel_x);
  convolution_.filter (*magnitude_x);
  //PCL_ERROR ("Convolve X: %g\n", tt.toc ()); tt.tic ();

  pcl::PointCloud<PointXYZI>::Ptr kernel_y (new pcl::PointCloud<PointXYZI>);
  pcl::PointCloud<PointXYZI>::Ptr magnitude_y (new pcl::PointCloud<PointXYZI>);
  kernel_.setKernelType (kernel<PointXYZI>::SOBEL_Y);
  kernel_.fetchKernel (*kernel_y);
  convolution_.setKernel (*kernel_y);
  convolution_.filter (*magnitude_y);
  //PCL_ERROR ("Convolve Y: %g\n", tt.toc ()); tt.tic ();

  const int height = input_->height;
  const int width = input_->width;

  output.resize (height * width);
  output.height = height;
  output.width = width;

  for (size_t i = 0; i < output.size (); ++i)
  {
    output[i].magnitude_x = (*magnitude_x)[i].intensity;
    output[i].magnitude_y = (*magnitude_y)[i].intensity;
    output[i].magnitude = 
      sqrtf ((*magnitude_x)[i].intensity * (*magnitude_x)[i].intensity + 
             (*magnitude_y)[i].intensity * (*magnitude_y)[i].intensity);
    output[i].direction = 
      atan2f ((*magnitude_y)[i].intensity, (*magnitude_x)[i].intensity);
  }
  //PCL_ERROR ("Rest: %g\n", tt.toc ());
}

//////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void
pcl::Edge<PointInT, PointOutT>::sobelMagnitudeDirection (
    const pcl::PointCloud<PointInT> &input_x, 
    const pcl::PointCloud<PointInT> &input_y,
    pcl::PointCloud<PointOutT> &output)
{
  convolution_.setInputCloud (input_x.makeShared());
  pcl::PointCloud<PointXYZI>::Ptr kernel_x (new pcl::PointCloud<PointXYZI>);
  pcl::PointCloud<PointXYZI>::Ptr magnitude_x (new pcl::PointCloud<PointXYZI>);
  kernel_.setKernelType (kernel<PointXYZI>::SOBEL_X);
  kernel_.fetchKernel (*kernel_x);
  convolution_.setKernel (*kernel_x);
  convolution_.filter (*magnitude_x);

  convolution_.setInputCloud (input_y.makeShared());
  pcl::PointCloud<PointXYZI>::Ptr kernel_y (new pcl::PointCloud<PointXYZI>);
  pcl::PointCloud<PointXYZI>::Ptr magnitude_y (new pcl::PointCloud<PointXYZI>);
  kernel_.setKernelType (kernel<PointXYZI>::SOBEL_Y);
  kernel_.fetchKernel (*kernel_y);
  convolution_.setKernel (*kernel_y);
  convolution_.filter (*magnitude_y);

  const int height = input_x.height;
  const int width = input_x.width;

  output.resize (height * width);
  output.height = height;
  output.width = width;

  for (size_t i = 0; i < output.size (); ++i)
  {
    output[i].magnitude_x = (*magnitude_x)[i].intensity;
    output[i].magnitude_y = (*magnitude_y)[i].intensity;
    output[i].magnitude = 
      sqrtf ((*magnitude_x)[i].intensity * (*magnitude_x)[i].intensity + 
             (*magnitude_y)[i].intensity * (*magnitude_y)[i].intensity);
    output[i].direction = 
      atan2f ((*magnitude_y)[i].intensity, (*magnitude_x)[i].intensity);
  }
}

//////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void
pcl::Edge<PointInT, PointOutT>::detectEdgePrewitt (pcl::PointCloud<PointOutT> &output)
{
  convolution_.setInputCloud (input_);

  pcl::PointCloud<PointXYZI>::Ptr kernel_x (new pcl::PointCloud<PointXYZI>);
  pcl::PointCloud<PointXYZI>::Ptr magnitude_x (new pcl::PointCloud<PointXYZI>);
  kernel_.setKernelType (kernel<PointXYZI>::PREWITT_X);
  kernel_.fetchKernel (*kernel_x);
  convolution_.setKernel (*kernel_x);
  convolution_.filter (*magnitude_x);

  pcl::PointCloud<PointXYZI>::Ptr kernel_y (new pcl::PointCloud<PointXYZI>);
  pcl::PointCloud<PointXYZI>::Ptr magnitude_y (new pcl::PointCloud<PointXYZI>);
  kernel_.setKernelType (kernel<PointXYZI>::PREWITT_Y);
  kernel_.fetchKernel (*kernel_y);
  convolution_.setKernel (*kernel_y);
  convolution_.filter (*magnitude_y);

  const int height = input_->height;
  const int width = input_->width;

  output.resize (height * width);
  output.height = height;
  output.width = width;

  for (size_t i = 0; i < output.size (); ++i)
  {
    output[i].magnitude_x = (*magnitude_x)[i].intensity;
    output[i].magnitude_y = (*magnitude_y)[i].intensity;
    output[i].magnitude = 
      sqrtf ((*magnitude_x)[i].intensity * (*magnitude_x)[i].intensity + 
             (*magnitude_y)[i].intensity * (*magnitude_y)[i].intensity);
    output[i].direction = 
      atan2f ((*magnitude_y)[i].intensity, (*magnitude_x)[i].intensity);
  }
}

//////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void
pcl::Edge<PointInT, PointOutT>::detectEdgeRoberts (pcl::PointCloud<PointOutT> &output)
{
  convolution_.setInputCloud (input_);

  pcl::PointCloud<PointXYZI>::Ptr kernel_x (new pcl::PointCloud<PointXYZI>);
  pcl::PointCloud<PointXYZI>::Ptr magnitude_x (new pcl::PointCloud<PointXYZI>);
  kernel_.setKernelType (kernel<PointXYZI>::ROBERTS_X);
  kernel_.fetchKernel (*kernel_x);
  convolution_.setKernel (*kernel_x);
  convolution_.filter (*magnitude_x);

  pcl::PointCloud<PointXYZI>::Ptr kernel_y (new pcl::PointCloud<PointXYZI>);
  pcl::PointCloud<PointXYZI>::Ptr magnitude_y (new pcl::PointCloud<PointXYZI>);
  kernel_.setKernelType (kernel<PointXYZI>::ROBERTS_Y);
  kernel_.fetchKernel (*kernel_y);
  convolution_.setKernel (*kernel_y);
  convolution_.filter (*magnitude_y);

  const int height = input_->height;
  const int width = input_->width;

  output.resize (height * width);
  output.height = height;
  output.width = width;

  for (size_t i = 0; i < output.size (); ++i)
  {
    output[i].magnitude_x = (*magnitude_x)[i].intensity;
    output[i].magnitude_y = (*magnitude_y)[i].intensity;
    output[i].magnitude = 
      sqrtf ((*magnitude_x)[i].intensity * (*magnitude_x)[i].intensity + 
             (*magnitude_y)[i].intensity * (*magnitude_y)[i].intensity);
    output[i].direction = 
      atan2f ((*magnitude_y)[i].intensity, (*magnitude_x)[i].intensity);
  }
}

//////////////////////////////////////////////////////////////////////////////
template<typename PointInT, typename PointOutT> void
pcl::Edge<PointInT, PointOutT>::cannyTraceEdge (
    int rowOffset, int colOffset, int row, int col, 
    pcl::PointCloud<PointXYZI> &maxima)
{
  int newRow = row + rowOffset;
  int newCol = col + colOffset;
  PointXYZI &pt = maxima (newCol, newRow);

  if (newRow > 0 && newRow < static_cast<int> (maxima.height) && newCol > 0 && newCol < static_cast<int> (maxima.width))
  {
    if (pt.intensity == 0.0f || pt.intensity == std::numeric_limits<float>::max ())
      return;

    pt.intensity = std::numeric_limits<float>::max ();
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

//////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void
pcl::Edge<PointInT, PointOutT>::discretizeAngles (pcl::PointCloud<PointOutT> &thet)
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

//////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void
pcl::Edge<PointInT, PointOutT>::suppressNonMaxima (
    const pcl::PointCloud<PointXYZIEdge> &edges, 
    pcl::PointCloud<PointXYZI> &maxima, float tLow)
{
  const int height = edges.height;
  const int width = edges.width;

  maxima.height = height;
  maxima.width = width;
  maxima.resize (height * width);

  for (size_t i = 0; i < maxima.size (); ++i)
    maxima[i].intensity = 0.0f;

  // tHigh and non-maximal supression
  for (int i = 1; i < height - 1; i++)
  {
    for (int j = 1; j < width - 1; j++)
    {
      const PointXYZIEdge &ptedge = edges (j, i);
      PointXYZI &ptmax = maxima (j, i);

      if (ptedge.magnitude < tLow)
        continue;

      //maxima (j, i).intensity = 0;
      
      switch (int (ptedge.direction))
      {
        case 0:
        {
          if (ptedge.magnitude >= edges (j - 1, i).magnitude && 
              ptedge.magnitude >= edges (j + 1, i).magnitude)
            ptmax.intensity = ptedge.magnitude;
          break;
        }
        case 45:
        {
          if (ptedge.magnitude >= edges (j - 1, i - 1).magnitude && 
              ptedge.magnitude >= edges (j + 1, i + 1).magnitude)
            ptmax.intensity = ptedge.magnitude;
          break;
        }
        case 90:
        {
          if (ptedge.magnitude >= edges (j, i - 1).magnitude && 
              ptedge.magnitude >= edges (j, i + 1).magnitude)
            ptmax.intensity = ptedge.magnitude;
          break;
        }
        case 135:
        {
          if (ptedge.magnitude >= edges (j + 1, i - 1).magnitude && 
              ptedge.magnitude >= edges (j - 1, i + 1).magnitude)
            ptmax.intensity = ptedge.magnitude;
          break;
        }
      }
    }
  }
}

//////////////////////////////////////////////////////////////////////////////
template<typename PointInT, typename PointOutT> void
pcl::Edge<PointInT, PointOutT>::detectEdgeCanny (pcl::PointCloud<PointOutT> &output)
{
  float tHigh = hysteresis_threshold_high_;
  float tLow = hysteresis_threshold_low_;
  const int height = input_->height;
  const int width = input_->width;

  output.resize (height * width);
  output.height = height;
  output.width = width;

  //pcl::console::TicToc tt;
  //tt.tic ();
  
  // Noise reduction using gaussian blurring
  pcl::PointCloud<PointXYZI>::Ptr gaussian_kernel (new pcl::PointCloud<PointXYZI>);
  PointCloudInPtr smoothed_cloud (new PointCloudIn);
  kernel_.setKernelSize (3);
  kernel_.setKernelSigma (1.0);
  kernel_.setKernelType (kernel<PointXYZI>::GAUSSIAN);
  kernel_.fetchKernel (*gaussian_kernel);
  convolution_.setKernel (*gaussian_kernel);
  convolution_.setInputCloud (input_);
  convolution_.filter (*smoothed_cloud);
  //PCL_ERROR ("Gaussian blur: %g\n", tt.toc ()); tt.tic ();
  
  // Edge detection usign Sobel
  pcl::PointCloud<PointXYZIEdge>::Ptr edges (new pcl::PointCloud<PointXYZIEdge>);
  setInputCloud (smoothed_cloud);
  detectEdgeSobel (*edges);
  //PCL_ERROR ("Sobel: %g\n", tt.toc ()); tt.tic ();
  
  // Edge discretization
  discretizeAngles (*edges);
  //PCL_ERROR ("Discretize: %g\n", tt.toc ()); tt.tic ();

  // tHigh and non-maximal supression
  pcl::PointCloud<PointXYZI>::Ptr maxima (new pcl::PointCloud<PointXYZI>);
  suppressNonMaxima (*edges, *maxima, tLow);
  //PCL_ERROR ("NM suppress: %g\n", tt.toc ()); tt.tic ();

  // Edge tracing
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
  //PCL_ERROR ("Edge tracing: %g\n", tt.toc ());

  // Final thresholding
  for (size_t i = 0; i < input_->size (); ++i)
  {
    if ((*maxima)[i].intensity == std::numeric_limits<float>::max ())
      output[i].magnitude = 255;
    else
      output[i].magnitude = 0;
  }
}

//////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void
pcl::Edge<PointInT, PointOutT>::canny (
    const pcl::PointCloud<PointInT> &input_x, 
    const pcl::PointCloud<PointInT> &input_y,
    pcl::PointCloud<PointOutT> &output)
{
  float tHigh = hysteresis_threshold_high_;
  float tLow = hysteresis_threshold_low_;
  const int height = input_x.height;
  const int width = input_x.width;

  output.resize (height * width);
  output.height = height;
  output.width = width;

  // Noise reduction using gaussian blurring
  pcl::PointCloud<PointXYZI>::Ptr gaussian_kernel (new pcl::PointCloud<PointXYZI>);
  kernel_.setKernelSize (3);
  kernel_.setKernelSigma (1.0);
  kernel_.setKernelType (kernel<PointXYZI>::GAUSSIAN);
  kernel_.fetchKernel (*gaussian_kernel);
  convolution_.setKernel (*gaussian_kernel);

  PointCloudIn smoothed_cloud_x;
  convolution_.setInputCloud (input_x.makeShared());
  convolution_.filter (smoothed_cloud_x);

  PointCloudIn smoothed_cloud_y;
  convolution_.setInputCloud (input_y.makeShared());
  convolution_.filter (smoothed_cloud_y);


  // Edge detection usign Sobel
  pcl::PointCloud<PointXYZIEdge>::Ptr edges (new pcl::PointCloud<PointXYZIEdge>);
  sobelMagnitudeDirection (smoothed_cloud_x, smoothed_cloud_y, *edges.get ());

  // Edge discretization
  discretizeAngles (*edges);

  pcl::PointCloud<PointXYZI>::Ptr maxima (new pcl::PointCloud<PointXYZI>);
  suppressNonMaxima (*edges, *maxima, tLow);

  // Edge tracing
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

  // Final thresholding
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

//////////////////////////////////////////////////////////////////////////////
template<typename PointInT, typename PointOutT> void
pcl::Edge<PointInT, PointOutT>::detectEdgeLoG (
    const float kernel_sigma, const float kernel_size,
    pcl::PointCloud<PointOutT> &output)
{
  convolution_.setInputCloud (input_);

  pcl::PointCloud<PointXYZI>::Ptr log_kernel (new pcl::PointCloud<PointXYZI>);
  kernel_.setKernelType (kernel<PointXYZI>::LOG);
  kernel_.setKernelSigma (kernel_sigma);
  kernel_.setKernelSize (kernel_size);
  kernel_.fetchKernel (*log_kernel);
  convolution_.setKernel (*log_kernel);
  convolution_.filter (output);
}

#endif
