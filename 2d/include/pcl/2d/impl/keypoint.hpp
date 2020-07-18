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
 *  keypoint.hpp
 *
 *  Created on: May 28, 2012
 *      Author: somani
 */

#pragma once

#include <pcl/2d/convolution.h>
#include <pcl/2d/edge.h>
#include <pcl/2d/keypoint.h> // for pcl::Keypoint

#include <limits>

namespace pcl {

template <typename ImageType>
void
Keypoint<ImageType>::harrisCorner(ImageType& output,
                                  ImageType& input,
                                  const float sigma_d,
                                  const float sigma_i,
                                  const float alpha,
                                  const float thresh)
{
  /*creating the gaussian kernels*/
  ImageType kernel_d;
  ImageType kernel_i;
  conv_2d.gaussianKernel(5, sigma_d, kernel_d);
  conv_2d.gaussianKernel(5, sigma_i, kernel_i);

  /*scaling the image with differentiation scale*/
  ImageType smoothed_image;
  conv_2d.convolve(smoothed_image, kernel_d, input);

  /*image derivatives*/
  ImageType I_x, I_y;
  edge_detection.ComputeDerivativeXCentral(I_x, smoothed_image);
  edge_detection.ComputeDerivativeYCentral(I_y, smoothed_image);

  /*second moment matrix*/
  ImageType I_x2, I_y2, I_xI_y;
  imageElementMultiply(I_x2, I_x, I_x);
  imageElementMultiply(I_y2, I_y, I_y);
  imageElementMultiply(I_xI_y, I_x, I_y);

  /*scaling second moment matrix with integration scale*/
  ImageType M00, M10, M11;
  conv_2d.convolve(M00, kernel_i, I_x2);
  conv_2d.convolve(M10, kernel_i, I_xI_y);
  conv_2d.convolve(M11, kernel_i, I_y2);

  /*harris function*/
  const std::size_t height = input.size();
  const std::size_t width = input[0].size();
  output.resize(height);
  for (std::size_t i = 0; i < height; i++) {
    output[i].resize(width);
    for (std::size_t j = 0; j < width; j++) {
      output[i][j] = M00[i][j] * M11[i][j] - (M10[i][j] * M10[i][j]) -
                     alpha * ((M00[i][j] + M11[i][j]) * (M00[i][j] + M11[i][j]));
      if (thresh != 0) {
        if (output[i][j] < thresh)
          output[i][j] = 0;
        else
          output[i][j] = 255;
      }
    }
  }

  /*local maxima*/
  for (std::size_t i = 1; i < height - 1; i++) {
    for (std::size_t j = 1; j < width - 1; j++) {
      if (output[i][j] > output[i - 1][j - 1] && output[i][j] > output[i - 1][j] &&
          output[i][j] > output[i - 1][j + 1] && output[i][j] > output[i][j - 1] &&
          output[i][j] > output[i][j + 1] && output[i][j] > output[i + 1][j - 1] &&
          output[i][j] > output[i + 1][j] && output[i][j] > output[i + 1][j + 1])
        ;
      else
        output[i][j] = 0;
    }
  }
}

template <typename ImageType>
void
Keypoint<ImageType>::hessianBlob(ImageType& output,
                                 ImageType& input,
                                 const float sigma,
                                 bool SCALED)
{
  /*creating the gaussian kernels*/
  ImageType kernel, cornerness;
  conv_2d.gaussianKernel(5, sigma, kernel);

  /*scaling the image with differentiation scale*/
  ImageType smoothed_image;
  conv_2d.convolve(smoothed_image, kernel, input);

  /*image derivatives*/
  ImageType I_x, I_y;
  edge_detection.ComputeDerivativeXCentral(I_x, smoothed_image);
  edge_detection.ComputeDerivativeYCentral(I_y, smoothed_image);

  /*second moment matrix*/
  ImageType I_xx, I_yy, I_xy;
  edge_detection.ComputeDerivativeXCentral(I_xx, I_x);
  edge_detection.ComputeDerivativeYCentral(I_xy, I_x);
  edge_detection.ComputeDerivativeYCentral(I_yy, I_y);
  /*Determinant of Hessian*/
  const std::size_t height = input.size();
  const std::size_t width = input[0].size();
  float min = std::numeric_limits<float>::max();
  float max = std::numeric_limits<float>::min();
  cornerness.resize(height);
  for (std::size_t i = 0; i < height; i++) {
    cornerness[i].resize(width);
    for (std::size_t j = 0; j < width; j++) {
      cornerness[i][j] =
          sigma * sigma * (I_xx[i][j] + I_yy[i][j] - I_xy[i][j] * I_xy[i][j]);
      if (SCALED) {
        if (cornerness[i][j] < min)
          min = cornerness[i][j];
        if (cornerness[i][j] > max)
          max = cornerness[i][j];
      }
    }

    /*local maxima*/
    output.resize(height);
    output[0].resize(width);
    output[height - 1].resize(width);
    for (std::size_t i = 1; i < height - 1; i++) {
      output[i].resize(width);
      for (std::size_t j = 1; j < width - 1; j++) {
        if (SCALED)
          output[i][j] = ((cornerness[i][j] - min) / (max - min));
        else
          output[i][j] = cornerness[i][j];
      }
    }
  }
}

template <typename ImageType>
void
Keypoint<ImageType>::hessianBlob(ImageType& output,
                                 ImageType& input,
                                 const float start_scale,
                                 const float scaling_factor,
                                 const int num_scales)
{
  const std::size_t height = input.size();
  const std::size_t width = input[0].size();
  const int local_search_radius = 1;
  float scale = start_scale;
  std::vector<ImageType> cornerness;
  cornerness.resize(num_scales);
  for (int i = 0; i < num_scales; i++) {
    hessianBlob(cornerness[i], input, scale, false);
    scale *= scaling_factor;
  }
  for (std::size_t i = 0; i < height; i++) {
    for (std::size_t j = 0; j < width; j++) {
      float scale_max = std::numeric_limits<float>::min();
      /*default output in case of no blob at the current point is 0*/
      output[i][j] = 0;
      for (int k = 0; k < num_scales; k++) {
        /*check if the current point (k,i,j) is a maximum in the defined search radius*/
        bool non_max_flag = false;
        const float local_max = cornerness[k][i][j];
        for (int n = -local_search_radius; n <= local_search_radius; n++) {
          if (n + k < 0 || n + k >= num_scales)
            continue;
          for (int l = -local_search_radius; l <= local_search_radius; l++) {
            if (l + i < 0 || l + i >= height)
              continue;
            for (int m = -local_search_radius; m <= local_search_radius; m++) {
              if (m + j < 0 || m + j >= width)
                continue;
              if (cornerness[n + k][l + i][m + j] > local_max) {
                non_max_flag = true;
                break;
              }
            }
            if (non_max_flag)
              break;
          }
          if (non_max_flag)
            break;
        }
        /*if the current point is a point of local maximum, check if it is a maximum
         * point across scales*/
        if (!non_max_flag) {
          if (cornerness[k][i][j] > scale_max) {
            scale_max = cornerness[k][i][j];
            /*output indicates the scale at which the blob is found at the current
             * location in the image*/
            output[i][j] = start_scale * pow(scaling_factor, k);
          }
        }
      }
    }
  }
}

template <typename ImageType>
void
Keypoint<ImageType>::imageElementMultiply(ImageType& output,
                                          ImageType& input1,
                                          ImageType& input2)
{
  const std::size_t height = input1.size();
  const std::size_t width = input1[0].size();
  output.resize(height);
  for (std::size_t i = 0; i < height; i++) {
    output[i].resize(width);
    for (std::size_t j = 0; j < width; j++) {
      output[i][j] = input1[i][j] * input2[i][j];
    }
  }
}
} // namespace pcl
