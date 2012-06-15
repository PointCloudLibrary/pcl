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
#include "../edge.h"
#include "../convolution_2d.h"
#include <vector>

#ifndef KEYPOINT_HPP_
#define KEYPOINT_HPP_

void
pcl::pcl_2d::keypoint::harris_corner  (ImageType &output, ImageType &input, const float sigma_d, const float sigma_i, const float alpha, const float thresh){

  /*creating the gaussian kernels*/
  ImageType kernel_d;
  ImageType kernel_i;
  conv_2d->gaussianKernel  (5, sigma_d, kernel_d);
  conv_2d->gaussianKernel  (5, sigma_i, kernel_i);

  /*scaling the image with differentiation scale*/
  ImageType smoothed_image;
  conv_2d->convolve  (smoothed_image, kernel_d, input);

  /*image derivatives*/
  ImageType I_x, I_y;
  edge_detection->ComputeDerivativeXCentral  (I_x, smoothed_image);
  edge_detection->ComputeDerivativeYCentral  (I_y, smoothed_image);

  /*second moment matrix*/
  ImageType I_x2, I_y2, I_xI_y;
  image_element_multiply  (I_x2, I_x, I_x);
  image_element_multiply  (I_y2, I_y, I_y);
  image_element_multiply  (I_xI_y, I_x, I_y);

  /*scaling second moment matrix with integration scale*/
  ImageType M00, M10, M11;
  conv_2d->convolve  (M00, kernel_i, I_x2);
  conv_2d->convolve  (M10, kernel_i, I_xI_y);
  conv_2d->convolve  (M11, kernel_i, I_y2);

  /*harris function*/
  const int height = input.size ();
  const int width = input[0].size ();
  output.resize (height);
  for (int i = 0; i < height; i++)
  {
    output[i].resize (width);
    for (int j = 0; j < width; j++)
    {
      output[i][j] = M00[i][j] * M11[i][j] - (M10[i][j] * M10[i][j]) - alpha * ((M00[i][j] + M11[i][j]) * (M00[i][j] + M11[i][j]));
      if (thresh != 0)
      {
        if (output[i][j] < thresh)
          output[i][j] = 0;
        else
          output[i][j] = 255;
      }
    }
  }

  /*local maxima*/
  for (int i = 1; i < height - 1; i++)
  {
    for (int j = 1; j < width - 1; j++)
    {
      if (output[i][j] > output[i - 1][j - 1] && output[i][j] > output[i - 1][j] && output[i][j] > output[i - 1][j + 1] &&
          output[i][j] > output[i][j - 1] && output[i][j] > output[i][j + 1] &&
          output[i][j] > output[i + 1][j - 1] && output[i][j] > output[i + 1][j] && output[i][j] > output[i + 1][j + 1])
        ;
      else
        output[i][j] = 0;
    }
  }

}

void
pcl::pcl_2d::keypoint::image_element_multiply  (ImageType &output, ImageType &input1, ImageType &input2){
  const int height = input1.size ();
  const int width = input1[0].size ();
  output.resize (height);
  for (int i = 0; i < height; i++)
  {
    output[i].resize (width);
    for (int j = 0; j < width; j++)
    {
      output[i][j] = input1[i][j] * input2[i][j];
    }
  }
}
#endif /* KEYPOINT_HPP_ */
