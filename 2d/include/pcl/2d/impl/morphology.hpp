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

#pragma once

#include <pcl/2d/morphology.h>

namespace pcl {

// Assumes input, kernel and output images have 0's and 1's only
template <typename PointT>
void
Morphology<PointT>::erosionBinary(pcl::PointCloud<PointT>& output)
{
  const int height = input_->height;
  const int width = input_->width;
  const int kernel_height = structuring_element_->height;
  const int kernel_width = structuring_element_->width;
  bool mismatch_flag;

  output.width = width;
  output.height = height;
  output.resize(width * height);

  for (int i = 0; i < height; i++) {
    for (int j = 0; j < width; j++) {
      // Operation done only at 1's
      if ((*input_)(j, i).intensity == 0) {
        output(j, i).intensity = 0;
        continue;
      }
      mismatch_flag = false;
      for (int k = 0; k < kernel_height; k++) {
        if (mismatch_flag)
          break;
        for (int l = 0; l < kernel_width; l++) {
          // We only check for 1's in the kernel
          if ((*structuring_element_)(l, k).intensity == 0)
            continue;
          if ((i + k - kernel_height / 2) < 0 ||
              (i + k - kernel_height / 2) >= height || (j + l - kernel_width / 2) < 0 ||
              (j + l - kernel_width / 2) >= width) {
            continue;
          }
          // If one of the elements of the kernel and image don't match,
          // the output image is 0. So, move to the next point.
          if ((*input_)(j + l - kernel_width / 2, i + k - kernel_height / 2)
                  .intensity != 1) {
            output(j, i).intensity = 0;
            mismatch_flag = true;
            break;
          }
        }
      }
      // Assign value according to mismatch flag
      output(j, i).intensity = (mismatch_flag) ? 0 : 1;
    }
  }
}

// Assumes input, kernel and output images have 0's and 1's only
template <typename PointT>
void
Morphology<PointT>::dilationBinary(pcl::PointCloud<PointT>& output)
{
  const int height = input_->height;
  const int width = input_->width;
  const int kernel_height = structuring_element_->height;
  const int kernel_width = structuring_element_->width;
  bool match_flag;

  output.width = width;
  output.height = height;
  output.resize(width * height);

  for (int i = 0; i < height; i++) {
    for (int j = 0; j < width; j++) {
      match_flag = false;
      for (int k = 0; k < kernel_height; k++) {
        if (match_flag)
          break;
        for (int l = 0; l < kernel_width; l++) {
          // We only check for 1's in the kernel
          if ((*structuring_element_)(l, k).intensity == 0)
            continue;
          if ((i + k - kernel_height / 2) < 0 ||
              (i + k - kernel_height / 2) >= height || (j + l - kernel_width / 2) < 0 ||
              (j + l - kernel_width / 2) >= height) {
            continue;
          }
          // If any position where kernel is 1 and image is also one is detected,
          // matching occurs
          if ((*input_)(j + l - kernel_width / 2, i + k - kernel_height / 2)
                  .intensity == 1) {
            match_flag = true;
            break;
          }
        }
      }
      // Assign value according to match flag
      output(j, i).intensity = (match_flag) ? 1 : 0;
    }
  }
}

// Assumes input, kernel and output images have 0's and 1's only
template <typename PointT>
void
Morphology<PointT>::openingBinary(pcl::PointCloud<PointT>& output)
{
  PointCloudInPtr intermediate_output(new PointCloudIn);
  erosionBinary(*intermediate_output);
  this->setInputCloud(intermediate_output);
  dilationBinary(output);
}

// Assumes input, kernel and output images have 0's and 1's only
template <typename PointT>
void
Morphology<PointT>::closingBinary(pcl::PointCloud<PointT>& output)
{
  PointCloudInPtr intermediate_output(new PointCloudIn);
  dilationBinary(*intermediate_output);
  this->setInputCloud(intermediate_output);
  erosionBinary(output);
}

template <typename PointT>
void
Morphology<PointT>::erosionGray(pcl::PointCloud<PointT>& output)
{
  const int height = input_->height;
  const int width = input_->width;
  const int kernel_height = structuring_element_->height;
  const int kernel_width = structuring_element_->width;
  float min;
  output.resize(width * height);
  output.width = width;
  output.height = height;

  for (int i = 0; i < height; i++) {
    for (int j = 0; j < width; j++) {
      min = -1;
      for (int k = 0; k < kernel_height; k++) {
        for (int l = 0; l < kernel_width; l++) {
          // We only check for 1's in the kernel
          if ((*structuring_element_)(l, k).intensity == 0)
            continue;
          if ((i + k - kernel_height / 2) < 0 ||
              (i + k - kernel_height / 2) >= height || (j + l - kernel_width / 2) < 0 ||
              (j + l - kernel_width / 2) >= width) {
            continue;
          }
          // If one of the elements of the kernel and image don't match,
          // the output image is 0. So, move to the next point.
          if ((*input_)(j + l - kernel_width / 2, i + k - kernel_height / 2).intensity <
                  min ||
              min == -1) {
            min = (*input_)(j + l - kernel_width / 2, i + k - kernel_height / 2)
                      .intensity;
          }
        }
      }
      // Assign value according to mismatch flag
      output(j, i).intensity = min;
    }
  }
}

template <typename PointT>
void
Morphology<PointT>::dilationGray(pcl::PointCloud<PointT>& output)
{
  const int height = input_->height;
  const int width = input_->width;
  const int kernel_height = structuring_element_->height;
  const int kernel_width = structuring_element_->width;
  float max;

  output.resize(width * height);
  output.width = width;
  output.height = height;

  for (int i = 0; i < height; i++) {
    for (int j = 0; j < width; j++) {
      max = -1;
      for (int k = 0; k < kernel_height; k++) {
        for (int l = 0; l < kernel_width; l++) {
          // We only check for 1's in the kernel
          if ((*structuring_element_)(l, k).intensity == 0)
            continue;
          if ((i + k - kernel_height / 2) < 0 ||
              (i + k - kernel_height / 2) >= height || (j + l - kernel_width / 2) < 0 ||
              (j + l - kernel_width / 2) >= width) {
            continue;
          }
          // If one of the elements of the kernel and image don't match,
          // the output image is 0. So, move to the next point.
          if ((*input_)(j + l - kernel_width / 2, i + k - kernel_height / 2).intensity >
                  max ||
              max == -1) {
            max = (*input_)(j + l - kernel_width / 2, i + k - kernel_height / 2)
                      .intensity;
          }
        }
      }
      // Assign value according to mismatch flag
      output(j, i).intensity = max;
    }
  }
}

template <typename PointT>
void
Morphology<PointT>::openingGray(pcl::PointCloud<PointT>& output)
{
  PointCloudInPtr intermediate_output(new PointCloudIn);
  erosionGray(*intermediate_output);
  this->setInputCloud(intermediate_output);
  dilationGray(output);
}

template <typename PointT>
void
Morphology<PointT>::closingGray(pcl::PointCloud<PointT>& output)
{
  PointCloudInPtr intermediate_output(new PointCloudIn);
  dilationGray(*intermediate_output);
  this->setInputCloud(intermediate_output);
  erosionGray(output);
}

template <typename PointT>
void
Morphology<PointT>::subtractionBinary(pcl::PointCloud<PointT>& output,
                                      const pcl::PointCloud<PointT>& input1,
                                      const pcl::PointCloud<PointT>& input2)
{
  const int height = (input1.height < input2.height) ? input1.height : input2.height;
  const int width = (input1.width < input2.width) ? input1.width : input2.width;
  output.width = width;
  output.height = height;
  output.resize(height * width);

  for (std::size_t i = 0; i < output.size(); ++i) {
    if (input1[i].intensity == 1 && input2[i].intensity == 0)
      output[i].intensity = 1;
    else
      output[i].intensity = 0;
  }
}

template <typename PointT>
void
Morphology<PointT>::unionBinary(pcl::PointCloud<PointT>& output,
                                const pcl::PointCloud<PointT>& input1,
                                const pcl::PointCloud<PointT>& input2)
{
  const int height = (input1.height < input2.height) ? input1.height : input2.height;
  const int width = (input1.width < input2.width) ? input1.width : input2.width;
  output.width = width;
  output.height = height;
  output.resize(height * width);

  for (std::size_t i = 0; i < output.size(); ++i) {
    if (input1[i].intensity == 1 || input2[i].intensity == 1)
      output[i].intensity = 1;
    else
      output[i].intensity = 0;
  }
}

template <typename PointT>
void
Morphology<PointT>::intersectionBinary(pcl::PointCloud<PointT>& output,
                                       const pcl::PointCloud<PointT>& input1,
                                       const pcl::PointCloud<PointT>& input2)
{
  const int height = (input1.height < input2.height) ? input1.height : input2.height;
  const int width = (input1.width < input2.width) ? input1.width : input2.width;
  output.width = width;
  output.height = height;
  output.resize(height * width);

  for (std::size_t i = 0; i < output.size(); ++i) {
    if (input1[i].intensity == 1 && input2[i].intensity == 1)
      output[i].intensity = 1;
    else
      output[i].intensity = 0;
  }
}

template <typename PointT>
void
Morphology<PointT>::structuringElementCircular(pcl::PointCloud<PointT>& kernel,
                                               const int radius)
{
  const int dim = 2 * radius;
  kernel.height = dim;
  kernel.width = dim;
  kernel.resize(dim * dim);

  for (int i = 0; i < dim; i++) {
    for (int j = 0; j < dim; j++) {
      if (((i - radius) * (i - radius) + (j - radius) * (j - radius)) < radius * radius)
        kernel(j, i).intensity = 1;
      else
        kernel(j, i).intensity = 0;
    }
  }
}

template <typename PointT>
void
Morphology<PointT>::structuringElementRectangle(pcl::PointCloud<PointT>& kernel,
                                                const int height,
                                                const int width)
{
  kernel.height = height;
  kernel.width = width;
  kernel.resize(height * width);
  for (std::size_t i = 0; i < kernel.size(); ++i)
    kernel[i].intensity = 1;
}

template <typename PointT>
void
Morphology<PointT>::setStructuringElement(const PointCloudInPtr& structuring_element)
{
  structuring_element_ = structuring_element;
}

} // namespace pcl
