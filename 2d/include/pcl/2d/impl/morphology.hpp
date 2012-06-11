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
 *  morphology.hpp
 *
 *  Created on: May 28, 2012
 *      Author: somani
 */

#ifndef MORPHOLOGY_HPP_
#define MORPHOLOGY_HPP_

/*assumes input, kernel and output images have 0's and 1's only*/
void
pcl::pcl_2d::morphology::erosionBinary  (ImageType &output, ImageType &kernel, ImageType &input){
  const int height = input.size ();
  const int width = input[0].size ();
  const int kernel_height = kernel.size ();
  const int kernel_width = kernel[0].size ();
  bool mismatch_flag;

  output.resize (height);
  for (int i = 0; i < height; i++)
  {
    output[i].resize (width);
    for (int j = 0; j < width; j++)
    {
      /*operation done only at 1's*/
      if (input[i][j] == 0)
      {
        output[i][j] = 0;
        continue;
      }
      mismatch_flag = false;
      for (int k = 0; k < kernel_height; k++)
      {
        if (mismatch_flag)
          break;
        for (int l = 0; l < kernel_width; l++)
        {
          /*we only check for 1's in the kernel*/
          if (kernel[k][l] == 0)
            continue;
          if ((i + k - kernel_height / 2) < 0 || (i + k - kernel_height / 2) >= height || (j + l - kernel_width / 2) < 0 || (j + l - kernel_width / 2) >= width)
          {
            continue;
          }
          /* if one of the elements of the kernel and image dont match, the output image is 0.
           * So, move to the next point.*/
          if (input[i + k - kernel_height / 2][j + l - kernel_width / 2] != 1)
          {
            output[i][j] = 0;
            mismatch_flag = true;
            break;
          }
        }
      }
      /*assign value according to mismatch flag*/
      output[i][j] = (mismatch_flag) ? 0 : 1;
    }
  }
}

/*assumes input, kernel and output images have 0's and 1's only*/
void
pcl::pcl_2d::morphology::dilationBinary  (ImageType &output, ImageType &kernel, ImageType &input){
  const int height = input.size ();
  const int width = input[0].size ();
  const int kernel_height = kernel.size ();
  const int kernel_width = kernel[0].size ();
  bool match_flag;

  output.resize (height);
  for (int i = 0; i < height; i++)
  {
    output[i].resize (width);
    for (int j = 0; j < width; j++)
    {
      match_flag = false;
      for (int k = 0; k < kernel_height; k++)
      {
        if (match_flag)
          break;
        for (int l = 0; l < kernel_width; l++)
        {
          /*we only check for 1's in the kernel*/
          if (kernel[k][l] == 0)
            continue;
          if ((i + k - kernel_height / 2) < 0 || (i + k - kernel_height / 2) >= height || (j + l - kernel_width / 2) < 0 || (j + l - kernel_width / 2) >= height)
          {
            continue;
          }
          /*if any position where kernel is 1 and image is also one is detected, matching occurs*/
          if (input[i + k - kernel_height / 2][j + l - kernel_width / 2] == 1)
          {
            match_flag = true;
            break;
          }
        }
      }
      /*assign value according to match flag*/
      output[i][j] = (match_flag) ? 1 : 0;
    }
  }
}

/*assumes input, kernel and output images have 0's and 1's only*/
void
pcl::pcl_2d::morphology::openingBinary  (ImageType &output, ImageType &kernel, ImageType &input){
  ImageType intermediate_output;
  openingBinary  (intermediate_output, kernel, input);
  closingBinary  (output, kernel, intermediate_output);
}

/*assumes input, kernel and output images have 0's and 1's only*/
void
pcl::pcl_2d::morphology::closingBinary  (ImageType &output, ImageType &kernel, ImageType &input){
  ImageType intermediate_output;
  closingBinary  (intermediate_output, kernel, input);
  openingBinary  (output, kernel, intermediate_output);
}

void
pcl::pcl_2d::morphology::erosionGray  (ImageType &output, ImageType &kernel, ImageType &input){
  const int height = input.size ();
  const int width = input[0].size ();
  const int kernel_height = kernel.size ();
  const int kernel_width = kernel[0].size ();
  float min;

  output.resize (height);
  for (int i = 0; i < height; i++)
  {
    output[i].resize (width);
    for (int j = 0; j < width; j++)
    {
      min = -1;
      for (int k = 0; k < kernel_height; k++)
      {
        for (int l = 0; l < kernel_width; l++)
        {
          /*we only check for 1's in the kernel*/
          if (kernel[k][l] == 0)
            continue;
          if ((i + k - kernel_height / 2) < 0 || (i + k - kernel_height / 2) >= height || (j + l - kernel_width / 2) < 0 || (j + l - kernel_width / 2) >= width)
          {
            continue;
          }
          /* if one of the elements of the kernel and image dont match, the output image is 0.
           * So, move to the next point.*/
          if (input[i + k - kernel_height / 2][j + l - kernel_width / 2] < min || min == -1)
          {
            min = input[i + k - kernel_height / 2][j + l - kernel_width / 2];
          }
        }
      }
      /*assign value according to mismatch flag*/
      output[i][j] = min;
    }
  }
}

void
pcl::pcl_2d::morphology::dilationGray  (ImageType &output, ImageType &kernel, ImageType &input){
  const int height = input.size ();
  const int width = input[0].size ();
  const int kernel_height = kernel.size ();
  const int kernel_width = kernel[0].size ();
  float max;

  output.resize (height);
  for (int i = 0; i < height; i++)
  {
    output[i].resize (width);
    for (int j = 0; j < width; j++)
    {
      max = -1;
      for (int k = 0; k < kernel_height; k++)
      {
        for (int l = 0; l < kernel_width; l++)
        {
          /*we only check for 1's in the kernel*/
          if (kernel[k][l] == 0)
            continue;
          if ((i + k - kernel_height / 2) < 0 || (i + k - kernel_height / 2) >= height || (j + l - kernel_width / 2) < 0 || (j + l - kernel_width / 2) >= width)
          {
            continue;
          }
          /* if one of the elements of the kernel and image dont match, the output image is 0.
           * So, move to the next point.*/
          if (input[i + k - kernel_height / 2][j + l - kernel_width / 2] > max || max == -1)
          {
            max = input[i + k - kernel_height / 2][j + l - kernel_width / 2];
          }
        }
      }
      /*assign value according to mismatch flag*/
      output[i][j] = max;
    }
  }
}

void
pcl::pcl_2d::morphology::openingGray  (ImageType &output, ImageType &kernel, ImageType &input){
  ImageType intermediate_output;
  erosionGray  (intermediate_output, kernel, input);
  dilationGray  (output, kernel, intermediate_output);
}


void
pcl::pcl_2d::morphology::closingGray  (ImageType &output, ImageType &kernel, ImageType &input){
  ImageType intermediate_output;
  dilationGray  (intermediate_output, kernel, input);
  erosionGray  (output, kernel, intermediate_output);
}

void
pcl::pcl_2d::morphology::subtractionBinary  (ImageType &output, ImageType &input1, ImageType &input2){
  const int height = (input1.size () < input2.size ()) ? input1.size () : input2.size ();
  const int width = (input1[0].size () < input2[0].size ()) ? input1[0].size () : input2[0].size ();
  output.resize (height);
  for (int i = 0; i < height; i++)
  {
    output[i].resize (width);
    for (int j = 0; j < width; j++)
    {
      if (input1[i][j] == 1 && input2[i][j] == 0)
        output[i][j] = 1;
      else
        output[i][j] = 0;
    }
  }
}

void
pcl::pcl_2d::morphology::unionBinary  (ImageType &output, ImageType &input1, ImageType &input2){
  const int height = (input1.size () < input2.size ()) ? input1.size () : input2.size ();
  const int width = (input1[0].size () < input2[0].size ()) ? input1[0].size () : input2[0].size ();
  output.resize (height);
  for (int i = 0; i < height; i++)
  {
    output[i].resize (width);
    for (int j = 0; j < width; j++)
    {
      if (input1[i][j] == 1 || input2[i][j] == 1)
        output[i][j] = 1;
      else
        output[i][j] = 0;
    }
  }
}

void
pcl::pcl_2d::morphology::intersectionBinary  (ImageType &output, ImageType &input1, ImageType &input2){
  const int height = (input1.size () < input2.size ()) ? input1.size () : input2.size ();
  const int width = (input1[0].size () < input2[0].size ()) ? input1[0].size () : input2[0].size ();
  output.resize (height);
  for (int i = 0; i < height; i++)
  {
    output[i].resize (width);
    for (int j = 0; j < width; j++)
    {
      if (input1[i][j] == 1 && input2[i][j] == 1)
        output[i][j] = 1;
      else
        output[i][j] = 0;
    }
  }
}

void
pcl::pcl_2d::morphology::structuringElementCircular  (ImageType &kernel, const int radius){
  const int dim = 2 * radius;
  kernel.resize (dim);
  for (int i = 0; i < dim; i++)
  {
    kernel[i].resize (dim);
    for (int j = 0; j < dim; j++)
    {
      if (((i - radius) * (i - radius) + (j - radius) * (j - radius)) < radius * radius)
        kernel[i][j] = 1;
      else
        kernel[i][j] = 0;
    }
  }
}

void
pcl::pcl_2d::morphology::structuringElementRectangle  (ImageType &kernel, const int height, const int width){
  kernel.resize (height);
  for (int i = 0; i < height; i++)
  {
    kernel[i].resize (width);
    for (int j = 0; j < width; j++)
    {
      kernel[i][j] = 1;
    }
  }
}

#endif /* MORPHOLOGY_HPP_ */
