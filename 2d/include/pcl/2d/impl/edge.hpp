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
 * $Id: edge.hpp nsomani $
 *
 */

#include "../convolution_2d.h"
#ifndef PCL_2D_EDGE_IMPL_HPP
#define PCL_2D_EDGE_IMPL_HPP


void
pcl::pcl_2d::edge::sobelXY  (ImageType &Gx, ImageType &Gy, ImageType &input)
{
  ImageType kernelX;
  kernelX.resize (3); kernelX[0].resize (3); kernelX[1].resize (3); kernelX[2].resize (3);
  kernelX[0][0] = -1; kernelX[0][1] = 0; kernelX[0][2] = 1;
  kernelX[1][0] = -2; kernelX[1][1] = 0; kernelX[1][2] = 2;
  kernelX[2][0] = -1; kernelX[2][1] = 0; kernelX[2][2] = 1;

  conv_2d->convolve  (Gx, kernelX, input);

  ImageType kernelY;
  kernelY.resize (3); kernelY[0].resize (3); kernelY[1].resize (3); kernelY[2].resize (3);
  kernelY[0][0] = 1; kernelY[0][1] = 2; kernelY[0][2] = 1;
  kernelY[1][0] = 0; kernelY[1][1] = 0; kernelY[1][2] = 0;
  kernelY[2][0] = -1; kernelY[2][1] = -2; kernelY[2][2] = -1;
  conv_2d->convolve  (Gy, kernelY, input);
}
void
pcl::pcl_2d::edge::sobelMagnitudeDirection  (ImageType &G, ImageType &thet, ImageType &input)
{
  ImageType Gx;
  ImageType Gy;
  sobelXY (Gx, Gy, input);
  G.resize (input.size ());
  thet.resize (input.size ());
  for (unsigned int i = 0; i < input.size (); i++)
  {
    G[i].resize (input[i].size ());
    thet[i].resize (input[i].size ());
    for (unsigned int j = 0; j < input[i].size (); j++)
    {
      G[i][j] = sqrt  (Gx[i][j] * Gx[i][j] + Gy[i][j] * Gy[i][j]);
      thet[i][j] = atan2  (Gy[i][j], Gx[i][j]);
    }
  }
}

void
pcl::pcl_2d::edge::prewittXY  (ImageType &Gx, ImageType &Gy, ImageType &input)
{
  ImageType kernelX;
  kernelX.resize (3); kernelX[0].resize (3); kernelX[1].resize (3); kernelX[2].resize (3);
  kernelX[0][0] = -1; kernelX[0][1] = 0; kernelX[0][2] = 1;
  kernelX[1][0] = -1; kernelX[1][1] = 0; kernelX[1][2] = 1;
  kernelX[2][0] = -1; kernelX[2][1] = 0; kernelX[2][2] = 1;
  conv_2d->convolve (Gx, kernelX, input);

  ImageType kernelY;
  kernelY.resize (3); kernelY[0].resize (3); kernelY[1].resize (3); kernelY[2].resize (3);
  kernelY[0][0] = 1; kernelY[0][1] = 1; kernelY[0][2] = 1;
  kernelY[1][0] = 0; kernelY[1][1] = 0; kernelY[1][2] = 0;
  kernelY[2][0] = -1; kernelY[2][1] = -1; kernelY[2][2] = -1;
  conv_2d->convolve (Gy, kernelY, input);
}
void
pcl::pcl_2d::edge::prewittMagnitudeDirection  (ImageType &G, ImageType &thet, ImageType &input)
{
  ImageType Gx;
  ImageType Gy;
  prewittXY (Gx, Gy, input);
  G.resize (input.size ());
  thet.resize (input.size ());
  for (int i = 0; i < input.size (); i++)
  {
    G[i].resize (input[i].size ());
    thet[i].resize (input[i].size ());
    for (int j = 0; j < input[i].size (); j++)
    {
      G[i][j] = sqrt (Gx[i][j] * Gx[i][j] + Gy[i][j] * Gy[i][j]);
      thet[i][j] = atan2 (Gy[i][j], Gx[i][j]);
    }
  }
}

void
pcl::pcl_2d::edge::robertsXY  (ImageType &Gx, ImageType &Gy, ImageType &input)
{
  ImageType kernelX;
  kernelX.resize (2); kernelX[0].resize (2); kernelX[1].resize (2);
  kernelX[0][0] = 1; kernelX[0][1] = 0;
  kernelX[1][0] = 0; kernelX[1][1] = -1;
  conv_2d->convolve (Gx, kernelX, input);

  ImageType kernelY;
  kernelY.resize (2); kernelY[0].resize (2); kernelY[1].resize (2);
  kernelY[0][0] = 0; kernelY[0][1] = 1;
  kernelY[1][0] = -1; kernelY[1][1] = 0;
  conv_2d->convolve (Gy, kernelY, input);
}
void
pcl::pcl_2d::edge::robertsMagnitudeDirection  (ImageType &G, ImageType &thet, ImageType &input){
  ImageType Gx;
  ImageType Gy;
  robertsXY (Gx, Gy, input);
  G.resize (input.size ());
  thet.resize (input.size ());
  for (int i = 0; i < input.size (); i++)
  {
    G[i].resize (input[i].size ());
    thet[i].resize (input[i].size ());
    for (int j = 0; j < input[i].size (); j++)
    {
      G[i][j] = sqrt (Gx[i][j] * Gx[i][j] + Gy[i][j] * Gy[i][j]);
      thet[i][j] = atan2 (Gy[i][j], Gx[i][j]);
    }
  }
}

void
pcl::pcl_2d::edge::cannyTraceEdge  (int rowOffset, int colOffset, int row, int col, float theta, float tLow, float tHigh, ImageType &G, ImageType &thet){
  int newRow = row + rowOffset;
  int newCol = col + colOffset;
  if (newRow > 0 && newRow < G.size () && newCol > 0 && newCol < G[0].size ())
  {
    if (G[newRow][newCol] < tLow || G[newRow][newCol] > tHigh || thet[newRow][newCol] != theta)
      return;
    G[newRow][newCol] = tHigh + 1;
    cannyTraceEdge  (rowOffset,colOffset, newRow, newCol, theta, tLow, tHigh, G, thet);
  }
}
void
pcl::pcl_2d::edge::canny  (ImageType &output, ImageType &input, float t_low, float t_high)
{
  float tHigh = t_high;
  float tLow = t_low;
  const int height = input.size();
  const int width = input[0].size();
  /*noise reduction using gaussian blurring*/
  ImageType gaussian_kernel;
  conv_2d->gaussianKernel  (5, 1.4, gaussian_kernel);
  conv_2d->convolve  (output, gaussian_kernel, input);

  /*edge detection usign Sobel*/
  ImageType G;
  ImageType thet;
  sobelMagnitudeDirection  (G, thet, input);

  /*edge discretization*/
  float angle;
  for (int i = 0; i < height; i++)
  {
    for (int j = 0; j < width; j++)
    {
      angle = (thet[i][j] / 3.14f) * 180;
      if (((angle < 22.5) && (angle > -22.5)) || (angle > 157.5) || (angle < -157.5))
        thet[i][j] = 0;
      else
        if (((angle > 22.5) && (angle < 67.5)) || ((angle < -112.5) && (angle > -157.5)))
          thet[i][j] = 45;
        else
          if (((angle > 67.5) && (angle < 112.5)) || ((angle < -67.5) && (angle > -112.5)))
            thet[i][j] = 90;
          else
            if (((angle > 112.5) && (angle < 157.5)) || ((angle < -22.5) && (angle > -67.5)))
              thet[i][j] = 135;
    }
  }

  float max;
  /*tHigh and non-maximal supression*/
  for (int i = 1; i < height - 1; i++)
  {
    for (int j = 1; j < width - 1; j++)
    {
      if (G[i][j] < tHigh)
        continue;
      max = G[i][j];
      switch ((int)thet[i][j])
      {
        case 0:
          if(G[i][j] < G[i][j-1] || G[i][j] < G[i][j+1])
            G[i][j] = 0;
          break;
        case 45:
          if(G[i][j] < G[i-1][j+1] || G[i][j] < G[i+1][j-1])
            G[i][j] = 0;
          break;
        case 90:
          if(G[i][j] < G[i-1][j] || G[i][j] < G[i+1][j])
            G[i][j] = 0;
          break;
        case 135:
          if(G[i][j] < G[i-1][j-1] || G[i][j] < G[i+1][j+1])
            G[i][j] = 0;
          break;
      }
    }
  }

  /*edge tracing*/
  for (int i = 0; i < height; i++)
  {
    for (int j = 0; j < width; j++)
    {
      if (G[i][j] < tHigh)
        continue;
      switch ((int)thet[i][j])
      {
        case 0:
          cannyTraceEdge  (1, 0, i, j, 0, tLow, tHigh, G, thet);
          cannyTraceEdge  (-1, 0, i, j, 0, tLow, tHigh, G, thet);
          break;
        case 45:
          cannyTraceEdge  (1, 1, i, j, 45, tLow, tHigh, G, thet);
          cannyTraceEdge  (-1, -1, i, j, 45, tLow, tHigh, G, thet);
          break;
        case 90:
          cannyTraceEdge  (0, -1, i, j, 90, tLow, tHigh, G, thet);
          cannyTraceEdge  (0, 1, i, j, 90, tLow, tHigh, G, thet);
          break;
        case 135:
          cannyTraceEdge  (-1, 1, i, j, 135, tLow, tHigh, G, thet);
          cannyTraceEdge  (1, -1, i, j, 135, tLow, tHigh, G, thet);
          break;
      }
    }
  }

  /*final thresholding*/
  output.resize (height);
  for (int i = 0; i < height; i++)
  {
    output[i].resize (width);
    for (int j = 0; j < width; j++)
    {
      if (G[i][j] < tHigh)
        output[i][j] = 0;
      else
        output[i][j] = 255;
    }
  }
}
void
pcl::pcl_2d::edge::canny  (ImageType &output, ImageType &input)
{
  float tHigh = 50;
  float tLow = 20;
  const int height = input.size();
  const int width = input[0].size();
  /*noise reduction using gaussian blurring*/
  ImageType gaussian_kernel;
  conv_2d->gaussianKernel  (5, 1.4, gaussian_kernel);
  conv_2d->convolve  (output, gaussian_kernel, input);

  /*edge detection usign Sobel*/
  ImageType G;
  ImageType thet;
  sobelMagnitudeDirection  (G, thet, input);

  /*edge discretization*/
  float angle;
  for (int i = 0; i < height; i++)
  {
    for (int j = 0; j < width; j++)
    {
      angle = (thet[i][j] / 3.14f) * 180;
      if (((angle < 22.5) && (angle > -22.5)) || (angle > 157.5) || (angle < -157.5))
        thet[i][j] = 0;
      else
        if (((angle > 22.5) && (angle < 67.5)) || ((angle < -112.5) && (angle > -157.5)))
          thet[i][j] = 45;
        else
          if (((angle > 67.5) && (angle < 112.5)) || ((angle < -67.5) && (angle > -112.5)))
            thet[i][j] = 90;
          else
            if (((angle > 112.5) && (angle < 157.5)) || ((angle < -22.5) && (angle > -67.5)))
              thet[i][j] = 135;
    }
  }

  float max;
  /*tHigh and non-maximal supression*/
  for (int i = 1; i < height - 1; i++)
  {
    for (int j = 1; j < width - 1; j++)
    {
      if (G[i][j] < tHigh)
        continue;
      max = G[i][j];
      switch ((int)thet[i][j])
      {
        case 0:
          if(G[i][j] < G[i][j-1] || G[i][j] < G[i][j+1])
            G[i][j] = 0;
          break;
        case 45:
          if(G[i][j] < G[i-1][j+1] || G[i][j] < G[i+1][j-1])
            G[i][j] = 0;
          break;
        case 90:
          if(G[i][j] < G[i-1][j] || G[i][j] < G[i+1][j])
            G[i][j] = 0;
          break;
        case 135:
          if(G[i][j] < G[i-1][j-1] || G[i][j] < G[i+1][j+1])
            G[i][j] = 0;
          break;
      }
    }
  }

  /*edge tracing*/
  for (int i = 0; i < height; i++)
  {
    for (int j = 0; j < width; j++)
    {
      if (G[i][j] < tHigh)
        continue;
      switch ((int)thet[i][j])
      {
        case 0:
          cannyTraceEdge  (1, 0, i, j, 0, tLow, tHigh, G, thet);
          cannyTraceEdge  (-1, 0, i, j, 0, tLow, tHigh, G, thet);
          break;
        case 45:
          cannyTraceEdge  (1, 1, i, j, 45, tLow, tHigh, G, thet);
          cannyTraceEdge  (-1, -1, i, j, 45, tLow, tHigh, G, thet);
          break;
        case 90:
          cannyTraceEdge  (0, -1, i, j, 90, tLow, tHigh, G, thet);
          cannyTraceEdge  (0, 1, i, j, 90, tLow, tHigh, G, thet);
          break;
        case 135:
          cannyTraceEdge  (-1, 1, i, j, 135, tLow, tHigh, G, thet);
          cannyTraceEdge  (1, -1, i, j, 135, tLow, tHigh, G, thet);
          break;
      }
    }
  }

  /*final thresholding*/
  output.resize (height);
  for (int i = 0; i < height; i++)
  {
    output[i].resize (width);
    for (int j = 0; j < width; j++)
    {
      if (G[i][j] < tHigh)
        output[i][j] = 0;
      else
        output[i][j] = 255;
    }
  }
}

void
pcl::pcl_2d::edge::LoGKernel  (ImageType &kernel, const int kernel_size, const float sigma)
{
  float sum = 0;
  float temp = 0;
  kernel.resize (kernel_size);
  for (int i = 0; i < kernel_size; i++)
  {
    kernel[i].resize (kernel_size);
    for (int j = 0; j < kernel_size; j++)
    {
      temp = (((i - kernel_size / 2) * (i - kernel_size / 2) + (j - kernel_size / 2) * (j - kernel_size / 2)) / (2 * sigma * sigma));
      kernel[i][j] = (1 - temp) * exp (-temp);
      sum += kernel[i][j];
    }
  }
  for (int i = 0; i < kernel_size; i++)
  {
    for (int j = 0; j < kernel_size; j++)
    {
      kernel[i][j] /= sum;
    }
  }
}
void
pcl::pcl_2d::edge::LoG  (ImageType &output, const int kernel_size, const float sigma, ImageType &input)
{
  ImageType kernel;
  LoGKernel  (kernel, kernel_size, sigma);
  conv_2d->convolve  (output, kernel, input);
}
void
pcl::pcl_2d::edge::LoG  (ImageType &output, ImageType &input)
{
  ImageType kernel;
  LoGKernel  (kernel, 9, 1.4f);
  conv_2d->convolve  (output, kernel, input);
}

void
pcl::pcl_2d::edge::ComputeDerivativeXCentral (ImageType &output, ImageType &input){
  ImageType kernel;
  kernel.resize (1);
  kernel[0].resize (3);
  kernel[0][0] = -1; kernel[0][1] = 0; kernel[0][2] = 1;
  conv_2d->convolve  (output, kernel, input);
}
void
pcl::pcl_2d::edge::ComputeDerivativeYCentral (ImageType &output, ImageType &input){
  ImageType kernel;
  kernel.resize (3);
  kernel[0].resize (1); kernel[1].resize (1); kernel[2].resize (1);
  kernel[0][0] = -1; kernel[1][0] = 0; kernel[2][0] = 1;
  conv_2d->convolve  (output, kernel, input);
}

#endif
