/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, wwww.pointclouds.org
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
 * $Id$
 *
 */

#include <pcl/common/gaussian.h>

void 
pcl::GaussianKernel::compute (float sigma, 
                              Eigen::VectorXf &kernel, 
                              unsigned kernel_width) const
{
  assert (kernel_width %2 == 1);
  assert (sigma >= 0);
  kernel.resize (kernel_width);
  static const float factor = 0.01f;
  static const float max_gauss = 1.0f;
  const int hw = kernel_width / 2;
  float sigma_sqr = 1.0f / (2.0f * sigma * sigma);
  for (int i = -hw, j = 0, k = kernel_width - 1; i < 0 ; i++, j++, k--)
    kernel[k] = kernel[j] = expf (-static_cast<float>(i) * static_cast<float>(i) * sigma_sqr);
  kernel[hw] = 1;
  unsigned g_width = kernel_width;
  for (unsigned i = 0; fabs (kernel[i]/max_gauss) < factor; i++, g_width-= 2) ;
  if (g_width == kernel_width)
  { 
    PCL_THROW_EXCEPTION (pcl::KernelWidthTooSmallException,
                        "kernel width " << kernel_width 
                        << "is too small for the given sigma " << sigma);
    return;
  }

  // Shift and resize if width less than kernel_width
  unsigned shift = (kernel_width - g_width)/2;
  for (unsigned i =0; i < g_width; i++)
    kernel[i] = kernel[i + shift];
  kernel.conservativeResize (g_width);

  // Normalize
  kernel/= kernel.sum ();
}

void 
pcl::GaussianKernel::compute (float sigma, 
                              Eigen::VectorXf &kernel, 
                              Eigen::VectorXf &derivative,
                              unsigned kernel_width) const
{
  assert (kernel_width %2 == 1);
  assert (sigma >= 0);
  kernel.resize (kernel_width);
  derivative.resize (kernel_width);
  const float factor = 0.01f;
  float max_gauss = 1.0f, max_deriv = float (sigma * exp (-0.5));
  int hw = kernel_width / 2;

  float sigma_sqr = 1.0f / (2.0f * sigma * sigma);
  for (int i = -hw, j = 0, k = kernel_width - 1; i < 0 ; i++, j++, k--)
  {
    kernel[k] = kernel[j] = expf (-static_cast<float>(i) * static_cast<float>(i) * sigma_sqr);
    derivative[j] = -static_cast<float>(i) * kernel[j];
    derivative[k] = -derivative[j];
  }
  kernel[hw] = 1;
  derivative[hw] = 0;
  // Compute kernel and derivative true width
  unsigned g_width = kernel_width;
  unsigned d_width = kernel_width;
  for (unsigned i = 0; fabs (derivative[i]/max_deriv) < factor; i++, d_width-= 2) ;
  for (unsigned i = 0; fabs (kernel[i]/max_gauss) < factor; i++, g_width-= 2) ;
  if (g_width == kernel_width || d_width == kernel_width)
  { 
    PCL_THROW_EXCEPTION (KernelWidthTooSmallException,
                        "kernel width " << kernel_width 
                        << "is too small for the given sigma " << sigma);
    return;
  }

  // Shift and resize if width less than kernel_width
  // Kernel
  unsigned shift = (kernel_width - g_width)/2;
  for (unsigned i =0; i < g_width; i++)
    kernel[i] = kernel[i + shift];
  // Normalize kernel
  kernel.conservativeResize (g_width);
  kernel/= kernel.sum ();

  // Derivative
  shift = (kernel_width - d_width)/2;
  for (unsigned i =0; i < d_width; i++)
    derivative[i] = derivative[i + shift];
  derivative.conservativeResize (d_width);
  // Normalize derivative
  hw = d_width / 2;
  float den = 0;
  for (int i = -hw ; i <= hw ; i++)
    den -=  static_cast<float>(i) * derivative[i+hw];
  derivative/= den;
}

void 
pcl::GaussianKernel::convolveRows (const pcl::PointCloud<float>& input,
                                   const Eigen::VectorXf& kernel,
                                   pcl::PointCloud<float>& output) const
{
  assert (kernel.size () % 2 == 1);
  size_t kernel_width = kernel.size () -1;
  size_t radius = kernel.size () / 2;
  const pcl::PointCloud<float>* input_;
  if (&input != &output)
  {
    if (output.height < input.height || output.width < input.width)
    {
      output.width = input.width;
      output.height = input.height;
      output.points.resize (input.height * input.width);
    }
    input_ = &input;
  }
  else
    input_ = new pcl::PointCloud<float>(input);
  
  size_t i;
  for (size_t j = 0; j < input_->height; j++)
  {
    for (i = 0 ; i < radius ; i++)
      output (i,j) = 0;

    for ( ; i < input_->width - radius ; i++)  
    {
      output (i,j) = 0;
      for (int k = static_cast<int>(kernel_width), l = static_cast<int>(i - radius); k >= 0 ; k--, l++)
        output (i,j) += (*input_) (l,j) * kernel[k];
    }

    for ( ; i < input_->width ; i++)
      output (i,j) = 0;
  }

  if (&input == &output)
  {
    delete input_;
  }
}

void 
pcl::GaussianKernel::convolveCols (const pcl::PointCloud<float>& input,
                                   const Eigen::VectorXf& kernel,
                                   pcl::PointCloud<float>& output) const
{
  assert (kernel.size () % 2 == 1);
  size_t kernel_width = kernel.size () -1;
  size_t radius = kernel.size () / 2;
  const pcl::PointCloud<float>* input_;
  if (&input != &output)
  {
    if (output.height < input.height || output.width < input.width)
    {
      output.width = input.width;
      output.height = input.height;
      output.resize (input.width * input.height);
    }
    input_ = &input;
  }
  else
    input_ = new pcl::PointCloud<float> (input);

  size_t j;
  for (size_t i = 0; i < input_->width; i++)
  {
    for (j = 0 ; j < radius ; j++)
      output (i,j) = 0;

    for ( ; j < input_->height - radius ; j++)  {
      output (i,j) = 0;
      for (int k = static_cast<int>(kernel_width), l = static_cast<int>(j - radius) ; k >= 0 ; k--, l++)
      {
        output (i,j) += (*input_) (i,l) * kernel[k];
      }
    }

    for ( ; j < input_->height ; j++)
      output (i,j) = 0;
  }
  if (&input == &output)
    delete input_;
}
