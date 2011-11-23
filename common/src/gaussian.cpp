/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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
 * Author: Nizar Sallem
 */

#include <pcl/common/gaussian.h>

void 
pcl::GaussianKernel::compute(double sigma, 
                             Eigen::VectorXd &kernel, 
                             unsigned kernel_width) const
{
  assert(kernel_width %2 == 1);
  assert(sigma >= 0);
  kernel.resize (kernel_width);
  static const double factor = 0.01f;
  static const double max_gauss = 1.0f;
  const int hw = kernel_width / 2;
  for(int i = -hw, j = 0, k = kernel_width - 1; i < 0 ; i++, j++, k--)
    kernel[k] = kernel[j] = double (exp (-i*i / (2*sigma*sigma)));
  kernel[hw] = 1;
  unsigned g_width = kernel_width;
  for(unsigned i = 0; fabs (kernel[i]/max_gauss) < factor; i++, g_width-= 2);
  if(g_width == kernel_width)
  { 
    PCL_THROW_EXCEPTION(KernelWidthTooSmallException,
                        "kernel width " << kernel_width 
                        << "is too small for the given sigma " << sigma);
    return;
  }

  // Shift and resize if width less than kernel_width
  unsigned shift = (kernel_width - g_width)/2;
  for(unsigned i =0; i < g_width; i++)
    kernel[i] = kernel[i + shift];
  kernel.conservativeResize (g_width);

  // Normalize
  kernel/= kernel.sum();
}

void 
pcl::GaussianKernel::compute(double sigma, 
                             Eigen::VectorXd &kernel, 
                             Eigen::VectorXd &derivative,
                             unsigned kernel_width) const
{
  assert(kernel_width %2 == 1);
  assert(sigma >= 0);
  kernel.resize (kernel_width);
  derivative.resize (kernel_width);
  const double factor = 0.01f;
  double max_gauss = 1.0f, max_deriv = double(sigma * exp(-0.5));
  int hw = kernel_width / 2;
  for(int i = -hw, j = 0, k = kernel_width - 1; i < 0 ; i++, j++, k--)
  {
    kernel[k] = kernel[j] = double (exp (-i*i / (2*sigma*sigma)));
    derivative[j] = -i * kernel[j];
    derivative[k] = -derivative[j];
  }
  kernel[hw] = 1;
  derivative[hw] = 0;
  // Compute kernel and derivative true width
  unsigned g_width = kernel_width;
  unsigned d_width = kernel_width;
  for(unsigned i = 0; fabs (derivative[i]/max_deriv) < factor; i++, d_width-= 2);
  for(unsigned i = 0; fabs (kernel[i]/max_gauss) < factor; i++, g_width-= 2);
  if(g_width == kernel_width || d_width == kernel_width)
  { 
    PCL_THROW_EXCEPTION(KernelWidthTooSmallException,
                        "kernel width " << kernel_width 
                        << "is too small for the given sigma " << sigma);
    return;
  }

  // Shift and resize if width less than kernel_width
  // Kernel
  unsigned shift = (kernel_width - g_width)/2;
  for(unsigned i =0; i < g_width; i++)
    kernel[i] = kernel[i + shift];
  // Normalize kernel
  kernel.conservativeResize (g_width);
  kernel/= kernel.sum ();

  // Derivative
  shift = (kernel_width - d_width)/2;
  for(unsigned i =0; i < d_width; i++)
    derivative[i] = derivative[i + shift];
  derivative.conservativeResize (d_width);
  // Normalize derivative
  hw = d_width / 2;
  double den = 0;
  for (int i = -hw ; i <= hw ; i++)
    den -=  i*derivative[i+hw];
  derivative/= den;
}

void 
pcl::GaussianKernel::convolveRows(const Eigen::MatrixXd& input,
                                  const Eigen::VectorXd& kernel,
                                  Eigen::MatrixXd& output) const
{
  assert(kernel.size () % 2 == 1);
  int kernel_width = kernel.size () -1;
  int radius = kernel.size () / 2.0;
  const Eigen::MatrixXd* input_;
  if(input.data () != output.data ())
  {
    if(output.rows () < input.rows () || output.cols () < input.cols ())
      output.resize (input.rows (), input.cols ());
    input_ = &input;
  }
  else
    input_ = new Eigen::MatrixXd(input);
  
  int i;
  for(int j = 0; j < input_->rows (); j++)
  {
    for (i = 0 ; i < radius ; i++)
      output(j,i) = 0;

    for ( ; i < input_->cols () - radius ; i++)  {
      output(j,i) = 0;
      for (int k = kernel_width, l = i - radius; k >= 0 ; k--, l++)
        output(j,i) += (*input_)(j,l) * kernel[k];
    }

    for ( ; i < input_->cols () ; i++)
      output(j,i) = 0;
  }
}

void 
pcl::GaussianKernel::convolveCols(const Eigen::MatrixXd& input,
                                  const Eigen::VectorXd& kernel,
                                  Eigen::MatrixXd& output) const
{
  assert(kernel.size () % 2 == 1);
  int kernel_width = kernel.size () -1;
  int radius = kernel.size () / 2.0;
  const Eigen::MatrixXd* input_;
  if(input.data () != output.data ())
  {
    if(output.rows () < input.rows () || output.cols () < input.cols ())
      output.resize (input.rows (), input.cols ());
    input_ = &input;
  }
  else
    input_ = new Eigen::MatrixXd(input);

  int j;
  for(int i = 0; i < input_->cols (); i++)
  {
    for (j = 0 ; j < radius ; j++)
      output(j,i) = 0;

    for ( ; j < input_->rows () - radius ; j++)  {
      output(j,i) = 0;
      for (int k = kernel_width, l = j - radius ; k >= 0 ; k--, l++)
      {
        output(j,i) += (*input_)(l,i) * kernel[k];
      }
    }

    for ( ; j < input_->rows () ; j++)
      output(j,i) = 0;
  }
}

void 
pcl::GaussianKernel::convolve(const Eigen::MatrixXd& input,
                              const Eigen::VectorXd& horiz_kernel,
                              const Eigen::VectorXd& vert_kernel,
                              Eigen::MatrixXd& output) const
{
  Eigen::MatrixXd *tmp = new Eigen::MatrixXd(input.rows (), input.cols ()) ;
  convolveRows(input, horiz_kernel, *tmp);
  convolveCols(*tmp, vert_kernel, output);
  delete tmp;
}

void 
pcl::GaussianKernel::computeGradients(const Eigen::MatrixXd& input,
                                      const Eigen::VectorXd& gaussian_kernel,
                                      const Eigen::VectorXd& gaussian_kernel_derivative,
                                      Eigen::MatrixXd& grad_x,                         
                                      Eigen::MatrixXd& grad_y) const
{
  // if(grad_x.rows () < input.rows () || grad_x.cols () < input.cols ())
  //   grad_x.resize (input.rows (), input.cols ());

  // if(grad_y.rows () < input.rows () || grad_y.cols () < input.cols ())
  //   grad_y.resize (input.rows (), input.cols ());

  convolve(input, gaussian_kernel_derivative, gaussian_kernel, grad_x);
  convolve(input, gaussian_kernel, gaussian_kernel_derivative, grad_y);
}

void 
pcl::GaussianKernel::smooth(const Eigen::MatrixXd& input,
                            const Eigen::VectorXd& gaussian_kernel,
                            Eigen::MatrixXd& output) const
{
  // if(output.rows () < input.rows () || output.cols () < input.cols ())
  //   output.resize (input.rows (), input.cols ());

  convolve(input, gaussian_kernel, gaussian_kernel, output);
}
