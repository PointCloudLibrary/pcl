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

#ifndef PCL_GAUSSIAN_KERNEL
#define PCL_GAUSSIAN_KERNEL

#include <sstream>
#include <Eigen/Core>

namespace pcl
{
    /** Class GaussianKernel assembles all the method for computing, 
      * convolving, smoothing, gradients computing an image using
      * a gaussian kernel.
      * \ingroup common
      */
    class GaussianKernel
    {
      public:
      static const unsigned MAX_KERNEL_WIDTH = 71;
      /** Computes the gaussian kernel and dervative assiociated to sigma.
        * The kernel and derivative width are adjusted according.
        * \param[IN] sigma
        * \param[OUT] kernel the computed gaussian kernel
        * \param[IN] kernel_width the desired kernel width upper bond
        * \throws pcl::KernelWidthTooSmallException
        */
      void
      compute(double sigma, 
              Eigen::VectorXd& kernel,
              unsigned kernel_width = MAX_KERNEL_WIDTH) const;

      /** Computes the gaussian kernel and dervative assiociated to sigma.
        * The kernel and derivative width are adjusted according.
        * \param[IN] sigma
        * \param[OUT] kernel the computed gaussian kernel
        * \param[OUT] derivative the computed kernel derivative
        * \param[IN] kernel_width the desired kernel width upper bond
        * \throws pcl::KernelWidthTooSmallException
        */
      void
      compute(double sigma, 
              Eigen::VectorXd& kernel, 
              Eigen::VectorXd& derivative, 
              unsigned kernel_width = MAX_KERNEL_WIDTH) const;

      /** Convolve a double image rows by a given kernel.
        * \param[IN] kernel convolution kernel
        * \param[IN] input the image to convolve
        * \param[OUT] output the convolved image
        * \note if output doesn't fit in input i.e. output.rows() < input.rows() or
        * output.cols() < input.cols() then output is resized to input sizes.
        */
      void
      convolveRows(const Eigen::MatrixXd& input,
                   const Eigen::VectorXd& kernel,
                   Eigen::MatrixXd& output) const;

      /** Convolve a double image columns by a given kernel.
        * \param[IN] kernel convolution kernel
        * \param[IN] input the image to convolve
        * \param[OUT] output the convolved image
        * \note if output doesn't fit in input i.e. output.rows() < input.rows() or
        * output.cols() < input.cols() then output is resized to input sizes.
        */
      void
      convolveCols(const Eigen::MatrixXd& input,
                   const Eigen::VectorXd& kernel,
                   Eigen::MatrixXd& output) const;

      /** Convolve a double image in the 2 directions
        * \param[IN] horiz_kernel kernel for convolving rows
        * \param[IN] vert_kernel kernel for convolving columns
        * \param[IN] input image to convolve
        * \param[OUT] output the convolved image
        * \note if output doesn't fit in input i.e. output.rows() < input.rows() or
        * output.cols() < input.cols() then output is resized to input sizes.
        */
      void
      convolve(const Eigen::MatrixXd& input,
               const Eigen::VectorXd& horiz_kernel,
               const Eigen::VectorXd& vert_kernel,
               Eigen::MatrixXd& output) const;
      
      /** Computes double image gradients using a gaussian kernel and gaussian kernel
        * derivative.
        * \param[IN] input image to compute gardients for
        * \param[IN] gaussian_kernel the gaussian kernel to be used
        * \param[IN] gaussian_kernel_derivative the associated derivative
        * \param[OUT] grad_x gradient along X direction
        * \param[OUT] grad_y gradient along Y direction
        * \note if output doesn't fit in input i.e. output.rows() < input.rows() or
        * output.cols() < input.cols() then output is resized to input sizes.
        */
      void
      computeGradients(const Eigen::MatrixXd& input,
                       const Eigen::VectorXd& gaussian_kernel,
                       const Eigen::VectorXd& gaussian_kernel_derivative,
                       Eigen::MatrixXd& grad_x,
                       Eigen::MatrixXd& grad_y) const;
      
      /** Smooth image using a gaussian kernel.
        * \param[IN] input image
        * \param[IN] sigma the gaussian kernel parameter
        * \param[OUT] output the smoothed image
        * \note if output doesn't fit in input i.e. output.rows() < input.rows() or
        * output.cols() < input.cols() then output is resized to input sizes.
        */
      void
      smooth(const Eigen::MatrixXd& input,
             const Eigen::VectorXd& gaussian_kernel,
             Eigen::MatrixXd& output) const;
    };
}

#endif // PCL_IMAGING_KERNEL
