/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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
 * $Id$
 *
 */

#ifndef PCL_GAUSSIAN_KERNEL
#define PCL_GAUSSIAN_KERNEL

#include <sstream>
#include <pcl/common/eigen.h>
#include <pcl/point_cloud.h>
#include <boost/function.hpp>

namespace pcl
{
  /** Class GaussianKernel assembles all the method for computing, 
    * convolving, smoothing, gradients computing an image using
    * a gaussian kernel. The image is stored in point cloud elements 
    * intensity member or rgb or...
    * \author Nizar Sallem
    * \ingroup common
    */
  class PCL_EXPORTS GaussianKernel
  {
    public:

      GaussianKernel () {}

      static const unsigned MAX_KERNEL_WIDTH = 71;
      /** Computes the gaussian kernel and dervative assiociated to sigma.
        * The kernel and derivative width are adjusted according.
        * \param[in] sigma
        * \param[out] kernel the computed gaussian kernel
        * \param[in] kernel_width the desired kernel width upper bond
        * \throws pcl::KernelWidthTooSmallException
        */
      void
      compute (float sigma, 
               Eigen::VectorXf &kernel,
               unsigned kernel_width = MAX_KERNEL_WIDTH) const;

      /** Computes the gaussian kernel and dervative assiociated to sigma.
        * The kernel and derivative width are adjusted according.
        * \param[in] sigma
        * \param[out] kernel the computed gaussian kernel
        * \param[out] derivative the computed kernel derivative
        * \param[in] kernel_width the desired kernel width upper bond
        * \throws pcl::KernelWidthTooSmallException
        */
      void
      compute (float sigma, 
               Eigen::VectorXf &kernel, 
               Eigen::VectorXf &derivative, 
               unsigned kernel_width = MAX_KERNEL_WIDTH) const;

      /** Convolve a float image rows by a given kernel.
        * \param[in] kernel convolution kernel
        * \param[in] input the image to convolve
        * \param[out] output the convolved image
        * \note if output doesn't fit in input i.e. output.rows () < input.rows () or
        * output.cols () < input.cols () then output is resized to input sizes.
        */
      void
      convolveRows (const pcl::PointCloud<float> &input,
                    const Eigen::VectorXf &kernel,
                    pcl::PointCloud<float> &output) const;

      /** Convolve a float image rows by a given kernel.
        * \param[in] input the image to convolve
        * \param[in] field_accessor a field accessor
        * \param[in] kernel convolution kernel
        * \param[out] output the convolved image
        * \note if output doesn't fit in input i.e. output.rows () < input.rows () or
        * output.cols () < input.cols () then output is resized to input sizes.
        */
     template <typename PointT> void
     convolveRows (const pcl::PointCloud<PointT> &input,
                   boost::function <float (const PointT& p)> field_accessor,
                   const Eigen::VectorXf &kernel,
                   pcl::PointCloud<float> &output) const;

      /** Convolve a float image columns by a given kernel.
        * \param[in] input the image to convolve
        * \param[in] kernel convolution kernel
        * \param[out] output the convolved image
        * \note if output doesn't fit in input i.e. output.rows () < input.rows () or
        * output.cols () < input.cols () then output is resized to input sizes.
        */
      void
      convolveCols (const pcl::PointCloud<float> &input,
                    const Eigen::VectorXf &kernel,
                    pcl::PointCloud<float> &output) const;

      /** Convolve a float image columns by a given kernel.
        * \param[in] input the image to convolve
        * \param[in] field_accessor a field accessor
        * \param[in] kernel convolution kernel
        * \param[out] output the convolved image
        * \note if output doesn't fit in input i.e. output.rows () < input.rows () or
        * output.cols () < input.cols () then output is resized to input sizes.
        */
      template <typename PointT> void
      convolveCols (const pcl::PointCloud<PointT> &input,
                    boost::function <float (const PointT& p)> field_accessor,
                    const Eigen::VectorXf &kernel,
                    pcl::PointCloud<float> &output) const;

      /** Convolve a float image in the 2 directions
        * \param[in] horiz_kernel kernel for convolving rows
        * \param[in] vert_kernel kernel for convolving columns
        * \param[in] input image to convolve
        * \param[out] output the convolved image
        * \note if output doesn't fit in input i.e. output.rows () < input.rows () or
        * output.cols () < input.cols () then output is resized to input sizes.
        */
      inline void
      convolve (const pcl::PointCloud<float> &input,
                const Eigen::VectorXf &horiz_kernel,
                const Eigen::VectorXf &vert_kernel,
                pcl::PointCloud<float> &output) const
      {
        std::cout << ">>> convolve cpp" << std::endl;
        pcl::PointCloud<float> tmp (input.width, input.height) ;
        convolveRows (input, horiz_kernel, tmp);        
        convolveCols (tmp, vert_kernel, output);
        std::cout << "<<< convolve cpp" << std::endl;
      }

      /** Convolve a float image in the 2 directions
        * \param[in] input image to convolve
        * \param[in] field_accessor a field accessor
        * \param[in] horiz_kernel kernel for convolving rows
        * \param[in] vert_kernel kernel for convolving columns
        * \param[out] output the convolved image
        * \note if output doesn't fit in input i.e. output.rows () < input.rows () or
        * output.cols () < input.cols () then output is resized to input sizes.
        */
      template <typename PointT> inline void
      convolve (const pcl::PointCloud<PointT> &input,
                boost::function <float (const PointT& p)> field_accessor,
                const Eigen::VectorXf &horiz_kernel,
                const Eigen::VectorXf &vert_kernel,
                pcl::PointCloud<float> &output) const
      {
        std::cout << ">>> convolve hpp" << std::endl;
        pcl::PointCloud<float> tmp (input.width, input.height);
        convolveRows<PointT>(input, field_accessor, horiz_kernel, tmp);
        convolveCols(tmp, vert_kernel, output);
        std::cout << "<<< convolve hpp" << std::endl;
      }
      
      /** Computes float image gradients using a gaussian kernel and gaussian kernel
        * derivative.
        * \param[in] input image to compute gardients for
        * \param[in] gaussian_kernel the gaussian kernel to be used
        * \param[in] gaussian_kernel_derivative the associated derivative
        * \param[out] grad_x gradient along X direction
        * \param[out] grad_y gradient along Y direction
        * \note if output doesn't fit in input i.e. output.rows () < input.rows () or
        * output.cols () < input.cols () then output is resized to input sizes.
        */
      inline void
      computeGradients (const pcl::PointCloud<float> &input,
                        const Eigen::VectorXf &gaussian_kernel,
                        const Eigen::VectorXf &gaussian_kernel_derivative,
                        pcl::PointCloud<float> &grad_x,
                        pcl::PointCloud<float> &grad_y) const
      {
        convolve (input, gaussian_kernel_derivative, gaussian_kernel, grad_x);
        convolve (input, gaussian_kernel, gaussian_kernel_derivative, grad_y);
      }

      /** Computes float image gradients using a gaussian kernel and gaussian kernel
        * derivative.
        * \param[in] input image to compute gardients for
        * \param[in] field_accessor a field accessor
        * \param[in] gaussian_kernel the gaussian kernel to be used
        * \param[in] gaussian_kernel_derivative the associated derivative
        * \param[out] grad_x gradient along X direction
        * \param[out] grad_y gradient along Y direction
        * \note if output doesn't fit in input i.e. output.rows () < input.rows () or
        * output.cols () < input.cols () then output is resized to input sizes.
        */
      template <typename PointT> inline void
      computeGradients (const pcl::PointCloud<PointT> &input,
                        boost::function <float (const PointT& p)> field_accessor,
                        const Eigen::VectorXf &gaussian_kernel,
                        const Eigen::VectorXf &gaussian_kernel_derivative,
                        pcl::PointCloud<float> &grad_x,
                        pcl::PointCloud<float> &grad_y) const
      {
        convolve<PointT> (input, field_accessor, gaussian_kernel_derivative, gaussian_kernel, grad_x);
        convolve<PointT> (input, field_accessor, gaussian_kernel, gaussian_kernel_derivative, grad_y);
      }
      
      /** Smooth image using a gaussian kernel.
        * \param[in] input image
        * \param[in] gaussian_kernel the gaussian kernel to be used
        * \param[out] output the smoothed image
        * \note if output doesn't fit in input i.e. output.rows () < input.rows () or
        * output.cols () < input.cols () then output is resized to input sizes.
        */
      inline void
      smooth (const pcl::PointCloud<float> &input,
              const Eigen::VectorXf &gaussian_kernel,
              pcl::PointCloud<float> &output) const
      {
        convolve (input, gaussian_kernel, gaussian_kernel, output);
      }

      /** Smooth image using a gaussian kernel.
        * \param[in] input image
        * \param[in] field_accessor a field accessor
        * \param[in] gaussian_kernel the gaussian kernel to be used
        * \param[out] output the smoothed image
        * \note if output doesn't fit in input i.e. output.rows () < input.rows () or
        * output.cols () < input.cols () then output is resized to input sizes.
        */
      template <typename PointT> inline void
      smooth (const pcl::PointCloud<PointT> &input,
              boost::function <float (const PointT& p)> field_accessor,
              const Eigen::VectorXf &gaussian_kernel,
              pcl::PointCloud<float> &output) const
      {
        convolve<PointT> (input, field_accessor, gaussian_kernel, gaussian_kernel, output);
      }
  };
}

#include <pcl/common/impl/gaussian.hpp>

#endif // PCL_GAUSSIAN_KERNEL
