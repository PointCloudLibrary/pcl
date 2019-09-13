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

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>

namespace pcl {

template <typename PointT>
class kernel {
public:
  /// Different types of kernels available.
  enum KERNEL_ENUM {
    SOBEL_X,               //!< SOBEL_X
    SOBEL_Y,               //!< SOBEL_Y
    PREWITT_X,             //!< PREWITT_X
    PREWITT_Y,             //!< PREWITT_Y
    ROBERTS_X,             //!< ROBERTS_X
    ROBERTS_Y,             //!< ROBERTS_Y
    LOG,                   //!< LOG
    DERIVATIVE_CENTRAL_X,  //!< DERIVATIVE_CENTRAL_X
    DERIVATIVE_FORWARD_X,  //!< DERIVATIVE_FORWARD_X
    DERIVATIVE_BACKWARD_X, //!< DERIVATIVE_BACKWARD_X
    DERIVATIVE_CENTRAL_Y,  //!< DERIVATIVE_CENTRAL_Y
    DERIVATIVE_FORWARD_Y,  //!< DERIVATIVE_FORWARD_Y
    DERIVATIVE_BACKWARD_Y, //!< DERIVATIVE_BACKWARD_Y
    GAUSSIAN               //!< GAUSSIAN
  };

  int kernel_size_;
  float sigma_;
  KERNEL_ENUM kernel_type_;

  kernel() : kernel_size_(3), sigma_(1.0), kernel_type_(GAUSSIAN) {}

  /**
   *
   * @param kernel Kernel point cloud passed by reference
   *
   * Helper function which returns the kernel selected by the kernel_type_ enum
   */
  void
  fetchKernel(pcl::PointCloud<PointT>& kernel);

  /**
   *
   * @param kernel Kernel point cloud passed by reference
   *
   * Gaussian kernel with size (kernel_size_ x kernel_size_) and variance sigma_
   */
  void
  gaussianKernel(pcl::PointCloud<PointT>& kernel);

  /**
   *
   * @param kernel Kernel point cloud passed by reference
   *
   * Laplacian of Gaussian kernel with size (kernel_size_ x kernel_size_) and variance
   * sigma_
   */
  void
  loGKernel(pcl::PointCloud<PointT>& kernel);

  /**
   *
   * @param kernel Kernel point cloud passed by reference
   *
   * 3x3 Sobel kernel in the X direction
   */
  void
  sobelKernelX(pcl::PointCloud<PointT>& kernel);

  /**
   *
   * @param kernel Kernel point cloud passed by reference
   *
   * 3x3 Prewitt kernel in the X direction
   */
  void
  prewittKernelX(pcl::PointCloud<PointT>& kernel);

  /**
   *
   * @param kernel Kernel point cloud passed by reference
   *
   * 2x2 Roberts kernel in the X direction
   */
  void
  robertsKernelX(pcl::PointCloud<PointT>& kernel);

  /**
   *
   * @param kernel Kernel point cloud passed by reference
   *
   * 3x3 Sobel kernel in the Y direction
   */
  void
  sobelKernelY(pcl::PointCloud<PointT>& kernel);

  /**
   *
   * @param kernel Kernel point cloud passed by reference
   *
   * 3x3 Prewitt kernel in the Y direction
   */
  void
  prewittKernelY(pcl::PointCloud<PointT>& kernel);

  /**
   *
   * @param kernel Kernel point cloud passed by reference
   *
   * 2x2 Roberts kernel in the Y direction
   */
  void
  robertsKernelY(pcl::PointCloud<PointT>& kernel);

  /**
   *
   * @param kernel Kernel point cloud passed by reference
   *
   * kernel [-1 0 1]
   */
  void
  derivativeXCentralKernel(pcl::PointCloud<PointT>& kernel);

  /**
   *
   * @param kernel Kernel point cloud passed by reference
   *
   * kernel [-1 0 1]'
   */
  void
  derivativeYCentralKernel(pcl::PointCloud<PointT>& kernel);

  /**
   *
   * @param kernel Kernel point cloud passed by reference
   *
   * kernel [0 -1 1]
   */
  void
  derivativeXForwardKernel(pcl::PointCloud<PointT>& kernel);

  /**
   *
   * @param kernel Kernel point cloud passed by reference
   *
   * kernel [0 -1 1]'
   */
  void
  derivativeYForwardKernel(pcl::PointCloud<PointT>& kernel);

  /**
   *
   * @param kernel Kernel point cloud passed by reference
   *
   * kernel [-1 1 0]
   */
  void
  derivativeXBackwardKernel(pcl::PointCloud<PointT>& kernel);

  /**
   *
   * @param kernel Kernel point cloud passed by reference
   *
   * kernel [-1 1 0]'
   */
  void
  derivativeYBackwardKernel(PointCloud<PointT>& kernel);

  /**
   *
   * @param kernel_type enum indicating the kernel type wanted
   *
   * select the kernel type.
   */
  void
  setKernelType(KERNEL_ENUM kernel_type);

  /**
   *
   * @param kernel_size kernel of size kernel_size x kernel_size is created(LoG and
   * Gaussian only)
   *
   * Setter function for kernel_size_
   */
  void
  setKernelSize(int kernel_size);

  /**
   *
   * @param kernel_sigma variance of the Gaussian or LoG kernels.
   *
   * Setter function for kernel_sigma_
   */
  void
  setKernelSigma(float kernel_sigma);
};

} // namespace pcl

#include <pcl/2d/impl/kernel.hpp>
