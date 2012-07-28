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
 * $Id$
 *
 */

#ifndef KERNEL_H_
#define KERNEL_H_
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
namespace pcl
{
  namespace pcl_2d
  {
    template<typename PointT>
    class kernel
    {
public:

    enum KERNEL_ENUM
    {
      SOBEL_X,
      SOBEL_Y,
      PREWITT_X,
      PREWITT_Y,
      ROBERTS_X,
      ROBERTS_Y,
      LOG,
      DERIVATIVE_CENTRAL_X,
      DERIVATIVE_FORWARD_X,
      DERIVATIVE_BACKWARD_X,
      DERIVATIVE_CENTRAL_Y,
      DERIVATIVE_FORWARD_Y,
      DERIVATIVE_BACKWARD_Y,
      GAUSSIAN
    };

    int kernel_size_;
    float sigma_;
    KERNEL_ENUM kernel_type_;

    kernel () :
      kernel_size_ (3),
      sigma_ (1.0),
      kernel_type_ (GAUSSIAN)
    {

    }

    void fetchKernel (PointCloud<PointT> &kernel);

    void gaussianKernel (PointCloud<PointT> &kernel);

    void loGKernel (PointCloud<PointT> &kernel);

    void sobelKernelX (PointCloud<PointT> &Kernel);

    void prewittKernelX (PointCloud<PointT> &Kernel);

    void robertsKernelX (PointCloud<PointT> &kernel);

    void sobelKernelY (PointCloud<PointT> &Kernel);

    void prewittKernelY (PointCloud<PointT> &Kernel);

    void robertsKernelY (PointCloud<PointT> &kernel);

    void derivativeXCentralKernel (PointCloud<PointT> &kernel);

    void derivativeYCentralKernel (PointCloud<PointT> &kernel);

    void derivativeXForwardKernel (PointCloud<PointT> &kernel);

    void derivativeYForwardKernel (PointCloud<PointT> &kernel);

    void derivativeXBackwardKernel (PointCloud<PointT> &kernel);

    void derivativeYBackwardKernel (PointCloud<PointT> &kernel);

    void setKernelType (KERNEL_ENUM kernel_type);

    void setKernelSize (int kernel_size);

    void setKernelSigma (float kernel_sigma);
    };
  }
}
#include <pcl/2d/impl/kernel.hpp>
#endif /* KERNEL_H_ */
