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
 * $Id: convolution_2d.h nsomani $
 *
 */

#ifndef PCL_2D_CONVOLUTION_2D_H
#define PCL_2D_CONVOLUTION_2D_H

#include <vector>
namespace pcl
{
  namespace pcl_2d
  {
    typedef std::vector<std::vector< float> > ImageType;
    class convolution_2d
    {

      public:
        /**
         * Extra pixels are added to the input image so that convolution can be performed over the entire image.
         *
         * (kernel_height/2) rows are added before the first row and after the last row
         * (kernel_column/2) columns are added before the first column and after the last column
         * border options define what values are set for these extra rows and columns
         *
         * BOUNDARY_OPTION_CLAMP : the extra pixels are set to the pixel value of the boundary pixel
         * BOUNDARY_OPTION_MIRROR : the input image is mirrored at the boundary.
         * BOUNDARY_OPTION_ZERO_PADDING : the extra pixels are simply set to 0
         *
         * The input image is not actually extended in size. Instead, based on these options, the convolution is
         * performed differently at the border pixels.
         */
        static const int BOUNDARY_OPTION_CLAMP = 0;
        static const int BOUNDARY_OPTION_MIRROR = 1;
        static const int BOUNDARY_OPTION_ZERO_PADDING = 2;

        convolution_2d  ()
        {

        }
        /**
         * Performs 2D convolution of the input image with the kernel.
         * Uses zero padding as the default boundary option.
         */
        void convolve  (ImageType &output, ImageType &kernel, ImageType &input);
        /**
         * \param boundary_options Boundary options are available as ENUM's
         * Performs 2D convolution of the input image with the kernel.
         */
        void convolve  (ImageType &output, ImageType &kernel, ImageType &input, const int boundary_options);
        /**
         * \param kernel_size The kernel is of size kernel_size x kernel_size.
         * \param sigma This is the variance of the kernel.
         *
         * This function creates a normalized Gaussian kernel.
         */
        void gaussianKernel  (const int kernel_size, const float sigma, ImageType &kernel);
        /**
         * This function applies Gaussian smoothing to the input image.
         * A normalized Gaussian kernel of size kernel_size x kernel_size and variance sigma is used.         *
         */
        void gaussianSmooth  (ImageType &input, ImageType &output, const int kernel_size, const float sigma);
    };
  }
}
#include <pcl/2d/impl/convolution_2d.hpp>
#endif // PCL_2D_CONVOLUTION_2D_H
