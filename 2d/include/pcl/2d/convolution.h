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

#ifndef PCL_2D_CONVOLUTION_2D_H
#define PCL_2D_CONVOLUTION_2D_H

#include <pcl/pcl_base.h>
namespace pcl
{
  namespace pcl_2d
  {
    /**
     * This typedef is used to represent a single channel 2D image.     *
     */
    typedef std::vector<std::vector< float> > ImageType;
    class convolution
    {

      public:

        /**
         * Extra pixels are added to the input image so that convolution can be performed over the entire image.
         *
         * (kernel_height/2) rows are added before the first row and after the last row
         * (kernel_width/2) columns are added before the first column and after the last column
         * border options define what values are set for these extra rows and columns
         *
         * Assume that the three rows of right edge of the image looks like this:
         *    .. 3 2 1
         *    .. 6 5 4
         *    .. 9 8 7
         *
         * BOUNDARY_OPTION_CLAMP : the extra pixels are set to the pixel value of the boundary pixel
         *    This option makes it seem as if it were:
         *    .. 3 2 1| 1 1 1 ..
         *    .. 6 5 4| 4 4 4 ..
         *    .. 9 8 7| 7 7 7 ..
         *
         * BOUNDARY_OPTION_MIRROR : the input image is mirrored at the boundary.
         *    This option makes it seem as if it were:
         *    .. 3 2 1| 1 2 3 ..
         *    .. 6 5 4| 4 5 6 ..
         *    .. 9 8 7| 7 8 9 ..
         *
         * BOUNDARY_OPTION_ZERO_PADDING : the extra pixels are simply set to 0
         *    This option makes it seem as if it were:
         *    .. 3 2 1| 0 0 0 ..
         *    .. 6 5 4| 0 0 0 ..
         *    .. 9 8 7| 0 0 0 ..
         *
         * Note that the input image is not actually extended in size. Instead, based on these options,
         * the convolution is performed differently at the border pixels.
         */
        typedef enum BOUNDARY_OPTIONS_ENUM{
          BOUNDARY_OPTION_CLAMP,
          BOUNDARY_OPTION_MIRROR,
          BOUNDARY_OPTION_ZERO_PADDING
        };

        convolution  ()
        {

        }

        /**
         * \param output The output image passed by reference
         * \param kernel The kernel for convolution
         * \param input The input image passed by reference
         *
         * Performs 2D convolution of the input image with the kernel.
         * Uses zero padding as the default boundary option.
         */
        void convolve  (ImageType &output, ImageType &kernel, ImageType &input);

        /**
         * \param output The output image passed by reference
         * \param kernel The kernel for convolution
         * \param input The input image passed by reference
         * \param boundary_options Boundary options are available as ENUM's
         *
         * Performs 2D convolution of the input image with the kernel.
         */
        void convolve  (ImageType &output, ImageType &kernel, ImageType &input, BOUNDARY_OPTIONS_ENUM boundary_options);

        /**
         * \param kernel_size The kernel is of size kernel_size x kernel_size.
         * \param kernel The output kernel passed by reference.
         * \param sigma This is the variance of the kernel.
         *
         * This function creates a normalized Gaussian kernel.
         */
        void gaussianKernel  (const int kernel_size, const float sigma, ImageType &kernel);

        /**
         * \param output The output image passed by reference
         * \param input The input image passed by reference
         * This function applies Gaussian smoothing to the input image.
         * A normalized Gaussian kernel of size kernel_size x kernel_size and variance sigma is used.         *
         */
        void gaussianSmooth  (ImageType &input, ImageType &output, const int kernel_size, const float sigma);

        BOUNDARY_OPTIONS_ENUM boundary_options;
        ImageType kernel;
        /**
         * \param output The output image passed by reference
         * \param kernel The kernel for convolution
         * \param input The input image passed by reference
         *
         * Performs 2D convolution of the input image with the kernel.
         * Uses zero padding as the default boundary option.
         */
        template<typename PointT>
        void convolve  (PointCloud<PointT> &output, PointCloud<PointT> &input);
    };
  }
}
#include <pcl/2d/impl/convolution.hpp>
#endif // PCL_2D_CONVOLUTION_2D_H
