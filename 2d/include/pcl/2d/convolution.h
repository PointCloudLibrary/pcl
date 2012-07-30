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

#ifndef PCL_2D_CONVOLUTION_H
#define PCL_2D_CONVOLUTION_H

#include <pcl/pcl_base.h>
#include <pcl/filters/filter.h>
#include <pcl/point_types.h>
namespace pcl
{
  namespace pcl_2d
  {
    /**
     * This typedef is used to represent a point cloud containing edge information
     */
    struct PointXYZIEdge
    {
      PCL_ADD_POINT4D;                    // preferred way of adding a XYZ+padding
      float magnitude;
      float direction;
      float magnitude_x;
      float magnitude_y;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // make sure our new allocators are aligned
    } EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

    template<typename PointT>
    class convolution : public Filter<PointT>
    {

    using Filter<PointT>::input_;

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
    enum BOUNDARY_OPTIONS_ENUM
    {
      BOUNDARY_OPTION_CLAMP,
      BOUNDARY_OPTION_MIRROR,
      BOUNDARY_OPTION_ZERO_PADDING
    };

    enum IMAGE_CHANNEL
    {
      IMAGE_CHANNEL_RGB,
      IMAGE_CHANNEL_INTENSITY
    };


    convolution ()
    {
      image_channel_ = IMAGE_CHANNEL_INTENSITY;
      boundary_options_ = BOUNDARY_OPTION_CLAMP;
    }

    /**
     * @param output Output point cloud passed by reference

     * Performs 2D convolution of the input point cloud with the kernel.
     * Uses clamp as the default boundary option.
     */
    void convolve (pcl::PointCloud<PointT> &output);

    /**
     *
     * @param output Output point cloud passed by reference
     *
     * This is an over-ride function for the pcl::Filter interface
     */
    void applyFilter (pcl::PointCloud<PointT> &output);

    /**
     *
     * @param kernel convolution kernel passed by reference
     *
     * Sets the kernel to be used for convolution
     */
    void setKernel (pcl::PointCloud<PointT> &kernel);

    /**
     *
     * @param boundary_options enum indicating the boundary options to be used for convolution
     */
    void setBoundaryOptions (BOUNDARY_OPTIONS_ENUM boundary_options);

    /**
     *
     * @param image_channel image channel to be used for the convolution
     */
    void setImageChannel(IMAGE_CHANNEL image_channel);

private:
    BOUNDARY_OPTIONS_ENUM boundary_options_;
    pcl::PointCloud<PointT> kernel_;
    IMAGE_CHANNEL image_channel_;

    };


  }
}
#include <pcl/2d/impl/convolution.hpp>
POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::pcl_2d::PointXYZIEdge,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, magnitude, magnitude)
    (float, direction, direction)
    (float, magnitude_x, magnitude_x)
    (float, magnitude_y, magnitude_y)
)
#endif // PCL_2D_CONVOLUTION_2D_H
