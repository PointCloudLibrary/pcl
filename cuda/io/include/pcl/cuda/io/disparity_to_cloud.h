/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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

#pragma once

#include <pcl/cuda/point_cloud.h>
#include <pcl/cuda/io/cloud_to_pcl.h>
#include <pcl/io/openni_camera/openni_image.h>
#include <pcl/io/openni_camera/openni_depth_image.h>
//#include <pcl/CameraInfo.h>
//#include <pcl/PCLImage.h>

#include <cstdint>

namespace pcl
{
namespace cuda
{
  /** \brief Compute the XYZ values for a point based on disparity information. */
  struct ComputeXYZ
  {
    int    width, height;
    int    center_x, center_y;
    float  constant;
    float bad_point;

    ComputeXYZ (int w, int h, int cx, int cy, float con) : 
      width(w), height(h), center_x(cx), center_y(cy), constant(con)
    {
      bad_point = std::numeric_limits<float>::quiet_NaN ();
    }

    template <typename Tuple> __inline__ __host__ __device__ PointXYZRGB
    operator () (const Tuple &t);
  };

  /** \brief Compute the XYZ and RGB values for a point based on disparity information. */
  struct ComputeXYZRGB
  {
    int    width, height;
    int    center_x, center_y;
    float  constant;
    float bad_point;

    ComputeXYZRGB (int w, int h, int cx, int cy, float con) : 
      width(w), height(h), center_x(cx), center_y(cy), constant(con) 
    {
      bad_point = std::numeric_limits<float>::quiet_NaN ();
    }

    template <typename Tuple> __inline__ __host__ __device__ PointXYZRGB
    operator () (const Tuple &t);
  };

  /** \brief Disparity to PointCloudAOS generator.
    */
  class PCL_EXPORTS DisparityToCloud
  {
    public:
//      // compute using ROS images, Device output
//      void
//      compute (const pcl::PCLImage::ConstPtr &depth_image,
//               const pcl::PCLImage::ConstPtr &rgb_image,
//               const pcl::CameraInfo::ConstPtr &info,
//               PointCloudAOS<Device>::Ptr &output);
//      
//      // compute using ROS images, Host output
//      void
//      compute (const pcl::PCLImage::ConstPtr &depth_image,
//               const pcl::PCLImage::ConstPtr &rgb_image,
//               const pcl::CameraInfo::ConstPtr &info,
//               PointCloudAOS<Host>::Ptr &output);

      // compute using OpenNI images, Device output
      template <template <typename> class Storage> void
      compute (const openni_wrapper::DepthImage::Ptr& depth_image,
               const openni_wrapper::Image::Ptr& image,
               float constant, 
               typename PointCloudAOS<Storage>::Ptr &output,
               bool downsample = false, int stride = 2, int smoothing_nr_iterations = 0, int smoothing_filter_size = 2);

      template <template <typename> class Storage> void
      compute (const std::uint16_t* depth_image,
               const OpenNIRGB* rgb_image,
               int width, int height,
               float constant,
               typename PointCloudAOS<Storage>::Ptr &output,
               int smoothing_nr_iterations = 0, int smoothing_filter_size = 2);

      // compute using OpenNI images, Host output
/*      void
      compute (const openni_wrapper::DepthImage::Ptr& depth_image,
               const openni_wrapper::Image::Ptr& image,
               float constant, 
               PointCloudAOS<Host>::Ptr &output);*/
      
      // ...
//      void
//      compute (const pcl::PCLImage::ConstPtr &depth_image,
//               const pcl::CameraInfo::ConstPtr &info,
//               PointCloudAOS<Device>::Ptr &output);
//
//      void
//      compute (const pcl::PCLImage::ConstPtr &depth_image,
//               const pcl::CameraInfo::ConstPtr &info,
//               PointCloudAOS<Host>::Ptr &output);

      void
      compute (const openni_wrapper::DepthImage::Ptr& depth_image,
                float constant,
                PointCloudAOS<Device>::Ptr &output);

      void
      compute (const openni_wrapper::DepthImage::Ptr& depth_image,
                float constant,
                PointCloudAOS<Host>::Ptr &output);
  };

} // namespace
} // namespace
