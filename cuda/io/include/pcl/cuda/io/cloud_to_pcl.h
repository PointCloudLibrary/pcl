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

#include <pcl/pcl_exports.h>

namespace pcl
{
  template <typename T>
  class PointCloud;

  struct PointXYZRGB;
  struct PointXYZRGBNormal;

  namespace cuda
  {
    // convert point cloud with color and normals
    PCL_EXPORTS void
    toPCL (const PointCloudAOS<Host> &input, const thrust::host_vector<float4> &normals, pcl::PointCloud<pcl::PointXYZRGBNormal> &output);
    PCL_EXPORTS void
    toPCL (const PointCloudAOS<Device> &input, const thrust::device_vector<float4> &normals, pcl::PointCloud<pcl::PointXYZRGBNormal> &output);
  
    // convert point cloud with color
    PCL_EXPORTS void
    toPCL (const PointCloudAOS<Host> &input, pcl::PointCloud<pcl::PointXYZRGB> &output);
    PCL_EXPORTS void
    toPCL (const PointCloudAOS<Device> &input, pcl::PointCloud<pcl::PointXYZRGB> &output);
  
    // convert pcl point cloud with color to pcl::cuda cloud
    PCL_EXPORTS void
    fromPCL (const pcl::PointCloud<pcl::PointXYZRGB> &input, PointCloudAOS<Host> &output);
    PCL_EXPORTS void
    fromPCL (const pcl::PointCloud<pcl::PointXYZRGB> &input, PointCloudAOS<Device> &output);
  } // namespace
} // namespace
