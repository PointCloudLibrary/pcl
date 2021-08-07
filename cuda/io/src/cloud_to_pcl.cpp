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

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/cuda/io/cloud_to_pcl.h>

namespace pcl
{
namespace cuda
{

//////////////////////////////////////////////////////////////////////////
void
toPCL (const PointCloudAOS<Host> &input, 
                 const thrust::host_vector<float4> &normals,
                 pcl::PointCloud<pcl::PointXYZRGBNormal> &output)
{
  output.resize (input.points.size ());
  for (std::size_t i = 0; i < input.points.size (); ++i)
  {
    output[i].x = input.points[i].x;
    output[i].y = input.points[i].y;
    output[i].z = input.points[i].z;
    // Pack RGB into a float
    output[i].rgb = *(float*)(&input.points[i].rgb);

    output[i].normal_x  = normals[i].x;
    output[i].normal_y  = normals[i].y;
    output[i].normal_z  = normals[i].z;
    output[i].curvature = normals[i].w;
  }

  output.width    = input.width;
  output.height   = input.height;
  output.is_dense = input.is_dense;
}

//////////////////////////////////////////////////////////////////////////
void
toPCL (const PointCloudAOS<Device> &d_input, 
                 const thrust::device_vector<float4> &d_normals,
                 pcl::PointCloud<pcl::PointXYZRGBNormal> &output)
{
  PointCloudAOS<Host> input;
  input << d_input;
  thrust::host_vector<float4> normals = d_normals;

  toPCL(input, normals, output);
}

//////////////////////////////////////////////////////////////////////////
void
toPCL (const PointCloudAOS<Host> &input, 
                 pcl::PointCloud<pcl::PointXYZRGB> &output)
{
  output.resize (input.points.size ());
  for (std::size_t i = 0; i < input.points.size (); ++i)
  {
    output[i].x = input.points[i].x;
    output[i].y = input.points[i].y;
    output[i].z = input.points[i].z;
    // Pack RGB into a float
    output[i].rgb = *(float*)(&input.points[i].rgb);
  }

  output.width    = input.width;
  output.height   = input.height;
  output.is_dense = input.is_dense;

/*  for (std::size_t i = 0; i < output.size (); ++i)
  std::cerr << 
    output[i].x << " " <<
    output[i].y << " " <<
    output[i].z << " " << std::endl;*/
}

//////////////////////////////////////////////////////////////////////////
void
toPCL (const PointCloudAOS<Device> &input, 
                 pcl::PointCloud<pcl::PointXYZRGB> &output)
{
  PointCloudAOS<Host> cloud;
  cloud << input;

  toPCL(cloud, output);
}
} // namespace
} // namespace

