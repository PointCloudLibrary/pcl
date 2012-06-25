/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2011, Willow Garage, Inc.
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
 */

//~ #include <pcl/gpu/kinfu_large_scale/raycaster_large_scale.h>
//~ #include <pcl/gpu/kinfu_large_scale/tsdf_volume.h>
//~ #include "internal.h"
//~ 
//~ using namespace pcl;
//~ using namespace pcl::gpu;
//~ using namespace pcl::device;
//~ using namespace Eigen;
//~ 
//~ ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//~ void 
//~ pcl::gpu::RayCaster::run(const TsdfVolume& volume, const Affine3f& camera_pose, pcl::gpu::tsdf_buffer* buffer)
//~ {  
  //~ camera_pose_.linear() = camera_pose.linear();
  //~ camera_pose_.translation() = camera_pose.translation();
  //~ volume_size_ = volume.getSize();
  //~ device::Intr intr (fx_, fy_, cx_, cy_);
//~ 
  //~ vertex_map_.create(rows * 3, cols);
  //~ normal_map_.create(rows * 3, cols);
//~ 
  //~ typedef Matrix<float, 3, 3, RowMajor> Matrix3f;
    //~ 
  //~ Matrix3f R = camera_pose_.linear();
  //~ Vector3f t = camera_pose_.translation();
//~ 
  //~ const  Mat33& device_R   = device_cast<const Mat33>(R);
  //~ // const float3& device_t   = device_cast<const float3>(t);
  //~ 
  //~ float3& device_t   = device_cast<float3>(t);
  //~ 
  //~ device_t.x -= buffer->origin_metric.x;
  //~ device_t.y -= buffer->origin_metric.y;
  //~ device_t.z -= buffer->origin_metric.z;
  //~ 
  //~ float tranc_dist = volume.getTsdfTruncDist();  
  //~ device::raycast (intr, device_R, device_t, tranc_dist, device_cast<const float3>(volume_size_), volume.data(), buffer, vertex_map_, normal_map_);  
//~ }
