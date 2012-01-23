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

#include "pcl/gpu/kinfu/kinfu.h"
#include "mc_tables.h"
#include "internal.h"

using namespace pcl;
using namespace pcl::gpu;

pcl::gpu::KinfuMarchingCubes::KinfuMarchingCubes()
{
  edgeTable_.upload(edgeTable, 256);
  numVertsTable_.upload(numVertsTable, 256);
  triTable_.upload(&triTable[0][0], 256 * 16);    
}

pcl::gpu::KinfuMarchingCubes::~KinfuMarchingCubes() {}

DeviceArray<pcl::gpu::KinfuMarchingCubes::PointType> pcl::gpu::KinfuMarchingCubes::operator()(const KinfuTracker& kinfu, DeviceArray<PointType>& triangles_buffer)
{  
  if (triangles_buffer.empty())
    triangles_buffer.create(DEFAULT_TRIANGLES_BUFFER_SIZE);
  occupied_voxels_buffer_.create(3, triangles_buffer.size() / 3);    

  device::bindTextures(edgeTable_, triTable_, numVertsTable_);
  int active_voxels = device::getOccupiedVoxels(kinfu.volume_, occupied_voxels_buffer_);  
  if(!active_voxels)
  {
    device::unbindTextures();
    return DeviceArray<PointType>();
  }

  DeviceArray2D<int> occupied_voxels(3, active_voxels, occupied_voxels_buffer_.ptr(), occupied_voxels_buffer_.step());

  int total_vertexes = device::computeOffsetsAndTotalVertexes(occupied_voxels);
  
  float3 volume_size = *reinterpret_cast<const float3*>(kinfu.volume_size_.data());
  device::generateTriangles(kinfu.volume_, occupied_voxels, volume_size, (DeviceArray<device::PointType>&)triangles_buffer);
    
  device::unbindTextures();
  return DeviceArray<PointType>(triangles_buffer.ptr(), total_vertexes);
}
