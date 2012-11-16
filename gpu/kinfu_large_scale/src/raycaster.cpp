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

#include <pcl/gpu/kinfu_large_scale/raycaster.h>
#include <pcl/gpu/kinfu_large_scale/tsdf_volume.h>
#include "internal.h"

using namespace pcl;
using namespace Eigen;
//using namespace pcl::gpu::kinfuLS;
//using namespace pcl::device::kinfuLS;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

pcl::gpu::kinfuLS::RayCaster::RayCaster(int rows_arg, int cols_arg, float fx, float fy, float cx, float cy)
   : cols(cols_arg), rows(rows_arg), fx_(fx), fy_(fy), cx_(cx < 0 ? cols/2 : cx), cy_(cy < 0 ? rows/2 : cy)
{ 
  vertex_map_.create(rows * 3, cols);
  normal_map_.create(rows * 3, cols);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

pcl::gpu::kinfuLS::RayCaster::~RayCaster()
{

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::gpu::kinfuLS::RayCaster::setIntrinsics(float fx, float fy, float cx, float cy)
{
  fx_ = fx;
  fy_ = fy;
  cx_ = cx < 0 ? cols/2 : cx;
  cy_ = cy < 0 ? rows/2 : cy;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::gpu::kinfuLS::RayCaster::run(const TsdfVolume& volume, const Affine3f& camera_pose, tsdf_buffer* buffer)
{  
  camera_pose_.linear() = camera_pose.linear();
  camera_pose_.translation() = camera_pose.translation();
  volume_size_ = volume.getSize();
  pcl::device::kinfuLS::Intr intr (fx_, fy_, cx_, cy_);

  vertex_map_.create(rows * 3, cols);
  normal_map_.create(rows * 3, cols);

  typedef Matrix<float, 3, 3, RowMajor> Matrix3f;
    
  Matrix3f R = camera_pose_.linear();
  Vector3f t = camera_pose_.translation();

  const  pcl::device::kinfuLS::Mat33& device_R   = pcl::device::kinfuLS::device_cast<const pcl::device::kinfuLS::Mat33>(R);
  // const float3& device_t   = device_cast<const float3>(t);
  
  float3& device_t   = pcl::device::kinfuLS::device_cast<float3>(t);
  
  device_t.x -= buffer->origin_metric.x;
  device_t.y -= buffer->origin_metric.y;
  device_t.z -= buffer->origin_metric.z;
  
  float tranc_dist = volume.getTsdfTruncDist();  
  pcl::device::kinfuLS::raycast (intr, device_R, device_t, tranc_dist, pcl::device::kinfuLS::device_cast<const float3>(volume_size_), volume.data(), buffer, vertex_map_, normal_map_);  
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::gpu::kinfuLS::RayCaster::generateSceneView(View& view) const
{
  generateSceneView(view, volume_size_ * (-3.f));
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::gpu::kinfuLS::RayCaster::generateSceneView(View& view, const Vector3f& light_source_pose) const
{
  pcl::device::kinfuLS::LightSource light;
  light.number = 1;  
  light.pos[0] = pcl::device::kinfuLS::device_cast<const float3>(light_source_pose);
  
  view.create(rows, cols);
  pcl::device::kinfuLS::generateImage (vertex_map_, normal_map_, light, view);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::gpu::kinfuLS::RayCaster::generateDepthImage(Depth& depth) const
{
  pcl::device::kinfuLS::Intr intr (fx_, fy_, cx_, cy_);
  
  depth.create(rows, cols);    
  
  Matrix<float, 3, 3, RowMajor> R_inv = camera_pose_.linear().inverse();
  Vector3f t = camera_pose_.translation();
  
  pcl::device::kinfuLS::generateDepth(pcl::device::kinfuLS::device_cast<pcl::device::kinfuLS::Mat33>(R_inv), pcl::device::kinfuLS::device_cast<const float3>(t), vertex_map_, depth);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
pcl::gpu::kinfuLS::RayCaster::MapArr
pcl::gpu::kinfuLS::RayCaster::getVertexMap() const
{
  return vertex_map_;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
pcl::gpu::kinfuLS::RayCaster::MapArr
pcl::gpu::kinfuLS::RayCaster::getNormalMap() const
{
  return normal_map_;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

namespace pcl
{
  namespace gpu
  {
    namespace kinfuLS
    {
      template<> PCL_EXPORTS void
      convertMapToOranizedCloud<pcl::PointXYZ>(const RayCaster::MapArr& map, DeviceArray2D<pcl::PointXYZ>& cloud)
      {
        cloud.create (map.rows()/3, map.cols());
        DeviceArray2D<float4>& c = (DeviceArray2D<float4>&)cloud;
        pcl::device::kinfuLS::convert (map, c);
      }

      template<> PCL_EXPORTS void
      convertMapToOranizedCloud<pcl::Normal> (const RayCaster::MapArr& map, DeviceArray2D<pcl::Normal>& cloud)
      {
        cloud.create (map.rows()/3, map.cols());
        DeviceArray2D<pcl::device::kinfuLS::float8>& n = (DeviceArray2D<pcl::device::kinfuLS::float8>&)cloud;
        pcl::device::kinfuLS::convert(map, n);
      }
    }
  }
}
