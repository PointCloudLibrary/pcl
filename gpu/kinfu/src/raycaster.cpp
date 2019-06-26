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

#include <pcl/gpu/kinfu/raycaster.h>
#include <pcl/gpu/kinfu/tsdf_volume.h>
#include "internal.h"

using namespace pcl;
using namespace pcl::gpu;
using namespace pcl::device;
using namespace Eigen;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

pcl::gpu::RayCaster::RayCaster(int rows_arg, int cols_arg, float fx, float fy, float cx, float cy)
   : cols(cols_arg), rows(rows_arg), fx_(fx), fy_(fy), cx_(cx < 0 ? cols/2 : cx), cy_(cy < 0 ? rows/2 : cy)
{ 
  vertex_map_.create(rows * 3, cols);
  normal_map_.create(rows * 3, cols);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

pcl::gpu::RayCaster::~RayCaster()
{

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::gpu::RayCaster::setIntrinsics(float fx, float fy, float cx, float cy)
{
  fx_ = fx;
  fy_ = fy;
  cx_ = cx < 0 ? cols/2 : cx;
  cy_ = cy < 0 ? rows/2 : cy;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::gpu::RayCaster::run(const TsdfVolume& volume, const Affine3f& camera_pose)
{  
  camera_pose_.linear() = camera_pose.linear();
  camera_pose_.translation() = camera_pose.translation();
  volume_size_ = volume.getSize();
  device::Intr intr (fx_, fy_, cx_, cy_);

  vertex_map_.create(rows * 3, cols);
  normal_map_.create(rows * 3, cols);

  using Matrix3f = Matrix<float, 3, 3, RowMajor>;
    
  Matrix3f R = camera_pose_.linear();
  Vector3f t = camera_pose_.translation();

  const  Mat33& device_R   = device_cast<const Mat33>(R);
  const float3& device_t   = device_cast<const float3>(t);
  
  float tranc_dist = volume.getTsdfTruncDist();  
  device::raycast (intr, device_R, device_t, tranc_dist, device_cast<const float3>(volume_size_), volume.data(), vertex_map_, normal_map_);  
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::gpu::RayCaster::generateSceneView(View& view) const
{
  generateSceneView(view, volume_size_ * (-3.f));
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::gpu::RayCaster::generateSceneView(View& view, const Vector3f& light_source_pose) const
{
  device::LightSource light;
  light.number = 1;  
  light.pos[0] = device_cast<const float3>(light_source_pose);
  
  view.create(rows, cols);
  device::generateImage (vertex_map_, normal_map_, light, view);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::gpu::RayCaster::generateDepthImage(Depth& depth) const
{
  device::Intr intr (fx_, fy_, cx_, cy_);
  
  depth.create(rows, cols);    
  
  Matrix<float, 3, 3, RowMajor> R_inv = camera_pose_.linear().inverse();
  Vector3f t = camera_pose_.translation();
  
  device::generateDepth(device_cast<Mat33>(R_inv), device_cast<const float3>(t), vertex_map_, depth);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
pcl::gpu::RayCaster::MapArr
pcl::gpu::RayCaster::getVertexMap() const
{
  return vertex_map_;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
pcl::gpu::RayCaster::MapArr
pcl::gpu::RayCaster::getNormalMap() const
{
  return normal_map_;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

namespace pcl
{
  namespace gpu
  {
    template<> PCL_EXPORTS void
    convertMapToOranizedCloud<pcl::PointXYZ>(const RayCaster::MapArr& map, DeviceArray2D<pcl::PointXYZ>& cloud)
    {
      cloud.create (map.rows()/3, map.cols());
      DeviceArray2D<float4>& c = (DeviceArray2D<float4>&)cloud;
      device::convert (map, c);
    }

    template<> PCL_EXPORTS void
    convertMapToOranizedCloud<pcl::Normal> (const RayCaster::MapArr& map, DeviceArray2D<pcl::Normal>& cloud)
    {
      cloud.create (map.rows()/3, map.cols());
      DeviceArray2D<float8>& n = (DeviceArray2D<float8>&)cloud;
      device::convert(map, n);
    }
  }
}
