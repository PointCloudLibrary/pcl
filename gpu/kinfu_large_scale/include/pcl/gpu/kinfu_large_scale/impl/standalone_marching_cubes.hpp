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

#ifndef PCL_STANDALONE_MARCHING_CUBES_IMPL_HPP_
#define PCL_STANDALONE_MARCHING_CUBES_IMPL_HPP_

#include <pcl/gpu/kinfu_large_scale/standalone_marching_cubes.h>

///////////////////////////////////////////////////////////////////////////////
template <typename PointT>
pcl::gpu::kinfuLS::StandaloneMarchingCubes<PointT>::StandaloneMarchingCubes (int new_voxels_x, int new_voxels_y, int new_voxels_z, float new_volume_size)
{
  voxels_x_ = new_voxels_x;
  voxels_y_ = new_voxels_y;
  voxels_z_ = new_voxels_z;
  volume_size_ = new_volume_size;  
  
  ///Creating GPU TSDF Volume instance
  const Eigen::Vector3f volume_size = Eigen::Vector3f::Constant (volume_size_);
  // std::cout << "VOLUME SIZE IS " << volume_size_ << std::endl;
  const Eigen::Vector3i volume_resolution (voxels_x_, voxels_y_, voxels_z_);
  tsdf_volume_gpu_ = TsdfVolume::Ptr ( new TsdfVolume (volume_resolution) );
  tsdf_volume_gpu_->setSize (volume_size);
  
  ///Creating CPU TSDF Volume instance
  int tsdf_total_size = voxels_x_ * voxels_y_ * voxels_z_;
  tsdf_volume_cpu_= std::vector<int> (tsdf_total_size,0);
  
  mesh_counter_ = 0;
}

///////////////////////////////////////////////////////////////////////////////

template <typename PointT> typename pcl::gpu::kinfuLS::StandaloneMarchingCubes<PointT>::MeshPtr
pcl::gpu::kinfuLS::StandaloneMarchingCubes<PointT>::getMeshFromTSDFCloud (const PointCloud &cloud)
{

  //Clearing TSDF GPU and cPU
  const Eigen::Vector3f volume_size = Eigen::Vector3f::Constant (volume_size_);
    std::cout << "VOLUME SIZE IS " << volume_size_ << std::endl;
  const Eigen::Vector3i volume_resolution (voxels_x_, voxels_y_, voxels_z_);

  //Clear values in TSDF Volume GPU
  tsdf_volume_gpu_->reset (); // This one uses the same tsdf volume but clears it before loading new values. This one is our friend.

  //Clear values in TSDF Volume CPU
  fill (tsdf_volume_cpu_.begin (), tsdf_volume_cpu_.end (), 0);
   
  //Loading values to GPU
  loadTsdfCloudToGPU (cloud);

  //Creating and returning mesh
  return ( runMarchingCubes () );
 
}

///////////////////////////////////////////////////////////////////////////////

//template <typename PointT> std::vector< typename pcl::gpu::StandaloneMarchingCubes<PointT>::MeshPtr >
template <typename PointT> void
pcl::gpu::kinfuLS::StandaloneMarchingCubes<PointT>::getMeshesFromTSDFVector (const std::vector<PointCloudPtr> &tsdf_clouds, const std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > &tsdf_offsets)
{
  std::vector< MeshPtr > meshes_vector;
  
  int max_iterations = std::min( tsdf_clouds.size (), tsdf_offsets.size () ); //Safety check
  PCL_INFO ("There are %d cubes to be processed \n", max_iterations);
  float cell_size = volume_size_ / voxels_x_;

  int mesh_counter = 0;
  
  for(int i = 0; i < max_iterations; ++i)
  {
    PCL_INFO ("Processing cube number %d\n", i);
    
    //Making cloud local
    Eigen::Affine3f cloud_transform; 
    
    float originX = (tsdf_offsets[i]).x();
    float originY = (tsdf_offsets[i]).y();
    float originZ = (tsdf_offsets[i]).z();
    
    cloud_transform.linear ().setIdentity ();
    cloud_transform.translation ()[0] = -originX;
    cloud_transform.translation ()[1] = -originY;
    cloud_transform.translation ()[2] = -originZ;
    
    transformPointCloud (*tsdf_clouds[i], *tsdf_clouds[i], cloud_transform);

    //Get mesh
    MeshPtr tmp = getMeshFromTSDFCloud (*tsdf_clouds[i]);
        
    if(tmp != 0)
    {
       meshes_vector.push_back (tmp);
       mesh_counter++;
    }
    else
    {
      PCL_INFO ("This cloud returned no faces, we skip it!\n");
      continue;
    }
    
    //Making cloud global
    cloud_transform.translation ()[0] = originX * cell_size;
    cloud_transform.translation ()[1] = originY * cell_size;
    cloud_transform.translation ()[2] = originZ * cell_size;
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tmp_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    fromPCLPointCloud2 ( (meshes_vector.back () )->cloud, *cloud_tmp_ptr);
    
    transformPointCloud (*cloud_tmp_ptr, *cloud_tmp_ptr, cloud_transform);
    
    toPCLPointCloud2 (*cloud_tmp_ptr, (meshes_vector.back () )->cloud);
    
    std::stringstream name;
    name << "mesh_" << mesh_counter << ".ply";
    PCL_INFO ("Saving mesh...%d \n", mesh_counter);
    pcl::io::savePLYFile (name.str (), *(meshes_vector.back ()));
    
  }
  return;
}

///////////////////////////////////////////////////////////////////////////////

template <typename PointT> pcl::gpu::kinfuLS::TsdfVolume::Ptr
pcl::gpu::kinfuLS::StandaloneMarchingCubes<PointT>::tsdfVolumeGPU ()
{
  return (tsdf_volume_gpu_);
}

///////////////////////////////////////////////////////////////////////////////

template <typename PointT> std::vector<int>& //todo
pcl::gpu::kinfuLS::StandaloneMarchingCubes<PointT>::tsdfVolumeCPU ()
{
  return (tsdf_volume_cpu_);
}

///////////////////////////////////////////////////////////////////////////////

template <typename PointT> void
pcl::gpu::kinfuLS::StandaloneMarchingCubes<PointT>::loadTsdfCloudToGPU (const PointCloud &cloud)
{
  //Converting Values
  convertTsdfVectors (cloud, tsdf_volume_cpu_);
  
  //Uploading data to GPU
	int cubeColumns = voxels_x_;
  tsdf_volume_gpu_->data ().upload (tsdf_volume_cpu_, cubeColumns);
}

///////////////////////////////////////////////////////////////////////////////

template <typename PointT> void 
pcl::gpu::kinfuLS::StandaloneMarchingCubes<PointT>::convertTsdfVectors (const PointCloud &cloud, std::vector<int> &output)
{
	  const int DIVISOR = 32767;     // SHRT_MAX;

    ///For every point in the cloud
#pragma omp parallel for
 	
	for(int i = 0; i < (int) cloud.points.size (); ++i)
	{
	  int x = cloud.points[i].x;
	  int y = cloud.points[i].y;
	  int z = cloud.points[i].z;
	  
	  if(x > 0  && x < voxels_x_ && y > 0 && y < voxels_y_ && z > 0 && z < voxels_z_)
	  {
	  ///Calculate the index to write to
	  int dst_index = x + voxels_x_ * y + voxels_y_ * voxels_x_ * z;
	        
	    short2& elem = *reinterpret_cast<short2*> (&output[dst_index]);
	    elem.x = static_cast<short> (cloud.points[i].intensity * DIVISOR);
	    elem.y = static_cast<short> (1);   
	  } 
  }
}

///////////////////////////////////////////////////////////////////////////////

template <typename PointT> typename pcl::gpu::kinfuLS::StandaloneMarchingCubes<PointT>::MeshPtr
pcl::gpu::kinfuLS::StandaloneMarchingCubes<PointT>::convertTrianglesToMesh (const pcl::gpu::DeviceArray<pcl::PointXYZ>& triangles)
{ 
  if (triangles.empty () )
  {
    return MeshPtr ();
  }

  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.width  = (int)triangles.size ();
  cloud.height = 1;
  triangles.download (cloud.points);

  boost::shared_ptr<pcl::PolygonMesh> mesh_ptr ( new pcl::PolygonMesh () ); 
  
  pcl::toPCLPointCloud2 (cloud, mesh_ptr->cloud);
      
  mesh_ptr->polygons.resize (triangles.size () / 3);
  for (size_t i = 0; i < mesh_ptr->polygons.size (); ++i)
  {
    pcl::Vertices v;
    v.vertices.push_back (i*3+0);
    v.vertices.push_back (i*3+2);
    v.vertices.push_back (i*3+1);              
    mesh_ptr->polygons[i] = v;
  }    
  return (mesh_ptr);
}

///////////////////////////////////////////////////////////////////////////////

template <typename PointT> typename pcl::gpu::kinfuLS::StandaloneMarchingCubes<PointT>::MeshPtr
pcl::gpu::kinfuLS::StandaloneMarchingCubes<PointT>::runMarchingCubes ()
{
  //Preparing the pointers and variables
  const TsdfVolume::Ptr tsdf_volume_const_ = tsdf_volume_gpu_;
  pcl::gpu::DeviceArray<pcl::PointXYZ> triangles_buffer_device_;
  
  //Creating Marching cubes instance
  MarchingCubes::Ptr marching_cubes_ = MarchingCubes::Ptr ( new MarchingCubes() );
  
  //Running marching cubes
  pcl::gpu::DeviceArray<pcl::PointXYZ> triangles_device = marching_cubes_->run (*tsdf_volume_const_, triangles_buffer_device_); 

  //Creating mesh
  boost::shared_ptr<pcl::PolygonMesh> mesh_ptr_ = convertTrianglesToMesh (triangles_device);

  if(mesh_ptr_ != 0)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tmp_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    fromPCLPointCloud2 ( mesh_ptr_->cloud, *cloud_tmp_ptr);
  }
  return (mesh_ptr_);
}

///////////////////////////////////////////////////////////////////////////////

#endif // PCL_STANDALONE_MARCHING_CUBES_IMPL_HPP_
 
