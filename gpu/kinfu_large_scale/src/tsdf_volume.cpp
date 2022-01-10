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

#include <pcl/gpu/kinfu_large_scale/tsdf_volume.h>
#include "internal.h"
#include <algorithm>
#include <Eigen/Core>

#include <iostream>

using namespace pcl;
using namespace pcl::gpu;
using namespace Eigen;
using pcl::device::kinfuLS::device_cast;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

pcl::gpu::kinfuLS::TsdfVolume::TsdfVolume(const Vector3i& resolution) : resolution_(resolution), volume_host_ (new std::vector<float>), weights_host_ (new std::vector<short>)
{
  int volume_x = resolution_(0);
  int volume_y = resolution_(1);
  int volume_z = resolution_(2);

  volume_.create (volume_y * volume_z, volume_x);
  
  const Vector3f default_volume_size = Vector3f::Constant (3.f); //meters
  const float    default_tranc_dist  = 0.03f; //meters

  setSize(default_volume_size);
  setTsdfTruncDist(default_tranc_dist);

  reset();
  
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void
pcl::gpu::kinfuLS::TsdfVolume::setSize(const Vector3f& size)
{  
  size_ = size;
  setTsdfTruncDist(tranc_dist_);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void
pcl::gpu::kinfuLS::TsdfVolume::setTsdfTruncDist (float distance)
{
  float cx = size_(0) / resolution_(0);
  float cy = size_(1) / resolution_(1);
  float cz = size_(2) / resolution_(2);

  tranc_dist_ = std::max (distance, 2.1f * std::max (cx, std::max (cy, cz)));  
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

pcl::gpu::DeviceArray2D<int> 
pcl::gpu::kinfuLS::TsdfVolume::data() const
{
  return volume_;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

const Eigen::Vector3f&
pcl::gpu::kinfuLS::TsdfVolume::getSize() const
{
    return size_;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

const Eigen::Vector3i&
pcl::gpu::kinfuLS::TsdfVolume::getResolution() const
{
  return resolution_;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

const Eigen::Vector3f
pcl::gpu::kinfuLS::TsdfVolume::getVoxelSize() const
{    
  return size_.array () / resolution_.array().cast<float>();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

float
pcl::gpu::kinfuLS::TsdfVolume::getTsdfTruncDist () const
{
  return tranc_dist_;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void 
pcl::gpu::kinfuLS::TsdfVolume::reset()
{
  pcl::device::kinfuLS::initVolume(volume_);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void
pcl::gpu::kinfuLS::TsdfVolume::fetchCloudHost (PointCloud<PointXYZI>& cloud, bool connected26) const
{
  PointCloud<PointXYZ>::Ptr cloud_ptr_ = PointCloud<PointXYZ>::Ptr (new PointCloud<PointXYZ>);
  PointCloud<PointIntensity>::Ptr cloud_i_ptr_ = PointCloud<PointIntensity>::Ptr (new PointCloud<PointIntensity>);
  fetchCloudHost(*cloud_ptr_);
  pcl::concatenateFields (*cloud_ptr_, *cloud_i_ptr_, cloud);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void
pcl::gpu::kinfuLS::TsdfVolume::fetchCloudHost (PointCloud<PointType>& cloud, bool connected26) const
{
  int volume_x = resolution_(0);
  int volume_y = resolution_(1);
  int volume_z = resolution_(2);

  int cols;
  std::vector<int> volume_host;
  volume_.download (volume_host, cols);

  cloud.clear ();
  cloud.reserve (10000);

  constexpr int DIVISOR = pcl::device::kinfuLS::DIVISOR; // std::numeric_limits<short>::max();

#define FETCH(x, y, z) volume_host[(x) + (y) * volume_x + (z) * volume_y * volume_x]

  Array3f cell_size = getVoxelSize();

  for (int x = 1; x < volume_x-1; ++x)
  {
    for (int y = 1; y < volume_y-1; ++y)
    {
      for (int z = 0; z < volume_z-1; ++z)
      {
        int tmp = FETCH (x, y, z);
        int W = reinterpret_cast<short2*>(&tmp)->y;
        int F = reinterpret_cast<short2*>(&tmp)->x;

        if (W == 0 || F == DIVISOR)
          continue;

        Vector3f V = ((Array3f(x, y, z) + 0.5f) * cell_size).matrix ();

        if (connected26)
        {
          int dz = 1;
          for (int dy = -1; dy < 2; ++dy)
            for (int dx = -1; dx < 2; ++dx)
            {
              int tmp = FETCH (x+dx, y+dy, z+dz);

              int Wn = reinterpret_cast<short2*>(&tmp)->y;
              int Fn = reinterpret_cast<short2*>(&tmp)->x;
              if (Wn == 0 || Fn == DIVISOR)
                continue;

              if ((F > 0 && Fn < 0) || (F < 0 && Fn > 0))
              {
                Vector3f Vn = ((Array3f (x+dx, y+dy, z+dz) + 0.5f) * cell_size).matrix ();
                Vector3f point = (V * std::abs (Fn) + Vn * std::abs (F)) / (std::abs (F) + std::abs (Fn));

                pcl::PointXYZ xyz;
                xyz.x = point (0);
                xyz.y = point (1);
                xyz.z = point (2);

                cloud.push_back (xyz);
              }
            }
          dz = 0;
          for (int dy = 0; dy < 2; ++dy)
            for (int dx = -1; dx < dy * 2; ++dx)
            {
              int tmp = FETCH (x+dx, y+dy, z+dz);

              int Wn = reinterpret_cast<short2*>(&tmp)->y;
              int Fn = reinterpret_cast<short2*>(&tmp)->x;
              if (Wn == 0 || Fn == DIVISOR)
                continue;

              if ((F > 0 && Fn < 0) || (F < 0 && Fn > 0))
              {
                Vector3f Vn = ((Array3f (x+dx, y+dy, z+dz) + 0.5f) * cell_size).matrix ();
                Vector3f point = (V * std::abs(Fn) + Vn * std::abs(F))/(std::abs(F) + std::abs (Fn));

                pcl::PointXYZ xyz;
                xyz.x = point (0);
                xyz.y = point (1);
                xyz.z = point (2);

                cloud.push_back (xyz);
              }
            }
        }
        else /* if (connected26) */
        {
          for (int i = 0; i < 3; ++i)
          {
            int ds[] = {0, 0, 0};
            ds[i] = 1;

            int dx = ds[0];
            int dy = ds[1];
            int dz = ds[2];

            int tmp = FETCH (x+dx, y+dy, z+dz);

            int Wn = reinterpret_cast<short2*>(&tmp)->y;
            int Fn = reinterpret_cast<short2*>(&tmp)->x;
            if (Wn == 0 || Fn == DIVISOR)
              continue;

            if ((F > 0 && Fn < 0) || (F < 0 && Fn > 0))
            {
              Vector3f Vn = ((Array3f (x+dx, y+dy, z+dz) + 0.5f) * cell_size).matrix ();
              Vector3f point = (V * std::abs (Fn) + Vn * std::abs (F)) / (std::abs (F) + std::abs (Fn));

              pcl::PointXYZ xyz;
              xyz.x = point (0);
              xyz.y = point (1);
              xyz.z = point (2);

              cloud.push_back (xyz);
            }
          }
        } /* if (connected26) */
      }
    }
  }
#undef FETCH
  cloud.width  = cloud.size ();
  cloud.height = 1;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
pcl::gpu::DeviceArray<pcl::gpu::kinfuLS::TsdfVolume::PointType>
pcl::gpu::kinfuLS::TsdfVolume::fetchCloud (DeviceArray<PointType>& cloud_buffer) const
{
  if (cloud_buffer.empty ())
    cloud_buffer.create (DEFAULT_CLOUD_BUFFER_SIZE);

  float3 device_volume_size = device_cast<const float3> (size_);
  std::size_t size = pcl::device::kinfuLS::extractCloud (volume_, device_volume_size, cloud_buffer);
  return (DeviceArray<PointType> (cloud_buffer.ptr (), size));
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void
pcl::gpu::kinfuLS::TsdfVolume::fetchNormals (const DeviceArray<PointType>& cloud, DeviceArray<PointType>& normals) const
{
  normals.create (cloud.size ());
  const float3 device_volume_size = device_cast<const float3> (size_);
  pcl::device::kinfuLS::extractNormals (volume_, device_volume_size, cloud, (pcl::device::kinfuLS::PointType*)normals.ptr ());
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void 
pcl::gpu::kinfuLS::TsdfVolume::pushSlice (PointCloud<PointXYZI>::Ptr existing_data_cloud, const pcl::gpu::kinfuLS::tsdf_buffer* buffer) const
{
  const auto gpu_array_size = existing_data_cloud->size ();

  if(gpu_array_size == 0)
  {
    //std::cout << "[KinfuTracker](pushSlice) Existing data cloud has no points\n";//CREATE AS PCL MESSAGE
    return;
  }

  const pcl::PointXYZI *first_point_ptr = &((*existing_data_cloud)[0]);

  pcl::gpu::DeviceArray<pcl::PointXYZI> cloud_gpu;
  cloud_gpu.upload (first_point_ptr, gpu_array_size);

  DeviceArray<float4>& cloud_cast = (DeviceArray<float4>&) cloud_gpu;
  //volume().pushCloudAsSlice (cloud_cast, &buffer_);
  pcl::device::kinfuLS::pushCloudAsSliceGPU (volume_, cloud_cast, buffer);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

size_t
pcl::gpu::kinfuLS::TsdfVolume::fetchSliceAsCloud (DeviceArray<PointType>& cloud_buffer_xyz, DeviceArray<float>& cloud_buffer_intensity, const pcl::gpu::kinfuLS::tsdf_buffer* buffer, int shiftX, int shiftY, int shiftZ ) const
{
  if (cloud_buffer_xyz.empty ())
    cloud_buffer_xyz.create (DEFAULT_CLOUD_BUFFER_SIZE/2);

  if (cloud_buffer_intensity.empty ()) {
    cloud_buffer_intensity.create (DEFAULT_CLOUD_BUFFER_SIZE/2);  
  }

  float3 device_volume_size = device_cast<const float3> (size_);
  
  std::size_t size = pcl::device::kinfuLS::extractSliceAsCloud (volume_, device_volume_size, buffer, shiftX, shiftY, shiftZ, cloud_buffer_xyz, cloud_buffer_intensity);
  
  std::cout << " SIZE IS " << size << std::endl;
  
  return (size);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void
pcl::gpu::kinfuLS::TsdfVolume::fetchNormals (const DeviceArray<PointType>& cloud, DeviceArray<NormalType>& normals) const
{
  normals.create (cloud.size ());
  const float3 device_volume_size = device_cast<const float3> (size_);
  pcl::device::kinfuLS::extractNormals (volume_, device_volume_size, cloud, (pcl::device::kinfuLS::float8*)normals.ptr ());
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void
pcl::gpu::kinfuLS::TsdfVolume::convertToTsdfCloud ( pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                                                    const unsigned step) const
{
  int sx = header_.resolution(0);
  int sy = header_.resolution(1);
  int sz = header_.resolution(2);

  const int cloud_size = static_cast<int> (header_.getVolumeSize() / (step*step*step));

  cloud->clear();
  cloud->reserve (std::min (cloud_size/10, 500000));

  int volume_idx = 0, cloud_idx = 0;
  for (int z = 0; z < sz; z+=step)
    for (int y = 0; y < sy; y+=step)
      for (int x = 0; x < sx; x+=step, ++cloud_idx)
      {
        volume_idx = sx*sy*z + sx*y + x;
        // pcl::PointXYZI &point = (*cloud)[cloud_idx];

        if (weights_host_->at(volume_idx) == 0 || volume_host_->at(volume_idx) > 0.98 )
          continue;

        pcl::PointXYZI point;
        point.x = x; point.y = y; point.z = z;//*64;
        point.intensity = volume_host_->at(volume_idx);
        cloud->push_back (point);
      }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void
pcl::gpu::kinfuLS::TsdfVolume::downloadTsdf (std::vector<float>& tsdf) const
{
  tsdf.resize (volume_.cols() * volume_.rows());
  volume_.download(&tsdf[0], volume_.cols() * sizeof(int));

#pragma omp parallel for \
  default(none) \
  shared(tsdf)
  for(int i = 0; i < (int) tsdf.size(); ++i)
  {
    float tmp = reinterpret_cast<short2*>(&tsdf[i])->x;
    tsdf[i] = tmp/pcl::device::kinfuLS::DIVISOR;
  }
}

void
pcl::gpu::kinfuLS::TsdfVolume::downloadTsdfLocal () const
{
  pcl::gpu::kinfuLS::TsdfVolume::downloadTsdf (*volume_host_);
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void
pcl::gpu::kinfuLS::TsdfVolume::downloadTsdfAndWeights (std::vector<float>& tsdf, std::vector<short>& weights) const
{
  int volumeSize = volume_.cols() * volume_.rows();
  tsdf.resize (volumeSize);
  weights.resize (volumeSize);
  volume_.download(&tsdf[0], volume_.cols() * sizeof(int));
  
  #pragma omp parallel for \
    default(none) \
    shared(tsdf, weights)
  for(int i = 0; i < (int) tsdf.size(); ++i)
  {
    short2 elem = *reinterpret_cast<short2*>(&tsdf[i]);
    tsdf[i] = (float)(elem.x)/pcl::device::kinfuLS::DIVISOR;    
    weights[i] = (short)(elem.y);    
  }
}


void
pcl::gpu::kinfuLS::TsdfVolume::downloadTsdfAndWeightsLocal () const
{
  pcl::gpu::kinfuLS::TsdfVolume::downloadTsdfAndWeights (*volume_host_, *weights_host_);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool
pcl::gpu::kinfuLS::TsdfVolume::save (const std::string &filename, bool binary) const
{
  pcl::console::print_info ("Saving TSDF volume to "); pcl::console::print_value ("%s ... ", filename.c_str());
  std::cout << std::flush;

  std::ofstream file (filename.c_str(), binary ? std::ios_base::binary : std::ios_base::out);

  if (file.is_open())
  {
    if (binary)
    {
      // HEADER
      // write resolution and size of volume
      file.write ((char*) &header_, sizeof (Header));
      /* file.write ((char*) &header_.resolution, sizeof(Eigen::Vector3i));
      file.write ((char*) &header_.volume_size, sizeof(Eigen::Vector3f));
      // write  element size
      int volume_element_size = sizeof(VolumeT);
      file.write ((char*) &volume_element_size, sizeof(int));
      int weights_element_size = sizeof(WeightT);
      file.write ((char*) &weights_element_size, sizeof(int)); */

      // DATA
      // write data
      file.write ((char*) &(volume_host_->at(0)), volume_host_->size()*sizeof(float));
      file.write ((char*) &(weights_host_->at(0)), weights_host_->size()*sizeof(short));
    }
    else
    {
      // write resolution and size of volume and element size
      file << header_.resolution(0) << " " << header_.resolution(1) << " " << header_.resolution(2) << std::endl;
      file << header_.volume_size(0) << " " << header_.volume_size(1) << " " << header_.volume_size(2) << std::endl;
      file << sizeof (float) << " " << sizeof(short) << std::endl;

      // write data
      for (std::vector<float>::const_iterator iter = volume_host_->begin(); iter != volume_host_->end(); ++iter)
        file << *iter << std::endl;
    }

    file.close();
  }
  else
  {
    pcl::console::print_error ("[saveTsdfVolume] Error: Couldn't open file %s.\n", filename.c_str());
    return false;
  }

  pcl::console::print_info ("done [%d voxels]\n", this->size ());

  return true;
}


bool
pcl::gpu::kinfuLS::TsdfVolume::load (const std::string &filename, bool binary)
{
  pcl::console::print_info ("Loading TSDF volume from "); pcl::console::print_value ("%s ... ", filename.c_str());
  std::cout << std::flush;

  std::ifstream file (filename.c_str());

  if (file.is_open())
  {
    if (binary)
    {
      // read HEADER
      file.read ((char*) &header_, sizeof (Header));
      /* file.read (&header_.resolution, sizeof(Eigen::Array3i));
      file.read (&header_.volume_size, sizeof(Eigen::Vector3f));
      file.read (&header_.volume_element_size, sizeof(int));
      file.read (&header_.weights_element_size, sizeof(int)); */

      // check if element size fits to data
      if (header_.volume_element_size != sizeof(float))
      {
        pcl::console::print_error ("[TSDFVolume::load] Error: Given volume element size (%d) doesn't fit data (%d)", sizeof(float), header_.volume_element_size);
        return false;
      }
      if ( header_.weights_element_size != sizeof(short))
      {
        pcl::console::print_error ("[TSDFVolume::load] Error: Given weights element size (%d) doesn't fit data (%d)", sizeof(short), header_.weights_element_size);
        return false;
      }

      // read DATA
      int num_elements = header_.getVolumeSize();
      volume_host_->resize (num_elements);
      weights_host_->resize (num_elements);
      file.read ((char*) &(*volume_host_)[0], num_elements * sizeof(float));
      file.read ((char*) &(*weights_host_)[0], num_elements * sizeof(short));
    }
    else
    {
      pcl::console::print_error ("[TSDFVolume::load] Error: ASCII loading not implemented.\n");
    }

    file.close ();
  }
  else
  {
    pcl::console::print_error ("[TSDFVolume::load] Error: Cloudn't read file %s.\n", filename.c_str());
    return false;
  }

  const Eigen::Vector3i &res = this->gridResolution();
  pcl::console::print_info ("done [%d voxels, res %dx%dx%d]\n", this->size(), res[0], res[1], res[2]);

  return true;
}
