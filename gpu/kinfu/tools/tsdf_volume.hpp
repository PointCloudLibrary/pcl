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
 *  $Id$
 */

#ifndef TSDF_VOLUME_HPP_
#define TSDF_VOLUME_HPP_

#include "tsdf_volume.h"

#include <fstream>


template <typename VoxelT, typename WeightT> bool
pcl::TSDFVolume<VoxelT, WeightT>::load (const std::string &filename, bool binary)
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
      if (header_.volume_element_size != sizeof(VoxelT))
      {
        pcl::console::print_error ("[TSDFVolume::load] Error: Given volume element size (%d) doesn't fit data (%d)", sizeof(VoxelT), header_.volume_element_size);
        return false;
      }
      if ( header_.weights_element_size != sizeof(WeightT))
      {
        pcl::console::print_error ("[TSDFVolume::load] Error: Given weights element size (%d) doesn't fit data (%d)", sizeof(WeightT), header_.weights_element_size);
        return false;
      }

      // read DATA
      int num_elements = header_.getVolumeSize();
      volume_->resize (num_elements);
      weights_->resize (num_elements);
      file.read ((char*) &(*volume_)[0], num_elements * sizeof(VoxelT));
      file.read ((char*) &(*weights_)[0], num_elements * sizeof(WeightT));
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


template <typename VoxelT, typename WeightT> bool
pcl::TSDFVolume<VoxelT, WeightT>::save (const std::string &filename, bool binary) const
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
      file.write ((char*) &(volume_->at(0)), volume_->size()*sizeof(VoxelT));
      file.write ((char*) &(weights_->at(0)), weights_->size()*sizeof(WeightT));
    }
    else
    {
      // write resolution and size of volume and element size
      file << header_.resolution(0) << " " << header_.resolution(1) << " " << header_.resolution(2) << std::endl;
      file << header_.volume_size(0) << " " << header_.volume_size(1) << " " << header_.volume_size(2) << std::endl;
      file << sizeof (VoxelT) << " " << sizeof(WeightT) << std::endl;

      // write data
      for (typename std::vector<VoxelT>::const_iterator iter = volume_->begin(); iter != volume_->end(); ++iter)
        file << *iter << std::endl;
    }

    file.close();
  }
  else
  {
    pcl::console::print_error ("[saveTsdfVolume] Error: Couldn't open file %s.\n", filename.c_str());
    return false;
  }

  pcl::console::print_info ("done [%d voxels]\n", this->size());

  return true;
}


template <typename VoxelT, typename WeightT> void
pcl::TSDFVolume<VoxelT, WeightT>::convertToTsdfCloud (pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                                                      const unsigned step) const
{
  int sx = header_.resolution(0);
  int sy = header_.resolution(1);
  int sz = header_.resolution(2);

  const int cloud_size = header_.getVolumeSize() / (step*step*step);

  cloud->clear();
  cloud->reserve (std::min (cloud_size/10, 500000));

  int volume_idx = 0, cloud_idx = 0;
  for (int z = 0; z < sz; z+=step)
    for (int y = 0; y < sy; y+=step)
      for (int x = 0; x < sx; x+=step, ++cloud_idx)
      {
        volume_idx = sx*sy*z + sx*y + x;
        // pcl::PointXYZI &point = (*cloud)[cloud_idx];

        if (weights_->at(volume_idx) == 0 || volume_->at(volume_idx) > 0.98 )
          continue;

        pcl::PointXYZI point;
        point.x = x; point.y = y; point.z = z;//*64;
        point.intensity = volume_->at(volume_idx);
        cloud->push_back (point);
      }

  // cloud->width = cloud_size;
  // cloud->height = 1;
}


template <typename VoxelT, typename WeightT> template <typename PointT> void
pcl::TSDFVolume<VoxelT, WeightT>::getVoxelCoord (const PointT &point, Eigen::Vector3i &coord) const
{
  static Eigen::Array3f voxel_size = voxelSize().array();

  // point coordinates in world coordinate frame and voxel coordinates
  Eigen::Array3f point_coord (point.x, point.y, point.z);
  Eigen::Array3f voxel_coord = (point_coord / voxel_size) - 0.5f; // 0.5f offset due to voxel center vs grid
  coord(0) = round(voxel_coord(0));
  coord(1) = round(voxel_coord(1));
  coord(2) = round(voxel_coord(2));
}


/** \brief Returns the 3D voxel coordinate and point offset wrt. to the voxel center (in mm) */
template <typename VoxelT, typename WeightT> template <typename PointT> void
pcl::TSDFVolume<VoxelT, WeightT>::getVoxelCoordAndOffset (const PointT &point,
                                                                          Eigen::Vector3i &coord, Eigen::Vector3f &offset) const
{
  static Eigen::Array3f voxel_size = voxelSize().array();

  // point coordinates in world coordinate frame and voxel coordinates
  Eigen::Array3f point_coord (point.x, point.y, point.z);
  Eigen::Array3f voxel_coord = (point_coord / voxel_size) - 0.5f; // 0.5f offset due to voxel center vs grid
  coord(0) = round(voxel_coord(0));
  coord(1) = round(voxel_coord(1));
  coord(2) = round(voxel_coord(2));

  // offset of point wrt. to voxel center
  offset = (voxel_coord - coord.cast<float>().array() * voxel_size).matrix();
}


template <typename VoxelT, typename WeightT> bool
pcl::TSDFVolume<VoxelT, WeightT>::extractNeighborhood (const Eigen::Vector3i &voxel_coord, int neighborhood_size,
                                                                       VoxelTVec &neighborhood) const
{
  // point_index is at the center of a cube of scale_ x scale_ x scale_ voxels
  int shift = (neighborhood_size - 1) / 2;
  Eigen::Vector3i min_index = voxel_coord.array() - shift;
  Eigen::Vector3i max_index = voxel_coord.array() + shift;

  // check that index is within range
  if (getLinearVoxelIndex(min_index) < 0 || getLinearVoxelIndex(max_index) >= (int)size())
  {
    pcl::console::print_info ("[extractNeighborhood] Skipping voxel with coord (%d, %d, %d).\n", voxel_coord(0), voxel_coord(1), voxel_coord(2));
    return false;
  }

  static const int descriptor_size = neighborhood_size*neighborhood_size*neighborhood_size;
  neighborhood.resize (descriptor_size);

  const Eigen::RowVector3i offset_vector (1, neighborhood_size, neighborhood_size*neighborhood_size);

  // loop over all voxels in 3D neighborhood
  #pragma omp parallel for \
    default(none)
  for (int z = min_index(2); z <= max_index(2); ++z)
  {
    for (int y = min_index(1); y <= max_index(1); ++y)
    {
      for (int x = min_index(0); x <= max_index(0); ++x)
      {
        // linear voxel index in volume and index in descriptor vector
        Eigen::Vector3i point (x,y,z);
        int volume_idx = getLinearVoxelIndex (point);
        int descr_idx  = offset_vector * (point - min_index);

/*        std::cout << "linear index " << volume_idx << std::endl;
        std::cout << "weight " << weights_->at (volume_idx) << std::endl;
        std::cout << "volume " << volume_->at (volume_idx) << std::endl;
        std::cout << "descr  " << neighborhood.rows() << "x" << neighborhood.cols() << ", val = " << neighborhood << std::endl;
        std::cout << "descr index = " << descr_idx << std::endl;
*/
        // get the TSDF value and store as descriptor entry
        if (weights_->at (volume_idx) != 0)
          neighborhood (descr_idx) = volume_->at (volume_idx);
        else
          neighborhood (descr_idx) = -1.0; // if never visited we assume inside of object (outside captured and thus filled with positive values)
      }
    }
  }

  return true;
}


template <typename VoxelT, typename WeightT> bool
pcl::TSDFVolume<VoxelT, WeightT>::addNeighborhood (const Eigen::Vector3i &voxel_coord, int neighborhood_size,
                                                                   const VoxelTVec &neighborhood, WeightT voxel_weight)
{
  // point_index is at the center of a cube of scale_ x scale_ x scale_ voxels
  int shift = (neighborhood_size - 1) / 2;
  Eigen::Vector3i min_index = voxel_coord.array() - shift;
  Eigen::Vector3i max_index = voxel_coord.array() + shift;

  // check that index is within range
  if (getLinearVoxelIndex(min_index) < 0 || getLinearVoxelIndex(max_index) >= (int)size())
  {
    pcl::console::print_info ("[addNeighborhood] Skipping voxel with coord (%d, %d, %d).\n", voxel_coord(0), voxel_coord(1), voxel_coord(2));
    return false;
  }

  // static const int descriptor_size = neighborhood_size*neighborhood_size*neighborhood_size;
  const Eigen::RowVector3i offset_vector (1, neighborhood_size, neighborhood_size*neighborhood_size);

  Eigen::Vector3i index = min_index;
  // loop over all voxels in 3D neighborhood
  #pragma omp parallel for \
    default(none)
  for (int z = min_index(2); z <= max_index(2); ++z)
  {
    for (int y = min_index(1); y <= max_index(1); ++y)
    {
      for (int x = min_index(0); x <= max_index(0); ++x)
      {
        // linear voxel index in volume and index in descriptor vector
        Eigen::Vector3i point (x,y,z);
        int volume_idx = getLinearVoxelIndex (point);
        int descr_idx  = offset_vector * (point - min_index);

        // add the descriptor entry to the volume
        VoxelT &voxel = volume_->at (volume_idx);
        WeightT &weight = weights_->at (volume_idx);

        // TODO check that this simple lock works correctly!!
        #pragma omp atomic
        voxel += neighborhood (descr_idx);

        #pragma omp atomic
        weight += voxel_weight;
      }
    }
  }

  return true;
}

template <typename VoxelT, typename WeightT> void
pcl::TSDFVolume<VoxelT, WeightT>::averageValues ()
{
  #pragma omp parallel for \
    default(none)
  for (std::size_t i = 0; i < volume_->size(); ++i)
  {
    WeightT &w = weights_->at(i);
    if (w > 0.0)
    {
      volume_->at(i) /= w;
      w = 1.0;
    }
  }
}

#define PCL_INSTANTIATE_TSDFVolume(VT,WT) template class PCL_EXPORTS pcl::reconstruction::TSDFVolume<VT,WT>;

#endif /* TSDF_VOLUME_HPP_ */
