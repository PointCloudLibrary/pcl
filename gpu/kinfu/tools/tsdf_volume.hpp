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
pcl::TSDFVolume<VoxelT, WeightT>::convertToTsdfCloud (pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud) const
{
  int sx = header_.resolution(0);
  int sy = header_.resolution(1);
  int sz = header_.resolution(2);

  const int step = 2;
  const int cloud_size = header_.getVolumeSize() / (step*step*step);

  cloud->clear();
  cloud->reserve (std::min (cloud_size/10, 500000));

  int volume_idx = 0, cloud_idx = 0;
//#pragma omp parallel for // if used, increment over idx not possible! use index calculation
  for (int z = 0; z < sz; z+=step)
    for (int y = 0; y < sy; y+=step)
      for (int x = 0; x < sx; x+=step, ++cloud_idx)
      {
        volume_idx = sx*sy*z + sx*y + x;
        // pcl::PointXYZI &point = cloud->points[cloud_idx];

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


/** \brief Retunrs the 3D voxel coordinate and point offset wrt. to the voxel center (in mm) */
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
  #pragma omp parallel for
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
  #pragma omp parallel for
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
  #pragma omp parallel for
  for (size_t i = 0; i < volume_->size(); ++i)
  {
    WeightT &w = weights_->at(i);
    if (w > 0.0)
    {
      volume_->at(i) /= w;
      w = 1.0;
    }
  }
}


/*template <typename VoxelT, typename WeightT> template <typename PointT> void
pcl::TSDFVolume<VoxelT, WeightT>::createFromCloud (const typename pcl::PointCloud<PointT>::ConstPtr &cloud, const Intr &intr)
{
  // get depth map from cloud
  float bad_point = std::numeric_limits<float>::quiet_NaN ();
  // Eigen::MatrixXf depth = Eigen::MatrixXf::Constant(cloud_->height, cloud_->width, bad_point);
  Eigen::MatrixXf depth (cloud->height, cloud->width);

  // determine max and min value
  float min = 3000.0, max = 0.0;
  for (int x = 0; x < cloud->width; ++x)
    for (int y = 0; y < cloud->height; ++y)
    {
      depth(y,x) = cloud->at(x,y).z;
      if (!isnan(depth(y,x)))
      {
        if (depth(y,x) > max) max = depth(y,x);
        if (depth(y,x) < min) min = depth(y,x);
      }
    }

  std::cout << "  depth size = " << depth.rows() << "x" << depth.cols() << ", min/max = " << min << "/" << max << std::endl;


  // BOOST_FOREACH (const PointT &p, cloud->points)
  typename pcl::PointCloud<PointT>::const_iterator iter = cloud->begin();
  for (; iter != cloud_>end(); ++iter)
  {
    const PointT &p = *iter;

    std::cout << "orig point = " << p << std::endl;

    Eigen::Array2f point (intr.fx * p.x + intr.cx * p.z,
                          intr.fx * p.y + intr.cy * p.z);
    Eigen::Array2i pixel (round(point(0))/p.z, round(point(1))/p.z);

    std::cout << "point = " << point.transpose() << std::endl;
    std::cout << "pixel = " << pixel.transpose() << std::endl;

    depth (pixel(1), pixel(0)) = p.z;
  }

  std::cout << "  scaling depth map" << std::endl;
  // scale depth map
  Eigen::MatrixXf depth_scaled;
  // scaleDepth (depth, depth_scaled, intr);
  // TODO find out what this should do! projection on unit sphere?!
  depth_scaled = depth;

  // generate volume
  // std::cout << " generating volume" << std::endl;
  // resizeDefaultSize();
  Eigen::Vector3f volume_size = volumeSize();
  Eigen::Vector3f voxel_size = voxelSize();

  float tranc_dist = std::max (DEFAULT_TRANCATION_DISTANCE, 2.1f * voxel_size.maxCoeff());

  Eigen::Matrix3f R_inv_init = Eigen::Matrix3f::Identity();
  Eigen::Vector3f t_init =  volume_size * 0.5f - Eigen::Vector3f (0, 0, volume_size(2)/2.0f * 1.2f);
  // std::cout << "initial pose: R_inv = " << R_inv_init << ", t_init = " << t_init.transpose() << std::endl;

  std::cout << "  integrating values" << std::endl;
  integrateVolume (depth_scaled, tranc_dist, R_inv_init, t_init, intr);
}*/


/*template <typename VoxelT, typename WeightT> void
pcl::TSDFVolume<VoxelT, WeightT>::scaleDepth (const Eigen::MatrixXf &depth, Eigen::MatrixXf &depth_scaled, const Intr &intr) const
{
  // function ported from KinFu GPU code
  depth_scaled.resizeLike (depth);

  float min = 3000.0, max = 0.0;
  // loop over depth image
  for (int x = 0; x < depth.cols(); ++x)
    for (int y = 0; y < depth.rows(); ++y)
    {
      int Dp = depth(y,x);

      float xl = (x - intr.cx) / intr.fx;
      float yl = (y - intr.cy) / intr.fy;
      float lambda = sqrtf (xl * xl + yl * yl + 1);

      depth_scaled(y,x) = Dp * lambda;

      if (!isnan(depth_scaled(y,x)))
      {
        if (depth_scaled(y,x) > max) max = depth_scaled(y,x);
        if (depth_scaled(y,x) < min) min = depth_scaled(y,x);
      }
    }

  std::cout << "depth_scaled size = " << depth_scaled.rows() << "x" << depth_scaled.cols() << ", min/max = " << min << "/" << max << std::endl;
}*/


/*template <typename VoxelT, typename WeightT> void
pcl::TSDFVolume<VoxelT, WeightT>::integrateVolume (const Eigen::MatrixXf &depth_scaled,
                                              float tranc_dist,
                                              const Eigen::Matrix3f &R_inv,
                                              const Eigen::Vector3f &t,
                                              const Intr &intr)
{
  Eigen::Array3f voxel_size = voxelSize();
  Eigen::Array3i volume_res = gridResolution();
  Eigen::Array3f intr_arr (intr.fx, intr.fy, 1.0f);
  Eigen::Array3i voxel_coord (0,0,0);

  // loop over grid in X and Y dimension
  #pragma omp parallel for
  // for (voxel_coord(0) = 0; voxel_coord(0) < volume_res(0); ++voxel_coord(0))
  for (int i = 0; i < volume_res(0); ++i)
  {
    voxel_coord(0) = i;

    // std::stringstream ss;
    // ss << voxel_coord(0) << "/" << volume_res(0) << " ";
    // std::cout << ss.str();
    std::cout << ". " << std::flush;

    for (voxel_coord(1) = 0; voxel_coord(1) < volume_res(1); ++voxel_coord(1))
    {
      voxel_coord(2) = 0;
      // points at depth 0, shifted by t
      Eigen::Vector3f v_g = (voxel_coord.cast<float>() + 0.5f) * voxel_size - t.array();
      float v_g_part_norm = v_g(0)*v_g(0) + v_g(1)*v_g(1);

      // rays in 3d
      Eigen::Vector3f v = (R_inv * v_g).array() * intr_arr;

      float z_scaled = 0;

      Eigen::Array3f R_inv_z_scaled = R_inv.col(2).array() * voxel_size(2) * intr_arr;

      float tranc_dist_inv = 1.0f / tranc_dist;

      // loop over depth values
      for (voxel_coord(2) = 0; voxel_coord(2) < volume_res(2); ++voxel_coord(2),
           v_g(2) += voxel_size(2),
           z_scaled += voxel_size(2),
           v(0) += R_inv_z_scaled(0),
           v(1) += R_inv_z_scaled(1))
      {
        float inv_z = 1.0f / (v(2) + R_inv(2,2) * z_scaled);

        // std::cout << "z = " << voxel_coord(2) << ", inv_z = " << inv_z << std::endl;

        if (inv_z < 0)
          continue;

        // project to camera
        Eigen::Array2i img_coord (round(v(0) * inv_z + intr.cx),
                                  round(v(1) * inv_z + intr.cy));

        // std::cout << "img_coord = " << img_coord.transpose() << std::endl;

        if (img_coord(0) >= 0 && img_coord(1) >= 0 && img_coord(0) < depth_scaled.cols() && img_coord(1) < depth_scaled.rows())         //6
        {
          float Dp_scaled = depth_scaled(img_coord(1), img_coord(0));

          // signed distance function
          float sdf = Dp_scaled - sqrtf (v_g(2) * v_g(2) + v_g_part_norm);

          if (Dp_scaled != 0 && sdf >= -tranc_dist)
          {
            // get truncated distance function value
            float tsdf = fmin (1.0f, sdf * tranc_dist_inv);

            // add values to volume
            int idx = getLinearVoxelIndex (voxel_coord);
            VoxelT &tsdf_val = volume_->at(idx);
            short  &weight   = weights_->at(idx);
            tsdf_val = tsdf_val * weight + tsdf;
            weight += 1;
          }
        }
      } // loop over depths
    }
  }
  std::cout << std::endl;
}*/

#define PCL_INSTANTIATE_TSDFVolume(VT,WT) template class PCL_EXPORTS pcl::reconstruction::TSDFVolume<VT,WT>;

#endif /* TSDF_VOLUME_HPP_ */
