/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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

#pragma once

#include <pcl/point_types.h>
#include <pcl/gpu/kinfu_large_scale/tsdf_volume.h>
#include <pcl/gpu/kinfu_large_scale/tsdf_buffer.h>
#include <Eigen/Core>
//#include <boost/graph/buffer_concepts.hpp>
#include <cuda_runtime.h>
#include <pcl/gpu/kinfu_large_scale/point_intensity.h>

#include <pcl/gpu/kinfu_large_scale/world_model.h>


#include <pcl/io/pcd_io.h>
namespace pcl
{
  namespace gpu
  {
    namespace kinfuLS
    { 
        
      /** \brief CyclicalBuffer implements a cyclical TSDF buffer.
        *  The class offers a simple interface, by handling shifts and maintaining the world autonomously.
        * \author Raphael Favier, Francisco Heredia
        */
      class PCL_EXPORTS CyclicalBuffer
      {
        public:
        
          
          /** \brief Constructor for a cubic CyclicalBuffer.
            * \param[in] distance_threshold distance between cube center and target point at which we decide to shift.
            * \param[in] cube_size physical size (in meters) of the volume (here, a cube) represented by the TSDF buffer.
            * \param[in] nb_voxels_per_axis number of voxels per axis of the volume represented by the TSDF buffer.
            */
          CyclicalBuffer (const double distance_threshold, const double cube_size = 3.f, const int nb_voxels_per_axis = 512)
          {
            distance_threshold_ = distance_threshold;
            buffer_.volume_size.x = cube_size; 
            buffer_.volume_size.y = cube_size; 
            buffer_.volume_size.z = cube_size;
            buffer_.voxels_size.x = nb_voxels_per_axis; 
            buffer_.voxels_size.y = nb_voxels_per_axis; 
            buffer_.voxels_size.z = nb_voxels_per_axis; 
          }


          /** \brief Constructor for a non-cubic CyclicalBuffer.
            * \param[in] distance_threshold distance between cube center and target point at which we decide to shift.
            * \param[in] volume_size_x physical size (in meters) of the volume, X axis.
            * \param[in] volume_size_y physical size (in meters) of the volume, Y axis.
            * \param[in] volume_size_z physical size (in meters) of the volume, Z axis.
            * \param[in] nb_voxels_x number of voxels for X axis of the volume represented by the TSDF buffer.
            * \param[in] nb_voxels_y number of voxels for Y axis of the volume represented by the TSDF buffer.
            * \param[in] nb_voxels_z number of voxels for Z axis of the volume represented by the TSDF buffer.
            */
          CyclicalBuffer (const double distance_threshold, const double volume_size_x, const double volume_size_y, const double volume_size_z, const int nb_voxels_x, const int nb_voxels_y, const int nb_voxels_z)
          {
            distance_threshold_ = distance_threshold;
            buffer_.volume_size.x = volume_size_x; 
            buffer_.volume_size.y = volume_size_y; 
            buffer_.volume_size.z = volume_size_z;
            buffer_.voxels_size.x = nb_voxels_x; 
            buffer_.voxels_size.y = nb_voxels_y; 
            buffer_.voxels_size.z = nb_voxels_z; 
          }

          /** \brief Check if shifting needs to be performed, returns true if so.
              Shifting is considered needed if the target point is farther than distance_threshold_.
              The target point is located at distance_camera_point on the local Z axis of the camera.
            * \param[in] volume pointer to the TSDFVolume living in GPU
            * \param[in] cam_pose global pose of the camera in the world
            * \param[in] distance_camera_target distance from the camera's origin to the target point
            * \param[in] perform_shift if set to false, shifting is not performed. The function will return true if shifting is needed.
            * \param[in] last_shift if set to true, the whole cube will be shifted. This is used to push the whole cube to the world model.
            * \param[in] force_shift if set to true, shifting is forced.
            * \return true is the cube needs to be or has been shifted.
            */
          bool checkForShift (const TsdfVolume::Ptr volume, const Eigen::Affine3f &cam_pose, const double distance_camera_target, const bool perform_shift = true, const bool last_shift = false, const bool force_shift = false);
          
          /** \brief Perform shifting operations:
              Compute offsets.
              Extract current slice from TSDF buffer.
              Extract existing data from world.
              Clear shifted slice in TSDF buffer.
              Push existing data into TSDF buffer.
              Update rolling buffer
              Update world model. 
            * \param[in] volume pointer to the TSDFVolume living in GPU
            * \param[in] target_point target point around which the new cube will be centered
            * \param[in] last_shift if set to true, the whole cube will be shifted. This is used to push the whole cube to the world model.
            */
          void performShift (const TsdfVolume::Ptr volume, const pcl::PointXYZ &target_point, const bool last_shift = false);

          /** \brief Sets the distance threshold between cube's center and target point that triggers a shift.
            * \param[in] threshold the distance in meters at which to trigger shift.
            */
          void setDistanceThreshold (const double threshold) 
          { 
            distance_threshold_ = threshold; 
            // PCL_INFO ("Shifting threshold set to %f meters.\n", distance_threshold_);
          }

          /** \brief Returns the distance threshold between cube's center and target point that triggers a shift. */
          float getDistanceThreshold () { return (distance_threshold_); }

          /** \brief get a pointer to the tsdf_buffer structure.
            * \return a pointer to the tsdf_buffer used by cyclical buffer object.
            */
          tsdf_buffer* getBuffer () { return (&buffer_); }

          /** \brief Set the physical size represented by the default TSDF volume.
          * \param[in] size_x size of the volume on X axis, in meters.
          * \param[in] size_y size of the volume on Y axis, in meters.
          * \param[in] size_z size of the volume on Z axis, in meters.
          */ 
          void setVolumeSize (const double size_x, const double size_y, const double size_z)
          {
            buffer_.volume_size.x = size_x;
            buffer_.volume_size.y = size_y;
            buffer_.volume_size.z = size_z;
          }

          /** \brief Set the physical size represented by the default TSDF volume.
          * \param[in] size size of the volume on all axis, in meters.
          */
          void setVolumeSize (const double size)
          {
            buffer_.volume_size.x = size;
            buffer_.volume_size.y = size;
            buffer_.volume_size.z = size;
          }

          /** \brief Computes and set the origin of the new cube (relative to the world), centered around a the target point.
            * \param[in] target_point the target point around which the new cube will be centered.
            * \param[out] shiftX shift on X axis (in indices).
            * \param[out] shiftY shift on Y axis (in indices).
            * \param[out] shiftZ shift on Z axis (in indices).
            */ 
          void computeAndSetNewCubeMetricOrigin (const pcl::PointXYZ &target_point, int &shiftX, int &shiftY, int &shiftZ);
          
          /** \brief Initializes memory pointers of the  cyclical buffer (start, end, current origin)
            * \param[in] tsdf_volume pointer to the TSDF volume managed by this cyclical buffer
            */ 
          void initBuffer (TsdfVolume::Ptr tsdf_volume)
          {
            PtrStep<short2> localVolume = tsdf_volume->data();
            
            buffer_.tsdf_memory_start = &(localVolume.ptr (0)[0]);
            buffer_.tsdf_memory_end = &(localVolume.ptr (buffer_.voxels_size.y * (buffer_.voxels_size.z - 1) + (buffer_.voxels_size.y - 1) )[buffer_.voxels_size.x - 1]);
            buffer_.tsdf_rolling_buff_origin = buffer_.tsdf_memory_start;
          }
          
          /** \brief Reset buffer structure
            * \param[in] tsdf_volume pointer to the TSDF volume managed by this cyclical buffer
            */ 
          void resetBuffer (TsdfVolume::Ptr tsdf_volume)
          {
            buffer_.origin_GRID.x = 0; buffer_.origin_GRID.y = 0; buffer_.origin_GRID.z = 0;
            buffer_.origin_GRID_global.x = 0.f; buffer_.origin_GRID_global.y = 0.f; buffer_.origin_GRID_global.z = 0.f;
            buffer_.origin_metric.x = 0.f; buffer_.origin_metric.y = 0.f; buffer_.origin_metric.z = 0.f;
            initBuffer (tsdf_volume);
          }
          
          /** \brief Return a pointer to the world model
            */ 
          pcl::kinfuLS::WorldModel<pcl::PointXYZI>*
          getWorldModel ()
          {
            return (&world_model_);
          }
                
          
        private:

          /** \brief buffer used to extract XYZ values from GPU */
          DeviceArray<PointXYZ> cloud_buffer_device_xyz_;
          
          /** \brief buffer used to extract Intensity values from GPU */
          DeviceArray<float> cloud_buffer_device_intensities_;

          /** \brief distance threshold (cube's center to target point) to trigger shift */
          double distance_threshold_;
          
          /** \brief world model object that maintains the known world */
          pcl::kinfuLS::WorldModel<pcl::PointXYZI> world_model_;

          /** \brief structure that contains all TSDF buffer's addresses */
          tsdf_buffer buffer_;
          
          /** \brief updates cyclical buffer origins given offsets on X, Y and Z
            * \param[in] tsdf_volume pointer to the TSDF volume managed by this cyclical buffer
            * \param[in] offset_x offset in indices on axis X
            * \param[in] offset_y offset in indices on axis Y
            * \param[in] offset_z offset in indices on axis Z
            */ 
          void shiftOrigin (TsdfVolume::Ptr tsdf_volume, const int offset_x, const int offset_y, const int offset_z)
          {
            // shift rolling origin (making sure they keep in [0 - NbVoxels[ )
            buffer_.origin_GRID.x += offset_x;
            if(buffer_.origin_GRID.x >= buffer_.voxels_size.x)
              buffer_.origin_GRID.x -= buffer_.voxels_size.x;
            else if(buffer_.origin_GRID.x < 0)
              buffer_.origin_GRID.x += buffer_.voxels_size.x;
              
            buffer_.origin_GRID.y += offset_y;
            if(buffer_.origin_GRID.y >= buffer_.voxels_size.y)
              buffer_.origin_GRID.y -= buffer_.voxels_size.y;
            else if(buffer_.origin_GRID.y < 0)
              buffer_.origin_GRID.y += buffer_.voxels_size.y;  
            
            buffer_.origin_GRID.z += offset_z;
            if(buffer_.origin_GRID.z >= buffer_.voxels_size.z)
              buffer_.origin_GRID.z -= buffer_.voxels_size.z;
            else if(buffer_.origin_GRID.z < 0)
              buffer_.origin_GRID.z += buffer_.voxels_size.z; 
          
            // update memory pointers
            PtrStep<short2> localVolume = tsdf_volume->data();
            buffer_.tsdf_memory_start = &(localVolume.ptr (0)[0]);
            buffer_.tsdf_memory_end = &(localVolume.ptr (buffer_.voxels_size.y * (buffer_.voxels_size.z - 1) + (buffer_.voxels_size.y - 1) )[buffer_.voxels_size.x - 1]);
            buffer_.tsdf_rolling_buff_origin = &(localVolume.ptr (buffer_.voxels_size.y * (buffer_.origin_GRID.z) + (buffer_.origin_GRID.y) )[buffer_.origin_GRID.x]);
            
            // update global origin
            buffer_.origin_GRID_global.x += offset_x;
            buffer_.origin_GRID_global.y += offset_y;
            buffer_.origin_GRID_global.z += offset_z;
          }
      
      };
    }
  }
}
