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


#include <pcl/gpu/kinfu_large_scale/cyclical_buffer.h>
#include <pcl/common/distances.h>
#include <pcl/common/transforms.h> // for transformPoint, transformPointCloud
#include "internal.h"



bool
pcl::gpu::kinfuLS::CyclicalBuffer::checkForShift (const TsdfVolume::Ptr volume, const Eigen::Affine3f &cam_pose, const double distance_camera_target, const bool perform_shift, const bool last_shift, const bool force_shift)
{
  bool result = false;

  // project the target point in the cube
  pcl::PointXYZ targetPoint;
  targetPoint.x = 0.0f;
  targetPoint.y = 0.0f;
  targetPoint.z = distance_camera_target; // place the point at camera position + distance_camera_target on Z
  targetPoint = pcl::transformPoint (targetPoint, cam_pose);

  // check distance from the cube's center
  pcl::PointXYZ center_cube;
  center_cube.x = buffer_.origin_metric.x + buffer_.volume_size.x/2.0f;
  center_cube.y = buffer_.origin_metric.y + buffer_.volume_size.y/2.0f;
  center_cube.z = buffer_.origin_metric.z + buffer_.volume_size.z/2.0f;

  if (pcl::euclideanDistance (targetPoint, center_cube) > distance_threshold_)
    result = true;

  if (!perform_shift && !force_shift)
    return (result);

  // perform shifting operations
  if (result || force_shift)
    performShift (volume, targetPoint, last_shift);

  return (result);
}


void
pcl::gpu::kinfuLS::CyclicalBuffer::performShift (const TsdfVolume::Ptr volume, const pcl::PointXYZ &target_point, const bool last_shift)
{
  // compute new origin and offsets
  int offset_x, offset_y, offset_z;
  computeAndSetNewCubeMetricOrigin (target_point, offset_x, offset_y, offset_z);

  // extract current slice from the TSDF volume (coordinates are in indices! (see fetchSliceAsCloud() )
  DeviceArray<PointXYZ> points;
  DeviceArray<float> intensities;
  int size;
  if(!last_shift)
  {
    size = volume->fetchSliceAsCloud (cloud_buffer_device_xyz_, cloud_buffer_device_intensities_, &buffer_, offset_x, offset_y, offset_z);
  }
  else
  {
    size = volume->fetchSliceAsCloud (cloud_buffer_device_xyz_, cloud_buffer_device_intensities_, &buffer_, buffer_.voxels_size.x - 1, buffer_.voxels_size.y - 1, buffer_.voxels_size.z - 1);
  }
  points = DeviceArray<PointXYZ> (cloud_buffer_device_xyz_.ptr (), size);
  intensities = DeviceArray<float> (cloud_buffer_device_intensities_.ptr(), size);

  PointCloud<PointXYZI>::Ptr current_slice (new PointCloud<PointXYZI>);
  PointCloud<PointXYZ>::Ptr current_slice_xyz (new PointCloud<PointXYZ>);
  PointCloud<PointIntensity>::Ptr current_slice_intensities (new PointCloud<PointIntensity>);

  // Retrieving XYZ
  points.download (current_slice_xyz->points);
  current_slice_xyz->width = current_slice_xyz->size ();
  current_slice_xyz->height = 1;

  // Retrieving intensities
  // TODO change this mechanism by using PointIntensity directly (in spite of float)
  // when tried, this lead to wrong intenisty values being extracted by fetchSliceAsCloud () (padding pbls?)
  std::vector<float , Eigen::aligned_allocator<float> > intensities_vector;
  intensities.download (intensities_vector);
  current_slice_intensities->resize (current_slice_xyz->size ());
  for(std::size_t i = 0 ; i < current_slice_intensities->size () ; ++i)
    (*current_slice_intensities)[i].intensity = intensities_vector[i];

  current_slice_intensities->width = current_slice_intensities->size ();
  current_slice_intensities->height = 1;

  // Concatenating XYZ and Intensities
  pcl::concatenateFields (*current_slice_xyz, *current_slice_intensities, *current_slice);
  current_slice->width = current_slice->size ();
  current_slice->height = 1;

  // transform the slice from local to global coordinates
  Eigen::Affine3f global_cloud_transformation;
  global_cloud_transformation.translation ()[0] = buffer_.origin_GRID_global.x;
  global_cloud_transformation.translation ()[1] = buffer_.origin_GRID_global.y;
  global_cloud_transformation.translation ()[2] = buffer_.origin_GRID_global.z;
  global_cloud_transformation.linear () = Eigen::Matrix3f::Identity ();
  transformPointCloud (*current_slice, *current_slice, global_cloud_transformation);

  // retrieve existing data from the world model
  PointCloud<PointXYZI>::Ptr previously_existing_slice (new  PointCloud<PointXYZI>);
  world_model_.getExistingData (buffer_.origin_GRID_global.x, buffer_.origin_GRID_global.y, buffer_.origin_GRID_global.z,
                                offset_x, offset_y, offset_z,
                                buffer_.voxels_size.x - 1, buffer_.voxels_size.y - 1, buffer_.voxels_size.z - 1,
                                *previously_existing_slice);

  //replace world model data with values extracted from the TSDF buffer slice
  world_model_.setSliceAsNans (buffer_.origin_GRID_global.x, buffer_.origin_GRID_global.y, buffer_.origin_GRID_global.z,
                               offset_x, offset_y, offset_z,
                               buffer_.voxels_size.x, buffer_.voxels_size.y, buffer_.voxels_size.z);


  PCL_INFO ("world contains %d points after update\n", world_model_.getWorldSize ());
  world_model_.cleanWorldFromNans ();
  PCL_INFO ("world contains %d points after cleaning\n", world_model_.getWorldSize ());

  // clear buffer slice and update the world model
  pcl::device::kinfuLS::clearTSDFSlice (volume->data (), &buffer_, offset_x, offset_y, offset_z);

  // insert current slice in the world if it contains any points
  if (!current_slice->points.empty ()) {
    world_model_.addSlice(current_slice);
  }

  // shift buffer addresses
  shiftOrigin (volume, offset_x, offset_y, offset_z);

  // push existing data in the TSDF buffer
  if (!previously_existing_slice->points.empty () ) {
    volume->pushSlice(previously_existing_slice, getBuffer () );
  }
}

void
pcl::gpu::kinfuLS::CyclicalBuffer::computeAndSetNewCubeMetricOrigin (const pcl::PointXYZ &target_point, int &shiftX, int &shiftY, int &shiftZ)
{
  // compute new origin for the cube, based on the target point
  float3 new_cube_origin_meters;
  new_cube_origin_meters.x = target_point.x - buffer_.volume_size.x/2.0f;
  new_cube_origin_meters.y = target_point.y - buffer_.volume_size.y/2.0f;
  new_cube_origin_meters.z = target_point.z - buffer_.volume_size.z/2.0f;
  PCL_INFO ("The old cube's metric origin was    (%f, %f, %f).\n", buffer_.origin_metric.x, buffer_.origin_metric.y, buffer_.origin_metric.z);
  PCL_INFO ("The new cube's metric origin is now (%f, %f, %f).\n", new_cube_origin_meters.x, new_cube_origin_meters.y, new_cube_origin_meters.z);

  // deduce each shift in indices
  shiftX = (int)( (new_cube_origin_meters.x - buffer_.origin_metric.x) * ( buffer_.voxels_size.x / (float) (buffer_.volume_size.x) ) );
  shiftY = (int)( (new_cube_origin_meters.y - buffer_.origin_metric.y) * ( buffer_.voxels_size.y / (float) (buffer_.volume_size.y) ) );
  shiftZ = (int)( (new_cube_origin_meters.z - buffer_.origin_metric.z) * ( buffer_.voxels_size.z / (float) (buffer_.volume_size.z) ) );

  // update the cube's metric origin
  buffer_.origin_metric = new_cube_origin_meters;
}
