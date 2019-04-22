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

#include <cuda_runtime.h>
//#include <boost/graph/buffer_concepts.hpp>
    
namespace pcl
{
  namespace gpu
  {
    namespace kinfuLS
    {
      /** \brief Structure to handle buffer addresses */
          struct tsdf_buffer
          {
            /** \brief */
            /** \brief Address of the first element of the TSDF volume in memory*/  
            short2* tsdf_memory_start;
            /** \brief Address of the last element of the TSDF volume in memory*/          
            short2* tsdf_memory_end;
            /** \brief Memory address of the origin of the rolling buffer. MUST BE UPDATED AFTER EACH SHIFT.*/
            short2* tsdf_rolling_buff_origin;   
            /** \brief Internal cube origin for rollign buffer.*/
            int3 origin_GRID; 
            /** \brief Cube origin in world coordinates.*/
            float3 origin_GRID_global;
            /** \brief Current metric origin of the cube, in world coordinates.*/ 
            float3 origin_metric;
            /** \brief Size of the volume, in meters.*/
            float3 volume_size; //3.0
            /** \brief Number of voxels in the volume, per axis*/
            int3 voxels_size; //512

            /** \brief Default constructor*/ 
            tsdf_buffer () 
            {
              tsdf_memory_start = nullptr;  tsdf_memory_end = nullptr; tsdf_rolling_buff_origin = nullptr; 
              origin_GRID.x = 0; origin_GRID.y = 0; origin_GRID.z = 0;
              origin_GRID_global.x = 0.f; origin_GRID_global.y = 0.f; origin_GRID_global.z = 0.f;
              origin_metric.x = 0.f; origin_metric.y = 0.f; origin_metric.z = 0.f;
              volume_size.x = 3.f; volume_size.y = 3.f; volume_size.z = 3.f;
              voxels_size.x = 512; voxels_size.y = 512; voxels_size.z = 512;
            }          

          };
    }
  }
}
