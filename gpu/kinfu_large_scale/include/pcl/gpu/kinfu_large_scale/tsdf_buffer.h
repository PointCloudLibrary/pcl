#ifndef PCL_TSDF_BUFFER_STRUCT_H_
#define PCL_TSDF_BUFFER_STRUCT_H_

#include <cuda_runtime.h>


    
namespace pcl
{    
  namespace gpu
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
            tsdf_memory_start = 0;  tsdf_memory_end = 0; tsdf_rolling_buff_origin = 0; 
            origin_GRID.x = 0; origin_GRID.y = 0; origin_GRID.z = 0;
            origin_GRID_global.x = 0.f; origin_GRID_global.y = 0.f; origin_GRID_global.z = 0.f;
            origin_metric.x = 0.f; origin_metric.y = 0.f; origin_metric.z = 0.f;
            volume_size.x = 3.f; volume_size.y = 3.f; volume_size.z = 3.f;
            voxels_size.x = 512; voxels_size.y = 512; voxels_size.z = 512;
          }          

        };
  }
}

#endif /*PCL_TSDF_BUFFER_STRUCT_H_*/
