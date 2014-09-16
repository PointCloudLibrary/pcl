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

#ifndef PCL_KINFU_TSDF_MARCHING_CUBES_H_
#define PCL_KINFU_TSDF_MARCHING_CUBES_H_

#include <pcl/pcl_macros.h>
#include <pcl/gpu/containers/device_array.h>
#include <Eigen/Core>
//#include <boost/graph/buffer_concepts.hpp>


namespace pcl
{
  namespace gpu
  {
    namespace kinfuLS
    {
      class TsdfVolume;
        
      /** \brief MarchingCubes implements MarchingCubes functionality for TSDF volume on GPU
        * \author Anatoly Baskeheev, Itseez Ltd, (myname.mysurname@mycompany.com)
        */
      class PCL_EXPORTS MarchingCubes
      {
      public:

        /** \brief Default size for triangles buffer */
        enum
        { 
          POINTS_PER_TRIANGLE = 3,
          DEFAULT_TRIANGLES_BUFFER_SIZE = 2 * 1000 * 1000 * POINTS_PER_TRIANGLE * 2     
        };
      
        /** \brief Point type. */
        typedef pcl::PointXYZ PointType;
        
        /** \brief Smart pointer. */
        typedef boost::shared_ptr<MarchingCubes> Ptr;
        
        /** \brief Default constructor */
        MarchingCubes();
        
        /** \brief Destructor */
        ~MarchingCubes();
        
        /** \brief Runs marching cubes triangulation.
            * \param[in] tsdf
            * \param[in] triangles_buffer Buffer for triangles. Its size determines max extracted triangles. If empty, it will be allocated with default size will be used.          
            * \return Array with triangles. Each 3 consequent poits belond to a single triangle. The returned array points to 'triangles_buffer' data.
            */
        DeviceArray<PointType> 
        run(const TsdfVolume& tsdf, DeviceArray<PointType>& triangles_buffer);

      private:             
        /** \brief Edge table for marching cubes  */
        DeviceArray<int> edgeTable_;
        
        /** \brief Number of vertextes table for marching cubes  */
        DeviceArray<int> numVertsTable_;
        
        /** \brief Triangles table for marching cubes  */
        DeviceArray<int> triTable_;     
        
        /** \brief Temporary buffer used by marching cubes (first row stores occuped voxes id, second number of vetexes, third poits offsets */
        DeviceArray2D<int> occupied_voxels_buffer_;
      };
    }
  }
}

#endif /* PCL_KINFU_MARCHING_CUBES_H_ */
