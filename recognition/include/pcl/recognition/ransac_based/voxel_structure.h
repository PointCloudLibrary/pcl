/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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
 *
 */

#ifndef PCL_RECOGNITION_VOXEL_STRUCTURE_H_
#define PCL_RECOGNITION_VOXEL_STRUCTURE_H_

#include <cstdlib>

namespace pcl
{
  namespace recognition
  {
    /** \brief This class is a box in R3 built of voxels ordered in a regular rectangular grid. Each voxel is of type T. */
    template<class T, typename REAL = float>
    class VoxelStructure
    {
    public:
      inline VoxelStructure (): voxels_(NULL){}
      inline virtual ~VoxelStructure (){ this->clear();}

      /** \brief Call this method before using an instance of this class. Parameter meaning is obvious. */
      inline void
      build (const REAL bounds[6], int num_of_voxels[3]);

      /** \brief Release the memory allocated for the voxels. */
      inline void
      clear (){ if ( voxels_ ){ delete[] voxels_; voxels_ = NULL;}}

      /** \brief Returns a pointer to the voxel which contains p or NULL if p is not inside the structure. */
      inline T*
      getVoxel (const REAL p[3]);

      /** \brief Returns a pointer to the voxel with integer id (x,y,z) or NULL if (x,y,z) is out of bounds. */
      inline T*
      getVoxel (int x, int y, int z) const;

      /** \brief Returns the linear voxel array. */
      const inline T*
      getVoxels () const
      {
        return voxels_;
      }

      /** \brief Returns the linear voxel array. */
      inline T*
      getVoxels ()
      {
        return voxels_;
      }

      /** \brief Converts a linear id to a 3D id, i.e., computes the integer 3D coordinates of a voxel from its position in the voxel array.
        *
        * \param[in]  linear_id the position of the voxel in the internal voxel array.
        * \param[out] id3d an array of 3 integers for the integer coordinates of the voxel. */
      inline void
      compute3dId (int linear_id, int id3d[3]) const
      {
        // Compute the z axis id
        id3d[2] = linear_id / num_of_voxels_xy_plane_;
        int proj_xy = linear_id % num_of_voxels_xy_plane_;
        // Compute the y axis id
        id3d[1] = proj_xy / num_of_voxels_[0];
        // Compute the x axis id
        id3d[0] = proj_xy % num_of_voxels_[0];
      }

      /** \brief Returns the number of voxels in x, y and z direction. */
      inline const int*
      getNumberOfVoxelsXYZ() const
      {
        return (num_of_voxels_);
      }

      /** \brief Computes the center of the voxel with given integer coordinates.
       *
       * \param[in]  id3 the integer coordinates along the x, y and z axis.
       * \param[out] center */
      inline void
      computeVoxelCenter (const int id3[3], REAL center[3]) const
      {
        center[0] = min_center_[0] + static_cast<float> (id3[0])*spacing_[0];
        center[1] = min_center_[1] + static_cast<float> (id3[1])*spacing_[1];
        center[2] = min_center_[2] + static_cast<float> (id3[2])*spacing_[2];
      }

      /** \brief Returns the total number of voxels. */
      inline int
      getNumberOfVoxels() const
      {
        return (total_num_of_voxels_);
      }

      /** \brief Returns the bounds of the voxel structure, which is pointer to the internal array of 6 doubles: (min_x, max_x, min_y, max_y, min_z, max_z). */
      inline const float*
      getBounds() const
      {
        return (bounds_);
      }

      /** \brief Copies the bounds of the voxel structure to 'b'. */
      inline void
      getBounds(REAL b[6]) const
      {
        b[0] = bounds_[0];
        b[1] = bounds_[1];
        b[2] = bounds_[2];
        b[3] = bounds_[3];
        b[4] = bounds_[4];
        b[5] = bounds_[5];
      }

      /** \brief Returns the voxel spacing in x, y and z direction. That's the same as the voxel size along each axis. */
      const REAL*
      getVoxelSpacing() const
      {
        return (spacing_);
      }

      /** \brief Saves pointers to the voxels which are neighbors of the voxels which contains 'p'. The containing voxel is returned too.
        * 'neighs' has to be an array of pointers with space for at least 27 pointers (27 = 3^3 which is the max number of neighbors). The */
      inline int
      getNeighbors (const REAL* p, T **neighs) const;

    protected:
      T *voxels_;
      int num_of_voxels_[3], num_of_voxels_xy_plane_, total_num_of_voxels_;
      REAL bounds_[6];
      REAL spacing_[3]; // spacing betwen the voxel in x, y and z direction = voxel size in x, y and z direction
      REAL min_center_[3]; // the center of the voxel with integer coordinates (0, 0, 0)
    };
  } // namespace recognition
} // namespace pcl

#include <pcl/recognition/impl/ransac_based/voxel_structure.hpp>

#endif // PCL_RECOGNITION_VOXEL_STRUCTURE_H_
