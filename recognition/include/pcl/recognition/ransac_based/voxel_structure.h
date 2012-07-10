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

#include <pcl/common/eigen.h>

namespace pcl
{
  namespace recognition
  {
    /** \brief This class is a box in R3 built of voxels ordered in a regular rectangular grid. Each voxel is of type T. */
    template<class T>
    class VoxelStructure
    {
    public:
      VoxelStructure (): voxels_(NULL){}
      virtual ~VoxelStructure (){ this->clear();}

      /** \brief Call this method before using an instance of this class. Parameter meaning is obvious. */
      void
      build (const double bounds[6], int num_of_voxels[3]);

      /** \brief Release the memory allocated for the voxels. */
      void
      clear (){ if ( voxels_ ){ delete[] voxels_; voxels_ = NULL;}}

      /** \brief Returns a pointer to the voxel which contains p or NULL if p is not inside the structure. */
      inline T*
      getVoxel (const double p[3]);

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
      void
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
      const int*
      getNumberOfVoxelsXYZ() const
      {
        return (num_of_voxels_);
      }

      /** \brief Computes the center of the voxel with given integer coordinates.
       *
       * \param[in]  id3 the integer coordinates along the x, y and z axis.
       * \param[out] center */
      void
      computeVoxelCenter (const int id3[3], double center[3]) const
      {
        center[0] = min_center_[0] + id3[0]*spacing_[0];
        center[1] = min_center_[1] + id3[1]*spacing_[1];
        center[2] = min_center_[2] + id3[2]*spacing_[2];
      }

      /** \brief Returns the total number of voxels. */
      int
      getNumberOfVoxels() const
      {
        return (total_num_of_voxels_);
      }

      /** \brief Returns the bounds of the voxel structure, which is pointer to the internal array of 6 doubles: (min_x, max_x, min_y, max_y, min_z, max_z). */
      const int*
      getBounds() const
      {
        return (bounds_);
      }

      /** \brief Returns the voxel spacing in x, y and z direction. */
      const double*
      getVoxelSpacing() const
      {
        return (spacing_);
      }

    protected:
      T *voxels_;
      int num_of_voxels_[3], num_of_voxels_xy_plane_, total_num_of_voxels_;
      double bounds_[6];
      double spacing_[3]; // spacing betwen the voxel in x, y and z direction = voxel size in x, y and z direction
      double min_center_[3]; // the center of the voxel with integer coordinates (0, 0, 0)
    };

// === inline methods ======================================================================================================================

    template<class T> inline T*
    VoxelStructure<T>::getVoxel(const double p[3])
    {
      if ( p[0] < bounds_[0] || p[0] >= bounds_[1] || p[1] < bounds_[2] || p[1] >= bounds_[3] || p[2] < bounds_[4] || p[2] >= bounds_[5] )
        return NULL;

      int x = static_cast<int>((p[0] - bounds_[0])/spacing_[0] + 0.5);
      int y = static_cast<int>((p[1] - bounds_[2])/spacing_[1] + 0.5);
      int z = static_cast<int>((p[2] - bounds_[4])/spacing_[2] + 0.5);

      return &voxels_[z*num_of_voxels_xy_plane_ + y*num_of_voxels_[0] + x];
    }

  } // namespace recognition
} // namespace pcl

#include <pcl/recognition/impl/ransac_based/voxel_structure.hpp>

#endif // PCL_RECOGNITION_VOXEL_STRUCTURE_H_
