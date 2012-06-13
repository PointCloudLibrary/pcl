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

#include <Eigen/Core>

namespace pcl
{
  namespace recognition
  {
    /** \brief This class is a box in R3 built of voxels ordered in a regular rectangular grid. Each voxel is of type T. */
    template<class T>
    class VoxelStructure
    {
    public:
      VoxelStructure(): voxels_(NULL){}
      virtual ~VoxelStructure(){ this->clear();}

      void build(const double bounds[6], int num_of_voxels[3]);
      void clear(){ if ( voxels_ ){ delete[] voxels_; voxels_ = NULL;}}

      /** \brief Returns a pointer to the voxel which contains p or NULL if p is not inside the structure. */
      T* getVoxel(Eigen::Vector3d& p){ return NULL;}

    protected:
      T *voxels_;
      double bounds_[6];
    };
  } // namespace recognition
} // namespace pcl

#endif // PCL_RECOGNITION_VOXEL_STRUCTURE_H_
