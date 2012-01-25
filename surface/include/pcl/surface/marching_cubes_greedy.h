/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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

#ifndef PCL_SURFACE_MARCHING_CUBES_GREEDY_H_
#define PCL_SURFACE_MARCHING_CUBES_GREEDY_H_

#include <pcl/surface/marching_cubes.h>

namespace pcl
{

  /** \brief The marching cubes surface reconstruction algorithm, using a "greedy" voxelization algorithm
    * \author Gregory Long
    * \ingroup surface
    */
  template <typename PointNT>
  class MarchingCubesGreedy : public MarchingCubes<PointNT>
  {
    public:
      using SurfaceReconstruction<PointNT>::input_;
      using MarchingCubes<PointNT>::cell_hash_map_;

      typedef typename MarchingCubes<PointNT>::Leaf Leaf;

      typedef typename pcl::PointCloud<PointNT>::Ptr PointCloudPtr;

      typedef typename pcl::KdTree<PointNT> KdTree;
      typedef typename pcl::KdTree<PointNT>::Ptr KdTreePtr;
      typedef boost::unordered_map<uint64_t, Leaf, boost::hash<uint64_t>, std::equal_to<uint64_t>, Eigen::aligned_allocator<uint64_t> > HashMap;

      /** \brief Constructor. */ 
      MarchingCubesGreedy ();

      /** \brief Destructor. */
      ~MarchingCubesGreedy ();

    private:
      /** \brief Convert the point cloud into voxel data. */
      void
      voxelizeData();

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}

#endif  // PCL_SURFACE_MARCHING_CUBES_H_
 
