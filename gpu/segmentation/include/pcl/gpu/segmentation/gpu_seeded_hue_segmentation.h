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
 * $Id:$
 *
 */

#pragma once

#include <pcl/PointIndices.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace pcl {
namespace gpu {
void
seededHueSegmentation(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& host_cloud_,
                      const pcl::gpu::Octree::Ptr& tree,
                      float tolerance,
                      PointIndices& clusters_in,
                      PointIndices& clusters_out,
                      float delta_hue = 0.0);

class SeededHueSegmentation {
public:
  using PointType = pcl::PointXYZ;
  using PointCloudHost = pcl::PointCloud<pcl::PointXYZ>;
  using PointCloudHostPtr = PointCloudHost::Ptr;
  using PointCloudHostConstPtr = PointCloudHost::ConstPtr;

  using PointIndicesPtr = PointIndices::Ptr;
  using PointIndicesConstPtr = PointIndices::ConstPtr;

  using GPUTree = pcl::gpu::Octree;
  using GPUTreePtr = pcl::gpu::Octree::Ptr;

  using CloudDevice = pcl::gpu::Octree::PointCloud;

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Empty constructor. */
  SeededHueSegmentation() = default;

  /** \brief Provide a pointer to the search object.
   * \param tree a pointer to the spatial search object.
   */
  inline void
  setSearchMethod(const GPUTreePtr& tree)
  {
    tree_ = tree;
  }

  /** \brief Get a pointer to the search method used.
   *  @todo fix this for a generic search tree
   */
  inline GPUTreePtr
  getSearchMethod()
  {
    return (tree_);
  }

  /** \brief Set the spatial cluster tolerance as a measure in the L2 Euclidean space
   * \param tolerance the spatial cluster tolerance as a measure in the L2 Euclidean
   * space
   */
  inline void
  setClusterTolerance(double tolerance)
  {
    cluster_tolerance_ = tolerance;
  }

  /** \brief Get the spatial cluster tolerance as a measure in the L2 Euclidean space.
   */
  inline double
  getClusterTolerance()
  {
    return (cluster_tolerance_);
  }

  inline void
  setInput(CloudDevice input)
  {
    input_ = input;
  }

  inline void
  setHostCloud(PointCloudHostPtr host_cloud)
  {
    host_cloud_ = host_cloud;
  }

  /** \brief Set the tollerance on the hue
   * \param[in] delta_hue the new delta hue
   */
  inline void
  setDeltaHue(float delta_hue)
  {
    delta_hue_ = delta_hue;
  }

  /** \brief Get the tolerance on the hue */
  inline float
  getDeltaHue()
  {
    return (delta_hue_);
  }

  /** \brief extract clusters of a PointCloud given by <setInputCloud(), setIndices()>
   * \param indices_in
   * \param indices_out
   */
  void
  segment(PointIndices& indices_in, PointIndices& indices_out);

protected:
  /** \brief the input cloud on the GPU */
  CloudDevice input_;

  /** \brief the original cloud the Host */
  PointCloudHostPtr host_cloud_;

  /** \brief A pointer to the spatial search object. */
  GPUTreePtr tree_;

  /** \brief The spatial cluster tolerance as a measure in the L2 Euclidean space. */
  double cluster_tolerance_{0};

  /** \brief The allowed difference on the hue*/
  float delta_hue_{0.0};

  /** \brief Class getName method. */
  virtual std::string
  getClassName() const
  {
    return ("gpu::SeededHueSegmentation");
  }
};
/** \brief Sort clusters method (for std::sort).
 * \ingroup segmentation
 */
inline bool
comparePointClusters(const pcl::PointIndices& a, const pcl::PointIndices& b)
{
  return (a.indices.size() < b.indices.size());
}
} // namespace gpu
} // namespace pcl
