/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009-2011, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
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
 *   * Neither the name of the copyright holder(s) nor the names of its
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
 * aabb_tree.h
 * Created on: Jun 02, 2022
 * Author: Ramzi Sabra
 */

#pragma once

#include <pcl/PolygonMesh.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace pcl {
/** \brief AABBTree represents the base ray intersection testing class for AABB tree
 * implementations. \author Ramzi Sabra \ingroup aabb_tree
 */
template <typename PointT>
class AABBTree {
public:
  using PointCloud = pcl::PointCloud<PointT>;
  using PointCloudPtr = typename PointCloud::Ptr;
  using PointCloudConstPtr = std::shared_ptr<const PointCloud>;

  virtual ~AABBTree(){};

  /** \brief Provide an input mesh to construct the tree.
   * \param[in] mesh the polygon mesh
   */
  virtual void
  setInputMesh(const pcl::PolygonMesh& mesh) = 0;

  /** \brief Provide an input mesh to construct the tree.
   * \param[in] mesh_cloud the mesh cloud
   * \param[in] mesh_polygons the mesh polygons
   */
  virtual void
  setInputMesh(PointCloudConstPtr mesh_cloud,
               const std::vector<Vertices>& mesh_polygons) = 0;

  /** \brief Check for the presence of intersection(s) for the given ray
   * \param[in] source the ray source point
   * \param[in] direction the ray direction vector
   */
  virtual bool
  checkForIntersection(const PointT& source,
                       const Eigen::Vector3f& direction) const = 0;

  /** \brief Check for the number of intersections for the given ray
   * \param[in] source the ray source point
   * \param[in] direction the ray direction vector
   */
  virtual size_t
  numberOfIntersections(const PointT& source,
                        const Eigen::Vector3f& direction) const = 0;

  /** \brief Get all intersections for the given ray
   * \param[in] source the ray source point
   * \param[in] direction the ray direction vector
   */
  virtual std::vector<index_t>
  getAllIntersections(const PointT& source, const Eigen::Vector3f& direction) const = 0;

  /** \brief Get any intersection for the given ray
   * \param[in] source the ray source point
   * \param[in] direction the ray direction vector
   */
  virtual index_t
  getAnyIntersection(const PointT& source, const Eigen::Vector3f& direction) const = 0;

  /** \brief Get the nearest intersection for the given ray
   * \param[in] source the ray source point
   * \param[in] direction the ray direction vector
   */
  virtual index_t
  getNearestIntersection(const PointT& source,
                         const Eigen::Vector3f& direction) const = 0;

protected:
  /** \brief Class getName method. */
  virtual std::string
  getName() const = 0;
};
} // namespace pcl
