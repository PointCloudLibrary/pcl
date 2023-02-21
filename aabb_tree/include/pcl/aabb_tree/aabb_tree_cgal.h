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
 * aabb_tree_cgal.h
 * Created on: Jun 02, 2022
 * Author: Ramzi Sabra
 */

#pragma once

#include <pcl/aabb_tree/aabb_tree.h>

#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_triangle_primitive.h>
#include <CGAL/Simple_cartesian.h>

namespace pcl {
/** \brief AABBTreeCGAL is a ray intersection testing class using AABB tree
 * structures. The class is making use of the CGAL (The Computational Geometry Algorithms
 * Library) project. \author Ramzi Sabra \ingroup aabb_tree
 */
template <typename PointT>
class AABBTreeCGAL : public AABBTree<PointT> {
protected:
  using K = CGAL::Simple_cartesian<float>;
  using CGALPoint = K::Point_3;
  using CGALVector = K::Vector_3;
  using Ray = K::Ray_3;
  using Triangle = K::Triangle_3;

  using Iterator = std::vector<Triangle>::const_iterator;
  using Primitive = CGAL::AABB_triangle_primitive<K, Iterator>;
  using AABB_triangle_traits = CGAL::AABB_traits<K, Primitive>;

public:
  using Tree = CGAL::AABB_tree<AABB_triangle_traits>;
  using TreePtr = std::shared_ptr<Tree>;
  using TreeConstPtr = std::shared_ptr<const Tree>;

  using PointCloud = typename AABBTree<PointT>::PointCloud;
  using PointCloudPtr = typename AABBTree<PointT>::PointCloudPtr;
  using PointCloudConstPtr = typename AABBTree<PointT>::PointCloudConstPtr;

  /** \brief Default constructor for AABBTreeCGAL.
   * \param[in] tree already-built CGAL AABB tree. Set to empty by default.
   */
  AABBTreeCGAL(TreeConstPtr tree = TreeConstPtr());

  /** \brief Set an already-build CGAL AABB tree as the tree.
   * \param[in] tree already-built CGAL AABB tree
   */
  void
  setTree(TreeConstPtr tree);

  /** \brief Provide an input mesh to construct the tree.
   * \param[in] mesh the polygon mesh
   */
  void
  setInputMesh(const pcl::PolygonMesh& mesh) override;

  /** \brief Provide an input mesh to construct the tree.
   * \param[in] mesh_cloud the mesh cloud
   * \param[in] mesh_polygons the mesh polygons
   */
  void
  setInputMesh(PointCloudConstPtr mesh_cloud,
               const std::vector<Vertices>& mesh_polygons) override;

  /** \brief Check for the presence of intersection(s) for the given ray
   * \param[in] source the ray source point
   * \param[in] direction the ray direction vector
   */
  bool
  checkForIntersection(const PointT& source,
                       const Eigen::Vector3f& direction) const override;

  /** \brief Check for the number of intersections for the given ray
   * \param[in] source the ray source point
   * \param[in] direction the ray direction vector
   */
  size_t
  numberOfIntersections(const PointT& source,
                        const Eigen::Vector3f& direction) const override;

  /** \brief Get all intersections for the given ray
   * \param[in] source the ray source point
   * \param[in] direction the ray direction vector
   */
  std::vector<index_t>
  getAllIntersections(const PointT& source,
                      const Eigen::Vector3f& direction) const override;

  /** \brief Get any intersection for the given ray
   * \param[in] source the ray source point
   * \param[in] direction the ray direction vector
   */
  index_t
  getAnyIntersection(const PointT& source,
                     const Eigen::Vector3f& direction) const override;

  /** \brief Get the nearest intersection for the given ray
   * \param[in] source the ray source point
   * \param[in] direction the ray direction vector
   */
  index_t
  getNearestIntersection(const PointT& source,
                         const Eigen::Vector3f& direction) const override;

protected:
  /** \brief Class getName method. */
  std::string
  getName() const override
  {
    return ("AABBTreeCGAL");
  }

  /** \brief A CGAL AABB tree object. */
  TreeConstPtr tree_;

  /** \brief CGAL triangles used for building the tree. */
  std::vector<Triangle> triangles_;

private:
  /** \brief Generate a CGAL ray from a source point and a direction vector
   * \param[in] source the ray source point
   * \param[in] direction the ray direction vector
   */
  static Ray
  generateRay(const PointT& source, const Eigen::Vector3f& direction);
};
} // namespace pcl

#ifdef PCL_NO_PRECOMPILE
#include <pcl/aabb_tree/impl/aabb_tree_cgal.hpp>
#endif
