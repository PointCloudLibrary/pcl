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
 * aabb_tree_cgal.cpp
 * Created on: Jun 02, 2022
 * Author: Ramzi Sabra
 */

#ifndef PCL_AABB_TREE_AABB_TREE_IMPL_CGAL_H_
#define PCL_AABB_TREE_AABB_TREE_IMPL_CGAL_H_

#include <pcl/aabb_tree/aabb_tree_cgal.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
pcl::AABBTreeCGAL<PointT>::AABBTreeCGAL(TreeConstPtr tree) : tree_(tree)
{}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void
pcl::AABBTreeCGAL<PointT>::setTree(TreeConstPtr tree)
{
  tree_ = tree;
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void
pcl::AABBTreeCGAL<PointT>::setInputMesh(const pcl::PolygonMesh& mesh)
{
  PointCloudPtr mesh_cloud = std::make_shared<PointCloud>();
  pcl::fromPCLPointCloud2(mesh.cloud, *mesh_cloud);
  setInputMesh(mesh_cloud, mesh.polygons);
}

template <typename PointT>
void
pcl::AABBTreeCGAL<PointT>::setInputMesh(PointCloudConstPtr mesh_cloud,
                                        const std::vector<pcl::Vertices>& mesh_polygons)
{
#if DEBUG
  for (const pcl::Vertices& vertices : mesh_polygons)
    assert(vertices.vertices.size() == 3 &&
           "Invalid number of vertices for polygon (should be 3)!");
#endif

  triangles_.resize(mesh_polygons.size());
  std::transform(mesh_polygons.cbegin(),
                 mesh_polygons.cend(),
                 triangles_.begin(),
                 [mesh_cloud](const Vertices& vertices) {
                   const pcl::Indices& indices = vertices.vertices;

                   std::array<CGALPoint, 3> cgal_points;
                   std::transform(indices.cbegin(),
                                  indices.cend(),
                                  cgal_points.begin(),
                                  [mesh_cloud](const index_t& index) {
                                    const PointT& point = mesh_cloud->points[index];
                                    return CGALPoint(point.x, point.y, point.z);
                                  });

                   return Triangle(cgal_points[0], cgal_points[1], cgal_points[2]);
                 });

  tree_.reset(new Tree(triangles_.cbegin(), triangles_.cend()));
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
bool
pcl::AABBTreeCGAL<PointT>::checkForIntersection(const PointT& source,
                                                const Eigen::Vector3f& direction) const
{
  Ray ray = generateRay(source, direction);
  return tree_->do_intersect(ray);
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
size_t
pcl::AABBTreeCGAL<PointT>::numberOfIntersections(const PointT& source,
                                                 const Eigen::Vector3f& direction) const
{
  Ray ray = generateRay(source, direction);
  return tree_->number_of_intersected_primitives(ray);
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
std::vector<pcl::index_t>
pcl::AABBTreeCGAL<PointT>::getAllIntersections(const PointT& source,
                                               const Eigen::Vector3f& direction) const
{
  Ray ray = generateRay(source, direction);
  std::vector<Iterator> primitive_ids;
  tree_->all_intersected_primitives(ray, std::back_inserter(primitive_ids));

  std::vector<pcl::index_t> triangle_indices(primitive_ids.size());
  Iterator first_primitive_id = triangles_.cbegin();

  std::transform(primitive_ids.cbegin(),
                 primitive_ids.cend(),
                 triangle_indices.begin(),
                 [&first_primitive_id](const Iterator& primitive_id) {
                   return std::distance(first_primitive_id, primitive_id);
                 });

  return triangle_indices;
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
pcl::index_t
pcl::AABBTreeCGAL<PointT>::getAnyIntersection(const PointT& source,
                                              const Eigen::Vector3f& direction) const
{
  Ray ray = generateRay(source, direction);
  auto primitive_id = tree_->any_intersected_primitive(ray);

  if (primitive_id.has_value())
    return std::distance(triangles_.cbegin(), primitive_id.value());
  return -1;
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
pcl::index_t
pcl::AABBTreeCGAL<PointT>::getNearestIntersection(
    const PointT& source, const Eigen::Vector3f& direction) const
{
  Ray ray = generateRay(source, direction);
  auto primitive_id = tree_->first_intersected_primitive(ray);

  if (primitive_id.has_value())
    return std::distance(triangles_.cbegin(), primitive_id.value());
  return -1;
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
typename pcl::AABBTreeCGAL<PointT>::Ray
pcl::AABBTreeCGAL<PointT>::generateRay(const PointT& source,
                                       const Eigen::Vector3f& direction)
{
  CGALPoint cgal_source(source.x, source.y, source.z);
  CGALVector cgal_direction(direction[0], direction[1], direction[2]);
  return Ray(cgal_source, cgal_direction);
}

#define PCL_INSTANTIATE_AABBTreeCGAL(T) template class PCL_EXPORTS pcl::AABBTreeCGAL<T>;

#endif //#ifndef _PCL_AABB_TREE_AABB_TREE_IMPL_CGAL_H_
