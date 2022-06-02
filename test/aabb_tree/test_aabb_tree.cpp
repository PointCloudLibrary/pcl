/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009-2011, Willow Garage, Inc.
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
 * test_aabb_tree.cpp
 * Adapted from: test/kdtree/test_kdtree.cpp
 *               filters/impl/crop_hull.hpp
 * Created on: Jun 05, 2022
 * Author: Ramzi Sabra
 */

#include <pcl/aabb_tree/aabb_tree_cgal.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/test/gtest.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

#include <iostream> // For debug
#include <iterator>
#include <unordered_set>

using namespace pcl;

using Point = PointXYZ;
using Cloud = PointCloud<Point>;

boost::property_tree::ptree xml_property_tree;

PolygonMesh mesh_in;
Cloud mesh_cloud;

bool
rayTriangleIntersect(const Point& point,
                     const Eigen::Vector3f& ray,
                     const Vertices& verts,
                     const Cloud& cloud)
{
  // Algorithm here is adapted from:
  // http://softsurfer.com/Archive/algorithm_0105/algorithm_0105.htm#intersect_RayTriangle()
  //
  // Original copyright notice:
  // Copyright 2001, softSurfer (www.softsurfer.com)
  // This code may be freely used and modified for any purpose
  // providing that this copyright notice is included with it.
  //
  assert(verts.vertices.size() == 3);

  const Eigen::Vector3f p = point.getVector3fMap();
  const Eigen::Vector3f a = cloud[verts.vertices[0]].getVector3fMap();
  const Eigen::Vector3f b = cloud[verts.vertices[1]].getVector3fMap();
  const Eigen::Vector3f c = cloud[verts.vertices[2]].getVector3fMap();
  const Eigen::Vector3f u = b - a;
  const Eigen::Vector3f v = c - a;
  const Eigen::Vector3f n = u.cross(v);
  const float n_dot_ray = n.dot(ray);

  if (std::fabs(n_dot_ray) < 1e-9)
    return (false);

  const float r = n.dot(a - p) / n_dot_ray;

  if (r < 0)
    return (false);

  const Eigen::Vector3f w = p + r * ray - a;
  const float denominator = u.dot(v) * u.dot(v) - u.dot(u) * v.dot(v);
  const float s_numerator = u.dot(v) * w.dot(v) - v.dot(v) * w.dot(u);
  const float s = s_numerator / denominator;
  if (s < 0 || s > 1)
    return (false);

  const float t_numerator = u.dot(v) * w.dot(u) - u.dot(u) * w.dot(v);
  const float t = t_numerator / denominator;
  if (t < 0 || s + t > 1)
    return (false);

  return (true);
}

template <typename T>
class PCLAABBTreeTestFixture : public ::testing::Test {
public:
  using Tree = T;
};

using AABBTreeTestTypes = ::testing::Types<AABBTreeCGAL<PointXYZ>>;
TYPED_TEST_SUITE(PCLAABBTreeTestFixture, AABBTreeTestTypes);

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TYPED_TEST(PCLAABBTreeTestFixture, AABBTree_intersections)
{
  using Tree = typename TestFixture::Tree;
  using Ray = std::pair<Point, Eigen::Vector3f>;

  std::vector<Ray> rays{
      {Point(0.0f, 70.0f, 0.0f), Eigen::Vector3f(1.0f, 0.0f, 0.0f)}, // 0 intersections
      {Point(0.0f, 70.0f, 0.0f), Eigen::Vector3f(1.0f, 1.0f, 1.0f)}, // 2 intersections
      {Point(70.0f, 100.0f, 80.0f), Eigen::Vector3f(1.0f, 0.0f, 0.0f)} // 1 intersection
  };

  std::vector<Indices> triangle_indices_vec(rays.size());

  std::transform(
      rays.cbegin(), rays.cend(), triangle_indices_vec.begin(), [](const Ray& ray) {
        const Point& source = ray.first;
        const Eigen::Vector3f& direction = ray.second;

        Indices indices;

        for (index_t index = 0; index < mesh_in.polygons.size(); ++index) {
          const Vertices& vertices = mesh_in.polygons[index];
          if (rayTriangleIntersect(source, direction, vertices, mesh_cloud))
            indices.push_back(index);
        }

        return indices;
      });

  Tree tree;
  tree.setInputMesh(mesh_in);

  auto ray_itr = rays.cbegin();
  auto triangle_indices_itr = triangle_indices_vec.cbegin();

  for (; ray_itr != rays.cend(); ++ray_itr, ++triangle_indices_itr) {
    const Ray& ray = *ray_itr;
    const Indices& triangle_indices = *triangle_indices_itr;

    const Point& source = ray.first;
    const Eigen::Vector3f& direction = ray.second;

    EXPECT_EQ(tree.checkForIntersection(source, direction),
              triangle_indices.size() != 0);
    EXPECT_EQ(tree.numberOfIntersections(source, direction), triangle_indices.size());

    std::unordered_set<index_t> triangle_indices_set, tree_triangle_indices_set;

    {
      auto inserter = std::inserter(triangle_indices_set, triangle_indices_set.end());
      std::copy(triangle_indices.cbegin(), triangle_indices.cend(), inserter);
    }

    {
      Indices tree_triangle_indices = tree.getAllIntersections(source, direction);
      auto inserter =
          std::inserter(tree_triangle_indices_set, tree_triangle_indices_set.end());
      std::copy(tree_triangle_indices.cbegin(), tree_triangle_indices.cend(), inserter);
    }

    EXPECT_EQ(tree_triangle_indices_set, triangle_indices_set);

    index_t any_triangle_index = tree.getAnyIntersection(source, direction);
    index_t nearest_triangle_index = tree.getNearestIntersection(source, direction);

    if (triangle_indices_set.size() == 0) {
      EXPECT_EQ(any_triangle_index, -1);
      EXPECT_EQ(nearest_triangle_index, -1);
    }
    else {
      EXPECT_TRUE(triangle_indices_set.count(any_triangle_index) == 1);
      EXPECT_TRUE(triangle_indices_set.count(nearest_triangle_index) == 1);
    }
  }
}

/* ---[ */
int
main(int argc, char** argv)
{
  // Load the standard PLY file from disk
  if (argc < 2) {
    std::cerr << "No test file given. Please download `tum_rabbit.vtk` and pass its "
                 "path to the test."
              << std::endl;
    return (-1);
  }

  // Load in the mesh
  io::loadPolygonFileVTK(argv[1], mesh_in);
  pcl::fromPCLPointCloud2(mesh_in.cloud, mesh_cloud);

  testing::InitGoogleTest(&argc, argv);

  return (RUN_ALL_TESTS());
}
/* ]--- */
