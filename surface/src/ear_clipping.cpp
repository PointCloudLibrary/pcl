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
 * $Id$
 *
 */

#include <pcl/surface/ear_clipping.h>
#include <pcl/conversions.h>
#include <pcl/pcl_config.h>

/////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::EarClipping::initCompute ()
{
  points_.reset (new pcl::PointCloud<pcl::PointXYZ>);

  if (!MeshProcessing::initCompute ())
    return (false);
  fromPCLPointCloud2 (input_mesh_->cloud, *points_);

  return (true);
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::EarClipping::performProcessing (PolygonMesh& output)
{
  output.polygons.clear ();
  output.cloud = input_mesh_->cloud;
  for (int i = 0; i < static_cast<int> (input_mesh_->polygons.size ()); ++i)
    triangulate (input_mesh_->polygons[i], output);
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::EarClipping::triangulate (const Vertices& vertices, PolygonMesh& output)
{
  const size_t n_vertices = vertices.vertices.size ();

  if (n_vertices < 3)
    return;
  else if (n_vertices == 3)
  {
    output.polygons.emplace_back( vertices );
    return;
  }

  std::vector<uint32_t> remaining_vertices = vertices.vertices;
  size_t count = triangulate(remaining_vertices, output);

  // if the input vertices order is anti-clockwise, it always left a
  // convex polygon and start infinite loops, which means will left more
  // than 3 points.
  if(remaining_vertices.size() < 3)return;

  output.polygons.erase(output.polygons.cend(), output.polygons.cend() + count);
  remaining_vertices.resize(n_vertices);
  for (size_t v = 0; v < n_vertices; v++)
      remaining_vertices[v] = vertices.vertices[n_vertices - 1 - v];
  triangulate(remaining_vertices, output);
}

/////////////////////////////////////////////////////////////////////////////////////////////
size_t
pcl::EarClipping::triangulate (std::vector<uint32_t>& vertices, PolygonMesh& output)
{
  // triangles count
  size_t count = 0;

  // Avoid closed loops.
  if (vertices.front () == vertices.back ())
    vertices.erase (vertices.end () - 1);

  // null_iterations avoids infinite loops if the polygon is not simple.
  for (int u = static_cast<int> (vertices.size ()) - 1, null_iterations = 0;
      vertices.size () > 2 && null_iterations < static_cast<int >(vertices.size () * 2);
      ++null_iterations, u = (u+1) % static_cast<int> (vertices.size ()))
  {
    int v = (u + 1) % static_cast<int> (vertices.size ());
    int w = (u + 2) % static_cast<int> (vertices.size ());

    if (vertices.size() == 3 || isEar (u, v, w, vertices))
    {
      Vertices triangle;
      triangle.vertices.resize (3);
      triangle.vertices[0] = vertices[u];
      triangle.vertices[1] = vertices[v];
      triangle.vertices[2] = vertices[w];
      output.polygons.emplace_back (triangle);
      vertices.erase (vertices.begin () + v);
      null_iterations = 0;
      count++;
    }
  }
  return count;
}

/////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::EarClipping::isEar (int u, int v, int w, const std::vector<uint32_t>& vertices)
{
  Eigen::Vector3f p_u, p_v, p_w;
  p_u = points_->points[vertices[u]].getVector3fMap();
  p_v = points_->points[vertices[v]].getVector3fMap();
  p_w = points_->points[vertices[w]].getVector3fMap();

  const float eps = 1e-15f;
  Eigen::Vector3f p_vu, p_vw;
  p_vu = p_u - p_v;
  p_vw = p_w - p_v;

  // Avoid flat triangles and concave vertex
  Eigen::Vector3f cross = p_vu.cross(p_vw);
  if ((cross[2] > 0) || (cross.norm() < eps))
    return (false);

  Eigen::Vector3f p;
  // Check if any other vertex is inside the triangle.
  for (int k = 0; k < static_cast<int> (vertices.size ()); k++)
  {
    if ((k == u) || (k == v) || (k == w))
      continue;
    p = points_->points[vertices[k]].getVector3fMap();

    if (isInsideTriangle (p_u, p_v, p_w, p))
      return (false);
  }
  return (true);
}

/////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::EarClipping::isInsideTriangle (const Eigen::Vector3f& u,
                                    const Eigen::Vector3f& v,
                                    const Eigen::Vector3f& w,
                                    const Eigen::Vector3f& p)
{
  // see http://www.blackpawn.com/texts/pointinpoly/default.html
  // Barycentric Coordinates
  Eigen::Vector3f v0 = w - u;
  Eigen::Vector3f v1 = v - u;
  Eigen::Vector3f v2 = p - u;

  // Compute dot products
  float dot00 = v0.dot(v0);
  float dot01 = v0.dot(v1);
  float dot02 = v0.dot(v2);
  float dot11 = v1.dot(v1);
  float dot12 = v1.dot(v2);

  // Compute barycentric coordinates
  float invDenom = 1 / (dot00 * dot11 - dot01 * dot01);
  float a = (dot11 * dot02 - dot01 * dot12) * invDenom;
  float b = (dot00 * dot12 - dot01 * dot02) * invDenom;

  // Check if point is in triangle
  return (a >= 0) && (b >= 0) && (a + b < 1);
}

