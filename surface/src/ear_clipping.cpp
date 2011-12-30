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

#include "pcl/surface/ear_clipping.h"
#include <pcl/ros/conversions.h>
#include <pcl/pcl_config.h>

/////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::EarClipping::initCompute ()
{
  points_.reset (new pcl::PointCloud<pcl::PointXYZ>);

  if (!MeshProcessing::initCompute ())
    return (false);
  fromROSMsg (input_mesh_->cloud, *points_);

  return (true);
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::EarClipping::performProcessing (PolygonMesh& output)
{
  output.polygons.clear ();
  output.cloud = input_mesh_->cloud;
  for (int i = 0; i < (int) input_mesh_->polygons.size (); ++i)
    triangulate (input_mesh_->polygons[i], output);
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::EarClipping::triangulate (const Vertices& vertices, PolygonMesh& output)
{
  const int n_vertices = vertices.vertices.size ();

  if (n_vertices <= 3)
    return;

  std::vector<uint32_t> remaining_vertices (n_vertices);
  if (area (vertices.vertices) > 0) // clockwise?
    remaining_vertices = vertices.vertices;
  else
    for (int v = 0; v < n_vertices; v++)
      remaining_vertices[v] = vertices.vertices[n_vertices - 1 - v];

  // Avoid closed loops.
  if (remaining_vertices.front () == remaining_vertices.back ())
    remaining_vertices.erase (remaining_vertices.end () - 1);

  // null_iterations avoids infinite loops if the polygon is not simple.
  for (int u = remaining_vertices.size () - 1, null_iterations = 0;
      remaining_vertices.size () > 2 && null_iterations < (int)(remaining_vertices.size()*2);
      ++null_iterations, u = (u+1) % remaining_vertices.size ())
  {
    int v = (u + 1) % remaining_vertices.size ();
    int w = (u + 2) % remaining_vertices.size ();

    if (isEar (u, v, w, remaining_vertices))
    {
      Vertices triangle;
      triangle.vertices.resize (3);
      triangle.vertices[0] = remaining_vertices[u];
      triangle.vertices[1] = remaining_vertices[v];
      triangle.vertices[2] = remaining_vertices[w];
      output.polygons.push_back (triangle);
      remaining_vertices.erase (remaining_vertices.begin () + v);
      null_iterations = 0;
    }
  }
}


/////////////////////////////////////////////////////////////////////////////////////////////
float
pcl::EarClipping::area (const std::vector<uint32_t>& vertices)
{
  int n = vertices.size ();
  float area = 0.0f;
  PointXY prev_p, cur_p;
  for (int prev = n - 1, cur = 0; cur < n; prev = cur++)
  {
    toPointXY (points_->points[vertices[prev]], prev_p);
    toPointXY (points_->points[vertices[cur]], cur_p);
    area += crossProduct (prev_p, cur_p);
  }
  return (area * 0.5f);
}


/////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::EarClipping::isEar (int u, int v, int w, const std::vector<uint32_t>& vertices)
{
  PointXY p_u, p_v, p_w;
  toPointXY (points_->points[vertices[u]], p_u);
  toPointXY (points_->points[vertices[v]], p_v);
  toPointXY (points_->points[vertices[w]], p_w);

  // Avoid flat triangles.
  // FIXME: triangulation would fail if all the triangles are flat in the X-Y axis
  const float eps = 1e-15;
  PointXY p_uv, p_uw;
  difference (p_v, p_u, p_uv);
  difference (p_w, p_u, p_uw);
  if (crossProduct (p_uv, p_uw) < eps)
    return (false);

  PointXY p; 
  // Check if any other vertex is inside the triangle.
  for (int k = 0; k < (int)vertices.size (); k++)
  {
    if ((k == u) || (k == v) || (k == w))
      continue;
    toPointXY (points_->points[vertices[k]], p);
    if (isInsideTriangle (p_u, p_v, p_w, p))
      return (false);
  }
  return (true);
}

/////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::EarClipping::isInsideTriangle (const PointXY& u, const PointXY& v, const PointXY& w,
                                    const PointXY& p)
{
  PointXY vw, wu, uv, up, vp, wp;
  difference (w, v, vw);
  difference (u, w, wu);
  difference (v, u, uv);
  difference (p, u, up);
  difference (p, v, vp);
  difference (p, w, wp);

  // Check first side.
  if (crossProduct (vw, vp) < 0)
    return (false);

  // Check second side.
  if (crossProduct (uv, up) < 0)
    return (false);

  // Check third side.
  if (crossProduct (wu, wp) < 0)
    return (false);

  return (true);
}

