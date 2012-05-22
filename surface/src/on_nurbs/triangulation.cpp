/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Thomas Mörwald, Jonathan Balzer, Inc.
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
 *   * Neither the name of Thomas Mörwald or Jonathan Balzer nor the names of its
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
 * @author thomas.moerwald
 *
 */

#include <pcl/surface/on_nurbs/triangulation.h>
#include <pcl/ros/conversions.h>

using namespace pcl;
using namespace on_nurbs;

void
Triangulation::createIndices (std::vector<pcl::Vertices> &vertices, unsigned vidx, unsigned segX, unsigned segY)
{
  for (unsigned j = 0; j < segY; j++)
  {
    for (unsigned i = 0; i < segX; i++)
    {

      unsigned i0 = vidx + (segX + 1) * j + i;
      unsigned i1 = vidx + (segX + 1) * j + i + 1;
      unsigned i2 = vidx + (segX + 1) * (j + 1) + i + 1;
      unsigned i3 = vidx + (segX + 1) * (j + 1) + i;

      pcl::Vertices v;
      v.vertices.push_back (i0);
      v.vertices.push_back (i1);
      v.vertices.push_back (i2);
      v.vertices.push_back (i3);

      vertices.push_back (v);

    }
  }
}

void
Triangulation::createVertices (pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, float x0, float y0, float z0, float width,
                               float height, unsigned segX, unsigned segY)
{
  pcl::PointXYZ v;
  float dx = width / segX;
  float dy = height / segY;

  for (unsigned j = 0; j <= segY; j++)
  {
    for (unsigned i = 0; i <= segX; i++)
    {
      v.x = x0 + i * dx;
      v.y = y0 + j * dy;
      v.z = z0;
      cloud->push_back (v);
    }
  }
}

void
Triangulation::convert (const ON_NurbsSurface &nurbs, PolygonMesh &mesh, unsigned resolution)
{
  // copy knots
  if (nurbs.m_knot_capacity[0] <= 1 || nurbs.m_knot_capacity[1] <= 1)
  {
    printf ("[Triangulation::convert] Warning: ON knot vector empty.\n");
    return;
  }

  double x0 = nurbs.Knot (0, 0);
  double x1 = nurbs.Knot (0, nurbs.m_knot_capacity[0] - 1);
  double w = x1 - x0;
  double y0 = nurbs.Knot (1, 0);
  double y1 = nurbs.Knot (1, nurbs.m_knot_capacity[1] - 1);
  double h = y1 - y0;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  createVertices (cloud, x0, y0, 0.0, w, h, resolution, resolution);
  createIndices (mesh.polygons, 0, resolution, resolution);

  for (unsigned i = 0; i < cloud->size (); i++)
  {
    pcl::PointXYZ &v = cloud->at (i);

    double point[9];
    nurbs.Evaluate (v.x, v.y, 1, 3, point);

    v.x = point[0];
    v.y = point[1];
    v.z = point[2];
  }

  toROSMsg (*cloud, mesh.cloud);
}
