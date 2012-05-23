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

#include <pcl/surface/on_nurbs/nurbs_data.h>
#include <pcl/surface/on_nurbs/triangulation.h>
#include <pcl/surface/on_nurbs/fitting_curve_2d_pdm.h>
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

      pcl::Vertices v1;
      v1.vertices.push_back (i0);
      v1.vertices.push_back (i1);
      v1.vertices.push_back (i2);
      vertices.push_back (v1);

      pcl::Vertices v2;
      v2.vertices.push_back (i0);
      v2.vertices.push_back (i2);
      v2.vertices.push_back (i3);
      vertices.push_back (v2);
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
Triangulation::convertSurface2PolygonMesh (const ON_NurbsSurface &nurbs, PolygonMesh &mesh, unsigned resolution)
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

void
Triangulation::convertTrimmedSurface2PolygonMesh (const ON_NurbsSurface &nurbs, const ON_NurbsCurve &curve,
                                                  PolygonMesh &mesh, unsigned resolution)
{
  // copy knots
  if (nurbs.m_knot_capacity[0] <= 1 || nurbs.m_knot_capacity[1] <= 1)
  {
    printf ("[Triangulation::convert] Warning: ON knot vector empty.\n");
    return;
  }

  mesh.polygons.clear ();

  double x0 = nurbs.Knot (0, 0);
  double x1 = nurbs.Knot (0, nurbs.m_knot_capacity[0] - 1);
  double w = x1 - x0;
  double y0 = nurbs.Knot (1, 0);
  double y1 = nurbs.Knot (1, nurbs.m_knot_capacity[1] - 1);
  double h = y1 - y0;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<pcl::Vertices> polygons;
  createVertices (cloud, x0, y0, 0.0, w, h, resolution, resolution);
  createIndices (polygons, 0, resolution, resolution);

  std::vector<uint32_t> out_idx;
  pcl::on_nurbs::vector_vec2d out_pc;

  for (unsigned i = 0; i < polygons.size (); i++)
  {
    unsigned in (0);
    pcl::Vertices &poly = polygons[i];

    std::vector<uint32_t> out_idx_tmp;
    pcl::on_nurbs::vector_vec2d out_pc_tmp;

    for (std::size_t j = 0; j < poly.vertices.size (); j++)
    {
      double err;
      Eigen::Vector2d pc, tc;
      uint32_t &vi = poly.vertices[j];
      pcl::PointXYZ &v = cloud->at (vi);
      Eigen::Vector2d vp (v.x, v.y);
      double param = pcl::on_nurbs::FittingCurve2d::findClosestElementMidPoint (curve, vp);
      pcl::on_nurbs::FittingCurve2d::inverseMapping (curve, vp, param, err, pc, tc);
      Eigen::Vector3d a (vp (0) - pc (0), vp (1) - pc (1), 0.0);
      Eigen::Vector3d b (tc (0), tc (1), 0.0);
      Eigen::Vector3d z = a.cross (b);
      if (z (2) >= 0.0)
        in++;
      else
      {
        out_idx_tmp.push_back (vi);
        out_pc_tmp.push_back (pc);
      }
    }

    if (in > 0)
    {
      mesh.polygons.push_back (poly);
      if (in < poly.vertices.size ())
      {
        for (std::size_t j = 0; j < out_idx_tmp.size (); j++)
        {
          out_idx.push_back (out_idx_tmp[j]);
          out_pc.push_back (out_pc_tmp[j]);
        }
      }
    }
  }

  for (std::size_t i = 0; i < out_idx.size (); i++)
  {
    pcl::PointXYZ &v = cloud->at (out_idx[i]);
    Eigen::Vector2d &pc = out_pc[i];
    v.x = pc (0);
    v.y = pc (1);
  }

  for (std::size_t i = 0; i < cloud->size (); i++)
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

void
Triangulation::convertSurface2Vertices (const ON_NurbsSurface &nurbs, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                        std::vector<pcl::Vertices> &vertices, unsigned resolution)
{
  // copy knots
  if (nurbs.m_knot_capacity[0] <= 1 || nurbs.m_knot_capacity[1] <= 1)
  {
    printf ("[Triangulation::convert] Warning: ON knot vector empty.\n");
    return;
  }

  cloud->clear ();
  vertices.clear ();

  double x0 = nurbs.Knot (0, 0);
  double x1 = nurbs.Knot (0, nurbs.m_knot_capacity[0] - 1);
  double w = x1 - x0;
  double y0 = nurbs.Knot (1, 0);
  double y1 = nurbs.Knot (1, nurbs.m_knot_capacity[1] - 1);
  double h = y1 - y0;

  createVertices (cloud, x0, y0, 0.0, w, h, resolution, resolution);
  createIndices (vertices, 0, resolution, resolution);

  for (unsigned i = 0; i < cloud->size (); i++)
  {
    pcl::PointXYZ &v = cloud->at (i);

    double point[9];
    nurbs.Evaluate (v.x, v.y, 1, 3, point);

    v.x = static_cast<float> (point[0]);
    v.y = static_cast<float> (point[1]);
    v.z = static_cast<float> (point[2]);
  }
}

void
Triangulation::convertCurve2PointCloud (const ON_NurbsCurve &nurbs, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                                        unsigned resolution)
{
  // copy knots
  if (nurbs.m_knot_capacity <= 1)
  {
    printf ("[Triangulation::convert] Warning: ON knot vector empty.\n");
    return;
  }

  cloud->clear ();

  if (resolution < 2)
    resolution = 2;

  int cp_red = nurbs.Order () - 2;

  // for each element in the nurbs curve
  for (int i = 1; i < nurbs.KnotCount () - 1 - cp_red; i++)
  {
    double dr = 1.0 / (resolution - 1);
    double xi0 = nurbs.m_knot[i];
    double xid = (nurbs.m_knot[i + 1] - xi0);

    for (unsigned j = 0; j < resolution; j++)
    {
      double xi = (xi0 + j * dr * xid);
      pcl::PointXYZRGB p;

      double points[3];
      nurbs.Evaluate (xi, 0, 3, points);
      p.x = static_cast<float> (points[0]);
      p.y = static_cast<float> (points[1]);
      p.z = static_cast<float> (points[2]);
      p.r = 255;
      p.g = 0;
      p.b = 0;

      cloud->push_back (p);
    }

  }
}

void
Triangulation::convertCurve2PointCloud (const ON_NurbsCurve &curve, const ON_NurbsSurface &surf,
                                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, unsigned resolution)
{
  // copy knots
  if (curve.m_knot_capacity <= 1)
  {
    printf ("[Triangulation::Convert] Warning: ON knot vector empty.\n");
    return;
  }

  cloud->clear ();

  if (resolution < 2)
    resolution = 2;

  int cp_red = curve.Order () - 2;

  // for each element of the nurbs curve
  for (int i = 1; i < curve.KnotCount () - 1 - cp_red; i++)
  {
    double dr = 1.0 / (resolution - 1);

    double xi0 = curve.m_knot[i];
    double xid = (curve.m_knot[i + 1] - xi0);

    for (unsigned j = 0; j < resolution; j++)
    {
      pcl::PointXYZRGB pt;

      double xi = (xi0 + j * dr * xid);

      double p[3];
      double pp[3];
      curve.Evaluate (xi, 0, 2, pp);
      surf.Evaluate (pp[0], pp[1], 0, 3, p);
      pt.x = p[0];
      pt.y = p[1];
      pt.z = p[2];
      pt.r = 255;
      pt.g = 0;
      pt.b = 0;

      cloud->push_back (pt);
    }

  }
}
