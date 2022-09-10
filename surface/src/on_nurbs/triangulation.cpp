/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012-, Open Perception, Inc.
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
 * 
 *
 */

#include <pcl/surface/on_nurbs/nurbs_data.h>
#include <pcl/surface/on_nurbs/triangulation.h>
#include <pcl/surface/on_nurbs/fitting_curve_2d_apdm.h>
#include <pcl/conversions.h>

#include <Eigen/Geometry> // for cross

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
  float dx = width / float (segX);
  float dy = height / float (segY);

  for (unsigned j = 0; j <= segY; j++)
  {
    for (unsigned i = 0; i <= segX; i++)
    {
      v.x = x0 + float (i) * dx;
      v.y = y0 + float (j) * dy;
      v.z = z0;
      cloud->push_back (v);
    }
  }
}

bool
Triangulation::isInside(const ON_NurbsCurve &curve, const pcl::PointXYZ &v)
{
  Eigen::Vector2d vp (v.x, v.y);

  Eigen::Vector3d a0, a1;
  pcl::on_nurbs::NurbsTools::computeBoundingBox (curve, a0, a1);
  double rScale = 1.0 / pcl::on_nurbs::NurbsTools::computeRScale (a0, a1);

  Eigen::Vector2d pc, tc;
  double err;

  if (curve.Order () == 2)
    pcl::on_nurbs::FittingCurve2dAPDM::inverseMappingO2 (curve, vp, err, pc, tc);
  else
  {
    double param = pcl::on_nurbs::FittingCurve2dAPDM::findClosestElementMidPoint (curve, vp);
    pcl::on_nurbs::FittingCurve2dAPDM::inverseMapping (curve, vp, param, err, pc, tc, rScale);
  }

  Eigen::Vector3d a (vp (0) - pc (0), vp (1) - pc (1), 0.0);
  Eigen::Vector3d b (tc (0), tc (1), 0.0);
  Eigen::Vector3d z = a.cross (b);

  return (z (2) >= 0.0);
}

//void
//Triangulation::convertObject2PolygonMesh (const NurbsObject &object, PolygonMesh &mesh, unsigned resolution)
//{
//  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
//
//  for (unsigned s = 0; s < object.size (); s++)
//  {
//    ON_NurbsSurface nurbs = object.getSurface (s);
//
//    if (nurbs.KnotCount(0) <= 1 || nurbs.KnotCount(1) <= 1)
//    {
//      printf ("[Triangulation::convertObject2PolygonMesh] Warning surface %d: ON knot vector empty.\n", s);
//      continue;
//    }
//
//    double x0 = nurbs.Knot (0, 0);
//    double x1 = nurbs.Knot (0, nurbs.KnotCount(0) - 1);
//    double w = x1 - x0;
//    double y0 = nurbs.Knot (1, 0);
//    double y1 = nurbs.Knot (1, nurbs.KnotCount(1) - 1);
//    double h = y1 - y0;
//
//    unsigned vidx = cloud->size ();
//    createVertices (cloud, x0, y0, 0.0, w, h, resolution, resolution);
//    createIndices (mesh.polygons, vidx, resolution, resolution);
//
//    for (unsigned i = vidx; i < cloud->size (); i++)
//    {
//      pcl::PointXYZ &v = cloud->at (i);
//
//      double point[9];
//      nurbs.Evaluate (v.x, v.y, 1, 3, point);
//
//      v.x = point[0];
//      v.y = point[1];
//      v.z = point[2];
//    }
//
//  }
//
//  toPCLPointCloud2 (*cloud, mesh.cloud);
//}

void
Triangulation::convertSurface2PolygonMesh (const ON_NurbsSurface &nurbs, PolygonMesh &mesh, unsigned resolution)
{
  // copy knots
  if (nurbs.KnotCount (0) <= 1 || nurbs.KnotCount (1) <= 1)
  {
    printf ("[Triangulation::convertSurface2PolygonMesh] Warning: ON knot vector empty.\n");
    return;
  }

  double x0 = nurbs.Knot (0, 0);
  double x1 = nurbs.Knot (0, nurbs.KnotCount (0) - 1);
  double w = x1 - x0;
  double y0 = nurbs.Knot (1, 0);
  double y1 = nurbs.Knot (1, nurbs.KnotCount (1) - 1);
  double h = y1 - y0;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  mesh.polygons.clear ();
  createVertices (cloud, float (x0), float (y0), 0.0f, float (w), float (h), resolution, resolution);
  createIndices (mesh.polygons, 0, resolution, resolution);

  for (auto &v : *cloud)
  {
    double point[9];
    nurbs.Evaluate (v.x, v.y, 1, 3, point);

    v.x = float (point[0]);
    v.y = float (point[1]);
    v.z = float (point[2]);
  }

  toPCLPointCloud2 (*cloud, mesh.cloud);
}

void
Triangulation::convertTrimmedSurface2PolygonMesh (const ON_NurbsSurface &nurbs, const ON_NurbsCurve &curve,
                                                  PolygonMesh &mesh, unsigned resolution)
{
  // copy knots
  if (nurbs.KnotCount (0) <= 1 || nurbs.KnotCount (1) <= 1 || curve.KnotCount () <= 1)
  {
    printf ("[Triangulation::convertTrimmedSurface2PolygonMesh] Warning: ON knot vector empty.\n");
    return;
  }

  mesh.polygons.clear ();

  double x0 = nurbs.Knot (0, 0);
  double x1 = nurbs.Knot (0, nurbs.KnotCount (0) - 1);
  double w = x1 - x0;
  double y0 = nurbs.Knot (1, 0);
  double y1 = nurbs.Knot (1, nurbs.KnotCount (1) - 1);
  double h = y1 - y0;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<pcl::Vertices> polygons;
  createVertices (cloud, float (x0), float (y0), 0.0f, float (w), float (h), resolution, resolution);
  createIndices (polygons, 0, resolution, resolution);

  vector_vec2d points (cloud->size (), Eigen::Vector2d ());
  std::vector<bool> pt_is_in (cloud->size (), false);

  Eigen::Vector3d a0, a1;
  pcl::on_nurbs::NurbsTools::computeBoundingBox (curve, a0, a1);
  double rScale = 1.0 / pcl::on_nurbs::NurbsTools::computeRScale (a0, a1);

  std::vector<std::uint32_t> out_idx;
  pcl::on_nurbs::vector_vec2d out_pc;

  for (std::size_t i = 0; i < cloud->size (); i++)
  {
    double err;
    Eigen::Vector2d pc, tc;
    pcl::PointXYZ &v = cloud->at (i);
    Eigen::Vector2d vp (v.x, v.y);

    if (curve.Order () == 2)
      pcl::on_nurbs::FittingCurve2dAPDM::inverseMappingO2 (curve, vp, err, pc, tc);
    else
    {
      double param = pcl::on_nurbs::FittingCurve2dAPDM::findClosestElementMidPoint (curve, vp);
      pcl::on_nurbs::FittingCurve2dAPDM::inverseMapping (curve, vp, param, err, pc, tc, rScale);
    }

    Eigen::Vector3d a (vp (0) - pc (0), vp (1) - pc (1), 0.0);
    Eigen::Vector3d b (tc (0), tc (1), 0.0);
    Eigen::Vector3d z = a.cross (b);

    points[i] = pc;
    pt_is_in[i] = (z (2) >= 0.0);
  }

  for (const auto &poly : polygons)
  {
    unsigned in (0);
    std::vector<std::uint32_t> out_idx_tmp;
    pcl::on_nurbs::vector_vec2d out_pc_tmp;

    for (const auto &vi : poly.vertices)
    {
      if (pt_is_in[vi])
        in++;
      else
      {
        out_idx_tmp.push_back (vi);
        out_pc_tmp.push_back (points[vi]);
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
    v.x = float (pc (0));
    v.y = float (pc (1));
  }

  for (auto &v : *cloud)
  {
    Eigen::Vector3d tu, tv;

    double point[3];
    nurbs.Evaluate (v.x, v.y, 0, 3, point);

    v.x = float (point[0]);
    v.y = float (point[1]);
    v.z = float (point[2]);
    //    tu[0] = point[3];
    //    tu[1] = point[4];
    //    tu[2] = point[5];
    //    tv[0] = point[6];
    //    tv[1] = point[7];
    //    tv[2] = point[8];

    // TODO: add normals to mesh
  }

  toPCLPointCloud2 (*cloud, mesh.cloud);
}

void
Triangulation::convertTrimmedSurface2PolygonMesh (const ON_NurbsSurface &nurbs, const ON_NurbsCurve &curve,
                                                  PolygonMesh &mesh, unsigned resolution, vector_vec3d &start,
                                                  vector_vec3d &end)
{
  // copy knots
  if (nurbs.KnotCount (0) <= 1 || nurbs.KnotCount (1) <= 1 || curve.KnotCount () <= 1)
  {
    printf ("[Triangulation::convertTrimmedSurface2PolygonMesh] Warning: ON knot vector empty.\n");
    return;
  }

  mesh.polygons.clear ();

  double x0 = nurbs.Knot (0, 0);
  double x1 = nurbs.Knot (0, nurbs.KnotCount (0) - 1);
  double w = x1 - x0;
  double y0 = nurbs.Knot (1, 0);
  double y1 = nurbs.Knot (1, nurbs.KnotCount (1) - 1);
  double h = y1 - y0;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<pcl::Vertices> polygons;
  createVertices (cloud, float (x0), float (y0), 0.0f, float (w), float (h), resolution, resolution);
  createIndices (polygons, 0, resolution, resolution);

  vector_vec2d points (cloud->size (), Eigen::Vector2d ());
  std::vector<double> params (cloud->size (), 0.0);
  std::vector<bool> pt_is_in (cloud->size (), false);

  std::vector<std::uint32_t> out_idx;
  pcl::on_nurbs::vector_vec2d out_pc;

  Eigen::Vector3d a0, a1;
  pcl::on_nurbs::NurbsTools::computeBoundingBox (curve, a0, a1);
  double rScale = 1.0 / pcl::on_nurbs::NurbsTools::computeRScale (a0, a1);

  for (std::size_t i = 0; i < cloud->size (); i++)
  {
    double err, param;
    Eigen::Vector2d pc, tc;
    pcl::PointXYZ &v = cloud->at (i);
    Eigen::Vector2d vp (v.x, v.y);

    if (curve.Order () == 2)
      param = pcl::on_nurbs::FittingCurve2dAPDM::inverseMappingO2 (curve, vp, err, pc, tc);
    else
    {
      param = pcl::on_nurbs::FittingCurve2dAPDM::findClosestElementMidPoint (curve, vp);
      param = pcl::on_nurbs::FittingCurve2dAPDM::inverseMapping (curve, vp, param, err, pc, tc, rScale, 100, 1e-4, true);
    }

    Eigen::Vector3d a (vp (0) - pc (0), vp (1) - pc (1), 0.0);
    Eigen::Vector3d b (tc (0), tc (1), 0.0);
    Eigen::Vector3d z = a.cross (b);

    points[i] = pc;
    params[i] = param;
    pt_is_in[i] = (z (2) >= 0.0);

    end.push_back (Eigen::Vector3d (pc (0), pc (1), 0.0));
    start.push_back (Eigen::Vector3d (vp (0), vp (1), 0.0));
  }

  for (const auto &poly : polygons)
  {
    unsigned in (0);
    std::vector<std::uint32_t> out_idx_tmp;
    pcl::on_nurbs::vector_vec2d out_pc_tmp;

    for (const auto &vi : poly.vertices)
    {
      if (pt_is_in[vi])
        in++;
      else
      {
        out_idx_tmp.push_back (vi);
        out_pc_tmp.push_back (points[vi]);
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
    v.x = float (pc (0));
    v.y = float (pc (1));
  }

  for (auto &v : *cloud)
  {
    double point[3];
    nurbs.Evaluate (v.x, v.y, 0, 3, point);

    v.x = float (point[0]);
    v.y = float (point[1]);
    v.z = float (point[2]);
  }

  for (std::size_t i = 0; i < start.size (); i++)
  {
    Eigen::Vector3d &p1 = start[i];
    Eigen::Vector3d &p2 = end[i];

    double point[3];
    nurbs.Evaluate (p1 (0), p1 (1), 0, 3, point);
    p1 (0) = point[0];
    p1 (1) = point[1];
    p1 (2) = point[2];

    nurbs.Evaluate (p2 (0), p2 (1), 0, 3, point);
    p2 (0) = point[0];
    p2 (1) = point[1];
    p2 (2) = point[2];
  }

  toPCLPointCloud2 (*cloud, mesh.cloud);
}

void
Triangulation::convertSurface2Vertices (const ON_NurbsSurface &nurbs, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                        std::vector<pcl::Vertices> &vertices, unsigned resolution)
{
  // copy knots
  if (nurbs.KnotCount (0) <= 1 || nurbs.KnotCount (1) <= 1)
  {
    printf ("[Triangulation::convertSurface2Vertices] Warning: ON knot vector empty.\n");
    return;
  }

  cloud->clear ();
  vertices.clear ();

  double x0 = nurbs.Knot (0, 0);
  double x1 = nurbs.Knot (0, nurbs.KnotCount (0) - 1);
  double w = x1 - x0;
  double y0 = nurbs.Knot (1, 0);
  double y1 = nurbs.Knot (1, nurbs.KnotCount (1) - 1);
  double h = y1 - y0;

  createVertices (cloud, float (x0), float (y0), 0.0f, float (w), float (h), resolution, resolution);
  createIndices (vertices, 0, resolution, resolution);

  for (auto &v : *cloud)
  {
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
    printf ("[Triangulation::convertCurve2PointCloud] Warning: ON knot vector empty.\n");
    return;
  }

  cloud->clear ();

  if (resolution < 2)
    resolution = 2;

  int cp_red = nurbs.Order () - 2;

  // for each element in the nurbs curve
  for (int i = cp_red; i < nurbs.KnotCount () - 1 - cp_red; i++)
  {
    double dr = 1.0 / (resolution - 1);
    double xi0 = nurbs.m_knot[i];
    double xid = (nurbs.m_knot[i + 1] - xi0);

    for (unsigned j = 0; j < resolution; j++)
    {
      double xi = (xi0 + dr * xid * j);
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
    printf ("[Triangulation::convertCurve2PointCloud] Warning: ON knot vector empty.\n");
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
      pt.x = float (p[0]);
      pt.y = float (p[1]);
      pt.z = float (p[2]);
      pt.r = 255;
      pt.g = 0;
      pt.b = 0;

      cloud->push_back (pt);
    }

  }
}
