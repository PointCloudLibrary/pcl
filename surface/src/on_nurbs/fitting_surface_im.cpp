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

#include <pcl/surface/on_nurbs/fitting_surface_im.h>
#include <stdexcept>

using namespace pcl;
using namespace on_nurbs;

pcl::PointXYZRGB
FittingSurfaceIM::computeMean () const
{
  pcl::PointXYZRGB u;
  u.x = 0.0;
  u.y = 0.0;
  u.z = 0.0;

  double ds = 1.0 / double (m_indices.size ());

  const pcl::PointCloud<pcl::PointXYZRGB> &cloud_ref = *m_cloud;
  for (size_t idx = 0; idx < m_indices.size (); idx++)
  {
    int i = m_indices[idx] % cloud_ref.width;
    int j = m_indices[idx] / cloud_ref.width;

    const pcl::PointXYZRGB &point = cloud_ref (i, j);
    if (pcl_isnan (point.x) || pcl_isnan (point.y) || pcl_isnan (point.z))
      continue;

    u.x += point.x * float (ds);
    u.y += point.y * float (ds);
    u.z += point.z * float (ds);
  }

  return u;
}

Eigen::Vector4d
FittingSurfaceIM::computeIndexBoundingBox (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                                           const std::vector<int> &indices)
{
  Eigen::Vector4d bb = Eigen::Vector4d (DBL_MAX, 0, DBL_MAX, 0);
  const pcl::PointCloud<pcl::PointXYZRGB> &cloud_ref = *cloud;

  for (size_t idx = 0; idx < indices.size (); idx++)
  {
    int i = indices[idx] % cloud_ref.width;
    int j = indices[idx] / cloud_ref.width;

    const pcl::PointXYZRGB &point = cloud_ref (i, j);
    if (pcl_isnan (point.x) || pcl_isnan (point.y) || pcl_isnan (point.z))
      continue;

    if (i < bb (0))
      bb (0) = i;
    if (i > bb (1))
      bb (1) = i;
    if (j < bb (2))
      bb (2) = j;
    if (j > bb (3))
      bb (3) = j;
  }
  return bb;
}

void
FittingSurfaceIM::setInputCloud (pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud)
{
  m_cloud = _cloud;
}

void
FittingSurfaceIM::setIndices (std::vector<int> &_indices)
{
  m_indices = _indices;
}

void
FittingSurfaceIM::setCamera (const Eigen::Matrix3d &i)
{
  m_intrinsic = i;
}

void
FittingSurfaceIM::setCamera (const Eigen::Matrix3f &i)
{
  printf("[FittingSurfaceIM::setCamera] Warning, this function is not tested!\n");
  m_intrinsic << i (0, 0), i (0, 1), i (0, 2), i (1, 0), i (1, 1), i (1, 2), i (2, 0), i (2, 1), i (2, 2);
}

std::vector<double>
FittingSurfaceIM::getElementVector (const ON_NurbsSurface &nurbs, int dim) // !
{
  std::vector<double> result;

  int idx_min = 0;
  int idx_max = nurbs.KnotCount (dim) - 1;
  if (nurbs.IsClosed (dim))
  {
    idx_min = nurbs.Order (dim) - 2;
    idx_max = nurbs.KnotCount (dim) - nurbs.Order (dim) + 1;
  }

  const double* knots = nurbs.Knot (dim);

  result.push_back (knots[idx_min]);

  //for(int E=(m_nurbs.Order(0)-2); E<(m_nurbs.KnotCount(0)-m_nurbs.Order(0)+2); E++) {
  for (int E = idx_min + 1; E <= idx_max; E++)
  {

    if (knots[E] != knots[E - 1]) // do not count double knots
      result.push_back (knots[E]);

  }

  return result;
}

void
FittingSurfaceIM::refine ()
{
  {
    int dim = 0;
    std::vector<double> elements = getElementVector (m_nurbs, dim);
    for (unsigned i = 0; i < elements.size () - 1; i++)
    {
      double xi = elements[i] + 0.5 * (elements[i + 1] - elements[i]);
      m_nurbs.InsertKnot (dim, xi, 1);
    }
  }
  {
    int dim = 1;
    std::vector<double> elements = getElementVector (m_nurbs, dim);
    for (unsigned i = 0; i < elements.size () - 1; i++)
    {
      double xi = elements[i] + 0.5 * (elements[i + 1] - elements[i]);
      m_nurbs.InsertKnot (dim, xi, 1);
    }
  }

  Eigen::Vector2d bbx (m_nurbs.Knot (0, 0), m_nurbs.Knot (0, m_nurbs.KnotCount (0) - 1));
  Eigen::Vector2d bby (m_nurbs.Knot (1, 0), m_nurbs.Knot (1, m_nurbs.KnotCount (1) - 1));

  int dx = int (bbx (1) - bbx (0));
  int dy = int (bby (1) - bby (0));
  double ddx = double (dx) / (m_nurbs.CVCount (0) - 1);
  double ddy = double (dy) / (m_nurbs.CVCount (1) - 1);

  m_cps_px.clear ();
  for (int i = 0; i < m_nurbs.CVCount (0); i++)
  {
    for (int j = 0; j < m_nurbs.CVCount (1); j++)
    {
      int px = int (bbx (0) + ddx * i);
      int py = int (bby (0) + ddy * j);
      m_cps_px.push_back (Eigen::Vector2i (px, py));
    }
  }
}

//void
//FittingSurfaceIM::compute (int nurbs_order, double damp)
//{
//  initSurface (nurbs_order);
//  assemble ();
//  solve (damp);
//}

void
FittingSurfaceIM::initSurface (int order, const Eigen::Vector4d &bb)
{
  m_cps_px.clear ();

  // bounding box in parameter space
  m_bb = bb;

  pcl::PointXYZRGB pt = computeMean ();

  double dx = bb (1) - bb (0);
  double dy = bb (3) - bb (2);
  double ddx = dx / (order - 1);
  double ddy = dy / (order - 1);

  //  printf ("cloud: %d %d\n", m_cloud->width, m_cloud->height);
  //  printf ("bb: %d %d %d %d\n", m_bb (0), m_bb (1), m_bb (2), m_bb (3));

  m_nurbs = ON_NurbsSurface (3, false, order, order, order, order);
  m_nurbs.MakeClampedUniformKnotVector (0, dx);
  m_nurbs.MakeClampedUniformKnotVector (1, dy);

  for (int i = 0; i < m_nurbs.KnotCount (0); i++)
  {
    double k = m_nurbs.Knot (0, i);
    m_nurbs.SetKnot (0, i, k + m_bb (0));
  }

  for (int i = 0; i < m_nurbs.KnotCount (1); i++)
  {
    double k = m_nurbs.Knot (1, i);
    m_nurbs.SetKnot (1, i, k + m_bb (2));
  }

  for (int i = 0; i < m_nurbs.Order (0); i++)
  {
    for (int j = 0; j < m_nurbs.Order (1); j++)
    {
      int px = int (m_bb (0) + ddx * i + 0.5);
      int py = int (m_bb (2) + ddy * j + 0.5);

      m_cps_px.push_back (Eigen::Vector2i (px, py));

      ON_3dPoint p;
      p.x = pt.z * (px - m_intrinsic (0, 2)) / m_intrinsic (0, 0);
      p.y = pt.z * (py - m_intrinsic (1, 2)) / m_intrinsic (1, 1);
      p.z = pt.z;

      m_nurbs.SetCV (i, j, p);
    }
  }

  //  ON_TextLog out;
  //  m_nurbs.Dump (out);
}

void
FittingSurfaceIM::assemble (bool inverse_mapping)
{
  int nInt = int (m_indices.size ());
  int nCageReg = (m_nurbs.m_cv_count[0] - 2) * (m_nurbs.m_cv_count[1] - 2);
  int nCageRegBnd = 2 * (m_nurbs.m_cv_count[0] - 1) + 2 * (m_nurbs.m_cv_count[1] - 1);

  int rows = nInt + nCageReg + nCageRegBnd;
  int ncps = m_nurbs.CVCount ();

  m_solver.assign (rows, ncps, 1);

  unsigned row (0);

  // assemble data points
  const pcl::PointCloud<pcl::PointXYZRGB> &cloud_ref = *m_cloud;
  for (size_t i = 0; i < m_indices.size (); i++)
  {
    int px = m_indices[i] % cloud_ref.width;
    int py = m_indices[i] / cloud_ref.width;

    const pcl::PointXYZRGB &pt = cloud_ref.at (m_indices[i]);
    Eigen::Vector2i params (px, py);

    if (pcl_isnan (pt.z) || pt.z == 0.0)
      throw std::runtime_error ("[FittingSurfaceIM::assemble] Error, not a number (pt.z)");

    if (inverse_mapping)
    {
      Eigen::Vector3d point (pt.x, pt.y, pt.z);
      double error;
      Eigen::Vector3d p, tu, tv;
      Eigen::Vector2d params1 (params (0), params (1));
      params1 = inverseMapping (m_nurbs, point, params1, error, p, tu, tv, 200, 1e-6, true);
      params (0) = int (params1 (0));
      params (1) = int (params1 (1));
    }

    addPointConstraint (params, pt.z, 1.0, row);

    //    double point[3];
    //    m_nurbs.Evaluate (params (0), params (1), 0, 3, point);
    //    Eigen::Vector3d p (point[0], point[1], point[2]);
    //
    //    if (!(params (0) % 20) && !(params (1) % 20))
    //    {
    //      m_viewer->AddLine3D (p (0), p (1), p (2), pt.x, pt.y, pt.z, 0, 255, 0, 1.0);
    //    }
  }

  // cage regularisation
  double smooth (0.1);
  if (nCageReg > 0)
    addCageInteriorRegularisation (smooth, row);

  if (nCageRegBnd > 0)
  {
    addCageBoundaryRegularisation (smooth, NORTH, row);
    addCageBoundaryRegularisation (smooth, SOUTH, row);
    addCageBoundaryRegularisation (smooth, WEST, row);
    addCageBoundaryRegularisation (smooth, EAST, row);
    addCageCornerRegularisation (smooth * 2.0, row);
  }

  //  printf ("\n\n");
  //  m_solver.printK ();
  //  printf ("\n\n");
  //  m_solver.printF ();
  //  printf ("\n\n");
}

void
FittingSurfaceIM::addPointConstraint (const Eigen::Vector2i &params, double z, double weight, unsigned &row)
{
  double *N0 = new double[m_nurbs.Order (0) * m_nurbs.Order (0)];
  double *N1 = new double[m_nurbs.Order (1) * m_nurbs.Order (1)];

  int E = ON_NurbsSpanIndex (m_nurbs.m_order[0], m_nurbs.m_cv_count[0], m_nurbs.m_knot[0], params (0), 0, 0);
  int F = ON_NurbsSpanIndex (m_nurbs.m_order[1], m_nurbs.m_cv_count[1], m_nurbs.m_knot[1], params (1), 0, 0);

  ON_EvaluateNurbsBasis (m_nurbs.Order (0), m_nurbs.m_knot[0] + E, params (0), N0);
  ON_EvaluateNurbsBasis (m_nurbs.Order (1), m_nurbs.m_knot[1] + F, params (1), N1);

  m_solver.f (row, 0, z * weight);

  for (int i = 0; i < m_nurbs.Order (0); i++)
  {

    for (int j = 0; j < m_nurbs.Order (1); j++)
    {

      m_solver.K (row, lrc2gl (E, F, i, j), weight * N0[i] * N1[j]);

    } // j

  } // i

  row++;

  delete [] N1;
  delete [] N0;
}

void
FittingSurfaceIM::addCageInteriorRegularisation (double weight, unsigned &row)
{
  for (int i = 1; i < (m_nurbs.m_cv_count[0] - 1); i++)
  {
    for (int j = 1; j < (m_nurbs.m_cv_count[1] - 1); j++)
    {

      m_solver.f (row, 0, 0.0);

      m_solver.K (row, grc2gl (i + 0, j + 0), -4.0 * weight);
      m_solver.K (row, grc2gl (i + 0, j - 1), 1.0 * weight);
      m_solver.K (row, grc2gl (i + 0, j + 1), 1.0 * weight);
      m_solver.K (row, grc2gl (i - 1, j + 0), 1.0 * weight);
      m_solver.K (row, grc2gl (i + 1, j + 0), 1.0 * weight);

      row++;
    }
  }
}

void
FittingSurfaceIM::addCageBoundaryRegularisation (double weight, int side, unsigned &row)
{
  int i = 0;
  int j = 0;

  switch (side)
  {
    case SOUTH:
      j = m_nurbs.m_cv_count[1] - 1;
    case NORTH:
      for (i = 1; i < (m_nurbs.m_cv_count[0] - 1); i++)
      {

        m_solver.f (row, 0, 0.0);

        m_solver.K (row, grc2gl (i + 0, j), -2.0 * weight);
        m_solver.K (row, grc2gl (i - 1, j), 1.0 * weight);
        m_solver.K (row, grc2gl (i + 1, j), 1.0 * weight);

        row++;
      }
      break;

    case EAST:
      i = m_nurbs.m_cv_count[0] - 1;
    case WEST:
      for (j = 1; j < (m_nurbs.m_cv_count[1] - 1); j++)
      {

        m_solver.f (row, 0, 0.0);

        m_solver.K (row, grc2gl (i, j + 0), -2.0 * weight);
        m_solver.K (row, grc2gl (i, j - 1), 1.0 * weight);
        m_solver.K (row, grc2gl (i, j + 1), 1.0 * weight);

        row++;
      }
      break;
  }
}

void
FittingSurfaceIM::addCageCornerRegularisation (double weight, unsigned &row)
{
  { // NORTH-WEST
    int i = 0;
    int j = 0;

    m_solver.f (row, 0, 0.0);

    m_solver.K (row, grc2gl (i + 0, j + 0), -2.0 * weight);
    m_solver.K (row, grc2gl (i + 1, j + 0), 1.0 * weight);
    m_solver.K (row, grc2gl (i + 0, j + 1), 1.0 * weight);

    row++;
  }

  { // NORTH-EAST
    int i = m_nurbs.m_cv_count[0] - 1;
    int j = 0;

    m_solver.f (row, 0, 0.0);

    m_solver.K (row, grc2gl (i + 0, j + 0), -2.0 * weight);
    m_solver.K (row, grc2gl (i - 1, j + 0), 1.0 * weight);
    m_solver.K (row, grc2gl (i + 0, j + 1), 1.0 * weight);

    row++;
  }

  { // SOUTH-EAST
    int i = m_nurbs.m_cv_count[0] - 1;
    int j = m_nurbs.m_cv_count[1] - 1;

    m_solver.f (row, 0, 0.0);

    m_solver.K (row, grc2gl (i + 0, j + 0), -2.0 * weight);
    m_solver.K (row, grc2gl (i - 1, j + 0), 1.0 * weight);
    m_solver.K (row, grc2gl (i + 0, j - 1), 1.0 * weight);

    row++;
  }

  { // SOUTH-WEST
    int i = 0;
    int j = m_nurbs.m_cv_count[1] - 1;

    m_solver.f (row, 0, 0.0);

    m_solver.K (row, grc2gl (i + 0, j + 0), -2.0 * weight);
    m_solver.K (row, grc2gl (i + 1, j + 0), 1.0 * weight);
    m_solver.K (row, grc2gl (i + 0, j - 1), 1.0 * weight);

    row++;
  }

}

void
FittingSurfaceIM::solve (double damp)
{
  if (m_solver.solve ())
    updateSurf (damp);
}

void
FittingSurfaceIM::updateSurf (double damp)
{
  int ncp = m_nurbs.CVCount ();

  for (int A = 0; A < ncp; A++)
  {

    int I = gl2gr (A);
    int J = gl2gc (A);

    ON_3dPoint cp_prev;
    m_nurbs.GetCV (I, J, cp_prev);

    ON_3dPoint cp;
    cp.z = cp_prev.z + damp * (m_solver.x (A, 0) - cp_prev.z);
    cp.x = cp.z * (m_cps_px[A] (0) - m_intrinsic (0, 2)) / m_intrinsic (0, 0);
    cp.y = cp.z * (m_cps_px[A] (1) - m_intrinsic (1, 2)) / m_intrinsic (1, 1);

    m_nurbs.SetCV (I, J, cp);

  }

}

Eigen::Vector2d
FittingSurfaceIM::findClosestElementMidPoint (const ON_NurbsSurface &nurbs, const Eigen::Vector3d &pt)
{
  Eigen::Vector2d hint;
  Eigen::Vector3d r;
  std::vector<double> elementsU = getElementVector (nurbs, 0);
  std::vector<double> elementsV = getElementVector (nurbs, 1);

  double d_shortest (DBL_MAX);
  for (unsigned i = 0; i < elementsU.size () - 1; i++)
  {
    for (unsigned j = 0; j < elementsV.size () - 1; j++)
    {
      double points[3];
      double d;

      double xi = elementsU[i] + 0.5 * (elementsU[i + 1] - elementsU[i]);
      double eta = elementsV[j] + 0.5 * (elementsV[j + 1] - elementsV[j]);

      nurbs.Evaluate (xi, eta, 0, 3, points);
      r (0) = points[0] - pt (0);
      r (1) = points[1] - pt (1);
      r (2) = points[2] - pt (2);

      d = r.squaredNorm ();

      if ((i == 0 && j == 0) || d < d_shortest)
      {
        d_shortest = d;
        hint (0) = xi;
        hint (1) = eta;
      }
    }
  }

  return hint;
}

Eigen::Vector2d
FittingSurfaceIM::inverseMapping (const ON_NurbsSurface &nurbs, const Eigen::Vector3d &pt, const Eigen::Vector2d &hint,
                                  double &error, Eigen::Vector3d &p, Eigen::Vector3d &tu, Eigen::Vector3d &tv,
                                  int maxSteps, double accuracy, bool quiet)
{

  double pointAndTangents[9];

  Eigen::Vector2d current, delta;
  Eigen::Matrix2d A;
  Eigen::Vector2d b;
  Eigen::Vector3d r;
  std::vector<double> elementsU = getElementVector (nurbs, 0);
  std::vector<double> elementsV = getElementVector (nurbs, 1);
  double minU = elementsU[0];
  double minV = elementsV[0];
  double maxU = elementsU[elementsU.size () - 1];
  double maxV = elementsV[elementsV.size () - 1];

  current = hint;

  for (int k = 0; k < maxSteps; k++)
  {

    nurbs.Evaluate (current (0), current (1), 1, 3, pointAndTangents);
    p (0) = pointAndTangents[0];
    p (1) = pointAndTangents[1];
    p (2) = pointAndTangents[2];
    tu (0) = pointAndTangents[3];
    tu (1) = pointAndTangents[4];
    tu (2) = pointAndTangents[5];
    tv (0) = pointAndTangents[6];
    tv (1) = pointAndTangents[7];
    tv (2) = pointAndTangents[8];

    r = p - pt;

    b (0) = -r.dot (tu);
    b (1) = -r.dot (tv);

    A (0, 0) = tu.dot (tu);
    A (0, 1) = tu.dot (tv);
    A (1, 0) = A (0, 1);
    A (1, 1) = tv.dot (tv);

    delta = A.ldlt ().solve (b);

    if (delta.norm () < accuracy)
    {

      error = r.norm ();
      return current;

    }
    else
    {
      current = current + delta;

      if (current (0) < minU)
        current (0) = minU;
      else if (current (0) > maxU)
        current (0) = maxU;

      if (current (1) < minV)
        current (1) = minV;
      else if (current (1) > maxV)
        current (1) = maxV;

    }

  }

  error = r.norm ();

  if (!quiet)
  {
    printf ("[FittingSurface::inverseMapping] Warning: Method did not converge (%e %d)\n", accuracy, maxSteps);
    printf ("  %f %f ... %f %f\n", hint (0), hint (1), current (0), current (1));
  }

  return current;

}
