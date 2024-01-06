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

#include <limits>
#include <stdexcept>
#include <pcl/surface/on_nurbs/fitting_sphere_pdm.h>
#include <pcl/pcl_macros.h>
#include <Eigen/Cholesky> // for ldlt
#include <Eigen/Geometry> // for cross

using namespace pcl;
using namespace on_nurbs;
using namespace Eigen;

FittingSphere::FittingSphere (int order, NurbsDataSurface *data)
{
  if (order < 2)
    throw std::runtime_error ("[FittingSphere::FittingSphere] Error order to low (order<2).");

  ON::Begin ();

  m_data = data;
  m_nurbs = initNurbsSphere (order, m_data);

  this->init ();
}

FittingSphere::FittingSphere (NurbsDataSurface *data, const ON_NurbsSurface &ns)
{
  ON::Begin ();

  m_nurbs = ON_NurbsSurface (ns);
  m_data = data;

  this->init ();
}

void
FittingSphere::init ()
{
  in_max_steps = 100;
  in_accuracy = 1e-4;

  m_quiet = true;
}

void
FittingSphere::assemble (double smoothness)
{
  int cp_red = (m_nurbs.m_order[1] - 2);
  int ncp = m_nurbs.m_cv_count[0] * (m_nurbs.m_cv_count[1] - 2 * cp_red);
  int nInt = int (m_data->interior.size ());
  int nCageRegInt = (m_nurbs.m_cv_count[0] - 2) * (m_nurbs.m_cv_count[1] - 2 * cp_red);
  int nCageRegBnd = 2 * (m_nurbs.m_cv_count[1] - 2 * cp_red);

  double wInt = 1.0;
  double wCageRegInt = smoothness;
  double wCageRegBnd = smoothness;

  int nrows = nInt + nCageRegInt + nCageRegBnd;

  if (!m_quiet)
    printf ("[FittingSphere::assemble] %dx%d (invmap: %f %d)\n", nrows, ncp, in_accuracy, in_max_steps);

  m_solver.assign (nrows, ncp, 3);

  unsigned row (0);

  // interior points should lie on surface
  assembleInterior (wInt, row);

  // cage regularisation
  if (nCageRegInt > 0)
    addCageInteriorRegularisation (wCageRegInt, row);

  if (nCageRegBnd > 0)
  {
    addCageBoundaryRegularisation (wCageRegBnd, WEST, row);
    addCageBoundaryRegularisation (wCageRegBnd, EAST, row);
  }
}

void
FittingSphere::solve (double damp)
{
  if (m_solver.solve ())
    updateSurf (damp);
}

void
FittingSphere::updateSurf (double)
{
  int cp_red = (m_nurbs.m_order[1] - 2);

  for (int j = 0; j < m_nurbs.m_cv_count[1] - 2 * cp_red; j++)
  {
    for (int i = 0; i < m_nurbs.m_cv_count[0]; i++)
    {

      int A = grc2gl (i, j);

      ON_3dPoint cp;
      cp.x = m_solver.x (A, 0);
      cp.y = m_solver.x (A, 1);
      cp.z = m_solver.x (A, 2);

      m_nurbs.SetCV (i, j + cp_red, cp);
    }
  }

  for (int j = 0; j < cp_red; j++)
  {
    for (int i = 0; i < m_nurbs.m_cv_count[0]; i++)
    {

      ON_3dPoint cp;
      m_nurbs.GetCV (i, m_nurbs.m_cv_count[1] - 1 - cp_red + j, cp);
      m_nurbs.SetCV (i, j, cp);

      m_nurbs.GetCV (i, cp_red - j, cp);
      m_nurbs.SetCV (i, m_nurbs.m_cv_count[1] - 1 - j, cp);
    }
  }
}

void
FittingSphere::setInvMapParams (int in_max_steps, double in_accuracy)
{
  this->in_max_steps = in_max_steps;
  this->in_accuracy = in_accuracy;
}

ON_NurbsSurface
FittingSphere::initNurbsSphere (int order, NurbsDataSurface *data, Eigen::Vector3d)
{
  Eigen::Vector3d mean = NurbsTools::computeMean (data->interior);

  Eigen::Vector3d _min(std::numeric_limits<double>::max(),
                       std::numeric_limits<double>::max(),
                       std::numeric_limits<double>::max());
  Eigen::Vector3d _max(std::numeric_limits<double>::lowest(),
                       std::numeric_limits<double>::lowest(),
                       std::numeric_limits<double>::lowest());
  for (const auto &i : data->interior)
  {
    Eigen::Vector3d p = i - mean;

    if (p (0) < _min (0))
      _min (0) = p (0);
    if (p (1) < _min (1))
      _min (1) = p (1);
    if (p (2) < _min (2))
      _min (2) = p (2);

    if (p (0) > _max (0))
      _max (0) = p (0);
    if (p (1) > _max (1))
      _max (1) = p (1);
    if (p (2) > _max (2))
      _max (2) = p (2);
  }

  int ncpsU (order + 2);
  int ncpsV (2 * order + 4);
  ON_NurbsSurface nurbs (3, false, order, order, ncpsU, ncpsV);
  nurbs.MakeClampedUniformKnotVector (0, 1.0);
  nurbs.MakePeriodicUniformKnotVector (1, 1.0 / (ncpsV - order + 1));

  double dcu = (_max (2) - _min (2)) / (ncpsU - 3);
  double dcv = (2.0 * M_PI) / (ncpsV - order + 1);

  double rx = std::max<double> (std::fabs (_min (0)), std::fabs (_max (0)));
  double ry = std::max<double> (std::fabs (_min (1)), std::fabs (_max (1)));

  Eigen::Vector3d cv_t, cv;
  for (int i = 1; i < ncpsU - 1; i++)
  //  for (int i = 0; i < ncpsU; i++)
  {
    for (int j = 0; j < ncpsV; j++)
    {

      cv (0) = rx * sin (dcv * j);
      cv (1) = ry * std::cos (dcv * j);
      cv (2) = _min (2) + dcu * (i - 1);
      cv_t = cv + mean;
      nurbs.SetCV (i, j, ON_3dPoint (cv_t (0), cv_t (1), cv_t (2)));
    }
  }

  for (int j = 0; j < ncpsV; j++)
  {
    //    cv (0) = 0.0;
    //    cv (1) = 0.0;
    cv (0) = 0.01 * rx * sin (dcv * j);
    cv (1) = 0.01 * ry * std::cos (dcv * j);
    cv (2) = _min (2);
    cv_t = cv + mean;
    nurbs.SetCV (0, j, ON_3dPoint (cv_t (0), cv_t (1), cv_t (2)));
  }

  for (int j = 0; j < ncpsV; j++)
  {
    //    cv (0) = 0.0;
    //    cv (1) = 0.0;
    cv (0) = 0.01 * rx * sin (dcv * j);
    cv (1) = 0.01 * ry * std::cos (dcv * j);
    cv (2) = _max (2);
    cv_t = cv + mean;
    nurbs.SetCV (ncpsU - 1, j, ON_3dPoint (cv_t (0), cv_t (1), cv_t (2)));
  }

  return nurbs;
}

std::vector<double>
FittingSphere::getElementVector (const ON_NurbsSurface &nurbs, int dim) // !
{
  std::vector<double> result;

  if (dim == 0)
  {
    int idx_min = 0;
    int idx_max = nurbs.KnotCount (0) - 1;
    if (nurbs.IsClosed (0))
    {
      idx_min = nurbs.Order (0) - 2;
      idx_max = nurbs.KnotCount (0) - nurbs.Order (0) + 1;
    }

    const double* knotsU = nurbs.Knot (0);

    result.push_back (knotsU[idx_min]);

    //for(int E=(m_nurbs.m_order[0]-2); E<(m_nurbs.m_knot_capacity[0]-m_nurbs.m_order[0]+2); E++) {
    for (int E = idx_min + 1; E <= idx_max; E++)
    {

      if (knotsU[E] != knotsU[E - 1]) // do not count double knots
        result.push_back (knotsU[E]);

    }

  }
  else if (dim == 1)
  {
    int idx_min = 0;
    int idx_max = nurbs.KnotCount (1) - 1;
    if (nurbs.IsClosed (1))
    {
      idx_min = nurbs.Order (1) - 2;
      idx_max = nurbs.KnotCount (1) - nurbs.Order (1) + 1;
    }
    const double* knotsV = nurbs.Knot (1);

    result.push_back (knotsV[idx_min]);

    //for(int F=(m_nurbs.m_order[1]-2); F<(m_nurbs.m_knot_capacity[1]-m_nurbs.m_order[1]+2); F++) {
    for (int F = idx_min + 1; F <= idx_max; F++)
    {

      if (knotsV[F] != knotsV[F - 1])
        result.push_back (knotsV[F]);

    }

  }
  else
    printf ("[FittingSphere::getElementVector] Error, index exceeds problem dimensions!\n");

  return result;
}

void
FittingSphere::assembleInterior (double wInt, unsigned &row)
{
  m_data->interior_line_start.clear ();
  m_data->interior_line_end.clear ();
  m_data->interior_error.clear ();
  m_data->interior_normals.clear ();
  unsigned nInt = unsigned (m_data->interior.size ());
  for (unsigned p = 0; p < nInt; p++)
  {
    Vector3d pcp;
    pcp = m_data->interior[p];

    // inverse mapping
    Vector2d params;
    Vector3d pt, tu, tv, n;
    double error;
    if (p < m_data->interior_param.size ())
    {
      params = inverseMapping (m_nurbs, pcp, m_data->interior_param[p], error, pt, tu, tv, in_max_steps, in_accuracy);
      m_data->interior_param[p] = params;
    }
    else
    {
      params = findClosestElementMidPoint (m_nurbs, pcp);
      params = inverseMapping (m_nurbs, pcp, params, error, pt, tu, tv, in_max_steps, in_accuracy);
      m_data->interior_param.push_back (params);
    }
    m_data->interior_error.push_back (error);

    n = tu.cross (tv);
    n.normalize ();

    m_data->interior_normals.push_back (n);
    m_data->interior_line_start.push_back (pcp);
    m_data->interior_line_end.push_back (pt);

    addPointConstraint (params, pcp, wInt, row);
  }
}

void
FittingSphere::addPointConstraint (const Eigen::Vector2d &params, const Eigen::Vector3d &point, double weight,
                                   unsigned &row)
{
  double *N0 = new double[m_nurbs.m_order[0] * m_nurbs.m_order[0]];
  double *N1 = new double[m_nurbs.m_order[1] * m_nurbs.m_order[1]];

  int E = ON_NurbsSpanIndex (m_nurbs.m_order[0], m_nurbs.m_cv_count[0], m_nurbs.m_knot[0], params (0), 0, 0);
  int F = ON_NurbsSpanIndex (m_nurbs.m_order[1], m_nurbs.m_cv_count[1], m_nurbs.m_knot[1], params (1), 0, 0);

  ON_EvaluateNurbsBasis (m_nurbs.m_order[0], m_nurbs.m_knot[0] + E, params (0), N0);
  ON_EvaluateNurbsBasis (m_nurbs.m_order[1], m_nurbs.m_knot[1] + F, params (1), N1);

  m_solver.f (row, 0, point (0) * weight);
  m_solver.f (row, 1, point (1) * weight);
  m_solver.f (row, 2, point (2) * weight);

  for (int i = 0; i < m_nurbs.m_order[0]; i++)
  {

    for (int j = 0; j < m_nurbs.m_order[1]; j++)
    {

      m_solver.K (row, lrc2gl (E, F, i, j), weight * N0[i] * N1[j]);

    } // j

  } // i

  row++;

  delete[] N1;
  delete[] N0;
}

void
FittingSphere::addCageInteriorRegularisation (double weight, unsigned &row)
{
  int cp_red = (m_nurbs.m_order[1] - 2);

  for (int i = 1; i < (m_nurbs.m_cv_count[0] - 1); i++)
  {
    for (int j = 0; j < (m_nurbs.m_cv_count[1] - 2 * cp_red); j++)
    {

      m_solver.f (row, 0, 0.0);
      m_solver.f (row, 1, 0.0);
      m_solver.f (row, 2, 0.0);

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
FittingSphere::addCageBoundaryRegularisation (double weight, int side, unsigned &row)
{
  int cp_red = (m_nurbs.m_order[1] - 2);
  int i = 0;
  int j = 0;

  switch (side)
  {
    case EAST:
      i = m_nurbs.m_cv_count[0] - 1;
      PCL_FALLTHROUGH
    case WEST:
      for (j = 1; j < (m_nurbs.m_cv_count[1] - 2 * cp_red) + 1; j++)
      {

        m_solver.f (row, 0, 0.0);
        m_solver.f (row, 1, 0.0);
        m_solver.f (row, 2, 0.0);

        m_solver.K (row, grc2gl (i, j + 0), -2.0 * weight);
        m_solver.K (row, grc2gl (i, j - 1), 1.0 * weight);
        m_solver.K (row, grc2gl (i, j + 1), 1.0 * weight);

        row++;
      }
      break;
  }
}

Vector2d
FittingSphere::inverseMapping (const ON_NurbsSurface &nurbs, const Vector3d &pt, const Vector2d &hint, double &error,
                               Vector3d &p, Vector3d &tu, Vector3d &tv, int maxSteps, double accuracy, bool quiet)
{
  double pointAndTangents[9];

  Vector2d current, delta;
  Matrix2d A;
  Vector2d b;
  Vector3d r;
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
    current += delta;
    if (current (0) < minU)
      current (0) = minU;
    else if (current (0) > maxU)
      current (0) = maxU;

    if (current (1) < minV)
      current (1) = maxV - (minV - current (1));
    else if (current (1) > maxV)
      current (1) = minV + (current (1) - maxV);
  }

  error = r.norm ();

  if (!quiet)
  {
    printf ("[FittingSphere::inverseMapping] Warning: Method did not converge (%e %d)\n", accuracy, maxSteps);
    printf ("  %f %f ... %f %f\n", hint (0), hint (1), current (0), current (1));
  }

  return current;
}

Vector2d
FittingSphere::findClosestElementMidPoint (const ON_NurbsSurface &nurbs, const Vector3d &pt)
{
  Vector2d hint;
  Vector3d r;
  std::vector<double> elementsU = getElementVector (nurbs, 0);
  std::vector<double> elementsV = getElementVector (nurbs, 1);

  double d_shortest = std::numeric_limits<double>::max ();
  for (std::size_t i = 0; i < elementsU.size () - 1; i++)
  {
    for (std::size_t j = 0; j < elementsV.size () - 1; j++)
    {
      double points[3];

      double xi = elementsU[i] + 0.5 * (elementsU[i + 1] - elementsU[i]);
      double eta = elementsV[j] + 0.5 * (elementsV[j + 1] - elementsV[j]);

      nurbs.Evaluate (xi, eta, 0, 3, points);
      r (0) = points[0] - pt (0);
      r (1) = points[1] - pt (1);
      r (2) = points[2] - pt (2);

      double d = r.squaredNorm ();

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

