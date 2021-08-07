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

#include <stdexcept>
#include <pcl/surface/on_nurbs/fitting_cylinder_pdm.h>
#include <pcl/pcl_macros.h>
#include <Eigen/Cholesky> // for ldlt
#include <Eigen/Geometry> // for cross
#include <Eigen/LU> // for inverse

using namespace pcl;
using namespace on_nurbs;
using namespace Eigen;

FittingCylinder::FittingCylinder (int order, NurbsDataSurface *data)
{
  if (order < 2)
    throw std::runtime_error ("[FittingCylinder::FittingCylinder] Error order to low (order<2).");

  ON::Begin ();

  m_data = data;
  m_nurbs = initNurbsPCACylinder (order, m_data);

  this->init ();
}

FittingCylinder::FittingCylinder (NurbsDataSurface *data, const ON_NurbsSurface &ns)
{
  ON::Begin ();

  m_nurbs = ON_NurbsSurface (ns);
  m_data = data;

  this->init ();
}

void
FittingCylinder::init ()
{
  in_max_steps = 100;
  in_accuracy = 1e-4;

  m_quiet = true;
}

void
FittingCylinder::refine (int dim)
{
  std::vector<double> xi;
  std::vector<double> elements = getElementVector (m_nurbs, dim);

  for (std::size_t i = 0; i < elements.size () - 1; i++)
    xi.push_back (elements[i] + 0.5 * (elements[i + 1] - elements[i]));

  for (const double &i : xi)
    m_nurbs.InsertKnot (dim, i, 1);
}

void
FittingCylinder::refine (int dim, double param)
{
  std::vector<double> elements = getElementVector (m_nurbs, dim);

  if (param == elements[elements.size () - 1])
  {
    std::size_t i = elements.size () - 2;
    double xi = elements[i] + 0.5 * (elements[i + 1] - elements[i]);
    m_nurbs.InsertKnot (dim, xi);
    return;
  }

  for (std::size_t i = 0; i < elements.size () - 1; i++)
  {
    if (param >= elements[i] && param < elements[i + 1])
    {
      double xi = elements[i] + 0.5 * (elements[i + 1] - elements[i]);
      m_nurbs.InsertKnot (dim, xi);
    }
  }
}

void
FittingCylinder::refine (int dim, unsigned span_index)
{
  std::vector<double> elements = getElementVector (m_nurbs, dim);

  if (span_index + 2 > elements.size ())
  {
    printf ("[NurbsTools::refine(int, unsigned)] Warning span index out of bounds\n");
    return;
  }

  double xi = elements[span_index] + 0.5 * (elements[span_index + 1] - elements[span_index]);

  m_nurbs.InsertKnot (dim, xi);
}

void
FittingCylinder::assemble (double smoothness)
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
    printf ("[FittingCylinder::assemble] %dx%d (invmap: %f %d)\n", nrows, ncp, in_accuracy, in_max_steps);

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
FittingCylinder::solve (double damp)
{
  if (m_solver.solve ())
    updateSurf (damp);
}

void
FittingCylinder::updateSurf (double)
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
FittingCylinder::setInvMapParams (int in_max_steps, double in_accuracy)
{
  this->in_max_steps = in_max_steps;
  this->in_accuracy = in_accuracy;
}

ON_NurbsSurface
FittingCylinder::initNurbsPCACylinder (int order, NurbsDataSurface *data)
{
  Eigen::Vector3d mean;
  Eigen::Matrix3d eigenvectors;
  Eigen::Vector3d eigenvalues;

  unsigned s = unsigned (data->interior.size ());

  NurbsTools::pca (data->interior, mean, eigenvectors, eigenvalues);

  data->mean = mean;
  data->eigenvectors = eigenvectors;

  eigenvalues /= s; // seems that the eigenvalues are dependent on the number of points (???)

  Eigen::Vector3d v_max (0.0, 0.0, 0.0);
  Eigen::Vector3d v_min (DBL_MAX, DBL_MAX, DBL_MAX);
  for (unsigned i = 0; i < s; i++)
  {
    Eigen::Vector3d p (eigenvectors.inverse () * (data->interior[i] - mean));

    if (p (0) > v_max (0))
      v_max (0) = p (0);
    if (p (1) > v_max (1))
      v_max (1) = p (1);
    if (p (2) > v_max (2))
      v_max (2) = p (2);

    if (p (0) < v_min (0))
      v_min (0) = p (0);
    if (p (1) < v_min (1))
      v_min (1) = p (1);
    if (p (2) < v_min (2))
      v_min (2) = p (2);
  }

  int ncpsU (order);
  int ncpsV (2 * order + 4);
  ON_NurbsSurface nurbs (3, false, order, order, ncpsU, ncpsV);
  nurbs.MakeClampedUniformKnotVector (0, 1.0);
  nurbs.MakePeriodicUniformKnotVector (1, 1.0 / (ncpsV - order + 1));

  double dcu = (v_max (0) - v_min (0)) / (ncpsU - 1);
  double dcv = (2.0 * M_PI) / (ncpsV - order + 1);

  double ry = std::max<double> (std::fabs (v_min (1)), std::fabs (v_max (1)));
  double rz = std::max<double> (std::fabs (v_min (2)), std::fabs (v_max (2)));

  Eigen::Vector3d cv_t, cv;
  for (int i = 0; i < ncpsU; i++)
  {
    for (int j = 0; j < ncpsV; j++)
    {
      cv (0) = v_min (0) + dcu * i;
      cv (1) = ry * sin (dcv * j);
      cv (2) = rz * std::cos (dcv * j);
      cv_t = eigenvectors * cv + mean;
      nurbs.SetCV (i, j, ON_3dPoint (cv_t (0), cv_t (1), cv_t (2)));
    }
  }

  return nurbs;
}

ON_NurbsSurface
FittingCylinder::initNurbsCylinderWithAxes (int order, NurbsDataSurface *data, Eigen::Matrix3d &axes)
{
  Eigen::Vector3d mean;

  unsigned s = unsigned (data->interior.size ());
  mean = NurbsTools::computeMean (data->interior);

  data->mean = mean;

  Eigen::Vector3d v_max (0.0, 0.0, 0.0);
  Eigen::Vector3d v_min (DBL_MAX, DBL_MAX, DBL_MAX);
  for (unsigned i = 0; i < s; i++)
  {
    Eigen::Vector3d p (axes.inverse () * (data->interior[i] - mean));

    if (p (0) > v_max (0))
      v_max (0) = p (0);
    if (p (1) > v_max (1))
      v_max (1) = p (1);
    if (p (2) > v_max (2))
      v_max (2) = p (2);

    if (p (0) < v_min (0))
      v_min (0) = p (0);
    if (p (1) < v_min (1))
      v_min (1) = p (1);
    if (p (2) < v_min (2))
      v_min (2) = p (2);
  }

  int ncpsU (order);
  int ncpsV (2 * order + 4);
  ON_NurbsSurface nurbs (3, false, order, order, ncpsU, ncpsV);
  nurbs.MakeClampedUniformKnotVector (0, 1.0);
  nurbs.MakePeriodicUniformKnotVector (1, 1.0 / (ncpsV - order + 1));

  double dcu = (v_max (0) - v_min (0)) / (ncpsU - 1);
  double dcv = (2.0 * M_PI) / (ncpsV - order + 1);

  double ry = std::max<double> (std::fabs (v_min (1)), std::fabs (v_max (1)));
  double rz = std::max<double> (std::fabs (v_min (2)), std::fabs (v_max (2)));

  Eigen::Vector3d cv_t, cv;
  for (int i = 0; i < ncpsU; i++)
  {
    for (int j = 0; j < ncpsV; j++)
    {
      cv (0) = v_min (0) + dcu * i;
      cv (1) = ry * sin (dcv * j);
      cv (2) = rz * std::cos (dcv * j);
      cv_t = axes * cv + mean;
      nurbs.SetCV (i, j, ON_3dPoint (cv_t (0), cv_t (1), cv_t (2)));
    }
  }

  return nurbs;
}

std::vector<double>
FittingCylinder::getElementVector (const ON_NurbsSurface &nurbs, int dim) // !
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
    printf ("[FittingCylinder::getElementVector] Error, index exceeds problem dimensions!\n");

  return result;
}

void
FittingCylinder::assembleInterior (double wInt, unsigned &row)
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
FittingCylinder::addPointConstraint (const Eigen::Vector2d &params, const Eigen::Vector3d &point, double weight,
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

  delete [] N1;
  delete [] N0;
}

void
FittingCylinder::addCageInteriorRegularisation (double weight, unsigned &row)
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
FittingCylinder::addCageBoundaryRegularisation (double weight, int side, unsigned &row)
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
FittingCylinder::inverseMapping (const ON_NurbsSurface &nurbs, const Vector3d &pt, const Vector2d &hint, double &error,
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
    printf ("[FittingCylinder::inverseMapping] Warning: Method did not converge (%e %d)\n", accuracy, maxSteps);
    printf ("  %f %f ... %f %f\n", hint (0), hint (1), current (0), current (1));
  }

  return current;
}

Vector2d
FittingCylinder::findClosestElementMidPoint (const ON_NurbsSurface &nurbs, const Vector3d &pt)
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

