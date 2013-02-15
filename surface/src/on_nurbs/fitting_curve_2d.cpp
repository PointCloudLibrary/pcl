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

#include <pcl/surface/on_nurbs/fitting_curve_2d.h>
#include <stdexcept>

using namespace pcl;
using namespace on_nurbs;

FittingCurve2d::FittingCurve2d (int order, NurbsDataCurve2d *data)
{
  if (order < 2)
    throw std::runtime_error ("[NurbsFittingCylinder::NurbsFittingCylinder] Error order to low (order<2).");

  ON::Begin ();

  m_data = data;
  m_nurbs = initNurbsPCA (order, m_data);

  in_max_steps = 200;
  in_accuracy = 1e-6;
  m_quiet = true;
}

FittingCurve2d::FittingCurve2d (NurbsDataCurve2d *data, const ON_NurbsCurve &nc)
{
  ON::Begin ();

  m_nurbs = ON_NurbsCurve (nc);
  m_data = data;

  in_max_steps = 200;
  in_accuracy = 1e-6;
  m_quiet = true;
}

int
FittingCurve2d::findElement (double xi, const std::vector<double> &elements)
{
  if (xi >= elements.back ())
    return (int (elements.size ()) - 2);

  for (unsigned i = 0; i < elements.size () - 1; i++)
  {
    if (xi >= elements[i] && xi < elements[i + 1])
    {
      return i;
    }
  }

  //  xi < elements.front()
  return 0;

}

void
FittingCurve2d::refine ()
{
  std::vector<double> xi;

  std::vector<double> elements = this->getElementVector (m_nurbs);

  for (unsigned i = 0; i < elements.size () - 1; i++)
    xi.push_back (elements[i] + 0.5 * (elements[i + 1] - elements[i]));

  for (unsigned i = 0; i < xi.size (); i++)
    m_nurbs.InsertKnot (xi[i], 1);
}

void
FittingCurve2d::refine (double xi)
{
  std::vector<double> elements = getElementVector (m_nurbs);

  int i = findElement (xi, elements);

  double _xi = elements[i] + 0.5 * (elements[i + 1] - elements[i]);
  m_nurbs.InsertKnot (_xi, 1);
  return;
}

void
FittingCurve2d::assemble (const Parameter &parameter)
{
  int ncp = m_nurbs.m_cv_count;
  int nCageReg = m_nurbs.m_cv_count - 2;
  int nInt = int (m_data->interior.size ());

  double wInt = 1.0;
  if (!m_data->interior_weight.empty ())
    wInt = m_data->interior_weight[0];

  unsigned nrows = nInt + nCageReg;

  m_solver.assign (nrows, ncp, 2);

  unsigned row (0);

  if (wInt > 0.0)
    assembleInterior (wInt, parameter.rScale, row);

  if (parameter.smoothness > 0.0)
    addCageRegularisation (parameter.smoothness, row);

  if (row < nrows)
  {
    m_solver.resize (row);
    if (!m_quiet)
      printf ("[FittingCurve2d::assemble] Warning: rows do not match: %d %d\n", row, nrows);
  }
}
void
FittingCurve2d::addControlPointConstraint (int i, const Eigen::Vector2d &f, double weight)
{
  if (i < 0 || i >= m_nurbs.CVCount ())
  {
    printf ("[FittingCurve2d::addControlPointConstraint] Warning, index out of bounds.\n");
    return;
  }

  // resize solver
  unsigned row, cols, dims;
  m_solver.getSize (row, cols, dims);
  m_solver.resize (row + 1);

  // add constraint for control point
  m_solver.f (row, 0, f (0) * weight);
  m_solver.f (row, 1, f (1) * weight);
  for (int j = 0; j < cols; j++)
    m_solver.K (row, j, 0.0);
  m_solver.K (row, i, weight);
}

double
FittingCurve2d::solve (double damp)
{
  double cps_diff (0.0);

  if (m_solver.solve ())
    cps_diff = updateCurve (damp);

  return cps_diff;
}

double
FittingCurve2d::updateCurve (double damp)
{
  int ncp = m_nurbs.CVCount ();

  double cps_diff (0.0);

  for (int j = 0; j < ncp; j++)
  {
    ON_3dPoint cp_prev;
    m_nurbs.GetCV (j, cp_prev);

    double x = m_solver.x (j, 0);
    double y = m_solver.x (j, 1);

    cps_diff += sqrt ((x - cp_prev.x) * (x - cp_prev.x) + (y - cp_prev.y) * (y - cp_prev.y));

    ON_3dPoint cp;
    cp.x = cp_prev.x + damp * (x - cp_prev.x);
    cp.y = cp_prev.y + damp * (y - cp_prev.y);
    cp.z = 0.0;

    m_nurbs.SetCV (j, cp);
  }

  return cps_diff / ncp;
}

void
FittingCurve2d::addPointConstraint (const double &param, const Eigen::Vector2d &point, double weight, unsigned &row)
{
  double *N = new double[m_nurbs.m_order * m_nurbs.m_order];

  int E = ON_NurbsSpanIndex (m_nurbs.m_order, m_nurbs.m_cv_count, m_nurbs.m_knot, param, 0, 0);

  ON_EvaluateNurbsBasis (m_nurbs.m_order, m_nurbs.m_knot + E, param, N);

  m_solver.f (row, 0, point (0) * weight);
  m_solver.f (row, 1, point (1) * weight);

  for (int i = 0; i < m_nurbs.m_order; i++)
    m_solver.K (row, (E + i), weight * N[i]);

  row++;

  delete[] N;
}

void
FittingCurve2d::addCageRegularisation (double weight, unsigned &row)
{
  int ncp = m_nurbs.m_cv_count;

  for (int j = 1; j < ncp - 1; j++)
  {
    m_solver.f (row, 0, 0.0);
    m_solver.f (row, 1, 0.0);

    m_solver.K (row, (j + 0), -2.0 * weight);
    m_solver.K (row, (j - 1), 1.0 * weight);
    m_solver.K (row, (j + 1), 1.0 * weight);

    row++;
  }
}

ON_NurbsCurve
FittingCurve2d::initNurbsCPS (int order, const vector_vec2d &cps)
{
  ON_NurbsCurve nurbs;
  if (cps.size () < order)
  {
    printf ("[FittingCurve2d::initCPsNurbsCurve2D] Warning, number of control points too low.\n");
    return nurbs;
  }

  printf ("[FittingCurve2d::initCPsNurbsCurve2D] Warning, this function is under development.\n");
  //
  //  size_t ncps = cps.size ();
  //  nurbs = ON_NurbsCurve (2, false, order, ncps);
  //  nurbs.MakePeriodicUniformKnotVector (1.0 / (ncps - order + 1));
  //
  //  for (int j = 0; j < cps.size (); j++)
  //    nurbs.SetCV (cp_red + j, ON_3dPoint (cps[j] (0), cps[j] (1), 0.0));
  //
  //  // close nurbs
  //  nurbs.SetCV (cp_red + int (cps.size ()), ON_3dPoint (cps[0] (0), cps[0] (1), 0.0));
  //
  //  // make smooth at closing point
  //  for (int j = 0; j < cp_red; j++)
  //  {
  //    ON_3dPoint cp;
  //    nurbs.GetCV (nurbs.CVCount () - 1 - cp_red + j, cp);
  //    nurbs.SetCV (j, cp);
  //
  //    nurbs.GetCV (cp_red - j, cp);
  //    nurbs.SetCV (nurbs.CVCount () - 1 - j, cp);
  //  }

  return nurbs;
}

ON_NurbsCurve
FittingCurve2d::initNurbsPCA (int order, NurbsDataCurve2d *data, int ncps)
{
  Eigen::Vector2d mean;
  Eigen::Matrix2d eigenvectors;
  Eigen::Vector2d eigenvalues;

  if (ncps < order)
    ncps = order;

  unsigned s = static_cast<unsigned> (data->interior.size ());
  data->interior_param.clear ();

  NurbsTools::pca (data->interior, mean, eigenvectors, eigenvalues);

  data->mean = mean;
  data->eigenvectors = eigenvectors;

  eigenvalues = eigenvalues / s; // seems that the eigenvalues are dependent on the number of points (???)
  Eigen::Matrix2d eigenvectors_inv = eigenvectors.inverse ();

  Eigen::Vector2d v_max (-DBL_MAX, -DBL_MAX);
  Eigen::Vector2d v_min (DBL_MAX, DBL_MAX);
  for (unsigned i = 0; i < s; i++)
  {
    Eigen::Vector2d p (eigenvectors_inv * (data->interior[i] - mean));
    data->interior_param.push_back (p (0));

    if (p (0) > v_max (0))
      v_max (0) = p (0);
    if (p (1) > v_max (1))
      v_max (1) = p (1);

    if (p (0) < v_min (0))
      v_min (0) = p (0);
    if (p (1) < v_min (1))
      v_min (1) = p (1);
  }

  for (unsigned i = 0; i < s; i++)
  {
    double &p = data->interior_param[i];
    if (v_max (0) > v_min (0))
    {
      p = (p - v_min (0)) / (v_max (0) - v_min (0));
    }
    else
    {
      throw std::runtime_error ("[FittingCurve2d::initNurbsPCABoundingBox] Error: v_max <= v_min");
    }
  }

  ON_NurbsCurve nurbs (2, false, order, ncps);
  double delta = 1.0 / (nurbs.KnotCount () - 3);
  nurbs.MakeClampedUniformKnotVector (delta);

  double dcu = (v_max (0) - v_min (0)) / (ncps - 1);

  Eigen::Vector2d cv_t, cv;
  for (int i = 0; i < ncps; i++)
  {
    cv (0) = v_min (0) + dcu * i;
    cv (1) = 0.0;
    cv_t = eigenvectors * cv + mean;
    nurbs.SetCV (i, ON_3dPoint (cv_t (0), cv_t (1), 0.0));
  }

  return nurbs;
}

void
FittingCurve2d::reverse (ON_NurbsCurve &curve)
{

  ON_NurbsCurve curve2 = curve;
  for (int i = 0; i < curve.CVCount (); i++)
  {
    int j = curve.CVCount () - 1 - i;
    ON_3dPoint p;
    curve.GetCV (i, p);
    curve2.SetCV (j, p);
  }
  curve = curve2;
}

std::vector<double>
FittingCurve2d::getElementVector (const ON_NurbsCurve &nurbs)
{
  std::vector<double> result;

  int idx_min = 0;
  int idx_max = nurbs.m_knot_capacity - 1;
  if (nurbs.IsClosed ())
  {
    idx_min = nurbs.m_order - 2;
    idx_max = nurbs.m_knot_capacity - nurbs.m_order + 1;
  }

  const double* knotsU = nurbs.Knot ();

  result.push_back (knotsU[idx_min]);

  //for(int E=(m_nurbs.m_order[0]-2); E<(m_nurbs.m_knot_capacity[0]-m_nurbs.m_order[0]+2); E++) {
  for (int E = idx_min + 1; E <= idx_max; E++)
  {

    if (knotsU[E] != knotsU[E - 1]) // do not count double knots
      result.push_back (knotsU[E]);

  }

  return result;
}

void
FittingCurve2d::assembleInterior (double wInt, double rScale, unsigned &row)
{
  int nInt = int (m_data->interior.size ());
  m_data->interior_error.clear ();
  m_data->interior_normals.clear ();
  m_data->interior_line_start.clear ();
  m_data->interior_line_end.clear ();

  for (int p = 0; p < nInt; p++)
  {
    Eigen::Vector2d &pcp = m_data->interior[p];

    // inverse mapping
    double param;
    Eigen::Vector2d pt, t;
    double error;
    if (p < int (m_data->interior_param.size ()))
    {
      param = findClosestElementMidPoint (m_nurbs, pcp, m_data->interior_param[p]);
      param = inverseMapping (m_nurbs, pcp, param, error, pt, t, rScale, in_max_steps, in_accuracy, m_quiet);
      m_data->interior_param[p] = param;
    }
    else
    {
      param = findClosestElementMidPoint (m_nurbs, pcp);
      param = inverseMapping (m_nurbs, pcp, param, error, pt, t, rScale, in_max_steps, in_accuracy, m_quiet);
      m_data->interior_param.push_back (param);
    }

    m_data->interior_error.push_back (error);

    if (p < int (m_data->interior_weight.size ()))
      wInt = m_data->interior_weight[p];

    m_data->interior_line_start.push_back (pcp);
    m_data->interior_line_end.push_back (pt);

    addPointConstraint (m_data->interior_param[p], m_data->interior[p], wInt, row);
  }
}

double
FittingCurve2d::inverseMapping (const ON_NurbsCurve &nurbs, const Eigen::Vector2d &pt, const double &hint,
                                double &error, Eigen::Vector2d &p, Eigen::Vector2d &t, double rScale, int maxSteps,
                                double accuracy, bool quiet)
{
  if (nurbs.Order () == 2)
    return inverseMappingO2 (nurbs, pt, error, p, t);

  double pointAndTangents[4];

  double current, delta;
  Eigen::Vector2d r;
  std::vector<double> elements = getElementVector (nurbs);
  double minU = elements[0];
  double maxU = elements[elements.size () - 1];

  current = hint;

  for (int k = 0; k < maxSteps; k++)
  {

    nurbs.Evaluate (current, 1, 2, pointAndTangents);

    p (0) = pointAndTangents[0];
    p (1) = pointAndTangents[1];

    t (0) = pointAndTangents[2];
    t (1) = pointAndTangents[3];

    r = p - pt;

    // step width control
    int E = findElement (current, elements);
    double e = elements[E + 1] - elements[E];

    delta = -(0.5 * e * rScale) * r.dot (t) / t.norm (); //  A.ldlt().solve(b);

    if (std::abs (delta) < accuracy)
    {

      error = r.norm ();
      return current;

    }
    else
    {
      current = current + delta;

      if (current < minU)
        current = minU;
      if (current > maxU)
        current = maxU;
    }

  }

  error = r.norm ();

  if (!quiet)
  {
    printf ("[FittingCurve2d::inverseMapping] Warning: Method did not converge (%e %d).\n", accuracy, maxSteps);
    printf ("[FittingCurve2d::inverseMapping] hint: %f current: %f delta: %f error: %f\n", hint, current, delta, error);
  }

  return current;
}

double
FittingCurve2d::inverseMappingO2 (const ON_NurbsCurve &nurbs, const Eigen::Vector2d &pt, double &error,
                                  Eigen::Vector2d &p, Eigen::Vector2d &t)
{
  if (nurbs.Order () != 2)
    printf ("[FittingCurve2d::inverseMappingO2] Error, order not 2 (polynomial degree 1)\n");

  std::vector<double> elements = getElementVector (nurbs);

  Eigen::Vector2d min_pt;
  double min_param (DBL_MAX);
  double min_dist (DBL_MAX);
  error = DBL_MAX;
  int is_corner (-1);

  for (unsigned i = 0; i < elements.size () - 1; i++)
  {
    Eigen::Vector2d p1;
    nurbs.Evaluate (elements[i], 0, 2, &p1 (0));

    Eigen::Vector2d p2;
    nurbs.Evaluate (elements[i + 1], 0, 2, &p2 (0));

    Eigen::Vector2d d1 (p2 (0) - p1 (0), p2 (1) - p1 (1));
    Eigen::Vector2d d2 (pt (0) - p1 (0), pt (1) - p1 (1));

    double d1_norm = d1.norm ();

    double d0_norm = d1.dot (d2) / d1_norm;
    Eigen::Vector2d d0 = d1 * d0_norm / d1_norm;
    Eigen::Vector2d p0 = p1 + d0;

    if (d0_norm < 0.0)
    {
      double tmp_dist = (p1 - pt).norm ();
      if (tmp_dist < min_dist)
      {
        min_dist = tmp_dist;
        min_pt = p1;
        min_param = elements[i];
        is_corner = i;
      }
    }
    else if (d0_norm >= d1_norm)
    {
      double tmp_dist = (p2 - pt).norm ();
      if (tmp_dist < min_dist)
      {
        min_dist = tmp_dist;
        min_pt = p2;
        min_param = elements[i + 1];
        is_corner = i + 1;
      }
    }
    else
    { // p0 lies on line segment
      double tmp_dist = (p0 - pt).norm ();
      if (tmp_dist < min_dist)
      {
        min_dist = tmp_dist;
        min_pt = p0;
        min_param = elements[i] + (d0_norm / d1_norm) * (elements[i + 1] - elements[i]);
        is_corner = -1;
      }
    }
  }

  if (is_corner >= 0)
  {
    double param1, param2;
    if (is_corner == 0 || is_corner == elements.size () - 1)
    {
      double x0a = elements[0];
      double x0b = elements[elements.size () - 1];
      double xa = elements[1];
      double xb = elements[elements.size () - 2];

      param1 = x0a + 0.5 * (xa - x0a);
      param2 = x0b + 0.5 * (xb - x0b);
    }
    else
    {
      double x0 = elements[is_corner];
      double x1 = elements[is_corner - 1];
      double x2 = elements[is_corner + 1];

      param1 = x0 + 0.5 * (x1 - x0);
      param2 = x0 + 0.5 * (x2 - x0);
    }

    double pt1[4];
    nurbs.Evaluate (param1, 1, 2, pt1);
    Eigen::Vector2d t1 (pt1[2], pt1[3]);
    t1.normalize ();

    double pt2[4];
    nurbs.Evaluate (param2, 1, 2, pt2);
    Eigen::Vector2d t2 (pt2[2], pt2[3]);
    t2.normalize ();

    t = 0.5 * (t1 + t2);
  }
  else
  {
    double point_tangent[4];
    nurbs.Evaluate (min_param, 1, 2, point_tangent);
    t (0) = point_tangent[2];
    t (1) = point_tangent[3];
  }

  t.normalize ();
  p = min_pt;
  return min_param;
}

//double
//FittingCurve2d::inverseMapping (const ON_NurbsCurve &nurbs, const Eigen::Vector2d &pt, double* phint, double &error,
//                                Eigen::Vector2d &p, Eigen::Vector2d &t, int maxSteps, double accuracy, bool quiet)
//{
//  double hint;
//  Eigen::Vector2d r;
//  std::vector<double> elements = getElementVector (nurbs);
//  double points[2];
//
//  if (phint == NULL)
//  {
//    double d_shortest (DBL_MAX);
//    for (unsigned i = 0; i < elements.size () - 1; i++)
//    {
//      double d;
//
//      double xi = elements[i] + 0.5 * (elements[i + 1] - elements[i]);
//
//      nurbs.Evaluate (xi, 0, 2, points);
//      p (0) = points[0];
//      p (1) = points[1];
//
//      r = p - pt;
//
//      d = r.norm ();
//
//      if (d < d_shortest)
//      {
//        d_shortest = d;
//        hint = xi;
//      }
//    }
//  }
//  else
//  {
//    hint = *phint;
//  }
//
//  return inverseMapping (nurbs, pt, hint, error, p, t, maxSteps, accuracy, quiet);
//}

double
FittingCurve2d::findClosestElementMidPoint (const ON_NurbsCurve &nurbs, const Eigen::Vector2d &pt, double hint)
{
  // evaluate hint
  double param = hint;
  double points[2];
  nurbs.Evaluate (param, 0, 2, points);
  Eigen::Vector2d p (points[0], points[1]);
  Eigen::Vector2d r = p - pt;

  double d_shortest_hint = r.squaredNorm ();
  double d_shortest_elem (DBL_MAX);

  // evaluate elements
  std::vector<double> elements = pcl::on_nurbs::FittingCurve2d::getElementVector (nurbs);
  double seg = 1.0 / (nurbs.Order () - 1);

  for (unsigned i = 0; i < elements.size () - 1; i++)
  {
    double &xi0 = elements[i];
    double &xi1 = elements[i + 1];
    double dxi = xi1 - xi0;

    for (unsigned j = 0; j < nurbs.Order (); j++)
    {
      double xi = xi0 + (seg * j) * dxi;

      nurbs.Evaluate (xi, 0, 2, points);
      p (0) = points[0];
      p (1) = points[1];

      r = p - pt;

      double d = r.squaredNorm ();

      if (d < d_shortest_elem)
      {
        d_shortest_elem = d;
        param = xi;
      }
    }
  }

  if (d_shortest_hint < d_shortest_elem)
    return hint;
  else
    return param;
}

double
FittingCurve2d::findClosestElementMidPoint (const ON_NurbsCurve &nurbs, const Eigen::Vector2d &pt)
{
  double param (0.0);
  Eigen::Vector2d p, r;
  std::vector<double> elements = pcl::on_nurbs::FittingCurve2d::getElementVector (nurbs);
  double points[2];

  double d_shortest (DBL_MAX);
  double seg = 1.0 / (nurbs.Order () - 1);

  for (unsigned i = 0; i < elements.size () - 1; i++)
  {
    double &xi0 = elements[i];
    double &xi1 = elements[i + 1];
    double dxi = xi1 - xi0;

    for (unsigned j = 0; j < nurbs.Order (); j++)
    {
      double xi = xi0 + (seg * j) * dxi;

      nurbs.Evaluate (xi, 0, 2, points);
      p (0) = points[0];
      p (1) = points[1];

      r = p - pt;

      double d = r.squaredNorm ();

      if (d < d_shortest)
      {
        d_shortest = d;
        param = xi;
      }
    }
  }

  return param;
}

