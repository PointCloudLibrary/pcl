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

#include <pcl/surface/on_nurbs/fitting_curve_pdm.h>
#include <pcl/pcl_macros.h>
#include <limits>
#include <stdexcept>

using namespace pcl;
using namespace on_nurbs;

FittingCurve::FittingCurve (int order, NurbsDataCurve *data)
{
  if (order < 2)
    throw std::runtime_error ("[NurbsFittingCylinder::NurbsFittingCylinder] Error order to low (order<2).");

  ON::Begin ();

  m_data = data;
  m_nurbs = initNurbsCurvePCA (order, m_data->interior);

  in_max_steps = 100;
  in_accuracy = 1e-4;
  m_quiet = true;
}

FittingCurve::FittingCurve (NurbsDataCurve *data, const ON_NurbsCurve &ns)
{
  ON::Begin ();

  m_nurbs = ON_NurbsCurve (ns);
  m_data = data;

  in_max_steps = 100;
  in_accuracy = 1e-4;
  m_quiet = true;
}

int
FittingCurve::findElement (double xi, const std::vector<double> &elements)
{
  if (xi >= elements.back ())
    return (int (elements.size ()) - 2);

  for (std::size_t i = 0; i < elements.size () - 1; i++)
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
FittingCurve::refine ()
{
  std::vector<double> xi;

  std::vector<double> elements = getElementVector (m_nurbs);

  for (std::size_t i = 0; i < elements.size () - 1; i++)
    xi.push_back (elements[i] + 0.5 * (elements[i + 1] - elements[i]));

  for (const double &i : xi)
    m_nurbs.InsertKnot (i, 1);
}

void
FittingCurve::assemble (const Parameter &parameter)
{
  int cp_red = m_nurbs.m_order - 2;
  int ncp = m_nurbs.m_cv_count - 2 * cp_red;
  int nCageReg = m_nurbs.m_cv_count - 2 * cp_red;
  int nInt = int (m_data->interior.size ());

  int nrows = nInt + nCageReg;

  double wInt = 1.0;
  double wCageReg = parameter.smoothness;

  m_solver.assign (nrows, ncp, 3);

  unsigned row (0);

  assembleInterior (wInt, row);

  addCageRegularisation (wCageReg, row);
}

void
FittingCurve::solve (double damp)
{
  if (m_solver.solve ())
    updateCurve (damp);
}

void
FittingCurve::updateCurve (double)
{
  int cp_red = m_nurbs.m_order - 2;
  int ncp = m_nurbs.m_cv_count - 2 * cp_red;

  for (int j = 0; j < ncp; j++)
  {

    ON_3dPoint cp;
    cp.x = m_solver.x (j, 0);
    cp.y = m_solver.x (j, 1);
    cp.z = m_solver.x (j, 2);

    m_nurbs.SetCV (j + cp_red, cp);
  }

  for (int j = 0; j < cp_red; j++)
  {

    ON_3dPoint cp;
    m_nurbs.GetCV (m_nurbs.m_cv_count - 1 - cp_red + j, cp);
    m_nurbs.SetCV (j, cp);

    m_nurbs.GetCV (cp_red - j, cp);
    m_nurbs.SetCV (m_nurbs.m_cv_count - 1 - j, cp);
  }
}

void
FittingCurve::addPointConstraint (const double &param, const Eigen::Vector3d &point, double weight, unsigned &row)
{
  int cp_red = m_nurbs.m_order - 2;
  int ncp = m_nurbs.m_cv_count - 2 * cp_red;
  double *N = new double[m_nurbs.m_order * m_nurbs.m_order];

  int E = ON_NurbsSpanIndex (m_nurbs.m_order, m_nurbs.m_cv_count, m_nurbs.m_knot, param, 0, 0);

  ON_EvaluateNurbsBasis (m_nurbs.m_order, m_nurbs.m_knot + E, param, N);

  m_solver.f (row, 0, point (0) * weight);
  m_solver.f (row, 1, point (1) * weight);
  m_solver.f (row, 2, point (2) * weight);

  for (int i = 0; i < m_nurbs.m_order; i++)
    m_solver.K (row, (E + i) % ncp, weight * N[i]);

  row++;

  delete [] N;
}

void
FittingCurve::addCageRegularisation (double weight, unsigned &row)
{
  int cp_red = (m_nurbs.m_order - 2);
  int ncpj = (m_nurbs.m_cv_count - 2 * cp_red);

  m_solver.f (row, 0, 0.0);
  m_solver.f (row, 1, 0.0);
  m_solver.f (row, 2, 0.0);

  for (int j = 1; j < ncpj + 1; j++)
  {

    m_solver.K (row, (j + 0) % ncpj, -2.0 * weight);
    m_solver.K (row, (j - 1) % ncpj, 1.0 * weight);
    m_solver.K (row, (j + 1) % ncpj, 1.0 * weight);

    row++;
  }
}

ON_NurbsCurve
FittingCurve::initNurbsCurve2D (int order, const vector_vec2d &data)
{
  if (data.empty ())
    printf ("[FittingCurve::initNurbsCurve2D] Warning, no boundary parameters available\n");

  Eigen::Vector2d mean = NurbsTools::computeMean (data);

  unsigned s = unsigned (data.size ());

  double r (0.0);
  for (unsigned i = 0; i < s; i++)
  {
    Eigen::Vector2d d = data[i] - mean;
    double sn = d.squaredNorm ();
    if (sn > r)
      r = sn;
  }
  r = sqrt (r);

  int ncpsV (2 * order);
  ON_NurbsCurve nurbs = ON_NurbsCurve (3, false, order, ncpsV);
  nurbs.MakePeriodicUniformKnotVector (1.0 / (ncpsV - order + 1));

  double dcv = (2.0 * M_PI) / (ncpsV - order + 1);
  Eigen::Vector2d cv;
  for (int j = 0; j < ncpsV; j++)
  {
    cv (0) = r * sin (dcv * j);
    cv (1) = r * std::cos (dcv * j);
    cv += mean;
    nurbs.SetCV (j, ON_3dPoint (cv (0), cv (1), 0.0));
  }

  return nurbs;
}

ON_NurbsCurve
FittingCurve::initNurbsCurvePCA (int order, const vector_vec3d &data, int ncps, double rf)
{
  if (data.empty ())
    printf ("[FittingCurve::initNurbsCurvePCA] Warning, no boundary parameters available\n");

  Eigen::Vector3d mean;
  Eigen::Matrix3d eigenvectors;
  Eigen::Vector3d eigenvalues;

  unsigned s = unsigned (data.size ());

  NurbsTools::pca (data, mean, eigenvectors, eigenvalues);

  eigenvalues /= s; // seems that the eigenvalues are dependent on the number of points (???)

  double r = rf * sqrt (eigenvalues (0));

  if (ncps < 2 * order)
    ncps = 2 * order;

  ON_NurbsCurve nurbs = ON_NurbsCurve (3, false, order, ncps);
  nurbs.MakePeriodicUniformKnotVector (1.0 / (ncps - order + 1));

  double dcv = (2.0 * M_PI) / (ncps - order + 1);
  Eigen::Vector3d cv, cv_t;
  for (int j = 0; j < ncps; j++)
  {
    cv (0) = r * sin (dcv * j);
    cv (1) = r * std::cos (dcv * j);
    cv (2) = 0.0;
    cv_t = eigenvectors * cv + mean;
    nurbs.SetCV (j, ON_3dPoint (cv_t (0), cv_t (1), cv_t (2)));
  }

  return nurbs;
}

std::vector<double>
FittingCurve::getElementVector (const ON_NurbsCurve &nurbs)
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
FittingCurve::assembleInterior (double wInt, unsigned &row)
{
  int nInt = int (m_data->interior.size ());
  m_data->interior_line_start.clear ();
  m_data->interior_line_end.clear ();
  m_data->interior_error.clear ();
  m_data->interior_normals.clear ();
  for (int p = 0; p < nInt; p++)
  {
    Eigen::Vector3d pcp (m_data->interior[p] (0), m_data->interior[p] (1), m_data->interior[p] (2));

    // inverse mapping
    double param;
    Eigen::Vector3d pt, t;
    double error;
    if (p < int (m_data->interior_param.size ()))
    {
      param = inverseMapping (m_nurbs, pcp, m_data->interior_param[p], error, pt, t, in_max_steps, in_accuracy);
      m_data->interior_param[p] = param;
    }
    else
    {
      param = findClosestElementMidPoint (m_nurbs, pcp);
      param = inverseMapping (m_nurbs, pcp, param, error, pt, t, in_max_steps, in_accuracy);
      m_data->interior_param.push_back (param);
    }

    m_data->interior_error.push_back (error);
    m_data->interior_line_start.push_back (pcp);
    m_data->interior_line_end.push_back (pt);

    addPointConstraint (m_data->interior_param[p], m_data->interior[p], wInt, row);
  }
}

double
FittingCurve::inverseMapping (const ON_NurbsCurve &nurbs, const Eigen::Vector3d &pt, const double &hint, double &error,
                              Eigen::Vector3d &p, Eigen::Vector3d &t, int maxSteps, double accuracy, bool quiet)
{
  //int cp_red = (nurbs.m_order - 2);
  //int ncpj = int (nurbs.m_cv_count - 2 * cp_red);
  double pointAndTangents[6];

  double current, delta;
  Eigen::Vector3d r;
  std::vector<double> elements = getElementVector (nurbs);
  double minU = elements[0];
  double maxU = elements[elements.size () - 1];

  current = hint;

  for (int k = 0; k < maxSteps; k++)
  {

    nurbs.Evaluate (current, 1, 3, pointAndTangents);

    p (0) = pointAndTangents[0];
    p (1) = pointAndTangents[1];
    p (2) = pointAndTangents[2];

    t (0) = pointAndTangents[3];
    t (1) = pointAndTangents[4];
    t (2) = pointAndTangents[5];

    r = p - pt;

    // step width control
    int E = findElement (current, elements);
    double e = elements[E + 1] - elements[E];

    delta = -(0.5 * e) * r.dot (t) / t.norm (); //  A.ldlt().solve(b);

    if (std::fabs (delta) < accuracy)
    {

      error = r.norm ();
      return current;

    }
    current += delta;

    if (current < minU)
      current = maxU - (minU - current);
    else if (current > maxU)
      current = minU + (current - maxU);
  }

  error = r.norm ();

  if (!quiet)
  {
    printf ("[FittingCurve::inverseMapping] Warning: Method did not converge (%e %d).\n", accuracy, maxSteps);
    printf ("[FittingCurve::inverseMapping] hint: %f current: %f delta: %f error: %f\n", hint, current, delta, error);
  }

  return current;
}

double
FittingCurve::findClosestElementMidPoint (const ON_NurbsCurve &nurbs, const Eigen::Vector3d &pt)
{
  double hint (0.0);
  Eigen::Vector3d p, r;
  std::vector<double> elements = getElementVector (nurbs);
  double points[3];

  double d_shortest (std::numeric_limits<double>::max());

  for (std::size_t i = 0; i < elements.size () - 1; i++)
  {
    double xi = elements[i] + 0.5 * (elements[i + 1] - elements[i]);

    nurbs.Evaluate (xi, 0, 3, points);
    p (0) = points[0];
    p (1) = points[1];
    p (2) = points[2];

    r = p - pt;

    double d = r.squaredNorm ();

    if (d < d_shortest)
    {
      d_shortest = d;
      hint = xi;
    }
  }

  return hint;
}

