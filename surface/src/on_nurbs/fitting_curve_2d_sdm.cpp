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

#include <pcl/surface/on_nurbs/fitting_curve_2d_sdm.h>
#include <stdexcept>

using namespace pcl;
using namespace on_nurbs;

FittingCurve2dSDM::FittingCurve2dSDM (int order, NurbsDataCurve2d *data) :
  FittingCurve2dPDM (order, data)
{

}

FittingCurve2dSDM::FittingCurve2dSDM (NurbsDataCurve2d *data, const ON_NurbsCurve &ns) :
  FittingCurve2dPDM (data, ns)
{

}

void
FittingCurve2dSDM::assemble (const FittingCurve2dPDM::Parameter &parameter)
{
  int cp_red = m_nurbs.m_order - 2;
  int ncp = m_nurbs.m_cv_count - 2 * cp_red;
  int nCageReg = m_nurbs.m_cv_count - 2 * cp_red;
  int nInt = int (m_data->interior.size ());

  double wInt = 1.0;
  if (!m_data->interior_weight.empty ())
    wInt = m_data->interior_weight[0];

  unsigned nrows = 4 * nInt + 2 * nCageReg;

  m_solver.assign (nrows, ncp * 2, 1);

  unsigned row (0);

  if (wInt > 0.0)
    assembleInterior (wInt, parameter.rScale, row);

  if (parameter.smoothness > 0.0)
    addCageRegularisation (parameter.smoothness, row);

  if (row < nrows)
  {
    m_solver.resize (row);
    if (!m_quiet)
      printf ("[FittingCurve2dSDM::assemble] Warning: rows do not match: %d %d\n", row, nrows);
  }
}

double
FittingCurve2dSDM::solve (double damp)
{
  double cps_diff (0.0);

  if (m_solver.solve ())
    cps_diff = updateCurve (damp);

  return cps_diff;
}

double
FittingCurve2dSDM::updateCurve (double damp)
{
  int cp_red = m_nurbs.m_order - 2;
  int ncp = m_nurbs.m_cv_count - 2 * cp_red;

  double cps_diff (0.0);

  // TODO this implementation rotates the control points, look up fitting_curve_2d_apdm for correct implementation

  for (int j = 0; j < ncp; j++)
  {

    ON_3dPoint cp_prev;
    m_nurbs.GetCV (j + cp_red, cp_prev);

    double x = m_solver.x (2 * j + 0, 0);
    double y = m_solver.x (2 * j + 1, 0);

    cps_diff += sqrt ((x - cp_prev.x) * (x - cp_prev.x) + (y - cp_prev.y) * (y - cp_prev.y));

    ON_3dPoint cp;
    cp.x = cp_prev.x + damp * (x - cp_prev.x);
    cp.y = cp_prev.y + damp * (y - cp_prev.y);
    cp.z = 0.0;

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
  return cps_diff / ncp;
}

void
FittingCurve2dSDM::addPointConstraint (const double &param, const Eigen::Vector2d &point,
                                       const Eigen::Vector2d &normal, const Eigen::Vector2d &tangent, double rho,
                                       double d, double weight, unsigned &row)
{
  int cp_red = m_nurbs.m_order - 2;
  int ncp = m_nurbs.m_cv_count - 2 * cp_red;
  double *N = new double[m_nurbs.m_order * m_nurbs.m_order];

  int E = ON_NurbsSpanIndex (m_nurbs.m_order, m_nurbs.m_cv_count, m_nurbs.m_knot, param, 0, 0);

  ON_EvaluateNurbsBasis (m_nurbs.m_order, m_nurbs.m_knot + E, param, N);

  m_solver.f (row, 0, normal (0) * point (0) * weight);
  for (int i = 0; i < m_nurbs.m_order; i++)
  {
    m_solver.K (row, 2 * ((E + i) % ncp) + 0, weight * normal (0) * N[i]);
  }
  row++;

  m_solver.f (row, 0, normal (1) * point (1) * weight);
  for (int i = 0; i < m_nurbs.m_order; i++)
  {
    m_solver.K (row, 2 * ((E + i) % ncp) + 1, weight * normal (1) * N[i]);
  }
  row++;

  //  if (d >= 0.0 && d > rho)
  //    printf("[FittingCurve2dSDM::addPointConstraint] Warning d > rho: %f > %f\n", d, rho);

  if (d < 0.0)
  {

    double a = d / (d - rho);

    m_solver.f (row, 0, a * a * tangent (0) * point (0) * weight);
    for (int i = 0; i < m_nurbs.m_order; i++)
      m_solver.K (row, 2 * ((E + i) % ncp) + 0, a * a * weight * tangent (0) * N[i]);
    row++;

    m_solver.f (row, 0, a * a * tangent (1) * point (1) * weight);
    for (int i = 0; i < m_nurbs.m_order; i++)
      m_solver.K (row, 2 * ((E + i) % ncp) + 1, a * a * weight * tangent (1) * N[i]);
    row++;

  }

  delete [] N;
}

void
FittingCurve2dSDM::addCageRegularisation (double weight, unsigned &row)
{
  int cp_red = (m_nurbs.m_order - 2);
  int ncp = (m_nurbs.m_cv_count - 2 * cp_red);

  for (int j = 1; j < ncp + 1; j++)
  {
    m_solver.K (row, 2 * ((j + 0) % ncp) + 0, -2.0 * weight);
    m_solver.K (row, 2 * ((j - 1) % ncp) + 0, 1.0 * weight);
    m_solver.K (row, 2 * ((j + 1) % ncp) + 0, 1.0 * weight);
    row++;

    m_solver.K (row, 2 * ((j + 0) % ncp) + 1, -2.0 * weight);
    m_solver.K (row, 2 * ((j - 1) % ncp) + 1, 1.0 * weight);
    m_solver.K (row, 2 * ((j + 1) % ncp) + 1, 1.0 * weight);
    row++;
  }
}

void
FittingCurve2dSDM::assembleInterior (double wInt, double rScale, unsigned &row)
{
  unsigned nInt = int (m_data->interior.size ());
  m_data->interior_line_start.clear ();
  m_data->interior_line_end.clear ();
  m_data->interior_error.clear ();
  m_data->interior_normals.clear ();

  unsigned updateTNR (false);
  if (m_data->interior_ncps_prev != m_nurbs.CVCount ())
  {
    if (!m_quiet)
      printf ("[FittingCurve2dSDM::assembleInterior] updating T, N, rho\n");
    m_data->interior_tangents.clear ();
    m_data->interior_normals.clear ();
    m_data->interior_rho.clear ();
    m_data->interior_ncps_prev = m_nurbs.CVCount ();
    updateTNR = true;
  }

  for (unsigned p = 0; p < nInt; p++)
  {
    Eigen::Vector2d &pcp = m_data->interior[p];

    // inverse mapping
    double param;
    Eigen::Vector2d pt, t, n;
    double error;
    if (p < static_cast<unsigned int>(m_data->interior_param.size ()))
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

    double dt, kappa, rho, rho_prev;
    Eigen::Vector2d n_prev, t_prev;

    double pointAndTangents[6];
    m_nurbs.Evaluate (param, 2, 2, pointAndTangents);
    pt (0) = pointAndTangents[0];
    pt (1) = pointAndTangents[1];
    t (0) = pointAndTangents[2];
    t (1) = pointAndTangents[3];
    n (0) = pointAndTangents[4];
    n (1) = pointAndTangents[5];

    dt = t.norm ();
    t /= dt;
    Eigen::Vector2d in (t (1), -t (0));
    n /= dt; // TODO something is wrong with the normal from nurbs.Evaluate(...)
    n = in * in.dot (n);

    kappa = n.norm ();
    rho = (1.0 / kappa);
    n *= rho;

    if (!updateTNR && m_data->interior_rho.size () == nInt)
    {
      n_prev = m_data->interior_normals[p];
      t_prev = m_data->interior_tangents[p];
      rho_prev = m_data->interior_rho[p];
      //        m_data->interior_normals[p] = n;
      //        m_data->interior_tangents[p] = t;
      //        m_data->interior_rho[p] = rho;
    }
    else
    {
      m_data->interior_tangents.push_back (t);
      m_data->interior_normals.push_back (n);
      m_data->interior_rho.push_back (rho);
      n_prev = n;
      t_prev = t;
      rho_prev = rho;
    }

    // signed distance
    double d;
    if ((pcp - pt).dot (n) >= 0.0)
      d = (pcp - pt).norm ();
    else
      d = -(pcp - pt).norm ();

    if (p < m_data->interior_weight.size ())
      wInt = m_data->interior_weight[p];

    m_data->interior_line_start.push_back (pt);
    m_data->interior_line_end.push_back (pcp);

    addPointConstraint (m_data->interior_param[p], m_data->interior[p], n_prev, t_prev, rho_prev, d, wInt, row);
  }

  //  printf("[FittingCurve2dSDM::assembleInterior] d>0: %d d<0: %d\n", i1, i2);
}
