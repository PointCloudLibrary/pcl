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

#include <pcl/surface/on_nurbs/fitting_curve_2d_tdm.h>
#include <stdexcept>

using namespace pcl;
using namespace on_nurbs;

FittingCurve2dTDM::FittingCurve2dTDM (int order, NurbsDataCurve2d *data) :
  FittingCurve2dPDM (order, data)
{
}

FittingCurve2dTDM::FittingCurve2dTDM (NurbsDataCurve2d *data, const ON_NurbsCurve &ns) :
  FittingCurve2dPDM (data, ns)
{
}

void
FittingCurve2dTDM::assemble (const FittingCurve2dPDM::Parameter &parameter)
{
  int cp_red = m_nurbs.m_order - 2;
  int ncp = m_nurbs.m_cv_count - 2 * cp_red;
  int nCageReg = m_nurbs.m_cv_count - 2 * cp_red;
  int nInt = int (m_data->interior.size ());

  double wInt = 1.0;
  if (!m_data->interior_weight.empty ())
    wInt = m_data->interior_weight[0];

  unsigned nrows = 2 * nInt + 2 * nCageReg;

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
      printf ("[FittingCurve2dTDM::assemble] Warning: rows do not match: %d %d\n", row, nrows);
  }
}

double
FittingCurve2dTDM::solve (double damp)
{
  double cps_diff (0.0);

  if (m_solver.solve ())
    cps_diff = updateCurve (damp);

  return cps_diff;
}

double
FittingCurve2dTDM::updateCurve (double damp)
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
FittingCurve2dTDM::addPointConstraint (const double &param, const Eigen::Vector2d &point,
                                        const Eigen::Vector2d &normal, double weight, unsigned &row)
{
  int cp_red = m_nurbs.m_order - 2;
  int ncp = m_nurbs.m_cv_count - 2 * cp_red;
  double *N = new double[m_nurbs.m_order * m_nurbs.m_order];

  int E = ON_NurbsSpanIndex (m_nurbs.m_order, m_nurbs.m_cv_count, m_nurbs.m_knot, param, 0, 0);

  ON_EvaluateNurbsBasis (m_nurbs.m_order, m_nurbs.m_knot + E, param, N);

  // X
  m_solver.f (row, 0, normal (0) * point (0) * weight);
  for (int i = 0; i < m_nurbs.m_order; i++)
    m_solver.K (row, 2 * ((E + i) % ncp) + 0, weight * normal (0) * N[i]);
  row++;

  // Y
  m_solver.f (row, 0, normal (1) * point (1) * weight);
  for (int i = 0; i < m_nurbs.m_order; i++)
    m_solver.K (row, 2 * ((E + i) % ncp) + 1, weight * normal (1) * N[i]);
  row++;

  delete [] N;
}

void
FittingCurve2dTDM::addCageRegularisation (double weight, unsigned &row)
{
  int cp_red = (m_nurbs.m_order - 2);
  int ncp = (m_nurbs.m_cv_count - 2 * cp_red);

  //  m_data->interior_line_start.clear();
  //  m_data->interior_line_end.clear();
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
FittingCurve2dTDM::assembleInterior (double wInt, double rScale, unsigned &row)
{
  int nInt = int (m_data->interior.size ());
  m_data->interior_line_start.clear ();
  m_data->interior_line_end.clear ();
  m_data->interior_error.clear ();
  m_data->interior_normals.clear ();
  for (int p = 0; p < nInt; p++)
  {
    Eigen::Vector2d &pcp = m_data->interior[p];

    // inverse mapping
    double param;
    Eigen::Vector2d pt, t, n;
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

    double pointAndTangents[6];
    m_nurbs.Evaluate (param, 2, 2, pointAndTangents);
    pt (0) = pointAndTangents[0];
    pt (1) = pointAndTangents[1];
    t (0) = pointAndTangents[2];
    t (1) = pointAndTangents[3];
    n (0) = pointAndTangents[4];
    n (1) = pointAndTangents[5];
    n.normalize ();

    if (p < int (m_data->interior_weight.size ()))
      wInt = m_data->interior_weight[p];

    addPointConstraint (m_data->interior_param[p], m_data->interior[p], n, wInt, row);
  }
}
