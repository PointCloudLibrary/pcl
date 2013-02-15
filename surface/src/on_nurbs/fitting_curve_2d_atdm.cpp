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

#include <pcl/surface/on_nurbs/fitting_curve_2d_atdm.h>
#include <stdexcept>

using namespace pcl;
using namespace on_nurbs;

FittingCurve2dATDM::FittingCurve2dATDM (int order, NurbsDataCurve2d *data) :
  FittingCurve2dAPDM (order, data)
{
}

FittingCurve2dATDM::FittingCurve2dATDM (NurbsDataCurve2d *data, const ON_NurbsCurve &ns) :
  FittingCurve2dAPDM (data, ns)
{
}

void
FittingCurve2dATDM::assemble (const FittingCurve2dAPDM::Parameter &parameter)
{
  int cp_red = m_nurbs.m_order - 2;
  int ncp = m_nurbs.m_cv_count - 2 * cp_red;
  int nCageReg = m_nurbs.m_cv_count - 2 * cp_red;
  int nInt = int (m_data->interior.size ());
  //  int nCommon = m_data->common.size();
  //  int nClosestP = parameter.closest_point_resolution;

  std::vector<double> elements = getElementVector (m_nurbs);
  int nClosestP = int (elements.size ());

  double wInt = 1.0;
  if (!m_data->interior_weight.empty ())
  {
    wInt = m_data->interior_weight[0];
  }

  double wCageReg = parameter.smoothness;

  unsigned nrows = 2 * nInt + 2 * nCageReg + 2 * nClosestP;

  m_solver.assign (nrows, ncp * 2, 1);

  unsigned row (0);

  if (wInt > 0.0)
    assembleInterior (wInt, parameter.interior_sigma2, parameter.rScale, row);

  assembleClosestPoints (elements, parameter.closest_point_weight, parameter.closest_point_sigma2, row);

  if (wCageReg > 0.0)
    addCageRegularisation (wCageReg, row, elements, parameter.smooth_concavity);

  if (row < nrows)
  {
    m_solver.resize (row);
    if (!m_quiet)
      printf ("[FittingCurve2dATDM::assemble] Warning: rows do not match: %d %d\n", row, nrows);
  }
}

double
FittingCurve2dATDM::solve (double damp)
{
  double cps_diff (0.0);

  if (m_solver.solve ())
    cps_diff = updateCurve (damp);

  return cps_diff;
}

double
FittingCurve2dATDM::updateCurve (double damp)
{
  int cp_red = m_nurbs.m_order - 2;
  int ncp = m_nurbs.m_cv_count - 2 * cp_red;

  double cps_diff (0.0);

  for (int j = 0; j < ncp; j++)
  {

    ON_3dPoint cp_prev;
    m_nurbs.GetCV (j, cp_prev);

    double x = m_solver.x (2 * j + 0, 0);
    double y = m_solver.x (2 * j + 1, 0);

    cps_diff += sqrt ((x - cp_prev.x) * (x - cp_prev.x) + (y - cp_prev.y) * (y - cp_prev.y));

    ON_3dPoint cp;
    cp.x = cp_prev.x + damp * (x - cp_prev.x);
    cp.y = cp_prev.y + damp * (y - cp_prev.y);
    cp.z = 0.0;

    m_nurbs.SetCV (j, cp);
  }

  for (int j = 0; j < 2 * cp_red; j++)
  {
    ON_3dPoint cp;
    m_nurbs.GetCV (2 * cp_red - 1 - j, cp);
    m_nurbs.SetCV (m_nurbs.m_cv_count - 1 - j, cp);
  }

  return cps_diff / ncp;
}

void
FittingCurve2dATDM::addPointConstraint (const double &param, const Eigen::Vector2d &point,
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

  delete[] N;
}

void
FittingCurve2dATDM::addCageRegularisation (double weight, unsigned &row, const std::vector<double> &elements,
                                           double wConcav)
{
  int cp_red = (m_nurbs.m_order - 2);
  int ncp = (m_nurbs.m_cv_count - 2 * cp_red);

  //  m_data->interior_line_start.clear();
  //  m_data->interior_line_end.clear();
  for (int j = 1; j < ncp + 1; j++)
  {

    if (wConcav == 0.0)
    {
    }
    else
    {
      int i = j % ncp;

      if (i >= int (m_data->closest_points_error.size () - 1))
      {
        printf ("[FittingCurve2dATDM::addCageRegularisation] Warning, index for closest_points_error out of bounds\n");
      }
      else
      {
        Eigen::Vector2d t, n;
        double pt[4];

        double xi = elements[i] + 0.5 * (elements[i + 1] - elements[i]);
        m_nurbs.Evaluate (xi, 1, 2, pt);
        t (0) = pt[2];
        t (1) = pt[3];
        n (0) = -t (1);
        n (1) = t (0);
        n.normalize ();

        double err = m_data->closest_points_error[i] + 0.5 * (m_data->closest_points_error[i + 1]
            - m_data->closest_points_error[i]);
        m_solver.f (row + 0, 0, err * wConcav * n (0));
        m_solver.f (row + 1, 0, err * wConcav * n (1));

        //        Eigen::Vector2d p1, p2;
        //        p1(0) = pt[0];
        //        p1(1) = pt[1];
        //        p2 = p1 + n * wConcav * err;
        //        m_data->interior_line_start.push_back(p1);
        //        m_data->interior_line_end.push_back(p2);
      }
    }

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
FittingCurve2dATDM::assembleInterior (double wInt, double sigma2, double rScale, unsigned &row)
{
  int nInt = int (m_data->interior.size ());
  bool wFunction (true);
  double ds = 1.0 / (2.0 * sigma2);
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

    // evaluate if point lies inside or outside the closed curve
    Eigen::Vector3d a (pcp (0) - pt (0), pcp (1) - pt (1), 0.0);
    Eigen::Vector3d b (t (0), t (1), 0.0);
    Eigen::Vector3d z = a.cross (b);

    if (p < int (m_data->interior_weight.size ()))
      wInt = m_data->interior_weight[p];

    if (p < int (m_data->interior_weight_function.size ()))
      wFunction = m_data->interior_weight_function[p];

    double w (wInt);
    if (z (2) > 0.0 && wFunction)
      w = wInt * exp (-(error * error) * ds);

    n.normalize ();
    //    m_data->interior_line_start.push_back(pt);
    //    m_data->interior_line_end.push_back(pt + n * 0.01);

    //      w = 0.5 * wInt * exp(-(error * error) * ds);

    // evaluate if this point is the closest point
    //    int idx = NurbsTools::getClosestPoint(pt, m_data->interior);
    //    if(idx == p)
    //      w = 2.0 * wInt;

    if (w > 1e-6) // avoids ill-conditioned matrix
      addPointConstraint (m_data->interior_param[p], m_data->interior[p], n, w, row);
    else
    {
      //      m_solver.K(row, 0, 0.0);
      //      row++;
    }
  }
}

void
FittingCurve2dATDM::assembleClosestPoints (const std::vector<double> &elements, double weight, double sigma2,
                                           unsigned &row)
{
  m_data->closest_points.clear ();
  m_data->closest_points_param.clear ();
  m_data->closest_points_error.clear ();
  m_data->interior_line_start.clear ();
  m_data->interior_line_end.clear ();

  double ds = 1.0 / (2.0 * sigma2);

  for (unsigned i = 0; i < elements.size (); i++)
  {

    int j = i % int (elements.size ());

    double dxi = elements[j] - elements[i];
    double xi = elements[i] + 0.5 * dxi;

    double points[6];
    Eigen::Vector2d p1, p2, p3, t, in, n;
    m_nurbs.Evaluate (xi, 2, 2, points);
    p1 (0) = points[0];
    p1 (1) = points[1];
    t (0) = points[2];
    t (1) = points[3];
    t.normalize ();
    in (0) = t (1);
    in (1) = -t (0);

    n (0) = points[4] * 0.01;
    n (1) = points[5] * 0.01;

    n = in * in.dot (n);

    unsigned idxcp;
    unsigned idx = NurbsTools::getClosestPoint (p1, in, m_data->interior, idxcp);
    p2 = m_data->interior[idx];
    p3 = m_data->interior[idxcp];

    //    double xi2 = m_data->interior_param[idx];

    double error2 = (p2 - p1).squaredNorm ();

    m_data->closest_points.push_back (p3);
    m_data->closest_points_param.push_back (xi);
    m_data->closest_points_error.push_back ((p3 - p1).squaredNorm ());

    double w (weight);
    w = 0.5 * weight * exp (-(error2) * ds);
    //    w = weight * std::fabs(in.dot(p2-p1));

    //    if (weight > 0.0 && (std::fabs(xi2 - xi) < std::fabs(dxi)))
    if (w > 0.0)
    {
      addPointConstraint (xi, p2, n, w, row);
      m_data->interior_line_start.push_back (p1);
      m_data->interior_line_end.push_back (p2);
    }

  }
}

void
FittingCurve2dATDM::assembleClosestPoints (int res, double weight, unsigned &row)
{
  std::vector<double> elements = FittingCurve2dATDM::getElementVector (m_nurbs);
  double xi_min = elements.front ();
  double xi_max = elements.back ();

  double step = (xi_max - xi_min) / res;

  //  m_data->interior_line_start.clear();
  //  m_data->interior_line_end.clear();
  m_data->closest_points.clear ();
  m_data->closest_points_param.clear ();
  m_data->closest_points_error.clear ();
  for (int i = 0; i < res; i++)
  {
    double xi = xi_min + i * step;

    double points[6];
    Eigen::Vector2d p1, p2, t, in, n;
    //    m_nurbs.Evaluate(xi, 0, 2, points);
    //    p1(0) = points[0];
    //    p1(1) = points[1];

    m_nurbs.Evaluate (xi, 2, 2, points);
    p1 (0) = points[0];
    p1 (1) = points[1];
    t (0) = points[2];
    t (1) = points[3];
    in (0) = t (1);
    in (1) = -t (0);
    n (0) = points[4];
    n (1) = points[5];
    n = in * in.dot (n);

    unsigned idx = NurbsTools::getClosestPoint (p1, m_data->interior);
    p2 = m_data->interior[idx];

    m_data->closest_points.push_back (p2);
    m_data->closest_points_param.push_back (xi);
    m_data->closest_points_error.push_back ((p2 - p1).squaredNorm ());
    //    m_data->interior_line_start.push_back(p1);
    //    m_data->interior_line_end.push_back(p2);

    addPointConstraint (xi, p2, n, weight, row);

  }
}
