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

#include <pcl/surface/on_nurbs/fitting_curve_2d_apdm.h>
#include <pcl/pcl_macros.h>
#include <limits>
#include <stdexcept>
#include <Eigen/Geometry> // for cross

using namespace pcl;
using namespace on_nurbs;

FittingCurve2dAPDM::FittingCurve2dAPDM (int order, NurbsDataCurve2d *data)
{
  if (order < 2)
    throw std::runtime_error ("[NurbsFittingCylinder::NurbsFittingCylinder] Error order to low (order<2).");

  ON::Begin ();

  m_data = data;
  m_nurbs = initNurbsCurve2D (order, m_data->interior);

  in_max_steps = 200;
  in_accuracy = 1e-6;
  m_quiet = true;
}

FittingCurve2dAPDM::FittingCurve2dAPDM (NurbsDataCurve2d *data, const ON_NurbsCurve &nc)
{
  ON::Begin ();

  m_nurbs = ON_NurbsCurve (nc);
  m_data = data;

  in_max_steps = 200;
  in_accuracy = 1e-6;
  m_quiet = true;
}

int
FittingCurve2dAPDM::findElement (double xi, const std::vector<double> &elements)
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
FittingCurve2dAPDM::refine ()
{
  std::vector<double> xi;

  std::vector<double> elements = getElementVector (m_nurbs);

  for (std::size_t i = 0; i < elements.size () - 1; i++)
    xi.push_back (elements[i] + 0.5 * (elements[i + 1] - elements[i]));

  for (const double &i : xi)
    m_nurbs.InsertKnot (i, 1);
}

void
FittingCurve2dAPDM::refine (double xi)
{
  std::vector<double> elements = getElementVector (m_nurbs);

  int i = findElement (xi, elements);

  double _xi = elements[i] + 0.5 * (elements[i + 1] - elements[i]);
  m_nurbs.InsertKnot (_xi, 1);
  return;
}

void
FittingCurve2dAPDM::fitting (FitParameter &param)
{
  bool stop (false);
  for (unsigned j = 0; j < param.iterations && !stop; j++)
  {
    if (2 * m_nurbs.CVCount () < static_cast<int>(m_data->interior.size ()))
      if (!(j % param.addCPsIteration))
        if (m_nurbs.CVCount () < static_cast<int>(param.maxCPs))
          addCPsOnClosestPointViolation (param.addCPsAccuracy);

    assemble (param.param);

    double cps_diff = solve ();

    if (!m_quiet)
      printf ("[FittingCurve2dAPDM::fitting] accuracy: %f / %f\n", cps_diff, param.accuracy);

    if (cps_diff < param.accuracy)
      stop = true;
  }
}

void
FittingCurve2dAPDM::assemble (const Parameter &parameter)
{
  std::vector<double> elements = getElementVector (m_nurbs);
  unsigned cp_res = std::max<unsigned> (1, parameter.closest_point_resolution);

  int cp_red = m_nurbs.m_order - 2;
  int ncp = m_nurbs.m_cv_count - 2 * cp_red;
  int nCageReg = m_nurbs.m_cv_count - 2 * cp_red;
  int nInt = int (m_data->interior.size ());
  int nClosestP = int (elements.size ()) * cp_res;

  double wInt = 1.0;
  if (!m_data->interior_weight.empty ())
  {
    wInt = m_data->interior_weight[0];
  }

  double wCageReg = parameter.smoothness;

  unsigned nrows = nInt + nClosestP + nCageReg;

  m_solver.assign (nrows, ncp, 2);

  unsigned row (0);

  if (wInt > 0.0)
    assembleInterior (wInt, parameter.interior_sigma2, parameter.rScale, row);

  assembleClosestPoints (elements, parameter.closest_point_weight, parameter.closest_point_sigma2, cp_res, row);

  if (wCageReg > 0.0)
    addCageRegularisation (wCageReg, row, elements, parameter.smooth_concavity);

  if (row < nrows)
  {
    m_solver.resize (row);
    if (!m_quiet)
      printf ("[FittingCurve2dAPDM::assemble] Warning: rows do not match: %d %d\n", row, nrows);
  }
}

double
FittingCurve2dAPDM::solve (double damp)
{
  double cps_diff (0.0);

  if (m_solver.solve ())
    cps_diff = updateCurve (damp);

  return cps_diff;
}

double
FittingCurve2dAPDM::updateCurve (double damp)
{
  int cp_red = m_nurbs.m_order - 2;
  int ncp = m_nurbs.m_cv_count - 2 * cp_red;

  double cps_diff (0.0);
  //  double cps_diff_max (0.0);

  for (int j = 0; j < ncp; j++)
  {

    ON_3dPoint cp_prev;
    m_nurbs.GetCV (j, cp_prev);

    double x = m_solver.x (j, 0);
    double y = m_solver.x (j, 1);

    cps_diff += sqrt ((x - cp_prev.x) * (x - cp_prev.x) + (y - cp_prev.y) * (y - cp_prev.y));

    //    if (cps_diff > cps_diff_max)
    //      cps_diff_max = cps_diff;

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
FittingCurve2dAPDM::addCPsOnClosestPointViolation (double max_error)
{
  std::vector<double> elements = getElementVector (m_nurbs);
  //  m_data->interior_line_start.clear ();
  //  m_data->interior_line_end.clear ();

  //int nknots (0);

  for (std::size_t i = 0; i < elements.size () - 1; i++)
  {

    bool inserted (false);
    double dxi = elements[i + 1] - elements[i];

    {
      double xi = elements[i];
      double points[2];
      Eigen::Vector2d p1, p2;
      m_nurbs.Evaluate (xi, 0, 2, points);
      p1 (0) = points[0];
      p1 (1) = points[1];

      unsigned idx = NurbsTools::getClosestPoint (p1, m_data->interior);
      p2 = m_data->interior[idx];

      double d = (p2 - p1).squaredNorm ();

      if (d > (max_error * max_error))
      {
        m_nurbs.InsertKnot (xi + 0.5 * dxi, 1);
        //        m_data->interior_line_start.push_back (p2);
        //        m_data->interior_line_end.push_back (p1);
        //nknots++;
        inserted = true;
      }
    }

    if (!inserted)
    {
      double xi = elements[i] + 0.5 * dxi;
      double points[2];
      Eigen::Vector2d p1, p2;
      m_nurbs.Evaluate (xi, 0, 2, points);
      p1 (0) = points[0];
      p1 (1) = points[1];

      unsigned idx = NurbsTools::getClosestPoint (p1, m_data->interior);
      p2 = m_data->interior[idx];

      double d = (p2 - p1).squaredNorm ();

      if (d > (max_error * max_error))
      {
        m_nurbs.InsertKnot (xi, 1);
        //        m_data->interior_line_start.push_back (p2);
        //        m_data->interior_line_end.push_back (p1);
        //nknots++;
      }
    }

  }
  //  printf("[FittingCurve2dAPDM::addCPsOnClosestPointViolation] %d knots inserted (%d, %d)\n", nknots,
  //      m_nurbs.CVCount(), m_nurbs.KnotCount());
}

ON_NurbsCurve
FittingCurve2dAPDM::removeCPsOnLine (const ON_NurbsCurve &nurbs, double min_curve_th)
{
  int cp_red = nurbs.Order () - 2;
  int ncp = nurbs.CVCount () - 2 * cp_red;

  std::vector<ON_3dPoint> cps;

  for (int j = 1; j < ncp + 1; j++)
  {
    ON_3dPoint cp0, cp1, cp2;
    nurbs.GetCV ((j + 0) % ncp, cp0);
    nurbs.GetCV ((j - 1) % ncp, cp1);
    nurbs.GetCV ((j + 1) % ncp, cp2);

    Eigen::Vector3d v1 (cp1.x - cp0.x, cp1.y - cp0.y, cp1.z - cp0.z);
    Eigen::Vector3d v2 (cp2.x - cp0.x, cp2.y - cp0.y, cp2.z - cp0.z);
    v1.normalize ();
    v2.normalize ();

    double d = v1.dot (v2);

    if (d >= min_curve_th)
    {
      cps.push_back (cp0);
    }

  }

  int order = nurbs.Order ();
  ON_NurbsCurve nurbs_opt = ON_NurbsCurve (2, false, order, int (cps.size ()) + 2 * cp_red);
  nurbs_opt.MakePeriodicUniformKnotVector (1.0 / (double (cps.size ())));
  nurbs_opt.m_knot[cp_red] = 0.0;
  nurbs_opt.m_knot[nurbs_opt.m_knot_capacity - cp_red - 1] = 1.0;

  for (std::size_t j = 0; j < cps.size (); j++)
    nurbs_opt.SetCV (j + cp_red, cps[j]);

  for (int j = 0; j < cp_red; j++)
  {
    ON_3dPoint cp;
    nurbs_opt.GetCV (nurbs_opt.m_cv_count - 1 - cp_red + j, cp);
    nurbs_opt.SetCV (j, cp);

    nurbs_opt.GetCV (cp_red - j, cp);
    nurbs_opt.SetCV (nurbs_opt.m_cv_count - 1 - j, cp);
  }

  return nurbs_opt;

  //  NurbsSolve solver;
  //  solver.assign(nrows, ncp, 2);
  //
  //  for (int i = 0; i < ncp; i++) {
  //    ON_3dPoint cp;
  //    m_nurbs.GetCV(i + cp_red, cp);
  //    solver.x(i, 0, cp.x);
  //    solver.x(i, 1, cp.y);
  //  }
  //
  //  // addCageRegularisation
  //  int row(0);
  //  {
  //    solver.f(row, 0, 0.0);
  //    solver.f(row, 1, 0.0);
  //    for (int j = 1; j < ncp + 1; j++) {
  //      solver.K(row, (j + 0) % ncp, -2.0);
  //      solver.K(row, (j - 1) % ncp, 1.0);
  //      solver.K(row, (j + 1) % ncp, 1.0);
  //      row++;
  //    }
  //  }
  //
  //  Eigen::MatrixXd d = solver.diff();
  //
  //  for (int i = 0; i < ncp; i++) {
  //    double dn = d.row(i).norm();
  //    printf("[FittingCurve2dAPDM::optimize] error: %f\n", dn);
  //    if (dn > max_curve_th)
  //      dbgWin.AddPoint3D(solver.x(i, 0), solver.x(i, 1), 0.0, 0, 0, 255, 10);
  //    if (dn < min_curve_th)
  //      dbgWin.AddPoint3D(solver.x(i, 0), solver.x(i, 1), 0.0, 255, 0, 0, 10);
  //  }
}

void
FittingCurve2dAPDM::addPointConstraint (const double &param, const Eigen::Vector2d &point, double weight, unsigned &row)
{
  int cp_red = m_nurbs.m_order - 2;
  int ncp = m_nurbs.m_cv_count - 2 * cp_red;
  double *N = new double[m_nurbs.m_order * m_nurbs.m_order];

  int E = ON_NurbsSpanIndex (m_nurbs.m_order, m_nurbs.m_cv_count, m_nurbs.m_knot, param, 0, 0);

  ON_EvaluateNurbsBasis (m_nurbs.m_order, m_nurbs.m_knot + E, param, N);

  m_solver.f (row, 0, point (0) * weight);
  m_solver.f (row, 1, point (1) * weight);

  for (int i = 0; i < m_nurbs.m_order; i++)
    m_solver.K (row, (E + i) % ncp, weight * N[i]);

  row++;

  delete[] N;
}

void
FittingCurve2dAPDM::addCageRegularisation (double weight, unsigned &row, const std::vector<double> &elements,
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
      m_solver.f (row, 0, 0.0);
      m_solver.f (row, 1, 0.0);
    }
    else
    {
      int i = j % ncp;

      if (i >= int (m_data->closest_points_error.size () - 1))
      {
        printf ("[FittingCurve2dAPDM::addCageRegularisation] Warning, index for closest_points_error out of bounds\n");
        m_solver.f (row, 0, 0.0);
        m_solver.f (row, 1, 0.0);
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
        m_solver.f (row, 0, err * wConcav * n (0));
        m_solver.f (row, 1, err * wConcav * n (1));

        Eigen::Vector2d p1, p2;
        p1 (0) = pt[0];
        p1 (1) = pt[1];
        p2 = p1 + n * wConcav * err;
        //        m_data->interior_line_start.push_back(p1);
        //        m_data->interior_line_end.push_back(p2);
      }
    }

    m_solver.K (row, (j + 0) % ncp, -2.0 * weight);
    m_solver.K (row, (j - 1) % ncp, 1.0 * weight);
    m_solver.K (row, (j + 1) % ncp, 1.0 * weight);

    row++;
  }
}

//ON_NurbsCurve
//FittingCurve2dAPDM::initCPsNurbsCurve2D (int order, const vector_vec2d &cps)
//{
//  ON_NurbsCurve nurbs;
//  if ((int)cps.size () < (2 * order))
//  {
//    printf ("[FittingCurve2dAPDM::initCPsNurbsCurve2D] Warning, number of control points too low.\n");
//    return nurbs;
//  }
//
//  int cp_red = order - 2;
//  int ncps = cps.size () + cp_red;
//  nurbs = ON_NurbsCurve (2, false, order, ncps);
//  nurbs.MakePeriodicUniformKnotVector (1.0 / (ncps - order + 1));
//
//  for (int j = 0; j < ncps; j++)
//    nurbs.SetCV (j, ON_3dPoint (cps[j] (0), cps[j] (1), 0.0));
//
//  for (int j = 0; j < cp_red; j++)
//  {
//    ON_3dPoint cp;
//    nurbs.GetCV (nurbs.m_cv_count - 1 - cp_red + j, cp);
//    nurbs.SetCV (j, cp);
//
//    nurbs.GetCV (cp_red - j, cp);
//    nurbs.SetCV (nurbs.m_cv_count - 1 - j, cp);
//  }
//
//  return nurbs;
//} // commented 6.6.2012 (thomas.moerwald)
// reason: B-Splines are not closed properly

ON_NurbsCurve
FittingCurve2dAPDM::initCPsNurbsCurve2D (int order, const vector_vec2d &cps)
{
  int cp_red = order - 2;
  ON_NurbsCurve nurbs;
  if (cps.size () < 3 || cps.size () < (2 * static_cast<std::size_t>(cp_red) + 1))
  {
    printf ("[FittingCurve2dAPDM::initCPsNurbsCurve2D] Warning, number of control points too low.\n");
    return nurbs;
  }

  int ncps = int (cps.size ()) + 2 * cp_red; // +2*cp_red for smoothness and +1 for closing
  nurbs = ON_NurbsCurve (2, false, order, ncps);
  nurbs.MakePeriodicUniformKnotVector (1.0 / (ncps - order + 1));

  for (std::size_t j = 0; j < cps.size (); j++)
    nurbs.SetCV (cp_red + j, ON_3dPoint (cps[j] (0), cps[j] (1), 0.0));

  // close nurbs
  nurbs.SetCV (cp_red + int (cps.size ()), ON_3dPoint (cps[0] (0), cps[0] (1), 0.0));

  // make smooth at closing point
  for (int j = 0; j < cp_red; j++)
  {
    ON_3dPoint cp;
    nurbs.GetCV (nurbs.CVCount () - 1 - cp_red + j, cp);
    nurbs.SetCV (j, cp);

    nurbs.GetCV (cp_red - j, cp);
    nurbs.SetCV (nurbs.CVCount () - 1 - j, cp);
  }

  return nurbs;
}

ON_NurbsCurve
FittingCurve2dAPDM::initNurbsCurve2D (int order, const vector_vec2d &data, int ncps, double radiusF)
{
  if (data.empty ())
    printf ("[FittingCurve2dAPDM::initNurbsCurve2D] Warning, no boundary parameters available\n");

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
  r = radiusF * sqrt (r);

  if (ncps < 2 * order)
    ncps = 2 * order;

  ON_NurbsCurve nurbs = ON_NurbsCurve (2, false, order, ncps);
  nurbs.MakePeriodicUniformKnotVector (1.0 / (ncps - order + 1));

  double dcv = (2.0 * M_PI) / (ncps - order + 1);
  Eigen::Vector2d cv;
  for (int j = 0; j < ncps; j++)
  {
    cv (0) = r * sin (dcv * j);
    cv (1) = r * std::cos (dcv * j);
    cv += mean;
    nurbs.SetCV (j, ON_3dPoint (cv (0), cv (1), 0.0));
  }

  return nurbs;
}

//ON_NurbsCurve FittingCurve2dAPDM::initNurbsCurvePCA(int order, const vector_vec2d &data)
//{
//  if (data.empty())
//    printf("[FittingCurve2dAPDM::initNurbsCurvePCA] Warning, no boundary parameters available\n");
//
//  Eigen::Vector3d mean;
//  Eigen::Matrix3d eigenvectors;
//  Eigen::Vector3d eigenvalues;
//
//  unsigned s = data.size();
//
//  NurbsTools::pca(data, mean, eigenvectors, eigenvalues);
//
//  eigenvalues = eigenvalues / s; // seems that the eigenvalues are dependent on the number of points (???)
//
//  double r = 2.0 * sqrt(eigenvalues(0));
//
//  int ncpsV(2 * order);
//  ON_NurbsCurve nurbs = ON_NurbsCurve(3, false, order, ncpsV);
//  nurbs.MakePeriodicUniformKnotVector(1.0 / (ncpsV - order + 1));
//
//  double dcv = (2.0 * M_PI) / (ncpsV - order + 1);
//  Eigen::Vector3d cv, cv_t;
//  for (int j = 0; j < ncpsV; j++) {
//    cv(0) = r * sin(dcv * j);
//    cv(1) = r * std::cos(dcv * j);
//    cv(2) = 0.0;
//    cv_t = eigenvectors * cv + mean;
//    nurbs.SetCV(j, ON_3dPoint(cv_t(0), cv_t(1), cv_t(2)));
//  }
//
//  return nurbs;
//}

std::vector<double>
FittingCurve2dAPDM::getElementVector (const ON_NurbsCurve &nurbs)
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
FittingCurve2dAPDM::assembleInterior (double wInt, double sigma2, double rScale, unsigned &row)
{
  int nInt = int (m_data->interior.size ());
  bool wFunction (true);
  double ds = 1.0 / (2.0 * sigma2);
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
    {
      w = wInt * std::exp (-(error * error) * ds);
    }

    {
      m_data->interior_line_start.push_back (pcp);
      m_data->interior_line_end.push_back (pt);
    }

    //      w = 0.5 * wInt * std::exp(-(error * error) * ds);

    // evaluate if this point is the closest point
    //    int idx = NurbsTools::getClosestPoint(pt, m_data->interior);
    //    if(idx == p)
    //      w = 2.0 * wInt;

    if (w > 1e-6) // avoids ill-conditioned matrix
      addPointConstraint (m_data->interior_param[p], m_data->interior[p], w, row);
    else
    {
      //      m_solver.K(row, 0, 0.0);
      //      row++;
    }
  }
}

//void FittingCurve2dAPDM::assembleCommon(double wCommon, unsigned &row)
//{
//  int nCommon = m_data->common.size();
//  for (int p = 0; p < nCommon; p++) {
//    Eigen::Vector2d &pcp = m_data->common[p];
//
//    // inverse mapping
//    double param;
//    Eigen::Vector2d pt, t;
//    double error;
//    if (p < (int) m_data->common_param.size()) {
//      param = inverseMapping(m_nurbs, pcp, m_data->common_param[p], error, pt, t, in_max_steps, in_accuracy);
//      m_data->common_param[p] = param;
//    } else {
//      param = inverseMapping(m_nurbs, pcp, (double*) NULL, error, pt, t, in_max_steps, in_accuracy);
//      m_data->common_param.push_back(param);
//    }
//
//    addPointConstraint(m_data->common_param[p], pcp, wCommon, row);
//  }
//}

void
FittingCurve2dAPDM::assembleClosestPoints (const std::vector<double> &elements, double weight, double sigma2,
                                           unsigned samples_per_element, unsigned &row)
{
  m_data->closest_points.clear ();
  m_data->closest_points_param.clear ();
  m_data->closest_points_error.clear ();
  //  m_data->interior_line_start.clear();
  //  m_data->interior_line_end.clear();

  if (samples_per_element <= 0)
    samples_per_element = 1;

  double ds = 1.0 / (2.0 * sigma2);
  double seg = 1.0 / (samples_per_element + 1);

  for (std::size_t i = 0; i < elements.size (); i++)
  {
    std::size_t k = i % elements.size ();
    double xi0 = elements[i];
    double dxi = elements[k] - xi0;

    for (unsigned j = 0; j < samples_per_element; j++)
    {
      double xi = xi0 + (seg * (j + 1)) * dxi;

      double points[4];
      Eigen::Vector2d p1, p2, p3, t, in;
      m_nurbs.Evaluate (xi, 1, 2, points);
      p1 (0) = points[0];
      p1 (1) = points[1];
      t (0) = points[2];
      t (1) = points[3];
      in (0) = t (1);
      in (1) = -t (0);
      in.normalize ();

      unsigned idxcp;
      unsigned idx = NurbsTools::getClosestPoint (p1, in, m_data->interior, idxcp);
      p2 = m_data->interior[idx];
      p3 = m_data->interior[idxcp];

      //    double xi2 = m_data->interior_param[idx];

      double error2_2 = (p2 - p1).squaredNorm ();
      double error2_3 = (p3 - p1).squaredNorm ();

      m_data->closest_points.push_back (p3);
      m_data->closest_points_param.push_back (xi);
      m_data->closest_points_error.push_back (error2_3);

      double w (weight);
      w = 0.5 * weight * std::exp (-(error2_2) * ds);
      //    w = weight * std::fabs(in.dot(p2-p1));

      //    if (weight > 0.0 && (std::fabs(xi2 - xi) < std::fabs(dxi)))
      if (w > 0.0)
      {
        addPointConstraint (xi, p2, w, row);
        //      m_data->interior_line_start.push_back(p1);
        //      m_data->interior_line_end.push_back(p2);
      }
    }
  }
}

double
FittingCurve2dAPDM::inverseMapping (const ON_NurbsCurve &nurbs, const Eigen::Vector2d &pt, const double &hint,
                                    double &error, Eigen::Vector2d &p, Eigen::Vector2d &t, double rScale, int maxSteps,
                                    double accuracy, bool quiet)
{
  if (nurbs.Order () == 2)
    return inverseMappingO2 (nurbs, pt, error, p, t);

  //int cp_red = (nurbs.m_order - 2);
  //int ncpj = (int (nurbs.m_cv_count) - 2 * cp_red);
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

    //    e = 0.5 * std::abs<double> (e);
    //    if (delta > e)
    //      delta = e;
    //    if (delta < -e)
    //      delta = -e;

    if (std::abs (delta) < accuracy)
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
    printf ("[FittingCurve2dAPDM::inverseMapping] Warning: Method did not converge (%e %d).\n", accuracy, maxSteps);
    printf ("[FittingCurve2dAPDM::inverseMapping] hint: %f current: %f delta: %f error: %f\n", hint, current, delta,
            error);
  }

  return current;
}

double
FittingCurve2dAPDM::inverseMappingO2 (const ON_NurbsCurve &nurbs, const Eigen::Vector2d &pt, double &error,
                                      Eigen::Vector2d &p, Eigen::Vector2d &t)
{
  if (nurbs.Order () != 2)
    printf ("[FittingCurve2dAPDM::inverseMappingO2] Error, order not 2 (polynomial degree 1)\n");

  std::vector<double> elements = getElementVector (nurbs);

  Eigen::Vector2d min_pt;
  double min_param (std::numeric_limits<double>::max());
  double min_dist (std::numeric_limits<double>::max());
  error = std::numeric_limits<double>::max();
  int is_corner (-1);

  for (std::size_t i = 0; i < elements.size () - 1; i++)
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
    if (is_corner == 0 || is_corner == static_cast<int>(elements.size ()) - 1)
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
//FittingCurve2dAPDM::inverseMapping (const ON_NurbsCurve &nurbs, const Eigen::Vector2d &pt, double* phint, double &error,
//                                Eigen::Vector2d &p, Eigen::Vector2d &t, int maxSteps, double accuracy, bool quiet)
//{
//  double hint;
//  Eigen::Vector2d r;
//  std::vector<double> elements = getElementVector (nurbs);
//  double points[2];
//
//  if (phint == NULL)
//  {
//    double d_shortest (std::numeric_limits<double>::max());
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
FittingCurve2dAPDM::findClosestElementMidPoint (const ON_NurbsCurve &nurbs, const Eigen::Vector2d &pt, double hint)
{
  // evaluate hint
  double param = hint;
  double points[2];
  nurbs.Evaluate (param, 0, 2, points);
  Eigen::Vector2d p (points[0], points[1]);
  Eigen::Vector2d r = p - pt;

  double d_shortest_hint = r.squaredNorm ();
  double d_shortest_elem (std::numeric_limits<double>::max());

  // evaluate elements
  std::vector<double> elements = pcl::on_nurbs::FittingCurve2dAPDM::getElementVector (nurbs);
  double seg = 1.0 / (nurbs.Order () - 1);

  for (std::size_t i = 0; i < elements.size () - 1; i++)
  {
    double &xi0 = elements[i];
    double &xi1 = elements[i + 1];
    double dxi = xi1 - xi0;

    for (std::size_t j = 0; j < static_cast<std::size_t>(nurbs.Order ()); j++)
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

  return param;
}

double
FittingCurve2dAPDM::findClosestElementMidPoint (const ON_NurbsCurve &nurbs, const Eigen::Vector2d &pt)
{
  double param (0.0);
  Eigen::Vector2d p, r;
  std::vector<double> elements = pcl::on_nurbs::FittingCurve2dAPDM::getElementVector (nurbs);
  double points[2];

  double d_shortest (std::numeric_limits<double>::max());
  double seg = 1.0 / (nurbs.Order () - 1);

  for (std::size_t i = 0; i < elements.size () - 1; i++)
  {
    double &xi0 = elements[i];
    double &xi1 = elements[i + 1];
    double dxi = xi1 - xi0;

    for (std::size_t j = 0; j < static_cast<std::size_t>(nurbs.Order ()); j++)
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

