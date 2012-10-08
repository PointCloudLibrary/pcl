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
#include <pcl/surface/on_nurbs/fitting_surface_tdm.h>

using namespace pcl;
using namespace on_nurbs;
using namespace Eigen;

FittingSurfaceTDM::FittingSurfaceTDM (NurbsDataSurface *data, const ON_NurbsSurface &ns) :
  FittingSurface (data, ns)
{
}
FittingSurfaceTDM::FittingSurfaceTDM (int order, NurbsDataSurface *data, Eigen::Vector3d z) :
  FittingSurface (order, data, z)
{
}

void
FittingSurfaceTDM::assemble (ParameterTDM param)
{
  int nBnd = static_cast<int> (m_data->boundary.size ());
  int nInt = static_cast<int> (m_data->interior.size ());
  int nCurInt = param.regularisation_resU * param.regularisation_resV;
  int nCurBnd = 2 * param.regularisation_resU + 2 * param.regularisation_resV;
  int nCageReg = (m_nurbs.CVCount (0) - 2) * (m_nurbs.CVCount (1) - 2);
  int nCageRegBnd = 2 * (m_nurbs.CVCount (0) - 1) + 2 * (m_nurbs.CVCount (1) - 1);

  if (param.boundary_weight <= 0.0)
    nBnd = 0;
  if (param.interior_weight <= 0.0)
    nInt = 0;
  if (param.boundary_regularisation <= 0.0)
    nCurBnd = 0;
  if (param.interior_regularisation <= 0.0)
    nCurInt = 0;
  if (param.interior_smoothness <= 0.0)
    nCageReg = 0;
  if (param.boundary_smoothness <= 0.0)
    nCageRegBnd = 0;

  int ncp = m_nurbs.CVCount (0) * m_nurbs.CVCount (1);
  int nrows = nBnd + nInt + nCurInt + nCurBnd + nCageReg + nCageRegBnd;

  m_solver.assign (3 * nrows, 3 * ncp, 1);

  unsigned row = 0;

  // boundary points should lie on edges of surface
  if (nBnd > 0)
    assembleBoundary (param.boundary_weight, param.boundary_tangent_weight, row);

  // interior points should lie on surface
  if (nInt > 0)
    assembleInterior (param.interior_weight, param.interior_tangent_weight, row);

  // cage regularisation
  if (nCageReg > 0)
    addCageInteriorRegularisation (param.interior_smoothness, row);

  if (nCageRegBnd > 0)
  {
    addCageBoundaryRegularisation (param.boundary_smoothness, NORTH, row);
    addCageBoundaryRegularisation (param.boundary_smoothness, SOUTH, row);
    addCageBoundaryRegularisation (param.boundary_smoothness, WEST, row);
    addCageBoundaryRegularisation (param.boundary_smoothness, EAST, row);
    addCageCornerRegularisation (param.boundary_smoothness * 2.0, row);
  }
}

void
FittingSurfaceTDM::solve (double damp)
{
  if (m_solver.solve ())
    updateSurf (damp);
}

void
FittingSurfaceTDM::updateSurf (double damp)
{
  int ncp = m_nurbs.CVCount (0) * m_nurbs.CVCount (1);

  for (int A = 0; A < ncp; A++)
  {

    int I = gl2gr (A);
    int J = gl2gc (A);

    ON_3dPoint cp_prev;
    m_nurbs.GetCV (I, J, cp_prev);

    ON_3dPoint cp;
    cp.x = cp_prev.x + damp * (m_solver.x (3 * A + 0, 0) - cp_prev.x);
    cp.y = cp_prev.y + damp * (m_solver.x (3 * A + 1, 0) - cp_prev.y);
    cp.z = cp_prev.z + damp * (m_solver.x (3 * A + 2, 0) - cp_prev.z);

    m_nurbs.SetCV (I, J, cp);

  }

}

void
FittingSurfaceTDM::assembleInterior (double wInt, double wTangent, unsigned &row)
{
  m_data->interior_line_start.clear ();
  m_data->interior_line_end.clear ();
  m_data->interior_error.clear ();
  m_data->interior_normals.clear ();
  unsigned nInt = static_cast<unsigned> (m_data->interior.size ());
  for (unsigned p = 0; p < nInt; p++)
  {
    Vector3d &pcp = m_data->interior[p];

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
      params = FittingSurface::findClosestElementMidPoint (m_nurbs, pcp);
      params = inverseMapping (m_nurbs, pcp, params, error, pt, tu, tv, in_max_steps, in_accuracy);
      m_data->interior_param.push_back (params);
    }
    m_data->interior_error.push_back (error);

    tu.normalize ();
    tv.normalize ();
    n = tu.cross (tv);

    m_data->interior_normals.push_back (n);
    m_data->interior_line_start.push_back (pcp);
    m_data->interior_line_end.push_back (pt);

    double w (wInt);
    if (p < m_data->interior_weight.size ())
      w = m_data->interior_weight[p];

    addPointConstraint (m_data->interior_param[p], m_data->interior[p], n, tu, tv, wTangent, w, row);
  }
}

void
FittingSurfaceTDM::assembleBoundary (double wBnd, double wTangent, unsigned &row)
{
  m_data->boundary_line_start.clear ();
  m_data->boundary_line_end.clear ();
  m_data->boundary_error.clear ();
  m_data->boundary_normals.clear ();
  unsigned nBnd = static_cast<unsigned> (m_data->boundary.size ());
  for (unsigned p = 0; p < nBnd; p++)
  {
    Vector3d &pcp = m_data->boundary[p];

    double error;
    Vector3d pt, tu, tv, n;
    Vector2d params = inverseMappingBoundary (m_nurbs, pcp, error, pt, tu, tv, in_max_steps, in_accuracy);
    m_data->boundary_error.push_back (error);

    if (p < m_data->boundary_param.size ())
    {
      m_data->boundary_param[p] = params;
    }
    else
    {
      m_data->boundary_param.push_back (params);
    }

    tu.normalize ();
    tv.normalize ();
    n = tu.cross (tv);

    m_data->boundary_normals.push_back (n);
    m_data->boundary_line_start.push_back (pcp);
    m_data->boundary_line_end.push_back (pt);

    double w (wBnd);
    if (p < m_data->boundary_weight.size ())
      w = m_data->boundary_weight[p];

    addPointConstraint (m_data->boundary_param[p], m_data->boundary[p], n, tu, tv, wTangent, w, row);
  }
}

void
FittingSurfaceTDM::addPointConstraint (const Eigen::Vector2d &params, const Eigen::Vector3d &p,
                                       const Eigen::Vector3d &n, const Eigen::Vector3d &tu, const Eigen::Vector3d &tv,
                                       double tangent_weight, double weight, unsigned &row)
{
  double *N0 = new double[m_nurbs.Order (0) * m_nurbs.Order (0)];
  double *N1 = new double[m_nurbs.Order (1) * m_nurbs.Order (1)];

  int E = ON_NurbsSpanIndex (m_nurbs.Order(0), m_nurbs.CVCount (0), m_nurbs.m_knot[0], params (0), 0, 0);
  int F = ON_NurbsSpanIndex (m_nurbs.Order(1), m_nurbs.CVCount (1), m_nurbs.m_knot[1], params (1), 0, 0);

  ON_EvaluateNurbsBasis (m_nurbs.Order (0), m_nurbs.m_knot[0] + E, params (0), N0);
  ON_EvaluateNurbsBasis (m_nurbs.Order (1), m_nurbs.m_knot[1] + F, params (1), N1);

  double wt = tangent_weight;
  Eigen::Vector3d td = n + wt * tu + wt * tv;

  m_solver.f (row + 0, 0, td (0) * p (0) * weight);
  m_solver.f (row + 1, 0, td (1) * p (1) * weight);
  m_solver.f (row + 2, 0, td (2) * p (2) * weight);

  for (int i = 0; i < m_nurbs.Order (0); i++)
  {
    for (int j = 0; j < m_nurbs.Order (1); j++)
    {

      double val = N0[i] * N1[j] * weight;
      unsigned ci = 3 * lrc2gl (E, F, i, j);
      m_solver.K (row + 0, ci + 0, td (0) * val);
      m_solver.K (row + 1, ci + 1, td (1) * val);
      m_solver.K (row + 2, ci + 2, td (2) * val);

    } // j
  } // i

  row += 3;

  delete [] N1;
  delete [] N0;
}

void
FittingSurfaceTDM::addCageInteriorRegularisation (double weight, unsigned &row)
{
  for (int i = 1; i < (m_nurbs.CVCount (0) - 1); i++)
  {
    for (int j = 1; j < (m_nurbs.CVCount (1) - 1); j++)
    {

      //      m_solver.f(row + 0, 0, 0.0);
      //      m_solver.f(row + 1, 0, 0.0);
      //      m_solver.f(row + 2, 0, 0.0);

      m_solver.K (row + 0, 3 * grc2gl (i + 0, j + 0) + 0, -4.0 * weight);
      m_solver.K (row + 0, 3 * grc2gl (i + 0, j - 1) + 0, 1.0 * weight);
      m_solver.K (row + 0, 3 * grc2gl (i + 0, j + 1) + 0, 1.0 * weight);
      m_solver.K (row + 0, 3 * grc2gl (i - 1, j + 0) + 0, 1.0 * weight);
      m_solver.K (row + 0, 3 * grc2gl (i + 1, j + 0) + 0, 1.0 * weight);

      m_solver.K (row + 1, 3 * grc2gl (i + 0, j + 0) + 1, -4.0 * weight);
      m_solver.K (row + 1, 3 * grc2gl (i + 0, j - 1) + 1, 1.0 * weight);
      m_solver.K (row + 1, 3 * grc2gl (i + 0, j + 1) + 1, 1.0 * weight);
      m_solver.K (row + 1, 3 * grc2gl (i - 1, j + 0) + 1, 1.0 * weight);
      m_solver.K (row + 1, 3 * grc2gl (i + 1, j + 0) + 1, 1.0 * weight);

      m_solver.K (row + 2, 3 * grc2gl (i + 0, j + 0) + 2, -4.0 * weight);
      m_solver.K (row + 2, 3 * grc2gl (i + 0, j - 1) + 2, 1.0 * weight);
      m_solver.K (row + 2, 3 * grc2gl (i + 0, j + 1) + 2, 1.0 * weight);
      m_solver.K (row + 2, 3 * grc2gl (i - 1, j + 0) + 2, 1.0 * weight);
      m_solver.K (row + 2, 3 * grc2gl (i + 1, j + 0) + 2, 1.0 * weight);

      row += 3;
    }
  }
}

void
FittingSurfaceTDM::addCageBoundaryRegularisation (double weight, int side, unsigned &row)
{
  int i = 0;
  int j = 0;

  switch (side)
  {
    case SOUTH:
      j = m_nurbs.CVCount (1) - 1;
    case NORTH:
      for (i = 1; i < (m_nurbs.CVCount (0) - 1); i++)
      {

        //      m_solver.f(row + 0, 0, 0.0);
        //      m_solver.f(row + 1, 0, 0.0);
        //      m_solver.f(row + 2, 0, 0.0);

        m_solver.K (row + 0, 3 * grc2gl (i + 0, j) + 0, -2.0 * weight);
        m_solver.K (row + 0, 3 * grc2gl (i - 1, j) + 0, 1.0 * weight);
        m_solver.K (row + 0, 3 * grc2gl (i + 1, j) + 0, 1.0 * weight);

        m_solver.K (row + 1, 3 * grc2gl (i + 0, j) + 1, -2.0 * weight);
        m_solver.K (row + 1, 3 * grc2gl (i - 1, j) + 1, 1.0 * weight);
        m_solver.K (row + 1, 3 * grc2gl (i + 1, j) + 1, 1.0 * weight);

        m_solver.K (row + 2, 3 * grc2gl (i + 0, j) + 2, -2.0 * weight);
        m_solver.K (row + 2, 3 * grc2gl (i - 1, j) + 2, 1.0 * weight);
        m_solver.K (row + 2, 3 * grc2gl (i + 1, j) + 2, 1.0 * weight);

        row += 3;
      }
      break;

    case EAST:
      i = m_nurbs.CVCount (0) - 1;
    case WEST:
      for (j = 1; j < (m_nurbs.CVCount (1) - 1); j++)
      {

        //      m_solver.f(row + 0, 0, 0.0);
        //      m_solver.f(row + 1, 0, 0.0);
        //      m_solver.f(row + 2, 0, 0.0);

        m_solver.K (row + 0, 3 * grc2gl (i, j + 0) + 0, -2.0 * weight);
        m_solver.K (row + 0, 3 * grc2gl (i, j - 1) + 0, 1.0 * weight);
        m_solver.K (row + 0, 3 * grc2gl (i, j + 1) + 0, 1.0 * weight);

        m_solver.K (row + 1, 3 * grc2gl (i, j + 0) + 1, -2.0 * weight);
        m_solver.K (row + 1, 3 * grc2gl (i, j - 1) + 1, 1.0 * weight);
        m_solver.K (row + 1, 3 * grc2gl (i, j + 1) + 1, 1.0 * weight);

        m_solver.K (row + 2, 3 * grc2gl (i, j + 0) + 2, -2.0 * weight);
        m_solver.K (row + 2, 3 * grc2gl (i, j - 1) + 2, 1.0 * weight);
        m_solver.K (row + 2, 3 * grc2gl (i, j + 1) + 2, 1.0 * weight);

        row += 3;
      }
      break;
  }
}

void
FittingSurfaceTDM::addCageCornerRegularisation (double weight, unsigned &row)
{
  { // NORTH-WEST
    int i = 0;
    int j = 0;

    //    m_solver.f(row + 0, 0, 0.0);
    //    m_solver.f(row + 1, 0, 0.0);
    //    m_solver.f(row + 2, 0, 0.0);

    m_solver.K (row + 0, 3 * grc2gl (i + 0, j + 0) + 0, -2.0 * weight);
    m_solver.K (row + 0, 3 * grc2gl (i + 1, j + 0) + 0, 1.0 * weight);
    m_solver.K (row + 0, 3 * grc2gl (i + 0, j + 1) + 0, 1.0 * weight);

    m_solver.K (row + 1, 3 * grc2gl (i + 0, j + 0) + 1, -2.0 * weight);
    m_solver.K (row + 1, 3 * grc2gl (i + 1, j + 0) + 1, 1.0 * weight);
    m_solver.K (row + 1, 3 * grc2gl (i + 0, j + 1) + 1, 1.0 * weight);

    m_solver.K (row + 2, 3 * grc2gl (i + 0, j + 0) + 2, -2.0 * weight);
    m_solver.K (row + 2, 3 * grc2gl (i + 1, j + 0) + 2, 1.0 * weight);
    m_solver.K (row + 2, 3 * grc2gl (i + 0, j + 1) + 2, 1.0 * weight);

    row += 3;

  }

  { // NORTH-EAST
    int i = m_nurbs.CVCount (0) - 1;
    int j = 0;

    //    m_solver.f(row + 0, 0, 0.0);
    //    m_solver.f(row + 1, 0, 0.0);
    //    m_solver.f(row + 2, 0, 0.0);

    m_solver.K (row + 0, 3 * grc2gl (i + 0, j + 0) + 0, -2.0 * weight);
    m_solver.K (row + 0, 3 * grc2gl (i - 1, j + 0) + 0, 1.0 * weight);
    m_solver.K (row + 0, 3 * grc2gl (i + 0, j + 1) + 0, 1.0 * weight);

    m_solver.K (row + 1, 3 * grc2gl (i + 0, j + 0) + 1, -2.0 * weight);
    m_solver.K (row + 1, 3 * grc2gl (i - 1, j + 0) + 1, 1.0 * weight);
    m_solver.K (row + 1, 3 * grc2gl (i + 0, j + 1) + 1, 1.0 * weight);

    m_solver.K (row + 2, 3 * grc2gl (i + 0, j + 0) + 2, -2.0 * weight);
    m_solver.K (row + 2, 3 * grc2gl (i - 1, j + 0) + 2, 1.0 * weight);
    m_solver.K (row + 2, 3 * grc2gl (i + 0, j + 1) + 2, 1.0 * weight);

    row += 3;
  }

  { // SOUTH-EAST
    int i = m_nurbs.CVCount (0) - 1;
    int j = m_nurbs.CVCount (1) - 1;

    //    m_solver.f(row + 0, 0, 0.0);
    //    m_solver.f(row + 1, 0, 0.0);
    //    m_solver.f(row + 2, 0, 0.0);

    m_solver.K (row + 0, 3 * grc2gl (i + 0, j + 0) + 0, -2.0 * weight);
    m_solver.K (row + 0, 3 * grc2gl (i - 1, j + 0) + 0, 1.0 * weight);
    m_solver.K (row + 0, 3 * grc2gl (i + 0, j - 1) + 0, 1.0 * weight);

    m_solver.K (row + 1, 3 * grc2gl (i + 0, j + 0) + 1, -2.0 * weight);
    m_solver.K (row + 1, 3 * grc2gl (i - 1, j + 0) + 1, 1.0 * weight);
    m_solver.K (row + 1, 3 * grc2gl (i + 0, j - 1) + 1, 1.0 * weight);

    m_solver.K (row + 2, 3 * grc2gl (i + 0, j + 0) + 2, -2.0 * weight);
    m_solver.K (row + 2, 3 * grc2gl (i - 1, j + 0) + 2, 1.0 * weight);
    m_solver.K (row + 2, 3 * grc2gl (i + 0, j - 1) + 2, 1.0 * weight);

    row += 3;
  }

  { // SOUTH-WEST
    int i = 0;
    int j = m_nurbs.CVCount (1) - 1;

    //    m_solver.f(row + 0, 0, 0.0);
    //    m_solver.f(row + 1, 0, 0.0);
    //    m_solver.f(row + 2, 0, 0.0);

    m_solver.K (row + 0, 3 * grc2gl (i + 0, j + 0) + 0, -2.0 * weight);
    m_solver.K (row + 0, 3 * grc2gl (i + 1, j + 0) + 0, 1.0 * weight);
    m_solver.K (row + 0, 3 * grc2gl (i + 0, j - 1) + 0, 1.0 * weight);

    m_solver.K (row + 1, 3 * grc2gl (i + 0, j + 0) + 1, -2.0 * weight);
    m_solver.K (row + 1, 3 * grc2gl (i + 1, j + 0) + 1, 1.0 * weight);
    m_solver.K (row + 1, 3 * grc2gl (i + 0, j - 1) + 1, 1.0 * weight);

    m_solver.K (row + 2, 3 * grc2gl (i + 0, j + 0) + 2, -2.0 * weight);
    m_solver.K (row + 2, 3 * grc2gl (i + 1, j + 0), 1.0 * weight);
    m_solver.K (row + 2, 3 * grc2gl (i + 0, j - 1) + 2 + 2, 1.0 * weight);

    row += 3;
  }

}
