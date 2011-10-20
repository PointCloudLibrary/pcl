/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Thomas Mörwald, Jonathan Balzer, Inc.
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
 *   * Neither the name of Thomas Mörwald or Jonathan Balzer nor the names of its
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
 * @author thomas.moerwald
 *
 */

#include <map>
#include <stdexcept>
#include "pcl/surface/nurbs/NurbsFitting.h"

using namespace nurbsfitting;
using namespace Eigen;

NurbsFitting::NurbsFitting (int order, NurbsData *data, ON_3dPoint ll, ON_3dPoint lr, ON_3dPoint ur, ON_3dPoint ul)
{
  if (order < 2)
    throw std::runtime_error ("[NurbsFitting::NurbsFitting] Error order to low (order<2).");

  ON::Begin ();

  this->data = data;

  m_patch = new ON_NurbsSurface (3, false, order, order, order, order);
  m_patch->MakeClampedUniformKnotVector (0, 1.0);
  m_patch->MakeClampedUniformKnotVector (1, 1.0);

  std::map<int, std::map<int, ON_3dPoint> > cv_map;

  double dc = 1.0 / (order - 1);

  for (int i = 0; i < order; i++)
  {

    double di = dc * i;
    cv_map[i][0] = ll + (lr - ll) * di;
    cv_map[0][i] = ll + (ul - ll) * di;
    cv_map[i][order - 1] = ul + (ur - ul) * di;
    cv_map[order - 1][i] = lr + (ur - lr) * di;

  } // i

  for (int i = 1; i < order - 1; i++)
  {
    for (int j = 1; j < order - 1; j++)
    {

      ON_3dPoint du = cv_map[0][j] + (cv_map[order - 1][j] - cv_map[0][j]) * dc * j;
      ON_3dPoint dv = cv_map[i][0] + (cv_map[i][order - 1] - cv_map[i][0]) * dc * i;
      cv_map[i][j] = du * 0.5 + dv * 0.5;

    } // j
  } // i

  for (int i = 0; i < order; i++)
  {
    for (int j = 0; j < order; j++)
    {

      m_patch->SetCV (i, j, cv_map[i][j]);

    } // j
  } // i

  //	m_patch->Dump(m_out);
  //	printf("closed: %d %d\n", m_patch->IsClosed(0), m_patch->IsClosed(1));
  //	printf("periodic: %d %d\n", m_patch->IsPeriodic(0), m_patch->IsPeriodic(1));

  this->init ();
}

NurbsFitting::NurbsFitting (int order, NurbsData *data, const std::vector<ON_3dPoint> &cv)
{
  if (order < 2)
    throw std::runtime_error ("[NurbsFitting::NurbsFitting] Error order to low (order<2).");

  if (order * order != cv.size ())
    throw std::runtime_error (
                              "[NurbsFitting::NurbsFitting] Error number of control points don't match: (order*order != cps.size()).\n");

  ON::Begin ();

  this->data = data;

  m_patch = new ON_NurbsSurface (3, false, order, order, order, order);
  m_patch->MakeClampedUniformKnotVector (0, 1.0);
  m_patch->MakeClampedUniformKnotVector (1, 1.0);

  unsigned cvi (0);
  for (int i = 0; i < order; i++)
  {
    for (int j = 0; j < order; j++)
    {

      m_patch->SetCV (i, j, cv[cvi++]);

    } // j
  } // i

  //	m_patch->Dump(m_out);
  //	printf("closed: %d %d\n", m_patch->IsClosed(0), m_patch->IsClosed(1));
  //	printf("periodic: %d %d\n", m_patch->IsPeriodic(0), m_patch->IsPeriodic(1));

  this->init ();
}

NurbsFitting::NurbsFitting (int order, NurbsData *data)
{
  if (order < 2)
    throw std::runtime_error ("[NurbsFitting::NurbsFitting] Error order to low (order<2).");

  ON::Begin ();

  this->data = data;

  double m_min[3];
  double m_max[3];
  double m_mid[3];

  data->boundary.GetBBox (m_min, m_max);

  m_mid[0] = m_min[0] + (m_max[0] - m_min[0]) * 0.5;
  m_mid[1] = m_min[1] + (m_max[1] - m_min[1]) * 0.5;
  m_mid[2] = m_min[2] + (m_max[2] - m_min[2]) * 0.5;

  m_patch = new ON_NurbsSurface (3, false, order, order, order, order);
  m_patch->MakeClampedUniformKnotVector (0, 1.0);
  m_patch->MakeClampedUniformKnotVector (1, 1.0);

  double dcu = (m_max[0] - m_min[0]) / (order - 1); // image sizes are power of two
  double dcv = (m_max[1] - m_min[1]) / (order - 1);

  ON_3dPoint cv (0.0, 0.0, 0.0);
  for (int i = 0; i < order; i++)
  {
    for (int j = 0; j < order; j++)
    {

      cv.x = m_min[0] + dcu * i;
      cv.y = m_min[1] + dcv * j;
      cv.z = m_mid[2];
      m_patch->SetCV (i, j, cv);

    } // j
  } // i

  //	m_patch->Dump(m_out);
  //	printf("closed: %d %d\n", m_patch->IsClosed(0), m_patch->IsClosed(1));
  //	printf("periodic: %d %d\n", m_patch->IsPeriodic(0), m_patch->IsPeriodic(1));

  this->init ();
}

NurbsFitting::NurbsFitting (NurbsData *data, const ON_NurbsSurface &ns)
{
  ON::Begin ();

  this->data = data;

  this->init ();
}

NurbsFitting::~NurbsFitting ()
{

  delete (m_patch);

  ON::End ();

}

void
NurbsFitting::refine (int dim)
{

  NurbsTools ntools (m_patch);

  std::vector<double> xi;

  std::vector<double> elements = ntools.getElementVector (dim);

  for (unsigned i = 0; i < elements.size () - 1; i++)
  {

    xi.push_back (elements[i] + 0.5 * (elements[i + 1] - elements[i]));

  } // i

  for (unsigned i = 0; i < xi.size (); i++)
  {

    m_patch->InsertKnot (dim, xi[i], 1);

  } // i

  m_elementsU = ntools.getElementVector (0);
  m_elementsV = ntools.getElementVector (1);
  m_minU = m_elementsU[0];
  m_minV = m_elementsV[0];
  m_maxU = m_elementsU[m_elementsU.size () - 1];
  m_maxV = m_elementsV[m_elementsV.size () - 1];

  m_xeig = Eigen::MatrixXd::Zero (m_patch->m_cv_count[0] * m_patch->m_cv_count[1], 3);

  //	m_patch->Dump(m_out);

}

void
NurbsFitting::init ()
{
  NurbsTools ntools (m_patch);
  m_elementsU = ntools.getElementVector (0);
  m_elementsV = ntools.getElementVector (1);
  m_minU = m_elementsU[0];
  m_minV = m_elementsV[0];
  m_maxU = m_elementsU[m_elementsU.size () - 1];
  m_maxV = m_elementsV[m_elementsV.size () - 1];

  m_xeig = Eigen::MatrixXd::Zero (m_patch->m_cv_count[0] * m_patch->m_cv_count[1], 3);

  invMapBnd_maxSteps = 100;
  invMapInt_maxSteps = 100;
  invMapBnd_accuracy = 1e-3;
  invMapInt_accuracy = 1e-3;

  m_quiet = true;
  use_int_hints = false;
}

void
NurbsFitting::assemble ()
{
  clock_t time_start, time_end;
  time_start = clock ();

  NurbsTools ntools (m_patch);

  int ncp = m_patch->m_cv_count[0] * m_patch->m_cv_count[1];
  int nInt = data->interior.PointCount ();
  int nCageRegInt = (m_patch->m_cv_count[0] - 2) * (m_patch->m_cv_count[1] - 2);
  int nCageRegBnd = 2 * (m_patch->m_cv_count[0] - 1) + 2 * (m_patch->m_cv_count[1] - 1);

  double wInt = 1.0;
  double wCageRegInt = 0.00001;
  double wCageRegBnd = 0.00001;

  int nrows = nInt + nCageRegInt + nCageRegBnd;

  m_Keig = Eigen::MatrixXd::Zero (nrows, ncp);
  m_feig = Eigen::MatrixXd::Zero (nrows, 3);

  int row (0);

  // interior points should lie on surface
  data->interior_line_start.clear ();
  data->interior_line_end.clear ();
  data->interior_error.clear ();
  for (int p = 0; p < nInt; p++)
  {
    Vector3d pcp;
    pcp (0) = data->interior[p].x;
    pcp (1) = data->interior[p].y;
    pcp (2) = data->interior[p].z;

    // inverse mapping
    Vector2d params;
    double error;
    if (p < (int)data->interior_param.size ())
    {
      params = ntools.inverseMapping (pcp, &data->interior_param[p], error, invMapInt_maxSteps, invMapInt_accuracy);
      data->interior_param[p] = params;
    }
    else
    {
      params = ntools.inverseMapping (pcp, NULL, error, invMapInt_maxSteps, invMapInt_accuracy);
      data->interior_param.push_back (params);
    }
    data->interior_error.push_back (error);

    double pointAndTangents[9];
    m_patch->Evaluate (params (0), params (1), 1, 3, pointAndTangents);
    data->interior_line_start.push_back (pcp);
    Vector3d r;
    r (0) = pointAndTangents[0];
    r (1) = pointAndTangents[1];
    r (2) = pointAndTangents[2];
    data->interior_line_end.push_back (r);

    addPointConstraint (params, pcp, wInt, row);
  } // p

  // cage regularisation
  if (nCageRegInt > 0)
    addCageInteriorRegularisation (wCageRegInt, row);

  if (nCageRegBnd > 0)
  {
    addCageBoundaryRegularisation (wCageRegBnd, NORTH, row);
    addCageBoundaryRegularisation (wCageRegBnd, SOUTH, row);
    addCageBoundaryRegularisation (wCageRegBnd, WEST, row);
    addCageBoundaryRegularisation (wCageRegBnd, EAST, row);
    addCageCornerRegularisation (wCageRegBnd * 4.0, row);
  }

  time_end = clock ();
  if (!m_quiet)
  {
    double solve_time = (double)(time_end - time_start) / (double)(CLOCKS_PER_SEC);
    printf ("[NurbsFitting::assemble()] (assemble (%d,%d): %f sec)\n", nrows, ncp, solve_time);
  }

}

void
NurbsFitting::assemble (std::vector<double> &wBnd, std::vector<double> &wInt, double wCageRegBnd, double wCageRegInt)
{
  if (wBnd.size () > data->boundary.PointCount ())
    throw std::runtime_error ("[NurbsFitting::assemble] Size of weight vector greater than point size (boundary).");

  if (wInt.size () > data->interior.PointCount ())
    throw std::runtime_error ("[NurbsFitting::assemble] Size of weight vector greater than point size (interior).");

  NurbsTools ntools (m_patch);

  clock_t time_start, time_end;
  time_start = clock ();

  int nBnd = wBnd.size ();
  int nInt = wInt.size ();
  int nCageRegInt = (m_patch->m_cv_count[0] - 2) * (m_patch->m_cv_count[1] - 2);
  int nCageRegBnd = 2 * (m_patch->m_cv_count[0] - 1) + 2 * (m_patch->m_cv_count[1] - 1);

  if (wCageRegInt <= 0.0)
    nCageRegInt = 0;
  if (wCageRegBnd <= 0.0)
    nCageRegBnd = 0;

  int ncp = m_patch->m_cv_count[0] * m_patch->m_cv_count[1];
  int nrows = nBnd + nInt + nCageRegInt + nCageRegBnd;

  m_Keig = Eigen::MatrixXd::Zero (nrows, ncp);
  m_feig = Eigen::MatrixXd::Zero (nrows, 3);

  int row = 0;

  // boundary points should lie on edges of surface
  data->boundary_line_start.clear ();
  data->boundary_line_end.clear ();
  data->boundary_error.clear ();
  for (int p = 0; p < nBnd; p++)
  {
    Vector3d pcp;
    pcp (0) = data->boundary[p].x;
    pcp (1) = data->boundary[p].y;
    pcp (2) = data->boundary[p].z;

    double error;
    Vector2d params = ntools.inverseMappingBoundary (pcp, error, invMapBnd_maxSteps, invMapBnd_accuracy);
    data->boundary_error.push_back (error);

    if (p < (int)data->boundary_param.size ())
    {
      data->boundary_param[p] = params;
    }
    else
    {
      data->boundary_param.push_back (params);
    }

    double pointAndTangents[9];
    m_patch->Evaluate (params (0), params (1), 1, 3, pointAndTangents);
    Vector3d r;
    r (0) = pointAndTangents[0];
    r (1) = pointAndTangents[1];
    r (2) = pointAndTangents[2];
    data->boundary_line_start.push_back (pcp);
    data->boundary_line_end.push_back (r);

    addPointConstraint (params, pcp, wBnd[p], row);
  } // p

  // interior points should lie on surface
  data->interior_line_start.clear ();
  data->interior_line_end.clear ();
  data->interior_error.clear ();
  for (int p = 0; p < nInt; p++)
  {
    Vector3d pcp;
    pcp (0) = data->interior[p].x;
    pcp (1) = data->interior[p].y;
    pcp (2) = data->interior[p].z;

    // inverse mapping
    Vector2d params;
    double error;
    if (p < (int)data->interior_param.size ())
    {
      params = ntools.inverseMapping (pcp, &data->interior_param[p], error, invMapInt_maxSteps, invMapInt_accuracy);
      data->interior_param[p] = params;
    }
    else
    {
      params = ntools.inverseMapping (pcp, NULL, error, invMapInt_maxSteps, invMapInt_accuracy);
      data->interior_param.push_back (params);
    }
    data->interior_error.push_back (error);

    double pointAndTangents[9];
    m_patch->Evaluate (params (0), params (1), 1, 3, pointAndTangents);
    data->interior_line_start.push_back (pcp);
    Vector3d r;
    r (0) = pointAndTangents[0];
    r (1) = pointAndTangents[1];
    r (2) = pointAndTangents[2];
    data->interior_line_end.push_back (r);

    addPointConstraint (params, pcp, wInt[p], row);
  } // p

  // cage regularisation
  if (nCageRegInt > 0)
    addCageInteriorRegularisation (wCageRegInt, row);

  if (nCageRegBnd > 0)
  {
    addCageBoundaryRegularisation (wCageRegBnd, NORTH, row);
    addCageBoundaryRegularisation (wCageRegBnd, SOUTH, row);
    addCageBoundaryRegularisation (wCageRegBnd, WEST, row);
    addCageBoundaryRegularisation (wCageRegBnd, EAST, row);
    addCageCornerRegularisation (wCageRegBnd * 4.0, row);
  }

  time_end = clock ();
  if (!m_quiet)
  {
    double solve_time = (double)(time_end - time_start) / (double)(CLOCKS_PER_SEC);
    printf ("[NurbsFitting::assemble()] (assemble (%d,%d): %f sec)\n", nrows, ncp, solve_time);
  }
}

void
NurbsFitting::assemble (int resU, int resV, double wBnd, double wInt, double wCurBnd, double wCurInt,
                        double wCageRegBnd, double wCageReg, double wCorner)
{

  NurbsTools ntools (m_patch);

  clock_t time_start, time_end;
  time_start = clock ();

  int nBnd = data->boundary.PointCount ();
  int nInt = data->interior.PointCount ();
  int nCurInt = resU * resV;
  int nCurBnd = 2 * resU + 2 * resV;
  int nCageReg = (m_patch->m_cv_count[0] - 2) * (m_patch->m_cv_count[1] - 2);
  int nCageRegBnd = 2 * (m_patch->m_cv_count[0] - 1) + 2 * (m_patch->m_cv_count[1] - 1);
  int nCorner = 4;

  if (wBnd <= 0.0)
    nBnd = 0;
  if (wInt <= 0.0)
    nInt = 0;
  if (wCurBnd <= 0.0)
    nCurBnd = 0;
  if (wCurInt <= 0.0)
    nCurInt = 0;
  if (wCageReg <= 0.0)
    nCageReg = 0;
  if (wCageRegBnd <= 0.0)
    nCageRegBnd = 0;
  if (wCorner <= 0.0)
    nCorner = 0;

  int ncp = m_patch->m_cv_count[0] * m_patch->m_cv_count[1];
  int nrows = nBnd + nInt + nCurInt + nCurBnd + nCorner + nCageReg + nCageRegBnd;

  m_Keig = Eigen::MatrixXd::Zero (nrows, ncp);
  m_feig = Eigen::MatrixXd::Zero (nrows, 3);

  int row = 0;

  // boundary points should lie on edges of surface
  data->boundary_line_start.clear ();
  data->boundary_line_end.clear ();
  data->boundary_error.clear ();
  for (int p = 0; p < nBnd; p++)
  {
    Vector3d pcp;
    pcp (0) = data->boundary[p].x;
    pcp (1) = data->boundary[p].y;
    pcp (2) = data->boundary[p].z;

    double error;
    Vector2d params = ntools.inverseMappingBoundary (pcp, error, invMapBnd_maxSteps, invMapBnd_accuracy);
    data->boundary_error.push_back (error);

    if (p < (int)data->boundary_param.size ())
    {
      data->boundary_param[p] = params;
    }
    else
    {
      data->boundary_param.push_back (params);
    }

    double pointAndTangents[9];
    m_patch->Evaluate (params (0), params (1), 1, 3, pointAndTangents);
    Vector3d r;
    r (0) = pointAndTangents[0];
    r (1) = pointAndTangents[1];
    r (2) = pointAndTangents[2];
    data->boundary_line_start.push_back (pcp);
    data->boundary_line_end.push_back (r);

    addPointConstraint (params, pcp, wBnd, row);
  } // p

  // interior points should lie on surface
  data->interior_line_start.clear ();
  data->interior_line_end.clear ();
  data->interior_error.clear ();
  for (int p = 0; p < nInt; p++)
  {
    Vector3d pcp;
    pcp (0) = data->interior[p].x;
    pcp (1) = data->interior[p].y;
    pcp (2) = data->interior[p].z;

    // inverse mapping
    Vector2d params;
    double error;
    if (p < (int)data->interior_param.size ())
    {
      params = ntools.inverseMapping (pcp, &data->interior_param[p], error, invMapInt_maxSteps, invMapInt_accuracy);
      data->interior_param[p] = params;
    }
    else
    {
      params = ntools.inverseMapping (pcp, NULL, error, invMapInt_maxSteps, invMapInt_accuracy);
      data->interior_param.push_back (params);
    }
    data->interior_error.push_back (error);

    double pointAndTangents[9];
    m_patch->Evaluate (params (0), params (1), 1, 3, pointAndTangents);
    data->interior_line_start.push_back (pcp);
    Vector3d r;
    r (0) = pointAndTangents[0];
    r (1) = pointAndTangents[1];
    r (2) = pointAndTangents[2];
    data->interior_line_end.push_back (r);

    addPointConstraint (params, pcp, wInt, row);
  } // p

  // minimal curvature on surface
  if (nCurInt > 0)
  {
    if (m_patch->m_order[0] < 3 || m_patch->m_order[1] < 3)
      printf ("[NurbsFitting::assemble] Error insufficient NURBS order to add curvature regularisation.\n");
    else
      addInteriorRegularisation (2, resU, resV, wCurInt / resU, row);
  }

  // minimal curvature on boundary
  if (nCurBnd > 0)
  {
    if (m_patch->m_order[0] < 3 || m_patch->m_order[1] < 3)
      printf ("[NurbsFitting::assemble] Error insufficient NURBS order to add curvature regularisation.\n");
    else
      addBoundaryRegularisation (2, resU, resV, wCurBnd / resU, row);
  }

  // cage regularisation
  if (nCageReg > 0)
    addCageInteriorRegularisation (wCageReg, row);

  if (nCageRegBnd > 0)
  {
    addCageBoundaryRegularisation (wCageRegBnd, NORTH, row);
    addCageBoundaryRegularisation (wCageRegBnd, SOUTH, row);
    addCageBoundaryRegularisation (wCageRegBnd, WEST, row);
    addCageBoundaryRegularisation (wCageRegBnd, EAST, row);
    addCageCornerRegularisation (wCageRegBnd * 4.0, row);
  }

  // corners of surface should lie on boundary
  if (nCorner > 0)
  {
    addBoundaryPointConstraint (m_minU, m_minV, 1.0, row);
    addBoundaryPointConstraint (m_minU, m_maxV, 1.0, row);
    addBoundaryPointConstraint (m_maxU, m_minV, 1.0, row);
    addBoundaryPointConstraint (m_maxU, m_maxV, 1.0, row);
  }

  time_end = clock ();
  if (!m_quiet)
  {
    double solve_time = (double)(time_end - time_start) / (double)(CLOCKS_PER_SEC);
    printf ("[NurbsFitting::assemble()] (assemble (%d,%d): %f sec)\n", nrows, ncp, solve_time);
  }
}

void
NurbsFitting::solve (double damp)
{
  solve_eigen (damp);
}

void
NurbsFitting::solve_eigen (double damp)
{

  clock_t time_start, time_end;
  time_start = clock ();

  m_xeig = m_Keig.colPivHouseholderQr ().solve (m_feig);
  //  Eigen::MatrixXd x = A.householderQr().solve(b);

  time_end = clock ();

  if (!m_quiet)
  {
    double solve_time = (double)(time_end - time_start) / (double)(CLOCKS_PER_SEC);
    printf ("[NurbsFitting::solve_eigen()] solution found! (%f sec)\n", solve_time);
  }
  updateSurf (damp);
}

void
NurbsFitting::updateSurf (double damp)
{

  NurbsTools ntools (m_patch);

  int ncp = m_patch->m_cv_count[0] * m_patch->m_cv_count[1];

  for (int A = 0; A < ncp; A++)
  {

    int I = ntools.I (A);
    int J = ntools.J (A);

    ON_3dPoint cp_prev;
    m_patch->GetCV (I, J, cp_prev);

    ON_3dPoint cp;
    cp.x = cp_prev.x + damp * (m_xeig (A, 0) - cp_prev.x);
    cp.y = cp_prev.y + damp * (m_xeig (A, 1) - cp_prev.y);
    cp.z = cp_prev.z + damp * (m_xeig (A, 2) - cp_prev.z);

    m_patch->SetCV (I, J, cp);

  }

}

void
NurbsFitting::setInvMapParams (double invMapBnd_maxSteps, double invMapInt_maxSteps, double invMapBnd_accuracy,
                               double invMapInt_accuracy)
{
  this->invMapBnd_maxSteps = invMapBnd_maxSteps;
  this->invMapInt_maxSteps = invMapInt_maxSteps;
  this->invMapBnd_accuracy = invMapBnd_accuracy;
  this->invMapInt_accuracy = invMapInt_accuracy;
}

void
NurbsFitting::addPointConstraint (Vector2d params, Vector3d point, double weight, int& row)
{

  NurbsTools ntools (m_patch);

  double N0[m_patch->m_order[0] * m_patch->m_order[0]];
  double N1[m_patch->m_order[1] * m_patch->m_order[1]];

  int E = ntools.E (params (0));
  int F = ntools.F (params (1));

  ON_EvaluateNurbsBasis (m_patch->m_order[0], m_patch->m_knot[0] + E, params (0), N0);
  ON_EvaluateNurbsBasis (m_patch->m_order[1], m_patch->m_knot[1] + F, params (1), N1);

  m_feig (row, 0) = point (0) * weight;
  m_feig (row, 1) = point (1) * weight;
  m_feig (row, 2) = point (2) * weight;

  for (int i = 0; i < m_patch->m_order[0]; i++)
  {

    for (int j = 0; j < m_patch->m_order[1]; j++)
    {

      m_Keig (row, ntools.A (E, F, i, j)) = weight * N0[i] * N1[j];

    } // j

  } // i

  row++;

}

void
NurbsFitting::addBoundaryPointConstraint (double paramU, double paramV, double weight, int &row)
{
  // edges on surface
  NurbsTools ntools (m_patch);

  double N0[m_patch->m_order[0] * m_patch->m_order[0]];
  double N1[m_patch->m_order[1] * m_patch->m_order[1]];

  double points[3];
  int E, F;
  ON_3dPoint closest;
  int closest_idx;

  m_patch->Evaluate (paramU, paramV, 0, 3, points);
  closest.x = points[0];
  closest.y = points[1];
  closest.z = points[2];
  data->boundary.GetClosestPoint (closest, &closest_idx);

  E = ntools.E (paramU);
  F = ntools.F (paramV);
  ON_EvaluateNurbsBasis (m_patch->m_order[0], m_patch->m_knot[0] + E, paramU, N0);
  ON_EvaluateNurbsBasis (m_patch->m_order[1], m_patch->m_knot[1] + F, paramV, N1);

  m_feig (row, 0) = data->boundary[closest_idx].x * weight;
  m_feig (row, 1) = data->boundary[closest_idx].y * weight;
  m_feig (row, 2) = data->boundary[closest_idx].z * weight;

  for (int i = 0; i < m_patch->m_order[0]; i++)
  {

    for (int j = 0; j < m_patch->m_order[1]; j++)
    {

      m_Keig (row, ntools.A (E, F, i, j)) = N0[i] * N1[j] * weight;

    } // j

  } // i

  row++;

}

void
NurbsFitting::addCageInteriorRegularisation (double weight, int &row)
{

  NurbsTools ntools (m_patch);

  for (int i = 1; i < (m_patch->m_cv_count[0] - 1); i++)
  {
    for (int j = 1; j < (m_patch->m_cv_count[1] - 1); j++)
    {

      m_feig (row, 0) = 0.0;
      m_feig (row, 1) = 0.0;
      m_feig (row, 2) = 0.0;

      m_Keig (row, ntools.A (i + 0, j + 0)) = -4.0 * weight;
      m_Keig (row, ntools.A (i + 0, j - 1)) = 1.0 * weight;
      m_Keig (row, ntools.A (i + 0, j + 1)) = 1.0 * weight;
      m_Keig (row, ntools.A (i - 1, j + 0)) = 1.0 * weight;
      m_Keig (row, ntools.A (i + 1, j + 0)) = 1.0 * weight;

      row++;
    }
  }
}

void
NurbsFitting::addCageBoundaryRegularisation (double weight, int side, int &row)
{

  NurbsTools ntools (m_patch);
  int i = 0;
  int j = 0;

  switch (side)
  {
    case SOUTH:
      j = m_patch->m_cv_count[1] - 1;
    case NORTH:
      for (i = 1; i < (m_patch->m_cv_count[0] - 1); i++)
      {

        m_feig (row, 0) = 0.0;
        m_feig (row, 1) = 0.0;
        m_feig (row, 2) = 0.0;

        m_Keig (row, ntools.A (i + 0, j)) = -2.0 * weight;
        m_Keig (row, ntools.A (i - 1, j)) = 1.0 * weight;
        m_Keig (row, ntools.A (i + 1, j)) = 1.0 * weight;

        row++;
      }
      break;

    case EAST:
      i = m_patch->m_cv_count[0] - 1;
    case WEST:
      for (j = 1; j < (m_patch->m_cv_count[1] - 1); j++)
      {

        m_feig (row, 0) = 0.0;
        m_feig (row, 1) = 0.0;
        m_feig (row, 2) = 0.0;

        m_Keig (row, ntools.A (i, j + 0)) = -2.0 * weight;
        m_Keig (row, ntools.A (i, j - 1)) = 1.0 * weight;
        m_Keig (row, ntools.A (i, j + 1)) = 1.0 * weight;

        row++;
      }
      break;
  }
}

void
NurbsFitting::addCageCornerRegularisation (double weight, int &row)
{

  NurbsTools ntools (m_patch);

  { // NORTH-WEST
    int i = 0;
    int j = 0;

    m_feig (row, 0) = 0.0;
    m_feig (row, 1) = 0.0;
    m_feig (row, 2) = 0.0;

    m_Keig (row, ntools.A (i + 0, j + 0)) = -2.0 * weight;
    m_Keig (row, ntools.A (i + 1, j + 0)) = 1.0 * weight;
    m_Keig (row, ntools.A (i + 0, j + 1)) = 1.0 * weight;

    row++;
  }

  { // NORTH-EAST
    int i = m_patch->m_cv_count[0] - 1;
    int j = 0;

    m_feig (row, 0) = 0.0;
    m_feig (row, 1) = 0.0;
    m_feig (row, 2) = 0.0;

    m_Keig (row, ntools.A (i + 0, j + 0)) = -2.0 * weight;
    m_Keig (row, ntools.A (i - 1, j + 0)) = 1.0 * weight;
    m_Keig (row, ntools.A (i + 0, j + 1)) = 1.0 * weight;

    row++;
  }

  { // SOUTH-EAST
    int i = m_patch->m_cv_count[0] - 1;
    int j = m_patch->m_cv_count[1] - 1;

    m_feig (row, 0) = 0.0;
    m_feig (row, 1) = 0.0;
    m_feig (row, 2) = 0.0;

    m_Keig (row, ntools.A (i + 0, j + 0)) = -2.0 * weight;
    m_Keig (row, ntools.A (i - 1, j + 0)) = 1.0 * weight;
    m_Keig (row, ntools.A (i + 0, j - 1)) = 1.0 * weight;

    row++;
  }

  { // SOUTH-WEST
    int i = 0;
    int j = m_patch->m_cv_count[1] - 1;

    m_feig (row, 0) = 0.0;
    m_feig (row, 1) = 0.0;
    m_feig (row, 2) = 0.0;

    m_Keig (row, ntools.A (i + 0, j + 0)) = -2.0 * weight;
    m_Keig (row, ntools.A (i + 1, j + 0)) = 1.0 * weight;
    m_Keig (row, ntools.A (i + 0, j - 1)) = 1.0 * weight;

    row++;
  }

}

void
NurbsFitting::addInteriorRegularisation (int order, int resU, int resV, double weight, int &row)
{

  NurbsTools ntools (m_patch);

  double N0[m_patch->m_order[0] * m_patch->m_order[0]];
  double N1[m_patch->m_order[1] * m_patch->m_order[1]];

  double dU = (m_maxU - m_minU) / resU;
  double dV = (m_maxV - m_minV) / resV;

  for (int u = 0; u < resU; u++)
  {
    for (int v = 0; v < resV; v++)
    {

      Vector2d params;
      params (0) = m_minU + u * dU + 0.5 * dU;
      params (1) = m_minV + v * dV + 0.5 * dV;

      //			printf("%f %f, %f %f\n", m_minU, dU, params(0), params(1));

      int E = ntools.E (params (0));
      int F = ntools.F (params (1));

      ON_EvaluateNurbsBasis (m_patch->m_order[0], m_patch->m_knot[0] + E, params (0), N0);
      ON_EvaluateNurbsBasis (m_patch->m_order[1], m_patch->m_knot[1] + F, params (1), N1);
      ON_EvaluateNurbsBasisDerivatives (m_patch->m_order[0], m_patch->m_knot[0] + E, order, N0); // derivative order?
      ON_EvaluateNurbsBasisDerivatives (m_patch->m_order[1], m_patch->m_knot[1] + F, order, N1);

      m_feig (row, 0) = 0.0;
      m_feig (row, 1) = 0.0;
      m_feig (row, 2) = 0.0;

      for (int i = 0; i < m_patch->m_order[0]; i++)
      {
        for (int j = 0; j < m_patch->m_order[1]; j++)
        {

          m_Keig (row, ntools.A (E, F, i, j)) = weight * (N0[order * m_patch->m_order[0] + i] * N1[j] + N0[i]
              * N1[order * m_patch->m_order[1] + j]);

        } // i
      } // j

      row++;
    } // u
  } // v

}

void
NurbsFitting::addBoundaryRegularisation (int order, int resU, int resV, double weight, int &row)
{

  NurbsTools ntools (m_patch);

  double N0[m_patch->m_order[0] * m_patch->m_order[0]];
  double N1[m_patch->m_order[1] * m_patch->m_order[1]];

  double dU = (m_maxU - m_minU) / resU;
  double dV = (m_maxV - m_minV) / resV;

  for (int u = 0; u < resU; u++)
  {

    Vector2d params;
    params (0) = m_minU + u * dU + 0.5 * dU;
    params (1) = m_minV;

    int E = ntools.E (params (0));
    int F = ntools.F (params (1));

    ON_EvaluateNurbsBasis (m_patch->m_order[0], m_patch->m_knot[0] + E, params (0), N0);
    ON_EvaluateNurbsBasis (m_patch->m_order[1], m_patch->m_knot[1] + F, params (1), N1);
    ON_EvaluateNurbsBasisDerivatives (m_patch->m_order[0], m_patch->m_knot[0] + E, order, N0); // derivative order?
    ON_EvaluateNurbsBasisDerivatives (m_patch->m_order[1], m_patch->m_knot[1] + F, order, N1);

    m_feig (row, 0) = 0.0;
    m_feig (row, 1) = 0.0;
    m_feig (row, 2) = 0.0;

    for (int i = 0; i < m_patch->m_order[0]; i++)
    {
      for (int j = 0; j < m_patch->m_order[1]; j++)
      {

        m_Keig (row, ntools.A (E, F, i, j)) = weight * (N0[order * m_patch->m_order[0] + i] * N1[j] + N0[i] * N1[order
            * m_patch->m_order[1] + j]);

      } // i
    } // j

    row++;

  } // u

  for (int u = 0; u < resU; u++)
  {

    Vector2d params;
    params (0) = m_minU + u * dU + 0.5 * dU;
    params (1) = m_maxV;

    int E = ntools.E (params (0));
    int F = ntools.F (params (1));

    ON_EvaluateNurbsBasis (m_patch->m_order[0], m_patch->m_knot[0] + E, params (0), N0);
    ON_EvaluateNurbsBasis (m_patch->m_order[1], m_patch->m_knot[1] + F, params (1), N1);
    ON_EvaluateNurbsBasisDerivatives (m_patch->m_order[0], m_patch->m_knot[0] + E, order, N0); // derivative order?
    ON_EvaluateNurbsBasisDerivatives (m_patch->m_order[1], m_patch->m_knot[1] + F, order, N1);

    m_feig (row, 0) = 0.0;
    m_feig (row, 1) = 0.0;
    m_feig (row, 2) = 0.0;

    for (int i = 0; i < m_patch->m_order[0]; i++)
    {
      for (int j = 0; j < m_patch->m_order[1]; j++)
      {

        m_Keig (row, ntools.A (E, F, i, j)) = weight * (N0[order * m_patch->m_order[0] + i] * N1[j] + N0[i] * N1[order
            * m_patch->m_order[1] + j]);

      } // i
    } // j

    row++;

  } // u

  for (int v = 0; v < resV; v++)
  {

    Vector2d params;
    params (0) = m_minU;
    params (1) = m_minV + v * dV + 0.5 * dV;

    int E = ntools.E (params (0));
    int F = ntools.F (params (1));

    ON_EvaluateNurbsBasis (m_patch->m_order[0], m_patch->m_knot[0] + E, params (0), N0);
    ON_EvaluateNurbsBasis (m_patch->m_order[1], m_patch->m_knot[1] + F, params (1), N1);
    ON_EvaluateNurbsBasisDerivatives (m_patch->m_order[0], m_patch->m_knot[0] + E, order, N0); // derivative order?
    ON_EvaluateNurbsBasisDerivatives (m_patch->m_order[1], m_patch->m_knot[1] + F, order, N1);

    m_feig (row, 0) = 0.0;
    m_feig (row, 1) = 0.0;
    m_feig (row, 2) = 0.0;

    for (int i = 0; i < m_patch->m_order[0]; i++)
    {
      for (int j = 0; j < m_patch->m_order[1]; j++)
      {

        m_Keig (row, ntools.A (E, F, i, j)) = weight * (N0[order * m_patch->m_order[0] + i] * N1[j] + N0[i] * N1[order
            * m_patch->m_order[1] + j]);

      } // i
    } // j

    row++;

  } // v

  for (int v = 0; v < resV; v++)
  {

    Vector2d params;
    params (0) = m_maxU;
    params (1) = m_minV + v * dV + 0.5 * dV;

    int E = ntools.E (params (0));
    int F = ntools.F (params (1));

    ON_EvaluateNurbsBasis (m_patch->m_order[0], m_patch->m_knot[0] + E, params (0), N0);
    ON_EvaluateNurbsBasis (m_patch->m_order[1], m_patch->m_knot[1] + F, params (1), N1);
    ON_EvaluateNurbsBasisDerivatives (m_patch->m_order[0], m_patch->m_knot[0] + E, order, N0); // derivative order?
    ON_EvaluateNurbsBasisDerivatives (m_patch->m_order[1], m_patch->m_knot[1] + F, order, N1);

    m_feig (row, 0) = 0.0;
    m_feig (row, 1) = 0.0;
    m_feig (row, 2) = 0.0;

    for (int i = 0; i < m_patch->m_order[0]; i++)
    {
      for (int j = 0; j < m_patch->m_order[1]; j++)
      {

        m_Keig (row, ntools.A (E, F, i, j)) = weight * (N0[order * m_patch->m_order[0] + i] * N1[j] + N0[i] * N1[order
            * m_patch->m_order[1] + j]);

      } // i
    } // j

    row++;

  } // v

}
