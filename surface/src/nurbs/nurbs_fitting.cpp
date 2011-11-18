/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Thomas Mörwald, Jonathan Balzer
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
#include "pcl/surface/nurbs/nurbs_fitting.h"
#include "pcl/common/common.h"

using namespace pcl;
using namespace Eigen;

NurbsFitting::NurbsFitting (int order, NurbsData *nurbs_data_, ON_3dPoint ll, ON_3dPoint lr, ON_3dPoint ur,
                            ON_3dPoint ul)
{
  if (order < 2)
    throw std::runtime_error ("[NurbsFitting::NurbsFitting] Error order to low (order<2).");

  ON::Begin ();

  this->nurbs_data_ = nurbs_data_;

  nurbs_patch_ = new ON_NurbsSurface (3, false, order, order, order, order);
  nurbs_patch_->MakeClampedUniformKnotVector (0, 1.0);
  nurbs_patch_->MakeClampedUniformKnotVector (1, 1.0);

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

      nurbs_patch_->SetCV (i, j, cv_map[i][j]);

    } // j
  } // i

  //	nurbs_patch_->Dump(on_out_);
  //	printf("closed: %d %d\n", nurbs_patch_->IsClosed(0), nurbs_patch_->IsClosed(1));
  //	printf("periodic: %d %d\n", nurbs_patch_->IsPeriodic(0), nurbs_patch_->IsPeriodic(1));

  this->init ();
}

NurbsFitting::NurbsFitting (int order, NurbsData *nurbs_data_, const std::vector<ON_3dPoint> &cv)
{
  if (order < 2)
    throw std::runtime_error ("[NurbsFitting::NurbsFitting] Error order to low (order<2).");

  if (unsigned (order * order) != cv.size ())
    throw std::runtime_error (
                              "[NurbsFitting::NurbsFitting] Error number of control points don't match: (order*order != cps.size()).\n");

  ON::Begin ();

  this->nurbs_data_ = nurbs_data_;

  nurbs_patch_ = new ON_NurbsSurface (3, false, order, order, order, order);
  nurbs_patch_->MakeClampedUniformKnotVector (0, 1.0);
  nurbs_patch_->MakeClampedUniformKnotVector (1, 1.0);

  unsigned cvi (0);
  for (int i = 0; i < order; i++)
  {
    for (int j = 0; j < order; j++)
    {

      nurbs_patch_->SetCV (i, j, cv[cvi++]);

    } // j
  } // i

  //	nurbs_patch_->Dump(on_out_);
  //	printf("closed: %d %d\n", nurbs_patch_->IsClosed(0), nurbs_patch_->IsClosed(1));
  //	printf("periodic: %d %d\n", nurbs_patch_->IsPeriodic(0), nurbs_patch_->IsPeriodic(1));

  this->init ();
}

NurbsFitting::NurbsFitting (int order, NurbsData *nurbs_data_)
{
  if (order < 2)
    throw std::runtime_error ("[NurbsFitting::NurbsFitting] Error order to low (order<2).");

  ON::Begin ();

  this->nurbs_data_ = nurbs_data_;

  double m_min[3];
  double m_max[3];
  double m_mid[3];

  pcl::PointXYZ min, max;
  pcl::getMinMax3D (*nurbs_data_->boundary, min, max);
  m_min[0] = min.x;
  m_min[1] = min.y, m_min[2] = min.z;
  m_max[0] = max.x;
  m_max[1] = max.y, m_max[2] = max.z;

  //  nurbs_data_->boundary.GetBBox (m_min, m_max);

  m_mid[0] = m_min[0] + (m_max[0] - m_min[0]) * 0.5;
  m_mid[1] = m_min[1] + (m_max[1] - m_min[1]) * 0.5;
  m_mid[2] = m_min[2] + (m_max[2] - m_min[2]) * 0.5;

  nurbs_patch_ = new ON_NurbsSurface (3, false, order, order, order, order);
  nurbs_patch_->MakeClampedUniformKnotVector (0, 1.0);
  nurbs_patch_->MakeClampedUniformKnotVector (1, 1.0);

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
      nurbs_patch_->SetCV (i, j, cv);

    } // j
  } // i

  //	nurbs_patch_->Dump(on_out_);
  //	printf("closed: %d %d\n", nurbs_patch_->IsClosed(0), nurbs_patch_->IsClosed(1));
  //	printf("periodic: %d %d\n", nurbs_patch_->IsPeriodic(0), nurbs_patch_->IsPeriodic(1));

  this->init ();
}

NurbsFitting::NurbsFitting (NurbsData *nurbs_data_, const ON_NurbsSurface &ns)
{
  ON::Begin ();

  this->nurbs_data_ = nurbs_data_;

  this->init ();
}

NurbsFitting::~NurbsFitting ()
{

  delete (nurbs_patch_);

  ON::End ();

}

void
NurbsFitting::refine (int dim)
{

  NurbsTools ntools (nurbs_patch_);

  std::vector<double> xi;

  std::vector<double> elements = ntools.getElementVector (dim);

  for (unsigned i = 0; i < elements.size () - 1; i++)
  {

    xi.push_back (elements[i] + 0.5 * (elements[i + 1] - elements[i]));

  } // i

  for (unsigned i = 0; i < xi.size (); i++)
  {

    nurbs_patch_->InsertKnot (dim, xi[i], 1);

  } // i

  m_elementsU = ntools.getElementVector (0);
  m_elementsV = ntools.getElementVector (1);
  min_u_ = m_elementsU[0];
  min_v_ = m_elementsV[0];
  max_u_ = m_elementsU[m_elementsU.size () - 1];
  max_v_ = m_elementsV[m_elementsV.size () - 1];

  x_eig_ = Eigen::MatrixXd::Zero (nurbs_patch_->m_cv_count[0] * nurbs_patch_->m_cv_count[1], 3);

  //	nurbs_patch_->Dump(on_out_);

}

void
NurbsFitting::init ()
{
  NurbsTools ntools (nurbs_patch_);
  m_elementsU = ntools.getElementVector (0);
  m_elementsV = ntools.getElementVector (1);
  min_u_ = m_elementsU[0];
  min_v_ = m_elementsV[0];
  max_u_ = m_elementsU[m_elementsU.size () - 1];
  max_v_ = m_elementsV[m_elementsV.size () - 1];

  x_eig_ = Eigen::MatrixXd::Zero (nurbs_patch_->m_cv_count[0] * nurbs_patch_->m_cv_count[1], 3);

  inv_map_iter_bnd_ = 100;
  inv_map_iter_int_ = 100;
  inv_map_accuracy_bnd_ = 1e-3;
  inv_map_accuracy_int_ = 1e-3;

  quiet_ = true;
  use_int_hints_ = false;
}

void
NurbsFitting::assemble ()
{
  clock_t time_start, time_end;
  time_start = clock ();

  NurbsTools ntools (nurbs_patch_);

  int ncp = nurbs_patch_->m_cv_count[0] * nurbs_patch_->m_cv_count[1];
  int nInt = nurbs_data_->interior->size ();
  int nCageRegInt = (nurbs_patch_->m_cv_count[0] - 2) * (nurbs_patch_->m_cv_count[1] - 2);
  int nCageRegBnd = 2 * (nurbs_patch_->m_cv_count[0] - 1) + 2 * (nurbs_patch_->m_cv_count[1] - 1);

  double wInt = 1.0;
  double wCageRegInt = 0.00001;
  double wCageRegBnd = 0.00001;

  int nrows = nInt + nCageRegInt + nCageRegBnd;

  K_eig_ = Eigen::MatrixXd::Zero (nrows, ncp);
  f_eig_ = Eigen::MatrixXd::Zero (nrows, 3);

  int row (0);

  // interior points should lie on surface
  nurbs_data_->interior_line_start.clear ();
  nurbs_data_->interior_line_end.clear ();
  nurbs_data_->interior_error.clear ();
  for (int p = 0; p < nInt; p++)
  {
    Vector3d pcp;
    pcp (0) = nurbs_data_->interior->at (p).x;
    pcp (1) = nurbs_data_->interior->at (p).y;
    pcp (2) = nurbs_data_->interior->at (p).z;

    // inverse mapping
    Vector2d params;
    double error;
    if (p < (int)nurbs_data_->interior_param.size ())
    {
      params = ntools.inverseMapping (pcp, &nurbs_data_->interior_param[p], error, inv_map_iter_int_,
                                      inv_map_accuracy_int_);
      nurbs_data_->interior_param[p] = params;
    }
    else
    {
      params = ntools.inverseMapping (pcp, NULL, error, inv_map_iter_int_, inv_map_accuracy_int_);
      nurbs_data_->interior_param.push_back (params);
    }
    nurbs_data_->interior_error.push_back (error);

    double pointAndTangents[9];
    nurbs_patch_->Evaluate (params (0), params (1), 1, 3, pointAndTangents);
    nurbs_data_->interior_line_start.push_back (pcp);
    Vector3d r;
    r (0) = pointAndTangents[0];
    r (1) = pointAndTangents[1];
    r (2) = pointAndTangents[2];
    nurbs_data_->interior_line_end.push_back (r);

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
  if (!quiet_)
  {
    double solve_time = (double)(time_end - time_start) / (double)(CLOCKS_PER_SEC);
    printf ("[NurbsFitting::assemble()] (assemble (%d,%d): %f sec)\n", nrows, ncp, solve_time);
  }

}

void
NurbsFitting::assemble (std::vector<double> &wBnd, std::vector<double> &wInt, double wCageRegBnd, double wCageRegInt)
{
  if (wBnd.size () > (unsigned)nurbs_data_->boundary->size ())
    throw std::runtime_error ("[NurbsFitting::assemble] Size of weight vector greater than point size (boundary).");

  if (wInt.size () > (unsigned)nurbs_data_->interior->size ())
    throw std::runtime_error ("[NurbsFitting::assemble] Size of weight vector greater than point size (interior).");

  NurbsTools ntools (nurbs_patch_);

  clock_t time_start, time_end;
  time_start = clock ();

  int nBnd = wBnd.size ();
  int nInt = wInt.size ();
  int nCageRegInt = (nurbs_patch_->m_cv_count[0] - 2) * (nurbs_patch_->m_cv_count[1] - 2);
  int nCageRegBnd = 2 * (nurbs_patch_->m_cv_count[0] - 1) + 2 * (nurbs_patch_->m_cv_count[1] - 1);

  if (wCageRegInt <= 0.0)
    nCageRegInt = 0;
  if (wCageRegBnd <= 0.0)
    nCageRegBnd = 0;

  int ncp = nurbs_patch_->m_cv_count[0] * nurbs_patch_->m_cv_count[1];
  int nrows = nBnd + nInt + nCageRegInt + nCageRegBnd;

  K_eig_ = Eigen::MatrixXd::Zero (nrows, ncp);
  f_eig_ = Eigen::MatrixXd::Zero (nrows, 3);

  int row = 0;

  // boundary points should lie on edges of surface
  nurbs_data_->boundary_line_start.clear ();
  nurbs_data_->boundary_line_end.clear ();
  nurbs_data_->boundary_error.clear ();
  for (int p = 0; p < nBnd; p++)
  {
    Vector3d pcp;
    pcp (0) = nurbs_data_->boundary->at (p).x;
    pcp (1) = nurbs_data_->boundary->at (p).y;
    pcp (2) = nurbs_data_->boundary->at (p).z;

    double error;
    Vector2d params = ntools.inverseMappingBoundary (pcp, error, inv_map_iter_bnd_, inv_map_accuracy_bnd_);
    nurbs_data_->boundary_error.push_back (error);

    if (p < (int)nurbs_data_->boundary_param.size ())
    {
      nurbs_data_->boundary_param[p] = params;
    }
    else
    {
      nurbs_data_->boundary_param.push_back (params);
    }

    double pointAndTangents[9];
    nurbs_patch_->Evaluate (params (0), params (1), 1, 3, pointAndTangents);
    Vector3d r;
    r (0) = pointAndTangents[0];
    r (1) = pointAndTangents[1];
    r (2) = pointAndTangents[2];
    nurbs_data_->boundary_line_start.push_back (pcp);
    nurbs_data_->boundary_line_end.push_back (r);

    addPointConstraint (params, pcp, wBnd[p], row);
  } // p

  // interior points should lie on surface
  nurbs_data_->interior_line_start.clear ();
  nurbs_data_->interior_line_end.clear ();
  nurbs_data_->interior_error.clear ();
  for (int p = 0; p < nInt; p++)
  {
    Vector3d pcp;
    pcp (0) = nurbs_data_->interior->at (p).x;
    pcp (1) = nurbs_data_->interior->at (p).y;
    pcp (2) = nurbs_data_->interior->at (p).z;

    // inverse mapping
    Vector2d params;
    double error;
    if (p < (int)nurbs_data_->interior_param.size ())
    {
      params = ntools.inverseMapping (pcp, &nurbs_data_->interior_param[p], error, inv_map_iter_int_,
                                      inv_map_accuracy_int_);
      nurbs_data_->interior_param[p] = params;
    }
    else
    {
      params = ntools.inverseMapping (pcp, NULL, error, inv_map_iter_int_, inv_map_accuracy_int_);
      nurbs_data_->interior_param.push_back (params);
    }
    nurbs_data_->interior_error.push_back (error);

    double pointAndTangents[9];
    nurbs_patch_->Evaluate (params (0), params (1), 1, 3, pointAndTangents);
    nurbs_data_->interior_line_start.push_back (pcp);
    Vector3d r;
    r (0) = pointAndTangents[0];
    r (1) = pointAndTangents[1];
    r (2) = pointAndTangents[2];
    nurbs_data_->interior_line_end.push_back (r);

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
  if (!quiet_)
  {
    double solve_time = (double)(time_end - time_start) / (double)(CLOCKS_PER_SEC);
    printf ("[NurbsFitting::assemble()] (assemble (%d,%d): %f sec)\n", nrows, ncp, solve_time);
  }
}

void
NurbsFitting::assemble (int resU, int resV, double wBnd, double wInt, double wCurBnd, double wCurInt,
                        double wCageRegBnd, double wCageReg, double wCorner)
{

  NurbsTools ntools (nurbs_patch_);

  clock_t time_start, time_end;
  time_start = clock ();

  int nBnd = nurbs_data_->boundary->size ();
  int nInt = nurbs_data_->interior->size ();
  int nCurInt = resU * resV;
  int nCurBnd = 2 * resU + 2 * resV;
  int nCageReg = (nurbs_patch_->m_cv_count[0] - 2) * (nurbs_patch_->m_cv_count[1] - 2);
  int nCageRegBnd = 2 * (nurbs_patch_->m_cv_count[0] - 1) + 2 * (nurbs_patch_->m_cv_count[1] - 1);
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

  int ncp = nurbs_patch_->m_cv_count[0] * nurbs_patch_->m_cv_count[1];
  int nrows = nBnd + nInt + nCurInt + nCurBnd + nCorner + nCageReg + nCageRegBnd;

  K_eig_ = Eigen::MatrixXd::Zero (nrows, ncp);
  f_eig_ = Eigen::MatrixXd::Zero (nrows, 3);

  int row = 0;

  // boundary points should lie on edges of surface
  nurbs_data_->boundary_line_start.clear ();
  nurbs_data_->boundary_line_end.clear ();
  nurbs_data_->boundary_error.clear ();
  for (int p = 0; p < nBnd; p++)
  {
    Vector3d pcp;
    pcp (0) = nurbs_data_->boundary->at (p).x;
    pcp (1) = nurbs_data_->boundary->at (p).y;
    pcp (2) = nurbs_data_->boundary->at (p).z;

    double error;
    Vector2d params = ntools.inverseMappingBoundary (pcp, error, inv_map_iter_bnd_, inv_map_accuracy_bnd_);
    nurbs_data_->boundary_error.push_back (error);

    if (p < (int)nurbs_data_->boundary_param.size ())
    {
      nurbs_data_->boundary_param[p] = params;
    }
    else
    {
      nurbs_data_->boundary_param.push_back (params);
    }

    double pointAndTangents[9];
    nurbs_patch_->Evaluate (params (0), params (1), 1, 3, pointAndTangents);
    Vector3d r;
    r (0) = pointAndTangents[0];
    r (1) = pointAndTangents[1];
    r (2) = pointAndTangents[2];
    nurbs_data_->boundary_line_start.push_back (pcp);
    nurbs_data_->boundary_line_end.push_back (r);

    addPointConstraint (params, pcp, wBnd, row);
  } // p

  // interior points should lie on surface
  nurbs_data_->interior_line_start.clear ();
  nurbs_data_->interior_line_end.clear ();
  nurbs_data_->interior_error.clear ();
  for (int p = 0; p < nInt; p++)
  {
    Vector3d pcp;
    pcp (0) = nurbs_data_->interior->at (p).x;
    pcp (1) = nurbs_data_->interior->at (p).y;
    pcp (2) = nurbs_data_->interior->at (p).z;

    // inverse mapping
    Vector2d params;
    double error;
    if (p < (int)nurbs_data_->interior_param.size ())
    {
      params = ntools.inverseMapping (pcp, &nurbs_data_->interior_param[p], error, inv_map_iter_int_,
                                      inv_map_accuracy_int_);
      nurbs_data_->interior_param[p] = params;
    }
    else
    {
      params = ntools.inverseMapping (pcp, NULL, error, inv_map_iter_int_, inv_map_accuracy_int_);
      nurbs_data_->interior_param.push_back (params);
    }
    nurbs_data_->interior_error.push_back (error);

    double pointAndTangents[9];
    nurbs_patch_->Evaluate (params (0), params (1), 1, 3, pointAndTangents);
    nurbs_data_->interior_line_start.push_back (pcp);
    Vector3d r;
    r (0) = pointAndTangents[0];
    r (1) = pointAndTangents[1];
    r (2) = pointAndTangents[2];
    nurbs_data_->interior_line_end.push_back (r);

    addPointConstraint (params, pcp, wInt, row);
  } // p

  // minimal curvature on surface
  if (nCurInt > 0)
  {
    if (nurbs_patch_->m_order[0] < 3 || nurbs_patch_->m_order[1] < 3)
      printf ("[NurbsFitting::assemble] Error insufficient NURBS order to add curvature regularisation.\n");
    else
      addInteriorRegularisation (2, resU, resV, wCurInt / resU, row);
  }

  // minimal curvature on boundary
  if (nCurBnd > 0)
  {
    if (nurbs_patch_->m_order[0] < 3 || nurbs_patch_->m_order[1] < 3)
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

  //  // corners of surface should lie on boundary
  //  if (nCorner > 0)
  //  {
  //    addBoundaryPointConstraint (min_u_, min_v_, 1.0, row);
  //    addBoundaryPointConstraint (min_u_, max_v_, 1.0, row);
  //    addBoundaryPointConstraint (max_u_, min_v_, 1.0, row);
  //    addBoundaryPointConstraint (max_u_, max_v_, 1.0, row);
  //  }

  time_end = clock ();
  if (!quiet_)
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

  x_eig_ = K_eig_.colPivHouseholderQr ().solve (f_eig_);
  //  Eigen::MatrixXd x = A.householderQr().solve(b);

  time_end = clock ();

  if (!quiet_)
  {
    double solve_time = (double)(time_end - time_start) / (double)(CLOCKS_PER_SEC);
    printf ("[NurbsFitting::solve_eigen()] solution found! (%f sec)\n", solve_time);
  }
  updateSurf (damp);
}

void
NurbsFitting::updateSurf (double damp)
{

  NurbsTools ntools (nurbs_patch_);

  int ncp = nurbs_patch_->m_cv_count[0] * nurbs_patch_->m_cv_count[1];

  for (int A = 0; A < ncp; A++)
  {

    int I = ntools.I (A);
    int J = ntools.J (A);

    ON_3dPoint cp_prev;
    nurbs_patch_->GetCV (I, J, cp_prev);

    ON_3dPoint cp;
    cp.x = cp_prev.x + damp * (x_eig_ (A, 0) - cp_prev.x);
    cp.y = cp_prev.y + damp * (x_eig_ (A, 1) - cp_prev.y);
    cp.z = cp_prev.z + damp * (x_eig_ (A, 2) - cp_prev.z);

    nurbs_patch_->SetCV (I, J, cp);

  }

}

void
NurbsFitting::setInvMapParams (double inv_map_iter_bnd, double inv_map_iter_int, double inv_map_accuracy_bnd,
                               double inv_map_accuracy_int)
{
  this->inv_map_iter_bnd_ = inv_map_iter_bnd;
  this->inv_map_iter_int_ = inv_map_iter_int;
  this->inv_map_accuracy_bnd_ = inv_map_accuracy_bnd;
  this->inv_map_accuracy_int_ = inv_map_accuracy_int;
}

void
NurbsFitting::addPointConstraint (Vector2d params, Vector3d point, double weight, int& row)
{

  NurbsTools ntools (nurbs_patch_);

  double N0[nurbs_patch_->m_order[0] * nurbs_patch_->m_order[0]];
  double N1[nurbs_patch_->m_order[1] * nurbs_patch_->m_order[1]];

  int E = ntools.E (params (0));
  int F = ntools.F (params (1));

  ON_EvaluateNurbsBasis (nurbs_patch_->m_order[0], nurbs_patch_->m_knot[0] + E, params (0), N0);
  ON_EvaluateNurbsBasis (nurbs_patch_->m_order[1], nurbs_patch_->m_knot[1] + F, params (1), N1);

  f_eig_ (row, 0) = point (0) * weight;
  f_eig_ (row, 1) = point (1) * weight;
  f_eig_ (row, 2) = point (2) * weight;

  for (int i = 0; i < nurbs_patch_->m_order[0]; i++)
  {

    for (int j = 0; j < nurbs_patch_->m_order[1]; j++)
    {

      K_eig_ (row, ntools.A (E, F, i, j)) = weight * N0[i] * N1[j];

    } // j

  } // i

  row++;

}

//void
//NurbsFitting::addBoundaryPointConstraint (double paramU, double paramV, double weight, int &row)
//{
//  // edges on surface
//  NurbsTools ntools (nurbs_patch_);
//
//  double N0[nurbs_patch_->m_order[0] * nurbs_patch_->m_order[0]];
//  double N1[nurbs_patch_->m_order[1] * nurbs_patch_->m_order[1]];
//
//  double points[3];
//  int E, F;
//  ON_3dPoint closest;
//  int closest_idx;
//
//  nurbs_patch_->Evaluate (paramU, paramV, 0, 3, points);
//  closest.x = points[0];
//  closest.y = points[1];
//  closest.z = points[2];
//  nurbs_data_->boundary.GetClosestPoint (closest, &closest_idx);
//
//  E = ntools.E (paramU);
//  F = ntools.F (paramV);
//  ON_EvaluateNurbsBasis (nurbs_patch_->m_order[0], nurbs_patch_->m_knot[0] + E, paramU, N0);
//  ON_EvaluateNurbsBasis (nurbs_patch_->m_order[1], nurbs_patch_->m_knot[1] + F, paramV, N1);
//
//  f_eig_ (row, 0) = nurbs_data_->boundary[closest_idx].x * weight;
//  f_eig_ (row, 1) = nurbs_data_->boundary[closest_idx].y * weight;
//  f_eig_ (row, 2) = nurbs_data_->boundary[closest_idx].z * weight;
//
//  for (int i = 0; i < nurbs_patch_->m_order[0]; i++)
//  {
//
//    for (int j = 0; j < nurbs_patch_->m_order[1]; j++)
//    {
//
//      K_eig_ (row, ntools.A (E, F, i, j)) = N0[i] * N1[j] * weight;
//
//    } // j
//
//  } // i
//
//  row++;
//
//}

void
NurbsFitting::addCageInteriorRegularisation (double weight, int &row)
{

  NurbsTools ntools (nurbs_patch_);

  for (int i = 1; i < (nurbs_patch_->m_cv_count[0] - 1); i++)
  {
    for (int j = 1; j < (nurbs_patch_->m_cv_count[1] - 1); j++)
    {

      f_eig_ (row, 0) = 0.0;
      f_eig_ (row, 1) = 0.0;
      f_eig_ (row, 2) = 0.0;

      K_eig_ (row, ntools.A (i + 0, j + 0)) = -4.0 * weight;
      K_eig_ (row, ntools.A (i + 0, j - 1)) = 1.0 * weight;
      K_eig_ (row, ntools.A (i + 0, j + 1)) = 1.0 * weight;
      K_eig_ (row, ntools.A (i - 1, j + 0)) = 1.0 * weight;
      K_eig_ (row, ntools.A (i + 1, j + 0)) = 1.0 * weight;

      row++;
    }
  }
}

void
NurbsFitting::addCageBoundaryRegularisation (double weight, int side, int &row)
{

  NurbsTools ntools (nurbs_patch_);
  int i = 0;
  int j = 0;

  switch (side)
  {
    case SOUTH:
      j = nurbs_patch_->m_cv_count[1] - 1;
    case NORTH:
      for (i = 1; i < (nurbs_patch_->m_cv_count[0] - 1); i++)
      {

        f_eig_ (row, 0) = 0.0;
        f_eig_ (row, 1) = 0.0;
        f_eig_ (row, 2) = 0.0;

        K_eig_ (row, ntools.A (i + 0, j)) = -2.0 * weight;
        K_eig_ (row, ntools.A (i - 1, j)) = 1.0 * weight;
        K_eig_ (row, ntools.A (i + 1, j)) = 1.0 * weight;

        row++;
      }
      break;

    case EAST:
      i = nurbs_patch_->m_cv_count[0] - 1;
    case WEST:
      for (j = 1; j < (nurbs_patch_->m_cv_count[1] - 1); j++)
      {

        f_eig_ (row, 0) = 0.0;
        f_eig_ (row, 1) = 0.0;
        f_eig_ (row, 2) = 0.0;

        K_eig_ (row, ntools.A (i, j + 0)) = -2.0 * weight;
        K_eig_ (row, ntools.A (i, j - 1)) = 1.0 * weight;
        K_eig_ (row, ntools.A (i, j + 1)) = 1.0 * weight;

        row++;
      }
      break;
  }
}

void
NurbsFitting::addCageCornerRegularisation (double weight, int &row)
{

  NurbsTools ntools (nurbs_patch_);

  { // NORTH-WEST
    int i = 0;
    int j = 0;

    f_eig_ (row, 0) = 0.0;
    f_eig_ (row, 1) = 0.0;
    f_eig_ (row, 2) = 0.0;

    K_eig_ (row, ntools.A (i + 0, j + 0)) = -2.0 * weight;
    K_eig_ (row, ntools.A (i + 1, j + 0)) = 1.0 * weight;
    K_eig_ (row, ntools.A (i + 0, j + 1)) = 1.0 * weight;

    row++;
  }

  { // NORTH-EAST
    int i = nurbs_patch_->m_cv_count[0] - 1;
    int j = 0;

    f_eig_ (row, 0) = 0.0;
    f_eig_ (row, 1) = 0.0;
    f_eig_ (row, 2) = 0.0;

    K_eig_ (row, ntools.A (i + 0, j + 0)) = -2.0 * weight;
    K_eig_ (row, ntools.A (i - 1, j + 0)) = 1.0 * weight;
    K_eig_ (row, ntools.A (i + 0, j + 1)) = 1.0 * weight;

    row++;
  }

  { // SOUTH-EAST
    int i = nurbs_patch_->m_cv_count[0] - 1;
    int j = nurbs_patch_->m_cv_count[1] - 1;

    f_eig_ (row, 0) = 0.0;
    f_eig_ (row, 1) = 0.0;
    f_eig_ (row, 2) = 0.0;

    K_eig_ (row, ntools.A (i + 0, j + 0)) = -2.0 * weight;
    K_eig_ (row, ntools.A (i - 1, j + 0)) = 1.0 * weight;
    K_eig_ (row, ntools.A (i + 0, j - 1)) = 1.0 * weight;

    row++;
  }

  { // SOUTH-WEST
    int i = 0;
    int j = nurbs_patch_->m_cv_count[1] - 1;

    f_eig_ (row, 0) = 0.0;
    f_eig_ (row, 1) = 0.0;
    f_eig_ (row, 2) = 0.0;

    K_eig_ (row, ntools.A (i + 0, j + 0)) = -2.0 * weight;
    K_eig_ (row, ntools.A (i + 1, j + 0)) = 1.0 * weight;
    K_eig_ (row, ntools.A (i + 0, j - 1)) = 1.0 * weight;

    row++;
  }

}

void
NurbsFitting::addInteriorRegularisation (int order, int resU, int resV, double weight, int &row)
{

  NurbsTools ntools (nurbs_patch_);

  double N0[nurbs_patch_->m_order[0] * nurbs_patch_->m_order[0]];
  double N1[nurbs_patch_->m_order[1] * nurbs_patch_->m_order[1]];

  double dU = (max_u_ - min_u_) / resU;
  double dV = (max_v_ - min_v_) / resV;

  for (int u = 0; u < resU; u++)
  {
    for (int v = 0; v < resV; v++)
    {

      Vector2d params;
      params (0) = min_u_ + u * dU + 0.5 * dU;
      params (1) = min_v_ + v * dV + 0.5 * dV;

      //			printf("%f %f, %f %f\n", min_u_, dU, params(0), params(1));

      int E = ntools.E (params (0));
      int F = ntools.F (params (1));

      ON_EvaluateNurbsBasis (nurbs_patch_->m_order[0], nurbs_patch_->m_knot[0] + E, params (0), N0);
      ON_EvaluateNurbsBasis (nurbs_patch_->m_order[1], nurbs_patch_->m_knot[1] + F, params (1), N1);
      ON_EvaluateNurbsBasisDerivatives (nurbs_patch_->m_order[0], nurbs_patch_->m_knot[0] + E, order, N0); // derivative order?
      ON_EvaluateNurbsBasisDerivatives (nurbs_patch_->m_order[1], nurbs_patch_->m_knot[1] + F, order, N1);

      f_eig_ (row, 0) = 0.0;
      f_eig_ (row, 1) = 0.0;
      f_eig_ (row, 2) = 0.0;

      for (int i = 0; i < nurbs_patch_->m_order[0]; i++)
      {
        for (int j = 0; j < nurbs_patch_->m_order[1]; j++)
        {

          K_eig_ (row, ntools.A (E, F, i, j)) = weight * (N0[order * nurbs_patch_->m_order[0] + i] * N1[j] + N0[i]
              * N1[order * nurbs_patch_->m_order[1] + j]);

        } // i
      } // j

      row++;
    } // u
  } // v

}

void
NurbsFitting::addBoundaryRegularisation (int order, int resU, int resV, double weight, int &row)
{

  NurbsTools ntools (nurbs_patch_);

  double N0[nurbs_patch_->m_order[0] * nurbs_patch_->m_order[0]];
  double N1[nurbs_patch_->m_order[1] * nurbs_patch_->m_order[1]];

  double dU = (max_u_ - min_u_) / resU;
  double dV = (max_v_ - min_v_) / resV;

  for (int u = 0; u < resU; u++)
  {

    Vector2d params;
    params (0) = min_u_ + u * dU + 0.5 * dU;
    params (1) = min_v_;

    int E = ntools.E (params (0));
    int F = ntools.F (params (1));

    ON_EvaluateNurbsBasis (nurbs_patch_->m_order[0], nurbs_patch_->m_knot[0] + E, params (0), N0);
    ON_EvaluateNurbsBasis (nurbs_patch_->m_order[1], nurbs_patch_->m_knot[1] + F, params (1), N1);
    ON_EvaluateNurbsBasisDerivatives (nurbs_patch_->m_order[0], nurbs_patch_->m_knot[0] + E, order, N0); // derivative order?
    ON_EvaluateNurbsBasisDerivatives (nurbs_patch_->m_order[1], nurbs_patch_->m_knot[1] + F, order, N1);

    f_eig_ (row, 0) = 0.0;
    f_eig_ (row, 1) = 0.0;
    f_eig_ (row, 2) = 0.0;

    for (int i = 0; i < nurbs_patch_->m_order[0]; i++)
    {
      for (int j = 0; j < nurbs_patch_->m_order[1]; j++)
      {

        K_eig_ (row, ntools.A (E, F, i, j)) = weight * (N0[order * nurbs_patch_->m_order[0] + i] * N1[j] + N0[i]
            * N1[order * nurbs_patch_->m_order[1] + j]);

      } // i
    } // j

    row++;

  } // u

  for (int u = 0; u < resU; u++)
  {

    Vector2d params;
    params (0) = min_u_ + u * dU + 0.5 * dU;
    params (1) = max_v_;

    int E = ntools.E (params (0));
    int F = ntools.F (params (1));

    ON_EvaluateNurbsBasis (nurbs_patch_->m_order[0], nurbs_patch_->m_knot[0] + E, params (0), N0);
    ON_EvaluateNurbsBasis (nurbs_patch_->m_order[1], nurbs_patch_->m_knot[1] + F, params (1), N1);
    ON_EvaluateNurbsBasisDerivatives (nurbs_patch_->m_order[0], nurbs_patch_->m_knot[0] + E, order, N0); // derivative order?
    ON_EvaluateNurbsBasisDerivatives (nurbs_patch_->m_order[1], nurbs_patch_->m_knot[1] + F, order, N1);

    f_eig_ (row, 0) = 0.0;
    f_eig_ (row, 1) = 0.0;
    f_eig_ (row, 2) = 0.0;

    for (int i = 0; i < nurbs_patch_->m_order[0]; i++)
    {
      for (int j = 0; j < nurbs_patch_->m_order[1]; j++)
      {

        K_eig_ (row, ntools.A (E, F, i, j)) = weight * (N0[order * nurbs_patch_->m_order[0] + i] * N1[j] + N0[i]
            * N1[order * nurbs_patch_->m_order[1] + j]);

      } // i
    } // j

    row++;

  } // u

  for (int v = 0; v < resV; v++)
  {

    Vector2d params;
    params (0) = min_u_;
    params (1) = min_v_ + v * dV + 0.5 * dV;

    int E = ntools.E (params (0));
    int F = ntools.F (params (1));

    ON_EvaluateNurbsBasis (nurbs_patch_->m_order[0], nurbs_patch_->m_knot[0] + E, params (0), N0);
    ON_EvaluateNurbsBasis (nurbs_patch_->m_order[1], nurbs_patch_->m_knot[1] + F, params (1), N1);
    ON_EvaluateNurbsBasisDerivatives (nurbs_patch_->m_order[0], nurbs_patch_->m_knot[0] + E, order, N0); // derivative order?
    ON_EvaluateNurbsBasisDerivatives (nurbs_patch_->m_order[1], nurbs_patch_->m_knot[1] + F, order, N1);

    f_eig_ (row, 0) = 0.0;
    f_eig_ (row, 1) = 0.0;
    f_eig_ (row, 2) = 0.0;

    for (int i = 0; i < nurbs_patch_->m_order[0]; i++)
    {
      for (int j = 0; j < nurbs_patch_->m_order[1]; j++)
      {

        K_eig_ (row, ntools.A (E, F, i, j)) = weight * (N0[order * nurbs_patch_->m_order[0] + i] * N1[j] + N0[i]
            * N1[order * nurbs_patch_->m_order[1] + j]);

      } // i
    } // j

    row++;

  } // v

  for (int v = 0; v < resV; v++)
  {

    Vector2d params;
    params (0) = max_u_;
    params (1) = min_v_ + v * dV + 0.5 * dV;

    int E = ntools.E (params (0));
    int F = ntools.F (params (1));

    ON_EvaluateNurbsBasis (nurbs_patch_->m_order[0], nurbs_patch_->m_knot[0] + E, params (0), N0);
    ON_EvaluateNurbsBasis (nurbs_patch_->m_order[1], nurbs_patch_->m_knot[1] + F, params (1), N1);
    ON_EvaluateNurbsBasisDerivatives (nurbs_patch_->m_order[0], nurbs_patch_->m_knot[0] + E, order, N0); // derivative order?
    ON_EvaluateNurbsBasisDerivatives (nurbs_patch_->m_order[1], nurbs_patch_->m_knot[1] + F, order, N1);

    f_eig_ (row, 0) = 0.0;
    f_eig_ (row, 1) = 0.0;
    f_eig_ (row, 2) = 0.0;

    for (int i = 0; i < nurbs_patch_->m_order[0]; i++)
    {
      for (int j = 0; j < nurbs_patch_->m_order[1]; j++)
      {

        K_eig_ (row, ntools.A (E, F, i, j)) = weight * (N0[order * nurbs_patch_->m_order[0] + i] * N1[j] + N0[i]
            * N1[order * nurbs_patch_->m_order[1] + j]);

      } // i
    } // j

    row++;

  } // v

}
