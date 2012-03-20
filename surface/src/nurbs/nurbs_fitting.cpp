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

#include <stdexcept>
#include <pcl/surface/nurbs/nurbs_fitting.h>
#include <ctime>
#include <float.h>
#include <map>

#include <Eigen/Dense>

using namespace pcl;
using namespace nurbs;
using namespace Eigen;

unsigned
GetClosestPoint (const vec3 &p, const vector_vec3 &v)
{
  double d = DBL_MAX;
  unsigned idx = 0;
  for (unsigned i = 0; i < v.size (); i++)
  {
    double dn = (v[i] - p).norm ();
    if (dn < d)
    {
      idx = i;
      d = dn;
    }
  }
  return idx;
}

void
GetBoundingBox (const vector_vec3 &v, vec3 &vmin, vec3 &vmax)
{
  vmin = vec3 (DBL_MAX, DBL_MAX, DBL_MAX);
  vmax = vec3 (0.0, 0.0, 0.0);

  for (unsigned i = 0; i < v.size (); i++)
  {
    vec3 p = v[i];
    for (unsigned k = 0; k < 3; k++)
    {
      if (p (k) < vmin (k))
        vmin (k) = p (k);
      if (p (k) > vmax (k))
        vmax (k) = p (k);
    }
  }
}

NurbsFitting::NurbsFitting (int order, NurbsData *data, vec4 ll, vec4 lr, vec4 ur, vec4 ul)
{
  if (order < 2)
    throw std::runtime_error ("[NurbsFitting::NurbsFitting] Error order to low (order<2).");

  this->data = data;

  std::map<int, std::map<int, vec4> > cv_map;
  vector_vec4 cps;

  double dc = 1.0 / (order - 1);

  for (int i = 0; i < order; i++)
  {
    double di = dc * i;
    cv_map[i][0] = ll + (lr - ll) * di;
    cv_map[0][i] = ll + (ul - ll) * di;
    cv_map[i][order - 1] = ul + (ur - ul) * di;
    cv_map[order - 1][i] = lr + (ur - lr) * di;
  }

  for (int i = 1; i < order - 1; i++)
  {
    for (int j = 1; j < order - 1; j++)
    {
      vec4 du = cv_map[0][j] + (cv_map[order - 1][j] - cv_map[0][j]) * dc * j;
      vec4 dv = cv_map[i][0] + (cv_map[i][order - 1] - cv_map[i][0]) * dc * i;
      cv_map[i][j] = du * 0.5 + dv * 0.5;
      cv_map[i][j] (3) = 1.0;
    }
  }

  for (int i = 0; i < order; i++)
    for (int j = 0; j < order; j++)
      cps.push_back (cv_map[i][j]);

  m_patch = new NurbsSurface (order - 1, order, order, cps);

  this->init ();
}

NurbsFitting::NurbsFitting (int order, NurbsData *data, const vector_vec3 &cv)
{
  if (order < 2)
    throw std::runtime_error ("[NurbsFitting::NurbsFitting] Error order to low (order<2).");

  if (order * order != (int)cv.size ())
    throw std::runtime_error (
                              "[NurbsFitting::NurbsFitting] Error number of control points don't match: (order*order != cps.size()).\n");

  this->data = data;

  vector_vec4 cps;

  unsigned cvi (0);
  for (int i = 0; i < order; i++)
  {
    for (int j = 0; j < order; j++)
    {
      vec3 cp = cv[cvi++];
      cps.push_back (vec4 (cp (0), cp (1), cp (2), 1.0));
    }
  }

  m_patch = new NurbsSurface (order - 1, order, order, cps);

  this->init ();
}

NurbsFitting::NurbsFitting (int order, NurbsData *data, vec3 z)
{
  if (order < 2)
    throw std::runtime_error ("[NurbsFitting::NurbsFitting] Error order to low (order<2).");

  this->data = data;

  vector_vec4 cps(order*order);
  m_patch = new NurbsSurface (order - 1, order, order, cps);
  NurbsTools::initNurbsPCA(m_patch, data, z);

  this->init ();
}

NurbsFitting::NurbsFitting (NurbsData *data, const NurbsSurface &ns)
{
  this->m_patch = new NurbsSurface (ns);
  this->data = data;

  this->init ();
}

NurbsFitting::~NurbsFitting ()
{

  delete (m_patch);

}

void
NurbsFitting::refine (int dim)
{
  NurbsTools ntools (m_patch);

  std::vector<double> xi;
  std::vector<double> elements = ntools.getElementVector (dim);

  for (unsigned i = 0; i < elements.size () - 1; i++)
    xi.push_back (elements[i] + 0.5 * (elements[i + 1] - elements[i]));

  for (unsigned i = 0; i < xi.size (); i++)
  {
    if (dim == 0)
      m_patch->insertKnotU (xi[i]);
    if (dim == 1)
      m_patch->insertKnotV (xi[i]);
  }

  m_elementsU = ntools.getElementVector (0);
  m_elementsV = ntools.getElementVector (1);
  min_u_ = m_elementsU[0];
  min_v_ = m_elementsV[0];
  max_u_ = m_elementsU[m_elementsU.size () - 1];
  max_v_ = m_elementsV[m_elementsV.size () - 1];

  m_xeig = Eigen::MatrixXd::Zero (m_patch->nbControlPointsU () * m_patch->nbControlPointsV (), 3);
}

void
NurbsFitting::assemble (double smoothness)
{
  clock_t time_start, time_end;
  time_start = clock ();

  NurbsTools ntools (m_patch);

  int ncp = m_patch->nb_control_points_u_ * m_patch->nb_control_points_v_;
  int nInt = data->interior.size ();
  int nCageRegInt = (m_patch->nb_control_points_u_ - 2) * (m_patch->nb_control_points_v_ - 2);
  int nCageRegBnd = 2 * (m_patch->nb_control_points_u_ - 1) + 2 * (m_patch->nb_control_points_v_ - 1);

  double wInt = 1.0;
  double wCageRegInt = smoothness;
  double wCageRegBnd = smoothness;

  int nrows = nInt + nCageRegInt + nCageRegBnd;

#ifdef USE_UMFPACK
  m_Ksparse.clear();
#else
  m_Keig = Eigen::MatrixXd::Zero (nrows, ncp);
#endif

  m_feig = Eigen::MatrixXd::Zero (nrows, 3);

  int row (0);

  // interior points should lie on surface
  data->interior_line_start.clear ();
  data->interior_line_end.clear ();
  data->interior_error.clear ();
  data->interior_normals.clear ();

  for (int p = 0; p < nInt; p++)
  {
    vec3 pcp;
    pcp = data->interior[p];

    // inverse mapping
    vec2 params;
    vec3 pt, tu, tv, n;
    double error;
    if (p < (int)data->interior_param.size ())
    {
      params = ntools.inverseMapping (pcp, &data->interior_param[p], error, pt, tu, tv, invMapInt_maxSteps,
                                      invMapInt_accuracy);
      data->interior_param[p] = params;
    }
    else
    {
      params = ntools.inverseMapping (pcp, NULL, error, pt, tu, tv, invMapInt_maxSteps, invMapInt_accuracy);
      data->interior_param.push_back (params);
    }
    data->interior_error.push_back (error);

    n = tu.cross (tv);
    n.normalize ();

    data->interior_normals.push_back (n);
    data->interior_line_start.push_back (pcp);
    data->interior_line_end.push_back (pt);

    addPointConstraint (params, pcp, wInt, row);
  }

  // cage regularisation
  if (nCageRegInt > 0)
    addCageInteriorRegularisation (wCageRegInt, row);

  if (nCageRegBnd > 0)
  {
    addCageBoundaryRegularisation (wCageRegBnd, NORTH, row);
    addCageBoundaryRegularisation (wCageRegBnd, SOUTH, row);
    addCageBoundaryRegularisation (wCageRegBnd, WEST, row);
    addCageBoundaryRegularisation (wCageRegBnd, EAST, row);
    addCageCornerRegularisation (wCageRegBnd * 2.0, row);
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
  if (wBnd.size () > data->boundary.size ())
    throw std::runtime_error ("[NurbsFitting::assemble] Size of weight vector greater than point size (boundary).");

  if (wInt.size () > data->interior.size ())
    throw std::runtime_error ("[NurbsFitting::assemble] Size of weight vector greater than point size (interior).");

  NurbsTools ntools (m_patch);

  clock_t time_start, time_end;
  time_start = clock ();

  int nBnd = wBnd.size ();
  int nInt = wInt.size ();
  int nCageRegInt = (m_patch->nb_control_points_u_ - 2) * (m_patch->nb_control_points_v_ - 2);
  int nCageRegBnd = 2 * (m_patch->nb_control_points_u_ - 1) + 2 * (m_patch->nb_control_points_v_ - 1);

  if (wCageRegInt <= 0.0)
    nCageRegInt = 0;
  if (wCageRegBnd <= 0.0)
    nCageRegBnd = 0;

  int ncp = m_patch->nb_control_points_u_ * m_patch->nb_control_points_v_;
  int nrows = nBnd + nInt + nCageRegInt + nCageRegBnd;

#ifdef USE_UMFPACK
  m_Ksparse.clear();
#else
  m_Keig = Eigen::MatrixXd::Zero (nrows, ncp);
#endif

  m_feig = Eigen::MatrixXd::Zero (nrows, 3);

  int row = 0;

  // boundary points should lie on edges of surface
  data->boundary_line_start.clear ();
  data->boundary_line_end.clear ();
  data->boundary_error.clear ();
  data->boundary_normals.clear ();
  for (int p = 0; p < nBnd; p++)
  {
    vec3 pcp;
    pcp = data->boundary[p];

    double error;
    vec3 pt, tu, tv, n;
    vec2 params = ntools.inverseMappingBoundary (pcp, error, pt, tu, tv, invMapBnd_maxSteps, invMapBnd_accuracy);
    data->boundary_error.push_back (error);

    if (p < (int)data->boundary_param.size ())
    {
      data->boundary_param[p] = params;
    }
    else
    {
      data->boundary_param.push_back (params);
    }

    n = tu.cross (tv);
    n.normalize ();

    data->boundary_normals.push_back (n);
    data->boundary_line_start.push_back (pcp);
    data->boundary_line_end.push_back (pt);

    addPointConstraint (params, pcp, wBnd[p], row);
  }

  // interior points should lie on surface
  data->interior_line_start.clear ();
  data->interior_line_end.clear ();
  data->interior_error.clear ();
  data->interior_normals.clear ();
  for (int p = 0; p < nInt; p++)
  {
    vec3 pcp;
    pcp = data->interior[p];

    // inverse mapping
    vec2 params;
    vec3 pt, tu, tv, n;
    double error;
    if (p < (int)data->interior_param.size ())
    {
      params = data->interior_param[p];
      params = ntools.inverseMapping (pcp, &params, error, pt, tu, tv, invMapInt_maxSteps, invMapInt_accuracy);
      data->interior_param[p] = params;
    }
    else
    {
      params = ntools.inverseMapping (pcp, NULL, error, pt, tu, tv, invMapInt_maxSteps, invMapInt_accuracy);
      data->interior_param.push_back (params);
    }
    data->interior_error.push_back (error);

    n = tu.cross (tv);
    n.normalize ();

    data->interior_normals.push_back (n);
    data->interior_line_start.push_back (pcp);
    data->interior_line_end.push_back (pt);

    addPointConstraint (params, pcp, wInt[p], row);
  }

  // cage regularisation
  if (nCageRegInt > 0)
    addCageInteriorRegularisation (wCageRegInt, row);

  if (nCageRegBnd > 0)
  {
    addCageBoundaryRegularisation (wCageRegBnd, NORTH, row);
    addCageBoundaryRegularisation (wCageRegBnd, SOUTH, row);
    addCageBoundaryRegularisation (wCageRegBnd, WEST, row);
    addCageBoundaryRegularisation (wCageRegBnd, EAST, row);
    addCageCornerRegularisation (wCageRegBnd * 2.0, row);
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

  NurbsTools ntools (m_patch);

  clock_t time_start, time_end;
  time_start = clock ();

  int nBnd = data->boundary.size ();
  int nInt = data->interior.size ();
  int nCurInt = resU * resV;
  int nCurBnd = 2 * resU + 2 * resV;
  int nCageReg = (m_patch->nb_control_points_u_ - 2) * (m_patch->nb_control_points_v_ - 2);
  int nCageRegBnd = 2 * (m_patch->nb_control_points_u_ - 1) + 2 * (m_patch->nb_control_points_v_ - 1);
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

  int ncp = m_patch->nb_control_points_u_ * m_patch->nb_control_points_v_;
  int nrows = nBnd + nInt + nCurInt + nCurBnd + nCorner + nCageReg + nCageRegBnd;

#ifdef USE_UMFPACK
  m_Ksparse.clear();
#else
  m_Keig = Eigen::MatrixXd::Zero (nrows, ncp);
#endif

  m_feig = Eigen::MatrixXd::Zero (nrows, 3);

  int row = 0;

  // boundary points should lie on edges of surface
  data->boundary_line_start.clear ();
  data->boundary_line_end.clear ();
  data->boundary_error.clear ();
  data->boundary_normals.clear ();
  for (int p = 0; p < nBnd; p++)
  {
    vec3 pcp;
    pcp = data->boundary[p];

    double error;
    vec3 pt, tu, tv, n;
    vec2 params = ntools.inverseMappingBoundary (pcp, error, pt, tu, tv, invMapBnd_maxSteps, invMapBnd_accuracy);
    data->boundary_error.push_back (error);

    if (p < (int)data->boundary_param.size ())
    {
      data->boundary_param[p] = params;
    }
    else
    {
      data->boundary_param.push_back (params);
    }

    n = tu.cross (tv);
    n.normalize ();

    data->boundary_normals.push_back (n);
    data->boundary_line_start.push_back (pcp);
    data->boundary_line_end.push_back (pt);

    addPointConstraint (params, pcp, wBnd, row);

  }

  // interior points should lie on surface
  data->interior_line_start.clear ();
  data->interior_line_end.clear ();
  data->interior_error.clear ();
  data->interior_normals.clear ();
  for (int p = 0; p < nInt; p++)
  {

    vec3 pcp = data->interior[p];

    // inverse mapping
    vec2 params;
    double error;
    vec3 pt, tu, tv, n;
    if (p < (int)data->interior_param.size ())
    {
      params = data->interior_param[p];
      params = ntools.inverseMapping (pcp, &params, error, pt, tu, tv, invMapInt_maxSteps, invMapInt_accuracy);
      data->interior_param[p] = params;
    }
    else
    {
      params = ntools.inverseMapping (pcp, NULL, error, pt, tu, tv, invMapInt_maxSteps, invMapInt_accuracy);
      data->interior_param.push_back (params);
    }
    data->interior_error.push_back (error);

    n = tu.cross (tv);
    n.normalize ();

    data->interior_normals.push_back (n);
    data->interior_line_start.push_back (pcp);
    data->interior_line_end.push_back (pt);

    addPointConstraint (params, pcp, wInt, row);

  }

  // minimal curvature on surface
  if (nCurInt > 0)
  {
    if (m_patch->degreeU () < 3 || m_patch->degreeV () < 3)
      printf ("[NurbsFitting::assemble] Error insufficient NURBS order to add curvature regularisation.\n");
    else
      addInteriorRegularisation (2, resU, resV, wCurInt / resU, row);
  }

  // minimal curvature on boundary
  if (nCurBnd > 0)
  {
    if (m_patch->degreeU () < 3 || m_patch->degreeV () < 3)
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
    addCageCornerRegularisation (wCageRegBnd * 2.0, row);
  }

  // corners of surface should lie on boundary
  if (nCorner > 0)
  {
    addBoundaryPointConstraint (min_u_, min_v_, 1.0, row);
    addBoundaryPointConstraint (min_u_, max_v_, 1.0, row);
    addBoundaryPointConstraint (max_u_, min_v_, 1.0, row);
    addBoundaryPointConstraint (max_u_, max_v_, 1.0, row);
  }

  time_end = clock ();
  if (!quiet_)
  {
    double solve_time = (double)(time_end - time_start) / (double)(CLOCKS_PER_SEC);
    printf ("[NurbsFitting::assemble()] (assemble (%d,%d): %f sec)\n", nrows, ncp, solve_time);
  }
}

void
NurbsFitting::init ()
{
  NurbsTools ntools (m_patch);
  m_elementsU = ntools.getElementVector (0);
  m_elementsV = ntools.getElementVector (1);
  min_u_ = m_elementsU[0];
  min_v_ = m_elementsV[0];
  max_u_ = m_elementsU[m_elementsU.size () - 1];
  max_v_ = m_elementsV[m_elementsV.size () - 1];

  m_xeig = Eigen::MatrixXd::Zero (m_patch->nb_control_points_u_ * m_patch->nb_control_points_v_, 3);

  invMapBnd_maxSteps = 100;
  invMapInt_maxSteps = 100;
  invMapBnd_accuracy = 1e-4;
  invMapInt_accuracy = 1e-4;

  quiet_ = true;
  use_int_hints_ = false;
}

void
NurbsFitting::solve (double damp)
{
#ifdef USE_UMFPACK
  solve_umfpack(damp);
#else
  solveEigen (damp);
#endif

}

#ifdef USE_UMFPACK
void NurbsFitting::solve_umfpack(double damp)
{
  NurbsTools ntools(m_patch);

  clock_t time_start, time_end;
  time_start = clock();

  cholmod_common c;
  cholmod_start(&c);

  if( m_patch->DegreeU() > 3 || m_patch->DegreeV() > 3 )
  printf("[NurbsFitting::solve_umfpack] Bug-Warning: UMFPACK cannot solve NURBS with higher order than 3 (at least for interior points).\n");

  int n_rows, n_cols, n_nz;
  m_Ksparse.size(n_rows, n_cols);
  n_nz = m_Ksparse.nonzeros();

  cholmod_sparse* K = cholmod_allocate_sparse(n_rows, n_cols, n_rows * n_cols, 0, 1, 0, CHOLMOD_REAL, &c);
  cholmod_dense* f = cholmod_allocate_dense(n_rows, 3, n_rows, CHOLMOD_REAL, &c);
  cholmod_dense* d = cholmod_allocate_dense(n_cols, 3, n_cols, CHOLMOD_REAL, &c);

  std::vector<int> rowinds;
  std::vector<int> colinds;
  std::vector<double> values;
  m_Ksparse.get(rowinds, colinds, values);

  double* vals = (double*) K->x;
  int* cols = (int*) K->p;
  int* rows = (int*) K->i;

  umfpack_di_triplet_to_col(n_rows, n_cols, n_nz, &rowinds[0], &colinds[0], &values[0], cols, rows, vals, NULL);

  double* temp = (double*) f->x;

  for( int j = 0; j < 3; j++ )
  {
    for( int i = 0; i < n_rows; i++ )
    {

      temp[j * n_rows + i] = m_feig(i, j);

    }
  }

  bool not_solve = ntools.solveSparseLinearSystemLQ(K, f, d);

  temp = (double*) d->x;

  for( int j = 0; j < 3; j++ )
  {
    for( int i = 0; i < n_cols; i++ )
    {

      m_xeig(i, j) = temp[j * n_cols + i];

    }
  }

  cholmod_free_sparse(&K, &c);
  cholmod_free_dense(&f, &c);
  cholmod_free_dense(&d, &c);

  cholmod_finish(&c);

  time_end = clock();

  if( not_solve )
  printf("[NurbsFitting::solve_umfpack()] Error: solution NOT found\n");
  else
  {
    if( !m_quiet )
    {
      double solve_time = (double) (time_end - time_start) / (double) (CLOCKS_PER_SEC);
      printf("[NurbsFitting::solve_umfpack()] solution found! (%f sec)\n", solve_time);
    }
    updateSurf(damp);
  }

}
#else

void
NurbsFitting::solveEigen (double damp)
{

  clock_t time_start, time_end;
  time_start = clock ();

  //  m_xeig = m_Keig.colPivHouseholderQr().solve(m_feig);
  //  Eigen::MatrixXd x = A.householderQr().solve(b);
  m_xeig = m_Keig.jacobiSvd (ComputeThinU | ComputeThinV).solve (m_feig);

  time_end = clock ();

  if (!quiet_)
  {
    double solve_time = (double)(time_end - time_start) / (double)(CLOCKS_PER_SEC);
    printf ("[NurbsFitting::solve_eigen()] solution found! (%f sec)\n", solve_time);
  }
  updateSurface (damp);
}
#endif

void
NurbsFitting::updateSurface (double damp)
{
  for (unsigned j = 0; j < m_patch->nb_control_points_v_; j++)
  {
    for (unsigned i = 0; i < m_patch->nb_control_points_u_; i++)
    {

      unsigned A = m_patch->index (i, j);

      vec4 cp_prev = m_patch->getControlPoint (i, j);
      vec4 cp;

      cp (0) = cp_prev (0) + damp * (m_xeig (A, 0) - cp_prev (0));
      cp (1) = cp_prev (1) + damp * (m_xeig (A, 1) - cp_prev (1));
      cp (2) = cp_prev (2) + damp * (m_xeig (A, 2) - cp_prev (2));
      cp (3) = 1.0;

      m_patch->setControlPoint (i, j, cp);
    }
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
NurbsFitting::addPointConstraint (const vec2 &params, const vec3 &point, double weight, int& row)
{

  NurbsTools ntools (m_patch);

  int E = ntools.E (params (0));
  int F = ntools.F (params (1));

  std::vector<double> N0, N1;
  m_patch->basis_u_.cox (params (0), N0);
  m_patch->basis_v_.cox (params (1), N1);

  unsigned degreeU = m_patch->degreeU ();
  unsigned degreeV = m_patch->degreeV ();

  m_feig (row, 0) = point (0) * weight;
  m_feig (row, 1) = point (1) * weight;
  m_feig (row, 2) = point (2) * weight;

  for (unsigned i = 0; i < degreeU + 1; i++)
  {
    unsigned iu = i + degreeU * (degreeU + 1);
    for (unsigned j = 0; j < degreeV + 1; j++)
    {

#ifdef USE_UMFPACK
      m_Ksparse.set(row, ntools.A(E, F, i, j), weight * N0[iu] * N1[j + degreeV * (degreeV + 1)]);
#else
      m_Keig (row, ntools.A (E, F, i, j)) = weight * N0[iu] * N1[j + degreeV * (degreeV + 1)];
#endif

    } // j

  } // i

  row++;

}

void
NurbsFitting::addBoundaryPointConstraint (double paramU, double paramV, double weight, int &row)
{
  // edges on surface
  NurbsTools ntools (m_patch);

  vec3 pt;
  int closest_idx;

  m_patch->evaluate (paramU, paramV, pt);
  closest_idx = GetClosestPoint (pt, data->boundary);

  int E = ntools.E (paramU);
  int F = ntools.F (paramV);
  std::vector<double> N0, N1;
  m_patch->basis_u_.cox (paramU, E, N0);
  m_patch->basis_v_.cox (paramV, F, N1);

  unsigned degreeU = m_patch->degreeU ();
  unsigned degreeV = m_patch->degreeV ();

  m_feig (row, 0) = data->boundary[closest_idx] (0) * weight;
  m_feig (row, 1) = data->boundary[closest_idx] (1) * weight;
  m_feig (row, 2) = data->boundary[closest_idx] (2) * weight;

  for (unsigned i = 0; i < m_patch->degreeU () + 1; i++)
  {
    unsigned iu = i + degreeU * (degreeU + 1);
    for (unsigned j = 0; j < m_patch->degreeV () + 1; j++)
    {
#ifdef USE_UMFPACK
      m_Ksparse.set(row, ntools.A(E, F, i, j), N0[iu] * N1[j + degreeV * (degreeV + 1)] * weight);
#else
      m_Keig (row, ntools.A (E, F, i, j)) = N0[iu] * N1[j + degreeV * (degreeV + 1)] * weight;
#endif

    } // j

  } // i

  row++;

}

void
NurbsFitting::addCageInteriorRegularisation (double weight, int &row)
{
  NurbsTools ntools (m_patch);

  for (unsigned i = 1; i < (m_patch->nb_control_points_u_ - 1); i++)
  {
    for (unsigned j = 1; j < (m_patch->nb_control_points_v_ - 1); j++)
    {

      m_feig (row, 0) = 0.0;
      m_feig (row, 1) = 0.0;
      m_feig (row, 2) = 0.0;

#ifdef USE_UMFPACK
      m_Ksparse.set(row, ntools.A(i + 0, j + 0), -4.0 * weight);
      m_Ksparse.set(row, ntools.A(i + 0, j - 1), 1.0 * weight);
      m_Ksparse.set(row, ntools.A(i + 0, j + 1), 1.0 * weight);
      m_Ksparse.set(row, ntools.A(i - 1, j + 0), 1.0 * weight);
      m_Ksparse.set(row, ntools.A(i + 1, j + 0), 1.0 * weight);
#else
      m_Keig (row, ntools.A (i + 0, j + 0)) = -4.0 * weight;
      m_Keig (row, ntools.A (i + 0, j - 1)) = 1.0 * weight;
      m_Keig (row, ntools.A (i + 0, j + 1)) = 1.0 * weight;
      m_Keig (row, ntools.A (i - 1, j + 0)) = 1.0 * weight;
      m_Keig (row, ntools.A (i + 1, j + 0)) = 1.0 * weight;
#endif

      row++;
    }
  }
}

void
NurbsFitting::addCageBoundaryRegularisation (double weight, int side, int &row)
{
  NurbsTools ntools (m_patch);
  unsigned i = 0;
  unsigned j = 0;

  switch (side)
  {
    case SOUTH:
      j = m_patch->nb_control_points_v_ - 1;
    case NORTH:
      for (i = 1; i < (m_patch->nb_control_points_u_ - 1); i++)
      {

        m_feig (row, 0) = 0.0;
        m_feig (row, 1) = 0.0;
        m_feig (row, 2) = 0.0;

#ifdef USE_UMFPACK
        m_Ksparse.set(row, ntools.A(i + 0, j), -2.0 * weight);
        m_Ksparse.set(row, ntools.A(i - 1, j), 1.0 * weight);
        m_Ksparse.set(row, ntools.A(i + 1, j), 1.0 * weight);
#else
        m_Keig (row, ntools.A (i + 0, j)) = -2.0 * weight;
        m_Keig (row, ntools.A (i - 1, j)) = 1.0 * weight;
        m_Keig (row, ntools.A (i + 1, j)) = 1.0 * weight;
#endif

        row++;
      }
      break;

    case EAST:
      i = m_patch->nb_control_points_u_ - 1;
    case WEST:
      for (j = 1; j < (m_patch->nb_control_points_v_ - 1); j++)
      {

        m_feig (row, 0) = 0.0;
        m_feig (row, 1) = 0.0;
        m_feig (row, 2) = 0.0;

#ifdef USE_UMFPACK
        m_Ksparse.set(row, ntools.A(i, j + 0), -2.0 * weight);
        m_Ksparse.set(row, ntools.A(i, j - 1), 1.0 * weight);
        m_Ksparse.set(row, ntools.A(i, j + 1), 1.0 * weight);
#else
        m_Keig (row, ntools.A (i, j + 0)) = -2.0 * weight;
        m_Keig (row, ntools.A (i, j - 1)) = 1.0 * weight;
        m_Keig (row, ntools.A (i, j + 1)) = 1.0 * weight;
#endif
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

#ifdef USE_UMFPACK
    m_Ksparse.set(row, ntools.A(i + 0, j + 0), -2.0 * weight);
    m_Ksparse.set(row, ntools.A(i + 1, j + 0), 1.0 * weight);
    m_Ksparse.set(row, ntools.A(i + 0, j + 1), 1.0 * weight);
#else
    m_Keig (row, ntools.A (i + 0, j + 0)) = -2.0 * weight;
    m_Keig (row, ntools.A (i + 1, j + 0)) = 1.0 * weight;
    m_Keig (row, ntools.A (i + 0, j + 1)) = 1.0 * weight;
#endif

    row++;
  }

  { // NORTH-EAST
    int i = m_patch->nb_control_points_u_ - 1;
    int j = 0;

    m_feig (row, 0) = 0.0;
    m_feig (row, 1) = 0.0;
    m_feig (row, 2) = 0.0;

#ifdef USE_UMFPACK
    m_Ksparse.set(row, ntools.A(i + 0, j + 0), -2.0 * weight);
    m_Ksparse.set(row, ntools.A(i - 1, j + 0), 1.0 * weight);
    m_Ksparse.set(row, ntools.A(i + 0, j + 1), 1.0 * weight);
#else
    m_Keig (row, ntools.A (i + 0, j + 0)) = -2.0 * weight;
    m_Keig (row, ntools.A (i - 1, j + 0)) = 1.0 * weight;
    m_Keig (row, ntools.A (i + 0, j + 1)) = 1.0 * weight;
#endif

    row++;
  }

  { // SOUTH-EAST
    int i = m_patch->nb_control_points_u_ - 1;
    int j = m_patch->nb_control_points_v_ - 1;

    m_feig (row, 0) = 0.0;
    m_feig (row, 1) = 0.0;
    m_feig (row, 2) = 0.0;

#ifdef USE_UMFPACK
    m_Ksparse.set(row, ntools.A(i + 0, j + 0), -2.0 * weight);
    m_Ksparse.set(row, ntools.A(i - 1, j + 0), 1.0 * weight);
    m_Ksparse.set(row, ntools.A(i + 0, j - 1), 1.0 * weight);
#else
    m_Keig (row, ntools.A (i + 0, j + 0)) = -2.0 * weight;
    m_Keig (row, ntools.A (i - 1, j + 0)) = 1.0 * weight;
    m_Keig (row, ntools.A (i + 0, j - 1)) = 1.0 * weight;
#endif

    row++;
  }

  { // SOUTH-WEST
    int i = 0;
    int j = m_patch->nb_control_points_v_ - 1;

    m_feig (row, 0) = 0.0;
    m_feig (row, 1) = 0.0;
    m_feig (row, 2) = 0.0;

#ifdef USE_UMFPACK
    m_Ksparse.set(row, ntools.A(i + 0, j + 0), -2.0 * weight);
    m_Ksparse.set(row, ntools.A(i + 1, j + 0), 1.0 * weight);
    m_Ksparse.set(row, ntools.A(i + 0, j - 1), 1.0 * weight);
#else
    m_Keig (row, ntools.A (i + 0, j + 0)) = -2.0 * weight;
    m_Keig (row, ntools.A (i + 1, j + 0)) = 1.0 * weight;
    m_Keig (row, ntools.A (i + 0, j - 1)) = 1.0 * weight;
#endif

    row++;
  }

}

void
NurbsFitting::addInteriorRegularisation (int order, int resU, int resV, double weight, int &row)
{
  // TODO: implement
  printf ("NurbsFitting::addInteriorRegularisation] Warning: Not implementet yet!\n");
  //  NurbsTools ntools(m_patch);
  //
  //  double N0[m_patch->DegreeU() * m_patch->DegreeU()];
  //  double N1[m_patch->DegreeV() * m_patch->DegreeV()];
  //
  //  double dU = (m_maxU - m_minU) / resU;
  //  double dV = (m_maxV - m_minV) / resV;
  //
  //  for (int u = 0; u < resU; u++) {
  //    for (int v = 0; v < resV; v++) {
  //
  //      vec2 params;
  //      params(0) = m_minU + u * dU + 0.5 * dU;
  //      params(1) = m_minV + v * dV + 0.5 * dV;
  //
  //      //                    printf("%f %f, %f %f\n", m_minU, dU, params(0), params(1));
  //
  //      int E = ntools.E(params(0));
  //      int F = ntools.F(params(1));
  //
  //      ON_EvaluateNurbsBasis(m_patch->DegreeU(), m_patch->m_knot[0] + E, params(0), N0);
  //      ON_EvaluateNurbsBasis(m_patch->DegreeV(), m_patch->m_knot[1] + F, params(1), N1);
  //      ON_EvaluateNurbsBasisDerivatives(m_patch->DegreeU(), m_patch->m_knot[0] + E, order, N0); // derivative order?
  //      ON_EvaluateNurbsBasisDerivatives(m_patch->DegreeV(), m_patch->m_knot[1] + F, order, N1);
  //
  //      m_feig(row, 0) = 0.0;
  //      m_feig(row, 1) = 0.0;
  //      m_feig(row, 2) = 0.0;
  //
  //      for (int i = 0; i < m_patch->DegreeU(); i++) {
  //
  //        for (int j = 0; j < m_patch->DegreeV(); j++) {
  //#ifdef USE_UMFPACK
  //          m_Ksparse.set(row, ntools.A(E, F, i, j), weight * (N0[order * m_patch->DegreeU() + i] * N1[j] + N0[i] * N1[order * m_patch->DegreeV() + j]));
  //#else
  //          m_Keig(row, ntools.A(E, F, i, j)) = weight * (N0[order * m_patch->DegreeU() + i] * N1[j] + N0[i] * N1[order
  //              * m_patch->DegreeV() + j]);
  //#endif
  //        } // i
  //
  //      } // j
  //
  //      row++;
  //
  //    }
  //  }

}

void
NurbsFitting::addBoundaryRegularisation (int order, int resU, int resV, double weight, int &row)
{
  // TODO: implement
  printf ("NurbsFitting::addBoundaryRegularisation] Warning: Not implementet yet!\n");
  //
  //  NurbsTools ntools(m_patch);
  //
  //  double N0[m_patch->DegreeU() * m_patch->DegreeU()];
  //  double N1[m_patch->DegreeV() * m_patch->DegreeV()];
  //
  //  double dU = (m_maxU - m_minU) / resU;
  //  double dV = (m_maxV - m_minV) / resV;
  //
  //  for (int u = 0; u < resU; u++) {
  //
  //    vec2 params;
  //    params(0) = m_minU + u * dU + 0.5 * dU;
  //    params(1) = m_minV;
  //
  //    int E = ntools.E(params(0));
  //    int F = ntools.F(params(1));
  //
  //    ON_EvaluateNurbsBasis(m_patch->DegreeU(), m_patch->m_knot[0] + E, params(0), N0);
  //    ON_EvaluateNurbsBasis(m_patch->DegreeV(), m_patch->m_knot[1] + F, params(1), N1);
  //    ON_EvaluateNurbsBasisDerivatives(m_patch->DegreeU(), m_patch->m_knot[0] + E, order, N0); // derivative order?
  //    ON_EvaluateNurbsBasisDerivatives(m_patch->DegreeV(), m_patch->m_knot[1] + F, order, N1);
  //
  //    m_feig(row, 0) = 0.0;
  //    m_feig(row, 1) = 0.0;
  //    m_feig(row, 2) = 0.0;
  //
  //    for (int i = 0; i < m_patch->DegreeU(); i++) {
  //
  //      for (int j = 0; j < m_patch->DegreeV(); j++) {
  //
  //#ifdef USE_UMFPACK
  //        m_Ksparse.set(row, ntools.A(E, F, i, j), weight * (N0[order * m_patch->DegreeU() + i] * N1[j] + N0[i] * N1[order * m_patch->DegreeV() + j]));
  //#else
  //        m_Keig(row, ntools.A(E, F, i, j)) = weight * (N0[order * m_patch->DegreeU() + i] * N1[j] + N0[i] * N1[order
  //            * m_patch->DegreeV() + j]);
  //#endif
  //
  //      } // i
  //
  //    } // j
  //
  //    row++;
  //
  //  }
  //
  //  for (int u = 0; u < resU; u++) {
  //
  //    vec2 params;
  //    params(0) = m_minU + u * dU + 0.5 * dU;
  //    params(1) = m_maxV;
  //
  //    int E = ntools.E(params(0));
  //    int F = ntools.F(params(1));
  //
  //    ON_EvaluateNurbsBasis(m_patch->DegreeU(), m_patch->m_knot[0] + E, params(0), N0);
  //    ON_EvaluateNurbsBasis(m_patch->DegreeV(), m_patch->m_knot[1] + F, params(1), N1);
  //    ON_EvaluateNurbsBasisDerivatives(m_patch->DegreeU(), m_patch->m_knot[0] + E, order, N0); // derivative order?
  //    ON_EvaluateNurbsBasisDerivatives(m_patch->DegreeV(), m_patch->m_knot[1] + F, order, N1);
  //
  //    m_feig(row, 0) = 0.0;
  //    m_feig(row, 1) = 0.0;
  //    m_feig(row, 2) = 0.0;
  //
  //    for (int i = 0; i < m_patch->DegreeU(); i++) {
  //
  //      for (int j = 0; j < m_patch->DegreeV(); j++) {
  //#ifdef USE_UMFPACK
  //        m_Ksparse.set(row, ntools.A(E, F, i, j), weight * (N0[order * m_patch->DegreeU() + i] * N1[j] + N0[i] * N1[order * m_patch->DegreeV() + j]));
  //#else
  //        m_Keig(row, ntools.A(E, F, i, j)) = weight * (N0[order * m_patch->DegreeU() + i] * N1[j] + N0[i] * N1[order
  //            * m_patch->DegreeV() + j]);
  //#endif
  //      } // i
  //
  //    } // j
  //
  //    row++;
  //
  //  }
  //
  //  for (int v = 0; v < resV; v++) {
  //
  //    vec2 params;
  //    params(0) = m_minU;
  //    params(1) = m_minV + v * dV + 0.5 * dV;
  //
  //    int E = ntools.E(params(0));
  //    int F = ntools.F(params(1));
  //
  //    ON_EvaluateNurbsBasis(m_patch->DegreeU(), m_patch->m_knot[0] + E, params(0), N0);
  //    ON_EvaluateNurbsBasis(m_patch->DegreeV(), m_patch->m_knot[1] + F, params(1), N1);
  //    ON_EvaluateNurbsBasisDerivatives(m_patch->DegreeU(), m_patch->m_knot[0] + E, order, N0); // derivative order?
  //    ON_EvaluateNurbsBasisDerivatives(m_patch->DegreeV(), m_patch->m_knot[1] + F, order, N1);
  //
  //    m_feig(row, 0) = 0.0;
  //    m_feig(row, 1) = 0.0;
  //    m_feig(row, 2) = 0.0;
  //
  //    for (int i = 0; i < m_patch->DegreeU(); i++) {
  //
  //      for (int j = 0; j < m_patch->DegreeV(); j++) {
  //#ifdef USE_UMFPACK
  //        m_Ksparse.set(row, ntools.A(E, F, i, j), weight * (N0[order * m_patch->DegreeU() + i] * N1[j] + N0[i] * N1[order * m_patch->DegreeV() + j]));
  //#else
  //        m_Keig(row, ntools.A(E, F, i, j)) = weight * (N0[order * m_patch->DegreeU() + i] * N1[j] + N0[i] * N1[order
  //            * m_patch->DegreeV() + j]);
  //#endif
  //      } // i
  //
  //    } // j
  //
  //    row++;
  //
  //  }
  //
  //  for (int v = 0; v < resV; v++) {
  //
  //    vec2 params;
  //    params(0) = m_maxU;
  //    params(1) = m_minV + v * dV + 0.5 * dV;
  //
  //    int E = ntools.E(params(0));
  //    int F = ntools.F(params(1));
  //
  //    ON_EvaluateNurbsBasis(m_patch->DegreeU(), m_patch->m_knot[0] + E, params(0), N0);
  //    ON_EvaluateNurbsBasis(m_patch->DegreeV(), m_patch->m_knot[1] + F, params(1), N1);
  //    ON_EvaluateNurbsBasisDerivatives(m_patch->DegreeU(), m_patch->m_knot[0] + E, order, N0); // derivative order?
  //    ON_EvaluateNurbsBasisDerivatives(m_patch->DegreeV(), m_patch->m_knot[1] + F, order, N1);
  //
  //    m_feig(row, 0) = 0.0;
  //    m_feig(row, 1) = 0.0;
  //    m_feig(row, 2) = 0.0;
  //
  //    for (int i = 0; i < m_patch->DegreeU(); i++) {
  //
  //      for (int j = 0; j < m_patch->DegreeV(); j++) {
  //#ifdef USE_UMFPACK
  //        m_Ksparse.set(row, ntools.A(E, F, i, j), weight * (N0[order * m_patch->DegreeU() + i] * N1[j] + N0[i] * N1[order * m_patch->DegreeV() + j]));
  //#else
  //        m_Keig(row, ntools.A(E, F, i, j)) = weight * (N0[order * m_patch->DegreeU() + i] * N1[j] + N0[i] * N1[order
  //            * m_patch->DegreeV() + j]);
  //#endif
  //      } // i
  //
  //    } // j
  //
  //    row++;
  //
  //  }

}
