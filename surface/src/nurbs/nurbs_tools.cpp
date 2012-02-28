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

#include <iostream>
#include <stdexcept>
#include <stdio.h>
#include <float.h>
#include "pcl/surface/nurbs/nurbs_tools.h"

#undef Success
#include <Eigen/Dense>

using namespace pcl;
using namespace nurbs;
using namespace Eigen;

NurbsTools::NurbsTools (NurbsSurface* surf)
{
  m_surf = surf;
}

void
NurbsTools::initNurbsPCA (NurbsSurface *nurbs, NurbsData *data, Eigen::Vector3d z)
{
  Eigen::Vector3d mean;
  Eigen::Matrix3d eigenvectors;
  Eigen::Vector3d eigenvalues;

  unsigned s = data->interior.size ();

  NurbsTools::pca (data->interior, mean, eigenvectors, eigenvalues);

  data->mean = mean;
  data->eigenvectors = eigenvectors;

  bool flip (false);
  if (eigenvectors.col (2).dot (z) < 0.0)
    flip = true;

  eigenvalues = eigenvalues / s; // seems that the eigenvalues are dependent on the number of points (???)

  Eigen::Vector3d sigma (sqrt (eigenvalues (0)), sqrt (eigenvalues (1)), sqrt (eigenvalues (2)));

  // +- 2 sigma -> 95,45 % aller Messwerte
  double dcu = (4.0 * sigma (0)) / (nurbs->DegreeU ());
  double dcv = (4.0 * sigma (1)) / (nurbs->DegreeV ());

  Eigen::Vector3d cv_t, cv;
  for (unsigned i = 0; i < nurbs->DegreeU () + 1; i++)
  {
    for (unsigned j = 0; j < nurbs->DegreeV () + 1; j++)
    {
      flip ? cv (0) = 2.0 * sigma (0) - dcu * i : cv (0) = -2.0 * sigma (0) + dcu * i;
      cv (1) = -2.0 * sigma (1) + dcv * j;
      cv (2) = 0.0;
      cv_t = eigenvectors * cv + mean;
      nurbs->SetCP (i, j, vec4(cv_t(0), cv_t(1), cv_t(2), 1.0));
    }
  }
}

void
NurbsTools::initNurbsPCABoundingBox (NurbsSurface *nurbs, NurbsData *data, Eigen::Vector3d z)
{
  Eigen::Vector3d mean;
  Eigen::Matrix3d eigenvectors;
  Eigen::Vector3d eigenvalues;

  unsigned s = data->interior.size ();

  NurbsTools::pca (data->interior, mean, eigenvectors, eigenvalues);

  data->mean = mean;
  data->eigenvectors = eigenvectors;

  bool flip (false);
  if (eigenvectors.col (2).dot (z) < 0.0)
    flip = true;

  eigenvalues = eigenvalues / s; // seems that the eigenvalues are dependent on the number of points (???)

  Eigen::Vector3d sigma (sqrt (eigenvalues (0)), sqrt (eigenvalues (1)), sqrt (eigenvalues (2)));

  Eigen::Vector3d v_max (0.0, 0.0, 0.0);
  Eigen::Vector3d v_min (DBL_MAX, DBL_MAX, DBL_MAX);
  for (unsigned i = 0; i < s; i++)
  {
    Eigen::Vector3d p = eigenvectors.inverse () * (data->interior[i] - mean);

    if (p (0) > v_max (0))
      v_max (0) = p (0);
    if (p (1) > v_max (1))
      v_max (1) = p (1);
    if (p (2) > v_max (2))
      v_max (2) = p (2);

    if (p (0) < v_min (0))
      v_min (0) = p (0);
    if (p (1) < v_min (1))
      v_min (1) = p (1);
    if (p (2) < v_min (2))
      v_min (2) = p (2);
  }

  double dcu = (v_max (0) - v_min (0)) / (nurbs->DegreeU ());
  double dcv = (v_max (1) - v_min (1)) / (nurbs->DegreeV ());

  Eigen::Vector3d cv_t, cv;
  for (unsigned i = 0; i < nurbs->DegreeU () + 1; i++)
  {
    for (unsigned j = 0; j < nurbs->DegreeV () + 1; j++)
    {
      flip ? cv (0) = -v_min (0) - dcu * i : cv (0) = v_min (0) + dcu * i;
      cv (1) = v_min (1) + dcv * j;
      cv (2) = 0.0;
      cv_t = eigenvectors * cv + mean;
      nurbs->SetCP (i, j, vec4(cv_t(0), cv_t(1), cv_t(2), 1.0));
    }
  }
}

Eigen::Vector3d
NurbsTools::computeMean (const vector_vec3d &data)
{
  Eigen::Vector3d u (0.0, 0.0, 0.0);

  unsigned s = data.size ();
  double ds = 1.0 / s;

  for (unsigned i = 0; i < s; i++)
    u += (data[i] * ds);

  return u;
}

void
NurbsTools::pca (const vector_vec3d &data, Eigen::Vector3d &mean, Eigen::Matrix3d &eigenvectors,
                 Eigen::Vector3d &eigenvalues)
{
  if (data.empty ())
  {
    printf ("[NurbsTools::pca] Error, data is empty\n");
    abort ();
  }

  mean = computeMean (data);

  unsigned s = data.size ();

  Eigen::MatrixXd Q (3, s);

  for (unsigned i = 0; i < s; i++)
    Q.col (i) << (data[i] - mean);

  Eigen::Matrix3d C = Q * Q.transpose ();

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver (C);
  if (eigensolver.info () != Success)
  {
    printf ("[nurbsfitting::NurbsTools::pca] Can not find eigenvalues.\n");
    abort ();
  }

  for (int i = 0; i < 3; ++i)
  {
    eigenvalues (i) = eigensolver.eigenvalues () (2 - i);
    if (i == 2)
      eigenvectors.col (2) = eigenvectors.col (0).cross (eigenvectors.col (1));
    else
      eigenvectors.col (i) = eigensolver.eigenvectors ().col (2 - i);
  }
}

void
NurbsTools::downsample_random (vector_vec3d &data, unsigned size)
{
  if (data.size () <= size && size > 0)
    return;

  unsigned s = data.size ();

  vector_vec3d data_tmp;

  for (unsigned i = 0; i < size; i++)
  {
    unsigned rnd = unsigned (s * (double (rand ()) / RAND_MAX));
    data_tmp.push_back (data[rnd]);
  }

  data = data_tmp;
}

vec3
NurbsTools::x (double u, double v)
{

  vec3 result;

  m_surf->Evaluate (u, v, result);

  return result;

}

MatrixXd
NurbsTools::jacX (double u, double v) // !
{

  MatrixXd Dx = MatrixXd::Zero (3, 2);

  vec3 pt, gu, gv;
  m_surf->Evaluate (u, v, pt, gu, gv);

  Dx (0, 0) = gu (0);
  Dx (1, 0) = gu (1);
  Dx (2, 0) = gu (2);

  Dx (0, 1) = gv (0);
  Dx (1, 1) = gv (1);
  Dx (2, 1) = gv (2);

  //  double pointAndTangents[9];
  //  m_surf->Evaluate(u, v, 1, 3, pointAndTangents);
  //
  //  Dx(0, 0) = pointAndTangents[3];
  //  Dx(1, 0) = pointAndTangents[4];
  //  Dx(2, 0) = pointAndTangents[5];
  //
  //  Dx(0, 1) = pointAndTangents[6];
  //  Dx(1, 1) = pointAndTangents[7];
  //  Dx(2, 1) = pointAndTangents[8];

  return Dx;

}

std::vector<double>
NurbsTools::getElementVector (int dim) // !
{
  std::vector<double> result;

  if (dim == 0)
  {
    m_surf->GetElementVectorU (result);
  }
  else
  {
    m_surf->GetElementVectorV (result);
  }

  return result;
}

std::vector<double>
NurbsTools::getElementVectorDeltas (int dim)
{

  std::vector<double> elements = this->getElementVector (dim);
  std::vector<double> deltas;

  for (unsigned i = 0; i < elements.size () - 1; i++)
  {

    deltas.push_back (elements[i + 1] - elements[i]);

  }

  return deltas;

}

vec2
NurbsTools::inverseMapping (const vec3 &pt, const vec2 &hint, double &error, vec3 &p, vec3 &tu, vec3 &tv, int maxSteps,
                            double accuracy, bool quiet)
{
  vec2 current (0.0, 0.0), delta (0.0, 0.0);
  Matrix2d A;
  vec2 b (0.0, 0.0);
  vec3 r (0.0, 0.0, 0.0);
  std::vector<double> elementsU = getElementVector (0);
  std::vector<double> elementsV = getElementVector (1);
  double minU = elementsU[0];
  double minV = elementsV[0];
  double maxU = elementsU[elementsU.size () - 1];
  double maxV = elementsV[elementsV.size () - 1];

  current = hint;

  for (int k = 0; k < maxSteps; k++)
  {

    m_surf->Evaluate (current (0), current (1), p, tu, tv);

    r = p - pt;

    b (0) = -r.dot (tu);
    b (1) = -r.dot (tv);

    A (0, 0) = tu.dot (tu);
    A (0, 1) = tu.dot (tv);
    A (1, 0) = A (0, 1);
    A (1, 1) = tv.dot (tv);

    delta = A.ldlt ().solve (b);

    if (delta.norm () < accuracy)
    {

      error = r.norm ();
      return current;

    }
    else
    {
      current = current + delta;

      //      bool stop = false;

      if (current (0) < minU)
      {
        current (0) = minU;
        //        stop = true;
      }
      else if (current (0) > maxU)
      {
        current (0) = maxU;
        //        stop = true;
      }

      if (current (1) < minV)
      {
        current (1) = minV;
        //        stop = true;
      }
      else if (current (1) > maxV)
      {
        current (1) = maxV;
        //        stop = true;
      }

      //      if( stop ) {
      //        error = arma::norm(r, 2);
      //        return current;
      //      }

    }

  }

  if (!quiet)
    std::cout << "[NurbsTools::inverseMapping] Warning: Method did not converge after maximum number of steps!"
        << std::endl;

  error = r.norm ();

  if (!quiet)
    std::cout << "  " << hint (0) << " " << hint (1) << " .. " << current (0) << " " << current (1) << std::endl;

  return current;

}

vec2
NurbsTools::inverseMapping (const vec3 &pt, vec2* phint, double &error, vec3 &p, vec3 &tu, vec3 &tv, int maxSteps,
                            double accuracy, bool quiet)
{

  vec2 hint;
  vec3 r;
  std::vector<double> elementsU = getElementVector (0);
  std::vector<double> elementsV = getElementVector (1);

  if (phint == NULL)
  {
    double d_shortest (0.0);
    for (unsigned i = 0; i < elementsU.size () - 1; i++)
    {
      for (unsigned j = 0; j < elementsV.size () - 1; j++)
      {
        vec3 point;
        double d;

        double xi = elementsU[i] + 0.5 * (elementsU[i + 1] - elementsU[i]);
        double eta = elementsV[j] + 0.5 * (elementsV[j + 1] - elementsV[j]);

        m_surf->Evaluate (xi, eta, point);

        r = point - pt;

        d = r.norm ();

        if ((i == 0 && j == 0) || d < d_shortest)
        {
          d_shortest = d;
          hint (0) = xi;
          hint (1) = eta;
        }
      }
    }
  }
  else
  {
    hint = *phint;
  }

  return inverseMapping (pt, hint, error, p, tu, tv, maxSteps, accuracy, quiet);
}

vec2
NurbsTools::inverseMappingBoundary (const vec3 &pt, double &error, vec3 &p, vec3 &tu, vec3 &tv, int maxSteps,
                                    double accuracy, bool quiet)
{

  vec2 result;
  double min_err = 100.0;
  std::vector<myvec> ini_points;
  double err_tmp;
  vec3 p_tmp, tu_tmp, tv_tmp;

  std::vector<double> elementsU = getElementVector (0);
  std::vector<double> elementsV = getElementVector (1);

  // NORTH - SOUTH
  for (unsigned i = 0; i < (elementsV.size () - 1); i++)
  {
    ini_points.push_back (myvec (WEST, elementsV[i] + 0.5 * (elementsV[i + 1] - elementsV[i])));
    ini_points.push_back (myvec (EAST, elementsV[i] + 0.5 * (elementsV[i + 1] - elementsV[i])));
  }

  // WEST - EAST
  for (unsigned i = 0; i < (elementsU.size () - 1); i++)
  {
    ini_points.push_back (myvec (NORTH, elementsU[i] + 0.5 * (elementsU[i + 1] - elementsU[i])));
    ini_points.push_back (myvec (SOUTH, elementsU[i] + 0.5 * (elementsU[i + 1] - elementsU[i])));
  }

  for (unsigned i = 0; i < ini_points.size (); i++)
  {

    vec2 params = inverseMappingBoundary (pt, ini_points[i].side, ini_points[i].hint, err_tmp, p_tmp, tu_tmp, tv_tmp,
                                          maxSteps, accuracy, quiet);

    if (i == 0 || err_tmp < min_err)
    {
      min_err = err_tmp;
      result = params;
      p = p_tmp;
      tu = tu_tmp;
      tv = tv_tmp;
    }
  }

  error = min_err;
  return result;

}

vec2
NurbsTools::inverseMappingBoundary (const vec3 &pt, int side, double hint, double &error, vec3 &p, vec3 &tu, vec3 &tv,
                                    int maxSteps, double accuracy, bool quiet)
{
  double current (0.0), delta (0.0);
  vec3 r (0.0, 0.0, 0.0), t (0.0, 0.0, 0.0);
  vec2 params (0.0, 0.0);

  current = hint;

  std::vector<double> elementsU = getElementVector (0);
  std::vector<double> elementsV = getElementVector (1);
  double minU = elementsU[0];
  double minV = elementsV[0];
  double maxU = elementsU[elementsU.size () - 1];
  double maxV = elementsV[elementsV.size () - 1];

  for (int k = 0; k < maxSteps; k++)
  {

    switch (side)
    {

      case WEST:

        params (0) = minU;
        params (1) = current;
        m_surf->Evaluate (minU, current, p, tu, tv);

        t = tv; // use tv

        break;
      case SOUTH:

        params (0) = current;
        params (1) = maxV;
        m_surf->Evaluate (current, maxV, p, tu, tv);

        t = tu; // use tu

        break;
      case EAST:

        params (0) = maxU;
        params (1) = current;
        m_surf->Evaluate (maxU, current, p, tu, tv);

        t = tv; // use tv

        break;
      case NORTH:

        params (0) = current;
        params (1) = minV;
        m_surf->Evaluate (current, minV, p, tu, tv);

        t = tu; // use tu

        break;
      default:
        throw std::runtime_error ("[NurbsFitting::inverseMappingBoundary] Warning: Specify a boundary!");

    }

    r = p - pt;

    delta = -0.5 * r.dot (t) / t.dot (t);

    if (fabs (delta) < accuracy)
    {

      error = r.norm ();
      return params;

    }
    else
    {

      current = current + delta;

      bool stop = false;

      switch (side)
      {

        case WEST:
        case EAST:
          if (current < minV)
          {
            params (1) = minV;
            stop = true;
          }
          else if (current > maxV)
          {
            params (1) = maxV;
            stop = true;
          }

          break;

        case NORTH:
        case SOUTH:
          if (current < minU)
          {
            params (0) = minU;
            stop = true;
          }
          else if (current > maxU)
          {
            params (0) = maxU;
            stop = true;
          }

          break;
      }

      if (stop)
      {
        error = r.norm ();
        return params;
      }

    }

  }

  error = r.norm ();
  if (!quiet)
    printf (
            "[NurbsTools::inverseMappingBoundary] Warning: Method did not converge! (residual: %f, delta: %f, params: %f %f)\n",
            error, delta, params (0), params (1));

  return params;
}

#ifdef USE_UMFPACK
bool NurbsTools::solveSparseLinearSystem(cholmod_sparse* A, cholmod_dense* b, cholmod_dense* x, bool transpose)
{

  double* vals = (double*) A->x;
  int* cols = (int*) A->p;
  int* rows = (int*) A->i;
  double* rhs = (double*) b->x;
  double* sol = (double*) x->x;

  int noOfVerts = b->nrow;
  int noOfCols = b->ncol;

  double* tempRhs = new double[noOfVerts];
  double* tempSol = new double[noOfVerts];

  int i, k, status;
  double* null = (double*) NULL;
  void *Symbolic, *Numeric;

  status = umfpack_di_symbolic(A->nrow, A->ncol, cols, rows, vals, &Symbolic, null, null);

  if( status != 0 )
  {

    std::cout << "[NurbsTools::solveSparseLinearSystem] Warning: something is wrong with input matrix!" << std::endl;
    return 1;

  }

  status = umfpack_di_numeric(cols, rows, vals, Symbolic, &Numeric, null, null);

  if( status != 0 )
  {

    std::cout << "[NurbsTools::solveSparseLinearSystem] Warning: ordering was ok but factorization failed!" << std::endl;
    return 1;

  }

  umfpack_di_free_symbolic(&Symbolic);

  for( i = 0; i < noOfCols; i++ )
  {

    for( k = 0; k < noOfVerts; k++ )
    tempRhs[k] = rhs[i * noOfVerts + k];

    // At or A?
    if( transpose )
    umfpack_di_solve(UMFPACK_At, cols, rows, vals, tempSol, tempRhs, Numeric, null, null);
    else
    umfpack_di_solve(UMFPACK_A, cols, rows, vals, tempSol, tempRhs, Numeric, null, null);

    for( k = 0; k < noOfVerts; k++ )
    sol[i * noOfVerts + k] = tempSol[k];

  }

  // clean up
  umfpack_di_free_numeric(&Numeric);
  delete tempRhs;
  delete tempSol;

  return 0;

}

bool NurbsTools::solveSparseLinearSystemLQ(cholmod_sparse* A, cholmod_dense* b, cholmod_dense* x)
{

  cholmod_common c;
  cholmod_start(&c);
  c.print = 4;

  cholmod_sparse* At = cholmod_allocate_sparse(A->ncol, A->nrow, 10 * A->nrow, 0, 1, 0, CHOLMOD_REAL, &c);
  //  cholmod_dense* Atb = cholmod_allocate_dense(b->nrow, b->ncol, b->nrow, CHOLMOD_REAL, &c);
  cholmod_dense* Atb = cholmod_allocate_dense(A->ncol, b->ncol, A->ncol, CHOLMOD_REAL, &c);

  double one[2] =
  { 1, 0};
  double zero[2] =
  { 0, 0};
  cholmod_sdmult(A, 1, one, zero, b, Atb, &c);

  cholmod_transpose_unsym(A, 1, NULL, NULL, A->ncol, At, &c);

  cholmod_sparse* AtA = cholmod_ssmult(At, A, 0, 1, 1, &c);

  //  cholmod_print_sparse(AtA,"A: ", &c);

  solveSparseLinearSystem(AtA, Atb, x, false);

  cholmod_free_sparse(&At, &c);
  cholmod_free_sparse(&AtA, &c);
  cholmod_free_dense(&Atb, &c);

  cholmod_finish(&c);

  return 0;

}
#endif
