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

#include <suitesparse/cholmod.h>
#include <suitesparse/umfpack.h>
#include <iostream>
#include <stdio.h>
#include <stdexcept>

#include <pcl/surface/on_nurbs/nurbs_solve.h>

using namespace std;
using namespace pcl;
using namespace on_nurbs;

void
NurbsSolve::assign (unsigned rows, unsigned cols, unsigned dims)
{
  m_Ksparse.clear ();
  m_xeig = Eigen::MatrixXd::Zero (cols, dims);
  m_feig = Eigen::MatrixXd::Zero (rows, dims);
}

void
NurbsSolve::K (unsigned i, unsigned j, double v)
{
  m_Ksparse.set (i, j, v);
}
void
NurbsSolve::x (unsigned i, unsigned j, double v)
{
  m_xeig (i, j) = v;
}
void
NurbsSolve::f (unsigned i, unsigned j, double v)
{
  m_feig (i, j) = v;
}

double
NurbsSolve::K (unsigned i, unsigned j)
{
  return m_Ksparse.get (i, j);
}
double
NurbsSolve::x (unsigned i, unsigned j)
{
  return m_xeig (i, j);
}
double
NurbsSolve::f (unsigned i, unsigned j)
{
  return m_feig (i, j);
}

void
NurbsSolve::resize (unsigned rows)
{
  m_feig.conservativeResize (rows, m_feig.cols ());
}

void
NurbsSolve::printK ()
{
  m_Ksparse.printLong ();
}

void
NurbsSolve::printX ()
{
  for (unsigned r = 0; r < m_xeig.rows (); r++)
  {
    for (unsigned c = 0; c < m_xeig.cols (); c++)
    {
      printf (" %f", m_xeig (r, c));
    }
    printf ("\n");
  }
}

void
NurbsSolve::printF ()
{
  for (unsigned r = 0; r < m_feig.rows (); r++)
  {
    for (unsigned c = 0; c < m_feig.cols (); c++)
    {
      printf (" %f", m_feig (r, c));
    }
    printf ("\n");
  }
}

namespace pcl
{
  namespace on_nurbs
  {
    bool
    solveSparseLinearSystem (cholmod_sparse* A, cholmod_dense* b, cholmod_dense* x, bool transpose)
    {
      double* vals = (double*)A->x;
      int* cols = (int*)A->p;
      int* rows = (int*)A->i;
      double* rhs = (double*)b->x;
      double* sol = (double*)x->x;

      int noOfVerts = b->nrow;
      int noOfCols = b->ncol;

      double* tempRhs = new double[noOfVerts];
      double* tempSol = new double[noOfVerts];

      int i, k, status;
      double* null = (double*)NULL;
      void *Symbolic, *Numeric;

      status = umfpack_di_symbolic (A->nrow, A->ncol, cols, rows, vals, &Symbolic, null, null);

      if (status != 0)
      {
        std::cout << "[NurbsSolve[UMFPACK]::solveSparseLinearSystem] Warning: something is wrong with input matrix!"
            << std::endl;
        delete [] tempRhs;
        delete [] tempSol;
        return false;

      }

      status = umfpack_di_numeric (cols, rows, vals, Symbolic, &Numeric, null, null);

      if (status != 0)
      {
        std::cout
            << "[NurbsSolve[UMFPACK]::solveSparseLinearSystem] Warning: ordering was ok but factorization failed!"
            << std::endl;
        delete [] tempRhs;
        delete [] tempSol;
        return false;
      }

      umfpack_di_free_symbolic (&Symbolic);

      for (i = 0; i < noOfCols; i++)
      {
        for (k = 0; k < noOfVerts; k++)
          tempRhs[k] = rhs[i * noOfVerts + k];

        // At or A?
        if (transpose)
          umfpack_di_solve (UMFPACK_At, cols, rows, vals, tempSol, tempRhs, Numeric, null, null);
        else
          umfpack_di_solve (UMFPACK_A, cols, rows, vals, tempSol, tempRhs, Numeric, null, null);

        for (k = 0; k < noOfVerts; k++)
          sol[i * noOfVerts + k] = tempSol[k];
      }

      // clean up
      umfpack_di_free_numeric (&Numeric);
      delete [] tempRhs;
      delete [] tempSol;
      return true;
    }

    bool
    solveSparseLinearSystemLQ (cholmod_sparse* A, cholmod_dense* b, cholmod_dense* x)
    {
      cholmod_common c;
      cholmod_start (&c);
      c.print = 4;

      cholmod_sparse* At = cholmod_allocate_sparse (A->ncol, A->nrow, A->ncol * A->nrow, 0, 1, 0, CHOLMOD_REAL, &c);
      //  cholmod_dense* Atb = cholmod_allocate_dense(b->nrow, b->ncol, b->nrow, CHOLMOD_REAL, &c);
      cholmod_dense* Atb = cholmod_allocate_dense (A->ncol, b->ncol, A->ncol, CHOLMOD_REAL, &c);

      double one[2] = {1, 0};
      double zero[2] = {0, 0};
      cholmod_sdmult (A, 1, one, zero, b, Atb, &c);

      unsigned trials (20);
      unsigned i (0);
      while (!cholmod_transpose_unsym (A, 1, NULL, NULL, A->ncol, At, &c) && i < trials)
      {
        printf (
                "[NurbsSolveUmfpack::solveSparseLinearSystemLQ] Warning allocated memory to low, trying to increase %dx\n",
                (i + 2));

        cholmod_free_sparse (&At, &c);
        cholmod_free_dense (&Atb, &c);

        At = cholmod_allocate_sparse (A->ncol, A->nrow, (i + 2) * A->ncol * A->nrow, 0, 1, 0, CHOLMOD_REAL, &c);
        Atb = cholmod_allocate_dense (A->ncol, b->ncol, A->ncol, CHOLMOD_REAL, &c);

        double one[2] = {1, 0};
        double zero[2] = {0, 0};
        cholmod_sdmult (A, 1, one, zero, b, Atb, &c);
        i++;
      }
      
      if (i == trials)
      {
        printf ("[NurbsSolveUmfpack::solveSparseLinearSystemLQ] Not enough memory for system to solve. (%d trials)\n",
                trials);
        cholmod_free_sparse (&At, &c);
        cholmod_free_dense (&Atb, &c);
        return false;
      }

      cholmod_sparse* AtA = cholmod_ssmult (At, A, 0, 1, 1, &c);

      //  cholmod_print_sparse(AtA,"A: ", &c);

      bool success = solveSparseLinearSystem (AtA, Atb, x, false);

      cholmod_free_sparse (&At, &c);
      cholmod_free_sparse (&AtA, &c);
      cholmod_free_dense (&Atb, &c);

      cholmod_finish (&c);

      return success;
    }
  }
}

bool
NurbsSolve::solve ()
{
  clock_t time_start, time_end;
  time_start = clock ();

  cholmod_common c;
  cholmod_start (&c);

  int n_rows, n_cols, n_dims, n_nz;
  m_Ksparse.size (n_rows, n_cols);
  n_nz = m_Ksparse.nonzeros ();
  n_dims = m_feig.cols ();

  //  m_Ksparse.printLong();

  cholmod_sparse* K = cholmod_allocate_sparse (n_rows, n_cols, n_rows * n_cols, 0, 1, 0, CHOLMOD_REAL, &c);
  cholmod_dense* f = cholmod_allocate_dense (n_rows, n_dims, n_rows, CHOLMOD_REAL, &c);
  cholmod_dense* d = cholmod_allocate_dense (n_cols, n_dims, n_cols, CHOLMOD_REAL, &c);

  std::vector<int> rowinds;
  std::vector<int> colinds;
  std::vector<double> values;
  m_Ksparse.get (rowinds, colinds, values);

  double* vals = (double*)K->x;
  int* cols = (int*)K->p;
  int* rows = (int*)K->i;

  umfpack_di_triplet_to_col (n_rows, n_cols, n_nz, &rowinds[0], &colinds[0], &values[0], cols, rows, vals, NULL);

  double* temp = (double*)f->x;

  for (int j = 0; j < n_dims; j++)
  {
    for (int i = 0; i < n_rows; i++)
    {

      temp[j * n_rows + i] = m_feig (i, j);

    }
  }

  bool success = solveSparseLinearSystemLQ (K, f, d);

  temp = (double*)d->x;

  for (int j = 0; j < n_dims; j++)
  {
    for (int i = 0; i < n_cols; i++)
    {

      m_xeig (i, j) = temp[j * n_cols + i];

    }
  }

  cholmod_free_sparse (&K, &c);
  cholmod_free_dense (&f, &c);
  cholmod_free_dense (&d, &c);

  cholmod_finish (&c);

  time_end = clock ();

  if (success)
  {
    if (!m_quiet)
    {
      double solve_time = (double)(time_end - time_start) / (double)(CLOCKS_PER_SEC);
      printf ("[NurbsSolve[UMFPACK]::solve()] solution found! (%f sec)\n", solve_time);
    }
  }
  else
  {
    printf ("[NurbsSolve[UMFPACK]::solve()] Error: solution NOT found\n");
  }

  return success;
}

Eigen::MatrixXd
NurbsSolve::diff ()
{

  int n_rows, n_cols, n_dims;
  m_Ksparse.size (n_rows, n_cols);
  n_dims = m_feig.cols ();

  if (n_rows != m_feig.rows ())
  {
    printf ("[NurbsSolve::diff] K.rows: %d  f.rows: %d\n", n_rows, (int)m_feig.rows ());
    throw std::runtime_error ("[NurbsSolve::diff] Rows of equation do not match\n");
  }

  Eigen::MatrixXd f = Eigen::MatrixXd::Zero (n_rows, n_dims);

  for (int r = 0; r < n_rows; r++)
  {
    for (int c = 0; c < n_cols; c++)
    {
      f.row (r) = f.row (r) + m_xeig.row (c) * m_Ksparse.get (r, c);
    }
  }

  return (f - m_feig);
}

