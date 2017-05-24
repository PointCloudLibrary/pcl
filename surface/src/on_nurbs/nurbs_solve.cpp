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

#include <iostream>
#include <stdexcept>
#include <ctime>

#include <pcl/surface/on_nurbs/nurbs_solve.h>
#ifdef USEUMFPACK
#include <pcl/surface/3rdparty/suitesparse/cholmod.h>
#include <pcl/surface/3rdparty/suitesparse/umfpack.h>
#endif

using namespace std;
using namespace pcl;
using namespace on_nurbs;



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
NurbsSolve::x (unsigned i, unsigned j, double v)
{
  m_xeig (i, j) = v;
}
void
NurbsSolve::f (unsigned i, unsigned j, double v)
{
  m_feig (i, j) = v;
  cerr <<"\nShouldn't assign f directly";
}


void
NurbsSolve::resize (unsigned rows)
{
	// Nothing to do...
}

#ifndef USEUMFPACK

void
NurbsSolve::assign (unsigned rows, unsigned cols, unsigned dims)
{
	m_xeig = Eigen::MatrixXd::Zero (cols, dims);
	m_feig = Eigen::MatrixXd::Zero (cols, dims);

	m_Ksparse.clear();
	m_KeigenSparse = Eigen::SparseMatrix<double>(cols,cols);
}

void
NurbsSolve::K (unsigned i, unsigned j, double v)
{
  m_KeigenSparse.insert(i,j) = v;
  cerr <<"\nShouldn't assign K directly";
}

double
NurbsSolve::K (unsigned i, unsigned j)
{
  return m_KeigenSparse.coeff(i,j);
}



 void NurbsSolve::AddRow(const double *row, const double *f, const unsigned rowNr)
 {
	 unsigned N = m_feig.rows();
	 double newVal = 0.0;
	 for(unsigned j=0; j<N; ++j)
	 {
		  if(abs(row[j]) < 1e-10) continue;
		  for(unsigned jj=0; jj<m_feig.cols(); ++jj)
		  {
			  m_feig(j,jj) += row[j]*f[jj];
		  }
		  for(unsigned jj=j; jj<N; ++jj)
		  {
			newVal = m_KeigenSparse.coeff(j,jj) + row[j]*row[jj];
			if(abs(newVal) > 1e-10)  
			{
				m_KeigenSparse.coeffRef(j,jj) = newVal;
				m_KeigenSparse.coeffRef(jj,j) = newVal;
			}
		  }
	 }
	 
 }
 

 
 
 void NurbsSolve::AddRow(const double *row, const double *f, const unsigned rowNr, vector<int> sparsePattern)
 {
	 unsigned N = m_feig.rows();
	 double newVal = 0.0;
	 int j,jj;
	 for(unsigned iSparse=0; iSparse<sparsePattern.size(); ++iSparse)
	 {
		  j = sparsePattern[iSparse];
		// if(abs(row[j]) < 1e-10) continue;
		  for(jj=0; jj<m_feig.cols(); ++jj)
		  {
			  m_feig(j,jj) += row[j]*f[jj];
		  }
		  for(unsigned jSparse=0; jSparse < sparsePattern.size(); ++jSparse)
		  {
			  jj = sparsePattern[jSparse];
			  if(jj<j) continue;
			  newVal = m_KeigenSparse.coeff(j,jj) + row[j]*row[jj];
			if(abs(newVal) > 1e-10)  
			{
				m_KeigenSparse.coeffRef(j,jj) = newVal;
				m_KeigenSparse.coeffRef(jj,j) = newVal;
			 }
		  }
	 }
	 
 }


void
NurbsSolve::printK ()
{
	for (unsigned r = 0; r < m_KeigenSparse.rows (); r++)
  {
    for (unsigned c = 0; c < m_KeigenSparse.cols (); c++)
    {
		printf (" %f", m_KeigenSparse.coeff (r, c));
    }
    printf ("\n");
  }
}


bool
NurbsSolve::solve ()
{
	Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>,Eigen::Lower> chol(m_KeigenSparse);
	if(chol.info() != Eigen::Success )
	{
		return false;
	}
	m_xeig = chol.solve(m_feig);
	return true;
}

Eigen::MatrixXd
NurbsSolve::diff ()
{	
	Eigen::MatrixXd f (m_KeigenSparse * m_xeig);
	return (f - m_feig);
}

#else

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
  } // namespace ON
} //namespace pcl


void
NurbsSolve::assign (unsigned rows, unsigned cols, unsigned dims)
{
	m_xeig = Eigen::MatrixXd::Zero (cols, dims);
	m_feig = Eigen::MatrixXd::Zero (cols, dims);

	m_Ksparse.clear();
}

void
NurbsSolve::K (unsigned i, unsigned j, double v)
{
  m_Ksparse.set (i, j, v);
  cerr <<"\nShouldn't assign K directly";
}

double
NurbsSolve::K (unsigned i, unsigned j)
{
  return m_Ksparse.get(i,j);
}

 void NurbsSolve::AddRow(const double *row, const double *f, const unsigned rowNr)
 {
	 unsigned N = m_feig.rows();
	 double newVal = 0.0;
	 for(unsigned j=0; j<N; ++j)
	 {
		 if(abs(row[j]) < 1e-10) continue;
		  for(unsigned jj=0; jj<m_feig.cols(); ++jj)
		  {
			  m_feig(j,jj) += row[j]*f[jj];
		  }
		  for(unsigned jj=j; jj<N; ++jj)
		  {
			  newVal = m_Ksparse.get(j,jj) + row[j]*row[jj];
			 if(abs(newVal) > 1e-10)  
			{
					m_Ksparse.set(j,jj,newVal);
					m_Ksparse.set(jj,j,newVal);
			 }
		  }
	 }
	 
 }
 

 
 
 void NurbsSolve::AddRow(const double *row, const double *f, const unsigned rowNr, vector<int> sparsePattern)
 {
	 unsigned N = m_feig.rows();
	 double newVal = 0.0;
	 int j,jj;
	 for(unsigned iSparse=0; iSparse<sparsePattern.size(); ++iSparse)
	 {
		  j = sparsePattern[iSparse];
		// if(abs(row[j]) < 1e-10) continue;
		  for(jj=0; jj<m_feig.cols(); ++jj)
		  {
			  m_feig(j,jj) += row[j]*f[jj];
		  }
		  for(unsigned jSparse=0; jSparse < sparsePattern.size(); ++jSparse)
		  {
			  jj = sparsePattern[jSparse];
			  if(jj<j) continue;
			  newVal = m_Ksparse.get(j,jj) + row[j]*row[jj];
			if(abs(newVal) > 1e-10)  
			{
				m_Ksparse.set(j,jj,newVal);
				m_Ksparse.set(jj,j,newVal);		
			 }
		  }
	 }
	 
 }

 

	bool
NurbsSolve::solve ()
{
	cholmod_common c;
	cholmod_start (&c);

	int n_rows, n_cols, n_dims, n_nz;
	n_cols =  m_feig.rows();  //m_Keig.cols();
	n_rows = n_cols;
	//m_Ksparse.size (n_rows, n_cols);
	n_nz = m_Ksparse.nonzeros ();
	n_dims = m_feig.cols ();
	
	cholmod_sparse* K = cholmod_allocate_sparse (n_rows, n_cols, n_nz, 0, 1, 0, CHOLMOD_REAL, &c);
	if(!K) return false;
	cholmod_dense* f = cholmod_allocate_dense (n_rows, n_dims, n_rows, CHOLMOD_REAL, &c);
	if(!f) return false;
	cholmod_dense* d = cholmod_allocate_dense (n_cols, n_dims, n_cols, CHOLMOD_REAL, &c);
	if(!d) return false;

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

	bool success = solveSparseLinearSystem (K, f, d,false);

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

	if (success)
	{
		if (!m_quiet)
		{
			printf ("[NurbsSolve[UMFPACK]::solve()] solution found!\n");
		}
	}
	 else
	{
		printf ("[NurbsSolve[UMFPACK]::solve()] Error: solution NOT found\n");
	}
	
	return success;


  //m_xeig = m_Keig.colPivHouseholderQr().solve(m_feig);
  return true;
}

Eigen::MatrixXd
NurbsSolve::diff ()
{	
  Eigen::MatrixXd f (m_Keig * m_xeig);
  return (f - m_feig);
}





#endif
