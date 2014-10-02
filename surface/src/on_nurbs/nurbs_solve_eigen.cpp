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
//  m_feig (i, j) = v;
  throw std::runtime_error("[NurbsSolve::f] Shouldn't assign f directly.");
}


void
NurbsSolve::resize (unsigned rows)
{
  // Nothing to do...
}

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
//  m_KeigenSparse.insert(i,j) = v;
  throw std::runtime_error("[NurbsSolve::K] Shouldn't assign K directly.");
}

double
NurbsSolve::K (unsigned i, unsigned j)
{
  return m_KeigenSparse.coeff(i,j);
}

//void
//NurbsSolve::AddRow(const std::vector<double>& row, const std::vector<double>& f)
//{
//  unsigned N = m_feig.rows();
//  double newVal = 0.0;
//  for(unsigned j=0; j<N; ++j)
//  {
//    if(abs(row[j]) < 1e-10) continue;
//    for(unsigned jj=0; jj<m_feig.cols(); ++jj)
//    {
//      m_feig(j,jj) += row[j]*f[jj];
//    }
//    for(unsigned jj=j; jj<N; ++jj)
//    {
//      newVal = m_KeigenSparse.coeff(j,jj) + row[j]*row[jj];
//      if(abs(newVal) > 1e-10)
//      {
//        m_KeigenSparse.coeffRef(j,jj) = newVal;
//        m_KeigenSparse.coeffRef(jj,j) = newVal;
//      }
//    }
//  }

//}

void
NurbsSolve::AddRow(const std::vector<double>& row,
                   const std::vector<double>& f,
                   const std::vector<int>& sparsePattern)
{
  if(row.size()!=sparsePattern.size())
    throw std::runtime_error("[NurbsSolve::AddRow] size does not match. (row,sparsePattern)");

  if(f.size()!=m_feig.cols())
    throw std::runtime_error("[NurbsSolve::AddRow] size does not match. (f,m_feig)");

  double newVal = 0.0;
  int j,jj;
  for(unsigned iSparse=0; iSparse<sparsePattern.size(); ++iSparse)
  {
    j = sparsePattern[iSparse];
    // if(abs(row[j]) < 1e-10) continue;
    for(jj=0; jj<m_feig.cols(); ++jj)
    {
      m_feig(j,jj) += row[iSparse]*f[jj];
    }
    for(unsigned jSparse=0; jSparse < sparsePattern.size(); ++jSparse)
    {
      jj = sparsePattern[jSparse];
      if(jj<j) continue;
      newVal = m_KeigenSparse.coeff(j,jj) + row[iSparse]*row[jSparse];
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
