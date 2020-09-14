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

#include <Eigen/SVD> // for jacobiSvd

#include <pcl/surface/on_nurbs/nurbs_solve.h>

using namespace pcl;
using namespace on_nurbs;

void
NurbsSolve::assign (unsigned rows, unsigned cols, unsigned dims)
{
  m_Keig = Eigen::MatrixXd::Zero (rows, cols);
  m_xeig = Eigen::MatrixXd::Zero (cols, dims);
  m_feig = Eigen::MatrixXd::Zero (rows, dims);
}

void
NurbsSolve::K (unsigned i, unsigned j, double v)
{
  m_Keig (i, j) = v;
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
  return m_Keig (i, j);
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
  m_Keig.conservativeResize (rows, m_Keig.cols ());
}

void
NurbsSolve::printK ()
{
  for (Eigen::Index r = 0; r < m_Keig.rows (); r++)
  {
    for (Eigen::Index c = 0; c < m_Keig.cols (); c++)
    {
      printf (" %f", m_Keig (r, c));
    }
    printf ("\n");
  }
}

void
NurbsSolve::printX ()
{
  for (Eigen::Index r = 0; r < m_xeig.rows (); r++)
  {
    for (Eigen::Index c = 0; c < m_xeig.cols (); c++)
    {
      printf (" %f", m_xeig (r, c));
    }
    printf ("\n");
  }
}

void
NurbsSolve::printF ()
{
  for (Eigen::Index r = 0; r < m_feig.rows (); r++)
  {
    for (Eigen::Index c = 0; c < m_feig.cols (); c++)
    {
      printf (" %f", m_feig (r, c));
    }
    printf ("\n");
  }
}

bool
NurbsSolve::solve ()
{
  //  m_xeig = m_Keig.colPivHouseholderQr().solve(m_feig);
  //  Eigen::MatrixXd x = A.householderQr().solve(b);
  m_xeig = m_Keig.jacobiSvd (Eigen::ComputeThinU | Eigen::ComputeThinV).solve (m_feig);

  return true;
}

Eigen::MatrixXd
NurbsSolve::diff ()
{
  Eigen::MatrixXd f (m_Keig * m_xeig);
  return (f - m_feig);
}

