/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Thomas Mörwald (University of Technology Vienna)
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
 *   * Neither the name of Thomas Mörwald nor the names of its
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

// [1] Isogeometric Analysis, Toward Integration of CAD and FEA; J.Austin Cottrell, Thomas J.R. Hughes, Yuri Bazilevs

#include <pcl/surface/nurbs/nurbs_basis.h>

#include <stdio.h>

using namespace pcl;
using namespace nurbs;

NurbsBasis::NurbsBasis ()
{

}

NurbsBasis::NurbsBasis (unsigned degree, unsigned ncps)
{
  if (degree >= ncps)
    throw std::runtime_error ("[NurbsBasis::NurbsBasis] Number of control points must be greater than degree");

  this->degree = degree;

  // make clamped knot vector
  unsigned nknots = ncps + (degree + 1);
  unsigned intknots = nknots - 2 * (degree + 1);
  double d = 1.0 / double (intknots + 1);

  for (unsigned i = 0; i < (degree + 1); i++)
    this->knots.push_back (0.0);

  for (unsigned i = 0; i < intknots; i++)
    this->knots.push_back (d * (i + 1));

  for (unsigned i = 0; i < (degree + 1); i++)
    this->knots.push_back (1.0);

}

int
NurbsBasis::GetSpan (double xi) const
{
  int span = -1;
  unsigned nknots = knots.size ();

  // find knot span
  for (unsigned s = 0; s < nknots - 1; s++)
  {
    if (xi >= knots[s] && xi < knots[s + 1])
    {
      span = s;
      break;
    }
    else if (xi == knots[nknots - 1])
    {
      span = nknots - (degree + 1) - 1;
      break;
    }
  }

  return span;
}

void
NurbsBasis::GetElementVector (std::vector<double> &result) const
{
  unsigned nknots = knots.size ();

  // assuming interpolating ends
  for (unsigned i = degree; i < nknots - degree; i++)
    result.push_back (knots[i]);
}

void
NurbsBasis::InsertKnot (double xi)
{
  unsigned nknots = knots.size ();
  if (xi < knots[0] || xi > knots[nknots - 1])
    throw std::runtime_error ("[NurbsBasis::InsertKnot] parameter 'xi' out of range.");

  // find knot span
  std::vector<double> knots_new;
  for (unsigned s = 0; s < nknots; s++)
  {
    knots_new.push_back (knots[s]);
    if (s < nknots - 1 && xi >= knots[s] && xi < knots[s + 1])
    {
      knots_new.push_back (xi);
    }
  }
  knots = knots_new;
}

void
NurbsBasis::cox (double xi, int knotSpan, unsigned degree, const std::vector<double> &supKnot, std::vector<double> &N)
{
  N.assign ((degree + 1) * (degree + 1), 0.0);
  for (unsigned p = 0; p < (degree + 1); p++)
  { // loop from lower degree to higher -> unwrapped recursion

    for (unsigned s = 0; s < (degree + 1); s++)
    { // evaluate the basis N for each knotspan s

      if (p == 0)
      {

        // Equation (2.1) in [1]
        if (xi >= supKnot[s] && xi < supKnot[s + 1])
          N[s] = 1.0;
        else if (s == degree && xi == supKnot[s + 1])
          N[s] = 1.0;
        else
          N[s] = 0.0;

      }
      else
      {
        // Equation (2.5)
        double A = 0.0;
        double B = 0.0;
        if ((xi - supKnot[s]) != 0.0 && (supKnot[s + p] - supKnot[s]) != 0.0)
          A = (xi - supKnot[s]) / (supKnot[s + p] - supKnot[s]);

        if ((supKnot[s + p + 1] - xi) != 0.0 && (supKnot[s + p + 1] - supKnot[s + 1]) != 0.0 && s < degree) // (s<degree) because N(s+i,p-1) does not support
          B = (supKnot[s + p + 1] - xi) / (supKnot[s + p + 1] - supKnot[s + 1]);

        N[s + p * (degree + 1)] = A * N[s + (p - 1) * (degree + 1)] + B * N[(s + 1) + (p - 1) * (degree + 1)];

      }
    }
  }
}

void
NurbsBasis::coxder (unsigned degree, const std::vector<double> &supKnot, const std::vector<double> &N,
                    std::vector<double> &Nd)
{
  Nd.assign ((degree + 1) * (degree + 1), 0.0);
  unsigned p = degree;

  for (unsigned s = 0; s < (degree + 1); s++)
  { // evaluate the basis N for each knotspan s
    // Equation (2.7)
    double Ad = 0.0;
    double Bd = 0.0;
    if ((supKnot[s + p] - supKnot[s]) != 0.0)
      Ad = p / (supKnot[s + p] - supKnot[s]);

    if ((supKnot[s + p + 1] - supKnot[s + 1]) != 0.0 && s < degree) // (s<degree) because N(s+i,p-1) does not support
      Bd = p / (supKnot[s + p + 1] - supKnot[s + 1]);

    Nd[s + p * (degree + 1)] = Ad * N[s + (p - 1) * (degree + 1)] - Bd * N[(s + 1) + (p - 1) * (degree + 1)];
  }
}

void
NurbsBasis::Cox (double xi, std::vector<double> &N) const
{
  int knotSpan = this->GetSpan (xi);
  if (knotSpan < 0)
    throw std::runtime_error ("[NurbsBasis::Cox] Paramater value 'xi' out of range");

  this->Cox (xi, knotSpan, N);

}

void
NurbsBasis::Cox (double xi, std::vector<double> &N, std::vector<double> &Nd) const
{
  int knotSpan = this->GetSpan (xi);
  if (knotSpan < 0)
    throw std::runtime_error ("[NurbsBasis::Cox] Paramater value 'xi' out of range");

  this->Cox (xi, knotSpan, N, Nd);
}

void
NurbsBasis::Cox (double xi, int knotSpan, std::vector<double> &N) const
{
  // get supporting knots
  int nsupKnot = 2 * (degree + 1);
  std::vector<double> supKnot;
  for (int s = 0; s < nsupKnot; s++)
    supKnot.push_back (knots[knotSpan - degree + s]);

  // evaluate cox-de-boor recursion
  cox (xi, knotSpan, degree, supKnot, N);
}

void
NurbsBasis::Cox (double xi, int knotSpan, std::vector<double> &N, std::vector<double> &Nd) const
{
  // get supporting knots
  int nsupKnot = 2 * (degree + 1);
  std::vector<double> supKnot;
  for (int s = 0; s < nsupKnot; s++)
    supKnot.push_back (knots[knotSpan - degree + s]);

  // evaluate cox-de-boor recursion
  cox (xi, knotSpan, degree, supKnot, N);

  // evaluate cox-de-boor recursion for derivatives
  coxder (degree, supKnot, N, Nd);
}
