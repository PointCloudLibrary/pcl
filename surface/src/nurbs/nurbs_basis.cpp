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

pcl::nurbs::NurbsBasis::NurbsBasis (std::size_t degree, std::size_t ncps)
{
  if (degree >= ncps)
    throw std::runtime_error ("[pcl::nurbs::NurbsBasis::NurbsBasis] Number of control points must be greater than degree");

  this->degree_ = degree;

  // make clamped knot vector
  std::size_t nknots = ncps + (degree + 1);
  std::size_t intknots = nknots - 2 * (degree + 1);
  double d = 1.0 / double (intknots + 1);

  for (std::size_t i = 0; i < (degree + 1); i++)
    knots_.push_back (0.0);

  for (std::size_t i = 0; i < intknots; i++)
    knots_.push_back (d * static_cast<double> (i + 1));

  for (std::size_t i = 0; i < (degree + 1); i++)
    knots_.push_back (1.0);

}

int
pcl::nurbs::NurbsBasis::getSpan (const double &xi) const
{
  int span = -1;
  std::size_t nknots = knots_.size ();

  // find knot span
  for (std::size_t s = 0; s < nknots - 1; ++s)
  {
    if (xi >= knots_[s] && xi < knots_[s + 1])
    {
      span = static_cast<int> (s);
      break;
    }
    else if (xi == knots_[nknots - 1])
    {
      span = static_cast<int> (nknots - (degree_ + 1) - 1);
      break;
    }
  }

  return span;
}

void
pcl::nurbs::NurbsBasis::getElementVector (std::vector<double> &result) const
{
  std::size_t nknots = knots_.size ();

  // assuming interpolating ends
  for (std::size_t i = degree_; i < nknots - degree_; ++i)
    result.push_back (knots_[i]);
}

void
pcl::nurbs::NurbsBasis::insertKnot (const double &xi)
{
  std::size_t nknots = knots_.size ();
  if (xi < knots_[0] || xi > knots_[nknots - 1])
    throw std::runtime_error ("[pcl::nurbs::NurbsBasis::InsertKnot] parameter 'xi' out of range.");

  // find knot span
  std::vector<double> knots_new;
  for (std::size_t s = 0; s < nknots; s++)
  {
    knots_new.push_back (knots_[s]);
    if (s < nknots - 1 && xi >= knots_[s] && xi < knots_[s + 1])
    {
      knots_new.push_back (xi);
    }
  }
  knots_ = knots_new;
}

void
pcl::nurbs::NurbsBasis::cox (const double &xi,
                             int knotSpan,
                             std::size_t degree,
                             const std::vector<double> &supKnot,
                             std::vector<double> &N)
{
  N.assign ((degree + 1) * (degree + 1), 0.0);
  for (std::size_t p = 0; p < (degree + 1); p++)
  { // loop from lower degree to higher -> unwrapped recursion

    for (std::size_t s = 0; s < (degree + 1); s++)
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

        if (((supKnot[s + p + 1] - xi) != 0.0) &&
            ((supKnot[s + p + 1] - supKnot[s + 1]) != 0.0) &&
            s < degree) // (s<degree) because N(s+i,p-1) does not support
          B = (supKnot[s + p + 1] - xi) / (supKnot[s + p + 1] - supKnot[s + 1]);

        N[s + p * (degree + 1)] = A * N[s + (p - 1) * (degree + 1)] + B * N[(s + 1) + (p - 1) * (degree + 1)];

      }
    }
  }
}

void
pcl::nurbs::NurbsBasis::coxder (std::size_t degree,
                                const std::vector<double> &supKnot,
                                const std::vector<double> &N,
                                std::vector<double> &Nd)
{
  Nd.assign ((degree + 1) * (degree + 1), 0.0);
  std::size_t p = degree;

  for (std::size_t s = 0; s < (degree + 1); s++)
  {
    // evaluate the basis N for each knotspan s
    // Equation (2.7)
    double Ad = 0.0;
    double Bd = 0.0;
    if ((supKnot[s + p] - supKnot[s]) != 0.0)
      Ad = static_cast<double> (p) / (supKnot[s + p] - supKnot[s]);

    if ((supKnot[s + p + 1] - supKnot[s + 1]) != 0.0 && s < degree) // (s<degree) because N(s+i,p-1) does not support
      Bd = static_cast<double> (p) / (supKnot[s + p + 1] - supKnot[s + 1]);

    Nd[s + p * (degree + 1)] = Ad * N[s + (p - 1) * (degree + 1)] - Bd * N[(s + 1) + (p - 1) * (degree + 1)];
  }
}

void
pcl::nurbs::NurbsBasis::cox (const double &xi, std::vector<double> &N) const
{
  int knotSpan = this->getSpan (xi);
  if (knotSpan < 0)
    throw std::runtime_error ("[pcl::nurbs::NurbsBasis::Cox] Paramater value 'xi' out of range");

  cox (xi, knotSpan, N);
}

void
pcl::nurbs::NurbsBasis::cox (const double &xi, std::vector<double> &N, std::vector<double> &Nd) const
{
  int knotSpan = this->getSpan (xi);
  if (knotSpan < 0)
    throw std::runtime_error ("[pcl::nurbs::NurbsBasis::Cox] Paramater value 'xi' out of range");

  cox (xi, knotSpan, N, Nd);
}

void
pcl::nurbs::NurbsBasis::cox (const double &xi, int knotSpan, std::vector<double> &N) const
{
  // get supporting knots
  std::size_t nsupKnot = 2 * (degree_ + 1);
  std::vector<double> supKnot;
  for (std::size_t s = 0; s < nsupKnot; ++s)
    supKnot.push_back (knots_[knotSpan - degree_ + s]);

  // evaluate cox-de-boor recursion
  cox (xi, knotSpan, degree_, supKnot, N);
}

void
pcl::nurbs::NurbsBasis::cox (const double &xi,
                             int knotSpan,
                             std::vector<double> &N,
                             std::vector<double> &Nd) const
{
  // get supporting knots
  std::size_t nsupKnot = 2 * (degree_ + 1);
  std::vector<double> supKnot;
  for (std::size_t s = 0; s < nsupKnot; ++s)
    supKnot.push_back (knots_[knotSpan - degree_ + s]);

  // evaluate cox-de-boor recursion
  cox (xi, knotSpan, degree_, supKnot, N);

  // evaluate cox-de-boor recursion for derivatives
  coxder (degree_, supKnot, N, Nd);
}
