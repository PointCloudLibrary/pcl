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

#include <pcl/surface/nurbs/nurbs_curve.h>
#include <stdio.h>

using namespace pcl;
using namespace nurbs;

NurbsCurve::NurbsCurve ()
{

}

NurbsCurve::NurbsCurve (unsigned degree, const vector_vec4 &cps) :
  basis (degree, cps.size ())
{
  this->cps = cps;
}

void
NurbsCurve::Evaluate (double xi, vec3 &point) const
{
  vec4 pt (0.0f, 0.0f, 0.0f, 0.0f);

  int span = basis.GetSpan (xi);
  if (span < 0)
    throw std::runtime_error ("[NurbsCurve::Evaluate] Paramater value 'xi' out of range");

  std::vector<double> N, Nd;
  basis.Cox (xi, span, N, Nd);

  unsigned degree = basis.degree;
  for (unsigned s = 0; s < (degree + 1); s++)
  {
    pt = pt + cps[span - degree + s] * N[s + degree * (degree + 1)];
  }

  point (0) = pt (0);
  point (1) = pt (1);
  point (2) = pt (2);
}

void
NurbsCurve::Evaluate (double xi, vec3 &point, vec3 &grad) const
{
  vec4 pt (0.0f, 0.0f, 0.0f, 0.0f);
  vec4 gd (0.0f, 0.0f, 0.0f, 0.0f);

  int span = basis.GetSpan (xi);
  if (span < 0)
    throw std::runtime_error ("[NurbsCurve::Evaluate] Paramater value 'xi' out of range");

  std::vector<double> N, Nd;
  basis.Cox (xi, span, N, Nd);

  unsigned degree = basis.degree;
  for (unsigned s = 0; s < (degree + 1); s++)
  {
    pt = pt + cps[span - degree + s] * N[s + degree * (degree + 1)];
    gd = gd + cps[span - degree + s] * Nd[s + degree * (degree + 1)];
  }

  point (0) = pt (0);
  point (1) = pt (1);
  point (2) = pt (2);

  grad (0) = gd (0);
  grad (1) = gd (1);
  grad (2) = gd (2);
}

void
NurbsCurve::InsertKnot (double xi)
{
  // http://www.cs.mtu.edu/~shene/COURSES/cs3621/NOTES/spline/NURBS-knot-insert.html
  unsigned p = basis.degree;
  unsigned k = basis.GetSpan (xi);
  vector_vec4 q;
  for (unsigned i = k - p + 1; i <= k; i++)
  {
    double ai = (xi - basis.knots[i]) / (basis.knots[i + p] - basis.knots[i]);
    vec4 qi = (1.0 - ai) * cps[i - 1] + ai * cps[i];
    q.push_back (qi);
  }

  vector_vec4 cps_new;
  for (unsigned i = 0; i <= k - p; i++)
    cps_new.push_back (cps[i]);

  for (unsigned i = 0; i < q.size (); i++)
    cps_new.push_back (q[i]);

  for (unsigned i = k; i < cps.size (); i++)
    cps_new.push_back (cps[i]);

  cps = cps_new;
  basis.InsertKnot (xi);
}

void
NurbsCurve::Dump () const
{
  printf ("[NurbsCurve::Dump]");
  printf ("  Degree: %d\n", basis.degree);
  printf ("  Knot Vector:\n");
  for (unsigned i = 0; i < basis.knots.size (); i++)
    printf ("    [%d] %.2f\n", i, basis.knots[i]);
  printf ("  Control Points:\n");
  for (unsigned i = 0; i < cps.size (); i++)
    printf ("    [%d] %.2f  %.2f  %.2f\n", i, cps[i] (0), cps[i] (1), cps[i] (2));
}
