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

void
pcl::nurbs::NurbsCurve::evaluate (const double &xi, vec3 &point) const
{
  vec4 pt (0.0f, 0.0f, 0.0f, 0.0f);

  int span = basis_.getSpan (xi);
  if (span < 0)
    throw std::runtime_error ("[pcl::nurbs::NurbsCurve::Evaluate] Paramater value 'xi' out of range");

  std::vector<double> N, Nd;
  basis_.cox (xi, span, N, Nd);

  unsigned degree = basis_.degree ();
  for (unsigned s = 0; s < (degree + 1); s++)
  {
    pt = pt + control_points_[span - degree + s] * N[s + degree * (degree + 1)];
  }

  point (0) = pt (0);
  point (1) = pt (1);
  point (2) = pt (2);
}

void
pcl::nurbs::NurbsCurve::evaluate (const double &xi, vec3 &point, vec3 &grad) const
{
  vec4 pt (0.0f, 0.0f, 0.0f, 0.0f);
  vec4 gd (0.0f, 0.0f, 0.0f, 0.0f);

  int span = basis_.getSpan (xi);
  if (span < 0)
    throw std::runtime_error ("[pcl::nurbs::NurbsCurve::Evaluate] Paramater value 'xi' out of range");

  std::vector<double> N, Nd;
  basis_.cox (xi, span, N, Nd);

  std::size_t degree = basis_.degree ();
  for (unsigned s = 0; s < (degree + 1); s++)
  {
    pt = pt + control_points_[span - degree + s] * N[s + degree * (degree + 1)];
    gd = gd + control_points_[span - degree + s] * Nd[s + degree * (degree + 1)];
  }

  point (0) = pt (0);
  point (1) = pt (1);
  point (2) = pt (2);

  grad (0) = gd (0);
  grad (1) = gd (1);
  grad (2) = gd (2);
}

void
pcl::nurbs::NurbsCurve::insertKnot (const double &xi)
{
  // http://www.cs.mtu.edu/~shene/COURSES/cs3621/NOTES/spline/NURBS-knot-insert.html
  std::size_t p = basis_.degree ();
  std::size_t k = basis_.getSpan (xi);
  vector_vec4 q;
  for (std::size_t i = k - p + 1; i <= k; i++)
  {
    double ai = (xi - basis_.knot (i)) / (basis_.knot (i + p) - basis_.knot (i));
    vec4 qi = (1.0 - ai) * control_points_[i - 1] + ai * control_points_[i];
    q.push_back (qi);
  }

  vector_vec4 cps_new;
  for (std::size_t i = 0; i <= k - p; i++)
    cps_new.push_back (control_points_[i]);

  for (std::size_t i = 0; i < q.size (); i++)
    cps_new.push_back (q[i]);

  for (std::size_t i = k; i < control_points_.size (); i++)
    cps_new.push_back (control_points_[i]);

  control_points_ = cps_new;
  basis_.insertKnot (xi);
}

void
pcl::nurbs::NurbsCurve::dump () const
{
  printf ("[pcl::nurbs::NurbsCurve::Dump]");
  printf ("  Degree: %d\n", basis_.degree ());
  printf ("  Knot Vector:\n");
  for (std::size_t i = 0; i < basis_.nbKnots (); i++)
    printf ("    [%d] %.2f\n", i, basis_.knot (i));
  printf ("  Control Points:\n");
  for (std::size_t i = 0; i < control_points_.size (); i++)
    printf ("    [%d] %.2f  %.2f  %.2f\n", i, control_points_[i] (0), control_points_[i] (1), control_points_[i] (2));
}
