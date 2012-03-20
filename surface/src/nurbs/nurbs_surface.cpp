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

#include <pcl/surface/nurbs/nurbs_surface.h>
#include <stdio.h>

void
pcl::nurbs::NurbsSurface::evaluate (double u, double v, vec3 &point) const
{
  vec4 pt (0.0f, 0.0f, 0.0f, 0.0f);

  int spanU = basis_u_.getSpan (u);
  if (spanU < 0)
    throw std::runtime_error ("[NurbsCurve::Evaluate] Paramater value 'u' out of range");
  int spanV = basis_v_.getSpan (v);
  if (spanV < 0)
    throw std::runtime_error ("[NurbsCurve::Evaluate] Paramater value 'v' out of range");

  std::vector<double> Nu, Nv, Ndu, Ndv;
  basis_u_.cox (u, spanU, Nu, Ndu);
  basis_v_.cox (v, spanV, Nv, Ndv);

  std::size_t degreeU = basis_u_.degree ();
  std::size_t degreeV = basis_v_.degree ();

  for (std::size_t su = 0; su < (degreeU + 1); ++su)
  {
    for (std::size_t sv = 0; sv < (degreeV + 1); ++sv)
    {
      vec4 B = getControlPoint (spanU - degreeU + su, spanV - degreeV + sv);
      double R = Nu[su + degreeU * (degreeU + 1)] * Nv[sv + degreeV * (degreeV + 1)]; // * B.w; // * dW;
      pt = pt + B * R;
    }
  }
  point (0) = pt (0);
  point (1) = pt (1);
  point (2) = pt (2);
}

void
pcl::nurbs::NurbsSurface::evaluate (double u, double v, vec3 &point, vec3 &gradU, vec3 &gradV) const
{
  vec4 pt (0.0f, 0.0f, 0.0f, 0.0f);
  vec4 gu (0.0f, 0.0f, 0.0f, 0.0f);
  vec4 gv (0.0f, 0.0f, 0.0f, 0.0f);

  int spanU = basis_u_.getSpan (u);
  if (spanU < 0)
    throw std::runtime_error ("[NurbsCurve::Evaluate] Paramater value 'u' out of range");
  int spanV = basis_v_.getSpan (v);
  if (spanV < 0)
    throw std::runtime_error ("[NurbsCurve::Evaluate] Paramater value 'v' out of range");

  std::vector<double> Nu, Nv, Ndu, Ndv;
  basis_u_.cox (u, spanU, Nu, Ndu);
  basis_v_.cox (v, spanV, Nv, Ndv);

  std::size_t degreeU = basis_u_.degree ();
  std::size_t degreeV = basis_v_.degree ();

  //  printf("knotsV: %d spanV: %d degreeV: %d %f %f\n", basisV.knots.size(), spanV, degreeV, u, v);

  for (std::size_t su = 0; su < (degreeU + 1); ++su)
  {
    for (std::size_t sv = 0; sv < (degreeV + 1); ++sv)
    {
      vec4 B = getControlPoint (spanU - degreeU + su, spanV - degreeV + sv);
      double R = Nu[su + degreeU * (degreeU + 1)] * Nv[sv + degreeV * (degreeV + 1)]; // * B.w; // * dW;
      pt = pt + B * R;

      double dU = Ndu[su + degreeU * (degreeU + 1)] * Nv[sv + degreeV * (degreeV + 1)];
      double dV = Nu[su + degreeU * (degreeU + 1)] * Ndv[sv + degreeV * (degreeV + 1)];
      gu = gu + B * dU;
      gv = gv + B * dV;
    }
  }

  point (0) = pt (0);
  point (1) = pt (1);
  point (2) = pt (2);

  gradU (0) = gu (0);
  gradU (1) = gu (1);
  gradU (2) = gu (2);

  gradV (0) = gv (0);
  gradV (1) = gv (1);
  gradV (2) = gv (2);
}

void
pcl::nurbs::NurbsSurface::insertKnotU (double u)
{
  // http://www.cs.mtu.edu/~shene/COURSES/cs3621/LAB/surface/knot-insrt.html
  std::size_t p = basis_u_.degree ();
  std::size_t k = basis_u_.getSpan (u);

  vector_vec4 cps_new;

  for (std::size_t j = 0; j < nb_control_points_v_; ++j)
  {

    for (std::size_t i = 0; i <= k - p; ++i)
      cps_new.push_back (control_points_[index (i, j)]);

    for (std::size_t i = k - p + 1; i <= k; ++i)
    {
      double ai = (u - basis_u_.knot (i)) / (basis_u_.knot (i + p) - basis_u_.knot (i));
      vec4 qi = (1.0 - ai) * control_points_[index (i - 1, j)] + ai * control_points_[index (i, j)];
      //      vec4 qi = (1.0 - ai) * cv[i - 1] + ai * cv[i];
      cps_new.push_back (qi);
    }

    //    for (std::size_t i = 0; i < q.size(); i++)
    //      cps_new.push_back(q[i]);

    for (std::size_t i = k; i < nb_control_points_u_; i++)
      cps_new.push_back (control_points_[index (i, j)]);

  }

  nb_control_points_u_++;
  control_points_ = cps_new;
  basis_u_.insertKnot (u);
}

void
pcl::nurbs::NurbsSurface::insertKnotV (double v)
{
  // http://www.cs.mtu.edu/~shene/COURSES/cs3621/LAB/surface/knot-insrt.html
  std::size_t p = basis_v_.degree ();
  std::size_t k = basis_v_.getSpan (v);

  vector_vec4 cps_new;

  for (std::size_t j = 0; j <= k - p; ++j)
  {
    for (std::size_t i = 0; i < nb_control_points_u_; ++i)
    {
      cps_new.push_back (control_points_[index (i, j)]);
    }
  }

  for (std::size_t j = k - p + 1; j <= k; ++j)
  {
    for (std::size_t i = 0; i < nb_control_points_u_; ++i)
    {
      double aj = (v - basis_v_.knot (j)) / (basis_v_.knot (j + p) - basis_v_.knot (j));
      vec4 qj = (1.0 - aj) * control_points_[index (i, j - 1)] + aj * control_points_[index (i, j)];
      //      vec4 qi = (1.0 - ai) * cv[i - 1] + ai * cv[i];
      cps_new.push_back (qj);
    }
  }

  for (std::size_t j = k; j < nb_control_points_v_; ++j)
  {
    for (std::size_t i = 0; i < nb_control_points_u_; ++i)
    {
      cps_new.push_back (control_points_[index (i, j)]);
    }
  }

  nb_control_points_v_++;
  control_points_ = cps_new;
  basis_v_.insertKnot (v);
}

std::size_t
pcl::nurbs::NurbsSurface::index (std::size_t i, std::size_t j) const
{
  return (j * nb_control_points_u_ + i);
}

pcl::nurbs::vec4
pcl::nurbs::NurbsSurface::getControlPoint (std::size_t i, std::size_t j) const
{
  std::size_t idx = index (i, j);
  if (idx >= control_points_.size ())
  {
    printf ("[pcl::nurbs::NurbsSurface::GetCP] Warning: index out of bounds. %d %d (i: %d, j: %d)\n", (std::size_t)control_points_.size (), idx,
            i, j);
    return control_points_[0];
  }
  return control_points_[idx];
}

void
pcl::nurbs::NurbsSurface::setControlPoint (std::size_t i, std::size_t j, const vec4 &cp)
{
  std::size_t idx = index (i, j);
  if (idx >= control_points_.size ())
  {
    printf ("[pcl::nurbs::NurbsSurface::GetCP] Warning: index out of bounds. %d %d\n", control_points_.size (), idx);
  }
  else
  {
    control_points_[idx] = cp;
  }
}

void
pcl::nurbs::NurbsSurface::dump () const
{
  printf ("\n[pcl::nurbs::NurbsSurface::Dump]");
  printf ("  Degree: u: %d  v: %d\n", basis_u_.degree (), basis_v_.degree ());
  printf ("  Knot Vector:\n    u: ");
  for (std::size_t i = 0; i < basis_u_.nbKnots (); ++i)
  {
    if (basis_u_.knot (i) >= 0.0)
      printf ("  %.4f, ", basis_u_.knot (i));
    else
      printf (" %.4f, ", basis_u_.knot (i));
  }
  printf ("\n    v: ");
  for (std::size_t j = 0; j < basis_v_.nbKnots (); ++j)
  {
    if (basis_v_.knot (j) >= 0.0)
      printf ("  %.4f, ", basis_v_.knot (j));
    else
      printf (" %.4f, ", basis_v_.knot (j));
  }

  printf ("\n  Control Points:\n");
  for (std::size_t i = 0; i < nb_control_points_u_; ++i)
  {
    for (std::size_t j = 0; j < nb_control_points_u_; ++j)
    {
      printf ("    CV [%d][%d] ", i, j);
      vec4 cp = control_points_[index (i, j)];
      for (std::size_t k = 0; k < 3; k++)
      {
        if (cp (k) >= 0.0)
          printf ("  %f", cp (k));
        else
          printf (" %f", cp (k));
      }
      printf ("\n");
    }
    printf ("\n");
  }
}
