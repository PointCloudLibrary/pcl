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

using namespace pcl;
using namespace nurbs;

NurbsSurface::NurbsSurface ()
{
}

NurbsSurface::NurbsSurface (unsigned degree, unsigned ncpsU, unsigned ncpsV, const vector_vec4 &cps) :
  basisU (degree, ncpsU), basisV (degree, ncpsV)
{
  this->cps = cps;

  this->ncpsU = ncpsU;
  this->ncpsV = ncpsV;
}

void
NurbsSurface::Evaluate (double u, double v, vec3 &point) const
{
  vec4 pt (0.0f, 0.0f, 0.0f, 0.0f);

  int spanU = basisU.GetSpan (u);
  if (spanU < 0)
    throw std::runtime_error ("[NurbsCurve::Evaluate] Paramater value 'u' out of range");
  int spanV = basisV.GetSpan (v);
  if (spanV < 0)
    throw std::runtime_error ("[NurbsCurve::Evaluate] Paramater value 'v' out of range");

  std::vector<double> Nu, Nv, Ndu, Ndv;
  basisU.Cox (u, spanU, Nu, Ndu);
  basisV.Cox (v, spanV, Nv, Ndv);

  unsigned degreeU = basisU.degree;
  unsigned degreeV = basisV.degree;

  for (unsigned su = 0; su < (degreeU + 1); su++)
  {
    for (unsigned sv = 0; sv < (degreeV + 1); sv++)
    {
      vec4 B = GetCP (spanU - degreeU + su, spanV - degreeV + sv);
      double R = Nu[su + degreeU * (degreeU + 1)] * Nv[sv + degreeV * (degreeV + 1)]; // * B.w; // * dW;
      pt = pt + B * R;
    }
  }
  point (0) = pt (0);
  point (1) = pt (1);
  point (2) = pt (2);
}

void
NurbsSurface::Evaluate (double u, double v, vec3 &point, vec3 &gradU, vec3 &gradV) const
{
  vec4 pt (0.0f, 0.0f, 0.0f, 0.0f);
  vec4 gu (0.0f, 0.0f, 0.0f, 0.0f);
  vec4 gv (0.0f, 0.0f, 0.0f, 0.0f);

  int spanU = basisU.GetSpan (u);
  if (spanU < 0)
    throw std::runtime_error ("[NurbsCurve::Evaluate] Paramater value 'u' out of range");
  int spanV = basisV.GetSpan (v);
  if (spanV < 0)
    throw std::runtime_error ("[NurbsCurve::Evaluate] Paramater value 'v' out of range");

  std::vector<double> Nu, Nv, Ndu, Ndv;
  basisU.Cox (u, spanU, Nu, Ndu);
  basisV.Cox (v, spanV, Nv, Ndv);

  unsigned degreeU = basisU.degree;
  unsigned degreeV = basisV.degree;

  //  printf("knotsV: %d spanV: %d degreeV: %d %f %f\n", basisV.knots.size(), spanV, degreeV, u, v);

  for (unsigned su = 0; su < (degreeU + 1); su++)
  {
    for (unsigned sv = 0; sv < (degreeV + 1); sv++)
    {
      vec4 B = GetCP (spanU - degreeU + su, spanV - degreeV + sv);
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
NurbsSurface::InsertKnotU (double u)
{
  // http://www.cs.mtu.edu/~shene/COURSES/cs3621/LAB/surface/knot-insrt.html
  unsigned p = basisU.degree;
  unsigned k = basisU.GetSpan (u);

  vector_vec4 cps_new;

  for (unsigned j = 0; j < ncpsV; j++)
  {

    for (unsigned i = 0; i <= k - p; i++)
      cps_new.push_back (cps[I (i, j)]);

    for (unsigned i = k - p + 1; i <= k; i++)
    {
      double ai = (u - basisU.knots[i]) / (basisU.knots[i + p] - basisU.knots[i]);
      vec4 qi = (1.0 - ai) * cps[I (i - 1, j)] + ai * cps[I (i, j)];
      //      vec4 qi = (1.0 - ai) * cv[i - 1] + ai * cv[i];
      cps_new.push_back (qi);
    }

    //    for (unsigned i = 0; i < q.size(); i++)
    //      cps_new.push_back(q[i]);

    for (unsigned i = k; i < ncpsU; i++)
      cps_new.push_back (cps[I (i, j)]);

  }

  ncpsU++;
  cps = cps_new;
  basisU.InsertKnot (u);
}

void
NurbsSurface::InsertKnotV (double v)
{
  // http://www.cs.mtu.edu/~shene/COURSES/cs3621/LAB/surface/knot-insrt.html
  unsigned p = basisV.degree;
  unsigned k = basisV.GetSpan (v);

  vector_vec4 cps_new;

  for (unsigned j = 0; j <= k - p; j++)
  {
    for (unsigned i = 0; i < ncpsU; i++)
    {
      cps_new.push_back (cps[I (i, j)]);
    }
  }

  for (unsigned j = k - p + 1; j <= k; j++)
  {
    for (unsigned i = 0; i < ncpsU; i++)
    {
      double aj = (v - basisV.knots[j]) / (basisV.knots[j + p] - basisV.knots[j]);
      vec4 qj = (1.0 - aj) * cps[I (i, j - 1)] + aj * cps[I (i, j)];
      //      vec4 qi = (1.0 - ai) * cv[i - 1] + ai * cv[i];
      cps_new.push_back (qj);
    }
  }

  for (unsigned j = k; j < ncpsV; j++)
  {
    for (unsigned i = 0; i < ncpsU; i++)
    {
      cps_new.push_back (cps[I (i, j)]);
    }
  }

  ncpsV++;
  cps = cps_new;
  basisV.InsertKnot (v);
}

unsigned
NurbsSurface::I (unsigned i, unsigned j) const
{
  return (j * ncpsU + i);
}

vec4
NurbsSurface::GetCP (unsigned i, unsigned j) const
{
  unsigned idx = I (i, j);
  if (idx >= cps.size () || idx < 0)
  {
    printf ("[NurbsSurface::GetCP] Warning: index out of bounds. %d %d (i: %d, j: %d)\n", (unsigned)cps.size (), idx,
            i, j);
    return cps[0];
  }
  return cps[idx];
}

void
NurbsSurface::SetCP (unsigned i, unsigned j, const vec4 &cp)
{
  unsigned idx = I (i, j);
  if (idx >= cps.size () || idx < 0)
  {
    printf ("[NurbsSurface::GetCP] Warning: index out of bounds. %d %d\n", (unsigned)cps.size (), idx);
  }
  else
  {
    cps[idx] = cp;
  }
}

void
NurbsSurface::Dump () const
{
  printf ("\n[NurbsSurface::Dump]");
  printf ("  Degree: u: %d  v: %d\n", basisU.degree, basisV.degree);
  printf ("  Knot Vector:\n    u: ");
  for (unsigned i = 0; i < basisU.knots.size (); i++)
  {
    if (basisU.knots[i] >= 0.0)
      printf ("  %.4f, ", basisU.knots[i]);
    else
      printf (" %.4f, ", basisU.knots[i]);
  }
  printf ("\n    v: ");
  for (unsigned j = 0; j < basisV.knots.size (); j++)
  {
    if (basisV.knots[j] >= 0.0)
      printf ("  %.4f, ", basisV.knots[j]);
    else
      printf (" %.4f, ", basisV.knots[j]);
  }

  printf ("\n  Control Points:\n");
  for (unsigned i = 0; i < ncpsU; i++)
  {
    for (unsigned j = 0; j < ncpsU; j++)
    {
      printf ("    CV [%d][%d] ", i, j);
      vec4 cp = cps[I (i, j)];
      for (unsigned k = 0; k < 3; k++)
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
