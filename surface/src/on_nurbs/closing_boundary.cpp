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

#include <pcl/surface/on_nurbs/closing_boundary.h>

using namespace pcl;
using namespace on_nurbs;

double
ClosingBoundary::getLineDistance (const Eigen::Vector3d &P0, const Eigen::Vector3d &u, const Eigen::Vector3d &Q0,
                                  const Eigen::Vector3d &v, Eigen::Vector3d &P, Eigen::Vector3d &Q)
{
  Eigen::Vector3d w0 = P0 - Q0;

  double a = u.dot (u);
  double b = u.dot (v);
  double c = v.dot (v);
  double d = u.dot (w0);
  double e = v.dot (w0);

  double s = (b * e - c * d) / (a * c - b * b);
  double t = (a * e - b * d) / (a * c - b * b);

  P = P0 + u * s;
  Q = Q0 + v * t;

  Eigen::Vector3d wc = P - Q;
  return wc.norm ();
}

Eigen::Vector3d
ClosingBoundary::intersectPlanes (const Eigen::Vector3d &N1, double d1, const Eigen::Vector3d &N2, double d2,
                                  const Eigen::Vector3d &N3, double d3)
{
  return (N2.cross (N3) * d1 + N3.cross (N1) * d2 + N1.cross (N2) * d3) / N1.dot (N2.cross (N3));
}

Eigen::Vector3d
ClosingBoundary::commonBoundaryPoint1 (
    ON_NurbsSurface &n1, ON_NurbsSurface &n2, 
    Eigen::Vector2d &params1, Eigen::Vector2d &params2, 
    const Eigen::Vector3d &start, 
    unsigned nsteps, double &error, double accuracy)
{
  Eigen::Vector3d current = start;

  double error1 (DBL_MAX);
  double error2 (DBL_MAX);

  Eigen::Vector3d p1, p2, tu1, tu2, tv1, tv2;

  params1 = FittingSurface::findClosestElementMidPoint (n1, current);
  params2 = FittingSurface::findClosestElementMidPoint (n2, current);
  params1 = FittingSurface::inverseMapping (n1, current, params1, error1, p1, tu1, tv1, nsteps, accuracy, true);
  params2 = FittingSurface::inverseMapping (n2, current, params2, error2, p2, tu2, tv2, nsteps, accuracy, true);

  for (unsigned i = 0; i < nsteps; i++)
  {
    params1 = FittingSurface::inverseMapping (n1, current, params1, error1, p1, tu1, tv1, nsteps, accuracy, true);
    params2 = FittingSurface::inverseMapping (n2, current, params2, error2, p2, tu2, tv2, nsteps, accuracy, true);

    //    dbgWin.AddLine3D(current(0), current(1), current(2), p1(0), p1(1), p1(2), 255, 0, 0);
    //    dbgWin.AddLine3D(current(0), current(1), current(2), p2(0), p2(1), p2(2), 255, 0, 0);
    //    dbgWin.AddPoint3D(current(0), current(1), current(2), 255, 0, 0, 3);

    current = (p1 + p2) * 0.5;
  }

  //  dbgWin.AddPoint3D(current(0), current(1), current(2), 255, 0, 255, 5);
  //  dbgWin.Update();

  error = 0.5 * (error1 + error2);

  return current;
}

Eigen::Vector3d
ClosingBoundary::commonBoundaryPoint2 (
    ON_NurbsSurface &n1, ON_NurbsSurface &n2, 
    Eigen::Vector2d &params1, Eigen::Vector2d &params2, 
    const Eigen::Vector3d &start, 
    unsigned nsteps, double &error, double accuracy)
{
  Eigen::Vector3d current = start;

  double error1 (DBL_MAX);
  double error2 (DBL_MAX);

  Eigen::Vector3d p1, p2, tu1, tu2, tv1, tv2;

  params1 = FittingSurface::findClosestElementMidPoint (n1, current);
  params2 = FittingSurface::findClosestElementMidPoint (n2, current);
  params1 = FittingSurface::inverseMapping (n1, current, params1, error1, p1, tu1, tv1, nsteps, accuracy, true);
  params2 = FittingSurface::inverseMapping (n2, current, params2, error2, p2, tu2, tv2, nsteps, accuracy, true);

  for (unsigned i = 0; i < nsteps; i++)
  {
    params1 = FittingSurface::inverseMapping (n1, current, params1, error1, p1, tu1, tv1, nsteps, accuracy, true);
    params2 = FittingSurface::inverseMapping (n2, current, params2, error2, p2, tu2, tv2, nsteps, accuracy, true);

    //    params1 = ntools1.inverseMappingBoundary(current, error1, p1, tu1, tv1, 10, 1e-2, true);
    //    params2 = ntools2.inverseMappingBoundary(current, error2, p2, tu2, tv2, 10, 1e-2, true);

    //    dbgWin.AddLine3D(current(0), current(1), current(2), p1(0), p1(1), p1(2), 0, 0, 255);
    //    dbgWin.AddLine3D(current(0), current(1), current(2), p2(0), p2(1), p2(2), 0, 0, 255);
    //    dbgWin.AddPoint3D(current(0), current(1), current(2), 0, 0, 255, 3);

    //    current = (p1 + p2) * 0.5;
    Eigen::Vector3d n1 = tu1.cross (tv1);
    n1.normalize ();

    Eigen::Vector3d n2 = tu2.cross (tv2);
    n2.normalize ();

    Eigen::Vector3d l1 = (p2 - n1 * n1.dot (p2 - p1)) - p1;
    l1.normalize ();

    Eigen::Vector3d l2 = (p1 - n2 * n2.dot (p1 - p2)) - p2;
    l2.normalize ();

    //        dbgWin.AddLine3D(p1(0), p1(1), p1(2), p1(0) + l1(0), p1(1) + l1(1), p1(2) + l1(2), 255, 0, 0);
    //        dbgWin.AddLine3D(p2(0), p2(1), p2(2), p2(0) + l2(0), p2(1) + l2(1), p2(2) + l2(2), 255, 0, 0);

    Eigen::Vector3d P, Q;
    getLineDistance (p1, l1, p2, l2, P, Q);

    current = (P + Q) * 0.5;
  }

  error = 0.5 * (error1 + error2);

  //  dbgWin.AddPoint3D(current(0), current(1), current(2), 255, 0, 255, 5);
  //  dbgWin.Update();

  return current;
}

Eigen::Vector3d
ClosingBoundary::commonBoundaryPoint3 (
    ON_NurbsSurface &n1, ON_NurbsSurface &n2, 
    Eigen::Vector2d &params1, Eigen::Vector2d &params2, 
    const Eigen::Vector3d &start, 
    unsigned nsteps, double &error, double accuracy)
{
  Eigen::Vector3d current = start;

  double error1 (DBL_MAX);
  double error2 (DBL_MAX);

  Eigen::Vector3d p1, p2, tu1, tu2, tv1, tv2;

  params1 = FittingSurface::findClosestElementMidPoint (n1, current);
  params2 = FittingSurface::findClosestElementMidPoint (n2, current);
  params1 = FittingSurface::inverseMapping (n1, current, params1, error1, p1, tu1, tv1, nsteps, accuracy, true);
  params2 = FittingSurface::inverseMapping (n2, current, params2, error2, p2, tu2, tv2, nsteps, accuracy, true);

  for (unsigned i = 0; i < nsteps; i++)
  {
    params1 = FittingSurface::inverseMapping (n1, current, params1, error1, p1, tu1, tv1, nsteps, accuracy, true);
    params2 = FittingSurface::inverseMapping (n2, current, params2, error2, p2, tu2, tv2, nsteps, accuracy, true);

    //    dbgWin.AddLine3D(current(0), current(1), current(2), p1(0), p1(1), p1(2), 0, 0, 255);
    //    dbgWin.AddLine3D(current(0), current(1), current(2), p2(0), p2(1), p2(2), 0, 0, 255);
    //    dbgWin.AddPoint3D(current(0), current(1), current(2), 0, 0, 255, 3);

    Eigen::Vector3d n1 = tu1.cross (tv1);
    n1.normalize ();
    double d1 = n1.dot (p1);

    Eigen::Vector3d n2 = tu2.cross (tv2);
    n2.normalize ();
    double d2 = n2.dot (p2);

    Eigen::Vector3d n3 = (p1 - current).cross (p2 - current);
    n3.normalize ();
    double d3 = n3.dot (current);

    current = intersectPlanes (n1, d1, n2, d2, n3, d3);
  }

  //  dbgWin.AddPoint3D(current(0), current(1), current(2), 255, 0, 255, 5);
  //  dbgWin.Update();

  error = 0.5 * (error1 + error2);

  return current;
}

void
ClosingBoundary::sampleUniform (ON_NurbsSurface *nurbs, vector_vec3d &point_list, unsigned samples)
{
  double ds = 1.0 / (samples - 1);

  double minU = nurbs->Knot (0, 0);
  double maxU = nurbs->Knot (0, nurbs->KnotCount (0) - 1);
  double minV = nurbs->Knot (1, 0);
  double maxV = nurbs->Knot (1, nurbs->KnotCount (1) - 1);

  Eigen::Vector2d params;
  Eigen::Vector3d point;

  double points[3];

  for (unsigned j = 0; j < samples; j++)
  {
    params (1) = minV + (maxV - minV) * ds * j;
    for (unsigned i = 0; i < samples; i++)
    {
      params (0) = minU + (maxU - minU) * ds * i;
      nurbs->Evaluate (params (0), params (1), 0, 3, points);
      point_list.push_back (Eigen::Vector3d (points[0], points[1], points[2]));
    }
  }
}

void
ClosingBoundary::sampleRandom (ON_NurbsSurface *nurbs, vector_vec3d &point_list, unsigned samples)
{
  double minU = nurbs->Knot (0, 0);
  double maxU = nurbs->Knot (0, nurbs->KnotCount (0) - 1);
  double minV = nurbs->Knot (1, 0);
  double maxV = nurbs->Knot (1, nurbs->KnotCount (1) - 1);

  Eigen::Vector2d params;
  Eigen::Vector3d point;

  double points[3];

  for (unsigned i = 0; i < samples; i++)
  {
    params (0) = minU + (maxU - minU) * (double (rand ()) / RAND_MAX);
    params (1) = minV + (maxV - minV) * (double (rand ()) / RAND_MAX);
    nurbs->Evaluate (params (0), params (1), 0, 3, points);
    point_list.push_back (Eigen::Vector3d (points[0], points[1], points[2]));
  }
}

void
ClosingBoundary::sampleFromBoundary (ON_NurbsSurface *nurbs, vector_vec3d &point_list, vector_vec2d &param_list,
                                     unsigned samples)
{
  double ds = 1.0 / (samples - 1);

  double minU = nurbs->Knot (0, 0);
  double maxU = nurbs->Knot (0, nurbs->KnotCount (0) - 1);
  double minV = nurbs->Knot (1, 0);
  double maxV = nurbs->Knot (1, nurbs->KnotCount (1) - 1);

  Eigen::Vector2d params;
  Eigen::Vector3d point;

  double points[3];

  // WEST
  params (0) = minU;
  for (unsigned i = 0; i < samples; i++)
  {
    params (1) = minV + (maxV - minV) * ds * i;
    nurbs->Evaluate (params (0), params (1), 0, 3, points);
    point_list.push_back (Eigen::Vector3d (points[0], points[1], points[2]));
    param_list.push_back (params);
  }

  // EAST
  params (0) = maxU;
  for (unsigned i = 0; i < samples; i++)
  {
    params (1) = minV + (maxV - minV) * ds * i;
    nurbs->Evaluate (params (0), params (1), 0, 3, points);
    point_list.push_back (Eigen::Vector3d (points[0], points[1], points[2]));
    param_list.push_back (params);
  }

  // SOUTH
  params (1) = minV;
  for (unsigned i = 0; i < samples; i++)
  {
    params (0) = minU + (maxU - minU) * ds * i;
    nurbs->Evaluate (params (0), params (1), 0, 3, points);
    point_list.push_back (Eigen::Vector3d (points[0], points[1], points[2]));
    param_list.push_back (params);
  }

  // NORTH
  params (1) = maxV;
  for (unsigned i = 0; i < samples; i++)
  {
    params (0) = minU + (maxU - minU) * ds * i;
    nurbs->Evaluate (params (0), params (1), 0, 3, points);
    point_list.push_back (Eigen::Vector3d (points[0], points[1], points[2]));
    param_list.push_back (params);
  }
}

void
ClosingBoundary::optimizeBoundary (std::vector<ON_NurbsSurface> &nurbs_list, std::vector<NurbsDataSurface> &data_list,
                                   Parameter param)
{
  for (unsigned n1 = 0; n1 < nurbs_list.size (); n1++)
    data_list[n1].clear_boundary ();

  // for each nurbs
  for (unsigned n1 = 0; n1 < nurbs_list.size (); n1++)
  {
    //  for (unsigned n1 = 0; n1 < 1; n1++) {
    ON_NurbsSurface *nurbs1 = &nurbs_list[n1];

    // sample point list from nurbs1
    vector_vec3d boundary1;
    vector_vec2d params1;
    sampleFromBoundary (nurbs1, boundary1, params1, param.samples);

    // for each other nurbs
    for (unsigned n2 = (n1 + 1); n2 < nurbs_list.size (); n2++)
    {
      ON_NurbsSurface *nurbs2 = &nurbs_list[n2];

      // for all points in the point list
      for (unsigned i = 0; i < boundary1.size (); i++)
      {
        double error;
        Eigen::Vector3d p, tu, tv;
        Eigen::Vector3d p0 = boundary1[i];
        Eigen::Vector2d params1, params2;

        switch (param.type)
        {
          case COMMON_BOUNDARY_POINT_MEAN:
            p = commonBoundaryPoint1 (*nurbs1, *nurbs2, params1, params2, p0, param.com_iter, error, param.accuracy);
            break;
          case COMMON_BOUNDARY_POINT_TANGENTS:
            p = commonBoundaryPoint2 (*nurbs1, *nurbs2, params1, params2, p0, param.com_iter, error, param.accuracy);
            break;
          case COMMON_BOUNDARY_POINT_PLANES:
            p = commonBoundaryPoint3 (*nurbs1, *nurbs2, params1, params2, p0, param.com_iter, error, param.accuracy);
            break;
          case CLOSEST_POINTS_INTERIOR:
            params1 = FittingSurface::findClosestElementMidPoint (*nurbs2, p0);
            FittingSurface::inverseMapping (*nurbs2, p0, params1, error, p, tu, tv, param.com_iter, param.accuracy, true);
            break;
          case CLOSEST_POINTS_BOUNDARY:
            FittingSurface::inverseMappingBoundary (*nurbs2, p0, error, p, tu, tv, param.com_iter, param.accuracy, true);
            break;
        }

        double dist = (p - p0).norm ();
        if (error < param.max_error && dist < param.max_dist)
        {
          data_list[n1].boundary.push_back (p);
          data_list[n2].boundary.push_back (p);
        }
      }

    } // for each other nurbs

    // nurbs fitting
    FittingSurface fit (&data_list[n1], nurbs_list[n1]);
    FittingSurface::Parameter paramFP (1.0, param.smoothness, 0.0, 1.0, param.smoothness, 0.0);

    std::vector<double> wBnd, wInt;
    for (unsigned i = 0; i < data_list[n1].boundary.size (); i++)
      data_list[n1].boundary_weight.push_back (param.boundary_weight);
    for (unsigned i = 0; i < data_list[n1].interior.size (); i++)
      data_list[n1].interior_weight.push_back (param.interior_weight);

    for (unsigned i = 0; i < param.fit_iter; i++)
    {
      fit.assemble (paramFP);
      fit.solve ();
    }
    nurbs_list[n1] = fit.m_nurbs;

  } // for each nurbs

}
