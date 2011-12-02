/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Thomas Mörwald, Jonathan Balzer
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
 *   * Neither the name of Thomas Mörwald or Jonathan Balzer nor the names of its
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
 */

#include <iostream>
#include <stdexcept>
#include "pcl/surface/nurbs/nurbs_tools.h"

using namespace pcl;
using namespace Eigen;

NurbsTools::NurbsTools (ON_NurbsSurface* surf)
{

  surf_ = surf;

}

Vector3d
NurbsTools::x (double u, double v)
{
  Vector3d result;

  double pointAndTangents[3];
  surf_->Evaluate (u, v, 0, 3, pointAndTangents);

  result (0) = pointAndTangents[0];
  result (1) = pointAndTangents[1];
  result (2) = pointAndTangents[2];

  return result;
}

MatrixXd
NurbsTools::jacX (double u, double v) // !
{
  MatrixXd Dx = MatrixXd::Zero (3, 2);

  double pointAndTangents[9];
  surf_->Evaluate (u, v, 1, 3, pointAndTangents);

  Dx (0, 0) = pointAndTangents[3];
  Dx (1, 0) = pointAndTangents[4];
  Dx (2, 0) = pointAndTangents[5];

  Dx (0, 1) = pointAndTangents[6];
  Dx (1, 1) = pointAndTangents[7];
  Dx (2, 1) = pointAndTangents[8];

  return Dx;
}

std::vector<double>
NurbsTools::getElementVector (int dim) // !
{
  std::vector<double> result;

  if (dim == 0)
  {
    int idx_min = 0;
    int idx_max = surf_->m_knot_capacity[0] - 1;
    if (surf_->IsClosed (0))
    {
      idx_min = surf_->m_order[0] - 2;
      idx_max = surf_->m_knot_capacity[0] - surf_->m_order[0] + 1;
    }

    const double* knotsU = surf_->Knot (0);

    result.push_back (knotsU[idx_min]);

    //for(int E=(surf_->m_order[0]-2); E<(surf_->m_knot_capacity[0]-surf_->m_order[0]+2); E++) {
    for (int E = idx_min + 1; E <= idx_max; E++)
    {

      if (knotsU[E] != knotsU[E - 1]) // do not count double knots
        result.push_back (knotsU[E]);

    }

  }
  else if (dim == 1)
  {
    int idx_min = 0;
    int idx_max = surf_->m_knot_capacity[1] - 1;
    if (surf_->IsClosed (1))
    {
      idx_min = surf_->m_order[1] - 2;
      idx_max = surf_->m_knot_capacity[1] - surf_->m_order[1] + 1;
    }
    const double* knotsV = surf_->Knot (1);

    result.push_back (knotsV[idx_min]);

    //for(int F=(surf_->m_order[1]-2); F<(surf_->m_knot_capacity[1]-surf_->m_order[1]+2); F++) {
    for (int F = idx_min + 1; F <= idx_max; F++)
    {

      if (knotsV[F] != knotsV[F - 1])
        result.push_back (knotsV[F]);

    }

  }
  else
    std::cout << "[NurbsTools::getElementVector] ERROR: Index exceeds problem dimensions!" << std::endl;

  return result;
}

std::vector<double>
NurbsTools::getElementVectorDeltas (int dim)
{
  std::vector<double> elements = this->getElementVector (dim);
  std::vector<double> deltas;

  for (unsigned i = 0; i < elements.size () - 1; i++)
  {

    deltas.push_back (elements[i + 1] - elements[i]);

  }

  return deltas;
}

Vector3d
NurbsTools::getDistanceVector (Vector3d pt, Vector2d params)
{
  Vector3d r;
  double pointAndTangents[9];

  surf_->Evaluate (params (0), params (1), 1, 3, pointAndTangents);

  r (0) = pointAndTangents[0] - pt (0);
  r (1) = pointAndTangents[1] - pt (1);
  r (2) = pointAndTangents[2] - pt (2);

  return r;
}

Vector2d
NurbsTools::inverseMapping (Vector3d pt, Vector2d hint, double &error, int maxSteps, double accuracy, bool quiet)
{
  double pointAndTangents[9];

  Vector2d current (hint), delta;
  Matrix2d A;
  Vector2d b;
  Vector3d r, tu, tv;

  std::vector<double> elementsU = getElementVector (0);
  std::vector<double> elementsV = getElementVector (1);
  double minU = elementsU[0];
  double minV = elementsV[0];
  double maxU = elementsU[elementsU.size () - 1];
  double maxV = elementsV[elementsV.size () - 1];

  if (maxSteps <= 0)
    r = getDistanceVector (pt, current);

  for (int k = 0; k < maxSteps; k++)
  {

    surf_->Evaluate (current (0), current (1), 1, 3, pointAndTangents);

    r (0) = pointAndTangents[0] - pt (0);
    r (1) = pointAndTangents[1] - pt (1);
    r (2) = pointAndTangents[2] - pt (2);

    tu (0) = pointAndTangents[3];
    tu (1) = pointAndTangents[4];
    tu (2) = pointAndTangents[5];

    tv (0) = pointAndTangents[6];
    tv (1) = pointAndTangents[7];
    tv (2) = pointAndTangents[8];

    b (0) = -r.dot (tu);
    b (1) = -r.dot (tv);

    A (0, 0) = tu.dot (tu);
    A (0, 1) = tu.dot (tv);
    A (1, 0) = A (0, 1);
    A (1, 1) = tv.dot (tv);

    delta = A.ldlt ().solve (b);

    if (delta.norm () < accuracy)
    {

      error = r.norm ();
      return current;

    }
    else
    {
      current = current + delta;

      //      bool stop = false;

      if (current (0) < minU)
      {
        current (0) = minU;
        //        stop = true;
      }
      else if (current (0) > maxU)
      {
        current (0) = maxU;
        //        stop = true;
      }

      if (current (1) < minV)
      {
        current (1) = minV;
        //        stop = true;
      }
      else if (current (1) > maxV)
      {
        current (1) = maxV;
        //        stop = true;
      }

      //      if( stop ) {
      //        error = arma::norm(r, 2);
      //        return current;
      //      }

    }

  }

  error = r.norm ();

  if (!quiet)
  {
    std::cout << "[NurbsTools::inverseMapping] ERROR: Method did not converge after maximum number of steps!"
        << std::endl;
    std::cout << "  " << hint (0) << " " << hint (1) << " .. " << current (0) << " " << current (1) << std::endl;
  }

  return current;
}

Vector2d
NurbsTools::inverseMappingBoundary (Vector3d pt, double &error, int maxSteps, double accuracy, bool quiet)
{
  Vector2d result;
  double min_err = 100.0;
  std::vector<myvec> ini_points;

  std::vector<double> elementsU = getElementVector (0);
  std::vector<double> elementsV = getElementVector (1);

  // NORTH - SOUTH
  for (unsigned i = 0; i < (elementsV.size () - 1); i++)
  {
    ini_points.push_back (myvec (WEST, elementsV[i] + 0.5 * (elementsV[i + 1] - elementsV[i])));
    ini_points.push_back (myvec (EAST, elementsV[i] + 0.5 * (elementsV[i + 1] - elementsV[i])));
  }

  // WEST - EAST
  for (unsigned i = 0; i < (elementsU.size () - 1); i++)
  {
    ini_points.push_back (myvec (NORTH, elementsU[i] + 0.5 * (elementsU[i + 1] - elementsU[i])));
    ini_points.push_back (myvec (SOUTH, elementsU[i] + 0.5 * (elementsU[i + 1] - elementsU[i])));
  }

  for (unsigned i = 0; i < ini_points.size (); i++)
  {
    double err;
    Vector2d params = inverseMappingBoundary (pt, ini_points[i].side, ini_points[i].hint, err);

    if (i == 0 || err < min_err)
    {
      min_err = err;
      result = params;
    }
  }

  double pointAndTangents[9];
  this->surf_->Evaluate (result (0), result (1), 1, 3, pointAndTangents);

  error = min_err;
  return result;
}

Vector2d
NurbsTools::inverseMappingBoundary (Vector3d pt, int side, double hint, double &error, int maxSteps, double accuracy,
                                    bool quiet)
{
  double pointAndTangents[9];
  double current (hint), delta (DBL_MAX);
  Vector3d r (0.0, 0.0, 0.0), t (0.0);
  Vector2d params;

  std::vector<double> elementsU = getElementVector (0);
  std::vector<double> elementsV = getElementVector (1);
  double minU = elementsU[0];
  double minV = elementsV[0];
  double maxU = elementsU[elementsU.size () - 1];
  double maxV = elementsV[elementsV.size () - 1];

  for (int k = 0; k < maxSteps; k++)
  {

    switch (side)
    {

      case WEST:

        params (0) = minU;
        params (1) = current;
        surf_->Evaluate (minU, current, 1, 3, pointAndTangents);

        t (0) = pointAndTangents[6]; // use tv
        t (1) = pointAndTangents[7];
        t (2) = pointAndTangents[8];

        break;
      case SOUTH:

        params (0) = current;
        params (1) = maxV;
        surf_->Evaluate (current, maxV, 1, 3, pointAndTangents);

        t (0) = pointAndTangents[3]; // use tu
        t (1) = pointAndTangents[4];
        t (2) = pointAndTangents[5];

        break;
      case EAST:

        params (0) = maxU;
        params (1) = current;
        surf_->Evaluate (maxU, current, 1, 3, pointAndTangents);

        t (0) = pointAndTangents[6]; // use tv
        t (1) = pointAndTangents[7];
        t (2) = pointAndTangents[8];

        break;
      case NORTH:

        params (0) = current;
        params (1) = minV;
        surf_->Evaluate (current, minV, 1, 3, pointAndTangents);

        t (0) = pointAndTangents[3]; // use tu
        t (1) = pointAndTangents[4];
        t (2) = pointAndTangents[5];

        break;
      default:
        throw std::runtime_error ("[PatchFitting::inverseMappingBoundary] ERROR: Specify a boundary!");

    } // switch

    r (0) = pointAndTangents[0] - pt (0);
    r (1) = pointAndTangents[1] - pt (1);
    r (2) = pointAndTangents[2] - pt (2);

    delta = -0.5 * r.dot (t) / t.dot (t);

    if (fabs (delta) < accuracy)
    {

      error = r.norm ();
      return params;

    }
    else
    {

      current = current + delta;

      bool stop = false;

      switch (side)
      {

        case WEST:
        case EAST:
          if (current < minV)
          {
            params (1) = minV;
            stop = true;
          }
          else if (current > maxV)
          {
            params (1) = maxV;
            stop = true;
          }

          break;

        case NORTH:
        case SOUTH:
          if (current < minU)
          {
            params (0) = minU;
            stop = true;
          }
          else if (current > maxU)
          {
            params (0) = maxU;
            stop = true;
          }

          break;
      } // switch

      if (stop)
      {
        error = r.norm ();
        return params;
      }

    } // else (fabs (delta) < accuracy)

  } // for (k)

  error = r.norm ();

  if (!quiet)
  {
    std::cout << "[NurbsTools::inverseMappingBoundary] ERROR: Method did not converge after maximum number of steps!"
        << std::endl;
    std::cout << "  error: " << error << "  delta: " << delta << "  params: " << params (0) << " " << params (1)
        << std::endl;
  }

  return params;
}

Vector2d
NurbsTools::inverseMapping (Vector3d pt, Vector2d* phint, double &error, int maxSteps, double accuracy, bool quiet)
{
  Vector3d r;
  Vector2d hint;

  std::vector<double> elementsU = getElementVector (0);
  std::vector<double> elementsV = getElementVector (1);

  if (phint == NULL)
  {
    double d_shortest (DBL_MAX);
    for (unsigned i = 0; i < elementsU.size () - 1; i++)
    {
      for (unsigned j = 0; j < elementsV.size () - 1; j++)
      {
        double points[3];
        double d;

        double xi = elementsU[i] + 0.5 * (elementsU[i + 1] - elementsU[i]);
        double eta = elementsV[j] + 0.5 * (elementsV[j + 1] - elementsV[j]);

        surf_->Evaluate (xi, eta, 0, 3, points);

        r (0) = points[0] - pt (0);
        r (1) = points[1] - pt (1);
        r (2) = points[2] - pt (2);

        d = r.norm ();

        if ((i == 0 && j == 0) || d < d_shortest)
        {
          d_shortest = d;
          hint (0) = xi;
          hint (1) = eta;
        }
      }
    }
  }
  else
  {
    hint = *phint;
  }

  return inverseMapping (pt, hint, error, maxSteps, accuracy, quiet);
}
