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
#include <limits>
#include <stdexcept>
#include <map>
#include <algorithm>
#include <Eigen/Eigenvalues> // for SelfAdjointEigenSolver
#include <Eigen/Geometry> // for cross
#include <pcl/surface/on_nurbs/nurbs_tools.h>

using namespace pcl;
using namespace on_nurbs;
using namespace Eigen;

void
NurbsTools::downsample_random (const vector_vec3d &data1, vector_vec3d &data2, unsigned size)
{
  if (data1.size () <= size && size > 0)
  {
    data2 = data1;
    return;
  }

  unsigned s = unsigned (data1.size ());
  data2.clear ();

  for (unsigned i = 0; i < size; i++)
  {
    unsigned rnd = unsigned (s * (double (rand ()) / RAND_MAX));
    data2.push_back (data1[rnd]);
  }
}

void
NurbsTools::downsample_random (vector_vec3d &data, unsigned size)
{
  if (data.size () <= size && size > 0)
    return;

  unsigned s = unsigned (data.size ());

  vector_vec3d data_tmp;

  for (unsigned i = 0; i < size; i++)
  {
    unsigned rnd = unsigned ((s - 1) * (double (rand ()) / RAND_MAX));
    data_tmp.push_back (data[rnd]);
  }

  data = data_tmp;
}

unsigned
NurbsTools::getClosestPoint (const Eigen::Vector2d &p, const vector_vec2d &data)
{
  if (data.empty ())
    throw std::runtime_error ("[NurbsTools::getClosestPoint(2d)] Data empty.\n");

  std::size_t idx = 0;
  double dist2 (std::numeric_limits<double>::max());
  for (std::size_t i = 0; i < data.size (); i++)
  {
    double d2 = (data[i] - p).squaredNorm ();
    if (d2 < dist2)
    {
      idx = i;
      dist2 = d2;
    }
  }
  return idx;
}

unsigned
NurbsTools::getClosestPoint (const Eigen::Vector3d &p, const vector_vec3d &data)
{
  if (data.empty ())
    throw std::runtime_error ("[NurbsTools::getClosestPoint(2d)] Data empty.\n");

  std::size_t idx = 0;
  double dist2 (std::numeric_limits<double>::max());
  for (std::size_t i = 0; i < data.size (); i++)
  {
    double d2 = (data[i] - p).squaredNorm ();
    if (d2 < dist2)
    {
      idx = i;
      dist2 = d2;
    }
  }
  return idx;
}

unsigned
NurbsTools::getClosestPoint (const Eigen::Vector2d &p, const Eigen::Vector2d &dir, const vector_vec2d &data,
                             unsigned &idxcp)
{
  if (data.empty ())
    throw std::runtime_error ("[NurbsTools::getClosestPoint(2d)] Data empty.\n");

  std::size_t idx = 0;
  idxcp = 0;
  double dist2 (0.0);
  double dist2cp (std::numeric_limits<double>::max());
  for (std::size_t i = 0; i < data.size (); i++)
  {
    Eigen::Vector2d v = (data[i] - p);
    double d2 = v.squaredNorm ();

    if (d2 < dist2cp)
    {
      idxcp = i;
      dist2cp = d2;
    }

    if (d2 == 0.0)
      return i;

    v.normalize ();

    double d1 = dir.dot (v);
    if (d1 / d2 > dist2)
    {
      idx = i;
      dist2 = d1 / d2;
    }
  }
  return idx;
}

Eigen::Vector3d
NurbsTools::computeMean (const vector_vec3d &data)
{
  Eigen::Vector3d u (0.0, 0.0, 0.0);

  unsigned s = unsigned (data.size ());
  double ds = 1.0 / s;

  for (unsigned i = 0; i < s; i++)
    u += (data[i] * ds);

  return u;
}

Eigen::Vector2d
NurbsTools::computeMean (const vector_vec2d &data)
{
  Eigen::Vector2d u (0.0, 0.0);

  std::size_t s = data.size ();
  double ds = 1.0 / double (s);

  for (std::size_t i = 0; i < s; i++)
    u += (data[i] * ds);

  return u;
}

Eigen::Vector3d
NurbsTools::computeVariance (const Eigen::Vector3d &mean, const vector_vec3d &data)
{
  Eigen::Vector3d var (0.0, 0.0, 0.0);

  std::size_t s = data.size ();
  double ds = 1.0 / double (s);

  for (std::size_t i = 0; i < s; i++)
  {
    Eigen::Vector3d v = data[i] - mean;
    var += Eigen::Vector3d (v (0) * v (0) * ds, v (1) * v (1) * ds, v (2) * v (2) * ds);
  }

  return var;
}

Eigen::Vector2d
NurbsTools::computeVariance (const Eigen::Vector2d &mean, const vector_vec2d &data)
{
  Eigen::Vector2d var (0.0, 0.0);

  std::size_t s = data.size ();
  double ds = 1.0 / double (s);

  for (std::size_t i = 0; i < s; i++)
  {
    Eigen::Vector2d v = data[i] - mean;
    var += Eigen::Vector2d (v (0) * v (0) * ds, v (1) * v (1) * ds);
  }

  return var;
}

void
NurbsTools::computeBoundingBox (const ON_NurbsCurve &nurbs, Eigen::Vector3d &_min, Eigen::Vector3d &_max)
{
  _min = Eigen::Vector3d (std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
  _max = Eigen::Vector3d(std::numeric_limits<double>::lowest(),
                         std::numeric_limits<double>::lowest(),
                         std::numeric_limits<double>::lowest());
  for (int i = 0; i < nurbs.CVCount (); i++)
  {
    ON_3dPoint p;
    nurbs.GetCV (i, p);

    if (p.x < _min (0))
      _min (0) = p.x;
    if (p.y < _min (1))
      _min (1) = p.y;
    if (p.z < _min (2))
      _min (2) = p.z;

    if (p.x > _max (0))
      _max (0) = p.x;
    if (p.y > _max (1))
      _max (1) = p.y;
    if (p.z > _max (2))
      _max (2) = p.z;
  }
}

void
NurbsTools::computeBoundingBox (const ON_NurbsSurface &nurbs, Eigen::Vector3d &_min, Eigen::Vector3d &_max)
{
  _min = Eigen::Vector3d(std::numeric_limits<double>::max(),
                         std::numeric_limits<double>::max(),
                         std::numeric_limits<double>::max());
  _max = Eigen::Vector3d(std::numeric_limits<double>::lowest(),
                         std::numeric_limits<double>::lowest(),
                         std::numeric_limits<double>::lowest());
  for (int i = 0; i < nurbs.CVCount (0); i++)
  {
    for (int j = 0; j < nurbs.CVCount (1); j++)
    {
      ON_3dPoint p;
      nurbs.GetCV (i, j, p);

      if (p.x < _min (0))
        _min (0) = p.x;
      if (p.y < _min (1))
        _min (1) = p.y;
      if (p.z < _min (2))
        _min (2) = p.z;

      if (p.x > _max (0))
        _max (0) = p.x;
      if (p.y > _max (1))
        _max (1) = p.y;
      if (p.z > _max (2))
        _max (2) = p.z;
    }
  }
}

double
NurbsTools::computeRScale (const Eigen::Vector3d &_min, const Eigen::Vector3d &_max)
{
  Eigen::Vector3d a = _max - _min;

  return std::max<double> (a (0), std::max<double> (a (1), a (2)));
}

void
NurbsTools::pca (const vector_vec3d &data, Eigen::Vector3d &mean, Eigen::Matrix3d &eigenvectors,
                 Eigen::Vector3d &eigenvalues)
{
  if (data.empty ())
  {
    printf ("[NurbsTools::pca] Error, data is empty\n");
    abort ();
  }

  mean = computeMean (data);

  unsigned s = unsigned (data.size ());

  Eigen::MatrixXd Q (3, s);

  for (unsigned i = 0; i < s; i++)
    Q.col (i) << (data[i] - mean);

  Eigen::Matrix3d C = Q * Q.transpose ();

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver (C);
  if (eigensolver.info () != Success)
  {
    printf ("[NurbsTools::pca] Can not find eigenvalues.\n");
    abort ();
  }

  for (int i = 0; i < 3; ++i)
  {
    eigenvalues (i) = eigensolver.eigenvalues () (2 - i);
    if (i == 2)
      eigenvectors.col (2) = eigenvectors.col (0).cross (eigenvectors.col (1));
    else
      eigenvectors.col (i) = eigensolver.eigenvectors ().col (2 - i);
  }
}

void
NurbsTools::pca (const vector_vec2d &data, Eigen::Vector2d &mean, Eigen::Matrix2d &eigenvectors,
                 Eigen::Vector2d &eigenvalues)
{
  if (data.empty ())
  {
    printf ("[NurbsTools::pca] Error, data is empty\n");
    abort ();
  }

  mean = computeMean (data);

  unsigned s = unsigned (data.size ());

  Eigen::MatrixXd Q (2, s);

  for (unsigned i = 0; i < s; i++)
    Q.col (i) << (data[i] - mean);

  Eigen::Matrix2d C = Q * Q.transpose ();

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigensolver (C);
  if (eigensolver.info () != Success)
  {
    printf ("[NurbsTools::pca] Can not find eigenvalues.\n");
    abort ();
  }

  for (int i = 0; i < 2; ++i)
  {
    eigenvalues (i) = eigensolver.eigenvalues () (1 - i);
    eigenvectors.col (i) = eigensolver.eigenvectors ().col (1 - i);
  }
}

