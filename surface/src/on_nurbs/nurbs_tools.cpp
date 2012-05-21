/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Thomas Mörwald, Jonathan Balzer, Inc.
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
 * @author thomas.moerwald
 *
 */

#include <iostream>
#include <stdexcept>
#include <map>
#include <algorithm>
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

  unsigned s = data1.size ();
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

  unsigned s = data.size ();

  vector_vec3d data_tmp;

  for (unsigned i = 0; i < size; i++)
  {
    unsigned rnd = unsigned ((s - 1) * (double (rand ()) / RAND_MAX));
    data_tmp.push_back (data[rnd]);
  }

  data = data_tmp;
}

std::list<unsigned>
NurbsTools::getClosestPoints (const Eigen::Vector2d &p, const vector_vec2d &data, unsigned s)
{
  if (data.empty ())
    throw std::runtime_error ("[NurbsTools::getClosestPoint(2d)] Data empty.\n");

  std::list<unsigned> idxs;

  double dist2 (DBL_MAX);
  for (unsigned i = 0; i < data.size (); i++)
  {
    double d2 = (data[i] - p).squaredNorm ();
    if (d2 < dist2)
    {
      idxs.push_front (i);
      if (idxs.size () > s)
        idxs.pop_back ();
      dist2 = d2;
    }
  }

  std::list<unsigned>::iterator it;
  printf ("NurbsTools::getClosestPoints");
  for (it = idxs.begin (); it != idxs.end (); it++)
  {
    printf (" %d", *it);
  }
  printf ("\n");

  return idxs;
}

unsigned
NurbsTools::getClosestPoint (const Eigen::Vector2d &p, const vector_vec2d &data)
{
  if (data.empty ())
    throw std::runtime_error ("[NurbsTools::getClosestPoint(2d)] Data empty.\n");

  unsigned idx (0);
  double dist2 (DBL_MAX);
  for (unsigned i = 0; i < data.size (); i++)
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

  unsigned idx (0);
  double dist2 (DBL_MAX);
  for (unsigned i = 0; i < data.size (); i++)
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

  unsigned idx (0);
  idxcp = 0;
  double dist2 (0.0);
  double dist2cp (DBL_MAX);
  for (unsigned i = 0; i < data.size (); i++)
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

  unsigned s = data.size ();
  double ds = 1.0 / s;

  for (unsigned i = 0; i < s; i++)
    u += (data[i] * ds);

  return u;
}

Eigen::Vector2d
NurbsTools::computeMean (const vector_vec2d &data)
{
  Eigen::Vector2d u (0.0, 0.0);

  unsigned s = data.size ();
  double ds = 1.0 / s;

  for (unsigned i = 0; i < s; i++)
    u += (data[i] * ds);

  return u;
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

  unsigned s = data.size ();

  Eigen::MatrixXd Q (3, s);

  for (unsigned i = 0; i < s; i++)
    Q.col (i) << (data[i] - mean);

  Eigen::Matrix3d C = Q * Q.transpose ();

  Eigen::SelfAdjointEigenSolver < Eigen::Matrix3d > eigensolver (C);
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

