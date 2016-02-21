/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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
 * $Id$
 *
 */

#ifndef PCL_COMMON_IMPL_H_
#define PCL_COMMON_IMPL_H_

#include <pcl/point_types.h>
#include <pcl/common/common.h>

//////////////////////////////////////////////////////////////////////////////////////////////
inline double
pcl::getAngle3D (const Eigen::Vector4f &v1, const Eigen::Vector4f &v2, const bool in_degree)
{
  // Compute the actual angle
  double rad = v1.normalized ().dot (v2.normalized ());
  if (rad < -1.0)
    rad = -1.0;
  else if (rad >  1.0)
    rad = 1.0;
  return (in_degree ? acos (rad) * 180.0 / M_PI : acos (rad));
}

inline double
pcl::getAngle3D (const Eigen::Vector3f &v1, const Eigen::Vector3f &v2, const bool in_degree)
{
  // Compute the actual angle
  double rad = v1.normalized ().dot (v2.normalized ());
  if (rad < -1.0)
    rad = -1.0;
  else if (rad >  1.0)
    rad = 1.0;
  return (in_degree ? acos (rad) * 180.0 / M_PI : acos (rad));
}

//////////////////////////////////////////////////////////////////////////////////////////////
inline void
pcl::getMeanStd (const std::vector<float> &values, double &mean, double &stddev)
{
  double sum = 0, sq_sum = 0;

  for (size_t i = 0; i < values.size (); ++i)
  {
    sum += values[i];
    sq_sum += values[i] * values[i];
  }
  mean = sum / static_cast<double>(values.size ());
  double variance = (sq_sum - sum * sum / static_cast<double>(values.size ())) / (static_cast<double>(values.size ()) - 1);
  stddev = sqrt (variance);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline void
pcl::getPointsInBox (const pcl::PointCloud<PointT> &cloud, 
                     Eigen::Vector4f &min_pt, Eigen::Vector4f &max_pt,
                     std::vector<int> &indices)
{
  indices.resize (cloud.points.size ());
  int l = 0;

  // If the data is dense, we don't need to check for NaN
  if (cloud.is_dense)
  {
    for (size_t i = 0; i < cloud.points.size (); ++i)
    {
      // Check if the point is inside bounds
      if (cloud.points[i].x < min_pt[0] || cloud.points[i].y < min_pt[1] || cloud.points[i].z < min_pt[2])
        continue;
      if (cloud.points[i].x > max_pt[0] || cloud.points[i].y > max_pt[1] || cloud.points[i].z > max_pt[2])
        continue;
      indices[l++] = int (i);
    }
  }
  // NaN or Inf values could exist => check for them
  else
  {
    for (size_t i = 0; i < cloud.points.size (); ++i)
    {
      // Check if the point is invalid
      if (!pcl_isfinite (cloud.points[i].x) || 
          !pcl_isfinite (cloud.points[i].y) || 
          !pcl_isfinite (cloud.points[i].z))
        continue;
      // Check if the point is inside bounds
      if (cloud.points[i].x < min_pt[0] || cloud.points[i].y < min_pt[1] || cloud.points[i].z < min_pt[2])
        continue;
      if (cloud.points[i].x > max_pt[0] || cloud.points[i].y > max_pt[1] || cloud.points[i].z > max_pt[2])
        continue;
      indices[l++] = int (i);
    }
  }
  indices.resize (l);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT> inline void
pcl::getMaxDistance (const pcl::PointCloud<PointT> &cloud, const Eigen::Vector4f &pivot_pt, Eigen::Vector4f &max_pt)
{
  float max_dist = -FLT_MAX;
  int max_idx = -1;
  float dist;
  const Eigen::Vector3f pivot_pt3 = pivot_pt.head<3> ();

  // If the data is dense, we don't need to check for NaN
  if (cloud.is_dense)
  {
    for (size_t i = 0; i < cloud.points.size (); ++i)
    {
      pcl::Vector3fMapConst pt = cloud.points[i].getVector3fMap ();
      dist = (pivot_pt3 - pt).norm ();
      if (dist > max_dist)
      {
        max_idx = int (i);
        max_dist = dist;
      }
    }
  }
  // NaN or Inf values could exist => check for them
  else
  {
    for (size_t i = 0; i < cloud.points.size (); ++i)
    {
      // Check if the point is invalid
      if (!pcl_isfinite (cloud.points[i].x) || !pcl_isfinite (cloud.points[i].y) || !pcl_isfinite (cloud.points[i].z))
        continue;
      pcl::Vector3fMapConst pt = cloud.points[i].getVector3fMap ();
      dist = (pivot_pt3 - pt).norm ();
      if (dist > max_dist)
      {
        max_idx = int (i);
        max_dist = dist;
      }
    }
  }

  if(max_idx != -1)
    max_pt = cloud.points[max_idx].getVector4fMap ();
  else
    max_pt = Eigen::Vector4f(std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::quiet_NaN());
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT> inline void
pcl::getMaxDistance (const pcl::PointCloud<PointT> &cloud, const std::vector<int> &indices,
                     const Eigen::Vector4f &pivot_pt, Eigen::Vector4f &max_pt)
{
  float max_dist = -FLT_MAX;
  int max_idx = -1;
  float dist;
  const Eigen::Vector3f pivot_pt3 = pivot_pt.head<3> ();

  // If the data is dense, we don't need to check for NaN
  if (cloud.is_dense)
  {
    for (size_t i = 0; i < indices.size (); ++i)
    {
      pcl::Vector3fMapConst pt = cloud.points[indices[i]].getVector3fMap ();
      dist = (pivot_pt3 - pt).norm ();
      if (dist > max_dist)
      {
        max_idx = static_cast<int> (i);
        max_dist = dist;
      }
    }
  }
  // NaN or Inf values could exist => check for them
  else
  {
    for (size_t i = 0; i < indices.size (); ++i)
    {
      // Check if the point is invalid
      if (!pcl_isfinite (cloud.points[indices[i]].x) || !pcl_isfinite (cloud.points[indices[i]].y)
          ||
          !pcl_isfinite (cloud.points[indices[i]].z))
        continue;

      pcl::Vector3fMapConst pt = cloud.points[indices[i]].getVector3fMap ();
      dist = (pivot_pt3 - pt).norm ();
      if (dist > max_dist)
      {
        max_idx = static_cast<int> (i);
        max_dist = dist;
      }
    }
  }

  if(max_idx != -1)
    max_pt = cloud.points[indices[max_idx]].getVector4fMap ();
  else
    max_pt = Eigen::Vector4f(std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::quiet_NaN());
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline void
pcl::getMinMax3D (const pcl::PointCloud<PointT> &cloud, PointT &min_pt, PointT &max_pt)
{
  Eigen::Array4f min_p, max_p;
  min_p.setConstant (FLT_MAX);
  max_p.setConstant (-FLT_MAX);

  // If the data is dense, we don't need to check for NaN
  if (cloud.is_dense)
  {
    for (size_t i = 0; i < cloud.points.size (); ++i)
    {
      pcl::Array4fMapConst pt = cloud.points[i].getArray4fMap ();
      min_p = min_p.min (pt);
      max_p = max_p.max (pt);
    }
  }
  // NaN or Inf values could exist => check for them
  else
  {
    for (size_t i = 0; i < cloud.points.size (); ++i)
    {
      // Check if the point is invalid
      if (!pcl_isfinite (cloud.points[i].x) || 
          !pcl_isfinite (cloud.points[i].y) || 
          !pcl_isfinite (cloud.points[i].z))
        continue;
      pcl::Array4fMapConst pt = cloud.points[i].getArray4fMap ();
      min_p = min_p.min (pt);
      max_p = max_p.max (pt);
    }
  }
  min_pt.x = min_p[0]; min_pt.y = min_p[1]; min_pt.z = min_p[2];
  max_pt.x = max_p[0]; max_pt.y = max_p[1]; max_pt.z = max_p[2];
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline void
pcl::getMinMax3D (const pcl::PointCloud<PointT> &cloud, Eigen::Vector4f &min_pt, Eigen::Vector4f &max_pt)
{
  Eigen::Array4f min_p, max_p;
  min_p.setConstant (FLT_MAX);
  max_p.setConstant (-FLT_MAX);

  // If the data is dense, we don't need to check for NaN
  if (cloud.is_dense)
  {
    for (size_t i = 0; i < cloud.points.size (); ++i)
    {
      pcl::Array4fMapConst pt = cloud.points[i].getArray4fMap ();
      min_p = min_p.min (pt);
      max_p = max_p.max (pt);
    }
  }
  // NaN or Inf values could exist => check for them
  else
  {
    for (size_t i = 0; i < cloud.points.size (); ++i)
    {
      // Check if the point is invalid
      if (!pcl_isfinite (cloud.points[i].x) || 
          !pcl_isfinite (cloud.points[i].y) || 
          !pcl_isfinite (cloud.points[i].z))
        continue;
      pcl::Array4fMapConst pt = cloud.points[i].getArray4fMap ();
      min_p = min_p.min (pt);
      max_p = max_p.max (pt);
    }
  }
  min_pt = min_p;
  max_pt = max_p;
}


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline void
pcl::getMinMax3D (const pcl::PointCloud<PointT> &cloud, const pcl::PointIndices &indices,
                  Eigen::Vector4f &min_pt, Eigen::Vector4f &max_pt)
{
  Eigen::Array4f min_p, max_p;
  min_p.setConstant (FLT_MAX);
  max_p.setConstant (-FLT_MAX);

  // If the data is dense, we don't need to check for NaN
  if (cloud.is_dense)
  {
    for (size_t i = 0; i < indices.indices.size (); ++i)
    {
      pcl::Array4fMapConst pt = cloud.points[indices.indices[i]].getArray4fMap ();
      min_p = min_p.min (pt);
      max_p = max_p.max (pt);
    }
  }
  // NaN or Inf values could exist => check for them
  else
  {
    for (size_t i = 0; i < indices.indices.size (); ++i)
    {
      // Check if the point is invalid
      if (!pcl_isfinite (cloud.points[indices.indices[i]].x) || 
          !pcl_isfinite (cloud.points[indices.indices[i]].y) || 
          !pcl_isfinite (cloud.points[indices.indices[i]].z))
        continue;
      pcl::Array4fMapConst pt = cloud.points[indices.indices[i]].getArray4fMap ();
      min_p = min_p.min (pt);
      max_p = max_p.max (pt);
    }
  }
  min_pt = min_p;
  max_pt = max_p;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline void
pcl::getMinMax3D (const pcl::PointCloud<PointT> &cloud, const std::vector<int> &indices,
                  Eigen::Vector4f &min_pt, Eigen::Vector4f &max_pt)
{
  min_pt.setConstant (FLT_MAX);
  max_pt.setConstant (-FLT_MAX);

  // If the data is dense, we don't need to check for NaN
  if (cloud.is_dense)
  {
    for (size_t i = 0; i < indices.size (); ++i)
    {
      pcl::Array4fMapConst pt = cloud.points[indices[i]].getArray4fMap ();
      min_pt = min_pt.array ().min (pt);
      max_pt = max_pt.array ().max (pt);
    }
  }
  // NaN or Inf values could exist => check for them
  else
  {
    for (size_t i = 0; i < indices.size (); ++i)
    {
      // Check if the point is invalid
      if (!pcl_isfinite (cloud.points[indices[i]].x) || 
          !pcl_isfinite (cloud.points[indices[i]].y) || 
          !pcl_isfinite (cloud.points[indices[i]].z))
        continue;
      pcl::Array4fMapConst pt = cloud.points[indices[i]].getArray4fMap ();
      min_pt = min_pt.array ().min (pt);
      max_pt = max_pt.array ().max (pt);
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline double 
pcl::getCircumcircleRadius (const PointT &pa, const PointT &pb, const PointT &pc)
{
  Eigen::Vector4f p1 (pa.x, pa.y, pa.z, 0);
  Eigen::Vector4f p2 (pb.x, pb.y, pb.z, 0);
  Eigen::Vector4f p3 (pc.x, pc.y, pc.z, 0);

  double p2p1 = (p2 - p1).norm (), p3p2 = (p3 - p2).norm (), p1p3 = (p1 - p3).norm ();
  // Calculate the area of the triangle using Heron's formula 
  // (http://en.wikipedia.org/wiki/Heron's_formula)
  double semiperimeter = (p2p1 + p3p2 + p1p3) / 2.0;
  double area = sqrt (semiperimeter * (semiperimeter - p2p1) * (semiperimeter - p3p2) * (semiperimeter - p1p3));
  // Compute the radius of the circumscribed circle
  return ((p2p1 * p3p2 * p1p3) / (4.0 * area));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline void 
pcl::getMinMax (const PointT &histogram, int len, float &min_p, float &max_p)
{
  min_p = FLT_MAX;
  max_p = -FLT_MAX;

  for (int i = 0; i < len; ++i)
  {
    min_p = (histogram[i] > min_p) ? min_p : histogram[i]; 
    max_p = (histogram[i] < max_p) ? max_p : histogram[i]; 
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline float
pcl::calculatePolygonArea (const pcl::PointCloud<PointT> &polygon) 
{
  float area = 0.0f;
  int num_points = polygon.size ();
  int j = 0;
  Eigen::Vector3f va,vb,res;

  res(0) = res(1) = res(2) = 0.0f;
  for (int i = 0; i < num_points; ++i) 
  {
    j = (i + 1) % num_points;
    va = polygon[i].getVector3fMap ();
    vb = polygon[j].getVector3fMap ();
    res += va.cross (vb);
  }
  area = res.norm ();
  return (area*0.5);
}

#endif  //#ifndef PCL_COMMON_IMPL_H_

