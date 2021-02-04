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
#include <cfloat> // for FLT_MAX

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
  return (in_degree ? std::acos (rad) * 180.0 / M_PI : std::acos (rad));
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
  return (in_degree ? std::acos (rad) * 180.0 / M_PI : std::acos (rad));
}

#ifdef __SSE__
inline __m128
pcl::acos_SSE (const __m128 &x)
{
  /*
  This python code generates the coefficients:
  import math, numpy, scipy.optimize
  def get_error(S):
      err_sum=0.0
      for x in numpy.arange(0.0, 1.0, 0.0025):
          if (S[3]+S[4]*x)<0.0:
              err_sum+=10.0
          else:
              err_sum+=((S[0]+x*(S[1]+x*S[2]))*numpy.sqrt(S[3]+S[4]*x)+S[5]+x*(S[6]+x*S[7])-math.acos(x))**2.0
      return err_sum/400.0

  print(scipy.optimize.minimize(fun=get_error, x0=[1.57, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0], method='Nelder-Mead', options={'maxiter':42000, 'maxfev':42000, 'disp':True, 'xatol':1e-6, 'fatol':1e-6}))
  */
  const __m128 mul_term = _mm_add_ps (_mm_set1_ps (1.59121552f), _mm_mul_ps (x, _mm_add_ps (_mm_set1_ps (-0.15461442f), _mm_mul_ps (x, _mm_set1_ps (0.05354897f)))));
  const __m128 add_term = _mm_add_ps (_mm_set1_ps (0.06681017f), _mm_mul_ps (x, _mm_add_ps (_mm_set1_ps (-0.09402311f), _mm_mul_ps (x, _mm_set1_ps (0.02708663f)))));
  return _mm_add_ps (_mm_mul_ps (mul_term, _mm_sqrt_ps (_mm_add_ps (_mm_set1_ps (0.89286965f), _mm_mul_ps (_mm_set1_ps (-0.89282669f), x)))), add_term);
}

inline __m128
pcl::getAcuteAngle3DSSE (const __m128 &x1, const __m128 &y1, const __m128 &z1, const __m128 &x2, const __m128 &y2, const __m128 &z2)
{
  const __m128 dot_product = _mm_add_ps (_mm_add_ps (_mm_mul_ps (x1, x2), _mm_mul_ps (y1, y2)), _mm_mul_ps (z1, z2));
  // The andnot-function realizes an abs-operation: the sign bit is removed
  // -0.0f (negative zero) means that all bits are 0, only the sign bit is 1
  return acos_SSE (_mm_min_ps (_mm_set1_ps (1.0f), _mm_andnot_ps (_mm_set1_ps (-0.0f), dot_product)));
}
#endif // ifdef __SSE__

#ifdef __AVX__
inline __m256
pcl::acos_AVX (const __m256 &x)
{
  const __m256 mul_term = _mm256_add_ps (_mm256_set1_ps (1.59121552f), _mm256_mul_ps (x, _mm256_add_ps (_mm256_set1_ps (-0.15461442f), _mm256_mul_ps (x, _mm256_set1_ps (0.05354897f)))));
  const __m256 add_term = _mm256_add_ps (_mm256_set1_ps (0.06681017f), _mm256_mul_ps (x, _mm256_add_ps (_mm256_set1_ps (-0.09402311f), _mm256_mul_ps (x, _mm256_set1_ps (0.02708663f)))));
  return _mm256_add_ps (_mm256_mul_ps (mul_term, _mm256_sqrt_ps (_mm256_add_ps (_mm256_set1_ps (0.89286965f), _mm256_mul_ps (_mm256_set1_ps (-0.89282669f), x)))), add_term);
}

inline __m256
pcl::getAcuteAngle3DAVX (const __m256 &x1, const __m256 &y1, const __m256 &z1, const __m256 &x2, const __m256 &y2, const __m256 &z2)
{
  const __m256 dot_product = _mm256_add_ps (_mm256_add_ps (_mm256_mul_ps (x1, x2), _mm256_mul_ps (y1, y2)), _mm256_mul_ps (z1, z2));
  // The andnot-function realizes an abs-operation: the sign bit is removed
  // -0.0f (negative zero) means that all bits are 0, only the sign bit is 1
  return acos_AVX (_mm256_min_ps (_mm256_set1_ps (1.0f), _mm256_andnot_ps (_mm256_set1_ps (-0.0f), dot_product)));
}
#endif // ifdef __AVX__

//////////////////////////////////////////////////////////////////////////////////////////////
inline void
pcl::getMeanStd (const std::vector<float> &values, double &mean, double &stddev)
{
  // throw an exception when the input array is empty
  if (values.empty ())
  {
    PCL_THROW_EXCEPTION (BadArgumentException, "Input array must have at least 1 element."); 
  }
  
  // when the array has only one element, mean is the number itself and standard dev is 0
  if (values.size () == 1)
  {
    mean = values.at (0);
    stddev = 0;
    return;
  }
  
  double sum = 0, sq_sum = 0;

  for (const float &value : values)
  {
    sum += value;
    sq_sum += value * value;
  }
  mean = sum / static_cast<double>(values.size ());
  double variance = (sq_sum - sum * sum / static_cast<double>(values.size ())) / (static_cast<double>(values.size ()) - 1);
  stddev = sqrt (variance);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline void
pcl::getPointsInBox (const pcl::PointCloud<PointT> &cloud, 
                     Eigen::Vector4f &min_pt, Eigen::Vector4f &max_pt,
                     Indices &indices)
{
  indices.resize (cloud.size ());
  int l = 0;

  // If the data is dense, we don't need to check for NaN
  if (cloud.is_dense)
  {
    for (std::size_t i = 0; i < cloud.size (); ++i)
    {
      // Check if the point is inside bounds
      if (cloud[i].x < min_pt[0] || cloud[i].y < min_pt[1] || cloud[i].z < min_pt[2])
        continue;
      if (cloud[i].x > max_pt[0] || cloud[i].y > max_pt[1] || cloud[i].z > max_pt[2])
        continue;
      indices[l++] = int (i);
    }
  }
  // NaN or Inf values could exist => check for them
  else
  {
    for (std::size_t i = 0; i < cloud.size (); ++i)
    {
      // Check if the point is invalid
      if (!std::isfinite (cloud[i].x) || 
          !std::isfinite (cloud[i].y) || 
          !std::isfinite (cloud[i].z))
        continue;
      // Check if the point is inside bounds
      if (cloud[i].x < min_pt[0] || cloud[i].y < min_pt[1] || cloud[i].z < min_pt[2])
        continue;
      if (cloud[i].x > max_pt[0] || cloud[i].y > max_pt[1] || cloud[i].z > max_pt[2])
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
    for (std::size_t i = 0; i < cloud.size (); ++i)
    {
      pcl::Vector3fMapConst pt = cloud[i].getVector3fMap ();
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
    for (std::size_t i = 0; i < cloud.size (); ++i)
    {
      // Check if the point is invalid
      if (!std::isfinite (cloud[i].x) || !std::isfinite (cloud[i].y) || !std::isfinite (cloud[i].z))
        continue;
      pcl::Vector3fMapConst pt = cloud[i].getVector3fMap ();
      dist = (pivot_pt3 - pt).norm ();
      if (dist > max_dist)
      {
        max_idx = int (i);
        max_dist = dist;
      }
    }
  }

  if(max_idx != -1)
    max_pt = cloud[max_idx].getVector4fMap ();
  else
    max_pt = Eigen::Vector4f(std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::quiet_NaN());
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT> inline void
pcl::getMaxDistance (const pcl::PointCloud<PointT> &cloud, const Indices &indices,
                     const Eigen::Vector4f &pivot_pt, Eigen::Vector4f &max_pt)
{
  float max_dist = -FLT_MAX;
  int max_idx = -1;
  float dist;
  const Eigen::Vector3f pivot_pt3 = pivot_pt.head<3> ();

  // If the data is dense, we don't need to check for NaN
  if (cloud.is_dense)
  {
    for (std::size_t i = 0; i < indices.size (); ++i)
    {
      pcl::Vector3fMapConst pt = cloud[indices[i]].getVector3fMap ();
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
    for (std::size_t i = 0; i < indices.size (); ++i)
    {
      // Check if the point is invalid
      if (!std::isfinite (cloud[indices[i]].x) || !std::isfinite (cloud[indices[i]].y)
          ||
          !std::isfinite (cloud[indices[i]].z))
        continue;

      pcl::Vector3fMapConst pt = cloud[indices[i]].getVector3fMap ();
      dist = (pivot_pt3 - pt).norm ();
      if (dist > max_dist)
      {
        max_idx = static_cast<int> (i);
        max_dist = dist;
      }
    }
  }

  if(max_idx != -1)
    max_pt = cloud[indices[max_idx]].getVector4fMap ();
  else
    max_pt = Eigen::Vector4f(std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::quiet_NaN());
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline void
pcl::getMinMax3D (const pcl::PointCloud<PointT> &cloud, PointT &min_pt, PointT &max_pt)
{
  Eigen::Vector4f min_p, max_p;
  pcl::getMinMax3D (cloud, min_p, max_p);
  min_pt.x = min_p[0]; min_pt.y = min_p[1]; min_pt.z = min_p[2];
  max_pt.x = max_p[0]; max_pt.y = max_p[1]; max_pt.z = max_p[2];
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline void
pcl::getMinMax3D (const pcl::PointCloud<PointT> &cloud, Eigen::Vector4f &min_pt, Eigen::Vector4f &max_pt)
{
  min_pt.setConstant (FLT_MAX);
  max_pt.setConstant (-FLT_MAX);

  // If the data is dense, we don't need to check for NaN
  if (cloud.is_dense)
  {
    for (const auto& point: cloud.points)
    {
      const pcl::Vector4fMapConst pt = point.getVector4fMap ();
      min_pt = min_pt.cwiseMin (pt);
      max_pt = max_pt.cwiseMax (pt);
    }
  }
  // NaN or Inf values could exist => check for them
  else
  {
    for (const auto& point: cloud.points)
    {
      // Check if the point is invalid
      if (!std::isfinite (point.x) ||
          !std::isfinite (point.y) ||
          !std::isfinite (point.z))
        continue;
      const pcl::Vector4fMapConst pt = point.getVector4fMap ();
      min_pt = min_pt.cwiseMin (pt);
      max_pt = max_pt.cwiseMax (pt);
    }
  }
}


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline void
pcl::getMinMax3D (const pcl::PointCloud<PointT> &cloud, const pcl::PointIndices &indices,
                  Eigen::Vector4f &min_pt, Eigen::Vector4f &max_pt)
{
  pcl::getMinMax3D (cloud, indices.indices, min_pt, max_pt);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline void
pcl::getMinMax3D (const pcl::PointCloud<PointT> &cloud, const Indices &indices,
                  Eigen::Vector4f &min_pt, Eigen::Vector4f &max_pt)
{
  min_pt.setConstant (FLT_MAX);
  max_pt.setConstant (-FLT_MAX);

  // If the data is dense, we don't need to check for NaN
  if (cloud.is_dense)
  {
    for (const auto &index : indices)
    {
      const pcl::Vector4fMapConst pt = cloud[index].getVector4fMap ();
      min_pt = min_pt.cwiseMin (pt);
      max_pt = max_pt.cwiseMax (pt);
    }
  }
  // NaN or Inf values could exist => check for them
  else
  {
    for (const auto &index : indices)
    {
      // Check if the point is invalid
      if (!std::isfinite (cloud[index].x) || 
          !std::isfinite (cloud[index].y) || 
          !std::isfinite (cloud[index].z))
        continue;
      const pcl::Vector4fMapConst pt = cloud[index].getVector4fMap ();
      min_pt = min_pt.cwiseMin (pt);
      max_pt = max_pt.cwiseMax (pt);
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
  Eigen::Vector3f va,vb,res;

  res(0) = res(1) = res(2) = 0.0f;
  for (int i = 0; i < num_points; ++i) 
  {
    int j = (i + 1) % num_points;
    va = polygon[i].getVector3fMap ();
    vb = polygon[j].getVector3fMap ();
    res += va.cross (vb);
  }
  area = res.norm ();
  return (area*0.5);
}

#endif  //#ifndef PCL_COMMON_IMPL_H_

