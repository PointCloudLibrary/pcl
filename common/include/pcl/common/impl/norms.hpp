/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
 *
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
 *  FOR a PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
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

#pragma once

#include <pcl/common/norms.h>
#include <pcl/console/print.h>
#include <pcl/pcl_macros.h>


namespace pcl
{

template <typename FloatVectorT> inline float
selectNorm (FloatVectorT a, FloatVectorT b, int dim, NormType norm_type)
{
  // {L1, L2_SQR, L2, LINF, JM, B, SUBLINEAR, CS, DIV, PF, K, KL, HIK};
  switch (norm_type)
  {
    case (L1):
        return L1_Norm (a, b, dim);
    case (L2_SQR):
        return L2_Norm_SQR (a, b, dim);
    case (L2):
        return L2_Norm  (a, b, dim);
    case (LINF):
        return Linf_Norm (a, b, dim);
    case (JM):
        return JM_Norm  (a, b, dim);
    case (B):
        return B_Norm  (a, b, dim);
    case (SUBLINEAR):
        return Sublinear_Norm (a, b, dim);
    case (CS):
        return CS_Norm (a, b, dim);
    case (DIV):
        return Div_Norm (a, b, dim);
    case (KL):
        return KL_Norm (a, b, dim);
    case (HIK):
        return HIK_Norm (a, b, dim);

    case (PF):
    case (K):
    default:
      PCL_ERROR ("[pcl::selectNorm] For PF and K norms you have to explicitly call the method, as they need additional parameters\n");
      return -1;
  }
}


template <typename FloatVectorT> inline float
L1_Norm (FloatVectorT a, FloatVectorT b, int dim)
{
  float norm = 0.0f;
  for (int i = 0; i < dim; ++i)
    norm += std::abs(a[i] - b[i]);
  return norm;
}


template <typename FloatVectorT> inline float
L2_Norm_SQR (FloatVectorT a, FloatVectorT b, int dim)
{
  float norm = 0.0;
  for (int i = 0; i < dim; ++i)
  {
    float diff  =  a[i] - b[i];
    norm += diff*diff;
  }
  return norm;
}


template <typename FloatVectorT> inline float
L2_Norm (FloatVectorT a, FloatVectorT b, int dim)
{
  return std::sqrt (L2_Norm_SQR(a, b, dim));
}


template <typename FloatVectorT> inline float
Linf_Norm (FloatVectorT a, FloatVectorT b, int dim)
{
  float norm = 0.0;
  for (int i = 0; i < dim; ++i)
    norm = (std::max)(std::abs(a[i] - b[i]), norm);
  return norm;
}


template <typename FloatVectorT> inline float
JM_Norm (FloatVectorT a, FloatVectorT b, int dim)
{
  float norm = 0.0;

  for (int i = 0; i < dim; ++i)
    norm += (std::sqrt (a[i]) - std::sqrt (b[i])) * (std::sqrt (a[i]) - std::sqrt (b[i]));

  return std::sqrt (norm);
}


template <typename FloatVectorT> inline float
B_Norm (FloatVectorT a, FloatVectorT b, int dim)
{
  float norm = 0.0, result;

  for (int i = 0; i < dim; ++i)
    norm += std::sqrt (a[i] * b[i]);

  if (norm > 0)
    result = -std::log (norm);
  else
    result = 0;

  return result;
}


template <typename FloatVectorT> inline float
Sublinear_Norm (FloatVectorT a, FloatVectorT b, int dim)
{
  float norm = 0.0;

  for (int i = 0; i < dim; ++i)
    norm += std::sqrt (std::abs (a[i] - b[i]));

  return norm;
}


template <typename FloatVectorT> inline float
CS_Norm (FloatVectorT a, FloatVectorT b, int dim)
{
  float norm = 0.0;

  for (int i = 0; i < dim; ++i)
    if ((a[i] + b[i]) != 0)
      norm += (a[i] - b[i]) * (a[i] - b[i]) / (a[i] + b[i]);
    else
      norm += 0;
  return norm;
}


template <typename FloatVectorT> inline float
Div_Norm (FloatVectorT a, FloatVectorT b, int dim)
{
  float norm = 0.0;

  for (int i = 0; i < dim; ++i)
    if ((a[i] / b[i]) > 0)
      norm += (a[i] - b[i]) * std::log (a[i] / b[i]);
    else
      norm += 0;
  return norm;
}


template <typename FloatVectorT> inline float
PF_Norm (FloatVectorT a, FloatVectorT b, int dim, float P1, float P2)
{
  float norm = 0.0;

  for (int i = 0; i < dim; ++i)
    norm += (P1 * a[i] - P2 * b[i]) * (P1 * a[i] - P2 * b[i]);
  return std::sqrt (norm);
}


template <typename FloatVectorT> inline float
K_Norm (FloatVectorT a, FloatVectorT b, int dim, float P1, float P2)
{
  float norm = 0.0;

  for (int i = 0; i < dim; ++i)
    norm += std::abs (P1 * a[i] - P2 * b[i]);
  return norm;
}


template <typename FloatVectorT> inline float
KL_Norm (FloatVectorT a, FloatVectorT b, int dim)
{
  float norm = 0.0;

  for (int i = 0; i < dim; ++i)
    if ( (b[i] != 0) && ((a[i] / b[i]) > 0) )
      norm += a[i] * std::log (a[i] / b[i]);
    else
      norm += 0;
  return norm;
}


template <typename FloatVectorT> inline float
HIK_Norm(FloatVectorT a, FloatVectorT b, int dim)
{
  float norm = 0.0f;
  for (int i = 0; i < dim; ++i)
    norm += (std::min)(a[i], b[i]);
  return norm;
}

} // namespace pcl

