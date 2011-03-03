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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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

#include <cmath>
#include "pcl/win32_macros.h"

namespace pcl
{

inline float L1_Norm (float *a, float *b, int dim)
{
  float norm = 0.0f;
  for (int i = 0; i < dim; ++i)
    norm += fabsf(*(a++) - *(b++));
  return norm;
}


inline float L2_Norm_SQR (float *a, float *b, int dim)
{
  float norm = 0.0;
  for (int i = 0; i < dim; ++i)
  {
    float diff  =  *(a++) - *(b++);
    norm += diff*diff;
  }
  return norm;
}


inline float L2_Norm (float *a, float *b, int dim)
{
  return sqrtf(L2_Norm_SQR(a, b, dim));
}


inline float Linf_Norm (float *a, float *b, int dim)
{
  float norm = 0.0;
  for (int i = 0; i < dim; ++i)
    norm = (std::max)(fabsf(*(a++) - *(b++)), norm);
  return norm;
}


inline float JM_Norm (float *a, float *b, int dim)
{
  float norm = 0.0;

  for (int i = 0; i < dim; ++i)
    norm += (sqrtf (a[i]) - sqrtf (b[i])) * (sqrtf (a[i]) - sqrtf (b[i]));

  return sqrtf (norm);
}


inline float B_Norm (float *a, float *b, int dim)
{
  float norm = 0.0, result;

  for (int i = 0; i < dim; ++i)
    norm += sqrtf (a[i] * b[i]);

  if (norm > 0)
    result = -log (norm);
  else
    result = 0;

  return result;
}


inline float Sublinear_Norm (float *a, float *b, int dim)
{
  float norm = 0.0;

  for (int i = 0; i < dim; ++i)
    norm += sqrtf (fabsf (a[i] - b[i]));

  return norm;
}


inline float CS_Norm (float *a, float *b, int dim)
{
  float norm = 0.0;

  for (int i = 0; i < dim; ++i)
    if ((a[i] + b[i]) != 0)
      norm += (a[i] - b[i]) * (a[i] - b[i]) / (a[i] + b[i]);
    else
      norm += 0;
  return norm;
}


inline float Div_Norm (float *a, float *b, int dim)
{
  float norm = 0.0;

  for (int i = 0; i < dim; ++i)
    if ((a[i] / b[i]) > 0)
      norm += (a[i] - b[i]) * log (a[i] / b[i]);
    else
      norm += 0;
  return norm;
}


inline float PF_Norm (float *a, float *b, int dim, float P1, float P2)
{
  float norm = 0.0;

  for (int i = 0; i < dim; ++i)
    norm += (P1 * a[i] - P2 * b[i]) * (P1 * a[i] - P2 * b[i]);
  return sqrtf (norm);
}


inline float K_Norm (float *a, float *b, int dim, float P1, float P2)
{
  float norm = 0.0;

  for (int i = 0; i < dim; ++i)
    norm += fabsf (P1 * a[i] - P2 * b[i]);
  return norm;
}


inline float KL_Norm (float *a, float *b, int dim)
{
  float norm = 0.0;

  for (int i = 0; i < dim; ++i)
    if ( (b[i] != 0) && ((a[i] / b[i]) > 0) )
      norm += a[i] * log (a[i] / b[i]);
    else
      norm += 0;
  return norm;
}

inline float HIK_Norm(float *a, float *b, int dim)
{
  float norm = 0.0f;
  for (int i = 0; i < dim; ++i)
    norm += (std::min)(*(a++), *(b++));
  return norm;
}

}
