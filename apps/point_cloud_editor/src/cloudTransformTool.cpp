///
/// Copyright (c) 2012, Texas A&M University
/// All rights reserved.
///
/// Redistribution and use in source and binary forms, with or without
/// modification, are permitted provided that the following conditions
/// are met:
///
///  * Redistributions of source code must retain the above copyright
///    notice, this list of conditions and the following disclaimer.
///  * Redistributions in binary form must reproduce the above
///    copyright notice, this list of conditions and the following
///    disclaimer in the documentation and/or other materials provided
///    with the distribution.
///  * Neither the name of Texas A&M University nor the names of its
///    contributors may be used to endorse or promote products derived
///    from this software without specific prior written permission.
///
/// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
/// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
/// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
/// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
/// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
/// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
/// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
/// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
/// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
/// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
/// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
/// POSSIBILITY OF SUCH DAMAGE.
///
/// The following software was written as part of a collaboration with the
/// University of South Carolina, Interdisciplinary Mathematics Institute.
///

/// @file cloudTransformTool.cpp
/// @details the implementation of class CloudTransformTool
/// @author Yue Li and Matthew Hielsberg

#include <algorithm>
#include <math.h>
#include <pcl/apps/point_cloud_editor/common.h>
#include <pcl/apps/point_cloud_editor/cloudTransformTool.h>
#include <pcl/apps/point_cloud_editor/cloud.h>

const float DEG_2_RADS = M_PI / 180.0f;

const float CloudTransformTool::DEFAULT_SCALE_FACTOR_ = 1.14;
const float CloudTransformTool::DEFAULT_TRANSLATE_FACTOR_ = 0.001f;


CloudTransformTool::CloudTransformTool (CloudPtr cloud_ptr)
  : cloud_ptr_(cloud_ptr), x_(0), y_(0), scale_factor_(DEFAULT_SCALE_FACTOR_),
    translate_factor_(DEFAULT_TRANSLATE_FACTOR_)
{
  setIdentity(transform_matrix_);
}

CloudTransformTool::~CloudTransformTool ()
{
}

void
CloudTransformTool::start (int x, int y, BitMask, BitMask)
{
  x_ = x;
  y_ = y;
    
  trackball_.start(x, y);
}

void
CloudTransformTool::update (int x, int y, BitMask modifiers, BitMask buttons)
{
  if (!cloud_ptr_)
    return;
  if (!(buttons & LEFT))
    return;
    
  float transform[MATRIX_SIZE];
    
  int dx = (x - x_);
  int dy = (y - y_);
  if (dx == 0 && dy == 0)
    return;
  trackball_.update(x, y);
  if (modifiers & CTRL)
    getTranslateMatrix(dx, dy, transform);
  else if (modifiers & ALT)
    getZTranslateMatrix(dy, transform);
  else if (modifiers & SHFT)
    getScaleMatrix(dy, transform);
  else
    trackball_.getRotationMatrix(transform);

  cloud_ptr_ -> multMatrix(transform);

  x_ = x;
  y_ = y;
}

void
CloudTransformTool::getTranslateMatrix (int dx, int dy, float* matrix)
{
  setIdentity(matrix);
  float scale = 1.0f / cloud_ptr_-> getScalingFactor();
  matrix[12] = float(dx) * translate_factor_ * scale;
  matrix[13] = float(-dy) * translate_factor_ * scale;
}

void
CloudTransformTool::getZTranslateMatrix (int dy, float* matrix)
{
  setIdentity(matrix);
  matrix[14] = float(dy) * translate_factor_ / cloud_ptr_-> getScalingFactor();
}

void
CloudTransformTool::getScaleMatrix (int dy, float* matrix)
{
  setIdentity(matrix);
  float scale = dy > 0 ? scale_factor_ : 1.0 / scale_factor_;
  for (unsigned int i = 0; i < MATRIX_SIZE-1; i+=MATRIX_SIZE_DIM+1)
    matrix[i] = scale;
}

