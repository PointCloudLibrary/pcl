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

/// @file selectionTransformTool.cpp
/// @details the implementation of class CloudTransformTool
/// @author Yue Li and Matthew Hielsberg

#include <pcl/apps/point_cloud_editor/selectionTransformTool.h>
#include <pcl/apps/point_cloud_editor/cloud.h>
#include <pcl/apps/point_cloud_editor/selection.h>
#include <pcl/apps/point_cloud_editor/transformCommand.h>
#include <pcl/apps/point_cloud_editor/commandQueue.h>
#include <pcl/apps/point_cloud_editor/common.h>

const float SelectionTransformTool::DEFAULT_TRANSLATE_FACTOR_ = 0.001;

SelectionTransformTool::SelectionTransformTool (ConstSelectionPtr selection_ptr,
                                                CloudPtr cloud_ptr,
                                                CommandQueuePtr command_queue_ptr)
  : selection_ptr_(std::move(selection_ptr)),
    cloud_ptr_(std::move(cloud_ptr)),
    command_queue_ptr_(std::move(command_queue_ptr)),
    translate_factor_(DEFAULT_TRANSLATE_FACTOR_)
{
  std::fill_n(center_xyz_, XYZ_SIZE, 0);
  setIdentity(transform_matrix_);
}

void
SelectionTransformTool::start (int x, int y, BitMask modifiers, BitMask buttons)
{
  if ((!cloud_ptr_) || (!selection_ptr_) || selection_ptr_->empty())
    return;
  if (!(buttons & LEFT))
    return;
  modifiers_ = modifiers;
  x_ = x;
  y_ = y;
  findSelectionCenter();
  setIdentity(transform_matrix_);
  trackball_.start(x, y);
}

void
SelectionTransformTool::update (int x, int y, BitMask, BitMask buttons)
{
  if (!cloud_ptr_)
    return;
  if (!(buttons & LEFT))
    return;
  int dx = (x - x_);
  int dy = (y - y_);
  if (dx == 0 && dy == 0)
    return;
  trackball_.update(x, y);

  if (modifiers_ & CTRL)
  {
    // selection motion is not applied directly (waits for end)
    // as such we can not update x and y immediately
    float scale = 1.0f / cloud_ptr_->getScalingFactor();
    cloud_ptr_->setSelectionTranslation ((float) dx * translate_factor_ * scale,
                                         (float) -dy * translate_factor_ * scale,
                                         0.0f);
    return;
  }
  if (modifiers_ & ALT)
  {
    // selection motion is not applied directly (waits for end)
    // as such we can not update x and y immediately
    float scale = 1.0f / cloud_ptr_->getScalingFactor();
    cloud_ptr_->setSelectionTranslation (0.0f,
                                         0.0f,
                                         (float) dy * translate_factor_ * scale);
    return;
  }
  float transform[MATRIX_SIZE];
  trackball_.getRotationMatrix(transform);
  transform_matrix_[12] -= center_xyz_[0];
  transform_matrix_[13] -= center_xyz_[1];
  transform_matrix_[14] -= center_xyz_[2];
  ::multMatrix(transform_matrix_, transform, transform_matrix_);
  transform_matrix_[12] += center_xyz_[0];
  transform_matrix_[13] += center_xyz_[1];
  transform_matrix_[14] += center_xyz_[2];
  cloud_ptr_ -> setSelectionRotation(transform_matrix_);

  x_ = x;
  y_ = y;
}

void
SelectionTransformTool::end (int x, int y, BitMask modifiers, BitMask buttons)
{
  if (!(buttons & LEFT))
    return;

  float scale = 1.0f / cloud_ptr_->getScalingFactor();
  int dx = (x - x_);
  int dy = (y - y_);
  update(x, y, modifiers, buttons);
  if (modifiers_ & CTRL)
  {
    std::shared_ptr<TransformCommand> c(new TransformCommand(selection_ptr_,
      cloud_ptr_, transform_matrix_, (float) dx * translate_factor_ * scale,
      (float) -dy * translate_factor_ * scale, 0.0f));
    command_queue_ptr_->execute(c);
  }
  else if (modifiers_ & ALT)
  {
    std::shared_ptr<TransformCommand> c(new TransformCommand(selection_ptr_,
      cloud_ptr_, transform_matrix_, 0.0f, 0.0f,
      (float) dy * translate_factor_ * scale));
    command_queue_ptr_->execute(c);
  }
  else
  {
    std::shared_ptr<TransformCommand> c(new TransformCommand(selection_ptr_,
      cloud_ptr_, transform_matrix_, 0.0f, 0.0f, 0.0f));
    command_queue_ptr_->execute(c);
  }
  setIdentity(transform_matrix_);
  cloud_ptr_->setSelectionRotation(transform_matrix_);
  cloud_ptr_->setSelectionTranslation(0.0f, 0.0f, 0.0f);
}

/*void
SelectionTransformTool::getRotateMatrix (int dx, int dy,
                                         float* rotation_matrix_a,
                                         float* rotation_matrix_b,
                                         BitMask buttons) const
{
  setIdentity(rotation_matrix_a);
  setIdentity(rotation_matrix_b);
  float rad_dx = (float) dx  * M_PI / 180.0f;
  float rad_dy = (float) dy  * M_PI / 180.0f;
  rotation_matrix_a[5]  = std::cos(rad_dy);
  rotation_matrix_a[6]  = std::sin(rad_dy);
  rotation_matrix_a[9]  = -std::sin(rad_dy);
  rotation_matrix_a[10] = std::cos(rad_dy);
  if (buttons & LEFT)
  {
    rotation_matrix_b[0]  = std::cos(rad_dx);
    rotation_matrix_b[2]  = -std::sin(rad_dx);
    rotation_matrix_b[8]  = std::sin(rad_dx);
    rotation_matrix_b[10] = std::cos(rad_dx);
  }
  else if (buttons & RIGHT)
  {
    rotation_matrix_b[0]  = std::cos(rad_dx);
    rotation_matrix_b[1]  = std::sin(rad_dx);
    rotation_matrix_b[4]  = -std::sin(rad_dx);
    rotation_matrix_b[5]  = std::cos(rad_dx);
  }
}*/

void
SelectionTransformTool::findSelectionCenter ()
{
  if (!selection_ptr_ || selection_ptr_->empty())
    return;
  float min_xyz[XYZ_SIZE] = {0.0f};
  float max_xyz[XYZ_SIZE] = {0.0f};
  Selection::const_iterator it = selection_ptr_->begin();
  Point3D point_3d = cloud_ptr_->getObjectSpacePoint (*it);
  float *pt = &(point_3d.data[X]);
  std::copy(pt, pt + XYZ_SIZE, max_xyz);
  std::copy(max_xyz, max_xyz + XYZ_SIZE, min_xyz);

  for (++it; it != selection_ptr_->end(); ++it)
  {
    Point3D point_3d = cloud_ptr_->getObjectSpacePoint (*it);
    pt = &(point_3d.data[X]);
    for (unsigned int j = 0; j < XYZ_SIZE; ++j)
    {
      min_xyz[j] = std::min(min_xyz[j], pt[j]);
      max_xyz[j] = std::max(max_xyz[j], pt[j]);
    }
  }
  for (unsigned int j = 0; j < XYZ_SIZE; ++j)
  {
    center_xyz_[j] = 0.5f * (max_xyz[j] + min_xyz[j]);
  }
}















