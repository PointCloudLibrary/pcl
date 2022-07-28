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

/// @file   transformCommand.cpp
/// @details the implementation of the class TransformCommand.
/// @author  Yue Li and Matthew Hielsberg

#include <algorithm>
#include <pcl/apps/point_cloud_editor/transformCommand.h>
#include <pcl/apps/point_cloud_editor/selection.h>
#include <pcl/apps/point_cloud_editor/common.h>

TransformCommand::TransformCommand(const ConstSelectionPtr& selection_ptr,
                                   CloudPtr cloud_ptr,
                                   const float *matrix,
                                   float translate_x,
                                   float translate_y,
                                   float translate_z)
  : selection_ptr_(selection_ptr), cloud_ptr_(std::move(cloud_ptr)),
    translate_x_(translate_x), translate_y_(translate_y),
    translate_z_(translate_z)
{
  internal_selection_ptr_ = SelectionPtr(new Selection(*selection_ptr));
  std::copy(matrix, matrix + MATRIX_SIZE, transform_matrix_);
  const float *cloud_matrix = cloud_ptr_->getMatrix();
  std::copy(cloud_matrix, cloud_matrix + MATRIX_SIZE, cloud_matrix_);
  invertMatrix(cloud_matrix, cloud_matrix_inv_);
  cloud_ptr_->getCenter(cloud_center_[X], cloud_center_[Y], cloud_center_[Z]);
}

void
TransformCommand::execute()
{
  if (!cloud_ptr_)
    return;
  applyTransform(selection_ptr_);
}


void
TransformCommand::undo()
{ 
  if (!cloud_ptr_)
    return;
  float transform_matrix_inv[MATRIX_SIZE];
  invertMatrix(transform_matrix_, transform_matrix_inv);
  for(const unsigned int &index : *internal_selection_ptr_)
  {
    Point3D pt;
    pt.x = (*cloud_ptr_)[index].x - cloud_center_[X];
    pt.y = (*cloud_ptr_)[index].y - cloud_center_[Y];
    pt.z = (*cloud_ptr_)[index].z - cloud_center_[Z];

    float x,y,z;
    x = pt.x * cloud_matrix_[0] +
        pt.y * cloud_matrix_[4] +
        pt.z * cloud_matrix_[8] + cloud_matrix_[12];
    y = pt.x * cloud_matrix_[1] +
        pt.y * cloud_matrix_[5] +
        pt.z * cloud_matrix_[9] + cloud_matrix_[13];
    z = pt.x * cloud_matrix_[2] +
        pt.y * cloud_matrix_[6] +
        pt.z * cloud_matrix_[10] + cloud_matrix_[14];

    pt.x = x - translate_x_;
    pt.y = y - translate_y_;
    pt.z = z - translate_z_;

    x = pt.x * transform_matrix_inv[0] +
        pt.y * transform_matrix_inv[4] +
        pt.z * transform_matrix_inv[8] + transform_matrix_inv[12];
    y = pt.x * transform_matrix_inv[1] +
        pt.y * transform_matrix_inv[5] +
        pt.z * transform_matrix_inv[9] + transform_matrix_inv[13];
    z = pt.x * transform_matrix_inv[2] +
        pt.y * transform_matrix_inv[6] +
        pt.z * transform_matrix_inv[10] + transform_matrix_inv[14];

    pt.x = x * cloud_matrix_inv_[0] +
           y * cloud_matrix_inv_[4] +
           z * cloud_matrix_inv_[8] + cloud_matrix_inv_[12];
    pt.y = x * cloud_matrix_inv_[1] +
           y * cloud_matrix_inv_[5] +
           z * cloud_matrix_inv_[9] + cloud_matrix_inv_[13];
    pt.z = x * cloud_matrix_inv_[2] +
           y * cloud_matrix_inv_[6] +
           z * cloud_matrix_inv_[10] + cloud_matrix_inv_[14];

    (*cloud_ptr_)[index].x = pt.x + cloud_center_[X];
    (*cloud_ptr_)[index].y = pt.y + cloud_center_[Y];
    (*cloud_ptr_)[index].z = pt.z + cloud_center_[Z];
  }
}

void
TransformCommand::applyTransform(const ConstSelectionPtr& sel_ptr)
{
  // now modify the selected points' coordinates
  for(const unsigned int &index : *sel_ptr)
  {
    Point3D pt = cloud_ptr_->getObjectSpacePoint(index);

    float x,y,z;
    x = pt.x * transform_matrix_[0] +
        pt.y * transform_matrix_[4] +
        pt.z * transform_matrix_[8] + transform_matrix_[12];
    y = pt.x * transform_matrix_[1] +
        pt.y * transform_matrix_[5] +
        pt.z * transform_matrix_[9] + transform_matrix_[13];
    z = pt.x * transform_matrix_[2] +
        pt.y * transform_matrix_[6] +
        pt.z * transform_matrix_[10] + transform_matrix_[14];

    pt.x = x + translate_x_;
    pt.y = y + translate_y_;
    pt.z = z + translate_z_;

    x = pt.x * cloud_matrix_inv_[0] +
        pt.y * cloud_matrix_inv_[4] +
        pt.z * cloud_matrix_inv_[8] + cloud_matrix_inv_[12];
    y = pt.x * cloud_matrix_inv_[1] +
        pt.y * cloud_matrix_inv_[5] +
        pt.z * cloud_matrix_inv_[9] + cloud_matrix_inv_[13];
    z = pt.x * cloud_matrix_inv_[2] +
        pt.y * cloud_matrix_inv_[6] +
        pt.z * cloud_matrix_inv_[10] + cloud_matrix_inv_[14];

    (*cloud_ptr_)[index].x = x + cloud_center_[X];
    (*cloud_ptr_)[index].y = y + cloud_center_[Y];
    (*cloud_ptr_)[index].z = z + cloud_center_[Z];
  }
}










