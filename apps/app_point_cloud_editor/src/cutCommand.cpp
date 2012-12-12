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

///
/// @file   cutCommand.cpp
/// @details the implementation of the class CutCommand
/// @author  Yue Li and Matthew Hielsberg

#include <pcl/apps/point_cloud_editor/cutCommand.h>
#include <pcl/apps/point_cloud_editor/copyBuffer.h>
#include <pcl/apps/point_cloud_editor/selection.h>

CutCommand::CutCommand (CopyBufferPtr copy_buffer_ptr,
                        SelectionPtr selection_ptr,
                        CloudPtr cloud_ptr)
  : selection_ptr_(selection_ptr), cloud_ptr_(cloud_ptr),
    copy_buffer_ptr_(copy_buffer_ptr), cut_selection_(cloud_ptr)
{
}

CutCommand::~CutCommand ()
{
}

void
CutCommand::execute ()
{
  if (!cloud_ptr_)
    return;
  if (selection_ptr_->empty())
    return;

  // do the copy
  copy_buffer_ptr_ -> set(cloud_ptr_, *selection_ptr_);

  // back up copied points for undo
  cut_cloud_buffer_ = *copy_buffer_ptr_;
  cut_selection_ = *selection_ptr_;

   // remove the copied points from the cloud.
  cloud_ptr_ -> remove(cut_selection_);

  // The selection points to the incorrect points or may have indices out of
  // bounds, so we must clear it.
  selection_ptr_ -> clear();

  // notify the cloud that the selection has changed
  cloud_ptr_ -> setSelection(selection_ptr_);
}

void
CutCommand::undo()
{
  if (!cloud_ptr_)
    return;
  cloud_ptr_ -> restore(cut_cloud_buffer_, cut_selection_);
}
