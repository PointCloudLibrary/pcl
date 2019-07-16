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
/// @file   deleteCommand.cpp
/// @details implementation of the class DeleteCommand
/// @author  Yue Li and Matthew Hielsberg

#include <pcl/apps/point_cloud_editor/deleteCommand.h>
#include <pcl/apps/point_cloud_editor/copyBuffer.h>
#include <pcl/apps/point_cloud_editor/selection.h>

DeleteCommand::DeleteCommand (SelectionPtr selection_ptr,
                              const CloudPtr& cloud_ptr)
  : cloud_ptr_(cloud_ptr), selection_ptr_(std::move(selection_ptr)), deleted_selection_(cloud_ptr) {}

void
DeleteCommand::execute ()
{
  if (!cloud_ptr_)
    return;
  if (selection_ptr_->empty())
    return;

  // back up the points to be deleted
  deleted_selection_ = *selection_ptr_;
  deleted_cloud_buffer_.set(cloud_ptr_, deleted_selection_);

  // delete the points
  cloud_ptr_->remove(deleted_selection_);

  // The selection points to the incorrect points or may have indices out of
  // bounds, so we must clear it.
  selection_ptr_->clear();

  // notify the cloud that the selection has changed
  cloud_ptr_ -> setSelection(selection_ptr_);
}

void
DeleteCommand::undo ()
{
  if (!cloud_ptr_)
    return;
  cloud_ptr_->restore(deleted_cloud_buffer_, deleted_selection_);
}













