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

/// @file copyCommand.h
/// @details A CopyCommand object provides functionality for filling the copy
/// buffer with the current selection.  The
/// @author Yue Li and Matthew Hielsberg

#pragma once

#include <pcl/apps/point_cloud_editor/command.h>
#include <pcl/apps/point_cloud_editor/localTypes.h>
#include <pcl/apps/point_cloud_editor/copyBuffer.h>

#include <pcl/memory.h>  // for pcl::shared_ptr

class Selection;

class CopyCommand : public Command
{
  public:
    /// The type for shared pointer pointing to a constant selection buffer
    using ConstSelectionPtr = pcl::shared_ptr<const Selection>;

    /// @brief Constructor
    /// @param copy_buffer_ptr a shared pointer pointing to the copy buffer.
    /// @param selection_ptr a shared pointer pointing to the selection object.
    /// @param cloud_ptr a shared pointer pointing to the cloud object.
    CopyCommand (CopyBufferPtr copy_buffer_ptr,
                 ConstSelectionPtr selection_ptr,
                 ConstCloudPtr cloud_ptr)
      : copy_buffer_ptr_(std::move(copy_buffer_ptr)), selection_ptr_(std::move(selection_ptr)),
        cloud_ptr_(std::move(cloud_ptr))
    {
      has_undo_ = false;
    }

    /// @brief Copy constructor - commands are non-copyable
    CopyCommand (const CopyCommand&) = delete;

    /// @brief Equal operator - commands are non-copyable
    CopyCommand&
    operator= (const CopyCommand&) = delete;

  protected:
    /// @brief Copy the selected points into the copy buffer.
    /// @pre Assumes the constructor was given appropriate pointers to the
    /// required objects.
    void
    execute () override
    {
      if (!cloud_ptr_)
        return;
      copy_buffer_ptr_ -> set(cloud_ptr_, *selection_ptr_);
    }

    /// @brief undo is not supported for this command.
    void
    undo () override
    {
      assert(false);
    }

  private:
    /// a pointer to the copy buffer.
    CopyBufferPtr copy_buffer_ptr_;

    /// a shared pointer pointing to the selection
    ConstSelectionPtr selection_ptr_;

    /// a shared pointer pointing to the cloud
    ConstCloudPtr cloud_ptr_;
};
