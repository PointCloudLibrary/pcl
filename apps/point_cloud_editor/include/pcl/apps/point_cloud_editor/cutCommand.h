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

/// @file   cutCommand.h
/// @details A CutCommand object provides functionality for removing selected
/// points from the cloud and filling the copy buffer.
/// @author  Yue Li and Matthew Hielsberg

#pragma once

#include <pcl/apps/point_cloud_editor/command.h>
#include <pcl/apps/point_cloud_editor/localTypes.h>
#include <pcl/apps/point_cloud_editor/copyBuffer.h>
#include <pcl/apps/point_cloud_editor/selection.h>

#include <pcl/memory.h>  // for pcl::shared_ptr

class CutCommand : public Command
{
  public:
    /// The type for shared pointer pointing to a selection buffer
    using SelectionPtr = pcl::shared_ptr<Selection>;

    /// @brief Constructor
    /// @param copy_buffer_ptr a shared pointer pointing to the copy buffer.
    /// @param selection_ptr a shared pointer pointing to the selection object.
    /// @param cloud_ptr a shared pointer pointing to the cloud object.
    CutCommand (CopyBufferPtr copy_buffer_ptr,
                SelectionPtr selection_ptr,
                const CloudPtr& cloud_ptr);

    /// @brief Copy constructor - commands are non-copyable
    CutCommand (const CutCommand&) = delete;

    /// @brief Equal operator - commands are non-copyable
    CutCommand&
    operator= (const CutCommand&) = delete;

    /// @brief Destructor
    ~CutCommand () override;

  protected:
    /// @brief Moves the selected points to the copy buffer and removes them
    /// from the cloud.
    /// @pre Assumes the constructor was given appropriate pointers to the
    /// required objects.
    void
    execute () override;

    /// @brief Returns the cut points to the cloud.  This does not reconstruct
    /// the original ordering of the point cloud.
    void
    undo () override;

  private:
    /// A shared pointer pointing to the selection object.
    SelectionPtr selection_ptr_;

    /// a pointer pointing to the cloud
    CloudPtr cloud_ptr_;

    /// a pointer pointing to the copy buffer.
    CopyBufferPtr copy_buffer_ptr_;

    /// a selection which backs up the index of the points cut in the
    /// original cloud.
    Selection cut_selection_;

    /// The copy buffer which backs up the points removed from the cloud.
    CopyBuffer cut_cloud_buffer_;
};
