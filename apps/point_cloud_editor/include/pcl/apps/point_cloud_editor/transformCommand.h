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

/// @file   transformCommand.h
/// @details a TransformCommand object provides transformation and undo
/// functionalities.  // XXX - transformation of what?
/// @author  Yue Li and Matthew Hielsberg

#pragma once

#include <pcl/apps/point_cloud_editor/command.h>
#include <pcl/apps/point_cloud_editor/localTypes.h>
#include <pcl/apps/point_cloud_editor/cloud.h>

#include <pcl/memory.h>  // for pcl::shared_ptr

class Selection;

class TransformCommand : public Command
{
  public:
    /// The type for shared pointer pointing to a constant selection buffer
    using ConstSelectionPtr = pcl::shared_ptr<const Selection>;

    /// @brief Constructor
    /// @param selection_ptr a shared pointer pointing to the selection object.
    /// @param cloud_ptr a shared pointer pointing to the cloud object.
    /// @param matrix a (4x4) transform matrix following OpenGL's format.
    /// @pre Assumes the selection_ptr is valid, non-NULL.
    TransformCommand (const ConstSelectionPtr& selection_ptr, CloudPtr cloud_ptr,
                      const float* matrix, float translate_x,
                      float translate_y, float translate_z);

    /// @brief Copy constructor - object is not copy-constructable
    TransformCommand (const TransformCommand&) = delete;

    /// @brief Equal operator - object is non-copyable
    TransformCommand&
    operator= (const TransformCommand&) = delete;

  protected:
    // Transforms the coordinates of the selected points according to the transform
    // matrix.
    void
    execute () override;

    // Restore the coordinates of the transformed points.
    void
    undo () override;

  private:
    /// @brief Applies the transformation to the point values
    /// @param sel_ptr A pointer to the selection object whose points are to be
    /// transformed.
    void
    applyTransform(const ConstSelectionPtr& sel_ptr);

    /// pointers to constructor params
    ConstSelectionPtr selection_ptr_;

    /// a pointer pointing to the cloud
    CloudPtr cloud_ptr_;

    float translate_x_, translate_y_, translate_z_;

    /// An internal selection object used to perform undo
    SelectionPtr internal_selection_ptr_;

    /// the transform matrix to be used to compute the new coordinates
    /// of the selected points
    float transform_matrix_[MATRIX_SIZE];

    /// The transform matrix of the cloud used by this command
    float cloud_matrix_[MATRIX_SIZE];
    /// The inverted transform matrix of the cloud used by this command
    float cloud_matrix_inv_[MATRIX_SIZE];

    /// The center of the cloud used by this command
    float cloud_center_[XYZ_SIZE];
};
