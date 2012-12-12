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

/// @file selectionTransformTool.h
/// @details This tool provides the ability to transform the current selection.
/// @author Yue Li and Matthew Hielsberg

#ifndef SELECTION_TRANSFORM_TOOL_H_
#define SELECTION_TRANSFORM_TOOL_H_

#include <pcl/apps/point_cloud_editor/toolInterface.h>
#include <pcl/apps/point_cloud_editor/localTypes.h>
#include <pcl/apps/point_cloud_editor/trackball.h>

/// @brief The selection transform tool computes the transform matrix from
/// mouse input.  It then updates the cloud's transform matrix for the
/// selected points so that the transformed and selected points will be
/// rendered appropriately. Note that, the actual coordinates of the selected
/// points are not updated until the end of the mouse input. At the end of a
/// mouse input (i.e. when the mouse button is released), a transform command is
/// created to update the actual coordinates of the selected points.
class SelectionTransformTool : public ToolInterface
{
  public:
    /// @brief Constructor
    /// @param selection_ptr a shared pointer pointing to the selection object.
    /// @param cloud_ptr a shared pointer pointing to the cloud object.
    /// @param command_queue_ptr a shared pointer pointing to the command queue.
    SelectionTransformTool (ConstSelectionPtr selection_ptr,
                            CloudPtr cloud_ptr,
                            CommandQueuePtr command_queue_ptr);

    /// @brief Destructor
    ~SelectionTransformTool ()
    {
    }

    /// @brief Initialize the transform tool with mouse screen coordinates
    /// and key modifiers.
    /// @param x the x value of the mouse screen coordinates.
    /// @param y the y value of the mouse screen coordinates.
    /// @param modifiers the key modifier.
    void
    start (int x, int y, BitMask modifiers, BitMask buttons);

    /// @brief Updates the transform matrix of this object with mouse screen
    /// coordinates and key modifiers. Then the selection_matrix_ in the cloud
    /// is further updated.
    /// @details We first compute the changes between the initial and the current
    /// mouse screen coordinates. Then depending on the passed modifiers, the
    /// transformation matrix is computed correspondingly. If CONTROL is pressed
    /// the selection will be translated (panned) parallel to the view plane. If
    /// ALT is pressed the selection witll be translated along the z-axis
    /// perpendicular to the view plane.  If no key modifiers is pressed the
    /// selection will be rotated.
    /// @param x the x value of the mouse screen coordinates.
    /// @param y the y value of the mouse screen coordinates.
    /// @param modifiers the key modifier. CONTROL pans the selection parallel
    /// to the view plane.  ALT moves the selection in/out along the z-axis
    /// (perpendicular to the view plane).  If no modifier is pressed then the
    /// selection is rotated.
    void
    update (int x, int y, BitMask modifiers, BitMask buttons);

    /// @brief Update the transform matrix for the selected points using the
    /// final position of the mouse. To finalize the transformation, we then
    /// create a transform command which computes the new coordinates of the
    /// selected points after transformation.
    /// @param x the x value of the mouse screen coordinates.
    /// @param y the y value of the mouse screen coordinates.
    /// @param modifiers the key modifier.
    void
    end (int x, int y, BitMask modifiers, BitMask buttons);

    /// @brief This does not do anything.
    void
    draw () const
    {
    }
    
  private:
    /// @brief Computes the modelview matrix for rotation.
    /// @param dx the distance between the x coordinates of the starting and
    /// ending cursor position.
    /// @param dy the distance between the y coordinates of the starting and
    /// ending cursor position.
    /// @param rotation_matrix_a a 4x4 matrix following OpenGL's format
    /// implementing rotation along x-axis.
    /// @param rotation_matrix_b a 4x4 matrix following OpenGL's format
    /// implementing rotation along y or z-axis, which depens on which the mouse
    /// button that is being pressed during the rotation operation.
    void
    getRotateMatrix (int dx, int dy, float* rotation_matrix_a,
                     float* rotation_matrix_b, BitMask buttons) const;

    /// @brief Computes the centroid of the selected points
    void
    findSelectionCenter ();

    /// a shared pointer pointing to the selection object
    ConstSelectionPtr selection_ptr_;

    /// a shared pointer pointing to the cloud object
    CloudPtr cloud_ptr_;

    /// a shared pointer pointing to the command queue of the widget.
    CommandQueuePtr command_queue_ptr_;

    /// the trackball associated with this transform
    TrackBall trackball_;

    /// last recorded mouse positions
    int x_, y_;

    /// The centroid of the selected points.
    float center_xyz_[XYZ_SIZE];

    /// the transform matrix to be used for updating the coordinates of the
    /// selected points in the cloud
    float transform_matrix_[MATRIX_SIZE];

    /// scaling factor used to control the speed which the display translates
    /// the point cloud
    float translate_factor_;

    /// default translation factor
    static const float DEFAULT_TRANSLATE_FACTOR_;

    /// the copy of the modifiers passed in the start function.
    BitMask modifiers_;

};
#endif // SELECTION_TRANSFORMER_H_
