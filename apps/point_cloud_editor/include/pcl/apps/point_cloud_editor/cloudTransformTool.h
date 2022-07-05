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

/// @file cloudTransformTool.h
/// @details Provides a tool for changing the view of cloud through a simple
/// transformation matrix using inputs from the mouse.
/// @author Yue Li and Matthew Hielsberg

#pragma once

#include <pcl/apps/point_cloud_editor/toolInterface.h>
#include <pcl/apps/point_cloud_editor/localTypes.h>
#include <pcl/apps/point_cloud_editor/trackball.h>

/// @brief The cloud transform tool computes the transform matrix from user's
/// mouse operation. It then updates the cloud with the new transform matrices
/// to make the cloud be rendered appropriately.
class CloudTransformTool : public ToolInterface
{
  public:
    /// @brief Constructor
    /// @param cloud_ptr a shared pointer pointing to the cloud object.
    CloudTransformTool (CloudPtr cloud_ptr);

    /// @brief Destructor
    ~CloudTransformTool () override;

    /// @brief Initialize the current transform with mouse screen coordinates
    /// and key modifiers.
    /// @param x the x value of the mouse screen coordinates.
    /// @param y the y value of the mouse screen coordinates.
    /// @param modifiers The keyboard modifiers.  This function does not make
    /// use of this parameter.
    /// @param buttons The state of the mouse buttons.  This function does not
    /// make use of this parameter.
    void
    start (int x, int y, BitMask modifiers, BitMask buttons) override;

    /// @brief Updates the transform matrix of this object with mouse screen
    /// coordinates and key modifiers.
    /// @details When the LEFT mouse button is down the motion of the mouse is
    /// used to compute various transforms for the cloud display.  Depending on
    /// the modifiers, the transformation matrix is computed correspondingly.
    /// When shift is pressed, the motion of mouse indicates a scale. If
    /// no key modifiers is pressed, the mouse move indicates a rotation.  The
    /// control key pans the display, and the alt key translates along the
    /// z-axis.
    /// @param x The x value of the mouse screen coordinates.
    /// @param y The y value of the mouse screen coordinates.
    /// @param modifiers the key modifier.  SHIFT scales the point cloud
    /// display. CONTROL pans the point cloud parallel to the view plane.  ALT
    /// moves the point cloud in/out along the z-axis (perpendicular to the
    /// view plane).  If no modifier is pressed then the cloud display is
    /// rotated.
    /// @param buttons The LEFT mouse button must be pressed for any transform
    /// to be generated.  All other buttons are ignored.
    void
    update (int x, int y, BitMask modifiers, BitMask buttons) override;

    /// @brief Updates the transform matrix of this object with mouse screen
    /// coordinates and key modifiers. Then right multiplies the cloud_matrix_
    /// matrix of the cloud object with the transform matrix of this object.
    /// @details This function is not required by this tool
    void
    end (int, int, BitMask, BitMask) override
    {
    }

    /// @brief This function does nothing for this cloud transform tool.
    void
    draw() const override
    {
    }

  private:

    /// generate translate matrix for the xy plane
    void
    getTranslateMatrix (int dx, int dy, float* matrix);

    /// generate translate matrix for the z direction
    void
    getZTranslateMatrix (int dy, float* matrix);

    /// generate scale matrix
    void
    getScaleMatrix (int dy, float* matrix) const;

    /// the transform matrix to be used for updating the coordinates of all
    /// the points in the cloud
    float transform_matrix_[MATRIX_SIZE];

    /// a shared pointer pointing to the cloud object.
    CloudPtr cloud_ptr_;

    /// the trackball associated with this transform
    TrackBall trackball_;
    
    /// last recorded mouse positions
    int x_, y_;

    /// scaling factor used to control the speed which the display scales the
    /// point cloud
    float scale_factor_;

    /// scaling factor used to control the speed which the display translates
    /// the point cloud
    float translate_factor_;

    /// default scaling factor
    static const float DEFAULT_SCALE_FACTOR_;

    /// default translation factor
    static const float DEFAULT_TRANSLATE_FACTOR_;
};
