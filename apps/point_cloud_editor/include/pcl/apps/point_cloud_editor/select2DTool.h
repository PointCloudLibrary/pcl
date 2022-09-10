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

/// @file   select2DTool.h
/// @details the declaration of the 2D selection tool class providing
/// functionalities to enable 2D selection.
/// @author  Yue Li and Matthew Hielsberg

#pragma once

#include <qgl.h>
#include <pcl/apps/point_cloud_editor/toolInterface.h>
#include <pcl/apps/point_cloud_editor/localTypes.h>

#include <pcl/memory.h>  // for pcl::shared_ptr

class Selection;

class Select2DTool : public ToolInterface
{
  public:
    /// The type for shared pointer pointing to a selection buffer
    using SelectionPtr = pcl::shared_ptr<Selection>;

    /// @brief Constructor
    /// @param selection_ptr a shared pointer pointing to the selection object.
    /// @param cloud_ptr a shared pointer pointing to the cloud object.
    Select2DTool (SelectionPtr selection_ptr, CloudPtr cloud_ptr);

    /// @brief Destructor
    ~Select2DTool () override;
  
    /// @brief Initializes the selection tool with the initial mouse screen
    /// coordinates and key modifiers. The passed coordinates are used for
    /// determining the coordinates of the upper left corner of the rubber band.
    /// @param x the x value of the mouse screen coordinates.
    /// @param y the y value of the mouse screen coordinates.
    /// @param modifiers the key modifier. There are three possible values for
    /// modifiers: 1. shift key, 2. ctrl key, 3. no modifier is pressed. Note
    /// that the ctrl key may be evaluated as the command key in OSX.
    void
    start (int x, int y, BitMask modifiers, BitMask mouseButton) override;

    /// @brief Update the selection tool from the current mouse screen
    /// coordinates and key modifiers.
    /// @details Creates a 2D rubberband. The coordinates of the lower right
    /// corner of the rubberband is computed from the passed coordinates.
    /// @param x the x value of the mouse screen coordinates.
    /// @param y the y value of the mouse screen coordinates.
    /// @param modifiers the key modifier.
    void
    update (int x, int y, BitMask modifiers, BitMask mouseButton) override;

    /// @brief Update the coordinates of the lower right corner of the rubber
    /// band and process the points in the rubber band.
    /// @details The points which fall into the selection region are processed
    /// according to the value of the modifier: If shift is pressed, the
    /// selected points are appended to the existing selection. If ctrl is
    /// pressed, the points will be removed from the existing selection
    /// if they were elected previously, otherwise nothing happens.
    /// @param x the x value of the mouse screen coordinates.
    /// @param y the y value of the mouse screen coordinates.
    /// @param modifiers the key modifier.
    void
    end (int x, int y, BitMask modifiers, BitMask mouseButton) override;

    /// @brief Checks whether a point is inside the selection region.
    /// @param pt the point to be checked against the selection region.
    /// @param project the projection matrix obtained from GL.
    /// @param viewport the current viewport obtained from GL.
    bool
    isInSelectBox (const Point3D& pt, const GLfloat* project,
                   const GLint* viewport) const;

    /// @brief Draws the rubber band as well as any highlighted points during
    /// the 'update' phase (i.e. before the selection is made by a call to end).
    void
    draw () const override;

    /// The default size in pixels of the rubberband tool outline
    static const float DEFAULT_TOOL_DISPLAY_SIZE_;

    /// The default color of the rubberband tool - red component
    static const float DEFAULT_TOOL_DISPLAY_COLOR_RED_;
    /// The default color of the rubberband tool - green component
    static const float DEFAULT_TOOL_DISPLAY_COLOR_GREEN_;
    /// The default color of the rubberband tool - blue component
    static const float DEFAULT_TOOL_DISPLAY_COLOR_BLUE_;


  private:
    /// @brief Default constructor - object is not default constructable
    Select2DTool()
    {
      assert(false);
    }

    /// @brief draw the 2D selection rubber band.
    /// @param viewport the viewport obtained from GL
    void
    drawRubberBand (GLint* viewport) const;

    /// @brief highlight all the points in the rubber band.
    /// @detail draw the cloud using a stencil buffer. During this time, the
    /// points that are highlighted will not be recorded by the selecion object.
    /// @param viewport the viewport obtained from GL
    void
    highlightPoints (GLint* viewport) const;

    /// a shared pointer pointing to the selection object
    SelectionPtr selection_ptr_;

    /// a shared pointer pointing to the cloud object
    CloudPtr cloud_ptr_;

    /// the original mouse screen coordinates
    int origin_x_, origin_y_;

    /// the final mouse screen coordinates
    int final_x_, final_y_;

    /// switch for selection box rendering
    bool display_box_;

};
