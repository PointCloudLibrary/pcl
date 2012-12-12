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

/// @file   select1DTool.h
/// @details Tool for selecting and deselecting individual points in the cloud.
/// @author  Yue Li and Matthew Hielsberg

#ifndef SELECT_1D_TOOL_H
#define SELECT_1D_TOOL_H

#include <pcl/apps/point_cloud_editor/toolInterface.h>
#include <pcl/apps/point_cloud_editor/localTypes.h>

class Select1DTool : public ToolInterface
{
  public:
    /// @brief Constructor
    /// @param selection_ptr a shared pointer pointing to the selection object.
    /// @param cloud_ptr a shared pointer pointing to the cloud object.
    Select1DTool (SelectionPtr selection_ptr, CloudPtr cloud_ptr);

    /// @brief Destructor
    ~Select1DTool ()
    {
    }
  
    /// @brief Does nothing for 1D selection.
    /// @sa end
    void
    start (int, int, BitMask, BitMask)
    {
    }
  
    /// @brief Does nothing for 1D selection.
    /// @sa end
    void
    update (int, int, BitMask, BitMask)
    {
    }

    /// @brief Select or deselect the point under the mouse using GL's selection
    /// facility.
    /// @details If shift is pressed when the selection is made, the selected
    /// point is appended to the existing selection. If instead ctrl is pressed,
    /// the selected point will be removed from the existing selection.  If
    /// neither shift nor ctrl is pressed when the selection is made then the
    /// selected point, if any, will replace any current selection. Note that
    /// the ctrl key may be evaluated as the command key in OSX.
    /// @param x the x value of the mouse screen coordinates.
    /// @param y the y value of the mouse screen coordinates.
    /// @param modifiers the key modifier. SHIFT adds selected points to the
    /// selection.  CTRL removes points and if neither are pressed then a new
    /// selection is made.
    /// @param buttons The state of the mouse buttons.  All interaction with
    /// this tool requires the LEFT mouse button.  All others are ignored.
    void
    end (int x, int y, BitMask modifiers, BitMask buttons);

    /// @brief This function does nothing.
    void
    draw () const
    {
    }

  private:
    /// @brief Default constructor - object is not default constructable
    Select1DTool()
    {
      assert(false);
    }

    /// a shared pointer pointing to the selection object
    SelectionPtr selection_ptr_;

    /// a shared pointer pointing to the cloud object
    CloudPtr cloud_ptr_;
};
#endif // SELECT_1D_TOOL_H
