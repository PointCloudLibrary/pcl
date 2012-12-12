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
/// @file   toolInterface.h
/// @details An abstract class that provides a single interface for all tools
/// in the editor.  Objects inheriting from this class are assumed to be based
/// on mouse inputs from the user along with modifier keys (ALT, CTRL, SHIFT).
/// Note that the CTRL key may be evaluated as the Command key in OSX.
/// @author  Yue Li and Matthew Hielsberg

#ifndef TOOL_INTERFACE_H_
#define TOOL_INTERFACE_H_

#include <pcl/apps/point_cloud_editor/localTypes.h>

/// @brief the parent class of all the select and the transform tool classes
class ToolInterface
{
  public:
    /// @brief Destructor.
    virtual ~ToolInterface ()
    {
    }

    /// @brief set the initial state of the tool from the screen coordinates
    /// of the mouse as well as the value of the modifier.
    /// @param x the x coordinate of the mouse screen coordinates.
    /// @param y the y coordinate of the mouse screen coordinates.
    /// @param modifiers The keyboard modifiers. We use modifier to change the
    /// behavior of a tool. Values of a modifier can be control key, alt key
    /// shift key, or no key is pressed. See the subclasses of this class
    /// for specific usages of the modifiers.
    /// @param buttons The state of the mouse buttons
    virtual
    void
    start (int x, int y, BitMask modifiers, BitMask buttons) = 0;

    /// @brief update the state of the tool from the screen coordinates
    /// of the mouse as well as the value of the modifier.
    /// @param x the x coordinate of the mouse screen coordinates.
    /// @param y the y coordinate of the mouse screen coordinates.
    /// @param modifiers The keyboard modifiers. We use modifier to change the
    /// behavior of a tool. Values of a modifier can be control key, alt key
    /// shift key, or no key is pressed. See the subclasses of this class
    /// for specific usages of the modifiers.
    /// @param buttons The state of the mouse buttons
    virtual
    void
    update (int x, int y, BitMask modifiers, BitMask buttons) = 0;

    /// @brief set final state of the tool from the screen coordinates
    /// of the mouse as well as the value of the modifier. Also performs the
    /// corresponding functionalities of the tool.
    /// @param x the x coordinate of the mouse screen coordinates.
    /// @param y the y coordinate of the mouse screen coordinates.
    /// @param modifiers The keyboard modifiers. We use modifier to change the
    /// behavior of a tool. Values of a modifier can be control key, alt key
    /// shift key, or no key is pressed. See the subclasses of this class
    /// for specific usages of the modifiers.
    /// @param buttons The state of the mouse buttons
    virtual
    void
    end (int x, int y, BitMask modifiers, BitMask buttons) = 0;
  
    /// @brief a rendering facility used by a tool. For instance, if this tool
    /// is a selection tool, this function draws highlighted points as well as
    /// selection region, e.g., rubberband, box, etc.
    virtual
    void
    draw () const = 0;
    
  protected:
    /// @brief Default constructor
    ToolInterface ()
    {
    }

  private:
    /// @brief Copy constructor - tools are non-copyable
    ToolInterface (const ToolInterface&)
    {
      assert(false);
    }

    /// @brief Equal operator - tools are non-copyable
    ToolInterface&
    operator= (const ToolInterface&)
    {
      assert(false); return (*this);
    }
};
#endif // TOOL_INTERFACE_H_
