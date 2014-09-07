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

/// @file   pasteCommand.h
/// @details A PasteCommand object that allows the duplication of selected
/// points in a cloud object as well as undo.
/// @author  Yue Li and Matthew Hielsberg


#ifndef PASTE_COMMAND_H_
#define PASTE_COMMAND_H_

#include <pcl/apps/point_cloud_editor/command.h>
#include <pcl/apps/point_cloud_editor/localTypes.h>

class PasteCommand : public Command
{
  public:
    /// @brief Constructor
    /// @param copy_buffer_ptr a shared pointer pointing to the copy buffer.
    /// @param selection_ptr a shared pointer pointing to the selection object.
    /// @param cloud_ptr a shared pointer pointing to the cloud object.
    PasteCommand (ConstCopyBufferPtr copy_buffer_ptr,
                  SelectionPtr selection_ptr, CloudPtr cloud_ptr);
    // comment that the selection is updated (also resets the matrix in cloud)

    /// @brief Destructor
    ~PasteCommand ()
    {
    }
  
  protected:
    /// @brief Appends the points in the copy buffer into the cloud.
    /// @details After appending the points to the cloud, this function also
    /// updates the selection object to point to the newly pasted points.  This
    /// also updates the selection object to point to the newly pasted points.
    void
    execute ();

    /// @brief Removes the points that were pasted to the cloud.
    void
    undo ();

  private:
    /// @brief Default constructor - object is not default constructable
    PasteCommand ()
    {
    }
    
    /// @brief Copy constructor - commands are non-copyable
    PasteCommand (const PasteCommand&)
    {
      assert(false);
    }

    /// @brief Equal operator - commands are non-copyable
    PasteCommand&
    operator= (const PasteCommand&)
    {
      assert(false); return (*this);
    }

    /// a pointer pointing to the copy buffer.
    ConstCopyBufferPtr copy_buffer_ptr_;

    /// A shared pointer pointing to the selection object.
    SelectionPtr selection_ptr_;

    /// a pointer pointing to the cloud
    CloudPtr cloud_ptr_;

    /// The size of the cloud before new points are pasted. This value is used
    /// to mark the point where points were added to the cloud. In order to
    /// support undo, one only has to resize the cloud using this value.
    unsigned int prev_cloud_size_;
};
#endif // PASTE_COMMAND_H_
