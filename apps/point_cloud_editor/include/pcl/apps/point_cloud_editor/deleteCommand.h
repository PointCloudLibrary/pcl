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

/// @file   deleteCommand.h
/// @details A Delete object provides the functionality for removing points
/// from the cloud as well as the ability to undo the removal.
/// @author  Yue Li and Matthew Hielsberg

#ifndef DELETE_COMMAND_H_
#define DELETE_COMMAND_H_

#include <pcl/apps/point_cloud_editor/command.h>
#include <pcl/apps/point_cloud_editor/localTypes.h>
#include <pcl/apps/point_cloud_editor/copyBuffer.h>
#include <pcl/apps/point_cloud_editor/selection.h>

class DeleteCommand : public Command
{
  public:
    /// @brief Constructor
    /// @param selection_ptr A shared pointer pointing to the selection object.
    /// @param cloud_ptr A shared pointer pointing to the cloud object.
    DeleteCommand (SelectionPtr selection_ptr, CloudPtr cloud_ptr);
   
    /// @brief Destructor
    ~DeleteCommand ()
    {
    }
    
  protected:
    /// @brief Removes the selected points and maintains a backup for undo.
    void 
    execute ();
    
    /// @brief Returns the deleted points to the cloud, Order is not preserved.
    void 
    undo ();

  private:
    /// @brief Default constructor - object is not default constructable
    DeleteCommand ():  deleted_selection_(CloudPtr())
    {
      assert(false);
    }
    
    /// @brief Copy constructor - commands are non-copyable
    DeleteCommand (const DeleteCommand& c)
      : deleted_selection_(c.deleted_selection_)
    {
      assert(false);
    }

    /// @brief Equal operator - commands are non-copyable
    DeleteCommand&
    operator= (const DeleteCommand&)
    {
      assert(false); return (*this);
    }

    /// a pointer pointing to the cloud
    CloudPtr cloud_ptr_;

    /// A shared pointer pointing to the selection object.
    SelectionPtr selection_ptr_;

    /// a selection which backs up the index of the deleted points in the
    /// original cloud.
    Selection deleted_selection_;

    /// a copy buffer which backs up the points deleted from the cloud.
    CopyBuffer deleted_cloud_buffer_;
};

#endif // DELETE_COMMAND_H_
