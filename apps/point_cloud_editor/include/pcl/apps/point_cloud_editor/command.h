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

/// @file command.h
/// @details The abstract parent class for each class implementing a specific
/// command of the cloud editor.  This class is designed to be used with the
/// CommandQueue.
/// @author Yue Li and Matthew Hielsberg

#pragma once

#include <pcl/apps/point_cloud_editor/localTypes.h>

/// @brief The abstract parent class of all the command classes. Commands are
/// non-copyable.
class Command
{
  public:
    /// @brief Destructor
    virtual ~Command ()
    = default;

  protected:
    /// Allows command queues to be the only objects which are able to execute
    /// commands.
    friend class CommandQueue;

    /// @brief The default constructor.
    /// @details Derived commands are assumed to have undo by default.  Each
    /// is free to override this.
    Command () : has_undo_(true)
    {
    }

    /// @brief Returns true if the command has an undo function.
    inline
    bool
    hasUndo () const
    {
      return (has_undo_);
    }

    /// @brief Executes the command.
    virtual
    void
    execute () = 0;

    /// @brief Undos the command.
    virtual
    void
    undo () = 0;

    /// @brief a flag indicates whether the command has an undo function.
    bool has_undo_;

  private:
    /// @brief Copy Constructor - object is non-copyable
    Command (const Command&)
    {
      assert(false);
    }

    /// @brief Equal Operator - object is non-copyable
    Command&
    operator= (const Command&)
    {
      assert(false); return (*this);
    }
};
