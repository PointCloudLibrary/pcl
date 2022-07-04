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

/// @file commandQueue.h
/// @details This object provides a dequeue of commands as well as operations to
/// manipulate the deque.  This class is designed to work with Command objects
/// in order to both execute them and undo them in the proper order.
/// @author Yue Li and Matthew Hielsberg

#pragma once

#include <deque>
#include <pcl/apps/point_cloud_editor/localTypes.h>

/// @brief A structure for managing commands
/// @details A command queue object provides a dequeue of commands as well as
/// operations to manipulate the commands in the queue. Operations include
/// executing and undoing the commands in the queue. A command queue object
/// is non-copyable.
class CommandQueue
{
  public:
    /// @brief Default Constructor
    /// @details Creates a command queue object and makes its depth limit
    /// be the default value.
    CommandQueue ();

    /// @brief Constructor
    /// @details Create a command queue with specified depth limit.
    /// @param max_size the value to be used to set the depth limit of this
    /// object.
    CommandQueue (unsigned int max_size);

    /// @brief Destructor
    ~CommandQueue ()
    = default;

    /// @brief Executes a command. If the command has an undo function, then
    /// adds the command to the queue.
    /// @param command_ptr a shared pointer pointing to a command object whose
    /// execute function will be invoked by this object.
    void
    execute (const CommandPtr&);

    /// @brief Undoes the last command by popping the tail of the queue, invoke
    /// the undo function of the command.
    void
    undo ();

    /// @brief Changes the command history limit.
    /// @details If the passed size is smaller than the current size then the
    /// oldest commands are removed (their undo functions are not called).
    /// @param size The new maximum number of commands that may exist in this
    /// queue for undo purposes.
    /// @return The actual maximum size set.  It may happen that the passed
    /// value is too large and cannot be set.
    unsigned int
    setMaxSize(unsigned int size);

    /// @brief The default maximal size of the depth limit
    static const unsigned int DEFAULT_MAX_SIZE_ = 200;
    
  private:
    /// @brief Copy constructor - object is non-copyable
    CommandQueue(const CommandQueue&)
    {
      assert(false);
    }
    
    /// @brief Equal operator - object is non-copyable
    CommandQueue& 
    operator= (const CommandQueue&)
    {
      assert(false); return (*this);
    }

    /// @brief Enforces the depth limit of the command queue. If the depth is
    /// larger than the depth limit, a deque operation will be invoked.
    void
    enforceDequeLimit ();

    /// The internal representation of this object.
    std::deque<CommandPtr> command_deque_;

    /// The depth limit of the command queue.
    unsigned int depth_limit_;
};
