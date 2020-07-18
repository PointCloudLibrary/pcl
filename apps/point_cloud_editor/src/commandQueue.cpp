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

/// @file   commandQueue.cpp
/// @details Implementation of the class CommandQueue, see commandQueue.h for
/// details.
/// @author  Yue Li and Matthew Hielsberg

#include <pcl/apps/point_cloud_editor/commandQueue.h>
#include <pcl/apps/point_cloud_editor/command.h>

CommandQueue::CommandQueue () : depth_limit_(DEFAULT_MAX_SIZE_)
{
}

CommandQueue::CommandQueue (unsigned int max_size) : depth_limit_(max_size)
{
}

void
CommandQueue::enforceDequeLimit ()
{
  // the following loop should actually execute only one iteration.
  while (command_deque_.size() > depth_limit_)
    command_deque_.pop_front();
}

void
CommandQueue::execute (const CommandPtr& command_ptr)
{
  if (!command_ptr)
    return;
  command_ptr->execute();
  if (command_ptr->hasUndo())
  {
    command_deque_.push_back(command_ptr);
    enforceDequeLimit();
  }
}

void
CommandQueue::undo ()
{
  // no-op when no command is in the queue.
  if (command_deque_.empty())
    return;
  (command_deque_.back())->undo();
  command_deque_.pop_back();
}

unsigned int
CommandQueue::setMaxSize (unsigned int size)
{
  depth_limit_ = size;
  if (depth_limit_ > command_deque_.max_size())
    depth_limit_ = command_deque_.max_size();
  // resize should be faster than enforceDequeLimit
  if (command_deque_.size() > depth_limit_)
    command_deque_.resize(depth_limit_);
  return (depth_limit_);
}



