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
///
/// @file   copyBuffer.cpp
/// @details implementation of the class CopyBuffer
/// @author  Yue Li and Matthew Hielsberg

#include <pcl/apps/point_cloud_editor/copyBuffer.h>
#include <pcl/apps/point_cloud_editor/selection.h>
#include <pcl/apps/point_cloud_editor/statistics.h>
#include <pcl/apps/point_cloud_editor/common.h>

CopyBuffer::CopyBuffer (const CopyBuffer& copy_buffer) :
  buffer_(copy_buffer.buffer_)
{
}

CopyBuffer&
CopyBuffer::operator= (const CopyBuffer& copy_buffer)
{
  buffer_ = copy_buffer.buffer_;
  return (*this);
}

void
CopyBuffer::set (ConstCloudPtr cloud_ptr, const Selection& selection)
{
  clean();
  if (!cloud_ptr)
      return;
  Selection::const_iterator s_it;
  for(s_it = selection.begin(); s_it != selection.end(); ++s_it)
    buffer_.append( (*cloud_ptr)[*s_it] );
}

const Cloud&
CopyBuffer::get () const
{
  return (buffer_);
}

Cloud&
CopyBuffer::get ()
{
  return (buffer_);
}

void
CopyBuffer::clean ()
{
  buffer_.clear();
}

std::string
CopyBuffer::getStat () const
{
  if (buffer_.size() == 0)
    return ("");
  std::string title = "The number of points copied to the clipboard: ";
  std::string num_str;
  ::toString(buffer_.size(), num_str);
  return (title + num_str);
}
