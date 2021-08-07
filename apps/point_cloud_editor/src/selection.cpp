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
/// @file   selection.cpp
/// @details implementation of the class Selection
/// @author  Yue Li and Matthew Hielsberg

#include <algorithm>
#include <pcl/apps/point_cloud_editor/selection.h>
#include <pcl/apps/point_cloud_editor/cloud.h>
#include <pcl/apps/point_cloud_editor/common.h>

Selection&
Selection::operator= (const Selection& selection)
{
  cloud_ptr_ = selection.cloud_ptr_;
  selected_indices_ = selection.selected_indices_;
  return (*this);
}

void
Selection::addIndex (unsigned int index)
{
  selected_indices_.insert(index);
}

void
Selection::removeIndex (unsigned int index)
{
  selected_indices_.erase(index);
}

void
Selection::addIndex (const IndexVector &indices)
{
  selected_indices_.insert(indices.begin(), indices.end());
}

void
Selection::removeIndex (const IndexVector &indices)
{
  for(const unsigned int &index : indices)
    removeIndex(index);
}

void
Selection::addIndexRange (unsigned int start, unsigned int num)
{
  for(unsigned int i = start; i < start + num; ++i)
    addIndex(i);
}

void
Selection::removeIndexRange (unsigned int start, unsigned int num)
{
  for(unsigned int i = start; i < start + num; ++i)
    removeIndex(i);
}

bool
Selection::isSelected (unsigned int index) const
{
  if (index >= cloud_ptr_->size())
    return (false);
  iterator it = selected_indices_.find(index);
  if (it != selected_indices_.end())
      return (true);
  return (false);
}

void
Selection::invertSelect ()
{
  std::set<unsigned int> s, complement;
  IncIndex inc;
  for(unsigned int i = 0; i < cloud_ptr_->size(); ++i)
    s.insert(i);
  std::set_difference(s.begin(), s.end(),
                      selected_indices_.begin(), selected_indices_.end(),
                      std::inserter(complement, complement.end()));
  selected_indices_ = complement;
}

std::string
Selection::getStat () const
{
  if (selected_indices_.empty ())
    return "";
  const std::string title = "Total number of selected points: ";
  const std::string num_str = std::to_string(selected_indices_.size());
  return title + num_str;
}
