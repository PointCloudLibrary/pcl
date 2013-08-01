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

/// @file   select1DTool.cpp
/// @details the implementation of Select1DTool class.
/// @author  Yue Li and Matthew Hielsberg

#include <algorithm>
#include <qgl.h>
#include <pcl/apps/point_cloud_editor/select1DTool.h>
#include <pcl/apps/point_cloud_editor/cloud.h>
#include <pcl/apps/point_cloud_editor/selection.h>
#include <pcl/apps/point_cloud_editor/localTypes.h>

Select1DTool::Select1DTool (SelectionPtr selection_ptr, CloudPtr cloud_ptr)
  : selection_ptr_(selection_ptr), cloud_ptr_(cloud_ptr)
{
}

void
Select1DTool::end (int x, int y, BitMask modifiers, BitMask buttons)
{
  if (!cloud_ptr_)
    return;
  if (!(buttons & LEFT))
    return;

  unsigned int index = 0;
  union
  {
    unsigned char pixel[4];// XXX - assume uchar = 1 byte
    unsigned int id;
  } u;
  // XXX - The following assumes sizeof(unsigned int) == 4 bytes
  glPushAttrib(GL_COLOR_BUFFER_BIT | GL_PIXEL_MODE_BIT | GL_HINT_BIT |
               GL_LINE_BIT | GL_POINT_BIT);
  {
    glDisable(GL_POINT_SMOOTH);
    glDisable(GL_LINE_SMOOTH);
    glDisable( GL_BLEND );
    glClearColor(0,0,0,0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    GLint viewport[4];
    glGetIntegerv(GL_VIEWPORT, viewport);
    IncIndex inc(1);// start the indexing from 1, since the clear color is 0
    unsigned int *index_arr = new unsigned int[cloud_ptr_->size()];
    std::generate_n(index_arr, cloud_ptr_->size(), inc);

    glPushClientAttrib(GL_CLIENT_VERTEX_ARRAY_BIT);
    {
      glEnableClientState(GL_COLOR_ARRAY);
      glColorPointer(4, GL_UNSIGNED_BYTE, 0, index_arr);
      cloud_ptr_->draw(true);
      glReadPixels(x, viewport[3] - y, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, u.pixel);
    }
    glPopClientAttrib();
    delete [] index_arr;
  }
  glPopAttrib();

  if (!u.id)
    return; // no selection - they did not hit a point

  // the color buffer used [1,n] to color the points - retrieve the point index
  index = u.id-1;

  if (modifiers & SHFT)
  {
    selection_ptr_->addIndex(index);
  }
  else if (modifiers & CTRL)
  {
    selection_ptr_->removeIndex(index);
  }
  else
  {
    selection_ptr_->clear();
    selection_ptr_->addIndex(index);
  }
  cloud_ptr_->setSelection(selection_ptr_);
}
