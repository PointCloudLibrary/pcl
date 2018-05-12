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

/// @file   select2DTool.cpp
/// @details the implementation of Select2DTool class.
/// @author  Yue Li and Matthew Hielsberg

#include <pcl/apps/point_cloud_editor/select2DTool.h>
#include <pcl/apps/point_cloud_editor/cloud.h>
#include <pcl/apps/point_cloud_editor/selection.h>

const float Select2DTool::DEFAULT_TOOL_DISPLAY_SIZE_ = 2.0f;

const float Select2DTool::DEFAULT_TOOL_DISPLAY_COLOR_RED_ = 1.0f;
const float Select2DTool::DEFAULT_TOOL_DISPLAY_COLOR_GREEN_ = 1.0f;
const float Select2DTool::DEFAULT_TOOL_DISPLAY_COLOR_BLUE_ = 1.0f;


Select2DTool::Select2DTool (SelectionPtr selection_ptr, CloudPtr cloud_ptr)
  : selection_ptr_(selection_ptr), cloud_ptr_(cloud_ptr), display_box_(false)
{
}

Select2DTool::~Select2DTool ()
{
}

void
Select2DTool::start (int x, int y, BitMask, BitMask)
{
  if (!cloud_ptr_)
    return;
  origin_x_ = x;
  origin_y_ = y;
}

void
Select2DTool::update (int x, int y, BitMask, BitMask)
{
  if (!cloud_ptr_)
    return;
  final_x_ = x;
  final_y_ = y;
  display_box_ = true;
}

void
Select2DTool::end (int x, int y, BitMask modifiers, BitMask)
{
  if (!cloud_ptr_)
    return;
  final_x_ = x;
  final_y_ = y;
  display_box_ = false;
  // don't select anything if we don't have a selection box.
  if ((final_x_ == origin_x_) || (final_y_ == origin_y_))
    return;

  GLint viewport[4];
  glGetIntegerv(GL_VIEWPORT,viewport);
  IndexVector indices;
  GLfloat project[16];
  glGetFloatv(GL_PROJECTION_MATRIX, project);

  Point3DVector ptsvec;
  cloud_ptr_->getDisplaySpacePoints(ptsvec);
  for(unsigned int i = 0; i < ptsvec.size(); ++i)
  {
    Point3D pt = ptsvec[i];
    if (isInSelectBox(pt, project, viewport))
      indices.push_back(i);
  }

  if (modifiers & SHFT)
  {
    selection_ptr_->addIndex(indices);
  }
  else if (modifiers & CTRL)
  {
    selection_ptr_->removeIndex(indices);
  }
  else
  {
    selection_ptr_->clear();
    selection_ptr_->addIndex(indices);
  }
  cloud_ptr_->setSelection(selection_ptr_);
}

bool
Select2DTool::isInSelectBox (const Point3D& pt,
                             const GLfloat* project,
                             const GLint* viewport) const
{
  float w = pt.z * project[11];
  float x = (pt.x * project[0] + pt.z * project[8]) / w;
  float y = (pt.y * project[5] + pt.z * project[9]) / w;
  float min_x = std::min(origin_x_, final_x_)/(viewport[2]*0.5) - 1.0;
  float max_x = std::max(final_x_, origin_x_)/(viewport[2]*0.5) - 1.0;
  float max_y = (viewport[3] - std::min(origin_y_, final_y_))/(viewport[3]*0.5) - 1.0;
  float min_y = (viewport[3] - std::max(origin_y_, final_y_))/(viewport[3]*0.5) - 1.0;
  // Ignore the points behind the camera
  if (w < 0)
    return (false);
  // Check the left and right sides
  if ((x < min_x) || (x > max_x))
    return (false);
  // Check the bottom and top
  if ((y < min_y) || (y > max_y))
    return (false);
  return (true);
}

void
Select2DTool::draw() const
{
  if (!display_box_)
    return;
  GLint viewport[4];
  glGetIntegerv(GL_VIEWPORT, viewport);
  // draw the selection box
  drawRubberBand(viewport);
  // highlight all the points in the rubberband
  highlightPoints(viewport);
}

void
Select2DTool::drawRubberBand (GLint* viewport) const
{
  // set the line width of the rubberband
  glLineWidth(DEFAULT_TOOL_DISPLAY_SIZE_);
  // set the color of the rubberband
  glColor3f(DEFAULT_TOOL_DISPLAY_COLOR_RED_,
            DEFAULT_TOOL_DISPLAY_COLOR_GREEN_,
            DEFAULT_TOOL_DISPLAY_COLOR_BLUE_);
  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  {
    glLoadIdentity();
    glOrtho(0, viewport[2], viewport[3], 0, -1, 1);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    {
      glLoadIdentity();
      glLineStipple (3, 0x8888);
      glEnable(GL_LINE_STIPPLE);
      glBegin(GL_LINE_LOOP);
      {
        glVertex2d(origin_x_, origin_y_); // Top Left
        glVertex2d(final_x_,  origin_y_); // Top Right
        glVertex2d(final_x_,  final_y_);  // Bottom Right
        glVertex2d(origin_x_, final_y_);  // Bottom Left
      }
      glEnd();
      glDisable(GL_LINE_STIPPLE);
    }
    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
  }
  glPopMatrix();
  glMatrixMode(GL_MODELVIEW);
}

void
Select2DTool::highlightPoints (GLint* viewport) const
{
  double width = abs(origin_x_ - final_x_);
  double height = abs(origin_y_ - final_y_);
  glPushAttrib(GL_SCISSOR_BIT);
  {
    glEnable(GL_SCISSOR_TEST);
    glScissor(std::min(origin_x_, final_x_),
              std::min(viewport[3] - final_y_, viewport[3] - origin_y_),
              width, height);
    cloud_ptr_ -> drawWithHighlightColor();
  }
  glPopAttrib();
}

