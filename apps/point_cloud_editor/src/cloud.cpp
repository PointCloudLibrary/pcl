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

/// @file   cloud.cpp
/// @details Implementation of the class Cloud, see cloud.h for details.
/// @author  Yue Li and Matthew Hielsberg

#include <algorithm>
#include <qgl.h>
#include <pcl/apps/point_cloud_editor/cloud.h>
#include <pcl/apps/point_cloud_editor/selection.h>
#include <pcl/apps/point_cloud_editor/localTypes.h>
#include <pcl/apps/point_cloud_editor/common.h>
#include <pcl/apps/point_cloud_editor/copyBuffer.h>

const float Cloud::DEFAULT_POINT_DISPLAY_SIZE_ = 2.0f;
const float Cloud::DEFAULT_POINT_HIGHLIGHT_SIZE_ = 4.0f;

const float Cloud::DEFAULT_POINT_DISPLAY_COLOR_RED_ = 1.0f;
const float Cloud::DEFAULT_POINT_DISPLAY_COLOR_GREEN_ = 1.0f;
const float Cloud::DEFAULT_POINT_DISPLAY_COLOR_BLUE_ = 1.0f;

const float Cloud::DEFAULT_POINT_HIGHLIGHT_COLOR_RED_ = 0.0f;
const float Cloud::DEFAULT_POINT_HIGHLIGHT_COLOR_GREEN_ = 1.0f;
const float Cloud::DEFAULT_POINT_HIGHLIGHT_COLOR_BLUE_ = 0.0f;


Cloud::Cloud ()
  : use_color_ramp_(true), color_ramp_axis_(Y),
  display_scale_(1.0f),
  point_size_(DEFAULT_POINT_DISPLAY_SIZE_),
  selected_point_size_(DEFAULT_POINT_HIGHLIGHT_SIZE_),
  select_translate_x_(0), select_translate_y_(0), select_translate_z_(0)
{
  std::fill_n(center_xyz_, XYZ_SIZE, 0.0f);
  setIdentity(cloud_matrix_);
  setIdentity(select_matrix_);
  color_[RED] = DEFAULT_POINT_DISPLAY_COLOR_RED_;
  color_[GREEN] = DEFAULT_POINT_DISPLAY_COLOR_GREEN_;
  color_[BLUE] = DEFAULT_POINT_DISPLAY_COLOR_BLUE_;
  highlight_color_[RED] = DEFAULT_POINT_HIGHLIGHT_COLOR_RED_;
  highlight_color_[GREEN] = DEFAULT_POINT_HIGHLIGHT_COLOR_GREEN_;
  highlight_color_[BLUE] = DEFAULT_POINT_HIGHLIGHT_COLOR_BLUE_;
}

Cloud::Cloud (const Cloud3D &cloud, bool register_stats)
  : cloud_(cloud),
  use_color_ramp_(true), color_ramp_axis_(Y),
  display_scale_(1.0f),
  point_size_(DEFAULT_POINT_DISPLAY_SIZE_),
  selected_point_size_(DEFAULT_POINT_HIGHLIGHT_SIZE_),
  select_translate_x_(0), select_translate_y_(0), select_translate_z_(0)
{
  std::fill_n(center_xyz_, XYZ_SIZE, 0.0f);
  setIdentity(cloud_matrix_);
  setIdentity(select_matrix_);
  color_[RED] = DEFAULT_POINT_DISPLAY_COLOR_RED_;
  color_[GREEN] = DEFAULT_POINT_DISPLAY_COLOR_GREEN_;
  color_[BLUE] = DEFAULT_POINT_DISPLAY_COLOR_BLUE_;
  highlight_color_[RED] = DEFAULT_POINT_HIGHLIGHT_COLOR_RED_;
  highlight_color_[GREEN] = DEFAULT_POINT_HIGHLIGHT_COLOR_GREEN_;
  highlight_color_[BLUE] = DEFAULT_POINT_HIGHLIGHT_COLOR_BLUE_;
  updateCloudMembers();
  if (register_stats)
    registerStats();
}

Cloud::Cloud (const Cloud &copy)
  : cloud_(copy.cloud_), selection_wk_ptr_(copy.selection_wk_ptr_),
  use_color_ramp_(copy.use_color_ramp_),
  color_ramp_axis_(copy.color_ramp_axis_),
  display_scale_(copy.display_scale_),
  point_size_(copy.point_size_),
  selected_point_size_(copy.selected_point_size_),
  select_translate_x_(copy.select_translate_x_),
  select_translate_y_(copy.select_translate_y_),
  select_translate_z_(copy.select_translate_z_),
  partitioned_indices_(copy.partitioned_indices_)
{
  std::copy(copy.center_xyz_, copy.center_xyz_+XYZ_SIZE, center_xyz_);
  std::copy(copy.cloud_matrix_, copy.cloud_matrix_+MATRIX_SIZE, cloud_matrix_);
  std::copy(copy.select_matrix_, copy.select_matrix_+MATRIX_SIZE,
            select_matrix_);
  std::copy(copy.color_, copy.color_+RGB, color_);
  std::copy(copy.highlight_color_, copy.highlight_color_+RGB, highlight_color_);
}

Cloud::~Cloud () {}

Cloud&
Cloud::operator= (const Cloud &cloud)
{
  cloud_ = cloud.cloud_;
  selection_wk_ptr_ = cloud.selection_wk_ptr_;
  use_color_ramp_ = cloud.use_color_ramp_;
  color_ramp_axis_ = cloud.color_ramp_axis_;
  display_scale_ = cloud.display_scale_;
  point_size_ = cloud.point_size_;
  selected_point_size_ = cloud.selected_point_size_;
  std::copy(cloud.center_xyz_, cloud.center_xyz_+XYZ_SIZE, center_xyz_);
  std::copy(cloud.cloud_matrix_, cloud.cloud_matrix_+MATRIX_SIZE,
            cloud_matrix_);
  std::copy(cloud.select_matrix_, cloud.select_matrix_+MATRIX_SIZE,
            select_matrix_);
  partitioned_indices_ = cloud.partitioned_indices_;
  std::copy(cloud.color_, cloud.color_+RGB, color_);
  std::copy(cloud.highlight_color_,cloud.highlight_color_+RGB,highlight_color_);
  select_translate_x_ = cloud.select_translate_x_;
  select_translate_y_ = cloud.select_translate_y_;
  select_translate_z_ = cloud.select_translate_z_;
  return (*this);
}

Point3D&
Cloud::operator[] (unsigned int index)
{
  assert(index < cloud_.size());
  return (cloud_[index]);
}

const Point3D&
Cloud::operator[] (unsigned int index) const
{
  assert(index < cloud_.size());
  return (cloud_[index]);
}

void
Cloud::loadMatrix (const float *matrix)
{
  std::copy(matrix, matrix+MATRIX_SIZE, cloud_matrix_);
}

void
Cloud::multMatrix (const float *matrix)
{
  ::multMatrix(cloud_matrix_, matrix, cloud_matrix_);
}

void
Cloud::setSelectionRotation (const float* matrix)
{
  std::copy(matrix, matrix+MATRIX_SIZE, select_matrix_);
}

void
Cloud::setSelectionTranslation (float dx, float dy, float dz)
{
  select_translate_x_ = dx;
  select_translate_y_ = dy;
  select_translate_z_ = dz;
}

void
Cloud::setSelection (SelectionPtr selection_ptr)
{
  selection_wk_ptr_ = selection_ptr;
  if (!selection_ptr || selection_ptr->empty())
    return;
  IncIndex inc;
  partitioned_indices_.resize(cloud_.size());
  std::generate(partitioned_indices_.begin(), partitioned_indices_.end(), inc);
  unsigned int pos = 0;
  Selection::const_iterator it;
  // assumes selection is sorted small to large
  for (it = selection_ptr->begin(); it != selection_ptr->end(); ++it, ++pos)
  {
    std::swap(partitioned_indices_[pos], partitioned_indices_[*it]);
  }
}

void
Cloud::setRGB (float r, float g, float b)
{
  color_[RED] = r;
  color_[GREEN] = g;
  color_[BLUE] = b;
}

void
Cloud::setHighlightColor (float r, float g, float b)
{
  highlight_color_[RED] = r;
  highlight_color_[GREEN] = g;
  highlight_color_[BLUE] = b;
}

void
Cloud::drawWithTexture () const
{
  enableTexture();
  draw();
  disableTexture();
}

void
Cloud::drawWithRGB () const
{
  glEnableClientState(GL_COLOR_ARRAY);
  glColorPointer(3, GL_UNSIGNED_BYTE, sizeof(Point3D),
                 &(cloud_.points[0].b));
  draw();
}

void
Cloud::drawWithPureColor () const
{
  glDisableClientState(GL_COLOR_ARRAY);
  glColor3fv(color_);
  draw();
}

void
Cloud::drawWithHighlightColor () const
{
  glDisableClientState(GL_COLOR_ARRAY);
  glDisable(GL_TEXTURE_1D);
  glColor3fv(highlight_color_);
  draw();
}

void
Cloud::draw (bool disable_highlight) const
{
  SelectionPtr selection_ptr = selection_wk_ptr_.lock();

  glPushAttrib(GL_CURRENT_BIT | GL_POINT_BIT | GL_COLOR_BUFFER_BIT);
  {
    glPointSize(point_size_);
    glPushClientAttrib(GL_CLIENT_VERTEX_ARRAY_BIT);
    {
      glPushMatrix();
      {
        glLoadIdentity();
        glTranslatef(0.0f, 0.0f, DISPLAY_Z_TRANSLATION);
        glScalef(display_scale_, display_scale_, display_scale_);
        glMultMatrixf(cloud_matrix_);
        glTranslatef(-center_xyz_[0], -center_xyz_[1], -center_xyz_[2]);

        glEnableClientState(GL_VERTEX_ARRAY);
        glVertexPointer(3, GL_FLOAT, sizeof(Point3D), &(cloud_.points[0].x));

        if (disable_highlight || (!selection_ptr) || selection_ptr->empty())
        {
          // draw the entire cloud
          glDrawArrays(GL_POINTS, 0, cloud_.size());
        }
        else
        {
          // draw the unselected points
          glDrawElements(GL_POINTS, cloud_.size()-selection_ptr->size(),
                         GL_UNSIGNED_INT,
                         (&(partitioned_indices_[selection_ptr->size()-1]))+1);

          // handle selection transformation
          glLoadIdentity();
          glTranslatef(0.0f, 0.0f, DISPLAY_Z_TRANSLATION);
          glScalef(display_scale_, display_scale_, display_scale_);
          glTranslatef(select_translate_x_,
                       select_translate_y_, select_translate_z_);
          glMultMatrixf(select_matrix_);
          glMultMatrixf(cloud_matrix_);
          glTranslatef(-center_xyz_[0], -center_xyz_[1], -center_xyz_[2]);


          // set up highlight display
          glDisable(GL_TEXTURE_1D);
          glDisableClientState(GL_COLOR_ARRAY);
          glColor3fv(highlight_color_);
          glPointSize(selected_point_size_);
          glBlendFunc( GL_SRC_ALPHA, GL_ZERO );

          // draw the selected points
          glDrawElements(GL_POINTS, selection_ptr->size(), GL_UNSIGNED_INT,
                         &(partitioned_indices_[0]));
        }
      }
      glPopMatrix();
    }
    glPopClientAttrib();
  }
  glPopAttrib();
}

void
Cloud::append (const Point3D &pt)
{
  cloud_.push_back(pt);
}

void
Cloud::append (const Cloud & cloud)
{
  cloud_ += cloud.cloud_;
}

void
Cloud::remove(const Selection& selection)
{
  unsigned int pos = cloud_.size();
  Selection::const_reverse_iterator rit;
  for (rit = selection.rbegin(); rit != selection.rend(); ++rit)
    std::swap(cloud_.points[--pos], cloud_.points[*rit]);
  resize(cloud_.size()-selection.size());
}

void
Cloud::resize(unsigned int new_size)
{
  cloud_.resize(new_size);
  cloud_.width = new_size;
  cloud_.height = 1;
}

void
Cloud::clear ()
{
  cloud_.clear();
}

void
Cloud::setPointSize (int size)
{
  point_size_ = size;
}

void
Cloud::setHighlightPointSize (int size)
{
  selected_point_size_ = size;
}

Point3D
Cloud::getObjectSpacePoint (unsigned int index) const
{
  Point3D pt = cloud_[index];
  float x, y, z;
  pt.x -= center_xyz_[0];
  pt.y -= center_xyz_[1];
  pt.z -= center_xyz_[2];
  x = cloud_matrix_[0] * pt.x +
      cloud_matrix_[4] * pt.y +
      cloud_matrix_[8] * pt.z +
      cloud_matrix_[12];
  y = cloud_matrix_[1] * pt.x +
      cloud_matrix_[5] * pt.y +
      cloud_matrix_[9] * pt.z +
      cloud_matrix_[13];
  z = cloud_matrix_[2] * pt.x +
      cloud_matrix_[6] * pt.y +
      cloud_matrix_[10] * pt.z +
      cloud_matrix_[14];
  pt.x = x;
  pt.y = y;
  pt.z = z;

  return (pt);
}

Point3D
Cloud::getDisplaySpacePoint (unsigned int index) const
{
  Point3D pt = cloud_[index];
  float x, y, z;
  pt.x -= center_xyz_[0];
  pt.y -= center_xyz_[1];
  pt.z -= center_xyz_[2];
  x = cloud_matrix_[0] * pt.x +
      cloud_matrix_[4] * pt.y +
      cloud_matrix_[8] * pt.z +
      cloud_matrix_[12];
  y = cloud_matrix_[1] * pt.x +
      cloud_matrix_[5] * pt.y +
      cloud_matrix_[9] * pt.z +
      cloud_matrix_[13];
  z = cloud_matrix_[2] * pt.x +
      cloud_matrix_[6] * pt.y +
      cloud_matrix_[10] * pt.z +
      cloud_matrix_[14];
  pt.x = x * display_scale_;
  pt.y = y * display_scale_;
  pt.z = z * display_scale_;
  pt.z += DISPLAY_Z_TRANSLATION;

  return (pt);
}

void
Cloud::getDisplaySpacePoints (Point3DVector& pts) const
{
  for(unsigned int i = 0; i < cloud_.size(); ++i)
    pts.push_back(getDisplaySpacePoint(i));
}

const Cloud3D&
Cloud::getInternalCloud () const
{
  return (cloud_);
}

void
Cloud::restore (const CopyBuffer& copy_buffer, const Selection& selection)
{

  if (selection.empty())
    return;
  const Cloud& copied_cloud = copy_buffer.get();
  if (copied_cloud.size() != selection.size())
    return;

  append(copied_cloud);
  unsigned int pos = cloud_.size();
  Selection::const_reverse_iterator rit;
  for (rit = selection.rbegin(); rit != selection.rend(); ++rit)
    std::swap(cloud_.points[--pos], cloud_.points[*rit]);
}

std::string
Cloud::getStat () const
{
  std::string title = "Total number of points: ";
  std::string num_str;
  ::toString(cloud_.size(), num_str);
  return (title + num_str);
}

void
Cloud::updateCloudMembers ()
{
  if (cloud_.empty())
      return;

  std::fill_n(min_xyz_, XYZ_SIZE, 0.0f);
  std::fill_n(max_xyz_, XYZ_SIZE, 0.0f);
  float *pt = &(cloud_.points[0].data[X]);
  std::copy(pt, pt+XYZ_SIZE, max_xyz_);
  std::copy(max_xyz_, max_xyz_+XYZ_SIZE, min_xyz_);
  for (unsigned int i = 1; i < cloud_.size(); ++i)
  {
    for (unsigned int j = 0; j < XYZ_SIZE; ++j)
    {
      min_xyz_[j] = std::min(min_xyz_[j], cloud_.points[i].data[j]);
      max_xyz_[j] = std::max(max_xyz_[j], cloud_.points[i].data[j]);
    }
  }
  float range = 0.0f;
  for (unsigned int j = 0; j < XYZ_SIZE; ++j)
  {
    range = std::max(range, max_xyz_[j] - min_xyz_[j]);
    center_xyz_[j] = 0.5f * (max_xyz_[j] + min_xyz_[j]);
  }
  display_scale_ = 1.0f / range;
}

void
Cloud::enableTexture () const
{
  if (!use_color_ramp_)
    return;
  float ranges[3] ={max_xyz_[0] - min_xyz_[0],
                    max_xyz_[1] - min_xyz_[1],
                    max_xyz_[2] - min_xyz_[2]};
  float transvals[3] = {-min_xyz_[0], -min_xyz_[1], -min_xyz_[2]};
  float range = ranges[color_ramp_axis_];
  float transval = transvals[color_ramp_axis_];
  glEnable(GL_TEXTURE_1D);
  glEnableClientState(GL_TEXTURE_COORD_ARRAY);
  glTexCoordPointer(1, GL_FLOAT, sizeof(Point3D),
                    &(cloud_.points[0].data[color_ramp_axis_]));
  glMatrixMode(GL_TEXTURE);
  glPushMatrix();
  glLoadIdentity();
  if (range <= 0.0f)
    range = 1.0f;
  glScalef(1.0f/range, 1.0f, 1.0f);
  glTranslatef(transval, 0.0f, 0.0f);
  glMatrixMode(GL_MODELVIEW);
}

void
Cloud::disableTexture () const
{
  if (!use_color_ramp_)
    return;
  glMatrixMode(GL_TEXTURE);
  glPopMatrix();
  glDisable(GL_TEXTURE_1D);
  glMatrixMode(GL_MODELVIEW);
}
