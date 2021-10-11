/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#pragma once

#include <set>
#include <map>

#include <pcl/pcl_macros.h>
#include <pcl/common/colors.h>
#include <pcl/common/io.h> // for getFieldIndex
#include <pcl/common/point_tests.h> // for pcl::isFinite


namespace pcl
{

namespace visualization
{

template <typename PointT> vtkSmartPointer<vtkDataArray>
PointCloudColorHandlerCustom<PointT>::getColor () const
{
  if (!capable_ || !cloud_)
    return nullptr;

  auto scalars = vtkSmartPointer<vtkUnsignedCharArray>::New ();
  scalars->SetNumberOfComponents (3);

  vtkIdType nr_points = cloud_->size ();
  scalars->SetNumberOfTuples (nr_points);

  // Get a random color
  unsigned char* colors = new unsigned char[nr_points * 3];

  // Color every point
  for (vtkIdType cp = 0; cp < nr_points; ++cp)
  {
    colors[cp * 3 + 0] = static_cast<unsigned char> (r_);
    colors[cp * 3 + 1] = static_cast<unsigned char> (g_);
    colors[cp * 3 + 2] = static_cast<unsigned char> (b_);
  }
  scalars->SetArray (colors, 3 * nr_points, 0, vtkUnsignedCharArray::VTK_DATA_ARRAY_DELETE);
  return scalars;
}


template <typename PointT> vtkSmartPointer<vtkDataArray>
PointCloudColorHandlerRandom<PointT>::getColor () const
{
  if (!capable_ || !cloud_)
    return nullptr;

  auto scalars = vtkSmartPointer<vtkUnsignedCharArray>::New ();
  scalars->SetNumberOfComponents (3);

  vtkIdType nr_points = cloud_->size ();
  scalars->SetNumberOfTuples (nr_points);

  // Get a random color
  unsigned char* colors = new unsigned char[nr_points * 3];
  double r, g, b;
  pcl::visualization::getRandomColors (r, g, b);

  int r_ = static_cast<int> (pcl_lrint (r * 255.0)),
      g_ = static_cast<int> (pcl_lrint (g * 255.0)),
      b_ = static_cast<int> (pcl_lrint (b * 255.0));

  // Color every point
  for (vtkIdType cp = 0; cp < nr_points; ++cp)
  {
    colors[cp * 3 + 0] = static_cast<unsigned char> (r_);
    colors[cp * 3 + 1] = static_cast<unsigned char> (g_);
    colors[cp * 3 + 2] = static_cast<unsigned char> (b_);
  }
  scalars->SetArray (colors, 3 * nr_points, 0, vtkUnsignedCharArray::VTK_DATA_ARRAY_DELETE);
  return scalars;
}


template <typename PointT> void
PointCloudColorHandlerRGBField<PointT>::setInputCloud (
    const PointCloudConstPtr &cloud)
{
  PointCloudColorHandler<PointT>::setInputCloud (cloud);
  // Handle the 24-bit packed RGB values
  field_idx_ = pcl::getFieldIndex<PointT> ("rgb", fields_);
  if (field_idx_ != -1)
  {
    capable_ = true;
    return;
  }
  else
  {
    field_idx_ = pcl::getFieldIndex<PointT> ("rgba", fields_);
    if (field_idx_ != -1)
      capable_ = true;
    else
      capable_ = false;
  }
}


template <typename PointT> vtkSmartPointer<vtkDataArray>
PointCloudColorHandlerRGBField<PointT>::getColor () const
{
  if (!capable_ || !cloud_)
    return nullptr;

   // Get the RGB field index
  std::vector<pcl::PCLPointField> fields;
  int rgba_index = -1;
  rgba_index = pcl::getFieldIndex<PointT> ("rgb", fields);
  if (rgba_index == -1)
    rgba_index = pcl::getFieldIndex<PointT> ("rgba", fields);

  int rgba_offset = fields[rgba_index].offset;

  auto scalars = vtkSmartPointer<vtkUnsignedCharArray>::New ();
  scalars->SetNumberOfComponents (3);

  vtkIdType nr_points = cloud_->size ();
  scalars->SetNumberOfTuples (nr_points);
  unsigned char* colors = scalars->GetPointer (0);

  // If XYZ present, check if the points are invalid
  int x_idx = -1;
  for (std::size_t d = 0; d < fields_.size (); ++d)
    if (fields_[d].name == "x")
      x_idx = static_cast<int> (d);

  pcl::RGB rgb;
  if (x_idx != -1)
  {
    int j = 0;
    // Color every point
    for (vtkIdType cp = 0; cp < nr_points; ++cp)
    {
      // Copy the value at the specified field
      if (!std::isfinite ((*cloud_)[cp].x) ||
          !std::isfinite ((*cloud_)[cp].y) ||
          !std::isfinite ((*cloud_)[cp].z))
        continue;

      memcpy (&rgb, (reinterpret_cast<const char *> (&(*cloud_)[cp])) + rgba_offset, sizeof (pcl::RGB));
      colors[j    ] = rgb.r;
      colors[j + 1] = rgb.g;
      colors[j + 2] = rgb.b;
      j += 3;
    }
  }
  else
  {
    // Color every point
    for (vtkIdType cp = 0; cp < nr_points; ++cp)
    {
      int idx = static_cast<int> (cp) * 3;
      memcpy (&rgb, (reinterpret_cast<const char *> (&(*cloud_)[cp])) + rgba_offset, sizeof (pcl::RGB));
      colors[idx    ] = rgb.r;
      colors[idx + 1] = rgb.g;
      colors[idx + 2] = rgb.b;
    }
  }
  return scalars;
}


template <typename PointT>
PointCloudColorHandlerHSVField<PointT>::PointCloudColorHandlerHSVField (const PointCloudConstPtr &cloud) :
  PointCloudColorHandler<PointT>::PointCloudColorHandler (cloud)
{
  // Check for the presence of the "H" field
  field_idx_ = pcl::getFieldIndex<PointT> ("h", fields_);
  if (field_idx_ == -1)
  {
    capable_ = false;
    return;
  }

  // Check for the presence of the "S" field
  s_field_idx_ = pcl::getFieldIndex<PointT> ("s", fields_);
  if (s_field_idx_ == -1)
  {
    capable_ = false;
    return;
  }

  // Check for the presence of the "V" field
  v_field_idx_ = pcl::getFieldIndex<PointT> ("v", fields_);
  if (v_field_idx_ == -1)
  {
    capable_ = false;
    return;
  }
  capable_ = true;
}


template <typename PointT> vtkSmartPointer<vtkDataArray>
PointCloudColorHandlerHSVField<PointT>::getColor () const
{
  if (!capable_ || !cloud_)
    return nullptr;

  auto scalars = vtkSmartPointer<vtkUnsignedCharArray>::New ();
  scalars->SetNumberOfComponents (3);

  vtkIdType nr_points = cloud_->size ();
  scalars->SetNumberOfTuples (nr_points);
  unsigned char* colors = scalars->GetPointer (0);

  int idx = 0;
  // If XYZ present, check if the points are invalid
  int x_idx = -1;

  for (std::size_t d = 0; d < fields_.size (); ++d)
    if (fields_[d].name == "x")
      x_idx = static_cast<int> (d);

  if (x_idx != -1)
  {
    // Color every point
    for (vtkIdType cp = 0; cp < nr_points; ++cp)
    {
      // Copy the value at the specified field
      if (!std::isfinite ((*cloud_)[cp].x) ||
          !std::isfinite ((*cloud_)[cp].y) ||
          !std::isfinite ((*cloud_)[cp].z))
        continue;

      ///@todo do this with the point_types_conversion in common, first template it!

      float h = (*cloud_)[cp].h;
      float v = (*cloud_)[cp].v;
      float s = (*cloud_)[cp].s;

      // Fill color data with HSV here:
      // restrict the hue value to [0,360[
      h = h < 0.0f ? h - (((int)h)/360 - 1)*360 : h - (((int)h)/360)*360;

      // restrict s and v to [0,1]
      if (s > 1.0f) s = 1.0f;
      if (s < 0.0f) s = 0.0f;
      if (v > 1.0f) v = 1.0f;
      if (v < 0.0f) v = 0.0f;

      if (s == 0.0f)
      {
        colors[idx] = colors[idx+1] = colors[idx+2] = v*255;
      }
      else
      {
        // calculate p, q, t from HSV-values
        float a = h / 60;
        int   i = std::floor (a);
        float f = a - i;
        float p = v * (1 - s);
        float q = v * (1 - s * f);
        float t = v * (1 - s * (1 - f));

        switch (i)
        {
          case 0:
            colors[idx] = v*255; colors[idx+1] = t*255; colors[idx+2] = p*255; break;
          case 1:
            colors[idx] = q*255; colors[idx+1] = v*255; colors[idx+2] = p*255; break;
          case 2:
            colors[idx] = p*255; colors[idx+1] = v*255; colors[idx+2] = t*255; break;
          case 3:
            colors[idx] = p*255; colors[idx+1] = q*255; colors[idx+2] = v*255; break;
          case 4:
            colors[idx] = t*255; colors[idx+1] = p*255; colors[idx+2] = v*255; break;
          case 5:
            colors[idx] = v*255; colors[idx+1] = p*255; colors[idx+2] = q*255; break;
        }
      }
      idx +=3;
    }
  }
  else
  {
    // Color every point
    for (vtkIdType cp = 0; cp < nr_points; ++cp)
    {
      float h = (*cloud_)[cp].h;
      float v = (*cloud_)[cp].v;
      float s = (*cloud_)[cp].s;

      // Fill color data with HSV here:
      // restrict the hue value to [0,360[
      h = h < 0.0f ? h - (((int)h)/360 - 1)*360 : h - (((int)h)/360)*360;

      // restrict s and v to [0,1]
      if (s > 1.0f) s = 1.0f;
      if (s < 0.0f) s = 0.0f;
      if (v > 1.0f) v = 1.0f;
      if (v < 0.0f) v = 0.0f;

      if (s == 0.0f)
      {
        colors[idx] = colors[idx+1] = colors[idx+2] = v*255;
      }
      else
      {
        // calculate p, q, t from HSV-values
        float a = h / 60;
        int   i = std::floor (a);
        float f = a - i;
        float p = v * (1 - s);
        float q = v * (1 - s * f);
        float t = v * (1 - s * (1 - f));

        switch (i)
        {
          case 0:
            colors[idx] = v*255; colors[idx+1] = t*255; colors[idx+2] = p*255; break;
          case 1:
            colors[idx] = q*255; colors[idx+1] = v*255; colors[idx+2] = p*255; break;
          case 2:
            colors[idx] = p*255; colors[idx+1] = v*255; colors[idx+2] = t*255; break;
          case 3:
            colors[idx] = p*255; colors[idx+1] = q*255; colors[idx+2] = v*255; break;
          case 4:
            colors[idx] = t*255; colors[idx+1] = p*255; colors[idx+2] = v*255; break;
          case 5:
            colors[idx] = v*255; colors[idx+1] = p*255; colors[idx+2] = q*255; break;
        }
      }
      idx +=3;
    }
  }
  return scalars;
}


template <typename PointT> void
PointCloudColorHandlerGenericField<PointT>::setInputCloud (
    const PointCloudConstPtr &cloud)
{
  PointCloudColorHandler<PointT>::setInputCloud (cloud);
  field_idx_  = pcl::getFieldIndex<PointT> (field_name_, fields_);
  if (field_idx_ != -1)
    capable_ = true;
  else
    capable_ = false;
}


template <typename PointT> vtkSmartPointer<vtkDataArray>
PointCloudColorHandlerGenericField<PointT>::getColor () const
{
  if (!capable_ || !cloud_)
    return nullptr;

  auto scalars = vtkSmartPointer<vtkFloatArray>::New ();
  scalars->SetNumberOfComponents (1);

  vtkIdType nr_points = cloud_->size ();
  scalars->SetNumberOfTuples (nr_points);

  using FieldList = typename pcl::traits::fieldList<PointT>::type;

  float* colors = new float[nr_points];
  float field_data;

  int j = 0;
  // If XYZ present, check if the points are invalid
  int x_idx = -1;
  for (std::size_t d = 0; d < fields_.size (); ++d)
    if (fields_[d].name == "x")
      x_idx = static_cast<int> (d);

  if (x_idx != -1)
  {
    // Color every point
    for (vtkIdType cp = 0; cp < nr_points; ++cp)
    {
      // Copy the value at the specified field
      if (!std::isfinite ((*cloud_)[cp].x) || !std::isfinite ((*cloud_)[cp].y) || !std::isfinite ((*cloud_)[cp].z))
        continue;

      const std::uint8_t* pt_data = reinterpret_cast<const std::uint8_t*> (&(*cloud_)[cp]);
      memcpy (&field_data, pt_data + fields_[field_idx_].offset, pcl::getFieldSize (fields_[field_idx_].datatype));

      colors[j] = field_data;
      j++;
    }
  }
  else
  {
    // Color every point
    for (vtkIdType cp = 0; cp < nr_points; ++cp)
    {
      const std::uint8_t* pt_data = reinterpret_cast<const std::uint8_t*> (&(*cloud_)[cp]);
      memcpy (&field_data, pt_data + fields_[field_idx_].offset, pcl::getFieldSize (fields_[field_idx_].datatype));

      if (!std::isfinite (field_data))
        continue;

      colors[j] = field_data;
      j++;
    }
  }
  scalars->SetArray (colors, j, 0, vtkFloatArray::VTK_DATA_ARRAY_DELETE);
  return scalars;
}


template <typename PointT> void
PointCloudColorHandlerRGBAField<PointT>::setInputCloud (
    const PointCloudConstPtr &cloud)
{
  PointCloudColorHandler<PointT>::setInputCloud (cloud);
  // Handle the 24-bit packed RGBA values
  field_idx_ = pcl::getFieldIndex<PointT> ("rgba", fields_);
  if (field_idx_ != -1)
    capable_ = true;
  else
    capable_ = false;
}


template <typename PointT> vtkSmartPointer<vtkDataArray>
PointCloudColorHandlerRGBAField<PointT>::getColor () const
{
  if (!capable_ || !cloud_)
    return nullptr;

  auto scalars = vtkSmartPointer<vtkUnsignedCharArray>::New ();
  scalars->SetNumberOfComponents (4);

  vtkIdType nr_points = cloud_->size ();
  scalars->SetNumberOfTuples (nr_points);
  unsigned char* colors = scalars->GetPointer (0);

  // If XYZ present, check if the points are invalid
  int x_idx = -1;
  for (std::size_t d = 0; d < fields_.size (); ++d)
    if (fields_[d].name == "x")
      x_idx = static_cast<int> (d);

  if (x_idx != -1)
  {
    int j = 0;
    // Color every point
    for (vtkIdType cp = 0; cp < nr_points; ++cp)
    {
      // Copy the value at the specified field
      if (!std::isfinite ((*cloud_)[cp].x) ||
          !std::isfinite ((*cloud_)[cp].y) ||
          !std::isfinite ((*cloud_)[cp].z))
        continue;

      colors[j    ] = (*cloud_)[cp].r;
      colors[j + 1] = (*cloud_)[cp].g;
      colors[j + 2] = (*cloud_)[cp].b;
      colors[j + 3] = (*cloud_)[cp].a;
      j += 4;
    }
  }
  else
  {
    // Color every point
    for (vtkIdType cp = 0; cp < nr_points; ++cp)
    {
      int idx = static_cast<int> (cp) * 4;
      colors[idx    ] = (*cloud_)[cp].r;
      colors[idx + 1] = (*cloud_)[cp].g;
      colors[idx + 2] = (*cloud_)[cp].b;
      colors[idx + 3] = (*cloud_)[cp].a;
    }
  }
  return scalars;
}


template <typename PointT> void
PointCloudColorHandlerLabelField<PointT>::setInputCloud (const PointCloudConstPtr &cloud)
{
  PointCloudColorHandler<PointT>::setInputCloud (cloud);
  field_idx_ = pcl::getFieldIndex<PointT> ("label", fields_);
  if (field_idx_ != -1)
  {
    capable_ = true;
    return;
  }
}


template <typename PointT> vtkSmartPointer<vtkDataArray>
PointCloudColorHandlerLabelField<PointT>::getColor () const
{
  if (!capable_ || !cloud_)
    return nullptr;

  auto scalars = vtkSmartPointer<vtkUnsignedCharArray>::New ();
  scalars->SetNumberOfComponents (3);

  vtkIdType nr_points = cloud_->size ();
  scalars->SetNumberOfTuples (nr_points);
  unsigned char* colors = scalars->GetPointer (0);


  std::map<std::uint32_t, pcl::RGB> colormap;
  if (!static_mapping_)
  {
    std::set<std::uint32_t> labels;
    // First pass: find unique labels
    for (vtkIdType i = 0; i < nr_points; ++i)
      labels.insert ((*cloud_)[i].label);

    // Assign Glasbey colors in ascending order of labels
    std::size_t color = 0;
    for (std::set<std::uint32_t>::iterator iter = labels.begin (); iter != labels.end (); ++iter, ++color)
      colormap[*iter] = GlasbeyLUT::at (color % GlasbeyLUT::size ());
  }

  int j = 0;
  for (vtkIdType cp = 0; cp < nr_points; ++cp)
  {
    if (pcl::isFinite ((*cloud_)[cp]))
    {
      const pcl::RGB& color = static_mapping_ ? GlasbeyLUT::at ((*cloud_)[cp].label % GlasbeyLUT::size ()) : colormap[(*cloud_)[cp].label];
      colors[j    ] = color.r;
      colors[j + 1] = color.g;
      colors[j + 2] = color.b;
      j += 3;
    }
  }

  return scalars;
}

} // namespace visualization
} // namespace pcl

