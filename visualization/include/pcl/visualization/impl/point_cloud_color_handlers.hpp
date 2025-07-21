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
#include <stdexcept>

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
      if (!pcl::isXYZFinite((*cloud_)[cp]))
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
      if (!pcl::isXYZFinite((*cloud_)[cp]))
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
      if (!pcl::isXYZFinite((*cloud_)[cp]))
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

// ----
// template specializations for PointCloud and PCLPointCloud2 to access certain
// data from PointCloudColorHandlerGenericField without needing to know the
// cloud type

// Get point fields from cloud. Could not get it to work with existing
// pcl::getFields
template <typename CloudT> inline std::vector<pcl::PCLPointField>
getFields(const CloudT& cloud)
{
  return pcl::getFields<typename CloudT::PointType>();
}

template <> inline std::vector<pcl::PCLPointField>
getFields<pcl::PCLPointCloud2>(const pcl::PCLPointCloud2& cloud) {
  return cloud.fields;
}


// Get point step. Does not directly exist in pcl::PointCloud
template <typename CloudT> inline int getPointStep(const CloudT&)
{
  return sizeof(typename CloudT::PointType);
}

template <> inline int
getPointStep<pcl::PCLPointCloud2>(const pcl::PCLPointCloud2& cloud) {
  return cloud.point_step;
}

// Get cloud data blob
template <typename CloudT> inline const std::uint8_t* getCloudData(const CloudT& cloud)
{
  return reinterpret_cast<const std::uint8_t*>(cloud.points.data());
}

template <> inline const std::uint8_t* getCloudData<pcl::PCLPointCloud2>(const typename pcl::PCLPointCloud2& cloud) {
  return reinterpret_cast<const std::uint8_t*>(cloud.data.data());
}


// copy of pcl::getFieldIndex() from impl/io.hpp, without the unused template
// parameter
static int getFieldIndex(const std::string& field_name,
              const std::vector<pcl::PCLPointField>& fields)
{
  const auto result =
      std::find_if(fields.begin(), fields.end(), [&field_name](const auto& field) {
        return field.name == field_name;
      });
  if (result == fields.end()) {
    return -1;
  }
  return std::distance(fields.begin(), result);
}

// Cloud type agnostic isXYZFinite wrappers to check if pointcloud or PCLPointCloud2 at
// given index is finite
template <typename CloudT> inline bool isXYZFiniteAt(const CloudT& cloud, int index)
{
  return pcl::isXYZFinite(cloud.at(index));
}

template <> inline bool isXYZFiniteAt(const PCLPointCloud2& cloud, int index)
{
  // get x,y,z field indices
  const auto x_field_idx = getFieldIndex("x", cloud.fields);
  const auto y_field_idx = getFieldIndex("y", cloud.fields);
  const auto z_field_idx = getFieldIndex("z", cloud.fields);

  // if any missing, error
  if (x_field_idx == -1 || y_field_idx == -1 || z_field_idx == -1) {
    throw std::out_of_range("getXData(): input cloud missing at least one of x, y, z fields");
  }
  // get x,y,z field values
  const auto position_x = index * cloud.point_step + cloud.fields[x_field_idx].offset;
  const auto position_y = index * cloud.point_step + cloud.fields[y_field_idx].offset;
  const auto position_z = index * cloud.point_step + cloud.fields[z_field_idx].offset;
  if (cloud.data.size () >= (position_x + sizeof(float)) &&
      cloud.data.size () >= (position_y + sizeof(float)) &&
      cloud.data.size () >= (position_z + sizeof(float))) {
    const float x = *reinterpret_cast<const float*>(cloud.data[position_x]);
    const float y = *reinterpret_cast<const float*>(cloud.data[position_y]);
    const float z = *reinterpret_cast<const float*>(cloud.data[position_z]);
    return isXYZFinite(PointXYZ(x, y, z));
  } else {
    // the last of the three is out of range
    throw std::out_of_range("getXData(): requested for index larger than number of points");
  }
}

inline const std::uint8_t* getCloudData(const typename pcl::PCLPointCloud2& cloud)
{
  return reinterpret_cast<const std::uint8_t*>(cloud.data.data());
}

template <typename DType, typename RType> RType reinterpret_and_cast(const std::uint8_t* p)
{
  return static_cast<RType>(*reinterpret_cast<const DType*>(p));
}

/**
 * @brief   Get the value of a point field from raw data pointer and field type.
 *
 * @tparam T    return type the field will be cast as
 * @param data  data pointer
 * @param type  point field type
 *
 * @return  field value
 */
template <typename T> T point_field_as(const std::uint8_t* data, const std::uint8_t type)
{
  switch (type) {
  case pcl::PCLPointField::PointFieldTypes::FLOAT32:
    return reinterpret_and_cast<float, T>(data);
    break;
  case pcl::PCLPointField::PointFieldTypes::UINT8:
    return reinterpret_and_cast<std::uint8_t, T>(data);
    break;
  case pcl::PCLPointField::PointFieldTypes::UINT16:
    return reinterpret_and_cast<std::uint16_t, T>(data);
    break;
  case pcl::PCLPointField::PointFieldTypes::UINT32:
    return reinterpret_and_cast<std::uint32_t, T>(data);
    break;
  case pcl::PCLPointField::PointFieldTypes::UINT64:
    return reinterpret_and_cast<std::uint64_t, T>(data);
    break;
  case pcl::PCLPointField::PointFieldTypes::BOOL:
    return reinterpret_and_cast<bool, T>(data);
    break;
  case pcl::PCLPointField::PointFieldTypes::FLOAT64:
    return reinterpret_and_cast<double, T>(data);
    break;
  case pcl::PCLPointField::PointFieldTypes::INT16:
    return reinterpret_and_cast<std::int16_t, T>(data);
    break;
  case pcl::PCLPointField::PointFieldTypes::INT32:
    return reinterpret_and_cast<std::int32_t, T>(data);
    break;
  case pcl::PCLPointField::PointFieldTypes::INT64:
    return reinterpret_and_cast<std::int64_t, T>(data);
    break;
  default:
    return 0;
    break;
  }
}

template <typename PointT>
PointCloudColorHandlerGenericField<PointT>::PointCloudColorHandlerGenericField(
    const PointCloudConstPtr& cloud, const std::string& field_name)
: PointCloudColorHandler<PointT>(cloud), field_name_(field_name)
{
  this->setInputCloud(cloud);
}

template <typename PointT>
PointCloudColorHandlerGenericField<PointT>::PointCloudColorHandlerGenericField(const std::string& field_name)
  : PointCloudColorHandler<PointT>(), field_name_(field_name) {}

template <typename PointT> void PointCloudColorHandlerGenericField<PointT>::setInputCloud(
    const PointCloudConstPtr& cloud)
{
  PointCloudColorHandler<PointT>::setInputCloud(cloud);
  this->fields_ = getFields(*cloud);
  this->field_idx_ = getFieldIndex(field_name_, this->fields_);
  this->capable_ = this->field_idx_ != -1;
}

template <typename PointT> std::string PointCloudColorHandlerGenericField<PointT>::getFieldName() const
{
  return field_name_;
}

template <typename PointT> vtkSmartPointer<vtkDataArray> PointCloudColorHandlerGenericField<PointT>::getColor() const
{
  if (!this->capable_ || !this->cloud_) {
    return nullptr;
  }

  auto scalars = vtkSmartPointer<vtkFloatArray>::New();
  scalars->SetNumberOfComponents(1);

  const vtkIdType nr_points = this->cloud_->width * this->cloud_->height;
  scalars->SetNumberOfTuples(nr_points);

  // per-point color as a float value. vtk turns that into rgb somehow
  float* colors = new float[nr_points];

  // Find X channel
  int x_channel_idx = -1;
  for (std::size_t channel_idx = 0; channel_idx < this->fields_.size(); ++channel_idx) {
    if (this->fields_[channel_idx].name == "x") {
      x_channel_idx = static_cast<int>(channel_idx);
    }
  }

  int point_offset = this->fields_[this->field_idx_].offset;
  const int point_step = getPointStep<PointCloud>(*this->cloud_);
  const std::uint8_t* p_data = getCloudData<PointCloud>(*this->cloud_);
  const std::uint8_t field_type = this->fields_[this->field_idx_].datatype;

  // current index into colors array
  int pt_idx = 0;

  // Color every point
  for (vtkIdType cp = 0; cp < nr_points; ++cp, point_offset += point_step) {

    if (x_channel_idx != -1 && !isXYZFiniteAt(*this->cloud_, cp)) {
        // no x channel in the cloud, or point is infinite
        continue;
    } else {
      // point data at index, field offset already baked into point_offset
      const std::uint8_t* pt_data = &p_data[point_offset];

      colors[pt_idx] = point_field_as<float>(pt_data, field_type);

    }

    ++pt_idx;
  }

  scalars->SetArray(colors, pt_idx, 0, vtkFloatArray::VTK_DATA_ARRAY_DELETE);
  return scalars;
}

} // namespace visualization
} // namespace pcl
