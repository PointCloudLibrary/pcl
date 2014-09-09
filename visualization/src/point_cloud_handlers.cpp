/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010, Willow Garage, Inc.
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

#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/visualization/impl/point_cloud_geometry_handlers.hpp>
#include <pcl/pcl_macros.h>
#include <pcl/common/io.h>
#include <pcl/impl/instantiate.hpp>
#include <pcl/point_types.h>

///////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::PointCloudColorHandlerCustom<pcl::PCLPointCloud2>::getColor (vtkSmartPointer<vtkDataArray> &scalars) const
{
  if (!capable_ || !cloud_)
    return (false);
  
  if (!scalars)
    scalars = vtkSmartPointer<vtkUnsignedCharArray>::New ();
  scalars->SetNumberOfComponents (3);
  
  vtkIdType nr_points = cloud_->width * cloud_->height;
  reinterpret_cast<vtkUnsignedCharArray*>(&(*scalars))->SetNumberOfTuples (nr_points);
  
  // Get a random color
  unsigned char* colors = new unsigned char[nr_points * 3];
  
  // Color every point
  for (vtkIdType cp = 0; cp < nr_points; ++cp)
  {
    colors[cp * 3 + 0] = static_cast<unsigned char> (r_);
    colors[cp * 3 + 1] = static_cast<unsigned char> (g_);
    colors[cp * 3 + 2] = static_cast<unsigned char> (b_);
  }
  reinterpret_cast<vtkUnsignedCharArray*>(&(*scalars))->SetArray (colors, 3 * nr_points, 0);
  return (true);
}

///////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::PointCloudColorHandlerRandom<pcl::PCLPointCloud2>::getColor (vtkSmartPointer<vtkDataArray> &scalars) const
{
  if (!capable_ || !cloud_)
    return (false);
  
  if (!scalars)
    scalars = vtkSmartPointer<vtkUnsignedCharArray>::New ();
  scalars->SetNumberOfComponents (3);
  
  vtkIdType nr_points = cloud_->width * cloud_->height;
  reinterpret_cast<vtkUnsignedCharArray*>(&(*scalars))->SetNumberOfTuples (nr_points);
  
  // Get a random color
  unsigned char* colors = new unsigned char[nr_points * 3];
  double r, g, b;
  pcl::visualization::getRandomColors (r, g, b);
  
  long r_ = pcl_lrint (r * 255.0), g_ = pcl_lrint (g * 255.0), b_ = pcl_lrint (b * 255.0);

  // Color every point
  for (vtkIdType cp = 0; cp < nr_points; ++cp)
  {
    colors[cp * 3 + 0] = static_cast<unsigned char> (r_);
    colors[cp * 3 + 1] = static_cast<unsigned char> (g_);
    colors[cp * 3 + 2] = static_cast<unsigned char> (b_);
  }
  reinterpret_cast<vtkUnsignedCharArray*>(&(*scalars))->SetArray (colors, 3 * nr_points, 0);
  return (true);
}

///////////////////////////////////////////////////////////////////////////////////////////
pcl::visualization::PointCloudColorHandlerRGBField<pcl::PCLPointCloud2>::PointCloudColorHandlerRGBField (
    const pcl::visualization::PointCloudColorHandler<pcl::PCLPointCloud2>::PointCloudConstPtr &cloud) :
  pcl::visualization::PointCloudColorHandler<pcl::PCLPointCloud2>::PointCloudColorHandler (cloud)
{
  // Handle the 24-bit packed RGB values
  field_idx_ = pcl::getFieldIndex (*cloud, "rgb");
  if (field_idx_ != -1)
  {
    capable_ = true;
    return;
  }
  else
  {
    field_idx_ = pcl::getFieldIndex (*cloud, "rgba");
    if (field_idx_ != -1)
      capable_ = true;
    else
      capable_ = false;
  }
}

///////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::PointCloudColorHandlerRGBField<pcl::PCLPointCloud2>::getColor (vtkSmartPointer<vtkDataArray> &scalars) const
{
  if (!capable_ || !cloud_)
    return (false);

  if (!scalars)
    scalars = vtkSmartPointer<vtkUnsignedCharArray>::New ();
  scalars->SetNumberOfComponents (3);

  vtkIdType nr_points = cloud_->width * cloud_->height;
  reinterpret_cast<vtkUnsignedCharArray*>(&(*scalars))->SetNumberOfTuples (nr_points);
  // Allocate enough memory to hold all colors
  unsigned char* colors = new unsigned char[nr_points * 3];

  pcl::RGB rgb_data;
  int point_offset = cloud_->fields[field_idx_].offset;
  int j = 0;
  
  // If XYZ present, check if the points are invalid
  int x_idx = pcl::getFieldIndex (*cloud_, "x");
  if (x_idx != -1)
  {
    float x_data, y_data, z_data;
    int x_point_offset = cloud_->fields[x_idx].offset;
    
    // Color every point
    for (vtkIdType cp = 0; cp < nr_points; ++cp, 
                                           point_offset += cloud_->point_step, 
                                           x_point_offset += cloud_->point_step)
    {
      // Copy the value at the specified field
      memcpy (&rgb_data, &cloud_->data[point_offset], sizeof (float));

      memcpy (&x_data, &cloud_->data[x_point_offset], sizeof (float));
      memcpy (&y_data, &cloud_->data[x_point_offset + sizeof (float)], sizeof (float));
      memcpy (&z_data, &cloud_->data[x_point_offset + 2 * sizeof (float)], sizeof (float));

      if (!pcl_isfinite (x_data) || !pcl_isfinite (y_data) || !pcl_isfinite (z_data))
        continue;

      colors[j + 0] = rgb_data.r;
      colors[j + 1] = rgb_data.g;
      colors[j + 2] = rgb_data.b;
      j += 3;
    }
  }
  // No XYZ data checks
  else
  {
    // Color every point
    for (vtkIdType cp = 0; cp < nr_points; ++cp, point_offset += cloud_->point_step)
    {
      // Copy the value at the specified field
      memcpy (&rgb_data, &cloud_->data[point_offset], sizeof (float));

      colors[j + 0] = rgb_data.r;
      colors[j + 1] = rgb_data.g;
      colors[j + 2] = rgb_data.b;
      j += 3;
    }
  }
  if (j != 0)
    reinterpret_cast<vtkUnsignedCharArray*>(&(*scalars))->SetArray (colors, j, 0);
  else
    reinterpret_cast<vtkUnsignedCharArray*>(&(*scalars))->SetNumberOfTuples (0);
  //delete [] colors;
  return (true);
}

///////////////////////////////////////////////////////////////////////////////////////////
pcl::visualization::PointCloudColorHandlerHSVField<pcl::PCLPointCloud2>::PointCloudColorHandlerHSVField (
    const pcl::visualization::PointCloudColorHandler<pcl::PCLPointCloud2>::PointCloudConstPtr &cloud) :
  pcl::visualization::PointCloudColorHandler<pcl::PCLPointCloud2>::PointCloudColorHandler (cloud),
  s_field_idx_ (-1), v_field_idx_ (-1)
{
  // Check for the presence of the "H" field
  field_idx_ = pcl::getFieldIndex (*cloud, "h");
  if (field_idx_ == -1)
  {
    capable_ = false;
    return;
  }

  // Check for the presence of the "S" field
  s_field_idx_ = pcl::getFieldIndex (*cloud, "s");
  if (s_field_idx_ == -1)
  {
    capable_ = false;
    return;
  }

  // Check for the presence of the "V" field
  v_field_idx_ = pcl::getFieldIndex (*cloud, "v");
  if (v_field_idx_ == -1)
  {
    capable_ = false;
    return;
  }
  capable_ = true;
}

///////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::PointCloudColorHandlerHSVField<pcl::PCLPointCloud2>::getColor (vtkSmartPointer<vtkDataArray> &scalars) const
{
  if (!capable_ || !cloud_)
    return (false);

  if (!scalars)
    scalars = vtkSmartPointer<vtkUnsignedCharArray>::New ();
  scalars->SetNumberOfComponents (3);

  vtkIdType nr_points = cloud_->width * cloud_->height;
  reinterpret_cast<vtkUnsignedCharArray*>(&(*scalars))->SetNumberOfTuples (nr_points);

  // Allocate enough memory to hold all colors
  // colors is taken over by SetArray (line 419)
  unsigned char* colors = new unsigned char[nr_points * 3];

  float h_data, v_data, s_data;
  int point_offset = cloud_->fields[field_idx_].offset;
  int j = 0;

  // If XYZ present, check if the points are invalid
  int x_idx = pcl::getFieldIndex (*cloud_, "x");
  if (x_idx != -1)
  {
    float x_data, y_data, z_data;
    int x_point_offset = cloud_->fields[x_idx].offset;

    // Color every point
    for (vtkIdType cp = 0; cp < nr_points; ++cp, 
                                           point_offset += cloud_->point_step, 
                                           x_point_offset += cloud_->point_step)
    {
      // Copy the value at the specified field
      memcpy (&h_data, &cloud_->data[point_offset], sizeof (float));
      memcpy (&s_data, &cloud_->data[point_offset + sizeof (float)], sizeof (float));
      memcpy (&v_data, &cloud_->data[point_offset + 2 * sizeof (float)], sizeof (float));

      if (!pcl_isfinite (h_data) || !pcl_isfinite (s_data) || !pcl_isfinite (v_data))
        continue;   //skip to next point

      memcpy (&x_data, &cloud_->data[x_point_offset], sizeof (float));
      memcpy (&y_data, &cloud_->data[x_point_offset + sizeof (float)], sizeof (float));
      memcpy (&z_data, &cloud_->data[x_point_offset + 2 * sizeof (float)], sizeof (float));

      if (!pcl_isfinite (x_data) || !pcl_isfinite (y_data) || !pcl_isfinite (z_data))
        continue;   //skip to next point

      int idx = j * 3;
      // Fill color data with HSV here:
      if (s_data == 0)
      {
        colors[idx] = colors[idx+1] = colors[idx+2] = static_cast<unsigned char> (v_data);
        continue;   //skip to next point
      } 
      float a = h_data / 60;
      int   i = static_cast<int> (floor (a));
      float f = a - static_cast<float> (i);
      float p = v_data * (1 - s_data);
      float q = v_data * (1 - s_data * f);
      float t = v_data * (1 - s_data * (1 - f));

      switch (i) 
      {
        case 0:
        {
          colors[idx]   = static_cast<unsigned char> (v_data);
          colors[idx+1] = static_cast<unsigned char> (t);
          colors[idx+2] = static_cast<unsigned char> (p);
          break;
        }
        case 1:
        {
          colors[idx]   = static_cast<unsigned char> (q);
          colors[idx+1] = static_cast<unsigned char> (v_data);
          colors[idx+2] = static_cast<unsigned char> (p);
          break;
        }
        case 2:
        {
          colors[idx]   = static_cast<unsigned char> (p);
          colors[idx+1] = static_cast<unsigned char> (v_data);
          colors[idx+2] = static_cast<unsigned char> (t);
          break;
        }
        case 3:
        {
          colors[idx]   = static_cast<unsigned char> (p);
          colors[idx+1] = static_cast<unsigned char> (q);
          colors[idx+2] = static_cast<unsigned char> (v_data);
          break;
        }
        case 4:
        {
          colors[idx]   = static_cast<unsigned char> (t);
          colors[idx+1] = static_cast<unsigned char> (p);
          colors[idx+2] = static_cast<unsigned char> (v_data);
          break;
        }
        default:
        {
          colors[idx]   = static_cast<unsigned char> (v_data);
          colors[idx+1] = static_cast<unsigned char> (p);
          colors[idx+2] = static_cast<unsigned char> (q);
          break;
        }
      }
      j++;
    }
  }
  // No XYZ data checks
  else
  {
    // Color every point
    for (vtkIdType cp = 0; cp < nr_points; ++cp, point_offset += cloud_->point_step)
    {
      // Copy the value at the specified field
      memcpy (&h_data, &cloud_->data[point_offset], sizeof (float));
      memcpy (&s_data, &cloud_->data[point_offset + sizeof (float)], sizeof (float));
      memcpy (&v_data, &cloud_->data[point_offset + 2 * sizeof (float)], sizeof (float));

      if (!pcl_isfinite (h_data) || !pcl_isfinite (s_data) || !pcl_isfinite (v_data))
        continue;   //skip to next point

      int idx = j * 3;
      // Fill color data with HSV here:
      if (s_data == 0)
      {
        colors[idx] = colors[idx+1] = colors[idx+2] = static_cast<unsigned char> (v_data);
        continue;   //skip to next point
      } 
      float a = h_data / 60;
      int   i = static_cast<int> (floor (a));
      float f = a - static_cast<float> (i);
      float p = v_data * (1 - s_data);
      float q = v_data * (1 - s_data * f);
      float t = v_data * (1 - s_data * (1 - f));

      switch (i) 
      {
        case 0:
        {
          colors[idx]   = static_cast<unsigned char> (v_data);
          colors[idx+1] = static_cast<unsigned char> (t);
          colors[idx+2] = static_cast<unsigned char> (p);
          break;
        }
        case 1:
        {
          colors[idx]   = static_cast<unsigned char> (q);
          colors[idx+1] = static_cast<unsigned char> (v_data);
          colors[idx+2] = static_cast<unsigned char> (p);
          break;
        }
        case 2:
        {
          colors[idx]   = static_cast<unsigned char> (p);
          colors[idx+1] = static_cast<unsigned char> (v_data);
          colors[idx+2] = static_cast<unsigned char> (t);
          break;
        }
        case 3:
        {
          colors[idx]   = static_cast<unsigned char> (p);
          colors[idx+1] = static_cast<unsigned char> (q);
          colors[idx+2] = static_cast<unsigned char> (v_data);
          break;
        }
        case 4:
        {
          colors[idx]   = static_cast<unsigned char> (t);
          colors[idx+1] = static_cast<unsigned char> (p);
          colors[idx+2] = static_cast<unsigned char> (v_data);
          break;
        }
        default:
        {
          colors[idx]   = static_cast<unsigned char> (v_data);
          colors[idx+1] = static_cast<unsigned char> (p);
          colors[idx+2] = static_cast<unsigned char> (q);
          break;
        }
      }
      j++;
    }
  }
  // Set array takes over allocation (Set save to 1 to keep the class from deleting the array when it cleans up or reallocates memory.)
  reinterpret_cast<vtkUnsignedCharArray*>(&(*scalars))->SetArray (colors, 3 * j, 0);
  return (true);
}

///////////////////////////////////////////////////////////////////////////////////////////
pcl::visualization::PointCloudColorHandlerGenericField<pcl::PCLPointCloud2>::PointCloudColorHandlerGenericField (
    const pcl::visualization::PointCloudColorHandler<pcl::PCLPointCloud2>::PointCloudConstPtr &cloud,
    const std::string &field_name) : 
  pcl::visualization::PointCloudColorHandler<pcl::PCLPointCloud2>::PointCloudColorHandler (cloud),
  field_name_ (field_name)
{
  field_idx_  = pcl::getFieldIndex (*cloud, field_name);
  if (field_idx_ != -1)
    capable_ = true;
  else
    capable_ = false;
}

///////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::PointCloudColorHandlerGenericField<pcl::PCLPointCloud2>::getColor (vtkSmartPointer<vtkDataArray> &scalars) const
{
  if (!capable_ || !cloud_)
    return (false);

  if (!scalars)
    scalars = vtkSmartPointer<vtkFloatArray>::New ();
  scalars->SetNumberOfComponents (1);

  vtkIdType nr_points = cloud_->width * cloud_->height;
  reinterpret_cast<vtkFloatArray*>(&(*scalars))->SetNumberOfTuples (nr_points);

  float* colors = new float[nr_points];
  float field_data;
  int j = 0;
  int point_offset = cloud_->fields[field_idx_].offset;

  // If XYZ present, check if the points are invalid
  int x_idx = pcl::getFieldIndex (*cloud_, "x");
  if (x_idx != -1)
  {
    float x_data, y_data, z_data;
    int x_point_offset = cloud_->fields[x_idx].offset;
    
    // Color every point
    for (vtkIdType cp = 0; cp < nr_points; ++cp,
                                           point_offset += cloud_->point_step, 
                                           x_point_offset += cloud_->point_step)
    {
      memcpy (&x_data, &cloud_->data[x_point_offset], sizeof (float));
      memcpy (&y_data, &cloud_->data[x_point_offset + sizeof (float)], sizeof (float));
      memcpy (&z_data, &cloud_->data[x_point_offset + 2 * sizeof (float)], sizeof (float));
      if (!pcl_isfinite (x_data) || !pcl_isfinite (y_data) || !pcl_isfinite (z_data))
        continue;

      // Copy the value at the specified field
      memcpy (&field_data, &cloud_->data[point_offset], pcl::getFieldSize (cloud_->fields[field_idx_].datatype));
      colors[j] = field_data;
      j++;
    }
  }
  // No XYZ data checks
  else
  {
    // Color every point
    for (vtkIdType cp = 0; cp < nr_points; ++cp, point_offset += cloud_->point_step)
    {
      // Copy the value at the specified field
      //memcpy (&field_data, &cloud_->data[point_offset], sizeof (float));
      memcpy (&field_data, &cloud_->data[point_offset], pcl::getFieldSize (cloud_->fields[field_idx_].datatype));

      if (!pcl_isfinite (field_data))
        continue;
      colors[j] = field_data;
      j++;
    }
  }
  reinterpret_cast<vtkFloatArray*>(&(*scalars))->SetArray (colors, j, 0);
  return (true);
}

///////////////////////////////////////////////////////////////////////////////////////////
pcl::visualization::PointCloudColorHandlerRGBAField<pcl::PCLPointCloud2>::PointCloudColorHandlerRGBAField (
    const pcl::visualization::PointCloudColorHandler<pcl::PCLPointCloud2>::PointCloudConstPtr &cloud) :
  pcl::visualization::PointCloudColorHandler<pcl::PCLPointCloud2>::PointCloudColorHandler (cloud)
{
  // Handle the 24-bit packed RGBA values
  field_idx_ = pcl::getFieldIndex (*cloud, "rgba");
  if (field_idx_ != -1)
    capable_ = true;
  else
    capable_ = false;
}

///////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::PointCloudColorHandlerRGBAField<pcl::PCLPointCloud2>::getColor (vtkSmartPointer<vtkDataArray> &scalars) const
{
  if (!capable_ || !cloud_)
    return (false);

  if (!scalars)
    scalars = vtkSmartPointer<vtkUnsignedCharArray>::New ();
  scalars->SetNumberOfComponents (4);

  vtkIdType nr_points = cloud_->width * cloud_->height;
  reinterpret_cast<vtkUnsignedCharArray*>(&(*scalars))->SetNumberOfTuples (nr_points);
  // Allocate enough memory to hold all colors
  unsigned char* colors = new unsigned char[nr_points * 4];

  pcl::RGB rgba_data;
  int point_offset = cloud_->fields[field_idx_].offset;
  int j = 0;

  // If XYZ present, check if the points are invalid
  int x_idx = pcl::getFieldIndex (*cloud_, "x");
  if (x_idx != -1)
  {
    float x_data, y_data, z_data;
    int x_point_offset = cloud_->fields[x_idx].offset;

    // Color every point
    for (vtkIdType cp = 0; cp < nr_points; ++cp,
                                           point_offset += cloud_->point_step,
                                           x_point_offset += cloud_->point_step)
    {
      // Copy the value at the specified field
      memcpy (&rgba_data, &cloud_->data[point_offset], sizeof (float));

      memcpy (&x_data, &cloud_->data[x_point_offset], sizeof (float));
      memcpy (&y_data, &cloud_->data[x_point_offset + sizeof (float)], sizeof (float));
      memcpy (&z_data, &cloud_->data[x_point_offset + 2 * sizeof (float)], sizeof (float));

      if (!pcl_isfinite (x_data) || !pcl_isfinite (y_data) || !pcl_isfinite (z_data))
        continue;

      colors[j + 0] = rgba_data.r;
      colors[j + 1] = rgba_data.g;
      colors[j + 2] = rgba_data.b;
      colors[j + 3] = rgba_data.a;
      j += 4;
    }
  }
  // No XYZ data checks
  else
  {
    // Color every point
    for (vtkIdType cp = 0; cp < nr_points; ++cp, point_offset += cloud_->point_step)
    {
      // Copy the value at the specified field
      memcpy (&rgba_data, &cloud_->data[point_offset], sizeof (float));

      colors[j + 0] = rgba_data.r;
      colors[j + 1] = rgba_data.g;
      colors[j + 2] = rgba_data.b;
      colors[j + 3] = rgba_data.a;
      j += 4;
    }
  }
  if (j != 0)
    reinterpret_cast<vtkUnsignedCharArray*>(&(*scalars))->SetArray (colors, j, 0);
  else
    reinterpret_cast<vtkUnsignedCharArray*>(&(*scalars))->SetNumberOfTuples (0);
  //delete [] colors;
  return (true);
}

///////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::visualization::PointCloudGeometryHandler<pcl::PCLPointCloud2>::getGeometry (vtkSmartPointer<vtkPoints> &points) const
{
  if (!capable_)
    return;

  if (!points)
    points = vtkSmartPointer<vtkPoints>::New ();
  points->SetDataTypeToFloat ();

  vtkSmartPointer<vtkFloatArray> data = vtkSmartPointer<vtkFloatArray>::New ();
  data->SetNumberOfComponents (3);
  vtkIdType nr_points = cloud_->width * cloud_->height;

  // Add all points
  float dim;
  vtkIdType j = 0;    // true point index
  float* pts = static_cast<float*> (malloc (nr_points * 3 * sizeof (float)));
  int point_offset = 0;

  // If the dataset has no invalid values, just copy all of them
  if (cloud_->is_dense)
  {
    for (vtkIdType i = 0; i < nr_points; ++i, point_offset+=cloud_->point_step)
    {
      // Copy the value at the specified field
      memcpy (&dim, &cloud_->data[point_offset + cloud_->fields[field_x_idx_].offset], sizeof (float));
      pts[i * 3 + 0] = dim;

      memcpy (&dim, &cloud_->data[point_offset + cloud_->fields[field_y_idx_].offset], sizeof (float));
      pts[i * 3 + 1] = dim;

      memcpy (&dim, &cloud_->data[point_offset + cloud_->fields[field_z_idx_].offset], sizeof (float));
      pts[i * 3 + 2] = dim;
    }
    data->SetArray (&pts[0], nr_points * 3, 0);
    points->SetData (data);
  }
  else
  {
    for (vtkIdType i = 0; i < nr_points; ++i, point_offset+=cloud_->point_step)
    {
      // Copy the value at the specified field
      memcpy (&dim, &cloud_->data[point_offset + cloud_->fields[field_x_idx_].offset], sizeof (float));
      if (!pcl_isfinite (dim))
        continue;
      pts[j * 3 + 0] = dim;

      memcpy (&dim, &cloud_->data[point_offset + cloud_->fields[field_y_idx_].offset], sizeof (float));
      if (!pcl_isfinite (dim))
        continue;
      pts[j * 3 + 1] = dim;

      memcpy (&dim, &cloud_->data[point_offset + cloud_->fields[field_z_idx_].offset], sizeof (float));
      if (!pcl_isfinite (dim))
        continue;
      pts[j * 3 + 2] = dim;

      // Set j and increment
      j++;
    }
    data->SetArray (&pts[0], j * 3, 0);
    points->SetData (data);
  }
}

///////////////////////////////////////////////////////////////////////////////////////////
pcl::visualization::PointCloudGeometryHandlerXYZ<pcl::PCLPointCloud2>::PointCloudGeometryHandlerXYZ (const PointCloudConstPtr &cloud)
: pcl::visualization::PointCloudGeometryHandler<pcl::PCLPointCloud2>::PointCloudGeometryHandler (cloud)
{
  field_x_idx_ = pcl::getFieldIndex (*cloud, "x");
  if (field_x_idx_ == -1)
    return;
  field_y_idx_ = pcl::getFieldIndex (*cloud, "y");
  if (field_y_idx_ == -1)
    return;
  field_z_idx_ = pcl::getFieldIndex (*cloud, "z");
  if (field_z_idx_ == -1)
    return;
  capable_ = true;
}

///////////////////////////////////////////////////////////////////////////////////////////
pcl::visualization::PointCloudGeometryHandlerSurfaceNormal<pcl::PCLPointCloud2>::PointCloudGeometryHandlerSurfaceNormal (const PointCloudConstPtr &cloud)
: pcl::visualization::PointCloudGeometryHandler<pcl::PCLPointCloud2>::PointCloudGeometryHandler (cloud)
{
  field_x_idx_ = pcl::getFieldIndex (*cloud, "normal_x");
  if (field_x_idx_ == -1)
    return;
  field_y_idx_ = pcl::getFieldIndex (*cloud, "normal_y");
  if (field_y_idx_ == -1)
    return;
  field_z_idx_ = pcl::getFieldIndex (*cloud, "normal_z");
  if (field_z_idx_ == -1)
    return;
  capable_ = true;
}

///////////////////////////////////////////////////////////////////////////////////////////
pcl::visualization::PointCloudGeometryHandlerCustom<pcl::PCLPointCloud2>::PointCloudGeometryHandlerCustom (
    const PointCloudConstPtr &cloud, const std::string &x_field_name, const std::string &y_field_name, const std::string &z_field_name) 
: pcl::visualization::PointCloudGeometryHandler<pcl::PCLPointCloud2>::PointCloudGeometryHandler (cloud), field_name_ ()
{
  field_x_idx_ = pcl::getFieldIndex (*cloud, x_field_name);
  if (field_x_idx_ == -1)
    return;
  field_y_idx_ = pcl::getFieldIndex (*cloud, y_field_name);
  if (field_y_idx_ == -1)
    return;
  field_z_idx_ = pcl::getFieldIndex (*cloud, z_field_name);
  if (field_z_idx_ == -1)
    return;
  field_name_ = x_field_name + y_field_name + z_field_name;
  capable_ = true;
}

// Instantiations of specific point types
#ifdef PCL_ONLY_CORE_POINT_TYPES
  PCL_INSTANTIATE(PointCloudGeometryHandlerXYZ, (pcl::PointSurfel)(pcl::PointXYZ)(pcl::PointXYZI)(pcl::PointXYZRGB)(pcl::PointXYZRGBA)(pcl::PointNormal)(pcl::PointXYZRGBNormal)(pcl::PointXYZRGBL))
  PCL_INSTANTIATE(PointCloudGeometryHandlerSurfaceNormal, (pcl::Normal)(pcl::PointNormal)(pcl::PointXYZRGBNormal))
#else
  PCL_INSTANTIATE(PointCloudGeometryHandlerXYZ, PCL_XYZ_POINT_TYPES)
  PCL_INSTANTIATE(PointCloudGeometryHandlerSurfaceNormal, PCL_NORMAL_POINT_TYPES)
#endif

