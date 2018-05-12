/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
 * $Id: vtk_io.hpp 4968 2012-05-03 06:39:52Z doria $
 *
 */

#ifndef PCL_IO_VTK_IO_IMPL_H_
#define PCL_IO_VTK_IO_IMPL_H_

// PCL
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_traits.h>

// VTK
// Ignore warnings in the above headers
#ifdef __GNUC__
#pragma GCC system_header
#endif
#include <vtkVersion.h>
#include <vtkFloatArray.h>
#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkUnsignedCharArray.h>
#include <vtkSmartPointer.h>
#include <vtkStructuredGrid.h>
#include <vtkVertexGlyphFilter.h>

// Support for VTK 7.1 upwards
#ifdef vtkGenericDataArray_h
#define SetTupleValue SetTypedTuple
#define InsertNextTupleValue InsertNextTypedTuple
#define GetTupleValue GetTypedTuple
#endif

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::io::vtkPolyDataToPointCloud (vtkPolyData* const polydata, pcl::PointCloud<PointT>& cloud)
{
  // This generic template will convert any VTK PolyData
  // to a coordinate-only PointXYZ PCL format.
  cloud.width = polydata->GetNumberOfPoints ();
  cloud.height = 1; // This indicates that the point cloud is unorganized
  cloud.is_dense = false;
  cloud.points.resize (cloud.width * cloud.height);

  // Get a list of all the fields available
  std::vector<pcl::PCLPointField> fields;
  pcl::for_each_type<typename pcl::traits::fieldList<PointT>::type>(pcl::detail::FieldAdder<PointT>(fields));

  // Check if XYZ is present
  int x_idx = -1, y_idx = -1, z_idx = -1;
  for (size_t d = 0; d < fields.size (); ++d)
  {
    if (fields[d].name == "x")
      x_idx = fields[d].offset;
    else if (fields[d].name == "y")
      y_idx = fields[d].offset;
    else if (fields[d].name == "z")
      z_idx = fields[d].offset;
  }
  // Set the coordinates of the pcl::PointCloud (if the pcl::PointCloud supports coordinates)
  if (x_idx != -1 && y_idx != -1 && z_idx != -1)
  {
    for (size_t i = 0; i < cloud.points.size (); ++i)
    {
      double coordinate[3];
      polydata->GetPoint (i, coordinate);
      pcl::setFieldValue<PointT, float> (cloud.points[i], x_idx, coordinate[0]);
      pcl::setFieldValue<PointT, float> (cloud.points[i], y_idx, coordinate[1]);
      pcl::setFieldValue<PointT, float> (cloud.points[i], z_idx, coordinate[2]);
    }
  }

  // Check if Normals are present
  int normal_x_idx = -1, normal_y_idx = -1, normal_z_idx = -1;
  for (size_t d = 0; d < fields.size (); ++d)
  {
    if (fields[d].name == "normal_x")
      normal_x_idx = fields[d].offset;
    else if (fields[d].name == "normal_y")
      normal_y_idx = fields[d].offset;
    else if (fields[d].name == "normal_z")
      normal_z_idx = fields[d].offset;
  }
  // Set the normals of the pcl::PointCloud (if the pcl::PointCloud supports normals and the input vtkPolyData has normals)
  vtkFloatArray* normals = vtkFloatArray::SafeDownCast (polydata->GetPointData ()->GetNormals ());
  if (normal_x_idx != -1 && normal_y_idx != -1 && normal_z_idx != -1 && normals)
  {
    for (size_t i = 0; i < cloud.points.size (); ++i)
    {
      float normal[3];
      normals->GetTupleValue (i, normal);
      pcl::setFieldValue<PointT, float> (cloud.points[i], normal_x_idx, normal[0]);
      pcl::setFieldValue<PointT, float> (cloud.points[i], normal_y_idx, normal[1]);
      pcl::setFieldValue<PointT, float> (cloud.points[i], normal_z_idx, normal[2]);
    }
  }

  // Set the colors of the pcl::PointCloud (if the pcl::PointCloud supports colors and the input vtkPolyData has colors)
  vtkUnsignedCharArray* colors = vtkUnsignedCharArray::SafeDownCast (polydata->GetPointData ()->GetScalars ());
  int rgb_idx = -1;
  for (size_t d = 0; d < fields.size (); ++d)
  {
    if (fields[d].name == "rgb" || fields[d].name == "rgba")
    {
      rgb_idx = fields[d].offset;
      break;
    }
  }

  if (rgb_idx != -1 && colors)
  {
    for (size_t i = 0; i < cloud.points.size (); ++i)
    {
      unsigned char color[3];
      colors->GetTupleValue (i, color);
      pcl::RGB rgb;
      rgb.r = color[0]; rgb.g = color[1]; rgb.b = color[2];
      pcl::setFieldValue<PointT, uint32_t> (cloud.points[i], rgb_idx, rgb.rgba);
    }
  }
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::io::vtkStructuredGridToPointCloud (vtkStructuredGrid* const structured_grid, pcl::PointCloud<PointT>& cloud)
{
  int dimensions[3];
  structured_grid->GetDimensions (dimensions);
  cloud.width = dimensions[0];
  cloud.height = dimensions[1]; // This indicates that the point cloud is organized
  cloud.is_dense = true;
  cloud.points.resize (cloud.width * cloud.height);

  // Get a list of all the fields available
  std::vector<pcl::PCLPointField> fields;
  pcl::for_each_type<typename pcl::traits::fieldList<PointT>::type>(pcl::detail::FieldAdder<PointT>(fields));

  // Check if XYZ is present
  int x_idx = -1, y_idx = -1, z_idx = -1;
  for (size_t d = 0; d < fields.size (); ++d)
  {
    if (fields[d].name == "x")
      x_idx = fields[d].offset;
    else if (fields[d].name == "y")
      y_idx = fields[d].offset;
    else if (fields[d].name == "z")
      z_idx = fields[d].offset;
  }

  if (x_idx != -1 && y_idx != -1 && z_idx != -1)
  {
    for (size_t i = 0; i < cloud.width; ++i)
    {
      for (size_t j = 0; j < cloud.height; ++j)
      {
        int queryPoint[3] = {i, j, 0};
        vtkIdType pointId = vtkStructuredData::ComputePointId (dimensions, queryPoint);
        double coordinate[3];
        if (structured_grid->IsPointVisible (pointId))
        {
          structured_grid->GetPoint (pointId, coordinate);
          pcl::setFieldValue<PointT, float> (cloud (i, j), x_idx, coordinate[0]);
          pcl::setFieldValue<PointT, float> (cloud (i, j), y_idx, coordinate[1]);
          pcl::setFieldValue<PointT, float> (cloud (i, j), z_idx, coordinate[2]);
        }
        else
        {
          // Fill the point with an "empty" point?
        }
      }
    }
  }

  // Check if Normals are present
  int normal_x_idx = -1, normal_y_idx = -1, normal_z_idx = -1;
  for (size_t d = 0; d < fields.size (); ++d)
  {
    if (fields[d].name == "normal_x")
      normal_x_idx = fields[d].offset;
    else if (fields[d].name == "normal_y")
      normal_y_idx = fields[d].offset;
    else if (fields[d].name == "normal_z")
      normal_z_idx = fields[d].offset;
  }
  // Set the normals of the pcl::PointCloud (if the pcl::PointCloud supports normals and the input vtkStructuredGrid has normals)
  vtkFloatArray* normals = vtkFloatArray::SafeDownCast (structured_grid->GetPointData ()->GetNormals ());

  if (normal_x_idx != -1 && normal_y_idx != -1 && normal_z_idx != -1 && normals)
  {
    for (size_t i = 0; i < cloud.width; ++i)
    {
      for (size_t j = 0; j < cloud.height; ++j)
      {
        int queryPoint[3] = {i, j, 0};
        vtkIdType pointId = vtkStructuredData::ComputePointId (dimensions, queryPoint);
        float normal[3];
        if (structured_grid->IsPointVisible (pointId))
        {
          normals->GetTupleValue (i, normal);
          pcl::setFieldValue<PointT, float> (cloud (i, j), normal_x_idx, normal[0]);
          pcl::setFieldValue<PointT, float> (cloud (i, j), normal_y_idx, normal[1]);
          pcl::setFieldValue<PointT, float> (cloud (i, j), normal_z_idx, normal[2]);
        }
        else
        {
          // Fill the point with an "empty" point?
        }
      }
    }
  }

  // Set the colors of the pcl::PointCloud (if the pcl::PointCloud supports colors and the input vtkStructuredGrid has colors)
  vtkUnsignedCharArray* colors = vtkUnsignedCharArray::SafeDownCast (structured_grid->GetPointData ()->GetArray ("Colors"));
  int rgb_idx = -1;
  for (size_t d = 0; d < fields.size (); ++d)
  {
    if (fields[d].name == "rgb" || fields[d].name == "rgba")
    {
      rgb_idx = fields[d].offset;
      break;
    }
  }

  if (rgb_idx != -1 && colors)
  {
    for (size_t i = 0; i < cloud.width; ++i)
    {
      for (size_t j = 0; j < cloud.height; ++j)
      {
        int queryPoint[3] = {i, j, 0};
        vtkIdType pointId = vtkStructuredData::ComputePointId(dimensions, queryPoint);
        unsigned char color[3];
        if (structured_grid->IsPointVisible (pointId))
        {
          colors->GetTupleValue (i, color);
          pcl::RGB rgb;
          rgb.r = color[0]; rgb.g = color[1]; rgb.b = color[2];
          pcl::setFieldValue<PointT, uint32_t> (cloud (i, j), rgb_idx, rgb.rgba);
        }
        else
        {
          // Fill the point with an "empty" point?
        }
      }
    }
  }
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::io::pointCloudTovtkPolyData (const pcl::PointCloud<PointT>& cloud, vtkPolyData* const pdata)
{
  // Get a list of all the fields available
  std::vector<pcl::PCLPointField> fields;
  pcl::for_each_type<typename pcl::traits::fieldList<PointT>::type>(pcl::detail::FieldAdder<PointT>(fields));

  // Coordinates (always must have coordinates)
  vtkIdType nr_points = cloud.points.size ();
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New ();
  points->SetNumberOfPoints (nr_points);
  // Get a pointer to the beginning of the data array
  float *data = (static_cast<vtkFloatArray*> (points->GetData ()))->GetPointer (0);

  // Set the points
  if (cloud.is_dense)
  {
    for (vtkIdType i = 0; i < nr_points; ++i)
      memcpy (&data[i * 3], &cloud[i].x, 12);    // sizeof (float) * 3
  }
  else
  {
    vtkIdType j = 0;    // true point index
    for (vtkIdType i = 0; i < nr_points; ++i)
    {
      // Check if the point is invalid
      if (!pcl_isfinite (cloud[i].x) ||
          !pcl_isfinite (cloud[i].y) ||
          !pcl_isfinite (cloud[i].z))
        continue;

      memcpy (&data[j * 3], &cloud[i].x, 12);    // sizeof (float) * 3
      j++;
    }
    nr_points = j;
    points->SetNumberOfPoints (nr_points);
  }

  // Create a temporary PolyData and add the points to it
  vtkSmartPointer<vtkPolyData> temp_polydata = vtkSmartPointer<vtkPolyData>::New ();
  temp_polydata->SetPoints (points);

  // Check if Normals are present
  int normal_x_idx = -1, normal_y_idx = -1, normal_z_idx = -1;
  for (size_t d = 0; d < fields.size (); ++d)
  {
    if (fields[d].name == "normal_x")
      normal_x_idx = fields[d].offset;
    else if (fields[d].name == "normal_y")
      normal_y_idx = fields[d].offset;
    else if (fields[d].name == "normal_z")
      normal_z_idx = fields[d].offset;
  }
  if (normal_x_idx != -1 && normal_y_idx != -1 && normal_z_idx != -1)
  {
    vtkSmartPointer<vtkFloatArray> normals = vtkSmartPointer<vtkFloatArray>::New ();
    normals->SetNumberOfComponents (3); //3d normals (ie x,y,z)
    normals->SetNumberOfTuples (cloud.size ());
    normals->SetName ("Normals");

    for (size_t i = 0; i < cloud.size (); ++i)
    {
      float normal[3];
      pcl::getFieldValue<PointT, float> (cloud[i], normal_x_idx, normal[0]);
      pcl::getFieldValue<PointT, float> (cloud[i], normal_y_idx, normal[1]);
      pcl::getFieldValue<PointT, float> (cloud[i], normal_z_idx, normal[2]);
      normals->SetTupleValue (i, normal);
    }
    temp_polydata->GetPointData ()->SetNormals (normals);
  }

  // Colors (optional)
  int rgb_idx = -1;
  for (size_t d = 0; d < fields.size (); ++d)
  {
    if (fields[d].name == "rgb" || fields[d].name == "rgba")
    {
      rgb_idx = fields[d].offset;
      break;
    }
  }
  if (rgb_idx != -1)
  {
    vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New ();
    colors->SetNumberOfComponents (3);
    colors->SetNumberOfTuples (cloud.size ());
    colors->SetName ("RGB");

    for (size_t i = 0; i < cloud.size (); ++i)
    {
      unsigned char color[3];
      pcl::RGB rgb;
      pcl::getFieldValue<PointT, uint32_t> (cloud[i], rgb_idx, rgb.rgba);
      color[0] = rgb.r; color[1] = rgb.g; color[2] = rgb.b;
      colors->SetTupleValue (i, color);
    }
    temp_polydata->GetPointData ()->SetScalars (colors);
  }

  // Add 0D topology to every point
  vtkSmartPointer<vtkVertexGlyphFilter> vertex_glyph_filter = vtkSmartPointer<vtkVertexGlyphFilter>::New ();
  #if VTK_MAJOR_VERSION < 6
    vertex_glyph_filter->AddInputConnection (temp_polydata->GetProducerPort ());
  #else
    vertex_glyph_filter->SetInputData (temp_polydata);
  #endif
  vertex_glyph_filter->Update ();

  pdata->DeepCopy (vertex_glyph_filter->GetOutput ());
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::io::pointCloudTovtkStructuredGrid (const pcl::PointCloud<PointT>& cloud, vtkStructuredGrid* const structured_grid)
{
  // Get a list of all the fields available
  std::vector<pcl::PCLPointField> fields;
  pcl::for_each_type<typename pcl::traits::fieldList<PointT>::type>(pcl::detail::FieldAdder<PointT>(fields));

  int dimensions[3] = {cloud.width, cloud.height, 1};
  structured_grid->SetDimensions (dimensions);

  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New ();
  points->SetNumberOfPoints (cloud.width * cloud.height);

  for (size_t i = 0; i < cloud.width; ++i)
  {
    for (size_t j = 0; j < cloud.height; ++j)
    {
      int queryPoint[3] = {i, j, 0};
      vtkIdType pointId = vtkStructuredData::ComputePointId (dimensions, queryPoint);
      const PointT &point = cloud (i, j);

      if (pcl::isFinite (point))
      {
        float p[3] = {point.x, point.y, point.z};
        points->SetPoint (pointId, p);
      }
      else
      {
      }
    }
  }

  structured_grid->SetPoints (points);

  // Check if Normals are present
  int normal_x_idx = -1, normal_y_idx = -1, normal_z_idx = -1;
  for (size_t d = 0; d < fields.size (); ++d)
  {
    if (fields[d].name == "normal_x")
      normal_x_idx = fields[d].offset;
    else if (fields[d].name == "normal_y")
      normal_y_idx = fields[d].offset;
    else if (fields[d].name == "normal_z")
      normal_z_idx = fields[d].offset;
  }

  if (normal_x_idx != -1 && normal_y_idx != -1 && normal_z_idx != -1)
  {
    vtkSmartPointer<vtkFloatArray> normals = vtkSmartPointer<vtkFloatArray>::New ();
    normals->SetNumberOfComponents (3); // Note this must come before the SetNumberOfTuples calls
    normals->SetNumberOfTuples (cloud.width * cloud.height);
    normals->SetName ("Normals");
    for (size_t i = 0; i < cloud.width; ++i)
    {
      for (size_t j = 0; j < cloud.height; ++j)
      {
        int queryPoint[3] = {i, j, 0};
        vtkIdType pointId = vtkStructuredData::ComputePointId (dimensions, queryPoint);
        const PointT &point = cloud (i, j);

        float normal[3];
        pcl::getFieldValue<PointT, float> (point, normal_x_idx, normal[0]);
        pcl::getFieldValue<PointT, float> (point, normal_y_idx, normal[1]);
        pcl::getFieldValue<PointT, float> (point, normal_z_idx, normal[2]);
        normals->SetTupleValue (pointId, normal);
      }
    }

    structured_grid->GetPointData ()->SetNormals (normals);
  }

  // Colors (optional)
  int rgb_idx = -1;
  for (size_t d = 0; d < fields.size (); ++d)
  {
    if (fields[d].name == "rgb" || fields[d].name == "rgba")
    {
      rgb_idx = fields[d].offset;
      break;
    }
  }

  if (rgb_idx != -1)
  {
    vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
    colors->SetNumberOfComponents (3); // Note this must come before the SetNumberOfTuples calls
    colors->SetNumberOfTuples (cloud.width * cloud.height);
    colors->SetName ("Colors");
    for (size_t i = 0; i < cloud.width; ++i)
    {
      for (size_t j = 0; j < cloud.height; ++j)
      {
        int queryPoint[3] = {i, j, 0};
        vtkIdType pointId = vtkStructuredData::ComputePointId (dimensions, queryPoint);
        const PointT &point = cloud (i, j);

        if (pcl::isFinite (point))
        {

          unsigned char color[3];
          pcl::RGB rgb;
          pcl::getFieldValue<PointT, uint32_t> (cloud[i], rgb_idx, rgb.rgba);
          color[0] = rgb.r; color[1] = rgb.g; color[2] = rgb.b;
          colors->SetTupleValue (pointId, color);
        }
        else
        {
        }
      }
    }
    structured_grid->GetPointData ()->AddArray (colors);
  }
}

#ifdef vtkGenericDataArray_h
#undef SetTupleValue
#undef InsertNextTupleValue
#undef GetTupleValue
#endif

#endif  //#ifndef PCL_IO_VTK_IO_H_

