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
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// VTK
// Ignore warnings in the above headers
#ifdef __GNUC__
#pragma GCC system_header 
#endif
#include <vtkFloatArray.h>
#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkUnsignedCharArray.h>
#include <vtkSmartPointer.h>
#include <vtkStructuredGrid.h>
#include <vtkVertexGlyphFilter.h>

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::io::vtkPolyDataToPointCloud (vtkPolyData* const polydata, pcl::PointCloud<PointT>& cloud)
{
  // This generic template will convert any VTK PolyData
  // to a coordinate-only PointXYZ PCL format.
  typedef pcl::PointCloud<PointT> CloudT;

  typedef typename pcl::traits::fieldList<typename CloudT::PointType>::type FieldList;

  cloud.width = polydata->GetNumberOfPoints ();
  cloud.height = 1; // This indicates that the point cloud is unorganized
  cloud.is_dense = false;
  cloud.points.resize (cloud.width);

  typename CloudT::PointType test_point = cloud.points[0];

  bool has_x = false; bool has_y = false; bool has_z = false;
  float x_val = 0.0f; float y_val = 0.0f; float z_val = 0.0f;
  pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> (test_point, "x", has_x, x_val));
  pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> (test_point, "y", has_y, y_val));
  pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> (test_point, "z", has_z, z_val));

  // Set the coordinates of the pcl::PointCloud (if the pcl::PointCloud supports coordinates)
  if (has_x && has_y && has_z)
  {
    for (size_t i = 0; i < cloud.points.size (); ++i)
    {
      double coordinate[3];
      polydata->GetPoint (i, coordinate);
      typename CloudT::PointType p = cloud.points[i];
      pcl::for_each_type<FieldList> (pcl::SetIfFieldExists<typename CloudT::PointType, float> (p, "x", coordinate[0]));
      pcl::for_each_type<FieldList> (pcl::SetIfFieldExists<typename CloudT::PointType, float> (p, "y", coordinate[1]));
      pcl::for_each_type<FieldList> (pcl::SetIfFieldExists<typename CloudT::PointType, float> (p, "z", coordinate[2]));
      cloud.points[i] = p;
    }
  }

  // Set the normals of the pcl::PointCloud (if the pcl::PointCloud supports normals and the input vtkPolyData has normals)
  bool has_normal_x = false; bool has_normal_y = false; bool has_normal_z = false;
  float normal_x_val = 0.0f; float normal_y_val = 0.0f; float normal_z_val = 0.0f;
  pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> (test_point,
                                                                                            "normal_x", has_normal_x, normal_x_val));
  pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> (test_point,
                                                                                            "y_normal", has_normal_y, normal_y_val));
  pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> (test_point,
                                                                                            "z_normal", has_normal_z, normal_z_val));

  vtkFloatArray* normals = vtkFloatArray::SafeDownCast (polydata->GetPointData ()->GetNormals ());
  if (has_normal_x && has_normal_y && has_normal_z && normals)
  {
    for (size_t i = 0; i < cloud.points.size (); ++i)
    {
      float normal[3];
      normals->GetTupleValue (i, normal);
      typename CloudT::PointType p = cloud.points[i];
      pcl::for_each_type<FieldList> (pcl::SetIfFieldExists<typename CloudT::PointType, float> (p, "normal_x", normal[0]));
      pcl::for_each_type<FieldList> (pcl::SetIfFieldExists<typename CloudT::PointType, float> (p, "normal_y", normal[1]));
      pcl::for_each_type<FieldList> (pcl::SetIfFieldExists<typename CloudT::PointType, float> (p, "normal_z", normal[2]));
      cloud.points[i] = p;
    }
  }

  // Set the colors of the pcl::PointCloud (if the pcl::PointCloud supports colors and the input vtkPolyData has colors)
  bool has_r = false; bool has_g = false; bool has_b = false;
  unsigned char r_val = 0.0f; unsigned char g_val = 0.0f; unsigned char b_val = 0.0f;
  pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, unsigned char> (test_point,
                                                                                            "r", has_r, r_val));
  pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, unsigned char> (test_point,
                                                                                            "g", has_g, g_val));
  pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, unsigned char> (test_point,
                                                                                            "b", has_b, b_val));

  vtkUnsignedCharArray* colors = vtkUnsignedCharArray::SafeDownCast (polydata->GetPointData ()->GetScalars ());
  if (has_r && has_g && has_b && colors)
  {
    for (size_t i = 0; i < cloud.points.size (); ++i)
    {
      unsigned char color[3];
      colors->GetTupleValue (i, color);
      typename CloudT::PointType p = cloud.points[i];
      pcl::for_each_type<FieldList> (pcl::SetIfFieldExists<typename CloudT::PointType, unsigned char> (p, "r", color[0]));
      pcl::for_each_type<FieldList> (pcl::SetIfFieldExists<typename CloudT::PointType, unsigned char> (p, "g", color[1]));
      pcl::for_each_type<FieldList> (pcl::SetIfFieldExists<typename CloudT::PointType, unsigned char> (p, "b", color[2]));
      cloud.points[i] = p;
    }
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::io::vtkStructuredGridToPointCloud (vtkStructuredGrid* const structured_grid, pcl::PointCloud<PointT>& cloud)
{
  typedef pcl::PointCloud<PointT> CloudT;

  int dimensions[3];
  structured_grid->GetDimensions (dimensions);
  cloud.width = dimensions[0];
  cloud.height = dimensions[1]; // This indicates that the point cloud is organized
  cloud.is_dense = true;
  cloud.points.resize (cloud.width * cloud.height);

  typename CloudT::PointType test_point = cloud.points[0];

  typedef typename pcl::traits::fieldList<typename CloudT::PointType>::type FieldList;
  
  bool has_x = false; bool has_y = false; bool has_z = false;
  float x_val = 0.0f; float y_val = 0.0f; float z_val = 0.0f;
  pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> (test_point, "x", has_x, x_val));
  pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> (test_point, "y", has_y, y_val));
  pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> (test_point, "z", has_z, z_val));

  if (has_x && has_y && has_z)
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
          typename CloudT::PointType p = cloud (i, j);
          pcl::for_each_type<FieldList> (pcl::SetIfFieldExists<typename CloudT::PointType, float> (p, "x", coordinate[0]));
          pcl::for_each_type<FieldList> (pcl::SetIfFieldExists<typename CloudT::PointType, float> (p, "y", coordinate[1]));
          pcl::for_each_type<FieldList> (pcl::SetIfFieldExists<typename CloudT::PointType, float> (p, "z", coordinate[2]));
          cloud (i, j) = p;
        }
        else
        {
          // Fill the point with an "empty" point?
        }
      }
    }
  }

  // Set the normals of the pcl::PointCloud (if the pcl::PointCloud supports normals and the input vtkStructuredGrid has normals)
  bool has_normal_x = false; bool has_normal_y = false; bool has_normal_z = false;
  float normal_x_val = 0.0f; float normal_y_val = 0.0f; float normal_z_val = 0.0f;
  pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> (test_point,
                                                                                            "x_normal", has_normal_x, normal_x_val));
  pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> (test_point,
                                                                                            "y_normal", has_normal_y, normal_y_val));
  pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> (test_point,
                                                                                            "z_normal", has_normal_z, normal_z_val));

  vtkFloatArray* normals = vtkFloatArray::SafeDownCast (structured_grid->GetPointData ()->GetNormals ());

  if (has_x && has_y && has_z)
  {
    for (size_t i = 0; i < cloud.width; ++i)
    {
      for (size_t j = 0; j < cloud.height; ++j)
      {
        int queryPoint[3] = {i, j, 0};
        vtkIdType pointId = vtkStructuredData::ComputePointId (dimensions, queryPoint);
        float normal[3];
        if(structured_grid->IsPointVisible (pointId))
        {
          normals->GetTupleValue (i, normal);
          typename CloudT::PointType p = cloud (i, j);
          pcl::for_each_type<FieldList> (pcl::SetIfFieldExists<typename CloudT::PointType, float> (p, "normal_x", normal[0]));
          pcl::for_each_type<FieldList> (pcl::SetIfFieldExists<typename CloudT::PointType, float> (p, "normal_y", normal[1]));
          pcl::for_each_type<FieldList> (pcl::SetIfFieldExists<typename CloudT::PointType, float> (p, "normal_z", normal[2]));
          cloud (i, j) = p;
        }
        else
        {
          // Fill the point with an "empty" point?
        }
      }
    }
  }

  // Set the colors of the pcl::PointCloud (if the pcl::PointCloud supports colors and the input vtkStructuredGrid has colors)
  bool has_r = false; bool has_g = false; bool has_b = false;
  unsigned char r_val = 0.0f; unsigned char g_val = 0.0f; unsigned char b_val = 0.0f;
  pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, unsigned char> (test_point,
                                                                                            "r", has_r, r_val));
  pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, unsigned char> (test_point,
                                                                                            "g", has_g, g_val));
  pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, unsigned char> (test_point,
                                                                                            "b", has_b, b_val));
  vtkUnsignedCharArray* colors = vtkUnsignedCharArray::SafeDownCast(structured_grid->GetPointData()->GetArray("Colors"));

  if (has_r && has_g && has_b && colors)
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
          typename CloudT::PointType p = cloud (i, j);
          pcl::for_each_type<FieldList> (pcl::SetIfFieldExists<typename CloudT::PointType, float> (p, "r", color[0]));
          pcl::for_each_type<FieldList> (pcl::SetIfFieldExists<typename CloudT::PointType, float> (p, "g", color[1]));
          pcl::for_each_type<FieldList> (pcl::SetIfFieldExists<typename CloudT::PointType, float> (p, "b", color[2]));
          cloud (i, j) = p;
        }
        else
        {
          // Fill the point with an "empty" point?
        }
      }
    }
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::io::pointCloudTovtkPolyData (const pcl::PointCloud<PointT>& cloud, vtkPolyData* const pdata)
{
  typedef pcl::PointCloud<PointT> CloudT;

  typename CloudT::PointType test_point = cloud.points[0];

  typedef typename pcl::traits::fieldList<typename CloudT::PointType>::type FieldList;

  // Coordiantes (always must have coordinates)
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New ();
  for (size_t i = 0; i < cloud.points.size (); ++i)
  {
    double p[3];
    p[0] = cloud.points[i].x;
    p[1] = cloud.points[i].y;
    p[2] = cloud.points[i].z;
    points->InsertNextPoint (p);
  }

  // Create a temporary PolyData and add the points to it
  vtkSmartPointer<vtkPolyData> temp_polydata = vtkSmartPointer<vtkPolyData>::New ();
  temp_polydata->SetPoints (points);

  // Normals (optional)
  bool has_normal_x = false; bool has_normal_y = false; bool has_normal_z = false;
  float normal_x_val = 0.0f; float normal_y_val = 0.0f; float normal_z_val = 0.0f;
  pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> (test_point, "normal_x", has_normal_x, normal_x_val));
  pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> (test_point, "normal_y", has_normal_y, normal_y_val));
  pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> (test_point, "normal_z", has_normal_z, normal_z_val));
  if (has_normal_x && has_normal_y && has_normal_z)
  {
    vtkSmartPointer<vtkFloatArray> normals = vtkSmartPointer<vtkFloatArray>::New ();
    normals->SetNumberOfComponents (3); //3d normals (ie x,y,z)
    normals->SetNumberOfTuples (cloud.points.size ());
    normals->SetName ("Normals");

    for (size_t i = 0; i < cloud.points.size (); ++i)
    {
      typename CloudT::PointType p = cloud.points[i];
      pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> (p, "x_normal", has_normal_x, normal_x_val));
      pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> (p, "y_normal", has_normal_y, normal_y_val));
      pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> (p, "z_normal", has_normal_z, normal_z_val));
      float normal[3] = {normal_x_val, normal_y_val, normal_z_val};
      normals->SetTupleValue (i, normal);
    }
    temp_polydata->GetPointData ()->SetNormals (normals);
  }

  // Colors (optional)
  bool has_r = false; bool has_g = false; bool has_b = false;
  unsigned char r_val = 0; unsigned char g_val = 0; unsigned char b_val = 0;
  pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, unsigned char> (test_point, "r", has_r, r_val));
  pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, unsigned char> (test_point, "g", has_g, g_val));
  pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, unsigned char> (test_point, "b", has_b, b_val));
  if (has_r && has_g && has_b)
  {
    vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New ();
    colors->SetNumberOfComponents (3);
    colors->SetNumberOfTuples (cloud.points.size ());
    colors->SetName ("RGB");

    for (size_t i = 0; i < cloud.points.size (); ++i)
    {
      typename CloudT::PointType p = cloud[i];
      pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, unsigned char> (p, "r", has_r, r_val));
      pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, unsigned char> (p, "g", has_g, g_val));
      pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, unsigned char> (p, "b", has_b, b_val));
      unsigned char color[3] = {r_val, g_val, b_val};
      colors->SetTupleValue (i, color);
    }
    temp_polydata->GetPointData ()->SetScalars (colors);
  }

  // Add 0D topology to every point
  vtkSmartPointer<vtkVertexGlyphFilter> vertex_glyph_filter = vtkSmartPointer<vtkVertexGlyphFilter>::New ();
  #if VTK_MAJOR_VERSION <= 5
    vertex_glyph_filter->AddInputConnection (temp_polydata->GetProducerPort ());
  #else
    vertex_glyph_filter->SetInputData (temp_polydata);
  #endif
  vertex_glyph_filter->Update ();

  pdata->DeepCopy (vertex_glyph_filter->GetOutput ());
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::io::pointCloudTovtkStructuredGrid (const pcl::PointCloud<PointT>& cloud, vtkStructuredGrid* const structured_grid)
{
  typedef pcl::PointCloud<PointT> CloudT;

  typename CloudT::PointType test_point = cloud.points[0];

  typedef typename pcl::traits::fieldList<typename CloudT::PointType>::type FieldList;
  
  int dimensions[3] = {cloud.width, cloud.height, 1};
  structured_grid->SetDimensions (dimensions);

  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New ();
  points->SetNumberOfPoints (cloud.width * cloud.height);

  unsigned int numberOfInvalidPoints = 0;

  for (size_t i = 0; i < cloud.width; ++i)
  {
    for (size_t j = 0; j < cloud.height; ++j)
    {
      int queryPoint[3] = {i, j, 0};
      vtkIdType pointId = vtkStructuredData::ComputePointId (dimensions, queryPoint);
      typename CloudT::PointType point = cloud (i, j);

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

  // Normals (optional)
  bool has_normal_x = false; bool has_normal_y = false; bool has_normal_z = false;
  float normal_x_val = 0.0f; float normal_y_val = 0.0f; float normal_z_val = 0.0f;
  pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> (test_point, "normal_x", has_normal_x, normal_x_val));
  pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> (test_point, "normal_y", has_normal_y, normal_y_val));
  pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> (test_point, "normal_z", has_normal_z, normal_z_val));
  if (has_normal_x && has_normal_y && has_normal_z)
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
        typename CloudT::PointType point = cloud (i, j);

        if (pcl::isFinite (point))
        {
          typename CloudT::PointType p = cloud.points[i];
          pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> (p, "x_normal", has_normal_x, normal_x_val));
          pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> (p, "y_normal", has_normal_y, normal_y_val));
          pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> (p, "z_normal", has_normal_z, normal_z_val));
          float normal[3] = {normal_x_val, normal_y_val, normal_z_val};
          normals->SetTupleValue (pointId, normal);
        }
        else
        {
        }
      }
    }

    structured_grid->GetPointData ()->SetNormals (normals);
  }

  // Colors (optional)
  bool has_r = false; bool has_g = false; bool has_b = false;
  unsigned char r_val = 0; unsigned char g_val = 0; unsigned char b_val = 0;
  pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, unsigned char> (test_point, "r", has_r, r_val));
  pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, unsigned char> (test_point, "g", has_g, g_val));
  pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, unsigned char> (test_point, "b", has_b, b_val));
  if (has_r && has_g && has_b)
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
        typename CloudT::PointType point = cloud (i, j);

        if (pcl::isFinite (point))
        {
          typename CloudT::PointType p = cloud[i];
          pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, unsigned char> (p, "r", has_r, r_val));
          pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, unsigned char> (p, "g", has_g, g_val));
          pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, unsigned char> (p, "b", has_b, b_val));
          unsigned char color[3] = {r_val, g_val, b_val};
          colors->SetTupleValue (i, color);
        }
        else
        {
        }
      }
    }
    structured_grid->GetPointData ()->AddArray (colors);
  }
}

#endif  //#ifndef PCL_IO_VTK_IO_H_

