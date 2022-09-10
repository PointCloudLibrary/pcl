/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2011, Willow Garage, Inc.
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
 * $Id$
 *
 */


#include <pcl/surface/vtk_smoothing/vtk_utils.h>

#include <pcl/PolygonMesh.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h> // for PointXYZ, PointXYZRGB, RGB
#include <vtkCellArray.h>
#include <vtkTriangleFilter.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPointData.h>
#include <vtkFloatArray.h>
#include <vtkUnsignedCharArray.h>

// Support for VTK 7.1 upwards
#ifdef vtkGenericDataArray_h
#define SetTupleValue SetTypedTuple
#define InsertNextTupleValue InsertNextTypedTuple
#define GetTupleValue GetTypedTuple
#endif

//////////////////////////////////////////////////////////////////////////////////////////////
int
pcl::VTKUtils::convertToVTK (const pcl::PolygonMesh &triangles, vtkSmartPointer<vtkPolyData> &triangles_out_vtk)
{
  if (triangles.cloud.data.empty ())
  {
    PCL_ERROR ("[pcl::surface::convertToVTK] Input point cloud has no data!\n");
    return (-1);
  }

  vtkSmartPointer<vtkPolyData> vtk_polygons;
  mesh2vtk (triangles, vtk_polygons);

  vtkSmartPointer<vtkTriangleFilter> vtk_triangles = vtkSmartPointer<vtkTriangleFilter>::New ();
  vtk_triangles->SetInputData (vtk_polygons);
  vtk_triangles->Update();

  triangles_out_vtk = vtk_triangles->GetOutput ();
  return 1;
}


//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::VTKUtils::convertToPCL (vtkSmartPointer<vtkPolyData> &vtk_polygons, pcl::PolygonMesh &triangles)
{
  vtk2mesh (vtk_polygons, triangles);
}

//////////////////////////////////////////////////////////////////////////////////////////////
int
pcl::VTKUtils::vtk2mesh (const vtkSmartPointer<vtkPolyData>& poly_data, pcl::PolygonMesh& mesh)
{
  mesh.polygons.clear ();
  mesh.cloud.data.clear ();
  mesh.cloud.width = mesh.cloud.height = 0;
  mesh.cloud.is_dense = true;

  vtkSmartPointer<vtkPoints> mesh_points = poly_data->GetPoints ();
  vtkIdType nr_points = mesh_points->GetNumberOfPoints ();
  vtkIdType nr_polygons = poly_data->GetNumberOfPolys ();

  if (nr_points == 0)
    return 0;

  vtkUnsignedCharArray* poly_colors = nullptr;
  if (poly_data->GetPointData() != nullptr)
    poly_colors = vtkUnsignedCharArray::SafeDownCast (poly_data->GetPointData ()->GetScalars ("Colors"));

  // Some applications do not save the name of scalars (including PCL's native vtk_io)
  if (!poly_colors)
    poly_colors = vtkUnsignedCharArray::SafeDownCast (poly_data->GetPointData ()->GetScalars ("scalars"));

  // TODO: currently only handles rgb values with 3 components
  if (poly_colors && (poly_colors->GetNumberOfComponents () == 3))
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_temp (new pcl::PointCloud<pcl::PointXYZRGB>());
    cloud_temp->points.resize (nr_points);
    double point_xyz[3];
    unsigned char point_color[3];
    for (vtkIdType i = 0; i < mesh_points->GetNumberOfPoints (); ++i)
    {
      mesh_points->GetPoint (i, &point_xyz[0]);
      (*cloud_temp)[i].x = static_cast<float> (point_xyz[0]);
      (*cloud_temp)[i].y = static_cast<float> (point_xyz[1]);
      (*cloud_temp)[i].z = static_cast<float> (point_xyz[2]);

      poly_colors->GetTupleValue (i, &point_color[0]);
      (*cloud_temp)[i].r = point_color[0];
      (*cloud_temp)[i].g = point_color[1];
      (*cloud_temp)[i].b = point_color[2];
    }
    cloud_temp->width = cloud_temp->size ();
    cloud_temp->height = 1;
    cloud_temp->is_dense = true;

    pcl::toPCLPointCloud2 (*cloud_temp, mesh.cloud);
  }
  else // in case points do not have color information:
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp (new pcl::PointCloud<pcl::PointXYZ> ());
    cloud_temp->points.resize (nr_points);
    double point_xyz[3];
    for (vtkIdType i = 0; i < mesh_points->GetNumberOfPoints (); ++i)
    {
      mesh_points->GetPoint (i, &point_xyz[0]);
      (*cloud_temp)[i].x = static_cast<float> (point_xyz[0]);
      (*cloud_temp)[i].y = static_cast<float> (point_xyz[1]);
      (*cloud_temp)[i].z = static_cast<float> (point_xyz[2]);
    }
    cloud_temp->width = cloud_temp->size ();
    cloud_temp->height = 1;
    cloud_temp->is_dense = true;

    pcl::toPCLPointCloud2 (*cloud_temp, mesh.cloud);
  }

  mesh.polygons.resize (nr_polygons);
#ifdef VTK_CELL_ARRAY_V2
  vtkIdType const *cell_points;
#else
  vtkIdType* cell_points;
#endif
  vtkIdType nr_cell_points;
  vtkCellArray * mesh_polygons = poly_data->GetPolys ();
  mesh_polygons->InitTraversal ();
  int id_poly = 0;
  while (mesh_polygons->GetNextCell (nr_cell_points, cell_points))
  {
    mesh.polygons[id_poly].vertices.resize (nr_cell_points);
    for (vtkIdType i = 0; i < nr_cell_points; ++i)
      mesh.polygons[id_poly].vertices[i] = static_cast<unsigned int> (cell_points[i]);
    ++id_poly;
  }

  return (static_cast<int> (nr_points));
}

//////////////////////////////////////////////////////////////////////////////////////////////
int
pcl::VTKUtils::mesh2vtk (const pcl::PolygonMesh& mesh, vtkSmartPointer<vtkPolyData> &poly_data)
{
  int nr_points = mesh.cloud.width * mesh.cloud.height;
  int nr_polygons = static_cast<int> (mesh.polygons.size ());

  // reset vtkPolyData object
  poly_data = vtkSmartPointer<vtkPolyData>::New (); // OR poly_data->Reset();
  vtkSmartPointer<vtkPoints> vtk_mesh_points = vtkSmartPointer<vtkPoints>::New ();
  vtkSmartPointer<vtkCellArray> vtk_mesh_polygons = vtkSmartPointer<vtkCellArray>::New ();
  poly_data->SetPoints (vtk_mesh_points);

  // get field indices for x, y, z (as well as rgb and/or rgba)
  int idx_x = -1, idx_y = -1, idx_z = -1, idx_rgb = -1, idx_rgba = -1, idx_normal_x = -1, idx_normal_y = -1, idx_normal_z = -1;
  for (int d = 0; d < static_cast<int> (mesh.cloud.fields.size ()); ++d)
  {
    if (mesh.cloud.fields[d].name == "x") idx_x = d;
    else if (mesh.cloud.fields[d].name == "y") idx_y = d;
    else if (mesh.cloud.fields[d].name == "z") idx_z = d;
    else if (mesh.cloud.fields[d].name == "rgb") idx_rgb = d;
    else if (mesh.cloud.fields[d].name == "rgba") idx_rgba = d;
    else if (mesh.cloud.fields[d].name == "normal_x") idx_normal_x = d;
    else if (mesh.cloud.fields[d].name == "normal_y") idx_normal_y = d;
    else if (mesh.cloud.fields[d].name == "normal_z") idx_normal_z = d;
  }
  if ( ( idx_x == -1 ) || ( idx_y == -1 ) || ( idx_z == -1 ) )
    nr_points = 0;

  // copy point data
  vtk_mesh_points->SetNumberOfPoints (nr_points);
  if (nr_points > 0)
  {
    Eigen::Vector4f pt = Eigen::Vector4f::Zero ();
    Eigen::Array4i xyz_offset (mesh.cloud.fields[idx_x].offset, mesh.cloud.fields[idx_y].offset, mesh.cloud.fields[idx_z].offset, 0);
    for (vtkIdType cp = 0; cp < nr_points; ++cp, xyz_offset += mesh.cloud.point_step)
    {
      memcpy(&pt[0], &mesh.cloud.data[xyz_offset[0]], sizeof(float));
      memcpy(&pt[1], &mesh.cloud.data[xyz_offset[1]], sizeof(float));
      memcpy(&pt[2], &mesh.cloud.data[xyz_offset[2]], sizeof(float));
      vtk_mesh_points->InsertPoint(cp, pt[0], pt[1], pt[2]);
    }
  }

  // copy polygon data
  if (nr_polygons > 0)
  {
    for (int i = 0; i < nr_polygons; i++)
    {
      auto nr_points_in_polygon = static_cast<unsigned int> (mesh.polygons[i].vertices.size ());
      vtk_mesh_polygons->InsertNextCell (nr_points_in_polygon);
      for (unsigned int j = 0; j < nr_points_in_polygon; j++)
        vtk_mesh_polygons->InsertCellPoint(mesh.polygons[i].vertices[j]);
    }
    poly_data->SetPolys (vtk_mesh_polygons);
  }

  // copy color information
  if (idx_rgb != -1 || idx_rgba != -1)
  {
    vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New ();
    colors->SetNumberOfComponents (3);
    colors->SetName ("Colors");
    pcl::RGB rgb;
    int offset = (idx_rgb != -1) ? mesh.cloud.fields[idx_rgb].offset : mesh.cloud.fields[idx_rgba].offset;
    for (vtkIdType cp = 0; cp < nr_points; ++cp)
    {
      memcpy (&rgb, &mesh.cloud.data[cp * mesh.cloud.point_step + offset], sizeof (pcl::RGB));
      const unsigned char color[3] = {rgb.r, rgb.g, rgb.b};
      colors->InsertNextTupleValue (color);
    }
    poly_data->GetPointData ()->SetScalars (colors);
  }

  // copy normal information
  if ( ( idx_normal_x != -1 ) && ( idx_normal_y != -1 ) && ( idx_normal_z != -1 ) )
  {
    vtkSmartPointer<vtkFloatArray> normals = vtkSmartPointer<vtkFloatArray>::New ();
    normals->SetNumberOfComponents (3);
    float nx = 0.0f, ny = 0.0f, nz = 0.0f;
    for (vtkIdType cp = 0; cp < nr_points; ++cp)
    {
      memcpy (&nx, &mesh.cloud.data[cp*mesh.cloud.point_step+mesh.cloud.fields[idx_normal_x].offset], sizeof(float));
      memcpy (&ny, &mesh.cloud.data[cp*mesh.cloud.point_step+mesh.cloud.fields[idx_normal_y].offset], sizeof(float));
      memcpy (&nz, &mesh.cloud.data[cp*mesh.cloud.point_step+mesh.cloud.fields[idx_normal_z].offset], sizeof(float));
      const float normal[3] = {nx, ny, nz};
      normals->InsertNextTupleValue (normal);
    }
    poly_data->GetPointData()->SetNormals (normals);
  }

  if (poly_data->GetPoints() == nullptr)
    return (0);
  return (static_cast<int> (poly_data->GetPoints()->GetNumberOfPoints ()));
}


