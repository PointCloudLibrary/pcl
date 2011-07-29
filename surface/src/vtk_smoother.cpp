/*
 * Software License Agreement (BSD License)
 *
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
 */

#include <pcl/surface/vtk_smoother.h>
#include <pcl/point_types.h>
#include <pcl/pcl_base.h>
#include <pcl/ros/conversions.h>
#include <pcl/common/common.h>
#include <vtkCellArray.h>
#include <vtkPoints.h>
#include <vtkTriangleFilter.h>
#include <vtkWindowedSincPolyDataFilter.h>
#include <vtkSmoothPolyDataFilter.h>
#include <vtkPolyData.h>
#include <vtkLinearSubdivisionFilter.h>
#include <vtkLoopSubdivisionFilter.h>
#include <vtkButterflySubdivisionFilter.h>
#include <vtkPolyDataWriter.h>

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::surface::VtkSmoother::VtkSmoother()
{
  vtk_polygons = vtkPolyData::New ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::surface::VtkSmoother::~VtkSmoother(){}

//////////////////////////////////////////////////////////////////////////////////////////////
int
pcl::surface::VtkSmoother::convertToVTK (const pcl::PolygonMesh &triangles)
{
  if (triangles.cloud.data.empty ())
  {
    PCL_ERROR ("[pcl::surface::convertToVTK] Input point cloud has no data!\n");
    return (-1);
  }
  vtkPoints *vtk_pts = vtkPoints::New ();

  int nr_points  = triangles.cloud.width * triangles.cloud.height;
  int point_size = triangles.cloud.data.size () / nr_points;
  for (int i = 0; i < nr_points; ++i)
  {
    int xyz = 0;
    float value[3];
    for (size_t d = 0; d < triangles.cloud.fields.size (); ++d)
    {
      int count = triangles.cloud.fields[d].count;
      if (count == 0)
        count = 1;
      int c = 0;
      if ((triangles.cloud.fields[d].datatype == sensor_msgs::PointField::FLOAT32) && (
          triangles.cloud.fields[d].name == "x" ||
          triangles.cloud.fields[d].name == "y" ||
          triangles.cloud.fields[d].name == "z"))
      {
        memcpy (&value[xyz], &triangles.cloud.data[i * point_size + triangles.cloud.fields[d].offset + c * sizeof (float)], sizeof (float));
        if (++xyz == 3)
          break;
      }
    }
    if (xyz != 3)
    {
      PCL_ERROR ("[pcl::io::saveVTKFile] Input point cloud has no XYZ data!\n");
      return (-2);
    }
    vtk_pts->InsertPoint(i,value);
  }

  vtkCellArray *vtk_cells = vtkCellArray::New ();
  for (size_t i = 0; i < triangles.polygons.size (); ++i)
  {
    vtk_cells->InsertNextCell (triangles.polygons[i].vertices.size ());
    size_t j = 0;
    for (j = 0; j < triangles.polygons[i].vertices.size (); ++j)
      vtk_cells->InsertCellPoint (triangles.polygons[i].vertices[j]);
  }

  vtk_polygons->SetPoints(vtk_pts);
  vtk_polygons->SetPolys(vtk_cells);
  vtk_polygons->Update();

  vtkTriangleFilter *vtk_triangles = vtkTriangleFilter::New ();
  vtk_triangles->SetInput (vtk_polygons);
  vtk_triangles->Update();

  vtk_polygons = vtk_triangles->GetOutput ();

  return 1;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::surface::VtkSmoother::subdivideMesh(int filter)
{
  vtkPolyDataAlgorithm *vtk_subdivision_filter;
  switch(filter)
  {
    case 0:
      vtk_subdivision_filter = vtkLinearSubdivisionFilter::New ();
      break;
    case 1:
      vtk_subdivision_filter = vtkLoopSubdivisionFilter::New ();
      break;
    case 2:
      vtk_subdivision_filter = vtkButterflySubdivisionFilter::New ();
      break;
    default:
      PCL_ERROR ("[pcl::surface::subdivideMesh] Invalid filter selection!\n");
      return;
      break;
  }

  vtk_subdivision_filter->SetInput (vtk_polygons);
  vtk_subdivision_filter->Update ();

  vtk_polygons = vtk_subdivision_filter->GetOutput ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::surface::VtkSmoother::smoothMeshWindowedSinc(int num_iter, float feature_angle, float pass_band)
{
  vtkWindowedSincPolyDataFilter *vtk_smoother = vtkWindowedSincPolyDataFilter::New ();
  vtk_smoother->SetInput (vtk_polygons);
  vtk_smoother->SetNumberOfIterations (num_iter);
  vtk_smoother->SetFeatureAngle (feature_angle);
  vtk_smoother->SetPassBand (pass_band);
  vtk_smoother->BoundarySmoothingOff ();
  vtk_smoother->FeatureEdgeSmoothingOff ();
  vtk_smoother->NonManifoldSmoothingOff ();
  vtk_smoother->NormalizeCoordinatesOn ();
  vtk_smoother->Update ();

  vtk_polygons = vtk_smoother->GetOutput ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::surface::VtkSmoother::smoothMeshLaplacian(int num_iter)
{
  vtkSmoothPolyDataFilter *vtk_smoother = vtkSmoothPolyDataFilter::New ();
  vtk_smoother->SetInput (vtk_polygons);
  vtk_smoother->SetNumberOfIterations (num_iter);
  vtk_smoother->Update ();

  vtk_polygons = vtk_smoother->GetOutput ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::surface::VtkSmoother::convertToPCL(pcl::PolygonMesh &triangles)
{

  pcl::PointCloud < pcl::PointXYZ > cloud;
  cloud.points.resize (vtk_polygons->GetNumberOfPoints ());
  for (vtkIdType i = 0; i < vtk_polygons->GetNumberOfPoints (); ++i)
  {
    double p[3];
    vtk_polygons->GetPoint (i, p);
    cloud.points[i].x = p[0];
    cloud.points[i].y = p[1];
    cloud.points[i].z = p[2];
  }
  pcl::toROSMsg (cloud, triangles.cloud);
  triangles.polygons.resize (vtk_polygons->GetNumberOfCells ());

  vtkCellArray *vtk_newcells = vtk_polygons->GetPolys ();
  vtk_newcells->InitTraversal ();

  for (vtkIdType i = 0; i < vtk_newcells->GetNumberOfCells (); ++i)
  {
    pcl::Vertices v;
    vtkIdType num_points = 0;
    vtkIdType *points = 0;
    vtk_newcells->GetNextCell (num_points, points);
    v.vertices.resize (num_points);
    for (vtkIdType j = 0; j < num_points; ++j)
    {
      v.vertices[j] = points[j];
    }
    triangles.polygons[i] = v;

  }
}
