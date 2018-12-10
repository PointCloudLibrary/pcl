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
#include <vtkVersion.h>
#include <vtkTriangleFilter.h>
#include <pcl/io/vtk_io.h>

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
  pcl::io::mesh2vtk (triangles, vtk_polygons);

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
  pcl::io::vtk2mesh (vtk_polygons, triangles);
}

//////////////////////////////////////////////////////////////////////////////////////////////
int
pcl::VTKUtils::vtk2mesh (const vtkSmartPointer<vtkPolyData>& poly_data, pcl::PolygonMesh& mesh)
{
  return pcl::io::vtk2mesh(poly_data, mesh);
}

//////////////////////////////////////////////////////////////////////////////////////////////
int
pcl::VTKUtils::mesh2vtk (const pcl::PolygonMesh& mesh, vtkSmartPointer<vtkPolyData> &poly_data)
{
  return pcl::io::mesh2vtk(mesh, poly_data);
}


