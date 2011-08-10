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
 * $Id$
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
  pcl::io::mesh2vtk(triangles, vtk_polygons);

  vtkSmartPointer<vtkTriangleFilter> vtk_triangles = vtkTriangleFilter::New ();
  vtk_triangles->SetInput (vtk_polygons);
  vtk_triangles->Update();

  vtk_polygons = vtk_triangles->GetOutput ();

  return 1;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::surface::VtkSmoother::subdivideMesh(int filter)
{
  vtkSmartPointer<vtkPolyDataAlgorithm> vtk_subdivision_filter;
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
  vtkSmartPointer<vtkWindowedSincPolyDataFilter> vtk_smoother = vtkWindowedSincPolyDataFilter::New ();
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
  vtkSmartPointer<vtkSmoothPolyDataFilter> vtk_smoother = vtkSmoothPolyDataFilter::New ();
  vtk_smoother->SetInput (vtk_polygons);
  vtk_smoother->SetNumberOfIterations (num_iter);
  vtk_smoother->Update ();

  vtk_polygons = vtk_smoother->GetOutput ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::surface::VtkSmoother::convertToPCL(pcl::PolygonMesh &triangles)
{
  pcl::io::vtk2mesh(vtk_polygons, triangles);
}
