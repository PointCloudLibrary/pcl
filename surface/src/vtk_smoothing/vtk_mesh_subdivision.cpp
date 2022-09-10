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
#include <pcl/console/print.h>
#include <pcl/surface/vtk_smoothing/vtk_mesh_subdivision.h>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>

#include <vtkLinearSubdivisionFilter.h>
#include <vtkLoopSubdivisionFilter.h>
#include <vtkButterflySubdivisionFilter.h>


//////////////////////////////////////////////////////////////////////////////////////////////
pcl::MeshSubdivisionVTK::MeshSubdivisionVTK ()
  : filter_type_ (LINEAR)
{
}


//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::MeshSubdivisionVTK::performProcessing (pcl::PolygonMesh &output)
{
  // Convert from PCL mesh representation to the VTK representation
  VTKUtils::convertToVTK (*input_mesh_, vtk_polygons_);

  // Apply the VTK algorithm
  vtkSmartPointer<vtkPolyDataAlgorithm> vtk_subdivision_filter;
  switch(filter_type_)
  {
    case LINEAR:
      vtk_subdivision_filter = vtkLinearSubdivisionFilter::New ();
      break;
    case LOOP:
      vtk_subdivision_filter = vtkLoopSubdivisionFilter::New ();
      break;
    case BUTTERFLY:
      vtk_subdivision_filter = vtkButterflySubdivisionFilter::New ();
      break;
    default:
      PCL_ERROR ("[pcl::surface::VTKSmoother::subdivideMesh] Invalid filter selection!\n");
      return;
      break;
  }

  vtk_subdivision_filter->SetInputData (vtk_polygons_);
  vtk_subdivision_filter->Update ();

  vtk_polygons_ = vtk_subdivision_filter->GetOutput ();

  // Convert the result back to the PCL representation
  VTKUtils::convertToPCL (vtk_polygons_, output);
}
