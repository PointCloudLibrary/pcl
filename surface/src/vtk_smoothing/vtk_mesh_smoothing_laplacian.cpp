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

#include <pcl/surface/vtk_smoothing/vtk_mesh_smoothing_laplacian.h>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>

#include <vtkVersion.h>
#include <vtkSmoothPolyDataFilter.h>


//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::MeshSmoothingLaplacianVTK::performProcessing (pcl::PolygonMesh &output)
{
  // Convert from PCL mesh representation to the VTK representation
  VTKUtils::convertToVTK (*input_mesh_, vtk_polygons_);

  // Apply the VTK algorithm
  vtkSmartPointer<vtkSmoothPolyDataFilter> vtk_smoother = vtkSmartPointer<vtkSmoothPolyDataFilter>::New ();
#if VTK_MAJOR_VERSION < 6
  vtk_smoother->SetInput (vtk_polygons_);
#else
  vtk_smoother->SetInputData (vtk_polygons_);
#endif
  vtk_smoother->SetNumberOfIterations (num_iter_);
  if (convergence_ != 0.0f)
    vtk_smoother->SetConvergence (convergence_);
  vtk_smoother->SetRelaxationFactor (relaxation_factor_);
  vtk_smoother->SetFeatureEdgeSmoothing (feature_edge_smoothing_);
  vtk_smoother->SetFeatureAngle (feature_angle_);
  vtk_smoother->SetEdgeAngle (edge_angle_);
  vtk_smoother->SetBoundarySmoothing (boundary_smoothing_);
  vtk_smoother->Update ();

  vtk_polygons_ = vtk_smoother->GetOutput ();

  // Convert the result back to the PCL representation
  VTKUtils::convertToPCL (vtk_polygons_, output);
}
