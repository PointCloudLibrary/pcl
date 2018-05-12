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

#include <pcl/surface/vtk_smoothing/vtk_mesh_quadric_decimation.h>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>

#include <vtkVersion.h>
#include <vtkQuadricDecimation.h>

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::MeshQuadricDecimationVTK::MeshQuadricDecimationVTK ()
  : pcl::MeshProcessing (),
    target_reduction_factor_ (0.5f)
{
}


//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::MeshQuadricDecimationVTK::performProcessing (pcl::PolygonMesh &output)
{
  // Convert from PCL mesh representation to the VTK representation
  VTKUtils::convertToVTK (*input_mesh_, vtk_polygons_);

  // Apply the VTK algorithm
  vtkSmartPointer<vtkQuadricDecimation> vtk_quadric_decimation_filter = vtkSmartPointer<vtkQuadricDecimation>::New();
  vtk_quadric_decimation_filter->SetTargetReduction (target_reduction_factor_);
#if VTK_MAJOR_VERSION < 6
  vtk_quadric_decimation_filter->SetInput (vtk_polygons_);
#else
  vtk_quadric_decimation_filter->SetInputData (vtk_polygons_);
#endif
  vtk_quadric_decimation_filter->Update ();

  vtk_polygons_ = vtk_quadric_decimation_filter->GetOutput ();

  // Convert the result back to the PCL representation
  VTKUtils::convertToPCL (vtk_polygons_, output);
}
