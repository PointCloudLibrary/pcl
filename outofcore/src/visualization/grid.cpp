/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception, Inc.
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


// PCL
#include <pcl/outofcore/visualization/geometry.h>
#include <pcl/outofcore/visualization/grid.h>

// VTK
#include <vtkVersion.h>
#include <vtkActor.h>
#include <vtkRectilinearGrid.h>
#include <vtkDoubleArray.h>
#include <vtkProperty.h>
#include <vtkSmartPointer.h>

// Operators
// -----------------------------------------------------------------------------
Grid::Grid (std::string name, int size/*=10*/, double spacing/*=1.0*/) :
    Object (name)
{
  grid_ = vtkSmartPointer<vtkRectilinearGrid>::New ();
  grid_actor_ = vtkSmartPointer<vtkActor>::New ();

  vtkSmartPointer<vtkDataSetMapper> grid_mapper = vtkSmartPointer<vtkDataSetMapper>::New ();

  vtkSmartPointer<vtkDoubleArray> xz_array = vtkSmartPointer<vtkDoubleArray>::New ();
  vtkSmartPointer<vtkDoubleArray> y_array = vtkSmartPointer<vtkDoubleArray>::New ();

  size++;

  // Create a grid
  grid_->SetDimensions (size, 1, size);

  // Fill arrays
  for (int i = -size / 2; i <= size / 2; i++)
    xz_array->InsertNextValue ((double)i * spacing);
  y_array->InsertNextValue (0.0);

  grid_->SetXCoordinates (xz_array);
  grid_->SetYCoordinates (y_array);
  grid_->SetZCoordinates (xz_array);

#if VTK_MAJOR_VERSION < 6
  grid_mapper->SetInputConnection (grid_->GetProducerPort ());
#else
  grid_mapper->SetInputData(grid_);
#endif

  vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New ();
  grid_actor_->SetMapper (grid_mapper);

  grid_actor_->GetProperty ()->SetRepresentationToWireframe ();
  grid_actor_->GetProperty ()->SetColor (0.5, 0.5, 0.5);
  grid_actor_->GetProperty ()->SetLighting (false);

  addActor (grid_actor_);
}
