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

#pragma once

// C++
#include <iostream>
#include <string>

// PCL
#include "object.h"

// VTK
#include <vtkVersion.h>
#include <vtkActor.h>
#include <vtkTubeFilter.h>
#include <vtkAxes.h>
//#include <vtkDataSetMapper.h>
#include <vtkFloatArray.h>
#include <vtkProperty.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkSmartPointer.h>

class Axes : public Object
{
public:

  // Operators
  // -----------------------------------------------------------------------------
  Axes (std::string name, float size = 1.0) :
      Object (name)
  {
    axes_ = vtkSmartPointer<vtkAxes>::New ();
    axes_->SetOrigin (0, 0, 0);
    axes_->SetScaleFactor (size);
    axes_->Update ();

    vtkSmartPointer<vtkFloatArray> axes_colors = vtkSmartPointer<vtkFloatArray>::New ();
    axes_colors->Allocate (6);
    axes_colors->InsertNextValue (0.0);
    axes_colors->InsertNextValue (0.0);
    axes_colors->InsertNextValue (0.5);
    axes_colors->InsertNextValue (0.5);
    axes_colors->InsertNextValue (1.0);
    axes_colors->InsertNextValue (1.0);

    vtkSmartPointer<vtkPolyData> axes_data = axes_->GetOutput ();
    axes_data->GetPointData ()->SetScalars (axes_colors);

    vtkSmartPointer<vtkTubeFilter> axes_tubes = vtkSmartPointer<vtkTubeFilter>::New ();
    axes_tubes->SetInputData (axes_data);
    axes_tubes->SetRadius (axes_->GetScaleFactor () / 100.0);
    axes_tubes->SetNumberOfSides (6);

    vtkSmartPointer<vtkPolyDataMapper> axes_mapper = vtkSmartPointer<vtkPolyDataMapper>::New ();
    axes_mapper->SetScalarModeToUsePointData ();
    axes_mapper->SetInputData (axes_tubes->GetOutput ());

    axes_actor_ = vtkSmartPointer<vtkActor>::New ();
    axes_actor_->GetProperty ()->SetLighting (false);
    axes_actor_->SetMapper (axes_mapper);

    addActor (axes_actor_);
  }
  //~Axes () { }

  // Accessors
  // -----------------------------------------------------------------------------
  inline vtkSmartPointer<vtkAxes>
  getAxes () const
  {
    return axes_;
  }

  vtkSmartPointer<vtkActor>
  getAxesActor () const
  {
    return axes_actor_;
  }

private:

  // Members
  // -----------------------------------------------------------------------------
  vtkSmartPointer<vtkAxes> axes_;
  vtkSmartPointer<vtkActor> axes_actor_;

};
