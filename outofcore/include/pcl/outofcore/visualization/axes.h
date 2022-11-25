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
      Object (name), axes_ (vtkSmartPointer<vtkAxes>::New ())
  {
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
