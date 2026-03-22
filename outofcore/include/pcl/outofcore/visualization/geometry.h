#pragma once

// C++
#include <string>
#include <iostream>

// PCL
#include "object.h"

// VTK
#include <vtkActor.h>
#include <vtkSmartPointer.h>

class Geometry : public Object
{
protected:

  // Operators
  // -----------------------------------------------------------------------------
  Geometry (std::string name) :
      Object (name)
  {
  }

public:

  
  ~Geometry () override = default;

public:

  // Accessors
  // -----------------------------------------------------------------------------
  virtual vtkSmartPointer<vtkActor>
  getActor () const
  {
    std::cout << "Get Geometry Actor" << std::endl;
    return nullptr;
  }

};
