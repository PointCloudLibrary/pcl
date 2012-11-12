#ifndef PCL_OUTOFCORE_GEOMETRY_H_
#define PCL_OUTOFCORE_GEOMETRY_H_

// C++
#include <string>

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

  virtual
  ~Geometry () { }

public:

  // Accessors
  // -----------------------------------------------------------------------------
  virtual vtkSmartPointer<vtkActor>
  getActor () const
  {
    std::cout << "Get Geometry Actor" << std::endl;
    return NULL;
  }

};

#endif
