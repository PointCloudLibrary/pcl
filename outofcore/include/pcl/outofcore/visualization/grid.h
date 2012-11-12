#ifndef PCL_OUTOFCORE_GRID_H_
#define PCL_OUTOFCORE_GRID_H_

// C++
#include <iostream>
#include <string>

// PCL
#include "geometry.h"
#include "object.h"

// VTK
#include <vtkActor.h>
#include <vtkRectilinearGrid.h>
#include <vtkDataSetMapper.h>
#include <vtkDoubleArray.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>

//class Grid : public Geometry
class Grid : public Object
{
public:

  // Operators
  // -----------------------------------------------------------------------------
  Grid (std::string name, int size = 10, double spacing = 1.0);
  ~Grid () { }

  // Accessors
  // -----------------------------------------------------------------------------
  inline vtkSmartPointer<vtkRectilinearGrid>
  getGrid () const
  {
    return grid_;
  }

//  virtual vtkSmartPointer<vtkActor>
  vtkSmartPointer<vtkActor>
  getGridActor () const
  {
    return grid_actor_;
  }

private:

  // Members
  // -----------------------------------------------------------------------------
  vtkSmartPointer<vtkRectilinearGrid> grid_;
  vtkSmartPointer<vtkActor> grid_actor_;

};

#endif
