// PCL
#include <pcl/outofcore/visualization/geometry.h>
#include <pcl/outofcore/visualization/grid.h>

// VTK
#include <vtkActor.h>
#include <vtkRectilinearGrid.h>
#include <vtkDoubleArray.h>
#include <vtkProperty.h>
#include <vtkSmartPointer.h>

// Operators
// -----------------------------------------------------------------------------
Grid::Grid (std::string name, int size/*=10*/, double spacing/*=1.0*/) :
    Object (name), grid_ (vtkSmartPointer<vtkRectilinearGrid>::New ()), grid_actor_ (vtkSmartPointer<vtkActor>::New ())
{
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

  grid_mapper->SetInputData(grid_);

  vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New ();
  grid_actor_->SetMapper (grid_mapper);

  grid_actor_->GetProperty ()->SetRepresentationToWireframe ();
  grid_actor_->GetProperty ()->SetColor (0.5, 0.5, 0.5);
  grid_actor_->GetProperty ()->SetLighting (false);

  addActor (grid_actor_);
}
