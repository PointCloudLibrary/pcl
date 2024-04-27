// PCL
#include <pcl/outofcore/visualization/common.h>

// VTK
#include <vtkCubeSource.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>

vtkSmartPointer<vtkPolyData>
getVtkCube (
    double x_min, double x_max, double y_min, double y_max, double z_min, double z_max)
{
  vtkSmartPointer<vtkCubeSource> cube = vtkSmartPointer<vtkCubeSource>::New();
  cube->SetBounds (x_min, x_max, y_min, y_max, z_min, z_max);
  return cube->GetOutput();
}
