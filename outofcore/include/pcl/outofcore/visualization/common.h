#pragma once

// VTK
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>

vtkSmartPointer<vtkPolyData>
getVtkCube (double x_min, double x_max, double y_min, double y_max, double z_min, double z_max);
