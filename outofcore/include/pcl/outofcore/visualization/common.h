#pragma once

// VTK
#include <vtkVersion.h>
#if VTK_MAJOR_VERSION == 9 && VTK_MINOR_VERSION == 0
#include <limits> // This must be included before vtkPolyData.h
#endif
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>

vtkSmartPointer<vtkPolyData>
getVtkCube (double x_min, double x_max, double y_min, double y_max, double z_min, double z_max);
