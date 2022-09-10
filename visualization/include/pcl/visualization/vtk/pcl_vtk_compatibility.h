/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2020-, Open Perception
 *
 *  All rights reserved
 */

#include <vtkCellArray.h>

#ifdef VTK_CELL_ARRAY_V2
  using vtkCellPtsPtr = vtkIdType const*;
#else
  using vtkCellPtsPtr = vtkIdType*;
#endif

