/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2020-, Open Perception
 *
 *  All rights reserved
 */
#include <pcl/pcl_macros.h>
#include <pcl/pcl_config.h>

#if HAVE_QVTK
#include <vtkVersion.h>
#include <vtkRenderWindow.h>

#if VTK_MAJOR_VERSION > 8
  #include <QVTKOpenGLNativeWidget.h>
  using PCLQVTKWidget = QVTKOpenGLNativeWidget;
#else 
  #include <QVTKWidget.h>
  using PCLQVTKWidget = QVTKWidget;
#endif // VTK_MAJOR_VERSION > 8


inline auto PCL_EXPORTS getInteractorCompat(PCLQVTKWidget& qvtk) {
#if VTK_MAJOR_VERSION > 8
  return qvtk.interactor();
#else
  return qvtk.GetInteractor();
#endif // VTK_MAJOR_VERSION > 8
}

inline auto PCL_EXPORTS getRenderWindowCompat(PCLQVTKWidget& qvtk) {
#if VTK_MAJOR_VERSION > 8
  return qvtk.renderWindow();
#else
  return qvtk.GetRenderWindow();
#endif // VTK_MAJOR_VERSION > 8
}

inline auto PCL_EXPORTS setRenderWindowCompat(PCLQVTKWidget& qvtk, vtkRenderWindow& window) {
#if VTK_MAJOR_VERSION > 8
  return qvtk.setRenderWindow(&window);
#else
  return qvtk.SetRenderWindow(&window);
#endif // VTK_MAJOR_VERSION > 8
}

#else
#error PCL is not compiled with QVTK.
#endif
