/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009-2012, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */
#ifndef PCL_PCL_VISUALIZER_VTK_H_
#define PCL_PCL_VISUALIZER_VTK_H_

#if defined __GNUC__
#pragma GCC system_header
#ifdef __DEPRECATED
#undef __DEPRECATED
#define __DEPRECATED_DISABLED__
#endif
#endif

#include <vtkVersion.h>
#include <vtkAppendPolyData.h>
#include <vtkAssemblyPath.h>
#include <vtkAxesActor.h>
#include <vtkActor.h>
#include <vtkBoxRepresentation.h>
#include <vtkBoxWidget.h>
#include <vtkBoxWidget2.h>
#include <vtkCellData.h>
#include <vtkMath.h>
#include <vtkLoopSubdivisionFilter.h>
#include <vtkLineSource.h>
#include <vtkLegendScaleActor.h>
#include <vtkLightKit.h>
#include <vtkPlatonicSolidSource.h>
#include <vtkPropPicker.h>
#include <vtkGeneralTransform.h>
#include <vtkSmartPointer.h>
#include <vtkDataSet.h>
#include <vtkDataSetSurfaceFilter.h>
#include <vtkExecutive.h>
#include <vtkPolygon.h>
#include <vtkPointPicker.h>
#include <vtkUnstructuredGrid.h>
#include <vtkConeSource.h>
#include <vtkDiskSource.h>
#include <vtkPlaneSource.h>
#include <vtkSphereSource.h>
#include <vtkIdentityTransform.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkTubeFilter.h>
#include <vtkCubeSource.h>
#include <vtkAxes.h>
#include <vtkFloatArray.h>
#include <vtkPointData.h>
#include <vtkPolyData.h>
#include <vtkPolyDataReader.h>
#include <vtkPolyDataMapper.h>
#include <vtkDataSetMapper.h>
#include <vtkCellArray.h>
#include <vtkCommand.h>
#include <vtkCellLocator.h>
#include <vtkPLYReader.h>
#include <vtkTransformFilter.h>
#include <vtkPolyLine.h>
#include <vtkVectorText.h>
#include <vtkFollower.h>
#include <vtkCallbackCommand.h>
#include <vtkInteractorStyle.h>
#include <vtkInformationVector.h>
#include <vtkDataArray.h>
#include <vtkUnsignedCharArray.h>
#include <vtkPoints.h>
#include <vtkRendererCollection.h>
#include <vtkPNGWriter.h>
#include <vtkWindowToImageFilter.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkProperty.h>
#include <vtkCamera.h>
#include <vtkObjectFactory.h>
#include <vtkScalarBarActor.h>
#include <vtkScalarsToColors.h>
#include <vtkClipPolyData.h>
#include <vtkPlanes.h>
#include <vtkImageImport.h>
#include <vtkImageViewer.h>
#include <vtkInteractorStyleImage.h>
#if ((VTK_MAJOR_VERSION == 5) && (VTK_MINOR_VERSION >= 10))
#include <vtkImageSlice.h>
#include <vtkImageProperty.h>
#include <vtkImageSliceMapper.h>
#endif
#include <vtkImageFlip.h>
#include <vtkTIFFWriter.h>
#include <vtkBMPWriter.h>
#include <vtkJPEGWriter.h>
#include <vtkImageViewer2.h>
#include <vtkRenderWindow.h>
#include <vtkXYPlotActor.h>
#include <vtkTextProperty.h>
#include <vtkProperty2D.h>
#include <vtkFieldData.h>
#include <vtkDoubleArray.h>
#include <vtkLODActor.h>
#include <vtkPolyDataWriter.h>
#include <vtkTextActor.h>
#include <vtkCleanPolyData.h>
#include <vtkRenderer.h>
#include <vtkObject.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkImageReslice.h>
#include <vtkImageChangeInformation.h>
#include <vtkImageCanvasSource2D.h>
#include <vtkImageBlend.h>
#include <vtkImageStencilData.h>
#if ((VTK_MAJOR_VERSION == 5) && (VTK_MINOR_VERSION <= 4))
#  include <pcl/visualization/interactor.h>
#else
#  include <vtkRenderWindowInteractor.h>
#  include <vtkChartXY.h>
#  include <vtkPlot.h>
#  include <vtkTable.h>
#  include <vtkContextView.h>
#  include <vtkContextScene.h>
#  include <vtkColorSeries.h>
#  include <vtkAxis.h>
#endif
#include <vtkSelection.h>

#if VTK_MAJOR_VERSION>=6 || (VTK_MAJOR_VERSION==5 && VTK_MINOR_VERSION>4)
#include <vtkHardwareSelector.h>
#else
#include <vtkVisibleCellSelector.h>
#endif

#include <vtkTriangle.h>
#include <vtkWorldPointPicker.h>

#include <vtkInteractorStyleRubberBandPick.h>
#include <vtkInteractorStyleTrackballActor.h>
#include <vtkAreaPicker.h>
#include <vtkExtractGeometry.h>
#include <vtkExtractPolyDataGeometry.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkIdFilter.h>
#include <vtkIdTypeArray.h>
#include <vtkImageReader2Factory.h>
#include <vtkImageReader2.h>
#include <vtkImageData.h>

#if defined __GNUC__ && defined __DEPRECATED_DISABLED__
#define __DEPRECATED
#undef __DEPRECATED_DISABLED__
#endif

#endif    // PCL_PCL_VISUALIZER_VTK_H_

