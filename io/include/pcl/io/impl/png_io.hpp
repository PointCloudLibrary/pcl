/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2013-, Open Perception, Inc.
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
 */

#ifndef PCL_PNG_IO_IMPL_HPP_
#define PCL_PNG_IO_IMPL_HPP_

#include <vtkPNGWriter.h>
#include <vtkSmartPointer.h>
#include <vtkDoubleArray.h>
#include <vtkRenderer.h>
#include <vtkXYPlotActor.h>
#include <vtkRenderWindow.h>
#include <vtkTextProperty.h>
#include <vtkFieldData.h>
#include <vtkDataObject.h>
#include <vtkWindowToImageFilter.h>
#include <vtkDoubleArray.h>
#include <vtkProperty2D.h>
#include <vtkRendererSource.h>
#include <vtkGraphicsFactory.h>
#include <vtkImagingFactory.h>

namespace {

  void
  renderAndWritePng (const std::string& file_name,
          vtkSmartPointer<vtkXYPlotActor>& xy_plot, int width, int height)
  {
    // Setup offscreen rendering
    vtkSmartPointer<vtkGraphicsFactory> graphics_factory = vtkSmartPointer<vtkGraphicsFactory>::New ();
    graphics_factory->SetOffScreenOnlyMode (1);
    graphics_factory->SetUseMesaClasses (1);

    vtkSmartPointer<vtkImagingFactory> imaging_factory = vtkSmartPointer<vtkImagingFactory>::New ();
    imaging_factory->SetUseMesaClasses (1);

    vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New ();
    renderer->AddActor2D (xy_plot);
    renderer->SetBackground (1, 1, 1); // Background color white

    // Render the window
    vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New ();
    renderWindow->AddRenderer (renderer);
    renderWindow->SetSize (width, height);
    renderWindow->SetBorders (1);
    renderWindow->SetOffScreenRendering (1);
    renderWindow->Render ();

    // Convert window to image
    vtkSmartPointer<vtkWindowToImageFilter> windowToImageFilter = vtkSmartPointer<vtkWindowToImageFilter>::New ();
    windowToImageFilter->SetInput (renderWindow);
    windowToImageFilter->SetInputBufferTypeToRGBA ();
    windowToImageFilter->Update ();

    // Write image to PNG file
    vtkSmartPointer<vtkPNGWriter> writer = vtkSmartPointer<vtkPNGWriter>::New ();
    writer->SetFileName (file_name.c_str ());
    writer->SetInputConnection (windowToImageFilter->GetOutputPort ());
    writer->Write ();
  }

  void
  saveHistPNGFile (const std::string& file_name,
          vtkSmartPointer<vtkDoubleArray>& xy_array, int width, int height)
  {
    vtkSmartPointer<vtkXYPlotActor> xy_plot = vtkSmartPointer<vtkXYPlotActor>::New ();
    xy_plot->SetDataObjectPlotModeToColumns ();
    xy_plot->SetXValuesToValue ();

    vtkSmartPointer<vtkFieldData> field_values = vtkSmartPointer<vtkFieldData>::New ();
    field_values->AddArray (xy_array);

    vtkSmartPointer<vtkDataObject> field_data = vtkSmartPointer<vtkDataObject>::New ();
    field_data->SetFieldData (field_values);

    xy_plot->AddDataObjectInput (field_data);
    xy_plot->SetPlotColor (0, 1.0, 0.0, 0.0);

    xy_plot->SetDataObjectXComponent (0, 0);
    xy_plot->SetDataObjectYComponent (0, 1);
    xy_plot->PlotPointsOn ();
    xy_plot->PlotCurveLinesOn ();

    double min_max[2];
    xy_array->GetRange (min_max, 1);

    xy_plot->SetYTitle (""); xy_plot->SetXTitle ("");
    xy_plot->SetYRange (min_max[0], min_max[1]);
    xy_plot->SetXRange (0, static_cast<double> (xy_array->GetNumberOfTuples () - 1));
    xy_plot->GetProperty ()->SetColor (0, 0, 0);

    // Adjust text properties
    vtkSmartPointer<vtkTextProperty> tprop = xy_plot->GetTitleTextProperty ();
    xy_plot->AdjustTitlePositionOn ();
    tprop->SetFontSize (8);
    tprop->ShadowOff (); tprop->ItalicOff ();
    tprop->SetColor (xy_plot->GetProperty ()->GetColor ());

    xy_plot->SetAxisLabelTextProperty (tprop);
    xy_plot->SetAxisTitleTextProperty (tprop);
    xy_plot->SetNumberOfXLabels (8);
    xy_plot->GetProperty ()->SetPointSize (3);
    xy_plot->GetProperty ()->SetLineWidth (2);

    xy_plot->SetPosition (0, 0);
    xy_plot->SetWidth (1); xy_plot->SetHeight (1);

    renderAndWritePng (file_name, xy_plot, width, height);

  }

};

template <typename HistT> void
pcl::io::savePNGFile (const std::string& file_name,
        const pcl::PointCloud<HistT>& cloud, const std::string& field_name,
        const int index, int width, int height)
{
  if (index < 0 || index >= cloud.points.size ())
  {
    PCL_ERROR ("[pcl::io::savePNGFile] Invalid point index (%d) given!\n", index);
    return;
  }

  std::vector<pcl::PCLPointField> fields;
  // Check if our field exists
  int field_idx = pcl::getFieldIndex<HistT> (cloud, field_name, fields);
  if (field_idx == -1)
  {
    PCL_ERROR ("[pcl::io::savePNGFile] The specified field <%s> does not exist!\n", field_name.c_str ());
    return;
  }

  vtkSmartPointer<vtkDoubleArray> xy_array = vtkSmartPointer<vtkDoubleArray>::New ();
  xy_array->SetNumberOfComponents (2);
  xy_array->SetNumberOfTuples (fields[field_idx].count);

  // Parse the cloud data and store it in the array
  double xy[2];
  for (size_t d = 0; d < fields[field_idx].count; ++d)
  {
    xy[0] = d;
    float data;
    memcpy (&data, reinterpret_cast<const char*> (&cloud.points[index]) + fields[field_idx].offset + d * sizeof (float), sizeof (float));
    xy[1] = data;
    xy_array->SetTuple (d, xy);
  }

  saveHistPNGFile (file_name, xy_array, width, height);

}

template <typename HistT> void
pcl::io::savePNGFile (const std::string& file_name,
        const pcl::PointCloud<HistT>& cloud, const int hsize,
        int width, int height )
{
  vtkSmartPointer<vtkDoubleArray> xy_array = vtkSmartPointer<vtkDoubleArray>::New ();
  xy_array->SetNumberOfComponents (2);
  xy_array->SetNumberOfTuples (hsize);

  // Parse the cloud data and store it in the array
  double xy[2];
  for (int d = 0; d < hsize; ++d)
  {
    xy[0] = d;
    xy[1] = cloud.points[0].histogram[d];
    xy_array->SetTuple (d, xy);
  }

  saveHistPNGFile (file_name, xy_array, width, height);

}

#endif //#ifndef PCL_PNG_IO_IMPL_HPP_
