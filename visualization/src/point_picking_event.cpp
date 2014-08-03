/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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
 */

#include <pcl/visualization/point_picking_event.h>
#include <pcl/visualization/interactor_style.h>
#include <vtkVersion.h>
#include <vtkPointPicker.h>
#include <vtkAreaPicker.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkDataSet.h>
#include <vtkPolyData.h>
#include <vtkIdTypeArray.h>
#include <vtkExtractGeometry.h>
#include <vtkPointData.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkPlanes.h>
#include <vtkXYPlotActor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PointPickingCallback::Execute (vtkObject *caller, unsigned long eventid, void*)
{
  PCLVisualizerInteractorStyle *style = reinterpret_cast<PCLVisualizerInteractorStyle*>(caller);
  vtkRenderWindowInteractor* iren = reinterpret_cast<pcl::visualization::PCLVisualizerInteractorStyle*>(caller)->GetInteractor ();
  if (style->CurrentMode == 0)
  {
    if ((eventid == vtkCommand::LeftButtonPressEvent) && (iren->GetShiftKey () > 0))
    {
      float x = 0, y = 0, z = 0;
      int idx = performSinglePick (iren, x, y, z);
      // Create a PointPickingEvent if a point was selected
      if (idx != -1)
      {
        PointPickingEvent event (idx, x, y, z);
        style->point_picking_signal_ (event);
      }
    }
    else if ((eventid == vtkCommand::LeftButtonPressEvent) && (iren->GetAltKey () == 1))
    {
      pick_first_ = !pick_first_;
      float x = 0, y = 0, z = 0;
      int idx = -1;
      if (pick_first_)
        idx_ = performSinglePick (iren, x_, y_, z_);
      else
        idx = performSinglePick (iren, x, y, z);
      // Create a PointPickingEvent
      PointPickingEvent event (idx_, idx, x_, y_, z_, x, y, z);
      style->point_picking_signal_ (event);
    }
    // Call the parent's class mouse events
    if (eventid == vtkCommand::LeftButtonPressEvent)
      style->OnLeftButtonDown ();
    else if (eventid == vtkCommand::LeftButtonReleaseEvent)
      style->OnLeftButtonUp ();
  }
  else
  {
    if (eventid == vtkCommand::LeftButtonPressEvent)
    {
      style->OnLeftButtonDown ();
      x_ = static_cast<float> (iren->GetEventPosition ()[0]);
      y_ = static_cast<float> (iren->GetEventPosition ()[1]);
    }
    else if (eventid == vtkCommand::LeftButtonReleaseEvent)
    {
      style->OnLeftButtonUp ();
      std::vector<int> indices;
      int nb_points = performAreaPick (iren, indices);
      AreaPickingEvent event (nb_points, indices);
      style->area_picking_signal_ (event);
    }
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////
int
pcl::visualization::PointPickingCallback::performSinglePick (vtkRenderWindowInteractor *iren)
{
  vtkPointPicker* point_picker = vtkPointPicker::SafeDownCast (iren->GetPicker ());
  
  if (!point_picker)
  {
    pcl::console::print_error ("Point picker not available, not selecting any points!\n");
    return -1;
  }

  int mouse_x = iren->GetEventPosition ()[0];
  int mouse_y = iren->GetEventPosition ()[1];

  iren->StartPickCallback ();
  vtkRenderer *ren = iren->FindPokedRenderer (mouse_x, mouse_y);
  point_picker->Pick (mouse_x, mouse_y, 0.0, ren);

  return (static_cast<int> (point_picker->GetPointId ()));
}

/////////////////////////////////////////////////////////////////////////////////////////////
int
pcl::visualization::PointPickingCallback::performSinglePick (
    vtkRenderWindowInteractor *iren,
    float &x, float &y, float &z)
{
  vtkPointPicker* point_picker = vtkPointPicker::SafeDownCast (iren->GetPicker ());

  if (!point_picker)
  {
    pcl::console::print_error ("Point picker not available, not selecting any points!\n");
    return (-1);
  }

  int mouse_x = iren->GetEventPosition ()[0];
  int mouse_y = iren->GetEventPosition ()[1];

  iren->StartPickCallback ();
  vtkRenderer *ren = iren->FindPokedRenderer (mouse_x, mouse_y);
  point_picker->Pick (mouse_x, mouse_y, 0.0, ren);

  int idx = static_cast<int> (point_picker->GetPointId ());
  if (point_picker->GetDataSet () != NULL)
  {
    double p[3];
    point_picker->GetDataSet ()->GetPoint (idx, p);
    x = float (p[0]); y = float (p[1]); z = float (p[2]);
  }

  return (idx);
}

/////////////////////////////////////////////////////////////////////////////////////////////
int
pcl::visualization::PointPickingCallback::performAreaPick (vtkRenderWindowInteractor *iren,
                                                           std::vector<int> &indices)
{
  vtkAreaPicker *picker = static_cast<vtkAreaPicker*> (iren->GetPicker ());
  vtkRenderer *ren = iren->FindPokedRenderer (iren->GetEventPosition ()[0], iren->GetEventPosition ()[1]);
  picker->AreaPick (x_, y_, iren->GetEventPosition ()[0], iren->GetEventPosition ()[1], ren);
  if (picker->GetDataSet ())
  {
    vtkPolyData* points = reinterpret_cast<vtkPolyData*> (picker->GetDataSet ());

    // This is a naive solution till we fugure out where to add the GlobalIds at an earlier stage
    if (!points->GetPointData ()->GetGlobalIds () ||
        points->GetPointData ()->GetGlobalIds ()->GetNumberOfTuples () == 0)
    {
      vtkSmartPointer<vtkIdTypeArray> global_ids = vtkIdTypeArray::New ();
      global_ids->SetNumberOfValues (picker->GetDataSet ()->GetNumberOfPoints ());
      for (vtkIdType i = 0; i < picker->GetDataSet ()->GetNumberOfPoints (); ++i)
        global_ids->SetValue (i,i);

      points->GetPointData ()->SetGlobalIds (global_ids);
    }

    vtkPlanes* frustum = picker->GetFrustum ();

    vtkSmartPointer<vtkExtractGeometry> extract_geometry = vtkSmartPointer<vtkExtractGeometry>::New ();
    extract_geometry->SetImplicitFunction (frustum);

#if VTK_MAJOR_VERSION < 6
    extract_geometry->SetInput (picker->GetDataSet ());
#else
    extract_geometry->SetInputData (picker->GetDataSet ());
#endif

    extract_geometry->Update ();

    vtkSmartPointer<vtkVertexGlyphFilter> glyph_filter = vtkSmartPointer<vtkVertexGlyphFilter>::New ();
    glyph_filter->SetInputConnection (extract_geometry->GetOutputPort ());
    glyph_filter->Update ();

    vtkPolyData* selected = glyph_filter->GetOutput ();
    vtkIdTypeArray* ids = vtkIdTypeArray::SafeDownCast(selected->GetPointData ()->GetGlobalIds ());
    assert (ids);
    indices.reserve (ids->GetNumberOfTuples ());

    for(vtkIdType i = 0; i < ids->GetNumberOfTuples (); i++)
      indices.push_back (static_cast<int> (ids->GetValue (i)));

    return (static_cast<int> (selected->GetNumberOfPoints ()));
  }
  return (-1);
}
