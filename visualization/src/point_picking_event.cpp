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
#include <vtkPointData.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkPlanes.h>
#include <vtkXYPlotActor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>

#include <vtkProp3D.h>
#include <vtkProp3DCollection.h>
#include <vtkActor.h>
#include <vtkActorCollection.h>
#include <vtkMapper.h>
#include <vtkPolyData.h>
#include <vtkIdTypeArray.h>
#include <vtkExtractGeometry.h>

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PointPickingCallback::Execute (vtkObject *caller, unsigned long eventid, void*)
{
  PCLVisualizerInteractorStyle *style = reinterpret_cast<PCLVisualizerInteractorStyle*>(caller);
  vtkRenderWindowInteractor* iren = reinterpret_cast<pcl::visualization::PCLVisualizerInteractorStyle*>(caller)->GetInteractor ();
  if (style->CurrentMode == 0)
  {
    //vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New() ;
    if ((eventid == vtkCommand::LeftButtonPressEvent) && (iren->GetShiftKey () > 0))
    {
      float x = 0, y = 0, z = 0;
      int idx = performSinglePick (iren, x, y, z);
      // Create a PointPickingEvent if a point was selected
      if (idx != -1)
      {
        PointPickingEvent event (idx, x, y, z, actor_);
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
      //std::cout<< "druhe zjisteni actor: " << actor <<std::endl;
      PointPickingEvent event (idx_, idx, x_, y_, z_, x, y, z, actor_);
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
      std::vector< std::vector<int> >indices;
      vtkActorCollection* actors = vtkActorCollection::New();
      int nb_points = performAreaPick (iren, indices,actors);
      AreaPickingEvent event (nb_points, indices, actors);
      style->area_picking_signal_ (event);
      actors->Delete();

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
  actor_ = point_picker->GetActor();

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
    actor_ = point_picker->GetActor();
  }

  return (idx);
}

/////////////////////////////////////////////////////////////////////////////////////////////
int
pcl::visualization::PointPickingCallback::performAreaPick (vtkRenderWindowInteractor *iren,
                                                           std::vector< std::vector<int> >&indices, vtkActorCollection* actors)
{
  vtkAreaPicker *picker = static_cast<vtkAreaPicker*> (iren->GetPicker ());
  vtkRenderer *ren = iren->FindPokedRenderer (iren->GetEventPosition ()[0], iren->GetEventPosition ()[1]);
  picker->AreaPick (x_, y_, iren->GetEventPosition ()[0], iren->GetEventPosition ()[1], ren);
  if (picker->GetProp3Ds())
  {
    int pt_numb=0;
    vtkProp3DCollection* props = picker->GetProp3Ds();
    vtkCollectionSimpleIterator pit;
    vtkProp3D* prop;
    for(props->InitTraversal(pit);prop = props->GetNextProp3D(pit);)
    {
      vtkActor* actor = vtkActor::SafeDownCast(prop);
      if(actor)
      {
        actors->AddItem(actor);

        std::vector<int> actorIndices;
        vtkPolyData* pd = vtkPolyData::SafeDownCast(actor->GetMapper()->GetInput());

        if(pd->GetPointData()->HasArray("Indices"))
            pd->GetPointData()->RemoveArray("Indices");


        vtkSmartPointer<vtkIdTypeArray> IDs = vtkSmartPointer<vtkIdTypeArray>::New();
        IDs->SetNumberOfComponents(1);
        IDs->SetName("Indices");
        for(vtkIdType i = 0; i < pd->GetNumberOfPoints(); i++)
          IDs->InsertNextValue(i);

        pd->GetPointData()->AddArray(IDs);

        vtkPlanes* frustum = picker->GetFrustum();

        vtkSmartPointer<vtkExtractGeometry> extract_geometry = vtkSmartPointer<vtkExtractGeometry>::New();
        extract_geometry->SetImplicitFunction(frustum);
  #if VTK_MAJOR_VERSION < 6
        extract_geometry->SetInput(pd);
  #else
        extract_geometry->SetInputData(pd);
  #endif

        extract_geometry->Update();

        vtkSmartPointer<vtkVertexGlyphFilter> glyph_filter = vtkSmartPointer<vtkVertexGlyphFilter>::New ();
        glyph_filter->SetInputConnection (extract_geometry->GetOutputPort ());
        glyph_filter->Update ();

        vtkPolyData* selected = glyph_filter->GetOutput ();

        vtkIdTypeArray* GlobalIDs  = vtkIdTypeArray::SafeDownCast(selected->GetPointData()->GetArray("Indices"));
        for (vtkIdType i = 0; i < selected->GetNumberOfPoints (); i++)
        {
          actorIndices.push_back(static_cast<int>(GlobalIDs->GetValue(i)));
          pt_numb++;
        }
        indices.push_back(actorIndices);

      }
    }
    return (pt_numb);
  }
  return (-1);
}
