/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception, Inc.
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


#include <pcl/apps/cloud_composer/point_selectors/rectangular_frustum_selector.h>
#include <pcl/apps/cloud_composer/point_selectors/selection_event.h>

#include <QDebug>

namespace pcl
{
  namespace cloud_composer
  {
    vtkStandardNewMacro(RectangularFrustumSelector);
  }
}

pcl::cloud_composer::RectangularFrustumSelector::RectangularFrustumSelector ()
  : vtkInteractorStyleRubberBandPick ()
{
  selection_complete_event_ = interactor_events::SELECTION_COMPLETE_EVENT;
}

pcl::cloud_composer::RectangularFrustumSelector::~RectangularFrustumSelector ()
{
  
}


void
pcl::cloud_composer::RectangularFrustumSelector::OnLeftButtonUp ()
{
           
  vtkSmartPointer<vtkActor> selected_actor = vtkSmartPointer<vtkActor>::New();
  vtkSmartPointer<vtkDataSetMapper> selected_mapper = vtkSmartPointer<vtkDataSetMapper>::New();
  selected_actor->SetMapper(selected_mapper);
  
  vtkInteractorStyleRubberBandPick::OnLeftButtonUp ();
 
  vtkPlanes* frustum = static_cast<vtkAreaPicker*> (this->GetInteractor ()->GetPicker ())->GetFrustum ();
 
  vtkSmartPointer<vtkIdFilter> id_filter = vtkSmartPointer<vtkIdFilter>::New ();
  id_filter->PointIdsOn ();
  
  vtkSmartPointer<vtkExtractGeometry> extract_geometry = vtkSmartPointer<vtkExtractGeometry>::New ();
  extract_geometry->SetImplicitFunction (frustum);
  extract_geometry->SetInputConnection (id_filter->GetOutputPort ());

  vtkSmartPointer<vtkVertexGlyphFilter> glyph_filter = vtkSmartPointer<vtkVertexGlyphFilter>::New ();
  glyph_filter->SetInputConnection (extract_geometry->GetOutputPort ());
  
  vtkSmartPointer<vtkAppendPolyData> append = vtkAppendPolyData::New ();
  
  QMap < QString, vtkPolyData* > id_selected_data_map;
  for (const auto &actor : *actors_)
  {
        const pcl::visualization::CloudActor *act = &actor.second;
        vtkMapper* mapper = act->actor->GetMapper ();
        vtkDataSet* data = mapper->GetInput ();
        vtkPolyData* poly_data = vtkPolyData::SafeDownCast (data);
        id_filter->SetInputData (poly_data);

        vtkSmartPointer<vtkPolyData> selected = vtkSmartPointer<vtkPolyData>::New ();
        glyph_filter->SetOutput (selected);
        glyph_filter->Update ();
        if (selected->GetNumberOfPoints() > 0)
        {
          qDebug () << "Selected " << selected->GetNumberOfPoints () << " points.";
          id_selected_data_map.insert ( QString::fromStdString (actor.first), selected);
          append->AddInputData (selected);
        }
  }
  append->Update ();
  vtkSmartPointer<vtkPolyData> all_points = append->GetOutput ();
  qDebug () << "Allpoints = " <<all_points->GetNumberOfPoints ();

  selected_mapper->SetInputData (all_points);
  selected_mapper->ScalarVisibilityOff ();
  
  selected_actor->GetProperty ()->SetColor (0.0, 1.0, 0.0); //(R,G,B)
  selected_actor->GetProperty ()->SetPointSize (3);

  this->CurrentRenderer->AddActor (selected_actor);
  this->GetInteractor ()->GetRenderWindow ()->Render ();
  this->HighlightProp (NULL);
 
  if (all_points->GetNumberOfPoints () > 0)
  {
    SelectionEvent* selected = new SelectionEvent (all_points, selected_actor, selected_mapper, id_selected_data_map, this->CurrentRenderer);
    this->InvokeEvent (this->selection_complete_event_, selected);
  }
}

