#include <pcl/apps/cloud_composer/point_selectors/rectangular_frustum_selector.h>
#include <pcl/apps/cloud_composer/point_selectors/selection_event.h>

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
  
  pcl::visualization::CloudActorMap::iterator it;
  it = actors_->begin ();
  QMap < QString, vtkPolyData* > id_selected_data_map;
  for (it = actors_->begin (); it != actors_->end (); ++it)
  {
        pcl::visualization::CloudActor *act = &(*it).second;
        vtkMapper* mapper = act->actor->GetMapper ();
        vtkDataSet* data = mapper->GetInput ();
        vtkPolyData* poly_data = vtkPolyData::SafeDownCast (data);
#if VTK_MAJOR_VERSION > 5
        id_filter->SetInputData (poly_data);
#else
        id_filter->SetInput (poly_data);
#endif
        //extract_geometry->SetInput (poly_data);
          
        vtkSmartPointer<vtkPolyData> selected = vtkSmartPointer<vtkPolyData>::New ();
        glyph_filter->SetOutput (selected);
        glyph_filter->Update ();
#if VTK_MAJOR_VERSION < 6
        selected->SetSource (0);
#endif
        if (selected->GetNumberOfPoints() > 0)
        {
          qDebug () << "Selected " << selected->GetNumberOfPoints () << " points.";
          id_selected_data_map.insert ( QString::fromStdString ((*it).first), selected);
          #if VTK_MAJOR_VERSION < 6
            append->AddInput (selected);
          #else // VTK 6
            append->AddInputData (selected);
          #endif
        }
        
        
  }
  append->Update ();
  vtkSmartPointer<vtkPolyData> all_points = append->GetOutput ();
  qDebug () << "Allpoints = " <<all_points->GetNumberOfPoints ();

#if VTK_MAJOR_VERSION < 6
  selected_mapper->SetInput (all_points);
#else
  selected_mapper->SetInputData (all_points);
#endif
  selected_mapper->ScalarVisibilityOff ();

  vtkIdTypeArray* ids = vtkIdTypeArray::SafeDownCast (all_points->GetPointData ()->GetArray ("OriginalIds"));
  
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

