#include <pcl/apps/cloud_composer/point_selectors/click_trackball_interactor_style.h>
#include <pcl/apps/cloud_composer/point_selectors/manipulation_event.h>
#include <pcl/apps/cloud_composer/project_model.h>


namespace pcl
{
  namespace cloud_composer
  {
    vtkStandardNewMacro(ClickTrackballStyleInteractor);
  }
}

pcl::cloud_composer::ClickTrackballStyleInteractor::ClickTrackballStyleInteractor ()
  : vtkInteractorStyleTrackballActor ()
{
  manipulation_complete_event_ = interactor_events::MANIPULATION_COMPLETE_EVENT;
  start_matrix_= vtkSmartPointer<vtkMatrix4x4>::New ();
  end_matrix_ = vtkSmartPointer<vtkMatrix4x4>::New ();
  transform_ = vtkSmartPointer<vtkTransform>::New ();
}

pcl::cloud_composer::ClickTrackballStyleInteractor::~ClickTrackballStyleInteractor ()
{
  
}

void
pcl::cloud_composer::ClickTrackballStyleInteractor::OnLeftButtonDown ()
{
  vtkInteractorStyleTrackballActor::OnLeftButtonDown();
  
  vtkActor* selected_actor = vtkActor::SafeDownCast(this->InteractionProp);
  if (selected_actor)
    selected_actor->GetMatrix (start_matrix_);
  
}

void
pcl::cloud_composer::ClickTrackballStyleInteractor::OnRightButtonDown ()
{
  vtkInteractorStyleTrackballActor::OnRightButtonDown();
  
  vtkActor* selected_actor = vtkActor::SafeDownCast(this->InteractionProp);
  if (selected_actor)
    selected_actor->GetMatrix (start_matrix_);
  
}

void
pcl::cloud_composer::ClickTrackballStyleInteractor::OnLeftButtonUp ()
{
  vtkInteractorStyleTrackballActor::OnLeftButtonUp();
  vtkSmartPointer<vtkActor> selected_actor = vtkActor::SafeDownCast(this->InteractionProp);
  if (selected_actor)
  {
    ManipulationEvent* manip_event = new ManipulationEvent ();
    //Fetch the actor we manipulated
    
    selected_actor->GetMatrix (end_matrix_);
    // Find the id of the actor we manipulated
    pcl::visualization::CloudActorMap::const_iterator end = actors_->end ();
    QString manipulated_id;
    for( pcl::visualization::CloudActorMap::const_iterator itr = actors_->begin (); itr != end; ++itr)
    {
      //qDebug () << "Id = "<<QString::fromStdString (itr->first);
      if ( (itr->second).actor == selected_actor)
      {
        manipulated_id = (QString::fromStdString (itr->first));
        
      }
    }
    if ( !manipulated_id.isEmpty() )
    {
      manip_event->addManipulation (manipulated_id, start_matrix_, end_matrix_);
      this->InvokeEvent (this->manipulation_complete_event_, manip_event);
    }
    else
    {
      qWarning () << "Could not find actor which matches manipulated actor in ClickTrackballStyleInteractor::OnLeftButtonUp!!!";
    }
  }
}

void
pcl::cloud_composer::ClickTrackballStyleInteractor::OnRightButtonUp ()
{
  vtkInteractorStyleTrackballActor::OnRightButtonUp();
  vtkSmartPointer<vtkActor> selected_actor = vtkActor::SafeDownCast(this->InteractionProp);
  if (selected_actor)
  {
    ManipulationEvent* manip_event = new ManipulationEvent ();
    //Fetch the actor we manipulated
    
    selected_actor->GetMatrix (end_matrix_);
    // Find the id of the actor we manipulated
    pcl::visualization::CloudActorMap::const_iterator end = actors_->end ();
    QString manipulated_id;
    for( pcl::visualization::CloudActorMap::const_iterator itr = actors_->begin (); itr != end; ++itr)
    {
      //qDebug () << "Id = "<<QString::fromStdString (itr->first);
      if ( (itr->second).actor == selected_actor)
      {
        manipulated_id = (QString::fromStdString (itr->first));
        
      }
    }
    if ( !manipulated_id.isEmpty() )
    {
      manip_event->addManipulation (manipulated_id, start_matrix_, end_matrix_);
      this->InvokeEvent (this->manipulation_complete_event_, manip_event);
    }
    else
    {
      qWarning () << "Could not find actor which matches manipulated actor in ClickTrackballStyleInteractor::OnRightButtonUp!!!";
    }
  }
   
}


