#include <pcl/apps/cloud_composer/point_selectors/selected_trackball_interactor_style.h>
#include <pcl/apps/cloud_composer/project_model.h>

namespace pcl
{
  namespace cloud_composer
  {
    vtkStandardNewMacro(SelectedTrackballStyleInteractor);
  }
}

pcl::cloud_composer::SelectedTrackballStyleInteractor::SelectedTrackballStyleInteractor ()
  : vtkInteractorStyleTrackballActor ()
{
  manipulation_complete_event_ = interactor_events::MANIPULATION_COMPLETE_EVENT;
}

pcl::cloud_composer::SelectedTrackballStyleInteractor::~SelectedTrackballStyleInteractor ()
{
  
}

void
pcl::cloud_composer::SelectedTrackballStyleInteractor::OnLeftButtonDown ()
{
  QModelIndexList selected_indexes = model_->getSelectionModel ()->selectedIndexes ();
  
  vtkInteractorStyleTrackballActor::OnLeftButtonDown();
}

void
pcl::cloud_composer::SelectedTrackballStyleInteractor::OnLeftButtonUp ()
{
           
  vtkInteractorStyleTrackballActor::OnLeftButtonUp();
}




