#include <pcl/apps/cloud_composer/point_selectors/click_trackball_interactor_style.h>
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
}

pcl::cloud_composer::ClickTrackballStyleInteractor::~ClickTrackballStyleInteractor ()
{
  
}

void
pcl::cloud_composer::ClickTrackballStyleInteractor::OnLeftButtonDown ()
{
  QModelIndexList selected_indexes = model_->getSelectionModel ()->selectedIndexes ();
  
  vtkInteractorStyleTrackballActor::OnLeftButtonDown();
}

void
pcl::cloud_composer::ClickTrackballStyleInteractor::OnLeftButtonUp ()
{
           
  vtkInteractorStyleTrackballActor::OnLeftButtonUp();
}




