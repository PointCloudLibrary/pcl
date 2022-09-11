#include <pcl/apps/cloud_composer/point_selectors/interactor_style_switch.h>
#include <pcl/apps/cloud_composer/point_selectors/rectangular_frustum_selector.h>
#include <pcl/apps/cloud_composer/point_selectors/selected_trackball_interactor_style.h>
#include <pcl/apps/cloud_composer/point_selectors/click_trackball_interactor_style.h>
#include <pcl/visualization/interactor_style.h>

#include <QDebug>

#include <vtkCallbackCommand.h>
#include <vtkObjectFactory.h>

namespace pcl
{
  namespace cloud_composer
  {
    vtkStandardNewMacro(InteractorStyleSwitch);
  }
}

pcl::cloud_composer::InteractorStyleSwitch::InteractorStyleSwitch ()
{
  pcl_vis_style_ = vtkSmartPointer<pcl::visualization::PCLVisualizerInteractorStyle>::New ();
  name_to_style_map_.insert (interactor_styles::PCL_VISUALIZER, pcl_vis_style_);
  
  rectangular_frustum_selector_ = vtkSmartPointer<RectangularFrustumSelector>::New ();
  name_to_style_map_.insert (interactor_styles::RECTANGULAR_FRUSTUM, rectangular_frustum_selector_);
  
  selected_trackball_interactor_style_ = vtkSmartPointer <SelectedTrackballStyleInteractor>::New ();
  name_to_style_map_.insert (interactor_styles::SELECTED_TRACKBALL, selected_trackball_interactor_style_);
  
  click_trackball_interactor_style_ = vtkSmartPointer <ClickTrackballStyleInteractor>::New ();
  name_to_style_map_.insert (interactor_styles::CLICK_TRACKBALL, click_trackball_interactor_style_);
  
  area_picker_ = vtkSmartPointer<vtkAreaPicker>::New();
  point_picker_ = vtkSmartPointer<vtkPointPicker>::New ();
  
  current_style_ = nullptr;
  
}

void
pcl::cloud_composer::InteractorStyleSwitch::initializeInteractorStyles (pcl::visualization::PCLVisualizer::Ptr vis, ProjectModel* model)
{
  qDebug () << "Initializing Interactor Styles";
  vis_ = std::move(vis);
  project_model_ = model;
  
  pcl_vis_style_->Initialize ();
  rens_ = vis_->getRendererCollection ();
  pcl_vis_style_->setRendererCollection (rens_);
  pcl_vis_style_->setCloudActorMap (vis_->getCloudActorMap ());
  
  rectangular_frustum_selector_->setCloudActorMap (vis_->getCloudActorMap ());
  
  selected_trackball_interactor_style_->setCloudActorMap (vis_->getCloudActorMap ());
  selected_trackball_interactor_style_->setProjectModel (project_model_);
  
  click_trackball_interactor_style_->setCloudActorMap (vis_->getCloudActorMap ());
  click_trackball_interactor_style_->setProjectModel (project_model_);
}

void
pcl::cloud_composer::InteractorStyleSwitch::setCurrentInteractorStyle (interactor_styles::INTERACTOR_STYLES interactor_style)
{
  qDebug () << "Setting interactor style";
  vtkSmartPointer <vtkInteractorStyle> style_ptr = name_to_style_map_.value (interactor_style);
  if (current_style_)
    current_style_->SetInteractor (nullptr);
  current_style_= style_ptr;
  
  if (current_style_)
  {
    qDebug () << "Modifying current interactor of style!";
    current_style_->SetInteractor (this->Interactor);
    current_style_->SetTDxStyle (this->TDxStyle);
    
    if (interactor_style == interactor_styles::RECTANGULAR_FRUSTUM)
    {
      vtkInteractorStyleRubberBandPick* rubber_band_style = vtkInteractorStyleRubberBandPick::SafeDownCast (current_style_);
      if (rubber_band_style)
      {
        vis_->getRenderWindow ()->GetInteractor ()->SetPicker (area_picker_);
        rubber_band_style->StartSelect ();
      }
    }
    
    
  }
  
      
 
}

//----------------------------------------------------------------------------
void 
pcl::cloud_composer::InteractorStyleSwitch::SetInteractor (vtkRenderWindowInteractor *iren)
{
  if(iren == this->Interactor)
  {
    return;
  }
  // if we already have an Interactor then stop observing it
  if(this->Interactor)
  {
    this->Interactor->RemoveObserver(this->EventCallbackCommand);
  }
  this->Interactor = iren;
  // add observers for each of the events handled in ProcessEvents
  if(iren)
  {
    iren->AddObserver(vtkCommand::CharEvent, 
                      this->EventCallbackCommand,
                      this->Priority);

    iren->AddObserver(vtkCommand::DeleteEvent, 
                      this->EventCallbackCommand,
                      this->Priority);
  }
}

//----------------------------------------------------------------------------
void 
pcl::cloud_composer::InteractorStyleSwitch::SetDefaultRenderer (vtkRenderer* renderer)
{
  vtkInteractorStyle::SetDefaultRenderer(renderer);
  pcl_vis_style_->SetDefaultRenderer(renderer);
  rectangular_frustum_selector_->SetDefaultRenderer(renderer);
}

//----------------------------------------------------------------------------
void 
pcl::cloud_composer::InteractorStyleSwitch::SetCurrentRenderer (vtkRenderer* renderer)
{
  this->vtkInteractorStyle::SetCurrentRenderer(renderer);
  pcl_vis_style_->SetCurrentRenderer(renderer);
  rectangular_frustum_selector_->SetCurrentRenderer(renderer);
}

void
pcl::cloud_composer::InteractorStyleSwitch::OnLeave ()
{
  qDebug () << "ON LEAVE";
}
