#include <pcl/apps/cloud_composer/point_selectors/interactor_style_switch.h>
#include <pcl/apps/cloud_composer/point_selectors/rectangular_frustum_selector.h>
#include <pcl/visualization/interactor_style.h>

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
  name_to_style_map_.insert (PCL_VISUALIZER, pcl_vis_style_);
  rectangular_frustum_selector_ = vtkSmartPointer<RectangularFrustumSelector>::New ();
  name_to_style_map_.insert (RECTANGULAR_FRUSTUM, rectangular_frustum_selector_);
  
  area_picker_ = vtkSmartPointer<vtkAreaPicker>::New();
  point_picker_ = vtkSmartPointer<vtkPointPicker>::New ();
  
  current_style_ = 0;
  
}

pcl::cloud_composer::InteractorStyleSwitch::~InteractorStyleSwitch ()
{
    
}

void
pcl::cloud_composer::InteractorStyleSwitch::initializeInteractorStyles (boost::shared_ptr<pcl::visualization::PCLVisualizer> vis)
{
  qDebug () << "Initializing Interactor Styles";
  vis_ = vis;
 
  pcl_vis_style_->Initialize ();
  rens_ = vis_->getRendererCollection ();
  pcl_vis_style_->setRendererCollection (rens_);
  pcl_vis_style_->setCloudActorMap (vis_->getCloudActorMap ());
  
  rectangular_frustum_selector_->setCloudActorMap (vis_->getCloudActorMap ());
}

void
pcl::cloud_composer::InteractorStyleSwitch::setCurrentInteractorStyle (INTERACTOR_STYLES interactor_style)
{
  qDebug () << "Setting interactor style";
  vtkSmartPointer <vtkInteractorStyle> style_ptr = name_to_style_map_.value (interactor_style);
  if (current_style_)
    current_style_->SetInteractor (0);
  current_style_= style_ptr;
  
  if (current_style_)
  {
    qDebug () << "Modifying current interactor of style!";
    current_style_->SetInteractor (this->Interactor);
    current_style_->SetTDxStyle (this->TDxStyle);
    vtkInteractorStyleRubberBandPick* rubber_band_style = vtkInteractorStyleRubberBandPick::SafeDownCast (current_style_);
    if (rubber_band_style)
    {
      vis_->getRenderWindow ()->GetInteractor ()->SetPicker (area_picker_);
      rubber_band_style->StartSelect ();
        
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
