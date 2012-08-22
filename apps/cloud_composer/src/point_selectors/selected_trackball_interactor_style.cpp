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
pcl::cloud_composer::SelectedTrackballStyleInteractor::setSelectedActors ()
{
  QList <QString> selected_cloud_ids;
  QModelIndexList selected_indexes = model_->getSelectionModel()->selectedIndexes ();
  foreach (QModelIndex index, selected_indexes)
  {
    QStandardItem* item = model_->itemFromIndex (index);
    CloudItem* cloud_item =  dynamic_cast <CloudItem*> (item);
    if (cloud_item)
      selected_cloud_ids.append (cloud_item->getId ());
  }
  
  pcl::visualization::CloudActorMap::iterator it;
  for (it = actors_->begin (); it != actors_->end (); ++it)
  {
    QString id = QString::fromStdString (it->first);
    if (selected_cloud_ids.contains (id))
    {
      vtkLODActor* actor = (it->second).actor;
      qDebug () << "Adding "<<id<< " to selected manip! ptr ="<<actor;
      selected_actors_map_.insert (id ,actor);
      vtkSmartPointer<vtkMatrix4x4> start_matrix = vtkSmartPointer<vtkMatrix4x4>::New ();
      actor->GetMatrix (start_matrix);
      start_matrix_map_.insert (id,start_matrix);
    }
  }
}

void
pcl::cloud_composer::SelectedTrackballStyleInteractor::OnLeftButtonDown ()
{
  vtkInteractorStyleTrackballActor::OnLeftButtonDown();
  
  setSelectedActors ();
 
}

void
pcl::cloud_composer::SelectedTrackballStyleInteractor::OnRightButtonDown ()
{
  vtkInteractorStyleTrackballActor::OnRightButtonDown();
  
  setSelectedActors ();
  
}

void
pcl::cloud_composer::SelectedTrackballStyleInteractor::OnLeftButtonUp ()
{
  vtkInteractorStyleTrackballActor::OnLeftButtonUp();
  foreach (QString id, selected_actors_map_.keys ())
  {
    vtkLODActor* actor = selected_actors_map_.value (id);
    ManipulationEvent* manip_event = new ManipulationEvent ();
    //Fetch the actor we manipulated
    vtkSmartPointer<vtkMatrix4x4> end_matrix = vtkSmartPointer<vtkMatrix4x4>::New ();
    actor->GetMatrix (end_matrix);
    manip_event->addManipulation (id, start_matrix_map_.value (id), end_matrix);
    this->InvokeEvent (this->manipulation_complete_event_, manip_event);
    
  }
}

void
pcl::cloud_composer::SelectedTrackballStyleInteractor::OnRightButtonUp ()
{
  vtkInteractorStyleTrackballActor::OnRightButtonUp();
  foreach (QString id, selected_actors_map_.keys ())
  {
    vtkLODActor* actor = selected_actors_map_.value (id);
    ManipulationEvent* manip_event = new ManipulationEvent ();
    //Fetch the actor we manipulated
    vtkSmartPointer<vtkMatrix4x4> end_matrix = vtkSmartPointer<vtkMatrix4x4>::New ();
    actor->GetMatrix (end_matrix);
    manip_event->addManipulation (id, start_matrix_map_.value (id), end_matrix);
    this->InvokeEvent (this->manipulation_complete_event_, manip_event);
    
  }
}

void
pcl::cloud_composer::SelectedTrackballStyleInteractor::Rotate ()
{
  if (this->CurrentRenderer == NULL || this->InteractionProp == NULL)
  {
    return;
  }
  
  vtkRenderWindowInteractor *rwi = this->Interactor;
  vtkCamera *cam = this->CurrentRenderer->GetActiveCamera();
  
  
  // First get the origin of the assembly
  double *obj_center = this->InteractionProp->GetCenter();
  
  // GetLength gets the length of the diagonal of the bounding box
  double boundRadius = this->InteractionProp->GetLength() * 0.5;
  
  // Get the view up and view right vectors
  double view_up[3], view_look[3], view_right[3];
  
  cam->OrthogonalizeViewUp();
  cam->ComputeViewPlaneNormal();
  cam->GetViewUp(view_up);
  vtkMath::Normalize(view_up);
  cam->GetViewPlaneNormal(view_look);
  vtkMath::Cross(view_up, view_look, view_right);
  vtkMath::Normalize(view_right);
  
  // Get the furtherest point from object position+origin
  double outsidept[3];
  
  outsidept[0] = obj_center[0] + view_right[0] * boundRadius;
  outsidept[1] = obj_center[1] + view_right[1] * boundRadius;
  outsidept[2] = obj_center[2] + view_right[2] * boundRadius;
  
  // Convert them to display coord
  double disp_obj_center[3];
  
  this->ComputeWorldToDisplay(obj_center[0], obj_center[1], obj_center[2], 
                              disp_obj_center);
  
  this->ComputeWorldToDisplay(outsidept[0], outsidept[1], outsidept[2], 
                              outsidept);
  
  double radius = sqrt(vtkMath::Distance2BetweenPoints(disp_obj_center,
                                                      outsidept));
  double nxf = (rwi->GetEventPosition()[0] - disp_obj_center[0]) / radius;
  
  double nyf = (rwi->GetEventPosition()[1] - disp_obj_center[1]) / radius;
  
  double oxf = (rwi->GetLastEventPosition()[0] - disp_obj_center[0]) / radius;
  
  double oyf = (rwi->GetLastEventPosition()[1] - disp_obj_center[1]) / radius;
  
  if (((nxf * nxf + nyf * nyf) <= 1.0) &&
    ((oxf * oxf + oyf * oyf) <= 1.0))
  {
    double newXAngle = vtkMath::DegreesFromRadians( asin( nxf ) );
    double newYAngle = vtkMath::DegreesFromRadians( asin( nyf ) );
    double oldXAngle = vtkMath::DegreesFromRadians( asin( oxf ) );
    double oldYAngle = vtkMath::DegreesFromRadians( asin( oyf ) );
    
    double scale[3];
    scale[0] = scale[1] = scale[2] = 1.0;
    
    double **rotate = new double*[2];
    
    rotate[0] = new double[4];
    rotate[1] = new double[4];
    
    rotate[0][0] = newXAngle - oldXAngle;
    rotate[0][1] = view_up[0];
    rotate[0][2] = view_up[1];
    rotate[0][3] = view_up[2];
    
    rotate[1][0] = oldYAngle - newYAngle;
    rotate[1][1] = view_right[0];
    rotate[1][2] = view_right[1];
    rotate[1][3] = view_right[2];
  
    foreach (QString id, selected_actors_map_.keys ())
    {
      vtkLODActor* actor = selected_actors_map_.value (id);
      this->Prop3DTransform(actor,
                          obj_center,
                          2, 
                          rotate, 
                          scale);
    }
    delete [] rotate[0];
    delete [] rotate[1];
    delete [] rotate;
  }  
  
  if (this->AutoAdjustCameraClippingRange)
  {
    this->CurrentRenderer->ResetCameraClippingRange();
  }
    
  rwi->Render();
    
}

void
pcl::cloud_composer::SelectedTrackballStyleInteractor::Spin ()
{
  if ( this->CurrentRenderer == NULL || this->InteractionProp == NULL )
  {
    return;
  }
  
  vtkRenderWindowInteractor *rwi = this->Interactor;
  vtkCamera *cam = this->CurrentRenderer->GetActiveCamera();
  
  // Get the axis to rotate around = vector from eye to origin
  
  double *obj_center = this->InteractionProp->GetCenter();
  
  double motion_vector[3];
  double view_point[3];
  
  if (cam->GetParallelProjection())
  {
    // If parallel projection, want to get the view plane normal...
    cam->ComputeViewPlaneNormal();
    cam->GetViewPlaneNormal( motion_vector );
  }
  else
  {   
    // Perspective projection, get vector from eye to center of actor
    cam->GetPosition( view_point );
    motion_vector[0] = view_point[0] - obj_center[0];
    motion_vector[1] = view_point[1] - obj_center[1];
    motion_vector[2] = view_point[2] - obj_center[2];
    vtkMath::Normalize(motion_vector);
  }
  
  double disp_obj_center[3];
  
  this->ComputeWorldToDisplay(obj_center[0], obj_center[1], obj_center[2], 
                              disp_obj_center);
  
  double newAngle = 
  vtkMath::DegreesFromRadians( atan2( rwi->GetEventPosition()[1] - disp_obj_center[1],
                                      rwi->GetEventPosition()[0] - disp_obj_center[0] ) );
  
  double oldAngle = 
  vtkMath::DegreesFromRadians( atan2( rwi->GetLastEventPosition()[1] - disp_obj_center[1],
                                      rwi->GetLastEventPosition()[0] - disp_obj_center[0] ) );
  
  double scale[3];
  scale[0] = scale[1] = scale[2] = 1.0;
  
  double **rotate = new double*[1];
  rotate[0] = new double[4];
  
  rotate[0][0] = newAngle - oldAngle;
  rotate[0][1] = motion_vector[0];
  rotate[0][2] = motion_vector[1];
  rotate[0][3] = motion_vector[2];
  
  foreach (QString id, selected_actors_map_.keys ())
  {
    vtkLODActor* actor = selected_actors_map_.value (id);
    this->Prop3DTransform(actor,
                          obj_center,
                          1, 
                          rotate, 
                          scale);
  }
  
  delete [] rotate[0];
  delete [] rotate;
  
  if ( this->AutoAdjustCameraClippingRange )
  {
    this->CurrentRenderer->ResetCameraClippingRange();
  }
  
  rwi->Render();
}

void
pcl::cloud_composer::SelectedTrackballStyleInteractor::Pan ()
{
  if (this->CurrentRenderer == NULL || this->InteractionProp == NULL)
  {
    return;
  }
  
  vtkRenderWindowInteractor *rwi = this->Interactor;
  
  // Use initial center as the origin from which to pan
  
  double *obj_center = this->InteractionProp->GetCenter();
  
  double disp_obj_center[3], new_pick_point[4];
  double old_pick_point[4], motion_vector[3];
  
  this->ComputeWorldToDisplay(obj_center[0], obj_center[1], obj_center[2], 
                              disp_obj_center);
  
  this->ComputeDisplayToWorld(rwi->GetEventPosition()[0], 
                              rwi->GetEventPosition()[1], 
                              disp_obj_center[2],
                              new_pick_point);
  
  this->ComputeDisplayToWorld(rwi->GetLastEventPosition()[0], 
                              rwi->GetLastEventPosition()[1], 
                              disp_obj_center[2],
                              old_pick_point);
  
  motion_vector[0] = new_pick_point[0] - old_pick_point[0];
  motion_vector[1] = new_pick_point[1] - old_pick_point[1];
  motion_vector[2] = new_pick_point[2] - old_pick_point[2];
  
  foreach (QString id, selected_actors_map_.keys ())
  {
    vtkLODActor* actor = selected_actors_map_.value (id);
    if (actor->GetUserMatrix() != NULL)
    {
      vtkTransform *t = vtkTransform::New();
      t->PostMultiply();
      t->SetMatrix(actor->GetUserMatrix());
      t->Translate(motion_vector[0], motion_vector[1], motion_vector[2]);
      actor->GetUserMatrix()->DeepCopy(t->GetMatrix());
      t->Delete();
    }
    else
    {
      actor->AddPosition(motion_vector[0],
                         motion_vector[1],
                         motion_vector[2]);
    }
  }
  
  
  if (this->AutoAdjustCameraClippingRange)
  {
    this->CurrentRenderer->ResetCameraClippingRange();
  }
  
  rwi->Render();
}

void
pcl::cloud_composer::SelectedTrackballStyleInteractor::UniformScale ()
{
  if (this->CurrentRenderer == NULL || this->InteractionProp == NULL)
  {
    return;
  }
  
  vtkRenderWindowInteractor *rwi = this->Interactor;
  
  int dy = rwi->GetEventPosition()[1] - rwi->GetLastEventPosition()[1];
  
  double *obj_center = this->InteractionProp->GetCenter();
  double *center = this->CurrentRenderer->GetCenter();
  
  double yf = dy / center[1] * this->MotionFactor;
  double scaleFactor = pow(1.1, yf);
  
  double **rotate = NULL;
  
  double scale[3];
  scale[0] = scale[1] = scale[2] = scaleFactor;
  foreach (QString id, selected_actors_map_.keys ())
  {
    vtkLODActor* actor = selected_actors_map_.value (id);
    this->Prop3DTransform(actor,
                          obj_center,
                          0, 
                          rotate, 
                          scale);
  }
  
  if (this->AutoAdjustCameraClippingRange)
  {
    this->CurrentRenderer->ResetCameraClippingRange();
  }
  
  rwi->Render();

}
