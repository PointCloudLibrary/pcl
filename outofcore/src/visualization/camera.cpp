// C++
#include <iostream>
#include <string>

// PCL - visualziation
#include <pcl/visualization/common/common.h>

// PCL - outofcore
#include <pcl/outofcore/visualization/camera.h>
#include <pcl/outofcore/visualization/object.h>

// VTK
#include <vtkActor.h>
#include <vtkCamera.h>
#include <vtkCameraActor.h>
#include <vtkHull.h>
#include <vtkPlanes.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkSmartPointer.h>

// Operators
// -----------------------------------------------------------------------------
Camera::Camera (std::string name) :
    Object (name), display_ (false)
{
  camera_ = vtkSmartPointer<vtkCamera>::New ();
  camera_->SetClippingRange(0.0001, 100000);

  camera_actor_ = vtkSmartPointer<vtkCameraActor>::New ();
  camera_actor_->SetCamera (camera_);
  camera_actor_->GetProperty ()->SetLighting (0);
  camera_actor_->GetProperty ()->SetLineStipplePattern (1010101010);

  for (int i = 0; i < 24; i++)
    frustum_[i] = 0;

  hull_actor_ = vtkSmartPointer<vtkActor>::New ();
  vtkSmartPointer<vtkPolyDataMapper> hull_mapper = vtkSmartPointer<vtkPolyDataMapper>::New ();

  hull_actor_->SetMapper (hull_mapper);
  hull_actor_->GetProperty ()->SetLighting (0);
  hull_actor_->GetProperty ()->SetColor (1.0, 0.0, 0.0);
  hull_actor_->GetProperty ()->SetOpacity (0.25);
}

Camera::Camera (std::string name, vtkSmartPointer<vtkCamera> camera) :
    Object (name), display_ (false)
{
  camera_ = camera;
  camera_->SetClippingRange(0.0001, 100000);

  camera_actor_ = vtkSmartPointer<vtkCameraActor>::New ();
  camera_actor_->SetCamera (camera_);
  camera_actor_->GetProperty ()->SetLighting (0);

  for (int i = 0; i < 24; i++)
    frustum_[i] = 0;

  hull_actor_ = vtkSmartPointer<vtkActor>::New ();
  vtkSmartPointer<vtkPolyDataMapper> hull_mapper = vtkSmartPointer<vtkPolyDataMapper>::New ();

  hull_actor_->SetMapper (hull_mapper);
  hull_actor_->GetProperty ()->SetLighting (0);
  hull_actor_->GetProperty ()->SetColor (1.0, 0.0, 0.0);
  hull_actor_->GetProperty ()->SetOpacity (0.25);

  prevUp_[0] = prevUp_[1] = prevUp_[2] = 0;
  prevFocal_[0] = prevFocal_[1] = prevFocal_[2] = 0;
  prevPos_[0] = prevPos_[1] = prevPos_[2] = 0;
}

//std::ostream & operator<<(std::ostream &os, const Camera& camera)
//{
//    return os << camera.getName();
//}

// Methods
// -----------------------------------------------------------------------------
void
//Camera::computeFrustum(double aspect)
Camera::computeFrustum ()
{
  // The planes array contains six plane equations of the form (Ax+By+Cz+D=0), the first four values are (A,B,C,D)
  // which repeats for each of the planes. The planes are given in the following order: -x,+x,-y,+y,-z,+z.
  //camera_->GetFrustumPlanes(aspect, frustum_);

  pcl::visualization::getViewFrustum (getViewProjectionMatrix (), frustum_);

//  vtkSmartPointer<vtkHull> hull = vtkSmartPointer<vtkHull>::New ();
//  vtkSmartPointer<vtkPlanes> planes = vtkSmartPointer<vtkPlanes>::New ();
//  vtkSmartPointer<vtkPolyData> hullData = vtkSmartPointer<vtkPolyData>::New ();
//
//  planes->SetFrustumPlanes (frustum_);
//  hull->SetPlanes (planes);
//  hull->GenerateHull (hullData, -200, 200, -200, 200, -200, 200);
//
//  vtkSmartPointer<vtkPolyDataMapper> hull_mapper = static_cast<vtkPolyDataMapper*> (hull_actor_->GetMapper ());
//
//#if VTK_MAJOR_VERSION < 6
//  hull_mapper->SetInput (hullData);
//#else
//  hull_mapper->SetInputData(hullData);
//#endif
//
//  hull_actor_->SetMapper (hull_mapper);
}

void
Camera::printFrustum ()
{
  for (int i = 0; i < 6; i++)
  {
    std::cout << frustum_[(i * 4)] << "x + " << frustum_[(i * 4) + 1] << "y + " << frustum_[(i * 4) + 2] << "z + "
        << frustum_[(i * 4) + 3] << std::endl;
  }
}

void
Camera::render (vtkRenderer* renderer)
{
  vtkSmartPointer<vtkCamera> active_camera = renderer->GetActiveCamera ();

//  if (camera_.GetPointer() != active_camera.GetPointer())
//  {
//    if (display_)
//    {
//      renderer->AddActor (camera_actor_);
//      renderer->AddActor (hull_actor_);
//    }
//    else
//    {
//      renderer->RemoveActor (camera_actor_);
//      renderer->RemoveActor (hull_actor_);
//    }
//    return;
//  }

  // Reset clipping range
  setClippingRange();

  double *up = active_camera->GetViewUp ();
  double *focal = active_camera->GetFocalPoint ();
  double *pos = active_camera->GetPosition ();

  bool viewpointChanged = false;

  // Check up vector
  if (up[0] != prevUp_[0] || up[1] != prevUp_[1] || up[2] != prevUp_[2])
    viewpointChanged = true;

  // Check focal point
  if (focal[0] != prevFocal_[0] || focal[1] != prevFocal_[1] || focal[2] != prevFocal_[2])
    viewpointChanged = true;

  // Check position
  if (pos[0] != prevPos_[0] || pos[1] != prevPos_[1] || pos[2] != prevPos_[2])
    viewpointChanged = true;

  // Break loop if the viewpoint hasn't changed
  if (viewpointChanged)
  {
    prevUp_[0] = up[0];
    prevUp_[1] = up[1];
    prevUp_[2] = up[2];
    prevFocal_[0] = focal[0];
    prevFocal_[1] = focal[1];
    prevFocal_[2] = focal[2];
    prevPos_[0] = pos[0];
    prevPos_[1] = pos[1];
    prevPos_[2] = pos[2];

//        std::cout << "View Changed" << std::endl;
//        std::cout << "Up: <" << up[0] << ", " << up[1] << ", " << up[2] << ">" << std::endl;
//        std::cout << "Focal: <" << focal[0] << ", " << focal[1] << ", " << focal[2] << ">" << std::endl;
//        std::cout << "Pos: <" << pos[0] << ", " << pos[1] << ", " << pos[2] << ">" << std::endl;

    {
      renderer->ComputeAspect ();
      double *aspect = renderer->GetAspect ();

      projection_matrix_ = pcl::visualization::vtkToEigen (active_camera->GetProjectionTransformMatrix (aspect[0] / aspect[1], 0.0, 1.0));
      model_view_matrix_ = pcl::visualization::vtkToEigen (active_camera->GetModelViewTransformMatrix ());

      //computeFrustum (renderer->GetTiledAspectRatio());
      computeFrustum ();
    }
  }

  Object::render(renderer);
}
